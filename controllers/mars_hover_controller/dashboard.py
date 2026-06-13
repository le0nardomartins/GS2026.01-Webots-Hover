import json
import os
import time
import threading
import http.server
import socketserver
import mimetypes
from urllib.parse import urlparse, unquote

import numpy as np
import cv2
from controller import Display

from config import ROI_TOP, ROI_BOTTOM, HIGH_RISK_RATIO, STATE_COLOR, RISK_COLOR
from config import STATE_LIVRE, STATE_DESVIAR_ESQ, STATE_DESVIAR_DIR, STATE_PARAR, STATE_ALERTA
from robot_setup import robot, display, proximity_sensor_names, proximity_sensors

# ── Servidor HTTP local para o dashboard JS ──────────────────────────────────
# O Webots abre o robot window num browser externo onde o objeto `webots` não
# existe. O JS faz polling em http://localhost:8765/ a cada 200 ms.

_HTTP_PORT   = 8765
_latest_json = json.dumps({"type": "telemetry"})

# Caminho do telemetry.json na pasta do plugin (mesmo diretório do HTML)
_PLUGIN_JSON = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "..", "plugins", "robot_windows", "mars_hover_dashboard", "telemetry.json"
))
_PLUGIN_DIR = os.path.dirname(_PLUGIN_JSON)


class _TelemetryHandler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        parsed_path = urlparse(self.path).path
        if parsed_path in ("/telemetry", "/telemetry.json"):
            self._send_json()
            return

        if parsed_path == "/":
            self._send_file(os.path.join(_PLUGIN_DIR, "mars_hover_dashboard.html"))
            return

        requested = os.path.normpath(os.path.join(_PLUGIN_DIR, unquote(parsed_path.lstrip("/"))))
        if os.path.commonpath([_PLUGIN_DIR, requested]) != _PLUGIN_DIR or not os.path.isfile(requested):
            self.send_error(404)
            return

        self._send_file(requested)

    def _send_json(self):
        data = _latest_json.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _send_file(self, path):
        try:
            with open(path, "rb") as f:
                data = f.read()
        except OSError:
            self.send_error(404)
            return

        content_type = mimetypes.guess_type(path)[0] or "application/octet-stream"
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, *args):
        pass   # silencia o log do servidor


class _ReuseServer(socketserver.TCPServer):
    allow_reuse_address = True


def _run_server():
    try:
        with _ReuseServer(("", _HTTP_PORT), _TelemetryHandler) as httpd:
            httpd.serve_forever()
    except Exception as e:
        print(f"[HTTP] Servidor falhou na porta {_HTTP_PORT}: {e}")


_server_thread = threading.Thread(target=_run_server, daemon=True)
_server_thread.start()
print(f"[HTTP] Dashboard disponível em http://localhost:{_HTTP_PORT}/")


# Layout 1024 × 900
# ┌──────────────────────┬───────────────────────┐ 520 px
# │  Câmera  (680×520)   │  Direção (344×260)    │
# │                      │  Proximidade (344×260) │
# ├──────────────────────┴───────────────────────┤ 185 px
# │  Pipeline: 4 × 256 px = 1024 px              │
# ├───────────────────────────────────────────────┤ 195 px
# │  Status                                       │
# └───────────────────────────────────────────────┘
_W, _H         = 1024, 900
_CAM_W, _CAM_H = 680, 520
_PNL_W, _PNL_H = 344, 520
_PIPE_W        = 256          # 4 × 256 = 1024
_PIPE_H        = 185
_STAT_H        = 195          # 520 + 185 + 195 = 900

AA = cv2.LINE_AA


def _txt(img, text, pos, scale, color, thick=2):
    """Texto com sombra preta para legibilidade em qualquer fundo."""
    cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thick + 3, AA)
    cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, AA)


# ── Anotações na câmera ──────────────────────────────────────────────────────

def draw_zones(img):
    h, w = img.shape[:2]
    y1, y2 = int(h * ROI_TOP), int(h * ROI_BOTTOM)
    z1, z2 = w // 3, 2 * w // 3
    for x in (z1, z2):
        cv2.line(img, (x, y1), (x, y2), (80, 80, 180), 1, AA)
    cv2.line(img, (0, y2), (w, y2), (60, 60, 150), 1, AA)
    for label, cx in [("ESQ", z1 // 2), ("CENTRO", (z1 + z2) // 2), ("DIR", (z2 + w) // 2)]:
        _txt(img, label, (cx - 30, y1 + 22), 0.55, (100, 100, 220), 1)


def draw_obstacles(img, obstacles):
    for i, obs in enumerate(obstacles, 1):
        color = (0, 0, 220) if obs["ratio"] >= HIGH_RISK_RATIO else (0, 200, 255)
        cv2.rectangle(img, (obs["x1"], obs["y1"]), (obs["x2"], obs["y2"]), color, 2)
        cv2.drawContours(img, [obs["contour"]], -1, color, 1)
        _txt(img, f"OBS{i} {obs['ratio']:.0%}",
             (obs["x1"], max(obs["y1"] - 8, 16)), 0.50, color, 1)


def draw_direction(img, state):
    h, w = img.shape[:2]
    cx, cy = w // 2, h // 2
    color = STATE_COLOR.get(state, (200, 200, 200))

    def arrow(p1, p2):
        cv2.arrowedLine(img, p1, p2, (0, 0, 0), 10, AA, tipLength=0.35)
        cv2.arrowedLine(img, p1, p2, color, 5, AA, tipLength=0.35)

    if state == STATE_LIVRE:
        arrow((cx, cy + 80), (cx, cy - 80))
    elif state == STATE_DESVIAR_ESQ:
        arrow((cx + 100, cy), (cx - 100, cy))
    elif state == STATE_DESVIAR_DIR:
        arrow((cx - 100, cy), (cx + 100, cy))
    elif state == STATE_PARAR:
        for p1, p2 in [((cx-55, cy-55), (cx+55, cy+55)), ((cx+55, cy-55), (cx-55, cy+55))]:
            cv2.line(img, p1, p2, (0, 0, 0), 10, AA)
            cv2.line(img, p1, p2, color, 5, AA)
    elif state == STATE_ALERTA:
        _txt(img, "!", (cx - 16, cy + 38), 3.0, color, 6)


# ── Painel direito: direção + manobra ────────────────────────────────────────

def _build_direction_half(state, maneuver_phase):
    h, w = _PNL_H // 2, _PNL_W
    panel = np.zeros((h, w, 3), dtype=np.uint8)
    cx, cy = w // 2, h // 2

    cv2.rectangle(panel, (3, 3), (w - 4, h - 4), (28, 28, 28), -1)
    cv2.rectangle(panel, (3, 3), (w - 4, h - 4), (60, 60, 60), 1)

    # Manobra ativa substitui o estado normal
    if maneuver_phase == "REVERSE":
        color = (0, 200, 255)
        label = "RECUANDO"
        _txt(panel, label, ((w - len(label) * 13) // 2, 34), 0.72, color, 2)
        cv2.line(panel, (20, 46), (w - 20, 46), (55, 55, 55), 1, AA)
        # Seta para baixo
        cv2.arrowedLine(panel, (cx, cy - 70), (cx, cy + 70), (0, 0, 0), 14, AA, tipLength=0.30)
        cv2.arrowedLine(panel, (cx, cy - 70), (cx, cy + 70), color, 7, AA, tipLength=0.30)
        return panel

    if maneuver_phase == "TURN":
        color = (0, 200, 255)
        label = "VIRANDO"
        _txt(panel, label, ((w - len(label) * 13) // 2, 34), 0.72, color, 2)
        cv2.line(panel, (20, 46), (w - 20, 46), (55, 55, 55), 1, AA)
        # Seta circular simulada com seta horizontal
        cv2.arrowedLine(panel, (cx + 80, cy), (cx - 80, cy), (0, 0, 0), 14, AA, tipLength=0.35)
        cv2.arrowedLine(panel, (cx + 80, cy), (cx - 80, cy), color, 7, AA, tipLength=0.35)
        return panel

    color = STATE_COLOR.get(state, (200, 200, 200))
    label = state.replace("_", " ")
    (tw, _), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.72, 2)
    _txt(panel, label, ((w - tw) // 2, 34), 0.72, color, 2)
    cv2.line(panel, (20, 46), (w - 20, 46), (55, 55, 55), 1, AA)

    def arrow(p1, p2):
        cv2.arrowedLine(panel, p1, p2, (0, 0, 0), 14, AA, tipLength=0.30)
        cv2.arrowedLine(panel, p1, p2, color, 7, AA, tipLength=0.30)

    if state == STATE_LIVRE:
        arrow((cx, cy + 70), (cx, cy - 70))
    elif state == STATE_DESVIAR_ESQ:
        arrow((cx + 90, cy), (cx - 90, cy))
    elif state == STATE_DESVIAR_DIR:
        arrow((cx - 90, cy), (cx + 90, cy))
    elif state == STATE_PARAR:
        for p1, p2 in [((cx-50, cy-50), (cx+50, cy+50)), ((cx+50, cy-50), (cx-50, cy+50))]:
            cv2.line(panel, p1, p2, (0, 0, 0), 14, AA)
            cv2.line(panel, p1, p2, color, 7, AA)
    elif state == STATE_ALERTA:
        _txt(panel, "!", (cx - 18, cy + 46), 3.8, color, 8)

    return panel


# ── Painel direito: sensores de proximidade ──────────────────────────────────

def _build_proximity_half(proximity_values):
    h, w = _PNL_H // 2, _PNL_W
    panel = np.zeros((h, w, 3), dtype=np.uint8)

    cv2.rectangle(panel, (3, 3), (w - 4, h - 4), (22, 22, 22), -1)
    cv2.rectangle(panel, (3, 3), (w - 4, h - 4), (60, 60, 60), 1)

    _txt(panel, "PROXIMIDADE", (14, 32), 0.65, (210, 210, 210), 1)
    cv2.line(panel, (20, 44), (w - 20, 44), (55, 55, 55), 1, AA)

    # Mapeamento: nome interno → label curta
    sensor_defs = [
        ("prox_front",       "FRENTE"),
        ("prox_front_left",  "FR-ESQ"),
        ("prox_front_right", "FR-DIR"),
        ("prox_left",        "ESQUER"),
        ("prox_right",       "DIREIT"),
    ]

    bar_x0  = 110
    bar_x1  = w - 12
    bar_len = bar_x1 - bar_x0

    for i, (name, label) in enumerate(sensor_defs):
        online = name in proximity_sensors   # sensor foi inicializado?
        value  = proximity_values.get(name, 0)
        y      = 62 + i * 37

        # Status online/offline
        status_color = (0, 180, 0) if online else (80, 80, 80)
        cv2.circle(panel, (w - 18, y - 6), 5, status_color, -1, AA)

        # Label + valor
        label_color = (160, 160, 160) if online else (70, 70, 70)
        _txt(panel, f"{label}", (12, y), 0.46, label_color, 1)

        if online:
            _txt(panel, f"{value:.0f}", (bar_x0 - 48, y), 0.42, (120, 120, 120), 1)
            # Barra de fundo
            cv2.rectangle(panel, (bar_x0, y - 14), (bar_x1 - 20, y - 3), (38, 38, 38), -1)
            cv2.rectangle(panel, (bar_x0, y - 14), (bar_x1 - 20, y - 3), (65, 65, 65), 1)
            # Barra de valor
            fill = int(min(bar_len - 20, value / 1000.0 * (bar_len - 20)))
            if fill > 0:
                bc = (0, 200, 0) if value <= 450 else (0, 165, 255) if value <= 750 else (0, 0, 220)
                cv2.rectangle(panel, (bar_x0, y - 14), (bar_x0 + fill, y - 3), bc, -1)
        else:
            _txt(panel, "OFFLINE", (bar_x0 - 48, y), 0.40, (60, 60, 60), 1)

    return panel


# ── Pipeline row ─────────────────────────────────────────────────────────────

def _build_pipeline_row(gray, blurred, contrast_vis, edges):
    steps = [
        (gray,         "1. CINZA"),
        (blurred,      "2. BLUR"),
        (contrast_vis, "3. CONTRASTE"),
        (edges,        "4. CANNY"),
    ]
    minis = []
    for src, label in steps:
        m = cv2.resize(src, (_PIPE_W, _PIPE_H))
        if m.ndim == 2:
            m = cv2.cvtColor(m, cv2.COLOR_GRAY2BGR)
        cv2.rectangle(m, (0, 0), (_PIPE_W - 1, _PIPE_H - 1), (55, 55, 55), 1)
        _txt(m, label, (8, _PIPE_H - 10), 0.50, (0, 230, 230), 1)
        minis.append(m)
    return np.hstack(minis)


# ── Status bar ───────────────────────────────────────────────────────────────

def _build_status_bar(state, action, risk, route, num_obs, total_current, maneuver_phase):
    panel = np.zeros((_STAT_H, _W, 3), dtype=np.uint8)
    cv2.rectangle(panel, (0, 0), (_W - 1, _STAT_H - 1), (18, 18, 18), -1)
    cv2.line(panel, (0, 0), (_W, 0), (70, 70, 70), 2)

    sc = STATE_COLOR.get(state, (200, 200, 200))
    rc = RISK_COLOR.get(risk, (200, 200, 200))

    # Durante manobra, indica visualmente
    if maneuver_phase != "NONE":
        sc = (0, 200, 255)

    status_txt = "OBSTACULO DETECTADO" if num_obs > 0 else "LIVRE"

    _txt(panel, f"STATUS: {status_txt}", (20,  42), 0.85, sc, 2)
    _txt(panel, f"RISCO:  {risk}",        (20,  84), 0.82, rc, 2)
    _txt(panel, f"ACAO:   {action}",      (20, 124), 0.70, sc, 2)
    _txt(panel, f"ROTA:   {route}",       (20, 162), 0.58, (160, 160, 160), 1)

    cv2.line(panel, (_W // 2, 14), (_W // 2, _STAT_H - 14), (55, 55, 55), 1, AA)

    _txt(panel, f"OBSTACULOS: {num_obs}",           (_W // 2 + 20,  42), 0.82, (180, 180, 180), 2)
    _txt(panel, f"CORRENTE:   {total_current:.1f} A", (_W // 2 + 20,  84), 0.82, (180, 180, 180), 2)

    if maneuver_phase != "NONE":
        label = "RECUANDO" if maneuver_phase == "REVERSE" else "VIRANDO"
        _txt(panel, f"MANOBRA: {label}", (_W // 2 + 20, 124), 0.70, (0, 200, 255), 2)

    return panel


# ── Dashboard completo ───────────────────────────────────────────────────────

def build_dashboard(img_ann, gray, blurred, contrast_vis, edges,
                    state, action, risk, route,
                    num_obs, proximity_values, total_current,
                    maneuver_phase="NONE"):

    main  = cv2.resize(img_ann, (_CAM_W, _CAM_H))
    right = np.vstack([
        _build_direction_half(state, maneuver_phase),
        _build_proximity_half(proximity_values),
    ])

    top_row  = np.hstack([main, right])
    pipe_row = _build_pipeline_row(gray, blurred, contrast_vis, edges)
    stat_bar = _build_status_bar(state, action, risk, route, num_obs, total_current, maneuver_phase)

    return np.vstack([top_row, pipe_row, stat_bar])


# ── Comunicação WWI com o plugin JS ─────────────────────────────────────────

_autonomous_mode = True
_forced_stop = False


def read_wwi_commands():
    """Drena a fila de mensagens do plugin JS e atualiza o modo de operação."""
    global _autonomous_mode, _forced_stop

    while True:
        message = robot.wwiReceiveText()
        if message is None:
            break

        try:
            data = json.loads(message)
        except Exception:
            continue

        if data.get("type") != "command":
            continue

        command = data.get("command")
        if command == "AUTO":
            _autonomous_mode = True
            _forced_stop = False
            print("[WWI] Modo autônomo ativado")
        elif command == "MANUAL":
            _autonomous_mode = False
            _forced_stop = False
            print("[WWI] Modo manual ativado")
        elif command == "STOP":
            _forced_stop = True
            print("[WWI] Parada forçada")


def send_wwi_telemetry(state, action, risk, route, obstacles_count, zones,
                       proximity_values, currents, total_current):
    """Atualiza o cache HTTP e envia via WWI (se disponível)."""
    global _latest_json

    payload = {
        "type": "telemetry",
        "state": state,
        "action": action,
        "risk": risk,
        "route": route,
        "obstacles": obstacles_count,
        "zones": zones,
        "proximity": proximity_values,
        "currents": currents,
        "totalCurrent": total_current,
        "mode": "STOP" if _forced_stop else ("AUTO" if _autonomous_mode else "MANUAL"),
        "timestamp": time.time(),
    }

    json_str     = json.dumps(payload)
    _latest_json = json_str   # lido pelo servidor HTTP (thread-safe via GIL)

    # Escreve telemetry.json na pasta do plugin — o JS faz fetch direto do arquivo
    try:
        with open(_PLUGIN_JSON, "w", encoding="utf-8") as f:
            f.write(json_str)
    except Exception as e:
        print(f"[JSON] Erro ao escrever telemetry.json: {e}")

    try:
        robot.wwiSendText(json_str)
    except Exception:
        pass


def is_autonomous_mode():
    return _autonomous_mode


def is_forced_stop():
    return _forced_stop


def send_dashboard_to_display(dashboard):
    if display is None:
        return

    target = cv2.resize(dashboard, (_W, _H))
    bgra   = cv2.cvtColor(target, cv2.COLOR_BGR2BGRA)

    try:
        image_ref = display.imageNew(bgra.tobytes(), Display.BGRA, _W, _H)
    except Exception:
        rgba      = cv2.cvtColor(target, cv2.COLOR_BGR2RGBA)
        image_ref = display.imageNew(rgba.tobytes(), Display.RGBA, _W, _H)

    display.imagePaste(image_ref, 0, 0, False)
    display.imageDelete(image_ref)
