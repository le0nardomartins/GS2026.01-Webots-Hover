from config import (
    AUTO_SPEED, TURN_SPEED, REVERSE_SPEED, REVERSE_STEPS, TURN_STEPS,
    LOW_RISK_RATIO, HIGH_RISK_RATIO, CRITICAL_RATIO,
    STATE_LIVRE, STATE_PARAR, STATE_DESVIAR_ESQ, STATE_DESVIAR_DIR, STATE_ALERTA,
)
from robot_setup import proximity_sensors
from drive import move_forward, move_backward, stop_rover, soft_left, soft_right, turn_left, turn_right


# ── Decisão por visão ────────────────────────────────────────────────────────

def _zone(cx, frame_width):
    if cx < frame_width // 3:
        return "ESQ"
    if cx < 2 * frame_width // 3:
        return "CENTRO"
    return "DIR"


def decide_by_vision(obstacles, frame_width):
    if not obstacles:
        return STATE_LIVRE, "SEGUIR EM FRENTE", "BAIXO", "CAMINHO LIVRE"

    max_ratio = max(o["ratio"] for o in obstacles)

    # Obstáculos pequenos: alertar mas não desviar, rover pode passar
    if max_ratio < LOW_RISK_RATIO:
        return STATE_ALERTA, "OBSTACULO PEQUENO - PASSANDO", "BAIXO", "MONITORAR"

    # Descarta obstáculos insignificantes antes da análise de zonas
    significant = [o for o in obstacles if o["ratio"] >= LOW_RISK_RATIO]

    if max_ratio >= CRITICAL_RATIO or len(significant) >= 5:
        risk = "CRITICO"
    elif max_ratio >= HIGH_RISK_RATIO or len(significant) >= 3:
        risk = "ALTO"
    else:
        risk = "MEDIO"

    zones = {"ESQ": 0, "CENTRO": 0, "DIR": 0}
    for obs in significant:
        cx = (obs["x1"] + obs["x2"]) // 2
        zones[_zone(cx, frame_width)] += 1

    has_esq    = zones["ESQ"] > 0
    has_centro = zones["CENTRO"] > 0
    has_dir    = zones["DIR"] > 0

    if has_centro:
        if not has_esq:
            return STATE_DESVIAR_ESQ, "DESVIAR PARA ESQUERDA", risk, "ESQUERDA LIVRE"
        if not has_dir:
            return STATE_DESVIAR_DIR, "DESVIAR PARA DIREITA", risk, "DIREITA LIVRE"
        return STATE_PARAR, "PARAR - SEM ROTA SEGURA", "CRITICO", "BLOQUEADO"

    if has_esq and not has_dir:
        return STATE_DESVIAR_DIR, "DESVIAR PARA DIREITA", risk, "DIREITA LIVRE"
    if has_dir and not has_esq:
        return STATE_DESVIAR_ESQ, "DESVIAR PARA ESQUERDA", risk, "ESQUERDA LIVRE"

    return STATE_ALERTA, "REDUZIR VELOCIDADE", risk, "MONITORAR FLANCOS"


# ── Proximidade ──────────────────────────────────────────────────────────────

_CRITICAL_PROX = 750
_WARNING_PROX  = 450


def read_proximity():
    return {name: sensor.getValue() for name, sensor in proximity_sensors.items()}


def proximity_risk(values):
    front       = values.get("prox_front", 0)
    front_left  = values.get("prox_front_left", 0)
    front_right = values.get("prox_front_right", 0)
    left        = values.get("prox_left", 0)
    right       = values.get("prox_right", 0)

    if front >= _CRITICAL_PROX:
        if front_left < front_right:
            return STATE_DESVIAR_ESQ, "PROX: DESVIAR ESQUERDA", "CRITICO", "FRENTE BLOQUEADA"
        return STATE_DESVIAR_DIR, "PROX: DESVIAR DIREITA", "CRITICO", "FRENTE BLOQUEADA"

    if front_left >= _CRITICAL_PROX:
        return STATE_DESVIAR_DIR, "PROX: OBSTACULO ESQUERDA", "ALTO", "DIREITA LIVRE"
    if front_right >= _CRITICAL_PROX:
        return STATE_DESVIAR_ESQ, "PROX: OBSTACULO DIREITA", "ALTO", "ESQUERDA LIVRE"
    if front >= _WARNING_PROX:
        return STATE_ALERTA, "PROX: REDUZIR VELOCIDADE", "MEDIO", "OBSTACULO PROXIMO"
    if left >= _CRITICAL_PROX and right >= _CRITICAL_PROX:
        return STATE_PARAR, "PROX: PARAR", "CRITICO", "LATERAIS BLOQUEADAS"

    return None


# ── Fusão ────────────────────────────────────────────────────────────────────

def fuse_decision(vision_decision, prox_decision):
    if prox_decision is not None:
        return prox_decision
    return vision_decision


# ── Manobra de contorno (ré + virada) ────────────────────────────────────────
#
# Quando o estado PARAR é detectado, o rover executa:
#   1. REVERSE  – dá ré por REVERSE_STEPS iterações
#   2. TURN     – vira por TURN_STEPS iterações
#   3. Volta ao modo normal de navegação
#
# Durante a manobra a decisão de visão/proximidade é ignorada.

_phase    = "NONE"   # "NONE" | "REVERSE" | "TURN"
_counter  = 0
_turn_dir = "ESQ"    # direção da virada da manobra atual
_last_desviar = None # último estado de desvio registrado


def get_maneuver_phase():
    return _phase


def apply_autonomous_action(state, risk):
    global _phase, _counter, _turn_dir, _last_desviar

    # ── Manobra ativa: ignora estado de visão ────────────────────────────
    if _phase == "REVERSE":
        move_backward(REVERSE_SPEED)
        _counter -= 1
        if _counter <= 0:
            _phase   = "TURN"
            _counter = TURN_STEPS
        return

    if _phase == "TURN":
        if _turn_dir == "ESQ":
            turn_left(TURN_SPEED)
        else:
            turn_right(TURN_SPEED)
        _counter -= 1
        if _counter <= 0:
            _phase = "NONE"
        return

    # ── Navegação normal ─────────────────────────────────────────────────
    if state == STATE_LIVRE:
        move_forward(AUTO_SPEED)

    elif state == STATE_DESVIAR_ESQ:
        _last_desviar = STATE_DESVIAR_ESQ
        soft_left(AUTO_SPEED)

    elif state == STATE_DESVIAR_DIR:
        _last_desviar = STATE_DESVIAR_DIR
        soft_right(AUTO_SPEED)

    elif state == STATE_ALERTA:
        move_forward(AUTO_SPEED * 0.45)

    elif state == STATE_PARAR:
        stop_rover()
        # Decide para que lado virar baseado no último desvio sugerido
        if _last_desviar == STATE_DESVIAR_ESQ:
            _turn_dir = "ESQ"
        elif _last_desviar == STATE_DESVIAR_DIR:
            _turn_dir = "DIR"
        else:
            _turn_dir = "ESQ"
        _phase   = "REVERSE"
        _counter = REVERSE_STEPS
        _last_desviar = None

    else:
        stop_rover()
