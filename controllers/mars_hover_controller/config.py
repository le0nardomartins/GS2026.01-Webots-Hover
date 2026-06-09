TIME_STEP = 32

MAX_SPEED = 6.0
TURN_SPEED = 3.0
AUTO_SPEED = 3.5

MOTOR_KT = 0.8


# ── Visão computacional ──────────────────────────────────────────────────────

# Área mínima como fração do frame (se adapta a qualquer resolução de câmera)
MIN_AREA_RATIO = 0.008     # 0.8 % do frame
MIN_BOX_DIM    = 18        # px — bounding box mínimo
MIN_SOLIDITY   = 0.28      # compacidade (0-1)
MIN_ASPECT_RATIO = 0.10    # altura/largura mínima

LOW_RISK_RATIO  = 0.020   # abaixo disso: só alerta, rover pode passar
HIGH_RISK_RATIO = 0.055
CRITICAL_RATIO  = 0.13

ROI_TOP    = 0.08
ROI_BOTTOM = 0.92


# ── Estados ──────────────────────────────────────────────────────────────────

STATE_LIVRE        = "LIVRE"
STATE_PARAR        = "PARAR"
STATE_DESVIAR_ESQ  = "DESVIAR_ESQ"
STATE_DESVIAR_DIR  = "DESVIAR_DIR"
STATE_ALERTA       = "ALERTA"

STATE_COLOR = {
    STATE_LIVRE:       (0, 220, 0),
    STATE_PARAR:       (0, 0, 220),
    STATE_DESVIAR_ESQ: (0, 140, 255),
    STATE_DESVIAR_DIR: (0, 140, 255),
    STATE_ALERTA:      (0, 200, 200),
}

RISK_COLOR = {
    "BAIXO":  (0, 220, 0),
    "MEDIO":  (0, 165, 255),
    "ALTO":   (0, 0, 220),
    "CRITICO":(0, 0, 160),
}


# ── Manobra de contorno (ré + virada) ────────────────────────────────────────

REVERSE_STEPS = 30    # iterações dando ré  (~0.96 s com TIME_STEP=32)
TURN_STEPS    = 55    # iterações virando   (~1.76 s)
REVERSE_SPEED = 2.0
