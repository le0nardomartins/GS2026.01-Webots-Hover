import numpy as np
import cv2

from config import ROI_TOP, ROI_BOTTOM, MIN_AREA_RATIO, MIN_BOX_DIM, MIN_SOLIDITY, MIN_ASPECT_RATIO
from robot_setup import camera, camera_width, camera_height


def get_camera_frame_bgr():
    image = camera.getImage()
    frame = np.frombuffer(image, np.uint8).reshape((camera_height, camera_width, 4))
    return frame[:, :, :3].copy()


def _obstacle_mask(blurred):
    """
    Detecta regiões escuras vs fundo claro com dois métodos combinados:

    1. Downscale background: downscala 1/8 + blur forte + upscala.
       O kernel efetivo é ~80 px, então pedras GRANDES não dominam
       sua própria média local — bug que existia no método anterior.

    2. Percentil: usa o brilho do 72-percentil como "nível do céu/solo"
       e detecta o que ficou abaixo. Complementa o método 1 quando
       a câmera está quase toda preenchida por rocha.

    Threshold com Otsu: automático, se adapta a qualquer contraste.
    """
    h, w = blurred.shape[:2]

    # Método 1 – background via downscale
    small  = cv2.resize(blurred, (max(8, w // 8), max(6, h // 8)), interpolation=cv2.INTER_AREA)
    bg_sm  = cv2.GaussianBlur(small, (11, 11), 0)
    bg     = cv2.resize(bg_sm, (w, h), interpolation=cv2.INTER_LINEAR)
    diff1  = np.clip(bg.astype(np.int16) - blurred.astype(np.int16), 0, 255).astype(np.uint8)

    # Método 2 – percentil de cena (80º percentil = menos sensível a pedras pequenas)
    p72    = float(np.percentile(blurred, 80))
    diff2  = np.clip(p72 - blurred.astype(np.float32), 0, 255).astype(np.uint8)

    diff   = cv2.max(diff1, diff2)
    vis    = cv2.normalize(diff, None, 0, 255, cv2.NORM_MINMAX)

    if int(diff.max()) > 8:
        _, binary = cv2.threshold(diff, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    else:
        binary = np.zeros_like(diff)

    return vis, binary


def cv_pipeline(frame):
    h, w = frame.shape[:2]
    frame_area = h * w

    roi_y1 = int(h * ROI_TOP)
    roi_y2 = int(h * ROI_BOTTOM)

    gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (7, 7), 0)

    contrast_vis, contrast_bin = _obstacle_mask(blurred)
    edges = cv2.Canny(blurred, 35, 110)

    combined  = cv2.bitwise_or(contrast_bin, edges)
    kernel    = np.ones((5, 5), np.uint8)
    full_mask = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, kernel, iterations=1)

    mask = np.zeros_like(full_mask)
    mask[roi_y1:roi_y2, :] = full_mask[roi_y1:roi_y2, :]

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Base do obstáculo deve estar abaixo de 52% do ROI — filtra paredes/horizonte e pedras distantes
    near_zone_y = roi_y1 + int((roi_y2 - roi_y1) * 0.52)
    min_area    = max(150, int(frame_area * MIN_AREA_RATIO))

    obstacles = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area:
            continue

        x, y, bw, bh = cv2.boundingRect(cnt)
        if bw < MIN_BOX_DIM or bh < MIN_BOX_DIM:
            continue

        if area / max(1, bw * bh) < MIN_SOLIDITY:
            continue

        if bh / max(1, bw) < MIN_ASPECT_RATIO:
            continue

        if y + bh < near_zone_y:
            continue

        obstacles.append({
            "x1": x, "y1": y,
            "x2": x + bw, "y2": y + bh,
            "area": area,
            "ratio": area / frame_area,
            "contour": cnt,
        })

    return obstacles, gray, blurred, contrast_vis, edges, mask
