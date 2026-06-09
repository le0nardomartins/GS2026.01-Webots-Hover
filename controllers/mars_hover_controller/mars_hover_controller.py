import robot_setup  # inicializa o robô e todos os dispositivos

from config import TIME_STEP
from robot_setup import robot
from vision import get_camera_frame_bgr, cv_pipeline
from decision import (
    decide_by_vision, read_proximity, proximity_risk,
    fuse_decision, apply_autonomous_action, get_maneuver_phase,
)
from telemetry import get_current_report
from dashboard import (
    draw_zones, draw_obstacles, draw_direction,
    build_dashboard, send_dashboard_to_display,
)

step_counter = 0

while robot.step(TIME_STEP) != -1:
    frame = get_camera_frame_bgr()

    obstacles, gray, blurred, contrast_m, edges, mask = cv_pipeline(frame)

    vision_decision  = decide_by_vision(obstacles, frame.shape[1])
    proximity_values = read_proximity()
    prox_decision    = proximity_risk(proximity_values)

    state, action, risk, route = fuse_decision(vision_decision, prox_decision)

    apply_autonomous_action(state, risk)
    maneuver_phase = get_maneuver_phase()

    img_ann = frame.copy()
    draw_zones(img_ann)
    draw_obstacles(img_ann, obstacles)
    draw_direction(img_ann, state)

    _, total_current = get_current_report()

    dashboard = build_dashboard(
        img_ann, gray, blurred, contrast_m, edges,
        state, action, risk, route,
        len(obstacles), proximity_values, total_current,
        maneuver_phase,
    )

    send_dashboard_to_display(dashboard)

    step_counter += 1
    if step_counter >= 30:
        print(
            f"[HUD] Estado={state} | Manobra={maneuver_phase} | Risco={risk} | "
            f"Obs={len(obstacles)} | Corrente={total_current:.1f} A"
        )
        step_counter = 0
