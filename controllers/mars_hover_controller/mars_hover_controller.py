import robot_setup  # inicializa o robô e todos os dispositivos

from config import TIME_STEP, STATE_PARAR
from robot_setup import robot
from vision import get_camera_frame_bgr, cv_pipeline
from decision import (
    decide_by_vision, read_proximity, proximity_risk,
    fuse_decision, apply_autonomous_action, get_maneuver_phase,
)
from drive import stop_rover
from telemetry import get_current_report
from dashboard import (
    draw_zones, draw_obstacles, draw_direction,
    build_dashboard, send_dashboard_to_display,
    read_wwi_commands, send_wwi_telemetry, is_forced_stop,
)

step_counter = 0
wwi_counter  = 0

while robot.step(TIME_STEP) != -1:
    read_wwi_commands()

    frame = get_camera_frame_bgr()

    obstacles, gray, blurred, contrast_m, edges, mask = cv_pipeline(frame)

    vision_decision  = decide_by_vision(obstacles, frame.shape[1])
    proximity_values = read_proximity()
    prox_decision    = proximity_risk(proximity_values)

    state, action, risk, route = fuse_decision(vision_decision, prox_decision)

    if is_forced_stop():
        stop_rover()
        state  = STATE_PARAR
        action = "PARADA FORCADA"
        risk   = "CRITICO"
        route  = "COMANDO DO DASHBOARD"
    else:
        apply_autonomous_action(state, risk)

    maneuver_phase = get_maneuver_phase()

    img_ann = frame.copy()
    draw_zones(img_ann)
    draw_obstacles(img_ann, obstacles)
    draw_direction(img_ann, state)

    current_lines, total_current = get_current_report()
    currents = {name: curr for name, curr, _ in current_lines}

    # Calcula zonas de visão para o payload WWI
    fw = frame.shape[1]
    zones = {"left": "Livre", "center": "Livre", "right": "Livre"}
    for obs in obstacles:
        cx = (obs["x1"] + obs["x2"]) // 2
        if cx < fw // 3:
            zones["left"] = "Ocupada"
        elif cx < 2 * fw // 3:
            zones["center"] = "Ocupada"
        else:
            zones["right"] = "Ocupada"

    dashboard = build_dashboard(
        img_ann, gray, blurred, contrast_m, edges,
        state, action, risk, route,
        len(obstacles), proximity_values, total_current,
        maneuver_phase,
    )

    send_dashboard_to_display(dashboard)

    wwi_counter += 1
    if wwi_counter >= 5:
        send_wwi_telemetry(
            state=state,
            action=action,
            risk=risk,
            route=route,
            obstacles_count=len(obstacles),
            zones=zones,
            proximity_values=proximity_values,
            currents=currents,
            total_current=total_current,
        )
        wwi_counter = 0

    step_counter += 1
    if step_counter >= 30:
        print(
            f"[HUD] Estado={state} | Manobra={maneuver_phase} | Risco={risk} | "
            f"Obs={len(obstacles)} | Corrente={total_current:.1f} A"
        )
        step_counter = 0
