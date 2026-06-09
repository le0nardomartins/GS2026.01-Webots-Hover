from config import MOTOR_KT
from robot_setup import motors, motor_names


def get_motor_current(motor):
    try:
        torque = motor.getTorqueFeedback()
    except Exception:
        torque = abs(motor.getVelocity()) * 0.5

    current = abs(torque) / MOTOR_KT
    return current, torque


def get_current_report():
    total_current = 0.0
    lines = []

    for name in motor_names:
        current, torque = get_motor_current(motors[name])
        total_current += current
        lines.append((name, current, torque))

    return lines, total_current
