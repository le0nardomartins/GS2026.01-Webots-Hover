from controller import Robot
from config import TIME_STEP

robot = Robot()

motor_names = [
    "motor_front_left",
    "motor_mid_left",
    "motor_back_left",
    "motor_front_right",
    "motor_mid_right",
    "motor_back_right",
]

motors = {}

for _name in motor_names:
    _motor = robot.getDevice(_name)
    _motor.setPosition(float("inf"))
    _motor.setVelocity(0.0)

    try:
        _motor.enableTorqueFeedback(TIME_STEP)
    except Exception:
        pass

    motors[_name] = _motor


camera = robot.getDevice("front_camera")
camera.enable(TIME_STEP)

camera_width = camera.getWidth()
camera_height = camera.getHeight()


display = None
try:
    display = robot.getDevice("hud_display")
except Exception:
    pass


proximity_sensor_names = [
    "prox_front",
    "prox_front_left",
    "prox_front_right",
    "prox_left",
    "prox_right",
]

proximity_sensors = {}

for _name in proximity_sensor_names:
    try:
        _sensor = robot.getDevice(_name)
        _sensor.enable(TIME_STEP)
        proximity_sensors[_name] = _sensor
    except Exception:
        print(f"[AVISO] Sensor não encontrado: {_name}")


print("[Mars_Hover] Controller iniciado — modo autônomo.")
