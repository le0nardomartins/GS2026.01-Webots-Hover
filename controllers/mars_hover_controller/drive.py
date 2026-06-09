from config import MAX_SPEED, TURN_SPEED, AUTO_SPEED
from robot_setup import motors


def set_left_speed(speed):
    motors["motor_front_left"].setVelocity(-speed)
    motors["motor_mid_left"].setVelocity(-speed)
    motors["motor_back_left"].setVelocity(-speed)


def set_right_speed(speed):
    motors["motor_front_right"].setVelocity(-speed)
    motors["motor_mid_right"].setVelocity(-speed)
    motors["motor_back_right"].setVelocity(-speed)


def stop_rover():
    set_left_speed(0.0)
    set_right_speed(0.0)


def move_forward(speed=MAX_SPEED):
    set_left_speed(speed)
    set_right_speed(speed)


def move_backward(speed=MAX_SPEED):
    set_left_speed(-speed)
    set_right_speed(-speed)


def turn_left(speed=TURN_SPEED):
    set_left_speed(speed)
    set_right_speed(-speed)


def turn_right(speed=TURN_SPEED):
    set_left_speed(-speed)
    set_right_speed(speed)


def soft_left(speed=AUTO_SPEED):
    set_left_speed(speed * 0.25)
    set_right_speed(speed)


def soft_right(speed=AUTO_SPEED):
    set_left_speed(speed)
    set_right_speed(speed * 0.25)
