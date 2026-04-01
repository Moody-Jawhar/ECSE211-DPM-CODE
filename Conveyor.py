from utils.brick import Motor, EV3GyroSensor, EV3ColorSensor, configure_ports, busy_sleep
import math
import threading

COLOR_SENSOR, GYRO, LEFT_MOTOR, RIGHT_MOTOR, CONVEYOR, ROTATING_MOTOR = configure_ports(
    PORT_1=EV3ColorSensor,
    PORT_3=EV3GyroSensor,
    PORT_A=Motor,
    PORT_B=Motor,
    PORT_C=Motor,
    PORT_D=Motor,
)
# ==========================================================================
# CONVEYOR HELPERS
# ==========================================================================
def pickup_brick():
    CONVEYOR.set_position_relative(220)

def throw_brick():
    CONVEYOR.set_position_relative(-250)
