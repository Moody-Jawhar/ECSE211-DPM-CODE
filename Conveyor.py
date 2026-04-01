from utils.brick import Motor, EV3GyroSensor, EV3ColorSensor, configure_ports, busy_sleep
import math
import threading

# ==========================================================================
# CONVEYOR HELPERS
# ==========================================================================
def pickup_brick():
    CONVEYOR.set_position_relative(220)

def throw_brick():
    CONVEYOR.set_position_relative(-250)
