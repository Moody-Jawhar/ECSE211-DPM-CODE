from utils.brick import Motor, EV3GyroSensor, EV3ColorSensor, TouchSensor, configure_ports, busy_sleep
import math
import threading
from config_ports import COLOR_SENSOR, TOUCH_SENSOR, GYRO, LEFT_MOTOR, RIGHT_MOTOR, CONVEYOR, ROTATING_MOTOR as COLOR_SENSOR, TOUCH_SENSOR, GYRO, LEFT_MOTOR, RIGHT_MOTOR, CONVEYOR, ROTATING_MOTOR



# --------------------------------------------------------------------------
# COLOR SENSOR CALIBRATION
# --------------------------------------------------------------------------
# ---- RED ---------------------------------------------------------------
GREEN_R_MEAN   = 0.433900
GREEN_G_MEAN   = 0.520179
GREEN_B_MEAN   = 0.045921
# ---- YELLOW ------------------------------------------------------------
YELLOW_R_MEAN = 0.570138
YELLOW_G_MEAN = 0.368785
YELLOW_B_MEAN = 0.061077
# ---- GREEN -------------------------------------------------------------
RED_R_MEAN  = 0.896804
RED_G_MEAN  = 0.070061
RED_B_MEAN  = 0.033135

# Number of samples of the color sensor
COLOR_SAMPLES = 20


# Color centroids as (R, G, B) tuples — used for Euclidean distance
COLOR_CENTROIDS = {
    "red":    (RED_R_MEAN,    RED_G_MEAN,    RED_B_MEAN),
    "yellow": (YELLOW_R_MEAN, YELLOW_G_MEAN, YELLOW_B_MEAN),
    "green":  (GREEN_R_MEAN,  GREEN_G_MEAN,  GREEN_B_MEAN),
}


def sample_color():
    """
    Take COLOR_SAMPLES readings, average them, then apply chromatic
    normalization: norm_R = R / (R+G+B), same for G and B.
    Returns [norm_R, norm_G, norm_B] or None if sensor not ready.
    Retries up to 3 times if all samples fail.
    """
    for _attempt in range(3):
        r_sum = 0.0
        g_sum = 0.0
        b_sum = 0.0
        valid = 0
        for _ in range(COLOR_SAMPLES):
            rgb = COLOR_SENSOR.get_rgb()
            if rgb is None or None in rgb:
                continue
            r_sum += rgb[0]
            g_sum += rgb[1]
            b_sum += rgb[2]
            valid += 1
        if valid == 0:
            busy_sleep(0.05)
            continue
        r_avg = r_sum / valid
        g_avg = g_sum / valid
        b_avg = b_sum / valid
        total = r_avg + g_avg + b_avg
        if total == 0.0:
            busy_sleep(0.05)
            continue
        return [r_avg / total, g_avg / total, b_avg / total]
    return None

def euclidean_distance(r, g, b, centroid):
    """
    Compute the 3D Euclidean distance between a normalized reading (r, g, b)
    and a color centroid (cr, cg, cb).
    """
    cr, cg, cb = centroid
    return math.sqrt((r - cr) ** 2 + (g - cg) ** 2 + (b - cb) ** 2)

def identify_color():
    """
    Sample the sensor and return the closest color by Euclidean distance.
    Always returns one of: "red", "yellow", "green".
    Defaults to "yellow" if the sensor is completely unresponsive.
    """
    rgb = sample_color()
    if rgb is None:
        print("  [color] sensor unresponsive, defaulting to yellow")
        return "yellow"
    r, g, b = rgb
    best_color    = None
    best_distance = float("inf")
    for color_name, centroid in COLOR_CENTROIDS.items():
        dist = euclidean_distance(r, g, b, centroid)
        print(f"  [color] dist to {color_name:6s}: {dist:.4f}")
        if dist < best_distance:
            best_distance = dist
            best_color    = color_name
    print(f"  [color] identified as: {best_color}  (dist={best_distance:.4f})")
    return best_color


