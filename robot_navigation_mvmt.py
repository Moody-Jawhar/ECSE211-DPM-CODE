"""
Drive forward/backward a set distance, and rotate 90 or 180 degrees.
Motors on port A (left wheel), port B (right wheel), port C (conveyor).
Uses multithreading so motors run concurrently and independently.
Gyro sensor on port 3 provides heading correction during drive and rotate.
Color sensor on port 1 identifies colors using Euclidean distance.
"""
# --------------------------------------------------------------------------
# IMPORTS
# --------------------------------------------------------------------------
from utils.brick import Motor, EV3GyroSensor, EV3ColorSensor, configure_ports, busy_sleep
import math
import threading


# CONSTANTS 

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


# Color centroids as (R, G, B) tuples — used for Euclidean distance
COLOR_CENTROIDS = {
    "red":    (RED_R_MEAN,    RED_G_MEAN,    RED_B_MEAN),
    "yellow": (YELLOW_R_MEAN, YELLOW_G_MEAN, YELLOW_B_MEAN),
    "green":  (GREEN_R_MEAN,  GREEN_G_MEAN,  GREEN_B_MEAN),
}


green_counter = 0


# --------------------------------------------------------------------------
# ROBOT CONFIGURATION
# --------------------------------------------------------------------------
WHEEL_DIAMETER_CM   = 4
AXLE_WIDTH_CM       = 13.175
DRIVE_SPEED         = 300
TURN_SPEED          = 300
COLOR_SAMPLES       = 10
RIGHT_MOTOR_FLIPPED = False
DRIVE_KP = 8.0
DRIVE_KI = 0.1
DRIVE_KD = 2.0


# Average multiple gyro reads per PID cycle to reduce noise on short drives.
GYRO_SAMPLES_PER_CYCLE = 5
TURN_KP = 6.0
TURN_KI = 0.05
TURN_KD = 1.5
TURN_TOLERANCE_DEG  = 1.0
DRIVE_CORRECTION_HZ = 50




WHEEL_CIRCUMFERENCE_CM = math.pi * WHEEL_DIAMETER_CM
MEASUREMENT_NB       = 8
SWEEP_ANGLE          = -30
ROTATING_MOTOR_SPEED = 150
# The sensor angle (cumulative degrees) that is directly in front of the
# robot — green found here means drive forward the most before rotating.
SWEEP_CENTRE_ANGLE = -60.0
# Maximum forward distance driven when green is at SWEEP_CENTRE_ANGLE.
# Tapers linearly to 0 cm as the angle moves away from centre.
SWEEP_CREEP_MAX_CM = 5.0
# --------------------------------------------------------------------------
# HARDWARE SETUP
# --------------------------------------------------------------------------
COLOR_SENSOR, GYRO, LEFT_MOTOR, RIGHT_MOTOR, CONVEYOR, ROTATING_MOTOR = configure_ports(
    PORT_1=EV3ColorSensor,
    PORT_3=EV3GyroSensor,
    PORT_A=Motor,
    PORT_B=Motor,
    PORT_C=Motor,
    PORT_D=Motor,
)
busy_sleep(3.0)
assert isinstance(COLOR_SENSOR, EV3ColorSensor), (
    "COLOR_SENSOR is not an EV3ColorSensor -- check your port wiring. "
    "Color sensor must be on PORT_1."
)
assert isinstance(GYRO, EV3GyroSensor), (
    "GYRO is not an EV3GyroSensor -- check your port wiring. "
    "Gyro must be on PORT_3."
)
assert isinstance(LEFT_MOTOR,  Motor), "LEFT_MOTOR (PORT_A) is not a Motor."
assert isinstance(RIGHT_MOTOR, Motor), "RIGHT_MOTOR (PORT_B) is not a Motor."
assert isinstance(CONVEYOR,    Motor), "CONVEYOR (PORT_C) is not a Motor."
# ==========================================================================
# GLOBAL HEADING STATE
# ==========================================================================
GLOBAL_ZERO: float = 0.0       # raw gyro reading captured once at startup
CURRENT_HEADING: float = 0.0   # cumulative heading offset from GLOBAL_ZERO

def init_gyro_zero():
    """
    Call ONCE at startup.
    Resets the gyro hardware, waits for a stable reading, then records
    that raw value as GLOBAL_ZERO.  All subsequent heading comparisons
    are made against this single reference — the gyro is never reset again.
    """
    global GLOBAL_ZERO, CURRENT_HEADING
    reset_gyro()
    GLOBAL_ZERO = gyro_angle_avg()
    CURRENT_HEADING = 0.0
    print(f"[gyro] Global zero set at raw={GLOBAL_ZERO:.2f}°")

def absolute_heading() -> float:
    """Return the robot's current heading in degrees relative to GLOBAL_ZERO."""
    return gyro_angle_avg() - GLOBAL_ZERO

# ==========================================================================
# GYRO HELPERS
# ==========================================================================
def gyro_angle() -> float:
    """
    Read the gyro, retrying up to 10 times on None.
    Returns a single scalar heading in degrees.
    """
    for _ in range(10):
        reading = GYRO.get_value()
        if reading is not None:
            return reading[0]
        busy_sleep(0.05)
    raise RuntimeError(
        "Gyro returned None 10 times in a row. "
        "Check that the sensor is plugged into PORT_3 and powered on."
    )

def gyro_angle_avg(n: int = GYRO_SAMPLES_PER_CYCLE) -> float:
    """
    Reads the gyro n times in quick succession and returns their mean.
    Suppresses momentary sensor noise that would otherwise cause the PID
    to over-correct and introduce drift on short drives.
    """
    total = 0.0
    for _ in range(n):
        total += gyro_angle()
    return total / n

def reset_gyro():
    GYRO.set_mode(EV3GyroSensor.Mode.BOTH)
    busy_sleep(2.0)
    for _ in range(100):
        if GYRO.get_value() is not None:
            return
        busy_sleep(0.05)
    GYRO.set_mode(EV3GyroSensor.Mode.DPS)
    busy_sleep(1.0)
    GYRO.set_mode(EV3GyroSensor.Mode.BOTH)
    busy_sleep(2.0)
    for _ in range(100):
        if GYRO.get_value() is not None:
            return
        busy_sleep(0.05)
    raise RuntimeError(
        "Gyro did not initialize after reset. "
        "Check the sensor connection on PORT_3."
    )

# ==========================================================================
# UNIT CONVERSION
# ==========================================================================
def cm_to_degrees(cm: float) -> float:
    return (cm / WHEEL_CIRCUMFERENCE_CM) * 360

# ==========================================================================
# LOW-LEVEL MOTOR HELPERS
# ==========================================================================
def _run_motor(motor: Motor, degrees: float, speed: int):
    motor.reset_encoder()
    motor.set_limits(dps=speed)
    motor.set_position(degrees)
    busy_sleep(0.1)
    motor.wait_is_moving()
    motor.wait_is_stopped()

def _run_motors_parallel(left_deg: float, right_deg: float, speed: int):
    t_left  = threading.Thread(target=_run_motor, args=(LEFT_MOTOR,  left_deg,  speed))
    t_right = threading.Thread(target=_run_motor, args=(RIGHT_MOTOR, right_deg, speed))
    t_left.start()
    t_right.start()
    t_left.join()
    t_right.join()

# ==========================================================================
# ROTATING MOTOR HELPERS
# ==========================================================================
def rotate_sensor_to(absolute_position: int):
    """
    Move the rotating sensor motor to an absolute position.
    Speed-capped and fully blocking — caller is guaranteed the motor
    has reached its target before this function returns.
    """
    ROTATING_MOTOR.set_limits(dps=ROTATING_MOTOR_SPEED)
    ROTATING_MOTOR.set_position(absolute_position)
    busy_sleep(0.3)
    ROTATING_MOTOR.wait_is_stopped()

def rotate_sensor_relative(delta_degrees: int):
    """
    Move the rotating sensor motor by a relative amount.
    Same reliability guarantees as rotate_sensor_to().
    """
    ROTATING_MOTOR.set_limits(dps=ROTATING_MOTOR_SPEED)
    ROTATING_MOTOR.set_position_relative(delta_degrees)
    busy_sleep(0.3)
    ROTATING_MOTOR.wait_is_stopped()

# ==========================================================================
# GYRO-CORRECTED FORWARD DRIVE  (uses global zero — no gyro reset)
# ==========================================================================
def drive_forward(distance_cm: float):
    target_deg = cm_to_degrees(distance_cm)
    right_sign = -1 if RIGHT_MOTOR_FLIPPED else 1

    LEFT_MOTOR.reset_encoder()
    RIGHT_MOTOR.reset_encoder()

    done    = threading.Event()
    dt      = 1.0 / DRIVE_CORRECTION_HZ
    barrier = threading.Barrier(2)

    def distance_watcher():
        barrier.wait()
        while not done.is_set():
            if abs(LEFT_MOTOR.get_encoder()) >= target_deg:
                LEFT_MOTOR.set_dps(0)
                RIGHT_MOTOR.set_dps(0)
                done.set()
                return
            busy_sleep(dt)

    def correction_loop():
        integral   = 0.0
        prev_error = 0.0
        barrier.wait()
        while not done.is_set():
            error      = absolute_heading() - CURRENT_HEADING
            integral  += error * dt
            derivative = (error - prev_error) / dt
            prev_error = error
            correction = (DRIVE_KP * error) + (DRIVE_KI * integral) + (DRIVE_KD * derivative)
            left_speed  = max(0, min(DRIVE_SPEED, round(DRIVE_SPEED - correction)))
            right_speed = max(0, min(DRIVE_SPEED, round(DRIVE_SPEED + correction)))
            if abs(correction) > 0.5:
                direction = "drifting RIGHT, slowing left & speeding right" if error > 0 else "drifting LEFT, speeding left & slowing right"
                print(f"  [gyro fwd] {direction} | err={error:+.2f}° corr={correction:+.1f} → L={left_speed} R={right_speed}")
            LEFT_MOTOR.set_dps(left_speed)
            RIGHT_MOTOR.set_dps(right_speed * right_sign)
            busy_sleep(dt)

    t_watcher = threading.Thread(target=distance_watcher)
    t_corr    = threading.Thread(target=correction_loop)
    t_watcher.start()
    t_corr.start()
    t_watcher.join()
    t_corr.join()
    LEFT_MOTOR.set_dps(0)
    RIGHT_MOTOR.set_dps(0)

# ==========================================================================
# GYRO-CORRECTED REVERSE DRIVE  (uses global zero — no gyro reset)
# ==========================================================================
def drive_backward(distance_cm: float):
    target_deg = cm_to_degrees(distance_cm)
    right_sign = -1 if RIGHT_MOTOR_FLIPPED else 1

    LEFT_MOTOR.reset_encoder()
    RIGHT_MOTOR.reset_encoder()

    done    = threading.Event()
    dt      = 1.0 / DRIVE_CORRECTION_HZ
    barrier = threading.Barrier(2)

    def distance_watcher():
        barrier.wait()
        while not done.is_set():
            if abs(LEFT_MOTOR.get_encoder()) >= target_deg:
                LEFT_MOTOR.set_dps(0)
                RIGHT_MOTOR.set_dps(0)
                done.set()
                return
            busy_sleep(dt)

    def correction_loop():
        integral   = 0.0
        prev_error = 0.0
        barrier.wait()
        while not done.is_set():
            error      = absolute_heading() - CURRENT_HEADING
            integral  += error * dt
            derivative = (error - prev_error) / dt
            prev_error = error
            correction = (DRIVE_KP * error) + (DRIVE_KI * integral) + (DRIVE_KD * derivative)
            left_speed  = min(0, max(-DRIVE_SPEED, round(-DRIVE_SPEED - correction)))
            right_speed = min(0, max(-DRIVE_SPEED, round(-DRIVE_SPEED + correction)))
            if abs(correction) > 0.5:
                direction = "drifting RIGHT, speeding left & slowing right" if error > 0 else "drifting LEFT, slowing left & speeding right"
                print(f"  [gyro bwd] {direction} | err={error:+.2f}° corr={correction:+.1f} → L={left_speed} R={right_speed}")
            LEFT_MOTOR.set_dps(left_speed)
            RIGHT_MOTOR.set_dps(right_speed * right_sign)
            busy_sleep(dt)

    t_watcher = threading.Thread(target=distance_watcher)
    t_corr    = threading.Thread(target=correction_loop)
    t_watcher.start()
    t_corr.start()
    t_watcher.join()
    t_corr.join()
    LEFT_MOTOR.set_dps(0)
    RIGHT_MOTOR.set_dps(0)

# ==========================================================================
# GYRO-CORRECTED IN-PLACE ROTATION  (updates CURRENT_HEADING)
# ==========================================================================
def rotate(angle_deg: float):
    global CURRENT_HEADING
    CURRENT_HEADING += angle_deg          # advance the global target heading
    target     = CURRENT_HEADING
    integral   = 0.0
    prev_error = 0.0
    dt         = 1.0 / DRIVE_CORRECTION_HZ

    while True:
        error      = target - absolute_heading()
        integral  += error * dt
        derivative = (error - prev_error) / dt
        prev_error = error
        output     = (TURN_KP * error) + (TURN_KI * integral) + (TURN_KD * derivative)
        speed      = max(-TURN_SPEED, min(TURN_SPEED, int(output)))
        LEFT_MOTOR.set_dps(speed)
        RIGHT_MOTOR.set_dps(-speed if not RIGHT_MOTOR_FLIPPED else speed)
        if abs(error) <= TURN_TOLERANCE_DEG:
            break
        busy_sleep(dt)

    LEFT_MOTOR.set_dps(0)
    RIGHT_MOTOR.set_dps(0)

# ==========================================================================
# CONVEYOR HELPERS
# ==========================================================================
def pickup_brick():
    CONVEYOR.set_position_relative(220)

def throw_brick():
    CONVEYOR.set_position_relative(-250)

# ==========================================================================
# NAMED MOVEMENT SHORTCUTS
# ==========================================================================
def turn_90_right():
    rotate(90)

def turn_90_left():
    rotate(-90)

def turn_180():
    rotate(180)

def stop():
    LEFT_MOTOR.set_dps(0)
    RIGHT_MOTOR.set_dps(0)

# ==========================================================================
# COLOR SAMPLING, NORMALIZATION, AND IDENTIFICATION
# ==========================================================================
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

# ==========================================================================
# FORWARD DISTANCE PROFILE
# ==========================================================================
# Minimum forward distance driven even at the furthest sweep angle.
SWEEP_CREEP_MIN_CM = 1.0

def _forward_distance_for_angle(sensor_angle: float) -> float:
    """
    Return how many cm forward the robot should drive (straight ahead,
    before rotating) when green was detected at `sensor_angle` degrees.
    The profile scales between SWEEP_CREEP_MIN_CM and SWEEP_CREEP_MAX_CM:
      - SWEEP_CREEP_MAX_CM at SWEEP_CENTRE_ANGLE (-60deg) -- closest target
      - SWEEP_CREEP_MIN_CM at the furthest sweep edge     -- always non-zero
    """
    first_angle = SWEEP_ANGLE
    last_angle  = MEASUREMENT_NB * SWEEP_ANGLE
    half_range = min(
        abs(SWEEP_CENTRE_ANGLE - first_angle),
        abs(SWEEP_CENTRE_ANGLE - last_angle),
    )
    if half_range == 0:
        return SWEEP_CREEP_MAX_CM
    angular_distance = abs(sensor_angle - SWEEP_CENTRE_ANGLE)
    fraction = max(0.0, 1.0 - angular_distance / half_range)
    return SWEEP_CREEP_MIN_CM + (SWEEP_CREEP_MAX_CM - SWEEP_CREEP_MIN_CM) * fraction + 3.0

# ==========================================================================
# SWEEP AND DETECT
# ==========================================================================
def sweep_and_detect():
    """
    Sweep the rotating sensor motor MEASUREMENT_NB steps of SWEEP_ANGLE
    degrees each. At each step, identify the color using Euclidean distance.
    The robot does NOT move during the sweep.
    Skips yellow (not actionable), acts on red or green immediately.
    Returns (color, cumulative_angle) for red/green, or (None, 0) if the
    full sweep completes with only yellow readings.
    """
    cumulative_angle = 0
    for _ in range(MEASUREMENT_NB):
        rotate_sensor_relative(SWEEP_ANGLE)
        cumulative_angle += SWEEP_ANGLE
        detected = identify_color()
        if detected == "yellow":
            print(f"  Yellow at {cumulative_angle}° — skipping.")
            continue
        if detected == "red":
            print(f"  Red confirmed at {cumulative_angle}°.")
            return "red", cumulative_angle
        if detected == "green":
            print(f"  Green confirmed at {cumulative_angle}°.")
            return "green", cumulative_angle
    return None, 0

# ==========================================================================
# HARDCODED NAVIGATION ROUTINES
# ==========================================================================
def go_pickup_blocks():
    block_picked_up = False
    while not block_picked_up:
        drive_forward(14.5)
        busy_sleep(0.1)
        rotate(90)
        busy_sleep(0.1)
        drive_forward(22)
        busy_sleep(0.1)
        rotate(90)
        busy_sleep(0.1)
        drive_forward(22.5)
        busy_sleep(0.5)
        pickup_brick()
        busy_sleep(3)
        drive_forward(4.4)
        busy_sleep(0.1)
        pickup_brick()
        block_picked_up = True

def main_hardcode_brick_to_first_room():
    drive_forward(34)

def hardcode_first_room_to_second_room():
    drive_backward(16)
    rotate(30)
    drive_forward(5)
    rotate(30)
    drive_forward(5)
    rotate(30)
    drive_forward(36)
    busy_sleep(0.1)
    rotate(-90)
    busy_sleep(0.1)
    drive_forward(12)
    busy_sleep(0.1)

def hardcode_second_room_to_third_room():
    drive_backward(12)
    busy_sleep(0.1)
    rotate(90)
    busy_sleep(0.1)
    drive_forward(24.5)
    busy_sleep(0.1)
    rotate(90)
    busy_sleep(0.1)
    drive_forward(12)

def hardcode_third_room_to_mid():
    drive_backward(12)
    busy_sleep(0.1)
    rotate(-90)
    busy_sleep(0.1)
    drive_forward(12.25)
    busy_sleep(0.1)
    rotate(90)
    busy_sleep(0.1)
    drive_forward(12)

def hardcode_mid_room_to_end():
    drive_backward(12)
    busy_sleep(0.1)
    rotate(-90)
    busy_sleep(0.1)
    drive_forward(12.25)
    busy_sleep(0.1)
    rotate(90)
    busy_sleep(0.1)
    drive_forward(12)

# ==========================================================================
# ROOM NAVIGATION  (continuous sweep-and-drive loop)
# ==========================================================================
def room_navigation():
    """
    Continuously sweep for colors and drive accordingly.

    GREEN detected — sequence:
        1. Sensor returns to 0°.
        2. Robot drives STRAIGHT FORWARD by the tent-function amount based
           on where green was found (peak at SWEEP_CENTRE_ANGLE, min at edges).
        3. Robot rotates to face the green target.
        4. Robot drives 2.5 cm forward to get close to the target.
        5. Robot drops the brick.
        6. Robot drives 2 cm backward after throwing.
        7. Robot rotates back to original heading.
        8. Robot drives backward (approach_cm + 0.5 cm net from steps 4+6).
        9. Robot reverses all room_navigation forward progress exactly.

    RED detected:
        - Robot reverses all accumulated forward distance back to start.
        - Loop ends.

    Full sweep with only yellow / nothing actionable:
        - Robot drives forward 3 cm and sweeps again.
    """
    forward_const = 0
    forward_cm    = 3.0

    while True:
        print(f"\n[room_navigation] sweep #{forward_const + 1} "
              f"(total forward so far: {forward_cm * forward_const:.1f} cm)")

        color, sensor_angle = sweep_and_detect()

        print("  Returning sensor to 0°...")
        rotate_sensor_to(0)

        # ── GREEN ──────────────────────────────────────────────────────────
        if color == "green":
            approach_cm = _forward_distance_for_angle(sensor_angle)
            print(f"  GREEN at sensor angle {sensor_angle}° → "
                  f"approach distance {approach_cm:.2f} cm.")

            # 1. Drive straight forward based on how central the angle is
            if approach_cm > 0.01:
                print(f"  Driving forward {approach_cm:.2f} cm (straight, before rotating)...")
                drive_forward(approach_cm)

            # 2. Rotate to face the green target.
            robot_rotation = -(sensor_angle - SWEEP_CENTRE_ANGLE)
            if abs(robot_rotation) > 0.5:
                print(f"  Rotating {robot_rotation:+.1f}° to face target...")
                rotate(robot_rotation)
                busy_sleep(0.1)
            else:
                print("  Target is straight ahead — no rotation needed.")

            # 3. Drive 2.5 cm forward to get close before throwing
            print("  Driving 2.5 cm forward to approach target...")
            drive_forward(2.5)

            # 4. Drop the brick
            print("  Throwing brick...")
            rotate_sensor_to(0)
            throw_brick()
            busy_sleep(1.0)


            green_counter += 1
            
            # 5. Drive 2 cm backward after throwing
            print("  Reversing 2 cm after throw...")
            drive_backward(2.0)

            # 6. Rotate back to original heading
            if abs(robot_rotation) > 0.5:
                print(f"  Rotating back {-robot_rotation:+.1f}°...")
                rotate(-robot_rotation)
                busy_sleep(0.1)

            # 7. Drive backward to undo the net forward movement from steps 3+5
            net_after_throw = 2.5 - 2.0  # = 0.5 cm
            if net_after_throw > 0.01:
                print(f"  Reversing net post-throw distance {net_after_throw:.2f} cm...")
                drive_backward(net_after_throw)

            # 8. Drive backward to undo the initial straight approach
            if approach_cm > 0.01:
                print(f"  Reversing approach distance {approach_cm:.2f} cm...")
                drive_backward(approach_cm)

            # 9. Reverse all room_navigation forward progress exactly
            total_reverse = forward_cm * forward_const
            if total_reverse > 0.01:
                print(f"  Reversing {total_reverse:.1f} cm back to start...")
                drive_backward(total_reverse)

            print("  Done — room navigation complete.")
            break

        # ── RED ────────────────────────────────────────────────────────────
        elif color == "red":
            print(f"  RED at sensor angle {sensor_angle}°.")
            total_reverse = forward_cm * forward_const
            if total_reverse > 0:
                print(f"  Reversing {total_reverse:.1f} cm back to start...")
                drive_backward(total_reverse)
                forward_const = 0
            else:
                print("  No forward distance to reverse.")
            print("  Done — room navigation complete.")
            break

        # ── ALL YELLOW / NOTHING ACTIONABLE ────────────────────────────────
        else:
            print(f"  No red/green found — driving forward {forward_cm} cm.")
            drive_forward(forward_cm)
            forward_const += 1

# ==========================================================================
# ENTRY POINT
# ==========================================================================
if __name__ == "__main__":
    init_gyro_zero()   # set global zero ONCE — gyro is never reset again
    room_navigation()
