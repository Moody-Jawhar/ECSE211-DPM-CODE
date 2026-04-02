from utils.brick import Motor, EV3GyroSensor, EV3UltrasonicSensor, EV3ColorSensor, TouchSensor, configure_ports, busy_sleep
import math
import threading
import brickpi3

BP = brickpi3.BrickPi3()





COLOR_SENSOR, TOUCH_SENSOR, GYRO, ULTRA, LEFT_MOTOR, RIGHT_MOTOR, CONVEYOR, ROTATING_MOTOR = configure_ports(
    PORT_1=EV3ColorSensor,
    PORT_2=TouchSensor,
    PORT_3=EV3GyroSensor,
    PORT_4=EV3UltrasonicSensor,
    PORT_A=Motor,
    PORT_B=Motor,
    PORT_C=Motor,
    PORT_D=Motor,
)

busy_sleep(3.0)  # FIX 1: sensors need time to initialize before use

#diameter of one of the wheels
WHEEL_DIAMETER_CM   = 4

#width between two wheels
AXLE_WIDTH_CM       = 13.175

#speed of driving
DRIVE_SPEED         = 300

#speed of turning
TURN_SPEED          = 300

RIGHT_MOTOR_FLIPPED = False

#CONSTANTD TO BE FIXED
DRIVE_KP = 8.0
DRIVE_KI = 0.1
DRIVE_KD = 2.0

# Average multiple gyro reads per PID cycle to reduce noise on short drives.
# Higher sample count + median filter to reject spike outliers and reduce drift.
GYRO_SAMPLES_PER_CYCLE = 15

#constant that fixes turning
TURN_KP = 6.0
TURN_KI = 0.05
TURN_KD = 1.5
TURN_TOLERANCE_DEG  = 1.0
DRIVE_CORRECTION_HZ = 50
WHEEL_CIRCUMFERENCE_CM = math.pi * WHEEL_DIAMETER_CM

GLOBAL_ZERO: float = 0.0       # raw gyro reading captured once at startup
CURRENT_HEADING: float = 0.0   # cumulative heading offset from GLOBAL_ZERO




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
    Reads the gyro n times in quick succession.
    Drops the top and bottom outlier quartile (25 % each side), then
    returns the mean of the remaining middle 50 % of readings.
    This rejects momentary sensor spikes that would otherwise cause the
    PID to over-correct and introduce drift on short drives.
    Requires n >= 4; falls back to a plain mean for smaller n.
    """
    readings = []
    for _ in range(n):
        readings.append(gyro_angle())
    if n < 4:
        return sum(readings) / n
    readings.sort()
    trim = max(1, n // 4)          # drop ~25 % from each end
    trimmed = readings[trim:-trim]
    return sum(trimmed) / len(trimmed)




def cm_to_degrees(cm: float) -> float:
    return (cm / WHEEL_CIRCUMFERENCE_CM) * 360


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
            left_speed  = max(0, min(DRIVE_SPEED, round(DRIVE_SPEED - correction)))  # FIX 2: was + correction
            right_speed = max(0, min(DRIVE_SPEED, round(DRIVE_SPEED + correction)))  # FIX 2: was - correction
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
    """
    Drive straight backward by distance_cm using gyro-corrected PID.

    Sign convention (reverse):
      - Both motors run at NEGATIVE dps (wheels spin backward).
      - If the robot drifts RIGHT (error > 0), we must speed up the LEFT
        wheel and slow the RIGHT wheel to push the nose back left.
        → left_speed  becomes MORE negative  (subtract correction)
        → right_speed becomes LESS  negative  (add    correction)
      This is the OPPOSITE of the forward case, because the robot is
      travelling in the opposite direction relative to the gyro frame.
    """
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

            # When reversing, correction direction flips:
            #   drift RIGHT → LEFT motor needs to go faster backward (more negative)
            #                  RIGHT motor needs to go slower backward (less negative)
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
# ABSOLUTE HEADING ALIGN  — 3 speed intervals, 4 direction cases
#
#   |error| > ALIGN_FAST_THRESHOLD   → TURN_SPEED   (fast)
#   |error| > ALIGN_MEDIUM_THRESHOLD → ALIGN_MEDIUM_SPEED (medium)
#   |error| > ALIGN_TOLERANCE        → ALIGN_CREEP_SPEED  (creep)
#   |error| <= ALIGN_TOLERANCE       → stop
# ==========================================================================
ALIGN_FAST_THRESHOLD   = 20.0   # deg: above this → fast
ALIGN_MEDIUM_THRESHOLD = 5.0    # deg: above this → medium
ALIGN_TOLERANCE        = 0.01   # deg: settle band
ALIGN_MEDIUM_SPEED     = 150    # dps
ALIGN_CREEP_SPEED      = 40     # dps — do not go below ~35 or motors stall
ALIGN_MAX_LOOPS        = 500    # safety exit if gyro cannot reach 0.01 deg


def align(target_angle_deg: float):
    """
    Rotate to an absolute heading from the startup zero (init_gyro_zero).

      align(0)   -> startup facing direction
      align(90)  -> 90 deg clockwise from zero
      align(-90) -> 90 deg counter-clockwise from zero

    Three speed intervals, four direction cases.
    Does NOT call rotate(). Re-syncs GLOBAL_ZERO on exit.
    """
    global CURRENT_HEADING, GLOBAL_ZERO

    dt    = 1.0 / DRIVE_CORRECTION_HZ
    loops = 0
    print(f"[align] target={target_angle_deg:+.1f}  "
          f"actual={(gyro_angle() - GLOBAL_ZERO):+.1f}")

    while True:
        actual = gyro_angle() - GLOBAL_ZERO
        error  = target_angle_deg - actual
        loops += 1

        if abs(error) <= ALIGN_TOLERANCE or loops >= ALIGN_MAX_LOOPS:
            break

        # 4 cases → 3 speed levels
        if error > ALIGN_FAST_THRESHOLD:
            speed =  TURN_SPEED                  # case 1: far right → fast right
        elif error > ALIGN_MEDIUM_THRESHOLD:
            speed =  ALIGN_MEDIUM_SPEED          # case 2: near right → medium right
        elif error > 0:
            speed =  ALIGN_CREEP_SPEED           # case 2b: very near right → creep right
        elif error < -ALIGN_FAST_THRESHOLD:
            speed = -TURN_SPEED                  # case 4: far left → fast left
        elif error < -ALIGN_MEDIUM_THRESHOLD:
            speed = -ALIGN_MEDIUM_SPEED          # case 3: near left → medium left
        else:
            speed = -ALIGN_CREEP_SPEED           # case 3b: very near left → creep left

        LEFT_MOTOR.set_dps(speed)
        RIGHT_MOTOR.set_dps(-speed if not RIGHT_MOTOR_FLIPPED else speed)
        busy_sleep(dt)

    LEFT_MOTOR.set_dps(0)
    RIGHT_MOTOR.set_dps(0)
    busy_sleep(0.02)
    LEFT_MOTOR.set_position_relative(0)
    RIGHT_MOTOR.set_position_relative(0)

    raw_now         = gyro_angle_avg()
    CURRENT_HEADING = target_angle_deg
    GLOBAL_ZERO     = raw_now - CURRENT_HEADING
    print(f"[align] done loops={loops}  "
          f"settled={(gyro_angle_avg() - GLOBAL_ZERO + CURRENT_HEADING):+.1f}  "
          f"GLOBAL_ZERO={GLOBAL_ZERO:.2f}")


# ==========================================================================
# ULTRASONIC HELPERS
# ==========================================================================
ULTRA_SAMPLES  = 5     # readings averaged per measurement
ULTRA_STEP_CM  = 3.0   # cm driven per iteration — smaller = more accurate stop

# X correction: if side reading drifts by more than this from baseline, correct
ULTRA_X_TOLERANCE = 1.0   # cm — lateral error threshold before a correction step


def read_ultra() -> float:
    """
    Take ULTRA_SAMPLES readings, drop min and max, return trimmed mean (cm).
    """
    readings = []
    for _ in range(ULTRA_SAMPLES):
        for _ in range(10):
            val = ULTRA.get_value()
            if val is not None:
                readings.append(val[0] if isinstance(val, (list, tuple)) else val)
                break
        busy_sleep(0.02)
    if not readings:
        raise RuntimeError("Ultrasonic returned no valid readings.")
    if len(readings) >= 3:
        readings = sorted(readings)[1:-1]
    return sum(readings) / len(readings)




# ==========================================================================
# DRIVE TO Y  — move along the forward axis until front reading = target_y
#
#   target_y: desired front ultrasonic reading (cm)
#   smaller  = closer to the wall in front
#   larger   = farther from the wall in front
#
#   Each iteration: align(0) → read Y → step forward or backward → repeat
#   Stops when |y_now - target_y| <= ULTRA_Y_TOLERANCE
# ==========================================================================
ULTRA_Y_TOLERANCE = 0.5   # cm — stop when within this band of target_y


def drive_to_y(target_y: float):
    """
    Drive forward or backward until the front ultrasonic reading equals target_y.

      target_y < current reading  →  drive forward  (approach front wall)
      target_y > current reading  →  drive backward (move away from front wall)

    Leaves the robot facing 0 deg on exit.
    """
    print(f"[drive_to_y] target_y={target_y:.2f} cm")

    while True:
        align(0)
        y_now  = read_ultra()
        y_error = y_now - target_y   # positive → too far → drive forward

        print(f"  [ultra] Y={y_now:.2f}  target={target_y:.2f}  error={y_error:+.2f}")

        if abs(y_error) <= ULTRA_Y_TOLERANCE:
            print("[drive_to_y] Y target reached.")
            align(0)
            break

        if y_error > 0:
            drive_forward(min(ULTRA_STEP_CM, abs(y_error)))   # approach
        else:
            drive_backward(min(ULTRA_STEP_CM, abs(y_error)))  # retreat


# ==========================================================================
# DRIVE TO X  — move along the side axis until side reading = target_x
#
#   target_x: desired side ultrasonic reading (cm)
#   smaller  = closer to the side wall
#   larger   = farther from the side wall
#
#   Each iteration: align(90) → read X → align(0) → step right/left → repeat
#   Stops when |x_now - target_x| <= ULTRA_X_TOLERANCE
# ==========================================================================
ULTRA_X_TOLERANCE = 1.0   # cm — stop when within this band of target_x


def drive_to_x(target_x: float):
    """
    Drive sideways until the side ultrasonic reading equals target_x.
    The robot faces 90 deg to read X, then drives forward/backward at 90 deg
    to move along the X axis, then returns to 0 deg between checks.

      target_x < current reading  →  drive toward side wall  (align 90, forward)
      target_x > current reading  →  drive away from side wall (align 90, backward)

    Leaves the robot facing 0 deg on exit.
    """
    print(f"[drive_to_x] target_x={target_x:.2f} cm")

    while True:
        align(90)
        x_now   = read_ultra()
        x_error = x_now - target_x   # positive → too far from side wall → move toward it

        print(f"  [ultra] X={x_now:.2f}  target={target_x:.2f}  error={x_error:+.2f}")

        if abs(x_error) <= ULTRA_X_TOLERANCE:
            print("[drive_to_x] X target reached.")
            align(0)
            break

        if x_error > 0:
            drive_forward(min(ULTRA_STEP_CM, abs(x_error)))   # approach side wall
        else:
            drive_backward(min(ULTRA_STEP_CM, abs(x_error)))  # retreat from side wall


def align_2(x):
    cur_angle = GYRO.get_abs_measure()
    if x > 0:
        if x > cur_angle:
            while True:
                cur_angle = GYRO.get_abs_measure()
                #print(cur_angle)
                error = x - cur_angle
                error = abs(error)
                if error == 0:
                    LEFT_MOTOR.set_power(0)
                    RIGHT_MOTOR.set_power(0)
                    break
                if error < 10:
                    LEFT_MOTOR.set_power(11)
                    RIGHT_MOTOR.set_power(-11)
                else:
                    LEFT_MOTOR.set_power(20)
                    RIGHT_MOTOR.set_power(-20)


if __name__ == "__main__":
    #init_gyro_zero()
    #drive_to_y(30)   # stop when front wall is 30 cm away
    #drive_to_x(15)   # then slide until side wall is 15 cm away
    align_2(180)
    #LEFT_MOTOR.set_power(0)
    #RIGHT_MOTOR.set_power(0)



