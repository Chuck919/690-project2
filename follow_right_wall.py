"""
E-Puck Right-Wall-Following Maze Controller for Webots
======================================================
Pure reactive control — no grid tracking, no encoders, no open-loop step counts.

Uses all 8 proximity sensors simultaneously every time step:
  ps0 = front-right (~10°)     ps7 = front-left (~350°)
  ps1 = right-front (~45°)     ps6 = left-front (~315°)
  ps2 = right       (~90°)     ps5 = left       (~270°)
  ps3 = right-back  (~135°)    ps4 = left-back  (~225°)

Algorithm:
  Right-wall follower with proportional distance-keeping.
  Stops when the camera sees the green wall.
"""

from controller import Robot, Camera

# ═══════════════════════════════════════════════════════════════
#  TUNABLE PARAMETERS — adjust these to change behaviour
# ═══════════════════════════════════════════════════════════════

TIME_STEP   = 64              # simulation step (ms) — match your world

# -- Speeds ----------------------------------------------------------
MAX_SPEED   = 6.28            # E-Puck max wheel speed (rad/s)
BASE_SPEED  = 0.75 * MAX_SPEED   # cruising speed while wall-following
TURN_SPEED  = 0.75  * MAX_SPEED   # speed for in-place left turns
VEER_LEFT   = 1 * MAX_SPEED   # left wheel speed when veering right (outer wheel)
VEER_RIGHT  = 0.01 * MAX_SPEED   # right wheel speed when veering right (inner wheel)

# -- Proximity thresholds -------------------------------------------
#   Sensors return ≈ 0 (nothing) up to ≈ 4000+ (touching wall).
#   Lower value = reacts further from the wall.
FRONT_WALL  = 80.0            # front sensors above this → "wall ahead"
SIDE_WALL   = 70.0            # right sensors above this → "wall on right"
TOO_CLOSE   = 150.0           # side sensor above this → "dangerously close"

# -- Wall-following PD controller ------------------------------------
TARGET_DIST   = 80.0          # desired right-side sensor reading (lower = farther)
DIST_WEIGHT   = 0.7           # weight for distance error
ANGLE_WEIGHT  = 0.3           # weight for angle error (front vs back)
PD_GAIN       = 0.003         # proportional gain for correction
EMERGENCY_PUSH = 200          # extra correction when TOO_CLOSE

# -- Green detection -------------------------------------------------
GREEN_RATIO = 1.2             # green avg must be > red*ratio AND > blue*ratio
RAM_STEPS   = 15              # steps to ram into green wall after detection
RAM_SPEED   = 0.3 * MAX_SPEED

# -- Startup ---------------------------------------------------------
WARMUP_STEPS = 10             # sim steps to let sensors settle before moving

# ═══════════════════════════════════════════════════════════════
#  ROBOT & DEVICE SETUP
# ═══════════════════════════════════════════════════════════════
robot = Robot()

# 8 proximity sensors
ps = []
for i in range(8):
    s = robot.getDevice(f'ps{i}')
    s.enable(TIME_STEP)
    ps.append(s)

# Wheel motors (velocity control)
leftMotor  = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

# Camera
camera = robot.getDevice('camera')
camera.enable(TIME_STEP)

# ═══════════════════════════════════════════════════════════════
#  GREEN DETECTION
# ═══════════════════════════════════════════════════════════════
def sees_green():
    """Return True when the camera image is dominated by green."""
    img = camera.getImage()
    if img is None:
        return False
    w, h = camera.getWidth(), camera.getHeight()
    rt = gt = bt = 0
    n = w * h
    for x in range(w):
        for y in range(h):
            rt += Camera.imageGetRed(img, w, x, y)
            gt += Camera.imageGetGreen(img, w, x, y)
            bt += Camera.imageGetBlue(img, w, x, y)
    ra, ga, ba = rt / n, gt / n, bt / n
    return ga > ra * GREEN_RATIO and ga > ba * GREEN_RATIO

# ═══════════════════════════════════════════════════════════════
#  WAIT FOR SENSORS TO SETTLE
# ═══════════════════════════════════════════════════════════════
for _ in range(WARMUP_STEPS):
    robot.step(TIME_STEP)

print("=" * 50)
print("  RIGHT-WALL FOLLOWER — running")
print("=" * 50)

# ═══════════════════════════════════════════════════════════════
#  MAIN LOOP — right-wall follower
#
#  Decision priority every time step:
#    1. Green detected ahead → stop (maze solved!)
#    2. Front blocked → turn left in place
#    3. No wall on right → veer right to re-acquire wall
#    4. Wall on right → drive forward, proportionally keeping
#       a set distance from the right wall
# ═══════════════════════════════════════════════════════════════
while robot.step(TIME_STEP) != -1:
    # Read all 8 sensors
    v = [s.getValue() for s in ps]

    # ── Sensor aliases ──────────────────────────────────────
    front_left   = v[7]   # ~350°
    front_right  = v[0]   # ~10°
    right_front  = v[1]   # ~45°
    right_side   = v[2]   # ~90°
    right_back   = v[3]   # ~135°
    left_back    = v[4]   # ~225°
    left_side    = v[5]   # ~270°
    left_front   = v[6]   # ~315°

    # Composite checks
    front_blocked = (front_left > FRONT_WALL or front_right > FRONT_WALL)
    right_exists  = (right_side > SIDE_WALL or right_front > SIDE_WALL)

    # ── 1. CHECK FOR GREEN ──────────────────────────────────
    # ── 1. CHECK FOR GREEN (every step, from any distance) ────
    if sees_green():
            leftMotor.setVelocity(BASE_SPEED)
            rightMotor.setVelocity(BASE_SPEED)
            continue

    # ── 2. FRONT BLOCKED → turn left in place ───────────────
    if front_blocked:
        leftMotor.setVelocity(-TURN_SPEED)
        rightMotor.setVelocity(TURN_SPEED)
        continue

    # ── 3. NO WALL ON RIGHT → veer right to find wall ──────
    if not right_exists:
        # Gentle right curve: left wheel fast, right wheel slow
        leftMotor.setVelocity(VEER_LEFT)
        rightMotor.setVelocity(VEER_RIGHT)
        continue

    # ── 4. WALL ON RIGHT → forward with proportional correction ─
    #
    #  error > 0 → too close to right wall → steer left
    #  error < 0 → too far from right wall → steer right

    # Distance error: keep right_side near TARGET_DIST
    dist_error  = right_side - TARGET_DIST

    # Angle error: if right_front > right_back, nose is angled toward wall
    angle_error = right_front - right_back

    # Combine
    total_error = DIST_WEIGHT * dist_error + ANGLE_WEIGHT * angle_error

    # Emergency: if left side is very close, push away from left wall
    if left_side > TOO_CLOSE or left_front > TOO_CLOSE:
        total_error -= EMERGENCY_PUSH

    # Emergency: if right side is very close, push away from right wall
    if right_side > TOO_CLOSE or right_front > TOO_CLOSE:
        total_error += EMERGENCY_PUSH

    correction = total_error * PD_GAIN

    left_speed  = BASE_SPEED + correction
    right_speed = BASE_SPEED - correction

    # Clamp — never fully stop a wheel (minimum keeps it moving)
    left_speed  = max(0.05 * MAX_SPEED, min(MAX_SPEED, left_speed))
    right_speed = max(0.05 * MAX_SPEED, min(MAX_SPEED, right_speed))

    leftMotor.setVelocity(left_speed)
    rightMotor.setVelocity(right_speed)
