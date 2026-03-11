"""
E-Puck Flood-Fill Maze Controller for Webots
=============================================
Phase 1 – EXPLORE : Systematic BFS-nearest-unvisited exploration.
          Probe-based wall detection: to check for a wall the robot
          drives forward ~8 cm at low speed, checks if front sensors
          rise above 120 (clearly a wall), then backs up to cell
          centre.  This avoids the noise-floor problem (sensors read
          60-77 whether or not a wall is 12.5 cm away at cell centre).
Phase 2 – RETURN  : BFS shortest path back to start.  Touch RED wall.
Phase 3 – SPEED   : BFS shortest path to GREEN wall.  Ram it to finish.

Movement is fully encoder-based: precise 90°/180° turns with stuck
detection, cell-to-cell drives with side-wall correction.

6×6 grid  ·  0.25 m cells  ·  1.5 m arena
Uses: 8×proximity, 2×wheel motors, 2×wheel encoders, 1×camera.
"""

from controller import Robot, Camera
from collections import deque
import math

# ═══════════════════════════════════════════════════════════════
#  TUNABLES
# ═══════════════════════════════════════════════════════════════

TIME_STEP = 64

# -- Speeds -----------------------------------------------------------
MAX_SPEED      = 6.28
PROBE_SPEED    = 0.12 * MAX_SPEED   # very slow for probing walls
EXPLORE_SPEED  = 0.35 * MAX_SPEED   # cell-to-cell during mapping
FAST_SPEED     = 0.55 * MAX_SPEED   # speed-run
TURN_SPEED_VAL = 0.20 * MAX_SPEED   # in-place turns

# -- Probe wall detection ---------------------------------------------
#    Noise floor is 58-77.  At ~4 cm from wall sensors read > 200.
#    Probe drives forward ~8 cm from centre → ~4.5 cm from wall.
PROBE_DIST  = 4.0     # encoder rads to creep forward (~8 cm)
WALL_DETECT = 120     # front sensor above this → wall confirmed

# -- Emergency stop during cell drive (very high → imminent crash) -----
FRONT_STOP = 500

# -- Colour detection --------------------------------------------------
GREEN_RATIO = 1.2
RED_RATIO   = 1.2
RAM_STEPS   = 20
RAM_SPEED   = 0.30 * MAX_SPEED

# -- Encoder geometry --------------------------------------------------
GRID_SIZE       = 6
CELL_SIZE       = 0.25
WHEEL_RADIUS    = 0.0205
AXLE_LENGTH     = 0.052
ENCODER_PER_CELL = CELL_SIZE / WHEEL_RADIUS                      # ≈ 12.2
ENCODER_TURN_90  = (math.pi / 2 * AXLE_LENGTH / 2) / WHEEL_RADIUS  # ≈ 2.0

# -- Directions --------------------------------------------------------
NORTH, EAST, SOUTH, WEST = 0, 1, 2, 3
DIR_NAME = {0: 'N', 1: 'E', 2: 'S', 3: 'W'}
DR = [-1,  0,  1,  0]
DC = [ 0,  1,  0, -1]

START_ROW     = GRID_SIZE - 1
START_COL     = 0
START_HEADING = NORTH
WARMUP_STEPS  = 15

# ═══════════════════════════════════════════════════════════════
#  ROBOT SETUP
# ═══════════════════════════════════════════════════════════════

robot = Robot()

ps = []
for i in range(8):
    s = robot.getDevice(f'ps{i}')
    s.enable(TIME_STEP)
    ps.append(s)

leftMotor  = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

leftEncoder  = robot.getDevice('left wheel sensor')
rightEncoder = robot.getDevice('right wheel sensor')
leftEncoder.enable(TIME_STEP)
rightEncoder.enable(TIME_STEP)

camera = robot.getDevice('camera')
camera.enable(TIME_STEP)

# ═══════════════════════════════════════════════════════════════
#  MAZE DATA
# ═══════════════════════════════════════════════════════════════

walls   = [[set() for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
visited = [[False] * GRID_SIZE for _ in range(GRID_SIZE)]

for i in range(GRID_SIZE):
    walls[0][i].add(NORTH)
    walls[GRID_SIZE - 1][i].add(SOUTH)
    walls[i][0].add(WEST)
    walls[i][GRID_SIZE - 1].add(EAST)

green_cell = None
green_dir  = None

cur_r   = START_ROW
cur_c   = START_COL
heading = START_HEADING

# ═══════════════════════════════════════════════════════════════
#  LOW-LEVEL HELPERS
# ═══════════════════════════════════════════════════════════════

def get_enc():
    lv = leftEncoder.getValue()
    rv = rightEncoder.getValue()
    if math.isnan(lv): lv = 0.0
    if math.isnan(rv): rv = 0.0
    return lv, rv


def add_wall(r, c, d):
    if 0 <= r < GRID_SIZE and 0 <= c < GRID_SIZE:
        walls[r][c].add(d)
    nr, nc = r + DR[d], c + DC[d]
    if 0 <= nr < GRID_SIZE and 0 <= nc < GRID_SIZE:
        walls[nr][nc].add((d + 2) % 4)


def stop():
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)
    robot.step(TIME_STEP)


def settle(n=3):
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)
    for _ in range(n):
        robot.step(TIME_STEP)


def _avg_colour():
    img = camera.getImage()
    if img is None:
        return None
    w, h = camera.getWidth(), camera.getHeight()
    rt = gt = bt = 0
    n = w * h
    for x in range(w):
        for y in range(h):
            rt += Camera.imageGetRed(img, w, x, y)
            gt += Camera.imageGetGreen(img, w, x, y)
            bt += Camera.imageGetBlue(img, w, x, y)
    return rt / n, gt / n, bt / n


def sees_green():
    c = _avg_colour()
    if c is None:
        return False
    r, g, b = c
    return g > r * GREEN_RATIO and g > b * GREEN_RATIO


def sees_red():
    c = _avg_colour()
    if c is None:
        return False
    r, g, b = c
    return r > g * RED_RATIO and r > b * RED_RATIO


# ═══════════════════════════════════════════════════════════════
#  ENCODER-BASED TURNS  (with stuck detection)
# ═══════════════════════════════════════════════════════════════

def turn_by_encoder(left_vel, right_vel, target_rads):
    """In-place turn with deceleration and stuck detection.
    If the robot catches on a wall, it nudges backward and retries."""
    settle(2)
    l0, r0 = get_enc()
    prev_avg = 0.0
    stuck_count = 0

    leftMotor.setVelocity(left_vel)
    rightMotor.setVelocity(right_vel)

    for _ in range(600):
        if robot.step(TIME_STEP) == -1:
            break
        l1, r1 = get_enc()
        avg = (abs(l1 - l0) + abs(r1 - r0)) / 2.0

        # Decelerate for last 25 %
        if avg >= target_rads * 0.75:
            leftMotor.setVelocity(left_vel  * 0.30)
            rightMotor.setVelocity(right_vel * 0.30)
        if avg >= target_rads:
            break

        # Stuck detection: if almost no progress for 8 consecutive steps
        if abs(avg - prev_avg) < 0.003:
            stuck_count += 1
            if stuck_count >= 8:
                print(f"    [STUCK] Turn stuck at {avg:.2f}/{target_rads:.2f} — nudging back")
                stop()
                leftMotor.setVelocity(-0.12 * MAX_SPEED)
                rightMotor.setVelocity(-0.12 * MAX_SPEED)
                for _ in range(8):
                    robot.step(TIME_STEP)
                stop()
                # Resume turn
                leftMotor.setVelocity(left_vel)
                rightMotor.setVelocity(right_vel)
                stuck_count = 0
        else:
            stuck_count = 0
        prev_avg = avg

    settle(2)


def do_turn_right():
    global heading
    turn_by_encoder(TURN_SPEED_VAL, -TURN_SPEED_VAL, ENCODER_TURN_90)
    heading = (heading + 1) % 4

def do_turn_left():
    global heading
    turn_by_encoder(-TURN_SPEED_VAL, TURN_SPEED_VAL, ENCODER_TURN_90)
    heading = (heading - 1) % 4

def do_turn_180():
    global heading
    turn_by_encoder(TURN_SPEED_VAL, -TURN_SPEED_VAL, ENCODER_TURN_90 * 2)
    heading = (heading + 2) % 4


def face(direction):
    """Turn to face *direction*.  Pre-checks for close front wall
    and backs up if needed to avoid catching during rotation."""
    diff = (direction - heading) % 4
    if diff == 0:
        return

    # If front sensor is high, back up a bit first to create clearance
    robot.step(TIME_STEP)
    f_max = max(ps[0].getValue(), ps[7].getValue())
    if f_max > 150:
        leftMotor.setVelocity(-0.10 * MAX_SPEED)
        rightMotor.setVelocity(-0.10 * MAX_SPEED)
        for _ in range(10):
            robot.step(TIME_STEP)
            if max(ps[0].getValue(), ps[7].getValue()) < 100:
                break
        stop()

    if   diff == 1: do_turn_right()
    elif diff == 2: do_turn_180()
    elif diff == 3: do_turn_left()


# ═══════════════════════════════════════════════════════════════
#  PROBE-BASED WALL DETECTION
# ═══════════════════════════════════════════════════════════════

def probe_direction(d):
    """Face direction d, creep forward, check if wall exists.
    Also checks camera for green when wall is found close-up.
    Backs up to starting encoder position afterwards.
    Returns True if wall detected."""
    global green_cell, green_dir

    face(d)
    l0, r0 = get_enc()
    wall_found = False

    leftMotor.setVelocity(PROBE_SPEED)
    rightMotor.setVelocity(PROBE_SPEED)

    for _ in range(300):
        if robot.step(TIME_STEP) == -1:
            break
        l1, r1 = get_enc()
        fwd = ((l1 - l0) + (r1 - r0)) / 2.0

        f_max = max(ps[0].getValue(), ps[7].getValue())
        if f_max > WALL_DETECT:
            wall_found = True
            break
        if fwd >= PROBE_DIST:
            break

    stop()

    # If wall found and we're close, check camera for green
    if wall_found and green_cell is None:
        robot.step(TIME_STEP)
        if sees_green():
            green_cell = (cur_r, cur_c)
            green_dir  = d
            print(f"  [PROBE] *** GREEN at ({cur_r},{cur_c}) "
                  f"facing {DIR_NAME[d]} ***")

    # Back up to starting position
    l_end, r_end = get_enc()
    actual_fwd = ((l_end - l0) + (r_end - r0)) / 2.0

    if actual_fwd > 0.3:
        leftMotor.setVelocity(-PROBE_SPEED)
        rightMotor.setVelocity(-PROBE_SPEED)
        for _ in range(300):
            if robot.step(TIME_STEP) == -1:
                break
            lc, rc = get_enc()
            backed = ((l_end - lc) + (r_end - rc)) / 2.0
            if backed >= actual_fwd * 0.92:
                break
        stop()

    if wall_found:
        print(f"    [PROBE] WALL  at ({cur_r},{cur_c}) {DIR_NAME[d]}")
    else:
        print(f"    [PROBE] open  at ({cur_r},{cur_c}) {DIR_NAME[d]}")

    return wall_found


# ═══════════════════════════════════════════════════════════════
#  CELL SCANNING (probe unknown directions + camera check perimeter)
# ═══════════════════════════════════════════════════════════════

def scan_cell():
    """Probe every direction we don't already have info about.
    For boundary walls, face them and check camera for green."""
    global green_cell, green_dir

    r, c = cur_r, cur_c
    print(f"  [SCAN] cell ({r},{c})")

    for d in range(4):
        nr, nc = r + DR[d], c + DC[d]
        is_perimeter = (nr < 0 or nr >= GRID_SIZE or nc < 0 or nc >= GRID_SIZE)

        # Already-known wall: just check camera for green if perimeter
        if d in walls[r][c]:
            if green_cell is None and is_perimeter:
                face(d)
                settle(2)
                if sees_green():
                    green_cell = (r, c)
                    green_dir  = d
                    print(f"  [SCAN] *** GREEN at ({r},{c}) "
                          f"facing {DIR_NAME[d]} ***")
            continue

        # Skip if the neighbour cell was already visited and confirmed
        # no wall on its side facing us  →  we know it's open
        if not is_perimeter:
            opp = (d + 2) % 4
            if visited[nr][nc] and opp not in walls[nr][nc]:
                continue

        # Perimeter directions are always walls by definition — shouldn't
        # reach here, but safety net:
        if is_perimeter:
            add_wall(r, c, d)
            continue

        # ── Probe this direction ────────────────────────────
        has_wall = probe_direction(d)
        if has_wall:
            add_wall(r, c, d)

    visited[r][c] = True
    wd = ','.join(DIR_NAME[w] for w in sorted(walls[r][c]))
    print(f"  [SCAN] ({r},{c}) walls={{{wd}}}")


# ═══════════════════════════════════════════════════════════════
#  CELL-TO-CELL DRIVE
# ═══════════════════════════════════════════════════════════════

def drive_one_cell(speed=EXPLORE_SPEED):
    """Drive forward one cell (encoder-based).
    Side-wall correction keeps the robot centred.
    Front-wall emergency stop prevents crashing.
    Returns True if crossed, False if a wall blocked us."""
    global cur_r, cur_c

    l0, r0 = get_enc()
    target = ENCODER_PER_CELL
    hit_wall = False

    for _ in range(600):
        if robot.step(TIME_STEP) == -1:
            break
        l1, r1 = get_enc()
        travelled = ((l1 - l0) + (r1 - r0)) / 2.0

        # Front-wall emergency stop (only after 30 % of cell to avoid
        # residual readings from previous position)
        if travelled > target * 0.30:
            f_max = max(ps[0].getValue(), ps[7].getValue())
            if f_max > FRONT_STOP:
                hit_wall = True
                break

        if travelled >= target:
            break

        # Deceleration near end
        frac = max(0.0, min(1.0, travelled / target))
        cur_speed = speed if frac < 0.75 else max(0.10 * MAX_SPEED, speed * 0.30)

        # Side-wall correction
        rp = ps[2].getValue()
        lp = ps[5].getValue()
        if lp > 80 and rp > 80:
            err = lp - rp
        elif lp > 80:
            err = lp - 100
        elif rp > 80:
            err = -(rp - 100)
        else:
            err = 0.0

        g = 0.0008
        leftMotor.setVelocity( max(0.05 * MAX_SPEED,
                                   min(MAX_SPEED, cur_speed - err * g)))
        rightMotor.setVelocity(max(0.05 * MAX_SPEED,
                                   min(MAX_SPEED, cur_speed + err * g)))

    stop()

    if hit_wall:
        # Back up to cell centre
        l_now, r_now = get_enc()
        total_fwd = ((l_now - l0) + (r_now - r0)) / 2.0
        leftMotor.setVelocity(-0.10 * MAX_SPEED)
        rightMotor.setVelocity(-0.10 * MAX_SPEED)
        for _ in range(300):
            if robot.step(TIME_STEP) == -1:
                break
            lc, rc = get_enc()
            backed = ((l_now - lc) + (r_now - rc)) / 2.0
            if backed >= total_fwd * 0.90:
                break
        stop()
        add_wall(cur_r, cur_c, heading)
        print(f"    [ESTOP] wall at ({cur_r},{cur_c}) {DIR_NAME[heading]}")
        return False

    nr = cur_r + DR[heading]
    nc = cur_c + DC[heading]
    cur_r = max(0, min(GRID_SIZE - 1, nr))
    cur_c = max(0, min(GRID_SIZE - 1, nc))
    return True


# ═══════════════════════════════════════════════════════════════
#  BFS FLOOD-FILL
# ═══════════════════════════════════════════════════════════════

def bfs(target_r, target_c):
    dist = [[999] * GRID_SIZE for _ in range(GRID_SIZE)]
    dist[target_r][target_c] = 0
    q = deque([(target_r, target_c)])
    while q:
        r, c = q.popleft()
        for d in range(4):
            if d in walls[r][c]:
                continue
            nr, nc = r + DR[d], c + DC[d]
            if 0 <= nr < GRID_SIZE and 0 <= nc < GRID_SIZE:
                if dist[nr][nc] > dist[r][c] + 1:
                    dist[nr][nc] = dist[r][c] + 1
                    q.append((nr, nc))
    return dist


def trace_path(sr, sc, dist):
    path = []
    r, c = sr, sc
    while dist[r][c] > 0:
        best_d, best_val = -1, dist[r][c]
        for d in range(4):
            if d in walls[r][c]:
                continue
            nr, nc = r + DR[d], c + DC[d]
            if 0 <= nr < GRID_SIZE and 0 <= nc < GRID_SIZE:
                if dist[nr][nc] < best_val:
                    best_val = dist[nr][nc]
                    best_d   = d
        if best_d == -1:
            break
        path.append(best_d)
        r, c = r + DR[best_d], c + DC[best_d]
    return path


# ═══════════════════════════════════════════════════════════════
#  NAVIGATION WITH REPLANNING
# ═══════════════════════════════════════════════════════════════

def navigate_to(tr, tc, speed):
    """BFS shortest path from current cell to (tr,tc).
    Replans after every cell in case new walls were found."""
    for _ in range(200):
        if (cur_r, cur_c) == (tr, tc):
            return True
        dist = bfs(tr, tc)
        if dist[cur_r][cur_c] >= 999:
            return False
        path = trace_path(cur_r, cur_c, dist)
        if not path:
            return (cur_r, cur_c) == (tr, tc)

        d = path[0]
        face(d)

        if not drive_one_cell(speed):
            # Hit unexpected wall — loop will replan
            continue

        # Scan new cell
        scan_cell()

    return (cur_r, cur_c) == (tr, tc)


# ═══════════════════════════════════════════════════════════════
#  ASCII MAP
# ═══════════════════════════════════════════════════════════════

def print_map():
    print()
    for r in range(GRID_SIZE):
        top = ""
        mid = ""
        for c in range(GRID_SIZE):
            top += "+" + ("--" if NORTH in walls[r][c] else "  ")
            mid += ("|" if WEST  in walls[r][c] else " ")
            if green_cell and (r, c) == green_cell:
                mid += "G "
            elif (r, c) == (START_ROW, START_COL):
                mid += "S "
            elif visited[r][c]:
                mid += ". "
            else:
                mid += "# "
        top += "+"
        mid += "|" if EAST in walls[r][GRID_SIZE - 1] else " "
        print(top)
        print(mid)
    print("+--" * GRID_SIZE + "+")
    print()


# ═══════════════════════════════════════════════════════════════
#  PHASE 1 — EXPLORATION
# ═══════════════════════════════════════════════════════════════

def nearest_unvisited():
    """Return (row,col) of closest unvisited reachable cell, or None."""
    dist = bfs(cur_r, cur_c)
    best = None
    for r in range(GRID_SIZE):
        for c in range(GRID_SIZE):
            if not visited[r][c] and dist[r][c] < 999:
                if best is None or dist[r][c] < best[0]:
                    best = (dist[r][c], r, c)
    return (best[1], best[2]) if best else None


def search_boundary_for_green():
    """Fallback: visit every boundary cell, face the edge, check camera."""
    global green_cell, green_dir
    print("  [SEARCH] Green not found — scanning boundary …")

    checks = []
    for c in range(GRID_SIZE):
        checks.append((0,             c, NORTH))
        checks.append((GRID_SIZE - 1, c, SOUTH))
    for r in range(GRID_SIZE):
        checks.append((r, 0,             WEST))
        checks.append((r, GRID_SIZE - 1, EAST))

    for r, c, d in checks:
        if green_cell is not None:
            return
        if not navigate_to(r, c, EXPLORE_SPEED):
            continue
        face(d)
        settle(3)
        if sees_green():
            green_cell = (r, c)
            green_dir  = d
            print(f"  [SEARCH] *** GREEN at ({r},{c}) "
                  f"facing {DIR_NAME[d]} ***")
            return

    print("  [SEARCH] WARNING: green not found!")


def explore():
    global green_cell, green_dir

    print(f"  Start at ({cur_r},{cur_c}) heading {DIR_NAME[heading]}")

    # Full probe-scan at starting cell
    scan_cell()
    print_map()

    iteration = 0
    while iteration < 250:
        iteration += 1
        target = nearest_unvisited()
        if target is None:
            print("  [EXPLORE] All reachable cells visited!")
            break

        tr, tc = target
        print(f"  [EXPLORE #{iteration}] ({cur_r},{cur_c}) → ({tr},{tc})")

        if not navigate_to(tr, tc, EXPLORE_SPEED):
            print(f"  [EXPLORE] Cannot reach ({tr},{tc}), skipping")
            visited[tr][tc] = True
            continue

        if iteration % 6 == 0:
            print_map()

    if green_cell is None:
        search_boundary_for_green()

    print_map()
    total = sum(row.count(True) for row in visited)
    print(f"  Visited {total}/{GRID_SIZE * GRID_SIZE} cells")


# ═══════════════════════════════════════════════════════════════
#  PHASE 2 — RETURN + RED WALL
# ═══════════════════════════════════════════════════════════════

def find_and_touch_red():
    print("  [RED] Scanning for red wall …")
    for d in [SOUTH, WEST, NORTH, EAST]:
        face(d)
        settle(3)
        if sees_red():
            print(f"  [RED] Found facing {DIR_NAME[d]}!")
            leftMotor.setVelocity(RAM_SPEED)
            rightMotor.setVelocity(RAM_SPEED)
            for _ in range(RAM_STEPS):
                robot.step(TIME_STEP)
            stop()
            leftMotor.setVelocity(-0.12 * MAX_SPEED)
            rightMotor.setVelocity(-0.12 * MAX_SPEED)
            for _ in range(15):
                robot.step(TIME_STEP)
            stop()
            return True
    print("  [RED] WARNING: not found!")
    return False


# ═══════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════

for _ in range(WARMUP_STEPS):
    robot.step(TIME_STEP)

le, re = get_enc()
print(f"[INIT] Encoders L={le:.2f} R={re:.2f}")

# ──── Phase 1 ────────────────────────────────────────────────
print("=" * 55)
print("  PHASE 1 — EXPLORATION  (probe-based BFS)")
print("=" * 55)
explore()

# ──── Phase 2 ────────────────────────────────────────────────
print("=" * 55)
print("  PHASE 2 — RETURN TO START + TOUCH RED")
print("=" * 55)

if green_cell is None:
    print("  [!] GREEN NEVER FOUND — wall-follower fallback")
    while robot.step(TIME_STEP) != -1:
        v = [s.getValue() for s in ps]
        fl, fr = v[7], v[0]
        rf, rs_v = v[1], v[2]
        ls_v, lf = v[5], v[6]
        fb  = (fl > 80 or fr > 80)
        rwe = (rs_v > 70 or rf > 70)
        gc = _avg_colour()
        if gc and gc[1] > gc[0] * GREEN_RATIO and gc[1] > gc[2] * GREEN_RATIO:
            if fb:
                leftMotor.setVelocity(RAM_SPEED)
                rightMotor.setVelocity(RAM_SPEED)
                for _ in range(RAM_STEPS):
                    robot.step(TIME_STEP)
                stop()
                break
            else:
                leftMotor.setVelocity(0.5 * MAX_SPEED)
                rightMotor.setVelocity(0.5 * MAX_SPEED)
                continue
        if fb:
            leftMotor.setVelocity(-0.5 * MAX_SPEED)
            rightMotor.setVelocity( 0.5 * MAX_SPEED)
            continue
        if not rwe:
            leftMotor.setVelocity(1.0 * MAX_SPEED)
            rightMotor.setVelocity(0.01 * MAX_SPEED)
            continue
        de = rs_v - 80
        ae = rf - v[3]
        c_ = (0.7 * de + 0.3 * ae) * 0.003
        leftMotor.setVelocity( max(0.05*MAX_SPEED, min(MAX_SPEED, 0.5*MAX_SPEED+c_)))
        rightMotor.setVelocity(max(0.05*MAX_SPEED, min(MAX_SPEED, 0.5*MAX_SPEED-c_)))
else:
    print(f"  Green at ({green_cell[0]},{green_cell[1]}) "
          f"facing {DIR_NAME[green_dir]}")

    navigate_to(START_ROW, START_COL, EXPLORE_SPEED)
    print(f"  [NAV] At start ({cur_r},{cur_c})")

    red_ok = find_and_touch_red()

    # ──── Phase 3 ────────────────────────────────────────────
    print("=" * 55)
    print("  PHASE 3 — SPEED RUN TO GREEN")
    print("=" * 55)

    if not red_ok:
        print("  [!] Red not touched — proceeding anyway")

    face(START_HEADING)

    dist = bfs(green_cell[0], green_cell[1])
    path = trace_path(cur_r, cur_c, dist)
    print(f"  Shortest path: {len(path)} cells")
    print(f"  Route: {[DIR_NAME[d] for d in path]}")

    for d in path:
        face(d)
        drive_one_cell(FAST_SPEED)

    # Ram the green wall
    face(green_dir)
    leftMotor.setVelocity(MAX_SPEED)
    rightMotor.setVelocity(MAX_SPEED)
    for _ in range(30):
        robot.step(TIME_STEP)
    stop()

print("=" * 55)
print("   MAZE COMPLETE!")
print("=" * 55)
