from controller import Robot, DistanceSensor, Motor, Camera
from collections import deque
import math
import json
import os

TIME_STEP = 64
MAX_SPEED = 6.28
THRESHOLD = 85

N, E, S, W = 0, 1, 2, 3
DELTA = {N: (0, 1), E: (1, 0), S: (0, -1), W: (-1, 0)}
OPPOSITE = {N: S, E: W, S: N, W: E}
DIR_NAME = ['N', 'E', 'S', 'W']
GRID = 6
WHEEL_R = 0.02
AXLE = 0.052
CELL_ENC = 0.25 / WHEEL_R
TURN_90_ENC = 2.21  # empirical: 11 steps at 0.5*MAX_SPEED = 2.21 encoder rad for 90°
GS_THRESHOLD = 700  # midpoint: dark tiles top out ~663, light tiles start ~710+
POST_GS_ENC = 6.1  # ~12.5cm to next cell center = 6.25 rad, minus sensor lag
MIN_GS_DIST = 4.0  # ignore GS transitions in the first 4 rad (8cm) to avoid startup noise
MAP_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'maze_map.json')

# --------------- robot setup ---------------
robot = Robot()

ps = []
for i in range(8):
    s = robot.getDevice(f'ps{i}')
    s.enable(TIME_STEP)
    ps.append(s)

gs = []
for name in ['gs0', 'gs1', 'gs2']:
    sensor = robot.getDevice(name)
    sensor.enable(TIME_STEP)
    gs.append(sensor)

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

leftEnc = robot.getDevice('left wheel sensor')
rightEnc = robot.getDevice('right wheel sensor')
leftEnc.enable(TIME_STEP)
rightEnc.enable(TIME_STEP)

camera = robot.getDevice('camera')
camera.enable(TIME_STEP)

for _ in range(5):
    robot.step(TIME_STEP)

print(f"[INIT] gs1={gs[1].getValue():.0f}  ps={[int(s.getValue()) for s in ps]}")

# --------------- maze state ---------------
walls = {}
visited = set()
traversed = set()  # (c, r, d) pairs the robot physically moved through in DFS
pos = [5, 0]
heading = W
goal_cell = None
goal_dir = None

for c in range(GRID):
    walls[(c, 0, S)] = True
    walls[(c, GRID - 1, N)] = True
for r in range(GRID):
    walls[(0, r, W)] = True
    walls[(GRID - 1, r, E)] = True

# --------------- helpers ---------------

def do_step():
    return robot.step(TIME_STEP) != -1

def stop_motors():
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)
    do_step()

def get_enc():
    return (leftEnc.getValue() + rightEnc.getValue()) / 2.0

# --------------- turns (encoder-based) ---------------
TURN_SPEED = 0.1 * MAX_SPEED  # slow turns for precision
ALIGN_SPEED = 0.03 * MAX_SPEED  # very slow for post-turn alignment correction

def realign():
    """
    Correct residual angular drift after a turn or forward move.
    Uses front proximity sensors when a wall is ahead, and side sensors
    when walls are present on the sides but not the front.
    """
    do_step()
    v = [s.getValue() for s in ps]
    if v[0] > 60 or v[7] > 60:
        # Front wall: balance ps0 vs ps7.
        for _ in range(60):
            err = v[0] - v[7]
            if abs(err) < 10:
                break
            sign = 1 if err > 0 else -1
            leftMotor.setVelocity(sign * ALIGN_SPEED)
            rightMotor.setVelocity(-sign * ALIGN_SPEED)
            do_step()
            v = [s.getValue() for s in ps]
        stop_motors()
    elif v[2] > 80 and v[5] > 80:
        # Side walls on both sides: balance ps2 (right) vs ps5 (left)
        for _ in range(60):
            err = v[5] - v[2]
            if abs(err) < 15:
                break
            sign = 1 if err > 0 else -1
            leftMotor.setVelocity(sign * ALIGN_SPEED)
            rightMotor.setVelocity(-sign * ALIGN_SPEED)
            do_step()
            v = [s.getValue() for s in ps]
        stop_motors()

def turn_right(speed_factor=None):
    global heading
    spd = (speed_factor * MAX_SPEED) if speed_factor is not None else TURN_SPEED
    stop_motors()
    sl, sr = leftEnc.getValue(), rightEnc.getValue()
    leftMotor.setVelocity(spd)
    rightMotor.setVelocity(-spd)
    while do_step():
        if (leftEnc.getValue() - sl + sr - rightEnc.getValue()) / 2.0 >= TURN_90_ENC:
            break
    stop_motors()
    realign()
    heading = (heading + 1) % 4

def turn_left(speed_factor=None):
    global heading
    spd = (speed_factor * MAX_SPEED) if speed_factor is not None else TURN_SPEED
    stop_motors()
    sl, sr = leftEnc.getValue(), rightEnc.getValue()
    leftMotor.setVelocity(-spd)
    rightMotor.setVelocity(spd)
    while do_step():
        if (sl - leftEnc.getValue() + rightEnc.getValue() - sr) / 2.0 >= TURN_90_ENC:
            break
    stop_motors()
    realign()
    heading = (heading + 3) % 4

def turn_around(speed_factor=None):
    global heading
    spd = (speed_factor * MAX_SPEED) if speed_factor is not None else TURN_SPEED
    stop_motors()
    sl, sr = leftEnc.getValue(), rightEnc.getValue()
    leftMotor.setVelocity(spd)
    rightMotor.setVelocity(-spd)
    while do_step():
        if (leftEnc.getValue() - sl + sr - rightEnc.getValue()) / 2.0 >= TURN_90_ENC * 2:
            break
    stop_motors()
    realign()
    heading = (heading + 2) % 4

def turn_to(target, speed_factor=None):
    diff = (target - heading) % 4
    if diff == 1:   turn_right(speed_factor)
    elif diff == 3: turn_left(speed_factor)
    elif diff == 2: turn_around(speed_factor)
    else:           realign()  # already facing right direction, correct any forward drift

# --------------- ground-sensor movement ---------------

def move_forward(speed_factor=0.5, record_wall=True):
    """
    Drive forward one cell.  Uses the ground sensor to detect tile-color
    transitions (checkerboard boundary), then continues a calibrated
    encoder distance to the next cell center.

    Returns True on success, False if a wall is hit mid-movement.
    When record_wall=True (exploration), the wall is recorded in the map.
    When record_wall=False (backtracking on known-clear paths), no wall
    is recorded since the hit is from position drift, not a real wall.
    """
    global pos, goal_cell, goal_dir
    realign()  # correct any heading drift before moving
    speed = speed_factor * MAX_SPEED
    start_enc = get_enc()
    hit_threshold = 300 if record_wall else 700
    print(f"  [MOVE] heading={DIR_NAME[heading]} from ({pos[0]},{pos[1]}) enc={start_enc:.1f} rec={record_wall}")
    initial_gs = gs[1].getValue()
    initial_light = initial_gs > GS_THRESHOLD
    gs_crossed = False
    gs_enc = None

    leftMotor.setVelocity(speed)
    rightMotor.setVelocity(speed)

    while True:
        if not do_step():
            return True

        vals = [s.getValue() for s in ps]

        # --- close-range wall hit → abort ---
        # For known-clear backtrack moves, require BOTH sensors to be high so
        # single-sensor drift readings don't abort a valid corridor.
        if record_wall:
            wall_hit = vals[0] > hit_threshold or vals[7] > hit_threshold
        else:
            wall_hit = vals[0] > hit_threshold and vals[7] > hit_threshold
        if wall_hit:
            dist = get_enc() - start_enc
            print(f"  [HIT] ps0={vals[0]:.0f} ps7={vals[7]:.0f} dist={dist:.1f}/{CELL_ENC:.1f} gs={gs[1].getValue():.0f}")
            stop_motors()
            if goal_cell is None and sees_green(verbose=True):
                goal_cell = (pos[0], pos[1])
                goal_dir = heading
                print(f"[MAP] GREEN hit at ({pos[0]},{pos[1]}) dir {DIR_NAME[heading]}")
            if record_wall:
                c, r = pos[0], pos[1]
                walls[(c, r, heading)] = True
                dc, dr = DELTA[heading]
                nc, nr = c + dc, r + dr
                if 0 <= nc < GRID and 0 <= nr < GRID:
                    walls[(nc, nr, OPPOSITE[heading])] = True
            traveled = get_enc() - start_enc
            if traveled > 0.5:
                leftMotor.setVelocity(-0.3 * speed)
                rightMotor.setVelocity(-0.3 * speed)
                rev_start = get_enc()
                while do_step():
                    if rev_start - get_enc() >= traveled:
                        break
                stop_motors()
            return False

        # --- straight-line correction: wall-following when walls present, IMU otherwise ---
        lv, rv = vals[5], vals[2]
        corr = 0.0
        if lv > 80 and rv > 80:
            corr = (lv - rv) * 0.002
        elif lv > 80:
            corr = (lv - 80) * 0.001
        elif rv > 80:
            corr = -(rv - 80) * 0.001
        leftMotor.setVelocity(min(MAX_SPEED, max(0.1, speed - corr)))
        rightMotor.setVelocity(min(MAX_SPEED, max(0.1, speed + corr)))

        # --- green detection during motion ---
        if goal_cell is None and sees_green():
            goal_cell = (pos[0], pos[1])
            goal_dir = heading
            print(f"[MAP] GREEN during move at ({pos[0]},{pos[1]}) heading {DIR_NAME[heading]}")

        # --- ground-sensor tile transition ---
        if not gs_crossed and (get_enc() - start_enc) >= MIN_GS_DIST:
            cur_gs = gs[1].getValue()
            if (cur_gs > GS_THRESHOLD) != initial_light:
                gs_crossed = True
                gs_enc = get_enc()
                print(f"  [GS] at enc={gs_enc-start_enc:.1f} gs={cur_gs:.0f} init={initial_gs:.0f}")

        # --- stop condition ---
        if gs_crossed:
            if get_enc() - gs_enc >= POST_GS_ENC:
                break
        elif get_enc() - start_enc >= CELL_ENC:
            break

    stop_motors()
    dc, dr = DELTA[heading]
    pos[0] += dc
    pos[1] += dr
    return True

# --------------- sensing ---------------

def sees_green(verbose=False):
    img = camera.getImage()
    if img is None:
        return False
    w, h = camera.getWidth(), camera.getHeight()
    rt = gt = bt = 0
    green_pixels = 0
    cnt = w * h
    for x in range(w):
        for y in range(h):
            r = Camera.imageGetRed(img, w, x, y)
            g = Camera.imageGetGreen(img, w, x, y)
            b = Camera.imageGetBlue(img, w, x, y)
            rt += r; gt += g; bt += b
            # Count pixels where green clearly dominates
            if g > 60 and g > r * 1.4 and g > b * 1.2:
                green_pixels += 1
    ra, ga, ba = rt / cnt, gt / cnt, bt / cnt
    green_frac = green_pixels / cnt
    is_green = green_frac > 0.05  # at least 5% of pixels are clearly green
    if verbose or is_green:
        print(f"  [CAM] rgb=({ra:.0f},{ga:.0f},{ba:.0f}) green_frac={green_frac:.2f} green={is_green}")
    return is_green

def scan_walls_center():
    v = [s.getValue() for s in ps]
    print(f"  [PS] {[int(x) for x in v]}  gs1={gs[1].getValue():.0f}")
    return (
        v[0] > THRESHOLD or v[7] > THRESHOLD,
        v[1] > THRESHOLD or v[2] > THRESHOLD,
        v[3] > THRESHOLD or v[4] > THRESHOLD,
        v[5] > THRESHOLD or v[6] > THRESHOLD,
    )

def scan_cell(came_from=None):
    """
    Read proximity sensors at cell center for a preliminary wall map.
    Then turn to face each detected wall and check camera for green.
    """
    global goal_cell, goal_dir
    c, r = pos[0], pos[1]

    front, right, back, left = scan_walls_center()
    rel = [front, right, back, left]
    cell_walls = {}
    for i, has in enumerate(rel):
        d = (heading + i) % 4
        cell_walls[d] = has

    if came_from is not None:
        cell_walls[came_from] = False

    for d, has in cell_walls.items():
        if has:
            walls[(c, r, d)] = True
            dc, dr = DELTA[d]
            nc, nr = c + dc, r + dr
            if 0 <= nc < GRID and 0 <= nr < GRID:
                walls[(nc, nr, OPPOSITE[d])] = True
        elif not walls.get((c, r, d), False):
            walls[(c, r, d)] = False

    if goal_cell is None:
        # Only check boundary walls — green is always on the outer maze wall.
        # Interior-wall checks cause unnecessary turns ("360" effect).
        # Green on interior walls is caught by continuous motion detection.
        boundary_dirs = []
        if r == GRID - 1: boundary_dirs.append(N)
        if c == GRID - 1: boundary_dirs.append(E)
        if r == 0:        boundary_dirs.append(S)
        if c == 0:        boundary_dirs.append(W)
        for d in boundary_dirs:
            if cell_walls.get(d, False):
                turn_to(d)
                do_step()
                if sees_green(verbose=True):
                    goal_cell = (c, r)
                    goal_dir = d
                    print(f"[MAP] GREEN at ({c},{r}) dir {DIR_NAME[d]}")
                    break

# --------------- pathfinding ---------------

def has_wall(c, r, d):
    return walls.get((c, r, d), False)

def get_unvisited_neighbors(c, r):
    result = []
    order = [heading, (heading+1)%4, (heading+3)%4, (heading+2)%4]
    for d in order:
        if not has_wall(c, r, d):
            dc, dr = DELTA[d]
            nc, nr = c + dc, r + dr
            if 0 <= nc < GRID and 0 <= nr < GRID and (nc, nr) not in visited:
                result.append((d, nc, nr))
    return result

def bfs(start, end):
    queue = deque([start])
    came_from = {start: None}
    while queue:
        cur = queue.popleft()
        if cur == end:
            break
        c, r = cur
        for d in [N, E, S, W]:
            if not has_wall(c, r, d):
                dc, dr = DELTA[d]
                nb = (c + dc, r + dr)
                if 0 <= nb[0] < GRID and 0 <= nb[1] < GRID and nb not in came_from:
                    came_from[nb] = cur
                    queue.append(nb)
    if end not in came_from:
        return []
    path = []
    node = end
    while node is not None:
        path.append(node)
        node = came_from[node]
    path.reverse()
    return path

def direction_between(a, b):
    for d, (dx, dy) in DELTA.items():
        if dx == b[0] - a[0] and dy == b[1] - a[1]:
            return d
    return None

def follow_path(path, speed_factor=0.5, turn_speed_factor=None, record_wall=True):
    for i in range(1, len(path)):
        d = direction_between(path[i - 1], path[i])
        if d is None:
            print(f"  [PATH] bad step {path[i-1]}->{path[i]}, aborting")
            return False
        turn_to(d, turn_speed_factor)
        if not move_forward(speed_factor, record_wall=record_wall):
            print(f"  [PATH] move failed at step {i}/{len(path)-1}")
            return False
    return True


def find_nearest_unvisited():
    start = (pos[0], pos[1])
    queue = deque([start])
    seen = {start}
    while queue:
        c, r = queue.popleft()
        if (c, r) not in visited:
            return (c, r)
        for d in [N, E, S, W]:
            if not has_wall(c, r, d):
                dc, dr = DELTA[d]
                nb = (c + dc, r + dr)
                if 0 <= nb[0] < GRID and 0 <= nb[1] < GRID and nb not in seen:
                    seen.add(nb)
                    queue.append(nb)
    return None

# --------------- DFS exploration ---------------

def fmt_walls(c, r):
    return ' '.join(f"{DIR_NAME[d]}={'W' if has_wall(c,r,d) else '.'}"
                    for d in [N, E, S, W])

def dfs_explore():
    stack = []

    c, r = pos[0], pos[1]
    visited.add((c, r))
    scan_cell(came_from=None)
    print(f"[DFS] Start ({c},{r})  {fmt_walls(c,r)}")

    while True:
        if goal_cell is not None:
            print(f"[DFS] Green found at {goal_cell}, stopping exploration early")
            break

        c, r = pos[0], pos[1]
        neighbors = get_unvisited_neighbors(c, r)

        moved = False
        while neighbors:
            d, nc, nr = neighbors.pop(0)
            turn_to(d)
            if move_forward():
                traversed.add((c, r, d))
                traversed.add((nc, nr, OPPOSITE[d]))
                stack.append((c, r))
                visited.add((nc, nr))
                scan_cell(came_from=OPPOSITE[d])
                print(f"[DFS] Visit ({nc},{nr})  {fmt_walls(nc,nr)}  stack={len(stack)}")
                moved = True
                break
            else:
                print(f"[DFS] Blocked ({c},{r}) dir {DIR_NAME[d]}")
                neighbors = get_unvisited_neighbors(c, r)

        if not moved:
            reached = False
            while stack and not reached:
                prev = stack.pop()
                cur = (pos[0], pos[1])

                if prev == cur:
                    reached = True
                    break

                d = direction_between(cur, prev)
                if d is not None:
                    turn_to(d)
                    if move_forward(record_wall=False):
                        traversed.add((cur[0], cur[1], d))
                        traversed.add((prev[0], prev[1], OPPOSITE[d]))
                        reached = True
                        break
                    print(f"[DFS] Direct back to {prev} failed, trying BFS")

                path = bfs(cur, prev)
                if path and len(path) > 1:
                    if follow_path(path, record_wall=False):
                        reached = True
                        break

                print(f"[DFS] Cannot reach {prev}, skipping  stack={len(stack)}")

            if not reached:
                target = find_nearest_unvisited()
                if target:
                    path = bfs((pos[0], pos[1]), target)
                    if path and follow_path(path, record_wall=False):
                        visited.add((pos[0], pos[1]))
                        scan_cell()
                        reached = True
                if not reached:
                    print("[DFS] Stack exhausted, exploration done")
                    break
            print(f"[DFS] Back  {prev}  stack={len(stack)}")

    return list(stack) + [(pos[0], pos[1])]

# --------------- map persistence ---------------

def save_map():
    data = {
        'walls': {f"{c},{r},{d}": v for (c, r, d), v in walls.items()},
        'traversed': [[c, r, d] for (c, r, d) in traversed],
        'goal_cell': list(goal_cell) if goal_cell else None,
        'goal_dir': goal_dir,
    }
    with open(MAP_FILE, 'w') as f:
        json.dump(data, f, indent=2)
    print(f"[MAP] Saved to {MAP_FILE}  goal={goal_cell} dir={DIR_NAME[goal_dir] if goal_dir is not None else '?'}  traversed={len(traversed)}")

def load_map():
    global walls, traversed, goal_cell, goal_dir
    with open(MAP_FILE) as f:
        data = json.load(f)
    walls.clear()
    for key, val in data['walls'].items():
        c, r, d = map(int, key.split(','))
        walls[(c, r, d)] = val
    traversed.clear()
    for item in data.get('traversed', []):
        traversed.add(tuple(item))
    goal_cell = tuple(data['goal_cell']) if data['goal_cell'] else None
    goal_dir = data['goal_dir']
    print(f"[MAP] Loaded from {MAP_FILE}  goal={goal_cell} dir={DIR_NAME[goal_dir] if goal_dir is not None else '?'}  traversed={len(traversed)}")

def bfs_confirmed(start, end):
    """BFS routing only through corridors physically confirmed during DFS (in traversed set)."""
    queue = deque([start])
    came_from = {start: None}
    while queue:
        cur = queue.popleft()
        if cur == end:
            break
        c, r = cur
        for d in [N, E, S, W]:
            if (c, r, d) in traversed:
                dc, dr = DELTA[d]
                nb = (c + dc, r + dr)
                if 0 <= nb[0] < GRID and 0 <= nb[1] < GRID and nb not in came_from:
                    came_from[nb] = cur
                    queue.append(nb)
    if end not in came_from:
        return []
    path = []
    node = end
    while node is not None:
        path.append(node)
        node = came_from[node]
    path.reverse()
    return path

# =============== main ===============

speed_run_mode = os.path.exists(MAP_FILE)

if speed_run_mode:
    # ── Second run: load saved map, skip straight to Phase 3 ──
    print("=== Speed Run Mode (loading saved map) ===")
    load_map()
    if goal_cell:
        print(f"Goal: {goal_cell} dir {DIR_NAME[goal_dir]}")
    else:
        print("WARNING: No goal in saved map — cannot speed run")

else:
    # ── First run: explore the maze, save the map, then stop ──
    print("=== Phase 1: DFS Exploration ===")
    dfs_explore()
    print(f"Explored {len(visited)}/{GRID*GRID} cells")

    if goal_cell is None:
        # Green not found yet — keep exploring until we find it
        while True:
            target = find_nearest_unvisited()
            if target is None:
                break
            print(f"[COVERAGE] {len(visited)}/{GRID*GRID} visited, recovering to {target}")
            path = bfs((pos[0], pos[1]), target)
            if not path:
                break
            if not follow_path(path, speed_factor=0.5, record_wall=False):
                print("[COVERAGE] Could not reach target — robot stuck, stopping coverage")
                break
            visited.add((pos[0], pos[1]))
            scan_cell()
            dfs_explore()

    if goal_cell is None:
        print("WARNING: green wall not found!")
    else:
        print(f"Goal: {goal_cell} dir {DIR_NAME[goal_dir]}")

    save_map()
    print("=== Map saved! Reset the world (Ctrl+Shift+T) and run again for speed run ===")

# ── Phase 3: Speed Run (only executes on second run) ──
if speed_run_mode and goal_cell:
    print("=== Phase 3: Speed Run ===")
    reached_goal = False
    ph3_speeds = [1.0, 0.7, 0.5, 0.3, 0.3]
    for attempt in range(5):
        # Use confirmed-corridor BFS (only corridors physically walked during DFS).
        # Falls back to wall-map BFS if traversed graph has no path (shouldn't happen).
        path_goal = bfs_confirmed((pos[0], pos[1]), goal_cell)
        if not path_goal:
            print("[Phase3] No confirmed path to goal, falling back to wall-map BFS")
            path_goal = bfs((pos[0], pos[1]), goal_cell)
        if not path_goal:
            print("No path to goal!")
            break
        spd = ph3_speeds[attempt]
        print(f"[Phase3] Attempt {attempt+1} speed={spd}: {len(path_goal)} cells  {path_goal}")
        if follow_path(path_goal, speed_factor=spd, turn_speed_factor=0.3, record_wall=False):
            turn_to(goal_dir)
            leftMotor.setVelocity(MAX_SPEED)
            rightMotor.setVelocity(MAX_SPEED)
            while do_step():
                v = [s.getValue() for s in ps]
                if v[0] > 500 or v[7] > 500:
                    stop_motors()
                    if sees_green(verbose=True):
                        print("REACHED GREEN WALL!")
                    else:
                        print("[WARN] Hit wall but camera sees no green — navigation error")
                    break
            reached_goal = True
            break
        # Failure is drift on a confirmed corridor — don't record a false wall.
        # Just retry at lower speed from wherever the robot stopped.
        c, r = pos[0], pos[1]
        print(f"[Phase3] Drift at ({c},{r}) {DIR_NAME[heading]}, retrying at lower speed")
    if not reached_goal:
        print("[WARN] Phase 3 failed after all attempts")
elif speed_run_mode:
    print("No goal in map — cannot speed run")
