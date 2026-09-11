import API
import sys
from collections import deque

def log(string):
    sys.stderr.write("{}\n".format(string))
    sys.stderr.flush()

# ── Directions ──────────────────────────────────────────
NORTH, EAST, SOUTH, WEST = 0, 1, 2, 3
DX = [0, 1, 0, -1]
DY = [1, 0, -1, 0]
WALL_CHARS = ["n", "e", "s", "w"]

# ── Maze State ──────────────────────────────────────────
width = 0
height = 0
walls = []
dist = []
visited = []
x, y = 0, 0
facing = NORTH

def init_maze():
    global width, height, walls, dist, visited, x, y, facing
    width = API.mazeWidth()
    height = API.mazeHeight()
    walls = [[0] * height for _ in range(width)]
    dist = [[255] * height for _ in range(width)]
    visited = [[False] * height for _ in range(width)]
    x, y, facing = 0, 0, NORTH
    for i in range(width):
        walls[i][0] |= 4
        walls[i][height-1] |= 1
    for j in range(height):
        walls[0][j] |= 8
        walls[width-1][j] |= 2

def get_goals():
    cx, cy = width // 2, height // 2
    goals = []
    for gx in [cx - 1, cx]:
        for gy in [cy - 1, cy]:
            if 0 <= gx < width and 0 <= gy < height:
                goals.append((gx, gy))
    return goals

# ── Wall Helpers ────────────────────────────────────────
def set_wall(wx, wy, d):
    bit = [1, 2, 4, 8][d]
    walls[wx][wy] |= bit
    nx, ny = wx + DX[d], wy + DY[d]
    if 0 <= nx < width and 0 <= ny < height:
        opp = [4, 8, 1, 2][d]
        walls[nx][ny] |= opp

def has_wall(wx, wy, d):
    return bool(walls[wx][wy] & [1, 2, 4, 8][d])

def scan_and_update_walls():
    front = facing
    left = (facing + 3) % 4
    right = (facing + 1) % 4
    if API.wallFront():
        set_wall(x, y, front)
        API.setWall(x, y, WALL_CHARS[front])
    if API.wallLeft():
        set_wall(x, y, left)
        API.setWall(x, y, WALL_CHARS[left])
    if API.wallRight():
        set_wall(x, y, right)
        API.setWall(x, y, WALL_CHARS[right])

# ── Flood Fill ──────────────────────────────────────────
def flood_fill(goals):
    global dist
    dist = [[255] * height for _ in range(width)]
    queue = deque()
    for gx, gy in goals:
        dist[gx][gy] = 0
        queue.append((gx, gy))
    while queue:
        cx, cy = queue.popleft()
        d = dist[cx][cy]
        for direction in range(4):
            if has_wall(cx, cy, direction):
                continue
            nx, ny = cx + DX[direction], cy + DY[direction]
            if 0 <= nx < width and 0 <= ny < height:
                if dist[nx][ny] > d + 1:
                    dist[nx][ny] = d + 1
                    queue.append((nx, ny))

def show_distances(goals, color_visited="C", color_goal="Y"):
    for vx in range(width):
        for vy in range(height):
            d = dist[vx][vy]
            if d < 255:
                API.setText(vx, vy, str(d))
            if (vx, vy) in goals:
                API.setColor(vx, vy, color_goal)
            elif visited[vx][vy]:
                API.setColor(vx, vy, color_visited)

# ── Movement ────────────────────────────────────────────
def turn_to(target_dir):
    global facing
    diff = (target_dir - facing) % 4
    if diff == 1:
        API.turnRight()
    elif diff == 2:
        API.turnRight()
        API.turnRight()
    elif diff == 3:
        API.turnLeft()
    facing = target_dir

def move_one():
    global x, y
    API.moveForward()
    x += DX[facing]
    y += DY[facing]

def best_neighbor(goals_for_ff):
    flood_fill(goals_for_ff)
    best_d = 255
    best_dir = None
    for d in range(4):
        if has_wall(x, y, d):
            continue
        nx, ny = x + DX[d], y + DY[d]
        if 0 <= nx < width and 0 <= ny < height:
            if dist[nx][ny] < best_d:
                best_d = dist[nx][ny]
                best_dir = d
    return best_dir

def explore_neighbor():
    for d in range(4):
        if has_wall(x, y, d):
            continue
        nx, ny = x + DX[d], y + DY[d]
        if 0 <= nx < width and 0 <= ny < height:
            if not visited[nx][ny]:
                return d
    return None

def backtrack_to_unvisited():
    unvis = []
    for ux in range(width):
        for uy in range(height):
            if not visited[ux][uy]:
                unvis.append((ux, uy))
    if not unvis:
        return None
    flood_fill(unvis)
    best_d = 255
    best_dir = None
    for d in range(4):
        if has_wall(x, y, d):
            continue
        nx, ny = x + DX[d], y + DY[d]
        if 0 <= nx < width and 0 <= ny < height:
            if dist[nx][ny] < best_d:
                best_d = dist[nx][ny]
                best_dir = d
    return best_dir

# ── Navigate: move from current position to target ─────
def navigate_to(targets, phase_name, path_color):
    log("{}: navigating to {}".format(phase_name, targets))
    while True:
        scan_and_update_walls()
        visited[x][y] = True

        if (x, y) in targets:
            API.setColor(x, y, "G")
            log("{}: REACHED ({},{})!".format(phase_name, x, y))
            return True

        API.setColor(x, y, path_color)

        # Try toward target
        d = best_neighbor(targets)
        if d is not None:
            show_distances(targets)
            turn_to(d)
            move_one()
            continue

        # Explore unvisited
        d = explore_neighbor()
        if d is not None:
            turn_to(d)
            move_one()
            continue

        # Backtrack
        d = backtrack_to_unvisited()
        if d is not None:
            turn_to(d)
            move_one()
            continue

        log("{}: STUCK at ({},{})".format(phase_name, x, y))
        return False

# ── Speed Run: follow shortest known path (no exploration) ─
def speed_run(targets, phase_name, path_color):
    log("{}: speed run to {}".format(phase_name, targets))
    flood_fill(targets)
    show_distances(targets, color_visited=path_color)

    while True:
        if (x, y) in targets:
            API.setColor(x, y, "G")
            log("{}: REACHED ({},{})!".format(phase_name, x, y))
            return True

        API.setColor(x, y, path_color)

        # Follow shortest path — just pick lowest distance neighbor
        best_d = 255
        best_dir = None
        for d in range(4):
            if has_wall(x, y, d):
                continue
            nx, ny = x + DX[d], y + DY[d]
            if 0 <= nx < width and 0 <= ny < height:
                if dist[nx][ny] < best_d:
                    best_d = dist[nx][ny]
                    best_dir = d

        if best_dir is None:
            log("{}: no path!".format(phase_name))
            return False

        turn_to(best_dir)
        move_one()

# ════════════════════════════════════════════════════════
# ALGORITHM: Flood Fill (3-phase)
#
#   Phase 1: SEARCH RUN — explore maze, reach goal center
#   Phase 2: RETURN RUN — go back to start (maps more walls)
#   Phase 3: SPEED RUN — shortest path from start to goal
#            using fully-mapped maze
# ════════════════════════════════════════════════════════
def run_flood_fill():
    goals = get_goals()
    start = [(0, 0)]

    log("=" * 40)
    log("FLOOD FILL — 3-Phase Run")
    log("Goals: {}".format(goals))
    log("=" * 40)

    API.setColor(0, 0, "G")
    for gx, gy in goals:
        API.setColor(gx, gy, "Y")

    # ── Phase 1: Search Run (explore → goal) ────────
    log("")
    log(">>> PHASE 1: SEARCH RUN (start -> goal)")
    if not navigate_to(goals, "SEARCH", "C"):
        log("Search run failed!")
        return
    log("Phase 1 complete!")

    # ── Phase 2: Return Run (goal → start) ──────────
    log("")
    log(">>> PHASE 2: RETURN RUN (goal -> start)")
    API.clearAllColor()
    for gx, gy in goals:
        API.setColor(gx, gy, "Y")
    if not navigate_to(start, "RETURN", "B"):
        log("Return run failed!")
        return
    log("Phase 2 complete!")

    # ── Phase 3: Speed Run (start → goal, shortest) ─
    log("")
    log(">>> PHASE 3: SPEED RUN (start -> goal, shortest path)")
    API.clearAllColor()
    API.clearAllText()
    for gx, gy in goals:
        API.setColor(gx, gy, "Y")

    # Recompute flood fill with fully-known maze
    flood_fill(goals)
    show_distances(goals, color_visited="c", color_goal="Y")
    optimal_dist = dist[0][0]
    log("Optimal distance from start to goal: {} cells".format(optimal_dist))

    if not speed_run(goals, "SPEED RUN", "G"):
        log("Speed run failed!")
        return

    log("")
    log("=" * 40)
    log("ALL 3 PHASES COMPLETE!")
    log("Search -> Return -> Speed Run DONE")
    log("=" * 40)

# ── Algorithm: Left Wall Follower ──────────────────────
def run_left_wall():
    log("Left Wall Follower")
    API.setColor(0, 0, "G")
    step = 0
    while True:
        step += 1
        API.setText(x, y, str(step))
        API.setColor(x, y, "C")
        if not API.wallLeft():
            API.turnLeft()
        while API.wallFront():
            API.turnRight()
        move_one()

# ── Algorithm: Right Wall Follower ─────────────────────
def run_right_wall():
    log("Right Wall Follower")
    API.setColor(0, 0, "G")
    step = 0
    while True:
        step += 1
        API.setText(x, y, str(step))
        API.setColor(x, y, "M")
        if not API.wallRight():
            API.turnRight()
        while API.wallFront():
            API.turnLeft()
        move_one()

# ── Algorithm: Dead-End Fill + Flood Fill ──────────────
def run_dead_end_fill():
    goals = get_goals()
    start = [(0, 0)]
    log("Dead-End Fill — 3-Phase Run")
    log("Goals: {}".format(goals))

    API.setColor(0, 0, "G")
    for gx, gy in goals:
        API.setColor(gx, gy, "Y")

    dead_end = [[False] * height for _ in range(width)]

    def dead_end_pass():
        changed = True
        while changed:
            changed = False
            for dx in range(width):
                for dy in range(height):
                    if dead_end[dx][dy]:
                        continue
                    if (dx, dy) in goals or (dx == 0 and dy == 0):
                        continue
                    if not visited[dx][dy]:
                        continue
                    open_count = 0
                    for d in range(4):
                        if not has_wall(dx, dy, d):
                            nx, ny = dx + DX[d], dy + DY[d]
                            if 0 <= nx < width and 0 <= ny < height:
                                if not dead_end[nx][ny]:
                                    open_count += 1
                    if open_count <= 1:
                        dead_end[dx][dy] = True
                        API.setColor(dx, dy, "a")
                        changed = True

    # Phase 1: Search with dead-end pruning
    log(">>> PHASE 1: SEARCH with dead-end fill")
    while True:
        scan_and_update_walls()
        visited[x][y] = True

        if (x, y) in goals:
            API.setColor(x, y, "G")
            log("SEARCH: REACHED GOAL!")
            break

        API.setColor(x, y, "C")
        dead_end_pass()
        d = best_neighbor(goals)
        if d is None:
            d = explore_neighbor()
        if d is None:
            d = backtrack_to_unvisited()
        if d is None:
            log("STUCK")
            return
        show_distances(goals)
        turn_to(d)
        move_one()

    # Phase 2: Return to start
    log(">>> PHASE 2: RETURN to start")
    API.clearAllColor()
    for gx, gy in goals:
        API.setColor(gx, gy, "Y")
    if not navigate_to(start, "RETURN", "B"):
        return

    # Phase 3: Speed run
    log(">>> PHASE 3: SPEED RUN")
    API.clearAllColor()
    API.clearAllText()
    dead_end = [[False] * height for _ in range(width)]
    dead_end_pass()
    flood_fill(goals)
    show_distances(goals, color_visited="c")
    log("Optimal distance: {} cells".format(dist[0][0]))
    speed_run(goals, "SPEED", "G")
    log("ALL PHASES COMPLETE!")

# ── Main ──────────────────────────────────────────────
ALGORITHMS = {
    "flood_fill": run_flood_fill,
    "left_wall": run_left_wall,
    "right_wall": run_right_wall,
    "dead_end_fill": run_dead_end_fill,
}

def main():
    algorithm = "flood_fill"
    for i, arg in enumerate(sys.argv):
        if arg == "--alg" and i + 1 < len(sys.argv):
            algorithm = sys.argv[i + 1]
        elif arg.startswith("--alg="):
            algorithm = arg.split("=")[1]

    init_maze()
    log("=== MICROMOUSE MMS ===")
    log("Maze: {}x{}".format(width, height))
    log("Algorithm: {}".format(algorithm))

    runner = ALGORITHMS.get(algorithm)
    if runner is None:
        log("Unknown: {}. Available: {}".format(algorithm, list(ALGORITHMS.keys())))
        return
    runner()

if __name__ == "__main__":
    main()
