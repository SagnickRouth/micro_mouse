import API
import sys
from collections import deque
import heapq

def log(string):
    sys.stderr.write("{}\n".format(string))
    sys.stderr.flush()

NORTH, EAST, SOUTH, WEST = 0, 1, 2, 3
DX = [0, 1, 0, -1]
DY = [1, 0, -1, 0]
WALL_CHARS = ["n", "e", "s", "w"]

# Turn cost penalties (tunable)
# Higher = more penalty for turning = prefers straighter paths
TURN_COST_90 = 3       # cost of a 90 degree turn
TURN_COST_180 = 6      # cost of a 180 degree turn
MOVE_COST = 2           # cost of moving one cell forward

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

def set_wall(wx, wy, d):
    bit = [1, 2, 4, 8][d]
    walls[wx][wy] |= bit
    nx, ny = wx + DX[d], wy + DY[d]
    if 0 <= nx < width and 0 <= ny < height:
        opp = [4, 8, 1, 2][d]
        walls[nx][ny] |= opp

def has_wall(wx, wy, d):
    return bool(walls[wx][wy] & [1, 2, 4, 8][d])

def scan_walls():
    new_walls = False
    front = facing
    left = (facing + 3) % 4
    right = (facing + 1) % 4
    if API.wallFront():
        if not has_wall(x, y, front):
            new_walls = True
        set_wall(x, y, front)
        API.setWall(x, y, WALL_CHARS[front])
    if API.wallLeft():
        if not has_wall(x, y, left):
            new_walls = True
        set_wall(x, y, left)
        API.setWall(x, y, WALL_CHARS[left])
    if API.wallRight():
        if not has_wall(x, y, right):
            new_walls = True
        set_wall(x, y, right)
        API.setWall(x, y, WALL_CHARS[right])
    return new_walls

# ── Standard BFS Flood Fill (for search/return phases) ──
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

# ── Turn-Penalized Dijkstra (for speed run) ─────────────
# State: (x, y, facing_direction)
# This finds the path that minimizes total time considering
# that turns are expensive (robot must stop, rotate, restart)

def dijkstra_turn_penalty(goals, start_x, start_y, start_facing):
    """Returns a direction-aware distance map and predecessor map.
    dist_map[x][y][d] = minimum cost to reach (x,y) facing direction d.
    prev_map[x][y][d] = (px, py, pd) = where we came from."""

    INF = 999999
    dist_map = [[[INF]*4 for _ in range(height)] for _ in range(width)]
    prev_map = [[[None]*4 for _ in range(height)] for _ in range(width)]

    # Start: we're at (start_x, start_y) facing start_facing, cost 0
    dist_map[start_x][start_y][start_facing] = 0

    # Priority queue: (cost, x, y, facing)
    pq = [(0, start_x, start_y, start_facing)]

    while pq:
        cost, cx, cy, cf = heapq.heappop(pq)

        if cost > dist_map[cx][cy][cf]:
            continue

        # Check if we reached a goal
        if (cx, cy) in goals:
            continue  # still explore to find optimal for all goals

        # Try moving in each direction
        for new_dir in range(4):
            if has_wall(cx, cy, new_dir):
                continue

            nx, ny = cx + DX[new_dir], cy + DY[new_dir]
            if not (0 <= nx < width and 0 <= ny < height):
                continue

            # Calculate turn cost
            turn_diff = (new_dir - cf) % 4
            if turn_diff == 0:
                turn_cost = 0               # straight ahead
            elif turn_diff == 1 or turn_diff == 3:
                turn_cost = TURN_COST_90    # 90 degree turn
            else:
                turn_cost = TURN_COST_180   # 180 degree turn

            new_cost = cost + MOVE_COST + turn_cost

            if new_cost < dist_map[nx][ny][new_dir]:
                dist_map[nx][ny][new_dir] = new_cost
                prev_map[nx][ny][new_dir] = (cx, cy, cf)
                heapq.heappush(pq, (new_cost, nx, ny, new_dir))

    return dist_map, prev_map

def reconstruct_path(prev_map, goals, start_x, start_y):
    """Find the goal cell+direction with lowest cost, trace back path."""
    # Find best goal entry
    INF = 999999
    best_cost = INF
    best_gx, best_gy, best_gd = -1, -1, -1

    # We need dist_map too, let's get it from the caller
    # Actually, just trace all goals
    return None  # handled inline below

def show_distances_simple(goals, color_visited="C", color_goal="Y"):
    for vx in range(width):
        for vy in range(height):
            d = dist[vx][vy]
            if d < 255:
                API.setText(vx, vy, str(d))
            if (vx, vy) in goals:
                API.setColor(vx, vy, color_goal)
            elif visited[vx][vy]:
                API.setColor(vx, vy, color_visited)

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

def best_open_neighbor(goals_for_ff):
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

def navigate_to(targets, phase_name, path_color):
    log("{}: navigating to {}".format(phase_name, targets))
    while True:
        scan_walls()
        visited[x][y] = True
        if (x, y) in targets:
            API.setColor(x, y, "G")
            log("{}: REACHED ({},{})!".format(phase_name, x, y))
            return True
        API.setColor(x, y, path_color)
        d = best_open_neighbor(targets)
        if d is not None:
            show_distances_simple(targets)
            turn_to(d)
            move_one()
            continue
        d = explore_neighbor()
        if d is not None:
            turn_to(d)
            move_one()
            continue
        d = backtrack_to_unvisited()
        if d is not None:
            turn_to(d)
            move_one()
            continue
        log("{}: STUCK".format(phase_name))
        return False

# ── Speed Run with Turn-Penalized Pathfinding ──────────
def speed_run_optimized(targets, phase_name):
    """Speed run using Dijkstra with turn penalties.
    Finds path that minimizes actual traversal time."""
    log("{}: computing turn-optimized path...".format(phase_name))

    dist_map, prev_map = dijkstra_turn_penalty(targets, x, y, facing)

    # Find best goal entry (lowest cost across all facing directions)
    INF = 999999
    best_cost = INF
    best_gx, best_gy, best_gd = -1, -1, -1
    for gx, gy in targets:
        for gd in range(4):
            if dist_map[gx][gy][gd] < best_cost:
                best_cost = dist_map[gx][gy][gd]
                best_gx, best_gy, best_gd = gx, gy, gd

    if best_cost >= INF:
        log("{}: no path to goal!".format(phase_name))
        return False

    log("{}: optimal cost = {} (with turn penalties)".format(phase_name, best_cost))

    # Also compute simple BFS distance for comparison
    flood_fill(targets)
    simple_dist = dist[x][y]
    log("{}: simple BFS distance = {} cells".format(phase_name, simple_dist))

    # Reconstruct path
    path = []
    cx, cy, cd = best_gx, best_gy, best_gd
    while (cx, cy) != (x, y) or cd != facing:
        path.append((cx, cy, cd))
        prev = prev_map[cx][cy][cd]
        if prev is None:
            break
        cx, cy, cd = prev
    path.reverse()

    # Count turns in path
    turns = 0
    prev_dir = facing
    for px, py, pd in path:
        if pd != prev_dir:
            turns += 1
        prev_dir = pd

    log("{}: path length = {} cells, {} turns".format(phase_name, len(path), turns))

    # Visualize the planned path
    for px, py, pd in path:
        API.setColor(px, py, "o")  # orange = planned path
    for gx, gy in targets:
        API.setColor(gx, gy, "Y")

    # Execute the path
    for px, py, pd in path:
        # Scan walls first (safety)
        new_walls = scan_walls()
        if new_walls:
            log("{}: new wall found at ({},{}), replanning...".format(phase_name, x, y))
            # Replan from current position
            return speed_run_optimized(targets, phase_name)

        API.setColor(x, y, "G")
        turn_to(pd)
        move_one()

    # Check if we reached goal
    scan_walls()
    if (x, y) in targets:
        API.setColor(x, y, "G")
        log("{}: REACHED ({},{})!".format(phase_name, x, y))
        return True

    log("{}: path ended but not at goal".format(phase_name))
    return False

# ════════════════════════════════════════════════════════
# FLOOD FILL — 3-Phase with Turn-Optimized Speed Run
# ════════════════════════════════════════════════════════
def run_flood_fill():
    goals = get_goals()
    start = [(0, 0)]

    log("=" * 50)
    log("FLOOD FILL - 3-Phase (Turn-Optimized Speed Run)")
    log("Goals: {}".format(goals))
    log("Turn cost 90: {}, 180: {}, Move: {}".format(
        TURN_COST_90, TURN_COST_180, MOVE_COST))
    log("=" * 50)

    API.setColor(0, 0, "G")
    for gx, gy in goals:
        API.setColor(gx, gy, "Y")

    # Phase 1: Search
    log("")
    log(">>> PHASE 1: SEARCH RUN (explore -> goal)")
    if not navigate_to(goals, "SEARCH", "C"):
        return
    log("Phase 1 complete!")

    # Phase 2: Return
    log("")
    log(">>> PHASE 2: RETURN RUN (goal -> start)")
    API.clearAllColor()
    for gx, gy in goals:
        API.setColor(gx, gy, "Y")
    if not navigate_to(start, "RETURN", "B"):
        return
    log("Phase 2 complete!")

    # Phase 3: Speed run with turn-penalized Dijkstra
    log("")
    log(">>> PHASE 3: SPEED RUN (turn-optimized shortest path)")
    API.clearAllColor()
    API.clearAllText()
    for gx, gy in goals:
        API.setColor(gx, gy, "Y")

    if not speed_run_optimized(goals, "SPEED"):
        return

    log("")
    log("=" * 50)
    log("ALL 3 PHASES COMPLETE!")
    log("=" * 50)

# ── Wall Followers (unchanged) ─────────────────────────
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

# ── Dead-End Fill — 3-Phase (also turn-optimized) ──────
def run_dead_end_fill():
    goals = get_goals()
    start = [(0, 0)]
    log("Dead-End Fill - 3-Phase (Turn-Optimized)")

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
                    if dead_end[dx][dy]: continue
                    if (dx, dy) in goals or (dx == 0 and dy == 0): continue
                    if not visited[dx][dy]: continue
                    oc = 0
                    for d in range(4):
                        if not has_wall(dx, dy, d):
                            nx, ny = dx + DX[d], dy + DY[d]
                            if 0 <= nx < width and 0 <= ny < height:
                                if not dead_end[nx][ny]: oc += 1
                    if oc <= 1:
                        dead_end[dx][dy] = True
                        API.setColor(dx, dy, "a")
                        changed = True

    # Phase 1
    log(">>> PHASE 1: SEARCH with dead-end fill")
    while True:
        scan_walls()
        visited[x][y] = True
        if (x, y) in goals:
            API.setColor(x, y, "G")
            log("GOAL REACHED!")
            break
        API.setColor(x, y, "C")
        dead_end_pass()
        d = best_open_neighbor(goals)
        if d is None: d = explore_neighbor()
        if d is None: d = backtrack_to_unvisited()
        if d is None:
            log("STUCK"); return
        show_distances_simple(goals)
        turn_to(d)
        move_one()

    # Phase 2
    log(">>> PHASE 2: RETURN")
    API.clearAllColor()
    if not navigate_to(start, "RETURN", "B"): return

    # Phase 3: Turn-optimized speed run
    log(">>> PHASE 3: SPEED RUN (turn-optimized)")
    API.clearAllColor()
    API.clearAllText()
    for gx, gy in goals:
        API.setColor(gx, gy, "Y")
    speed_run_optimized(goals, "SPEED")
    log("COMPLETE!")

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
