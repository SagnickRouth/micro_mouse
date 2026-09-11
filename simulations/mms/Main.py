import API
import sys
from collections import deque

def log(string):
    sys.stderr.write("{}\n".format(string))
    sys.stderr.flush()

# ── Directions ──────────────────────────────────────────
NORTH, EAST, SOUTH, WEST = 0, 1, 2, 3
DIRECTION_NAMES = ["N", "E", "S", "W"]
DX = [0, 1, 0, -1]
DY = [1, 0, -1, 0]
WALL_CHARS = ["n", "e", "s", "w"]

# ── Maze State ──────────────────────────────────────────
width = 0
height = 0
walls = []       # walls[x][y] = bitmask (1=N, 2=E, 4=S, 8=W)
dist = []        # flood fill distances
visited = []
x, y = 0, 0      # current position
facing = NORTH    # current direction

def init_maze():
    global width, height, walls, dist, visited, x, y, facing
    width = API.mazeWidth()
    height = API.mazeHeight()
    walls = [[0] * height for _ in range(width)]
    dist = [[255] * height for _ in range(width)]
    visited = [[False] * height for _ in range(width)]
    x, y, facing = 0, 0, NORTH

    # Set outer boundary walls
    for i in range(width):
        walls[i][0] |= 4        # south wall
        walls[i][height-1] |= 1  # north wall
    for j in range(height):
        walls[0][j] |= 8        # west wall
        walls[width-1][j] |= 2  # east wall

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

def update_walls():
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

def visualize(goals):
    for vx in range(width):
        for vy in range(height):
            d = dist[vx][vy]
            if d < 255:
                API.setText(vx, vy, str(d))
            if (vx, vy) in goals:
                API.setColor(vx, vy, "Y")
            elif visited[vx][vy]:
                API.setColor(vx, vy, "C")

# ── Movement ───────────────────────────────────────────
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

def move_forward():
    global x, y
    API.moveForward()
    x += DX[facing]
    y += DY[facing]

# ── Algorithm: Flood Fill ──────────────────────────────
def run_flood_fill():
    goals = get_goals()
    log("Flood Fill - Goals: {}".format(goals))
    API.setColor(0, 0, "G")
    for gx, gy in goals:
        API.setColor(gx, gy, "Y")

    while True:
        update_walls()
        visited[x][y] = True
        API.setColor(x, y, "C")

        if (x, y) in goals:
            API.setColor(x, y, "G")
            log("GOAL REACHED at ({},{})!".format(x, y))
            return

        flood_fill(goals)
        visualize(goals)

        # Find best neighbor
        best_dist = 255
        best_dir = None
        for d in range(4):
            if has_wall(x, y, d):
                continue
            nx, ny = x + DX[d], y + DY[d]
            if 0 <= nx < width and 0 <= ny < height:
                if dist[nx][ny] < best_dist:
                    best_dist = dist[nx][ny]
                    best_dir = d

        if best_dir is None:
            # No path to goal — flood fill to nearest unvisited
            unvisited_goals = []
            for ux in range(width):
                for uy in range(height):
                    if not visited[ux][uy]:
                        unvisited_goals.append((ux, uy))
            if not unvisited_goals:
                log("All cells visited, goal not reachable")
                return
            flood_fill(unvisited_goals)
            best_dist = 255
            for d in range(4):
                if has_wall(x, y, d):
                    continue
                nx, ny = x + DX[d], y + DY[d]
                if 0 <= nx < width and 0 <= ny < height:
                    if dist[nx][ny] < best_dist:
                        best_dist = dist[nx][ny]
                        best_dir = d
            if best_dir is None:
                log("Completely stuck")
                return

        turn_to(best_dir)
        move_forward()

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
        move_forward()

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
        move_forward()

# ── Algorithm: Dead-End Fill + Flood Fill ──────────────
def run_dead_end_fill():
    goals = get_goals()
    log("Dead-End Fill - Goals: {}".format(goals))
    API.setColor(0, 0, "G")
    for gx, gy in goals:
        API.setColor(gx, gy, "Y")

    dead_end = [[False] * height for _ in range(width)]

    while True:
        update_walls()
        visited[x][y] = True
        API.setColor(x, y, "C")

        if (x, y) in goals:
            API.setColor(x, y, "G")
            log("GOAL REACHED at ({},{})!".format(x, y))
            return

        # Dead-end fill pass
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

        # Flood fill avoiding dead ends
        global dist
        dist = [[255] * height for _ in range(width)]
        queue = deque()
        for gx, gy in goals:
            dist[gx][gy] = 0
            queue.append((gx, gy))
        while queue:
            cx, cy = queue.popleft()
            dd = dist[cx][cy]
            for d in range(4):
                if has_wall(cx, cy, d):
                    continue
                nx, ny = cx + DX[d], cy + DY[d]
                if 0 <= nx < width and 0 <= ny < height:
                    if not dead_end[nx][ny] and dist[nx][ny] > dd + 1:
                        dist[nx][ny] = dd + 1
                        queue.append((nx, ny))

        visualize(goals)

        best_dist = 255
        best_dir = None
        for d in range(4):
            if has_wall(x, y, d):
                continue
            nx, ny = x + DX[d], y + DY[d]
            if 0 <= nx < width and 0 <= ny < height:
                if dist[nx][ny] < best_dist:
                    best_dist = dist[nx][ny]
                    best_dir = d

        if best_dir is None:
            for d in range(4):
                if has_wall(x, y, d):
                    continue
                nx, ny = x + DX[d], y + DY[d]
                if 0 <= nx < width and 0 <= ny < height:
                    if not visited[nx][ny]:
                        best_dir = d
                        break
            if best_dir is None:
                for d in range(4):
                    if not has_wall(x, y, d):
                        best_dir = d
                        break
            if best_dir is None:
                log("Stuck")
                return

        turn_to(best_dir)
        move_forward()

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
        log("Unknown algorithm: {}".format(algorithm))
        log("Available: {}".format(list(ALGORITHMS.keys())))
        return

    runner()
    log("=== RUN COMPLETE ===")

if __name__ == "__main__":
    main()
