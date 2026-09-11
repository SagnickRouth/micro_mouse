\"\"\"
Flood Fill maze solver for MMS simulator.
Implements BFS-based flood fill with visualization.

CELESTA'26 compatible:
  - Goal can be anywhere (default: center, but configurable)
  - 20cm cells
  - Tracks visited cells
\"\"\"

from collections import deque
import API

# Directions: 0=North, 1=East, 2=South, 3=West
NORTH, EAST, SOUTH, WEST = 0, 1, 2, 3
DIR_NAMES = ["N", "E", "S", "W"]
DX = [0, 1, 0, -1]
DY = [1, 0, -1, 0]

# Wall bitmask
WALL_N, WALL_E, WALL_S, WALL_W = 1, 2, 4, 8


class FloodFillSolver:
    def __init__(self, width: int, height: int):
        self.w = width
        self.h = height
        self.x = 0  # Current position
        self.y = 0
        self.facing = NORTH  # Starting direction

        # Maze data
        self.walls = [[0] * height for _ in range(width)]
        self.distance = [[255] * height for _ in range(width)]
        self.visited = [[False] * height for _ in range(width)]

        # Set outer boundary walls
        for i in range(width):
            self.walls[i][0] |= WALL_S
            self.walls[i][height - 1] |= WALL_N
        for j in range(height):
            self.walls[0][j] |= WALL_W
            self.walls[width - 1][j] |= WALL_E

        # Goal: center of maze (default)
        self.goals = []
        cx, cy = width // 2, height // 2
        for gx in range(cx - 1, cx + 1):
            for gy in range(cy - 1, cy + 1):
                if 0 <= gx < width and 0 <= gy < height:
                    self.goals.append((gx, gy))

        API.log(f"Goals: {self.goals}")

        # Initial flood fill
        self._flood_fill()
        self._visualize()

        # Mark start
        API.setColor(0, 0, "g")

    def step(self, wall_l: bool, wall_f: bool, wall_r: bool) -> str:
        \"\"\"One step: update walls, flood fill, decide direction, return action.\"\"\"

        # Update walls from sensor readings
        self._update_walls(wall_l, wall_f, wall_r)
        self.visited[self.x][self.y] = True
        API.setColor(self.x, self.y, "c")

        # Check if at goal
        if (self.x, self.y) in self.goals:
            API.setColor(self.x, self.y, "g")
            return "done"

        # Recompute flood fill
        self._flood_fill()
        self._visualize()

        # Find best neighbor (lowest distance, no wall blocking)
        best_dir = self._best_direction()

        if best_dir is None:
            API.log("No path found!")
            return "done"

        # Convert to action
        return self._direction_to_action(best_dir)

    def _update_walls(self, wall_l: bool, wall_f: bool, wall_r: bool):
        \"\"\"Update wall map from relative sensor readings.\"\"\"
        x, y, f = self.x, self.y, self.facing

        # Map relative to absolute
        front = f
        left = (f + 3) % 4  # Left of facing
        right = (f + 1) % 4  # Right of facing

        wall_bits = [(front, wall_f), (left, wall_l), (right, wall_r)]

        for direction, has_wall in wall_bits:
            if has_wall:
                self._set_wall(x, y, direction)
            # Show walls in simulator
            dir_char = "nesw"[direction]
            if has_wall:
                API.setWall(x, y, dir_char)

    def _set_wall(self, x: int, y: int, direction: int):
        \"\"\"Set a wall and its symmetric neighbor wall.\"\"\"
        bit = [WALL_N, WALL_E, WALL_S, WALL_W][direction]
        self.walls[x][y] |= bit

        # Set neighbor's wall too
        nx, ny = x + DX[direction], y + DY[direction]
        if 0 <= nx < self.w and 0 <= ny < self.h:
            opp_bit = [WALL_S, WALL_W, WALL_N, WALL_E][direction]
            self.walls[nx][ny] |= opp_bit

    def _has_wall(self, x: int, y: int, direction: int) -> bool:
        bit = [WALL_N, WALL_E, WALL_S, WALL_W][direction]
        return bool(self.walls[x][y] & bit)

    def _flood_fill(self):
        \"\"\"BFS flood fill from goals.\"\"\"
        self.distance = [[255] * self.h for _ in range(self.w)]
        queue = deque()

        for gx, gy in self.goals:
            self.distance[gx][gy] = 0
            queue.append((gx, gy))

        while queue:
            cx, cy = queue.popleft()
            d = self.distance[cx][cy]

            for direction in range(4):
                if self._has_wall(cx, cy, direction):
                    continue
                nx, ny = cx + DX[direction], cy + DY[direction]
                if 0 <= nx < self.w and 0 <= ny < self.h:
                    if self.distance[nx][ny] > d + 1:
                        self.distance[nx][ny] = d + 1
                        queue.append((nx, ny))

    def _best_direction(self) -> int | None:
        \"\"\"Find direction with lowest distance, no wall.\"\"\"
        best_dist = 255
        best_dir = None

        for direction in range(4):
            if self._has_wall(self.x, self.y, direction):
                continue
            nx, ny = self.x + DX[direction], self.y + DY[direction]
            if 0 <= nx < self.w and 0 <= ny < self.h:
                if self.distance[nx][ny] < best_dist:
                    best_dist = self.distance[nx][ny]
                    best_dir = direction

        return best_dir

    def _direction_to_action(self, target_dir: int) -> str:
        \"\"\"Convert absolute direction to relative action.\"\"\"
        diff = (target_dir - self.facing) % 4

        if diff == 0:
            # Move forward
            self.x += DX[target_dir]
            self.y += DY[target_dir]
            return "forward"
        elif diff == 1:
            # Turn right then move
            self.facing = target_dir
            self.x += DX[target_dir]
            self.y += DY[target_dir]
            return "right"
        elif diff == 3:
            # Turn left then move
            self.facing = target_dir
            self.x += DX[target_dir]
            self.y += DY[target_dir]
            return "left"
        elif diff == 2:
            # Turn around then move
            self.facing = target_dir
            self.x += DX[target_dir]
            self.y += DY[target_dir]
            return "turn_around"

        return "forward"

    def _visualize(self):
        \"\"\"Show flood fill distances and colors in simulator.\"\"\"
        for x in range(self.w):
            for y in range(self.h):
                d = self.distance[x][y]
                if d < 255:
                    API.setText(x, y, str(d))
                if (x, y) in self.goals:
                    API.setColor(x, y, "y")
                elif self.visited[x][y]:
                    API.setColor(x, y, "c")
