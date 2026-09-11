"""
Flood Fill maze solver for MMS simulator.
BFS-based with live visualization.
"""

from collections import deque
import API

NORTH, EAST, SOUTH, WEST = 0, 1, 2, 3
DX = [0, 1, 0, -1]
DY = [1, 0, -1, 0]
WALL_N, WALL_E, WALL_S, WALL_W = 1, 2, 4, 8


class FloodFillSolver:
    def __init__(self, width, height):
        self.w = width
        self.h = height
        self.x = 0
        self.y = 0
        self.facing = NORTH

        self.walls = [[0] * height for _ in range(width)]
        self.distance = [[255] * height for _ in range(width)]
        self.visited = [[False] * height for _ in range(width)]

        # Outer boundary walls
        for i in range(width):
            self.walls[i][0] |= WALL_S
            self.walls[i][height - 1] |= WALL_N
        for j in range(height):
            self.walls[0][j] |= WALL_W
            self.walls[width - 1][j] |= WALL_E

        # Goal: center of maze
        self.goals = []
        cx, cy = width // 2, height // 2
        for gx in range(cx - 1, cx + 1):
            for gy in range(cy - 1, cy + 1):
                if 0 <= gx < width and 0 <= gy < height:
                    self.goals.append((gx, gy))

        API.log("Goals: " + str(self.goals))
        self._flood_fill()
        self._visualize()
        API.setColor(0, 0, "g")

    def step(self, wall_l, wall_f, wall_r):
        self._update_walls(wall_l, wall_f, wall_r)
        self.visited[self.x][self.y] = True
        API.setColor(self.x, self.y, "c")

        if (self.x, self.y) in self.goals:
            API.setColor(self.x, self.y, "g")
            return "done"

        self._flood_fill()
        self._visualize()

        best_dir = self._best_direction()
        if best_dir is None:
            API.log("No path found!")
            return "done"

        return self._direction_to_action(best_dir)

    def _update_walls(self, wall_l, wall_f, wall_r):
        x, y, f = self.x, self.y, self.facing
        front = f
        left = (f + 3) % 4
        right = (f + 1) % 4

        for direction, has_wall in [(front, wall_f), (left, wall_l), (right, wall_r)]:
            if has_wall:
                self._set_wall(x, y, direction)
            dir_char = "nesw"[direction]
            if has_wall:
                API.setWall(x, y, dir_char)

    def _set_wall(self, x, y, direction):
        bit = [WALL_N, WALL_E, WALL_S, WALL_W][direction]
        self.walls[x][y] |= bit
        nx, ny = x + DX[direction], y + DY[direction]
        if 0 <= nx < self.w and 0 <= ny < self.h:
            opp_bit = [WALL_S, WALL_W, WALL_N, WALL_E][direction]
            self.walls[nx][ny] |= opp_bit

    def _has_wall(self, x, y, direction):
        bit = [WALL_N, WALL_E, WALL_S, WALL_W][direction]
        return bool(self.walls[x][y] & bit)

    def _flood_fill(self):
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

    def _best_direction(self):
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

    def _direction_to_action(self, target_dir):
        diff = (target_dir - self.facing) % 4
        if diff == 0:
            self.x += DX[target_dir]
            self.y += DY[target_dir]
            return "forward"
        elif diff == 1:
            self.facing = target_dir
            self.x += DX[target_dir]
            self.y += DY[target_dir]
            return "right"
        elif diff == 3:
            self.facing = target_dir
            self.x += DX[target_dir]
            self.y += DY[target_dir]
            return "left"
        else:
            self.facing = target_dir
            self.x += DX[target_dir]
            self.y += DY[target_dir]
            return "turn_around"

    def _visualize(self):
        for x in range(self.w):
            for y in range(self.h):
                d = self.distance[x][y]
                if d < 255:
                    API.setText(x, y, str(d))
                if (x, y) in self.goals:
                    API.setColor(x, y, "y")
                elif self.visited[x][y]:
                    API.setColor(x, y, "c")
