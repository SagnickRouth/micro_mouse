"""
Flood Fill maze solver for MMS simulator.
Fixed: proper exploration with backtracking when no direct path exists.
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
        self.step_count = 0

        self.walls = [[0] * height for _ in range(width)]
        self.distance = [[255] * height for _ in range(width)]
        self.visited = [[False] * height for _ in range(width)]

        # Outer boundaries
        for i in range(width):
            self.walls[i][0] |= WALL_S
            self.walls[i][height - 1] |= WALL_N
        for j in range(height):
            self.walls[0][j] |= WALL_W
            self.walls[width - 1][j] |= WALL_E

        # Goal: center 2x2 block
        self.goals = []
        cx = width // 2
        cy = height // 2
        for gx in [cx - 1, cx]:
            for gy in [cy - 1, cy]:
                if 0 <= gx < width and 0 <= gy < height:
                    self.goals.append((gx, gy))

        API.log("Maze: {}x{}".format(width, height))
        API.log("Goals: " + str(self.goals))
        API.log("Start: (0, 0) facing NORTH")

        self._flood_fill()
        self._visualize()
        API.setColor(0, 0, "g")
        for gx, gy in self.goals:
            API.setColor(gx, gy, "y")

    def step(self, wall_l, wall_f, wall_r):
        self.step_count += 1

        # Update walls at current position
        self._update_walls(wall_l, wall_f, wall_r)
        self.visited[self.x][self.y] = True

        # Check if we reached a goal
        if (self.x, self.y) in self.goals:
            API.setColor(self.x, self.y, "g")
            API.log("GOAL REACHED at ({},{}) in {} steps".format(
                self.x, self.y, self.step_count))
            return "done"

        API.setColor(self.x, self.y, "c")

        # Recompute flood fill toward goals
        self._flood_fill()
        self._visualize()

        # Try to move toward goal
        best_dir = self._best_direction()

        if best_dir is not None:
            action = self._get_action(best_dir)
            self.x += DX[best_dir]
            self.y += DY[best_dir]
            self.facing = best_dir
            return action

        # No path to goal found — explore unvisited cells instead
        API.log("No path to goal from ({},{}), exploring...".format(self.x, self.y))
        explore_dir = self._find_explore_direction()

        if explore_dir is not None:
            action = self._get_action(explore_dir)
            self.x += DX[explore_dir]
            self.y += DY[explore_dir]
            self.facing = explore_dir
            return action

        # Truly stuck — try to backtrack to any unvisited cell
        API.log("Stuck at ({},{}), backtracking...".format(self.x, self.y))
        backtrack_dir = self._find_backtrack_direction()

        if backtrack_dir is not None:
            action = self._get_action(backtrack_dir)
            self.x += DX[backtrack_dir]
            self.y += DY[backtrack_dir]
            self.facing = backtrack_dir
            return action

        API.log("Completely stuck — no moves available")
        return "done"

    def _get_action(self, target_dir):
        diff = (target_dir - self.facing) % 4
        if diff == 0:
            return "forward"
        elif diff == 1:
            return "right"
        elif diff == 3:
            return "left"
        else:
            return "turn_around"

    def _find_explore_direction(self):
        """Find direction to an unvisited accessible neighbor."""
        for direction in range(4):
            if self._has_wall(self.x, self.y, direction):
                continue
            nx, ny = self.x + DX[direction], self.y + DY[direction]
            if 0 <= nx < self.w and 0 <= ny < self.h:
                if not self.visited[nx][ny]:
                    return direction
        return None

    def _find_backtrack_direction(self):
        """Find direction to any accessible neighbor (even visited).
        Prefer cells with lower distance to an unvisited cell."""
        # Flood fill from all unvisited cells to find nearest one
        bt_dist = [[255] * self.h for _ in range(self.w)]
        queue = deque()

        for ux in range(self.w):
            for uy in range(self.h):
                if not self.visited[ux][uy]:
                    bt_dist[ux][uy] = 0
                    queue.append((ux, uy))

        while queue:
            cx, cy = queue.popleft()
            d = bt_dist[cx][cy]
            for direction in range(4):
                if self._has_wall(cx, cy, direction):
                    continue
                nx, ny = cx + DX[direction], cy + DY[direction]
                if 0 <= nx < self.w and 0 <= ny < self.h:
                    if bt_dist[nx][ny] > d + 1:
                        bt_dist[nx][ny] = d + 1
                        queue.append((nx, ny))

        # Pick neighbor with lowest distance to unvisited
        best_dist = 255
        best_dir = None
        for direction in range(4):
            if self._has_wall(self.x, self.y, direction):
                continue
            nx, ny = self.x + DX[direction], self.y + DY[direction]
            if 0 <= nx < self.w and 0 <= ny < self.h:
                if bt_dist[nx][ny] < best_dist:
                    best_dist = bt_dist[nx][ny]
                    best_dir = direction

        return best_dir

    def _update_walls(self, wall_l, wall_f, wall_r):
        x, y, f = self.x, self.y, self.facing
        front = f
        left = (f + 3) % 4
        right = (f + 1) % 4

        for direction, has_wall in [(front, wall_f), (left, wall_l), (right, wall_r)]:
            if has_wall:
                self._set_wall(x, y, direction)
                API.setWall(x, y, "nesw"[direction])

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
