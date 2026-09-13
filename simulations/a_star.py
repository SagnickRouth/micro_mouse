"""
A* Micromouse solver for mackorone/mms.

This program is intended to be run by the MMS simulator. MMS communicates with
algorithms over stdin/stdout, so stdout is reserved for MMS commands and all
human-readable diagnostics go to stderr.

Algorithm:
    1. Start at (0, 0), facing North.
    2. Sense front/left/right walls and update the internal maze map.
    3. Run A* from the current cell to the nearest goal cell.
    4. Follow the first step of that path.
    5. Re-sense, update the map, and re-plan whenever new information is found.

Unknown edges are treated as open during planning. Before every move the
mouse senses the cell, so a newly discovered wall causes A* to re-plan rather
than blindly driving through it.

Run from MMS by configuring the algorithm to execute this file, for example:
    python simulations/a_star.py
"""

import heapq
import sys
from typing import Dict, List, Optional, Tuple

# Directions: North, East, South, West
N, E, S, W = range(4)
DX = (0, 1, 0, -1)
DY = (1, 0, -1, 0)
WALL = (1, 2, 4, 8)
OPPOSITE = (4, 8, 1, 2)
DIR_NAMES = ("N", "E", "S", "W")

SIZE = 16
GOALS = {(7, 7), (7, 8), (8, 7), (8, 8)}
INF = 10**9


def log(message: str) -> None:
    """Write diagnostics without interfering with MMS stdout protocol."""
    print(message, file=sys.stderr, flush=True)


class MMS:
    """Small stdin/stdout adapter for the mackorone/mms mouse API."""

    def command(self, command: str) -> str:
        print(command, flush=True)
        response = sys.stdin.readline()
        if response == "":
            raise RuntimeError("MMS closed the API pipe")
        return response.strip()

    def bool_command(self, command: str) -> bool:
        return self.command(command).lower() == "true"

    def int_command(self, command: str) -> int:
        return int(self.command(command))

    def maze_width(self) -> int:
        return self.int_command("mazeWidth")

    def maze_height(self) -> int:
        return self.int_command("mazeHeight")

    def wall_front(self) -> bool:
        return self.bool_command("wallFront")

    def wall_left(self) -> bool:
        return self.bool_command("wallLeft")

    def wall_right(self) -> bool:
        return self.bool_command("wallRight")

    def move_forward(self) -> str:
        return self.command("moveForward")

    def turn_left(self) -> str:
        return self.command("turnLeft")

    def turn_right(self) -> str:
        return self.command("turnRight")

    def set_wall(self, x: int, y: int, direction: int) -> None:
        self.command(f"setWall {x} {y} {DIR_NAMES[direction].lower()}")

    def set_color(self, x: int, y: int, color: str) -> None:
        self.command(f"setColor {x} {y} {color}")

    def set_text(self, x: int, y: int, text: str) -> None:
        # Keep cell text short; MMS accepts printable ASCII.
        safe = "".join(c if 32 <= ord(c) <= 126 else "?" for c in text)
        self.command(f"setText {x} {y} {safe}")

    def was_reset(self) -> bool:
        return self.bool_command("wasReset")

    def ack_reset(self) -> None:
        self.command("ackReset")


class Maze:
    """Known wall map used by A*."""

    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height
        self.walls = [[0 for _ in range(height)] for _ in range(width)]
        self.known = [[0 for _ in range(height)] for _ in range(width)]

        # Physical maze boundaries are always known.
        for x in range(width):
            self._set_wall(x, 0, S, True, known=True)
            self._set_wall(x, height - 1, N, True, known=True)
        for y in range(height):
            self._set_wall(0, y, W, True, known=True)
            self._set_wall(width - 1, y, E, True, known=True)

    def inside(self, x: int, y: int) -> bool:
        return 0 <= x < self.width and 0 <= y < self.height

    def _set_wall(
        self, x: int, y: int, direction: int, present: bool, known: bool
    ) -> None:
        if not self.inside(x, y):
            return

        mask = WALL[direction]
        if present:
            self.walls[x][y] |= mask
        else:
            self.walls[x][y] &= ~mask

        if known:
            self.known[x][y] |= mask

        nx = x + DX[direction]
        ny = y + DY[direction]
        if self.inside(nx, ny):
            opposite = OPPOSITE[direction]
            if present:
                self.walls[nx][ny] |= opposite
            else:
                self.walls[nx][ny] &= ~opposite
            if known:
                self.known[nx][ny] |= opposite

    def update_wall(self, x: int, y: int, direction: int, present: bool) -> None:
        self._set_wall(x, y, direction, present, known=True)

    def has_wall(self, x: int, y: int, direction: int) -> bool:
        return bool(self.walls[x][y] & WALL[direction])

    def is_known(self, x: int, y: int, direction: int) -> bool:
        return bool(self.known[x][y] & WALL[direction])

    def neighbours(self, x: int, y: int):
        for direction in range(4):
            nx = x + DX[direction]
            ny = y + DY[direction]
            if self.inside(nx, ny) and not self.has_wall(x, y, direction):
                yield nx, ny, direction


class AStar:
    """A* path planner over the currently known maze."""

    @staticmethod
    def heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> int:
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def search(
        self, maze: Maze, start: Tuple[int, int], goals: set
    ) -> Optional[List[Tuple[int, int]]]:
        if start in goals:
            return [start]

        frontier: List[Tuple[int, int, Tuple[int, int]]] = []
        sequence = 0
        h = min(self.heuristic(start, goal) for goal in goals)
        heapq.heappush(frontier, (h, sequence, start))

        came_from: Dict[Tuple[int, int], Optional[Tuple[int, int]]] = {start: None}
        cost_so_far: Dict[Tuple[int, int], int] = {start: 0}

        while frontier:
            _, _, current = heapq.heappop(frontier)

            if current in goals:
                return self._reconstruct(came_from, current)

            for nx, ny, _ in maze.neighbours(*current):
                nxt = (nx, ny)
                new_cost = cost_so_far[current] + 1

                if new_cost < cost_so_far.get(nxt, INF):
                    cost_so_far[nxt] = new_cost
                    priority = new_cost + min(
                        self.heuristic(nxt, goal) for goal in goals
                    )
                    sequence += 1
                    heapq.heappush(frontier, (priority, sequence, nxt))
                    came_from[nxt] = current

        return None

    @staticmethod
    def _reconstruct(
        came_from: Dict[Tuple[int, int], Optional[Tuple[int, int]]],
        current: Tuple[int, int],
    ) -> List[Tuple[int, int]]:
        path = [current]
        while came_from[current] is not None:
            current = came_from[current]  # type: ignore[assignment]
            path.append(current)
        path.reverse()
        return path


def direction_between(a: Tuple[int, int], b: Tuple[int, int]) -> int:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    for direction in range(4):
        if DX[direction] == dx and DY[direction] == dy:
            return direction
    raise ValueError(f"Cells are not adjacent: {a} -> {b}")


def relative_direction(current: int, relative: str) -> int:
    if relative == "front":
        return current
    if relative == "left":
        return (current + 3) % 4
    if relative == "right":
        return (current + 1) % 4
    raise ValueError(relative)


def sense(mms: MMS, maze: Maze, x: int, y: int, direction: int) -> None:
    """Read all three forward-facing sensors and update the known maze."""
    readings = {
        "front": mms.wall_front(),
        "left": mms.wall_left(),
        "right": mms.wall_right(),
    }

    for relative, present in readings.items():
        absolute = relative_direction(direction, relative)
        maze.update_wall(x, y, absolute, present)

        # Mirror the discovered wall in the MMS visualization.
        mms.set_wall(x, y, DIR_NAMES[absolute].lower()) if present else None


def rotate_to(mms: MMS, current: int, target: int) -> int:
    """Turn to target heading using the shortest sequence of 90-degree turns."""
    delta = (target - current) % 4

    if delta == 1:
        mms.turn_right()
    elif delta == 2:
        mms.turn_right()
        mms.turn_right()
    elif delta == 3:
        mms.turn_left()

    return target


def reset_state(maze: Maze) -> Tuple[int, int, int]:
    """Return the physical starting pose after an MMS reset."""
    # Keep the learned maze: MMS reset is a physical reset, not a new maze.
    return 0, 0, N


def main() -> None:
    mms = MMS()

    width = mms.maze_width()
    height = mms.maze_height()

    if width <= 0 or height <= 0:
        raise RuntimeError(f"Invalid MMS maze size: {width}x{height}")

    maze = Maze(width, height)
    goals = {
        (x, y)
        for x in range(width)
        for y in range(height)
        if x in (width // 2 - 1, width // 2)
        and y in (height // 2 - 1, height // 2)
    }
    planner = AStar()

    x, y, heading = 0, 0, N
    steps = 0
    replans = 0

    log(f"A* started: MMS maze {width}x{height}, goals={sorted(goals)}")
    mms.set_color(x, y, "b")

    while (x, y) not in goals:
        if mms.was_reset():
            x, y, heading = reset_state(maze)
            mms.ack_reset()
            log("MMS reset acknowledged")
            continue

        # Always sense before planning. This makes the planner reactive to newly
        # discovered walls and avoids relying on a complete maze map.
        sense(mms, maze, x, y, heading)
        mms.set_color(x, y, "y")
        mms.set_text(x, y, "A")

        path = planner.search(maze, (x, y), goals)
        replans += 1

        if not path or len(path) < 2:
            raise RuntimeError(
                f"A* found no path from ({x},{y}) using known maze information"
            )

        next_cell = path[1]
        target_heading = direction_between((x, y), next_cell)
        heading = rotate_to(mms, heading, target_heading)

        # Sense again after turning. The forward sensor now corresponds to the
        # actual movement direction.
        if mms.wall_front():
            log(
                f"New wall discovered at ({x},{y}) heading {DIR_NAMES[heading]}; re-planning"
            )
            maze.update_wall(x, y, heading, True)
            mms.set_wall(x, y, DIR_NAMES[heading].lower())
            continue

        result = mms.move_forward()
        if result != "ack":
            if result == "crash":
                raise RuntimeError(
                    f"MMS reported crash at ({x},{y}) heading {DIR_NAMES[heading]}"
                )
            raise RuntimeError(f"MMS moveForward returned: {result}")

        x += DX[heading]
        y += DY[heading]
        steps += 1
        mms.set_color(x, y, "y")

        if steps % 4 == 0:
            log(f"A*: position=({x},{y}) heading={DIR_NAMES[heading]} steps={steps} replans={replans}")

    mms.set_color(x, y, "g")
    mms.set_text(x, y, "GOAL")
    log(f"A* reached goal at ({x},{y}) in {steps} moves and {replans} planning cycles")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        log(f"A* ERROR: {exc}")
        raise
