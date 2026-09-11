#!/usr/bin/env python3
\"\"\"
Micromouse MMS Simulator — Main Entry Point

Implements all 4 algorithms from the micro_mouse repo:
  0: Flood Fill (default)
  1: Left Wall Follower
  2: Right Wall Follower
  3: Dead-End Fill + Flood Fill

Usage in MMS:
  1. Open MMS simulator
  2. Set algorithm directory to this folder
  3. Set run command to: python3 main.py
  4. Press Run

Algorithm selection:
  - Set ALGORITHM variable below, or
  - Pass as command line arg: python3 main.py --alg flood_fill

Maze cell size: 20cm × 20cm (CELESTA'26 rules)
Finish: Can be anywhere (not necessarily center)
\"\"\"

import sys
import API
from flood_fill import FloodFillSolver
from wall_follower import LeftWallSolver, RightWallSolver
from dead_end_fill import DeadEndFillSolver

# ── Algorithm Selection ─────────────────────────────────────
ALGORITHM = "flood_fill"  # Options: flood_fill, left_wall, right_wall, dead_end_fill

ALGORITHMS = {
    "flood_fill": FloodFillSolver,
    "left_wall": LeftWallSolver,
    "right_wall": RightWallSolver,
    "dead_end_fill": DeadEndFillSolver,
}


def main():
    global ALGORITHM

    # Parse command line args
    if len(sys.argv) > 1:
        for arg in sys.argv[1:]:
            if arg.startswith("--alg="):
                ALGORITHM = arg.split("=")[1]
            elif arg == "--alg" and sys.argv.index(arg) + 1 < len(sys.argv):
                ALGORITHM = sys.argv[sys.argv.index(arg) + 1]

    API.log(f"=== MICROMOUSE MMS ===")
    API.log(f"Algorithm: {ALGORITHM}")

    width = API.mazeWidth()
    height = API.mazeHeight()
    API.log(f"Maze size: {width} x {height}")

    # Create solver
    solver_class = ALGORITHMS.get(ALGORITHM)
    if solver_class is None:
        API.log(f"Unknown algorithm: {ALGORITHM}")
        API.log(f"Available: {list(ALGORITHMS.keys())}")
        return

    solver = solver_class(width, height)
    API.log(f"Solver initialized: {solver.__class__.__name__}")

    # ── Main Loop ─────────────────────────────────────────
    while True:
        # Check for reset
        if API.wasReset():
            API.log("Reset detected — restarting")
            API.ackReset()
            solver = solver_class(width, height)
            continue

        # Get wall readings
        wall_f = API.wallFront()
        wall_l = API.wallLeft()
        wall_r = API.wallRight()

        # Let solver decide next action
        action = solver.step(wall_l, wall_f, wall_r)

        if action == "done":
            API.log("=== MAZE SOLVED! ===")
            break
        elif action == "forward":
            API.moveForward()
        elif action == "left":
            API.turnLeft()
            API.moveForward()
        elif action == "right":
            API.turnRight()
            API.moveForward()
        elif action == "turn_left":
            API.turnLeft()
        elif action == "turn_right":
            API.turnRight()
        elif action == "turn_around":
            API.turnAround()
            API.moveForward()
        else:
            API.log(f"Unknown action: {action}")
            break

    API.log("Run complete.")


if __name__ == "__main__":
    main()
