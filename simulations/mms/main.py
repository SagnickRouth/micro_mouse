"""
Micromouse MMS Simulator - Main Entry Point

Algorithms:
  flood_fill     - BFS optimal (default)
  left_wall      - Left wall follower
  right_wall     - Right wall follower
  dead_end_fill  - Dead-end pruning + flood fill

Usage in MMS:
  Run command: python main.py
  Or: python main.py --alg left_wall
"""

import sys
import API
from flood_fill import FloodFillSolver
from wall_follower import LeftWallSolver, RightWallSolver
from dead_end_fill import DeadEndFillSolver

ALGORITHM = "flood_fill"

ALGORITHMS = {
    "flood_fill": FloodFillSolver,
    "left_wall": LeftWallSolver,
    "right_wall": RightWallSolver,
    "dead_end_fill": DeadEndFillSolver,
}


def main():
    global ALGORITHM

    # Parse command line args
    for i, arg in enumerate(sys.argv):
        if arg == "--alg" and i + 1 < len(sys.argv):
            ALGORITHM = sys.argv[i + 1]
        elif arg.startswith("--alg="):
            ALGORITHM = arg.split("=")[1]

    API.log("=== MICROMOUSE MMS ===")
    API.log("Algorithm: " + ALGORITHM)

    width = API.mazeWidth()
    height = API.mazeHeight()
    API.log("Maze size: {}x{}".format(width, height))

    solver_class = ALGORITHMS.get(ALGORITHM)
    if solver_class is None:
        API.log("Unknown algorithm: " + ALGORITHM)
        API.log("Available: " + str(list(ALGORITHMS.keys())))
        return

    solver = solver_class(width, height)
    API.log("Solver: " + solver.__class__.__name__)

    # Main loop
    while True:
        if API.wasReset():
            API.log("Reset detected")
            API.ackReset()
            solver = solver_class(width, height)
            continue

        wall_f = API.wallFront()
        wall_l = API.wallLeft()
        wall_r = API.wallRight()

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
            API.log("Unknown action: " + str(action))
            break

    API.log("Run complete.")


if __name__ == "__main__":
    main()
