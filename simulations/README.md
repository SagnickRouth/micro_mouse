# Micromouse Simulations

## A* with mackorone/mms

`a_star.py` is an MMS-compatible, online A* solver for a 16×16 Micromouse maze.

It does not assume that the complete maze is known. At every cell it:

1. Reads `wallFront`, `wallLeft`, and `wallRight` from MMS.
2. Updates its internal wall map.
3. Runs A* from the current cell to the nearest center goal cell.
4. Turns toward the first step of the selected path.
5. Checks the forward wall again.
6. Moves one cell and repeats.

This makes the simulation behave more like a real Micromouse: the planner only
knows walls that the robot has discovered.

### Run with MMS

Use the Windows MMS executable and configure the mouse/algorithm command to run:

```text
python simulations/a_star.py
```

Or, from the repository root:

```text
python simulations/a_star.py
```

MMS communicates with the algorithm through stdin/stdout. Therefore the A*
program sends only MMS API commands to stdout; diagnostic messages are written
to stderr.

### Algorithm

The planner uses:

- Manhattan distance as the A* heuristic.
- Unit cost for every cell-to-cell movement.
- The four center cells `(7,7)`, `(7,8)`, `(8,7)`, `(8,8)` as goals for a 16×16 maze.
- Dynamic re-planning after every sensor update.

### Visualization

The solver calls MMS `setWall`, `setColor`, and `setText` so the MMS window can
show the walls discovered by the simulated mouse and the cells it has visited.

### Relationship to firmware

The Python program is an algorithm simulation for MMS. The embedded firmware
in `src/maze.c` remains the STM32 implementation and currently uses flood fill.
The A* implementation is intentionally isolated in `simulations/a_star.py`, so
it can be tested in MMS without changing the STM32 firmware.
