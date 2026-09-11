# 🐭 Micromouse MMS Simulator Integration

Test your maze-solving algorithms visually using [MMS (Mackorone's Micromouse Simulator)](https://github.com/mackorone/mms).

## Setup

1. **Download & build MMS** from [github.com/mackorone/mms](https://github.com/mackorone/mms)
2. Open MMS
3. Click **File → Algorithm → Configure**
4. Set **Directory** to this folder (`simulations/mms/`)
5. Set **Run command** to: `python3 main.py`
6. Click **OK**, then **Run**

## Algorithms

Switch algorithms via command line:

```bash
python3 main.py --alg flood_fill      # Default — BFS optimal
python3 main.py --alg left_wall       # Left wall follower
python3 main.py --alg right_wall      # Right wall follower  
python3 main.py --alg dead_end_fill   # Dead-end pruning + flood fill
```

Or change the `ALGORITHM` variable in `main.py`.

## Visualization

| Color | Meaning |
|-------|---------|
| 🟢 Green | Start / Goal reached |
| 🟡 Yellow | Goal cells |
| 🔵 Cyan | Visited cells |
| ⬜ Gray | Dead-end (pruned) |
| Numbers | Flood fill distance to goal |

## Files

| File | Description |
|------|-------------|
| `API.py` | MMS simulator API wrapper (stdin/stdout protocol) |
| `main.py` | Entry point — algorithm selection + main loop |
| `flood_fill.py` | BFS flood fill solver with visualization |
| `wall_follower.py` | Left & right wall follower solvers |
| `dead_end_fill.py` | Dead-end fill + flood fill hybrid |

## Notes

- **CELESTA'26**: Wall-hugging algorithms (left/right wall) will NOT find the destination. Use flood fill or dead-end fill for competition.
- Goal defaults to center 2×2 block. Change `self.goals` in the solver for a different finish location.
- The simulator uses `stdout` for commands and `stdin` for responses. Debug logs go to `stderr`.
