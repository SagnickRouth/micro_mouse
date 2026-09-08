/**
 * @file    maze.h
 * @brief   Maze data structure and flood fill solver.
 *
 * Updated for CELESTA'26:
 *   - Dynamic finish point (not hardcoded center)
 *   - Checkpoint tracking support
 *   - Configurable goal cells
 */

#ifndef MAZE_H
#define MAZE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

/* ── Maze Data ──────────────────────────────────────────── */
extern uint8_t maze_walls[MAZE_SIZE][MAZE_SIZE];
extern uint8_t maze_distance[MAZE_SIZE][MAZE_SIZE];
extern bool    maze_visited[MAZE_SIZE][MAZE_SIZE];

/* ── Goal Management ────────────────────────────────────── */
/* CELESTA'26: Finish tile is highlighted, not necessarily center.
 * Support setting goal dynamically. */
typedef struct {
    uint8_t x;
    uint8_t y;
} MazeCell;

#define MAX_GOAL_CELLS      4

extern MazeCell maze_goals[MAX_GOAL_CELLS];
extern uint8_t  maze_goal_count;

/* ── Checkpoint Management ──────────────────────────────── */
#define MAX_CHECKPOINTS     16

extern MazeCell maze_checkpoints[MAX_CHECKPOINTS];
extern uint8_t  maze_checkpoint_count;

/* ── Core Functions ─────────────────────────────────────── */

/** Initialize maze (clear walls, distances, set outer boundaries). */
void maze_init(void);

/** Set goal cell(s) for flood fill. Default: center block. */
void maze_set_goals_default(void);

/** Set a specific cell as the goal (for detected finish tile). */
void maze_set_goal(uint8_t x, uint8_t y);

/** Clear all goals. */
void maze_clear_goals(void);

/** Update walls at (x, y) from sensor data + heading. */
void maze_update_walls(uint8_t x, uint8_t y, uint8_t walls);

/** Run flood fill from current goal(s). Updates maze_distance. */
void maze_flood_fill(void);

/** Run flood fill to a specific target (e.g., for return-to-start). */
void maze_flood_fill_to(uint8_t target_x, uint8_t target_y);

/** Get the best direction to move from (x, y) given current facing. */
Direction maze_best_direction(uint8_t x, uint8_t y, Direction facing);

/** Check if (x, y) is a goal cell. */
bool maze_is_goal(uint8_t x, uint8_t y);

/** Mark cell as visited. */
void maze_mark_visited(uint8_t x, uint8_t y);

/** Check if cell was visited. */
bool maze_is_visited(uint8_t x, uint8_t y);

/** Get wall flags at (x, y). */
uint8_t maze_get_walls(uint8_t x, uint8_t y);

/** Get distance value at (x, y). */
uint8_t maze_get_distance(uint8_t x, uint8_t y);

/** Register a checkpoint cell. */
void maze_add_checkpoint(uint8_t x, uint8_t y);

/** Check if (x, y) is a checkpoint. */
bool maze_is_checkpoint(uint8_t x, uint8_t y);

/** Get nearest checkpoint to (x, y) that has been reached. */
bool maze_nearest_checkpoint(uint8_t x, uint8_t y, uint8_t *out_x, uint8_t *out_y);

/** Save maze to flash (for speed run after search run). */
void maze_save_to_flash(void);

/** Load maze from flash (for run 2 speed run). */
bool maze_load_from_flash(void);

#ifdef __cplusplus
}
#endif

#endif /* MAZE_H */
