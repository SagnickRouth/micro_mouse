/**
 * @file    a_star.h
 * @brief   A* path planner for the 16x16 micromouse maze.
 *
 * The planner operates on the same wall representation used by maze.c and
 * supports the current dynamic goal set through maze_is_goal(). It is kept
 * separate from the existing flood-fill selector so it can be tested and
 * integrated without changing the four existing switch modes.
 */

#ifndef A_STAR_H
#define A_STAR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

#define A_STAR_MAX_PATH MAZE_SIZE * MAZE_SIZE

typedef struct {
    Direction directions[A_STAR_MAX_PATH];
    uint16_t length;
} AStarPath;

/**
 * Find the shortest path from (start_x,start_y) to any configured goal.
 *
 * The maze is read from the global wall map maintained by maze.c. Unknown
 * cells are treated as open, matching the existing flood-fill architecture.
 * AStarPath::directions contains one absolute direction per cell transition.
 *
 * @return true when a path exists, false when no goal is reachable.
 */
bool a_star_find_path(uint8_t start_x, uint8_t start_y, AStarPath *path);

/**
 * Calculate only the next absolute direction from the current cell.
 * This is the function intended for the cell-by-cell navigation loop.
 */
bool a_star_next_direction(uint8_t start_x, uint8_t start_y,
                           Direction facing, Direction *next_direction);

/** Clear the cached planner state. Safe to call before a new run. */
void a_star_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* A_STAR_H */
