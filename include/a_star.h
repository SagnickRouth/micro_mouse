/**
 * @file    a_star.h
 * @brief   Heading-aware A* planner for the micromouse maze.
 *
 * The planner follows the behavior of simulations/mms/Main.py:
 *   - state includes (x, y, facing)
 *   - forward motion has a cost
 *   - 90/180 degree turns have configurable penalties
 *   - planning is performed against the currently known wall map
 *   - unknown edges remain open until a sensor discovers a wall
 *
 * This is intentionally implemented without dynamic allocation for the
 * STM32F103 target. Gyro-based motion execution remains in motion.c.
 */

#ifndef A_STAR_H
#define A_STAR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

#define A_STAR_MAX_PATH       (MAZE_SIZE * MAZE_SIZE)
#define A_STAR_MOVE_COST      2u
#define A_STAR_TURN_COST_90   3u
#define A_STAR_TURN_COST_180  6u

typedef struct {
    Direction directions[A_STAR_MAX_PATH];
    uint16_t length;
    uint16_t cost;
} AStarPath;

/**
 * Find the minimum-cost path from the start pose to any configured goal.
 *
 * Cost model is deliberately identical to Main.py's optimized speed run:
 *   straight: +MOVE_COST
 *   90-degree turn + move: +(TURN_COST_90 + MOVE_COST)
 *   180-degree turn + move: +(TURN_COST_180 + MOVE_COST)
 *
 * The returned directions are absolute N/E/S/W movement directions.
 */
bool a_star_find_path(uint8_t start_x, uint8_t start_y,
                      Direction facing, AStarPath *path);

/**
 * Calculate only the first movement direction of the optimal path.
 * Intended for the normal cell-by-cell navigation loop.
 */
bool a_star_next_direction(uint8_t start_x, uint8_t start_y,
                           Direction facing, Direction *next_direction);

/** Update the internal maze map with the current L/F/R sensor readings. */
void a_star_update_walls(uint8_t x, uint8_t y, Direction facing,
                         bool wall_l, bool wall_f, bool wall_r);

/** Clear planner working state. */
void a_star_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* A_STAR_H */
