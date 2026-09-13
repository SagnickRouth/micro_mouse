/**
 * @file    algorithm.h
 * @brief   Maze-solving algorithms with switch/button selection.
 * @date    2026-09-13
 *
 * Existing switch mappings remain unchanged for algorithms 0-3. A* is added
 * as algorithm 4 and can be selected through algorithm_cycle_next() or by
 * future hardware-selection logic without changing the existing two-bit DIP
 * mapping.
 */

#ifndef ALGORITHM_H
#define ALGORITHM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

/* ── Algorithm Types ────────────────────────────────────── */
typedef enum {
    ALG_FLOOD_FILL       = 0,   /* BFS flood fill */
    ALG_LEFT_WALL        = 1,   /* Left wall follower */
    ALG_RIGHT_WALL       = 2,   /* Right wall follower */
    ALG_DEAD_END_FILL    = 3,   /* Dead-end fill + flood fill */
    ALG_A_STAR            = 4,   /* A* shortest path */
    ALG_COUNT             = 5
} AlgorithmType;

/* ── Run Mode ───────────────────────────────────────────── */
typedef enum {
    MODE_SEARCH          = 0,
    MODE_SPEED_RUN       = 1
} RunMode;

/* ── Speed Profile ──────────────────────────────────────── */
typedef enum {
    SPEED_CAUTIOUS       = 0,
    SPEED_NORMAL         = 1,
    SPEED_AGGRESSIVE     = 2
} SpeedProfile;

/* ── Algorithm State ────────────────────────────────────── */
typedef struct {
    AlgorithmType  algorithm;
    RunMode        mode;
    SpeedProfile   speed;
    const char    *alg_name;
} AlgorithmConfig;

AlgorithmConfig algorithm_read_switches(void);
AlgorithmConfig algorithm_cycle_next(void);
const AlgorithmConfig* algorithm_get_config(void);
const char* algorithm_get_name(AlgorithmType alg);

/* ── Algorithm Execution ────────────────────────────────── */
Direction algorithm_next_direction(
    uint8_t x, uint8_t y, Direction facing,
    bool wall_l, bool wall_f, bool wall_r
);

Direction alg_flood_fill_step(uint8_t x, uint8_t y, Direction facing,
                              bool wall_l, bool wall_f, bool wall_r);
Direction alg_left_wall_step(uint8_t x, uint8_t y, Direction facing,
                             bool wall_l, bool wall_f, bool wall_r);
Direction alg_right_wall_step(uint8_t x, uint8_t y, Direction facing,
                              bool wall_l, bool wall_f, bool wall_r);
Direction alg_dead_end_fill_step(uint8_t x, uint8_t y, Direction facing,
                                 bool wall_l, bool wall_f, bool wall_r);

/**
 * A* step using the current known maze map and configured goal cells.
 * Returns facing unchanged if A* cannot find a route.
 */
Direction alg_a_star_step(uint8_t x, uint8_t y, Direction facing,
                          bool wall_l, bool wall_f, bool wall_r);

#ifdef __cplusplus
}
#endif

#endif /* ALGORITHM_H */
