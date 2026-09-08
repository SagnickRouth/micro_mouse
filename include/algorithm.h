/**
 * @file    algorithm.h
 * @brief   Multiple maze-solving algorithms with switch-based selection.
 * @date    2026-09-08
 *
 * CELESTA'26 allows adjusting switch settings between runs to change
 * algorithms. This module provides multiple solving strategies:
 *
 *   Switch State → Algorithm
 *   ─────────────────────────
 *   00 (both OFF)  → Flood Fill (default, optimal)
 *   01 (SW2 ON)    → Left Wall Follower (simple, reliable)
 *   10 (SW1 ON)    → Right Wall Follower (alternate simple)
 *   11 (both ON)   → Dead-End Fill + Flood Fill (hybrid)
 *
 * Additionally, a speed mode switch selects between:
 *   MODE button held at boot → Speed Run (use stored maze from Run 1)
 *   MODE button not held     → Search Run (explore unknown maze)
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
    ALG_FLOOD_FILL       = 0,   /* BFS flood fill (optimal) */
    ALG_LEFT_WALL        = 1,   /* Left wall follower */
    ALG_RIGHT_WALL       = 2,   /* Right wall follower */
    ALG_DEAD_END_FILL    = 3,   /* Dead-end fill + flood fill hybrid */
    ALG_COUNT            = 4
} AlgorithmType;

/* ── Run Mode ───────────────────────────────────────────── */
typedef enum {
    MODE_SEARCH          = 0,   /* Explore unknown maze */
    MODE_SPEED_RUN       = 1    /* Use stored maze, run fastest path */
} RunMode;

/* ── Speed Profile ──────────────────────────────────────── */
typedef enum {
    SPEED_CAUTIOUS       = 0,   /* Slow and safe (first run) */
    SPEED_NORMAL         = 1,   /* Balanced */
    SPEED_AGGRESSIVE     = 2    /* Maximum speed (speed run) */
} SpeedProfile;

/* ── Algorithm State ────────────────────────────────────── */
typedef struct {
    AlgorithmType  algorithm;
    RunMode        mode;
    SpeedProfile   speed;
    const char    *alg_name;
} AlgorithmConfig;

/* ── Switch Reading ─────────────────────────────────────── */

/**
 * Read physical switches and determine algorithm + mode.
 * Call once at boot (before run starts).
 *
 * Switch mapping (using existing buttons + optional DIP switches):
 *   BTN_START (PB8): Start the run
 *   BTN_MODE  (PB9): Hold at boot → Speed Run mode
 *   SW1 (PA2):  Algorithm select bit 0  (optional DIP switch)
 *   SW2 (PA3):  Algorithm select bit 1  (optional DIP switch)
 *
 * If no DIP switches available, cycle through algorithms
 * by pressing MODE button repeatedly before starting.
 */
AlgorithmConfig algorithm_read_switches(void);

/**
 * Cycle to next algorithm (for button-based selection).
 * Returns the new algorithm config.
 */
AlgorithmConfig algorithm_cycle_next(void);

/** Get current algorithm config. */
const AlgorithmConfig* algorithm_get_config(void);

/** Get algorithm name string. */
const char* algorithm_get_name(AlgorithmType alg);

/* ── Algorithm Execution ────────────────────────────────── */

/**
 * Get next direction based on current algorithm.
 * This is the main decision function called each cell.
 *
 * @param x       Current cell X
 * @param y       Current cell Y
 * @param facing  Current heading
 * @param wall_l  Left wall present
 * @param wall_f  Front wall present
 * @param wall_r  Right wall present
 * @return        Direction to move next
 */
Direction algorithm_next_direction(
    uint8_t x, uint8_t y, Direction facing,
    bool wall_l, bool wall_f, bool wall_r
);

/* ── Individual Algorithm Implementations ───────────────── */

/** Flood fill: update walls + recompute + pick best neighbor. */
Direction alg_flood_fill_step(uint8_t x, uint8_t y, Direction facing,
                              bool wall_l, bool wall_f, bool wall_r);

/** Left wall follower: always turn left if possible. */
Direction alg_left_wall_step(uint8_t x, uint8_t y, Direction facing,
                             bool wall_l, bool wall_f, bool wall_r);

/** Right wall follower: always turn right if possible. */
Direction alg_right_wall_step(uint8_t x, uint8_t y, Direction facing,
                              bool wall_l, bool wall_f, bool wall_r);

/** Dead-end fill: mark dead-ends, then flood fill. */
Direction alg_dead_end_fill_step(uint8_t x, uint8_t y, Direction facing,
                                 bool wall_l, bool wall_f, bool wall_r);

#ifdef __cplusplus
}
#endif

#endif /* ALGORITHM_H */
