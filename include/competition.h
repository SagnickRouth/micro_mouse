/**
 * @file    competition.h
 * @brief   Competition rules and constraints — CELESTA'26, IIT Patna.
 * @date    2026-09-08
 *
 * MICROMOUSE'26 RULES (key parameters):
 *   - Robot max size: 15cm × 15cm × 10cm (L × W × H)
 *   - Robot max weight: 3 kg
 *   - Cell size: 20cm × 20cm (wall spacing = 20cm)
 *   - Maze dimensional tolerance: ±5%
 *   - Time limit: 7 minutes per run
 *   - 2 runs allowed, best time wins
 *   - Restart penalty: +5 seconds per restart
 *   - Maze damage penalty: +10 seconds
 *   - Checkpoints throughout maze (restart from nearest)
 *   - Finish tile is highlighted (NOT necessarily center)
 *   - Wall-hugging will NOT find destination (flood-fill required)
 *   - Floor: painted plywood (smooth, low friction)
 *   - No pre-loaded maze data allowed
 *   - Can adjust switches/sensors/speed between runs
 */

#ifndef COMPETITION_H
#define COMPETITION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ── Physical Constraints ───────────────────────────────── */
#define ROBOT_MAX_LENGTH_MM     150      /* 15cm */
#define ROBOT_MAX_WIDTH_MM      150      /* 15cm */
#define ROBOT_MAX_HEIGHT_MM     100      /* 10cm */
#define ROBOT_MAX_WEIGHT_G      3000     /* 3 kg */

/* ── Maze Parameters (CELESTA'26) ───────────────────────── */
#define COMPETITION_CELL_SIZE_MM   200   /* 20cm × 20cm cells */
#define COMPETITION_WALL_SPACING   200   /* 20cm between walls */
#define MAZE_TOLERANCE_PERCENT     5     /* ±5% dimensional tolerance */
/* Effective cell range: 190mm – 210mm */
#define CELL_SIZE_MIN_MM           (COMPETITION_CELL_SIZE_MM * (100 - MAZE_TOLERANCE_PERCENT) / 100)
#define CELL_SIZE_MAX_MM           (COMPETITION_CELL_SIZE_MM * (100 + MAZE_TOLERANCE_PERCENT) / 100)

/* ── Time Limits ────────────────────────────────────────── */
#define RUN_TIME_LIMIT_MS       420000   /* 7 minutes = 420,000 ms */
#define SETUP_TIME_MS           120000   /* 2 minutes setup before run */
#define MAX_RUNS                2        /* Best of 2 runs */

/* ── Scoring ────────────────────────────────────────────── */
#define RESTART_PENALTY_MS      5000     /* 5 seconds per restart */
#define DAMAGE_PENALTY_MS       10000    /* 10 seconds for maze damage */

/* ── Run State ──────────────────────────────────────────── */
typedef struct {
    uint8_t   current_run;           /* 1 or 2 */
    uint32_t  run_start_time_ms;     /* millis() when run started */
    uint32_t  elapsed_ms;            /* current elapsed time */
    uint8_t   restart_count;         /* number of manual restarts */
    uint32_t  penalty_ms;            /* accumulated penalties */
    bool      run_complete;          /* reached finish */
    bool      timed_out;             /* exceeded 7 min */
    /* Checkpoint tracking */
    uint8_t   last_checkpoint_x;
    uint8_t   last_checkpoint_y;
    bool      checkpoint_reached;
} RunState;

/** Initialize run state for a new run. */
void run_state_init(RunState *rs, uint8_t run_number);

/** Update elapsed time. Returns true if timed out. */
bool run_state_update(RunState *rs, uint32_t current_millis);

/** Record a restart (adds penalty). */
void run_state_add_restart(RunState *rs);

/** Record reaching a checkpoint. */
void run_state_set_checkpoint(RunState *rs, uint8_t x, uint8_t y);

/** Get total time including penalties. */
uint32_t run_state_total_time(const RunState *rs);

/** Check if time limit exceeded. */
bool run_state_timed_out(const RunState *rs);

#ifdef __cplusplus
}
#endif

#endif /* COMPETITION_H */
