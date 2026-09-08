/**
 * @file    config_competition.h
 * @brief   Competition-specific overrides for CELESTA'26, IIT Patna.
 * @date    2026-09-08
 *
 * Include this AFTER config.h to override defaults with competition values.
 *
 * KEY CHANGES FROM STANDARD:
 *   - Cell size: 200mm (was 180mm)
 *   - Maze size: configurable via switch (not hardcoded 16)
 *   - Finish point: detected dynamically (not center 2x2)
 *   - 7-minute time limit
 *   - Checkpoint support
 *   - Low-friction floor compensation
 */

#ifndef CONFIG_COMPETITION_H
#define CONFIG_COMPETITION_H

/* ── Override Cell Size ─────────────────────────────────── */
#undef  CELL_SIZE_MM
#define CELL_SIZE_MM            200      /* CELESTA'26: 20cm cells */

/* ── Override Goal (finish is NOT necessarily center) ────── */
#undef  GOAL_MIN
#undef  GOAL_MAX
/* Goal is dynamically detected — highlighted finish tile.
 * We still use flood-fill, but the goal cell(s) are set at runtime
 * when the robot detects the highlighted finish marker.
 * Default to center as fallback if no marker detection. */
#define GOAL_MIN                7        /* Fallback: center area */
#define GOAL_MAX                8

/* ── Maze Size (may not be 16×16) ───────────────────────── */
/* The rules don't specify a fixed maze size. Default to 16
 * but allow runtime configuration via switch/button. */
/* #undef  MAZE_SIZE */
/* #define MAZE_SIZE            16 */   /* Keep 16 as default */

/* ── Low Friction Floor Compensation ────────────────────── */
/* Painted plywood = smoother than standard maze floors.
 * Reduce speed and increase decel for safety. */
#undef  SEARCH_SPEED_MMPS
#define SEARCH_SPEED_MMPS       180      /* Slower for low friction (was 200) */

#undef  DECEL_MMPS2
#define DECEL_MMPS2             1200     /* Stronger decel for slippery floor (was 1000) */

/* ── Turn Parameters (adjust for low friction) ──────────── */
#undef  TURN_SPEED_MMPS
#define TURN_SPEED_MMPS         120      /* Slower turns on smooth floor (was 150) */

#undef  TURN_DEADBAND_DEG
#define TURN_DEADBAND_DEG       3.0f     /* Wider deadband for slippery surface (was 2.0) */

/* ── Encoder Constants (recalculated for 200mm cells) ───── */
/* MM_PER_TICK stays the same — it's based on wheel diameter.
 * But distance-per-cell changes, so motion_move_cell() uses
 * the updated CELL_SIZE_MM automatically. */

#endif /* CONFIG_COMPETITION_H */
