/**
 * @file    motion.h
 * @brief   Motion control — gyro-primary architecture.
 *
 * ARCHITECTURE:
 *   - Straight-line: gyro yaw-hold PID + encoder distance
 *   - Turns: gyro heading-based rotation
 *   - NO wall-follow PID (binary sensors can't provide gradient)
 *   - Front-wall squaring for yaw drift reset
 */

#ifndef MOTION_H
#define MOTION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "config.h"

/** Initialize motion controller (PID instances, state). */
void motion_init(void);

/** Drive forward one cell (CELL_SIZE_MM) at search speed. */
void motion_move_cell(void);

/** Drive forward a specified distance (mm) at given speed. Blocking. */
void motion_move(float distance_mm, float end_speed);

/** Turn in-place by the given angle (degrees). Positive = right. Blocking. */
void motion_turn(float angle_deg);

/** Turn left 90°. */
void motion_turn_left(void);

/** Turn right 90°. */
void motion_turn_right(void);

/** Turn 180° (about-face). */
void motion_turn_180(void);

/**
 * Execute a direction change relative to current heading.
 * Computes required turn, executes it, then moves one cell.
 * Updates robot pose.
 */
void motion_execute_direction(Direction target_dir);

/**
 * Square up against a front wall to re-zero gyro yaw.
 * Call when front wall is detected to correct drift.
 */
void motion_square_up(void);

/** Update motion controller (called from control loop ISR or main loop). */
void motion_update(void);

/** Check if current motion command is complete. */
bool motion_is_complete(void);

/** Emergency stop — brake both motors immediately. */
void motion_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTION_H */
