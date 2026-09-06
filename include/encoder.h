/**
 * @file    encoder.h
 * @brief   Quadrature encoder interface — TIM2 (left) / TIM3 (right).
 *
 * Pin mapping fix: Left uses PA0/PA1 (TIM2), Right uses PA6/PA7 (TIM3).
 * Previous version had both on PA6/PA7 causing conflicts.
 */

#ifndef ENCODER_H
#define ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** Initialize TIM2 and TIM3 in hardware encoder mode. */
void encoder_init(void);

/** Update encoder counts and speed (call at CONTROL_FREQ_HZ). */
void encoder_update(void);

/** Reset both encoder counters to zero. */
void encoder_reset(void);

/** Get cumulative tick counts. */
int32_t encoder_get_left_count(void);
int32_t encoder_get_right_count(void);

/** Get current speed in mm/s. */
float encoder_get_left_speed(void);
float encoder_get_right_speed(void);

/** Convert ticks to distance in mm. */
float encoder_ticks_to_mm(int32_t ticks);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_H */
