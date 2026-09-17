/**
 * @file    encoder.h
 * @brief   Quadrature encoder interface.
 *
 * Left encoder:  TIM1, PA8/PA9
 * Right encoder: TIM4, PB6/PB7
 */

#ifndef ENCODER_H
#define ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** Initialize TIM1 (left) and TIM4 (right) in hardware encoder mode. */
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
