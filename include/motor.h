/**
 * @file    motor.h
 * @brief   Motor driver interface (TB6612FNG).
 */

#ifndef MOTOR_H
#define MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** Initialize motor GPIO and PWM (TIM1). */
void motor_init(void);

/** Set left motor speed. Positive = forward, negative = backward. */
void motor_set_left(int16_t speed);

/** Set right motor speed. Positive = forward, negative = backward. */
void motor_set_right(int16_t speed);

/** Enable motor driver (STBY high). */
void motor_enable(void);

/** Disable motor driver (STBY low). */
void motor_disable(void);

/** Brake both motors (short both H-bridges). */
void motor_brake(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_H */
