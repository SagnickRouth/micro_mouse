/**
 * @file    motor.c
 * @brief   Motor driver for TB6612FNG dual H-bridge.
 * @author  Sagnick Routh
 * @date    2026-09-06
 *
 * Pin mapping:
 *   PA8  → PWMA (TIM1_CH1) — Left motor
 *   PA11 → PWMB (TIM1_CH4) — Right motor
 *   PB12/PB13 → AIN1/AIN2  — Left direction
 *   PB14/PB15 → BIN1/BIN2  — Right direction
 *   PA15 → STBY            — Standby (enable/disable)
 */

#include "motor.h"
#include "config.h"

/* ── GPIO/PWM Stubs (replace with real HAL) ─────────────── */

static void gpio_write(void *port, uint16_t pin, bool state) {
    /* TODO: HAL_GPIO_WritePin(port, pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET); */
    (void)port; (void)pin; (void)state;
}

static void pwm_set(uint8_t channel, uint16_t value) {
    /* TODO: __HAL_TIM_SET_COMPARE(&htim1, channel, value); */
    (void)channel; (void)value;
}

/* ── Initialization ─────────────────────────────────────── */
void motor_init(void) {
    /* TODO: Configure GPIO pins for AIN1/AIN2/BIN1/BIN2/STBY as outputs
     *       Configure TIM1 CH1 and CH4 as PWM output (~20kHz)
     *       Start PWM: HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1/4);
     */
    motor_disable();
    motor_brake();
}

/* ── Set Motor Speed ────────────────────────────────────── */
void motor_set_left(int16_t speed) {
    if (speed > MOTOR_PWM_MAX)  speed = MOTOR_PWM_MAX;
    if (speed < -MOTOR_PWM_MAX) speed = -MOTOR_PWM_MAX;

    if (speed > 0) {
        gpio_write(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN, true);
        gpio_write(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN, false);
        pwm_set(1, (uint16_t)speed);   /* TIM1_CH1 */
    } else if (speed < 0) {
        gpio_write(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN, false);
        gpio_write(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN, true);
        pwm_set(1, (uint16_t)(-speed));
    } else {
        gpio_write(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN, false);
        gpio_write(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN, false);
        pwm_set(1, 0);
    }
}

void motor_set_right(int16_t speed) {
    if (speed > MOTOR_PWM_MAX)  speed = MOTOR_PWM_MAX;
    if (speed < -MOTOR_PWM_MAX) speed = -MOTOR_PWM_MAX;

    if (speed > 0) {
        gpio_write(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN, true);
        gpio_write(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN, false);
        pwm_set(4, (uint16_t)speed);   /* TIM1_CH4 */
    } else if (speed < 0) {
        gpio_write(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN, false);
        gpio_write(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN, true);
        pwm_set(4, (uint16_t)(-speed));
    } else {
        gpio_write(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN, false);
        gpio_write(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN, false);
        pwm_set(4, 0);
    }
}

/* ── Enable / Disable ───────────────────────────────────── */
void motor_enable(void)  { gpio_write(MOTOR_STBY_PORT, MOTOR_STBY_PIN, true);  }
void motor_disable(void) { gpio_write(MOTOR_STBY_PORT, MOTOR_STBY_PIN, false); }

/* ── Brake ──────────────────────────────────────────────── */
void motor_brake(void) {
    motor_set_left(0);
    motor_set_right(0);
}
