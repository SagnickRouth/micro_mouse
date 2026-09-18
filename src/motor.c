#include "motor.h"
#include "config.h"
#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim3;

/*
 * GPIO and timer peripheral initialization is owned by CubeMX/.ioc.
 * motor_init() only starts PWM and places the driver in a safe disabled state.
 */

static void set_dir(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    HAL_GPIO_WritePin(port, pin, state);
}

void motor_init(void)
{
    HAL_GPIO_WritePin(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_STBY_PORT, MOTOR_STBY_PIN, GPIO_PIN_RESET);

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);

    (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}

static void pwm_left(uint16_t value)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, value);
}

static void pwm_right(uint16_t value)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, value);
}

void motor_set_left(int16_t speed)
{
    if (speed > MOTOR_PWM_MAX) speed = MOTOR_PWM_MAX;
    if (speed < -MOTOR_PWM_MAX) speed = -MOTOR_PWM_MAX;

    if (speed > 0) {
        set_dir(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN, GPIO_PIN_SET);
        set_dir(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN, GPIO_PIN_RESET);
        pwm_left((uint16_t)speed);
    } else if (speed < 0) {
        set_dir(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN, GPIO_PIN_RESET);
        set_dir(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN, GPIO_PIN_SET);
        pwm_left((uint16_t)-speed);
    } else {
        set_dir(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN, GPIO_PIN_RESET);
        set_dir(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN, GPIO_PIN_RESET);
        pwm_left(0);
    }
}

void motor_set_right(int16_t speed)
{
    if (speed > MOTOR_PWM_MAX) speed = MOTOR_PWM_MAX;
    if (speed < -MOTOR_PWM_MAX) speed = -MOTOR_PWM_MAX;

    if (speed > 0) {
        set_dir(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN, GPIO_PIN_SET);
        set_dir(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN, GPIO_PIN_RESET);
        pwm_right((uint16_t)speed);
    } else if (speed < 0) {
        set_dir(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN, GPIO_PIN_RESET);
        set_dir(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN, GPIO_PIN_SET);
        pwm_right((uint16_t)-speed);
    } else {
        set_dir(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN, GPIO_PIN_RESET);
        set_dir(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN, GPIO_PIN_RESET);
        pwm_right(0);
    }
}

void motor_enable(void)
{
    HAL_GPIO_WritePin(MOTOR_STBY_PORT, MOTOR_STBY_PIN, GPIO_PIN_SET);
}

void motor_disable(void)
{
    HAL_GPIO_WritePin(MOTOR_STBY_PORT, MOTOR_STBY_PIN, GPIO_PIN_RESET);
}

void motor_brake(void)
{
    /*
     * TB6612 braking is implemented by driving both inputs HIGH.
     * This is intentionally separate from motor_set_* (which uses coast at 0).
     */
    set_dir(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN, GPIO_PIN_SET);
    set_dir(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN, GPIO_PIN_SET);
    set_dir(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN, GPIO_PIN_SET);
    set_dir(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN, GPIO_PIN_SET);
    pwm_left(MOTOR_PWM_MAX);
    pwm_right(MOTOR_PWM_MAX);
}

