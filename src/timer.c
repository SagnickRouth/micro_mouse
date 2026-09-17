#include "timer.h"
#include "config.h"

/*
 * STM32F401 timer/GPIO configuration.
 *
 * TIM1 encoder: PA8 = CH1, PA9 = CH2, AF1
 * TIM3 PWM:     PA6 = CH1, PA7 = CH2, AF2
 * TIM4 encoder: PB6 = CH1, PB7 = CH2, AF2
 *
 * System clock is configured to 84 MHz in main.c.
 * TIM3 runs from 84 MHz with PSC=3 and ARR=999 -> 21 kHz PWM.
 */

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

static void timer_gpio_init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* TIM1 encoder: PA8/PA9, AF1. */
    g.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &g);

    /* TIM3 motor PWM: PA6/PA7, AF2. */
    g.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &g);

    /* TIM4 encoder: PB6/PB7, AF2. */
    g.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &g);
}

void MX_TIM1_Init(void)
{
    TIM_Encoder_InitTypeDef s = {0};
    TIM_MasterConfigTypeDef master = {0};

    __HAL_RCC_TIM1_CLK_ENABLE();

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 0xFFFF;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    s.EncoderMode = TIM_ENCODERMODE_TI12;

    s.IC1Polarity = TIM_ICPOLARITY_RISING;
    s.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    s.IC1Prescaler = TIM_ICPSC_DIV1;
    s.IC1Filter = 4;

    s.IC2Polarity = TIM_ICPOLARITY_RISING;
    s.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    s.IC2Prescaler = TIM_ICPSC_DIV1;
    s.IC2Filter = 4;

    if (HAL_TIM_Encoder_Init(&htim1, &s) != HAL_OK)
        Error_Handler();

    master.MasterOutputTrigger = TIM_TRGO_RESET;
    master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &master) != HAL_OK)
        Error_Handler();
}

void MX_TIM3_Init(void)
{
    TIM_OC_InitTypeDef s = {0};
    TIM_MasterConfigTypeDef master = {0};

    __HAL_RCC_TIM3_CLK_ENABLE();

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 3;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = MOTOR_PWM_MAX;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
        Error_Handler();

    s.OCMode = TIM_OCMODE_PWM1;
    s.Pulse = 0;
    s.OCPolarity = TIM_OCPOLARITY_HIGH;
    s.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&htim3, &s, TIM_CHANNEL_1) != HAL_OK)
        Error_Handler();

    if (HAL_TIM_PWM_ConfigChannel(&htim3, &s, TIM_CHANNEL_2) != HAL_OK)
        Error_Handler();

    master.MasterOutputTrigger = TIM_TRGO_RESET;
    master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &master) != HAL_OK)
        Error_Handler();
}

void MX_TIM4_Init(void)
{
    TIM_Encoder_InitTypeDef s = {0};
    TIM_MasterConfigTypeDef master = {0};

    __HAL_RCC_TIM4_CLK_ENABLE();

    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 0;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 0xFFFF;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    s.EncoderMode = TIM_ENCODERMODE_TI12;

    s.IC1Polarity = TIM_ICPOLARITY_RISING;
    s.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    s.IC1Prescaler = TIM_ICPSC_DIV1;
    s.IC1Filter = 4;

    s.IC2Polarity = TIM_ICPOLARITY_RISING;
    s.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    s.IC2Prescaler = TIM_ICPSC_DIV1;
    s.IC2Filter = 4;

    if (HAL_TIM_Encoder_Init(&htim4, &s) != HAL_OK)
        Error_Handler();

    master.MasterOutputTrigger = TIM_TRGO_RESET;
    master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &master) != HAL_OK)
        Error_Handler();
}

/* Call after SystemClock_Config() and before starting the timers. */
void timer_gpio_and_peripheral_init(void)
{
    timer_gpio_init();
    MX_TIM1_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
}
