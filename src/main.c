#include "main.h"
#include <stdbool.h>
#include <stdint.h>
#include "config.h"
#include "algorithm.h"
#include "oled.h"
#include "i2c.h"

static bool led_state = false;

void SystemClock_Config(void);
void Error_Handler(void);
static void UI_GPIO_Init(void);
static bool Key_Pressed(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    UI_GPIO_Init();

    /* Initialize I2C1 before using the OLED. */
    MX_I2C1_Init();
    oled_init();

    /* PB4 = ALG0, PB3 = ALG1. */
    AlgorithmConfig selected_algorithm = algorithm_read_switches();
    oled_show_algorithm(selected_algorithm.alg_name, false);

    while (1)
    {
        /*
         * PB4/PB3 algorithm selection:
         * OFF/OFF = Flood Fill
         * ON/OFF  = Left Wall
         * OFF/ON  = Right Wall
         * ON/ON   = A*
         */
        AlgorithmConfig current_algorithm = algorithm_read_switches();

        if (current_algorithm.algorithm != selected_algorithm.algorithm)
        {
            selected_algorithm = current_algorithm;
            oled_show_algorithm(selected_algorithm.alg_name, false);
        }

        /* PA0 button toggles the PC13 onboard LED. */
        if (Key_Pressed())
        {
            led_state = !led_state;
            HAL_GPIO_WritePin(
                GPIOC,
                GPIO_PIN_13,
                led_state ? GPIO_PIN_RESET : GPIO_PIN_SET
            );
        }

        HAL_Delay(10);
    }
}

static void UI_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* PB4 = ALG0, PB3 = ALG1; active LOW with pull-ups. */
    GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* PA0 = push button; active LOW with pull-up. */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PC13 = onboard LED; active LOW. */
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
}

static bool Key_Pressed(void)
{
    static GPIO_PinState stable_state = GPIO_PIN_SET;
    static GPIO_PinState last_raw_state = GPIO_PIN_SET;
    static uint32_t change_time = 0;

    GPIO_PinState raw_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
    uint32_t now = HAL_GetTick();

    if (raw_state != last_raw_state)
    {
        last_raw_state = raw_state;
        change_time = now;
    }

    if ((raw_state != stable_state) &&
        ((now - change_time) >= 30U))
    {
        stable_state = raw_state;

        if (stable_state == GPIO_PIN_RESET)
        {
            return true;
        }
    }

    return false;
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 7;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK |
        RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1 |
        RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}
