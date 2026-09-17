#include "main.h"
#include "config.h"
#include "algorithm.h"
#include "oled.h"
#include "i2c.h"
#include <stdbool.h>
#include <stdint.h>

/* Keep the diagnostic firmware on the STM32F401 HSI clock only.
 * This removes PLL startup from the OLED/push-button test path. */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState = RCC_PLL_NONE;

    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        Error_Handler();
    }

    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }
}

static void ui_gpio_init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* PB4 = ALG0, PB3 = ALG1. DIP switches are active-low. */
    g.Pin = DIP_ALG0_PIN | DIP_ALG1_PIN;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &g);

    /* PA0 = temporary test push button, active-low. */
    g.Pin = KEY_PIN;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(KEY_PORT, &g);

    /* PC13 = onboard LED, active-low. */
    g.Pin = LED_PIN;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &g);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
}

static bool button_pressed_event(void)
{
    static GPIO_PinState previous = GPIO_PIN_SET;
    GPIO_PinState now = HAL_GPIO_ReadPin(KEY_PORT, KEY_PIN);
    bool pressed = (previous == GPIO_PIN_SET && now == GPIO_PIN_RESET);
    previous = now;
    return pressed;
}

static void show_diagnostic(void)
{
    AlgorithmConfig alg = algorithm_read_switches();
    GPIO_PinState key = HAL_GPIO_ReadPin(KEY_PORT, KEY_PIN);

    /* The OLED font supports letters and spaces; the algorithm name gives
       us the DIP result while the key state is reflected in the LED. */
    oled_show_algorithm(alg.alg_name, key == GPIO_PIN_RESET);
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    ui_gpio_init();

    /* Allow the OLED power rail to settle before the first I2C transaction. */
    HAL_Delay(1000);

    MX_I2C1_Init();

    /* Retry OLED startup a few times to make cold-power-up behavior visible
       without requiring an NRESET press. */
    for (uint8_t i = 0; i < 3; i++) {
        oled_init();
        HAL_Delay(100);
    }

    show_diagnostic();

    AlgorithmType shown = algorithm_read_switches().algorithm;
    bool led_on = false;
    uint32_t last_display = HAL_GetTick();

    while (1) {
        AlgorithmConfig current = algorithm_read_switches();

        /* Change the OLED immediately when the DIP selection changes. */
        if (current.algorithm != shown) {
            shown = current.algorithm;
            show_diagnostic();
        }

        /* PA0 press toggles the active-low PC13 onboard LED. */
        if (button_pressed_event()) {
            led_on = !led_on;
            HAL_GPIO_WritePin(LED_PORT, LED_PIN,
                              led_on ? GPIO_PIN_RESET : GPIO_PIN_SET);
            show_diagnostic();
        }

        /* Refresh periodically so the diagnostic screen reflects PA0. */
        if ((HAL_GetTick() - last_display) >= 250U) {
            last_display = HAL_GetTick();
            show_diagnostic();
        }

        HAL_Delay(10);
    }
}

void Error_Handler(void)
{
    while (1) {
        HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
        HAL_Delay(250);
    }
}
