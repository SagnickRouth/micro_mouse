#include "main.h"
#include "config.h"
#include "algorithm.h"
#include "oled.h"
#include "i2c.h"
#include <stdbool.h>
#include <stdint.h>

static bool button_pressed(void)
{
    static GPIO_PinState stable = GPIO_PIN_SET;
    static GPIO_PinState last_raw = GPIO_PIN_SET;
    static uint32_t changed_at = 0;

    GPIO_PinState raw = HAL_GPIO_ReadPin(KEY_PORT, KEY_PIN);
    uint32_t now = HAL_GetTick();

    if (raw != last_raw) {
        last_raw = raw;
        changed_at = now;
    }

    if ((now - changed_at) >= 30U && raw != stable) {
        GPIO_PinState old = stable;
        stable = raw;
        return (old == GPIO_PIN_SET && stable == GPIO_PIN_RESET);
    }

    return false;
}

static void ui_gpio_init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* PB4 = ALG0, PB3 = ALG1. Both DIP inputs are active-low. */
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

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    ui_gpio_init();
    MX_I2C1_Init();
    oled_init();

    AlgorithmConfig selected = algorithm_read_switches();
    oled_show_algorithm(selected.alg_name, false);

    bool led_on = false;
    AlgorithmType shown = selected.algorithm;

    while (1) {
        /* Update OLED when either algorithm DIP switch changes. */
        AlgorithmConfig current = algorithm_read_switches();
        if (current.algorithm != shown) {
            shown = current.algorithm;
            oled_show_algorithm(current.alg_name, false);
        }

        /* PA0 button toggles the PC13 onboard LED once per press. */
        if (button_pressed()) {
            led_on = !led_on;
            HAL_GPIO_WritePin(LED_PORT, LED_PIN,
                              led_on ? GPIO_PIN_RESET : GPIO_PIN_SET);
        }

        HAL_Delay(10);
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {
        HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
        HAL_Delay(250);
    }
}
