#include "main.h"
#include "config.h"
#include "oled.h"
#include "i2c.h"
#include <stdbool.h>
#include <stdint.h>

/* Diagnostic firmware: HSI 16 MHz, no PLL. */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState = RCC_PLL_NONE;

    if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();

    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_0) != HAL_OK) Error_Handler();
}

static void ui_gpio_init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* Release PB3/PB4 from their normal debug/JTAG role and use them as GPIO.
       SWD itself remains available on PA13/PA14. */
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    HAL_SYSCFG_DisableFastModePlus(SYSCFG_PB3); /* harmless on F4 families that expose it */

    /* PB4 = DIP0, PB3 = DIP1. External switches should connect to GND when ON. */
    g.Pin = GPIO_PIN_3 | GPIO_PIN_4;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &g);

    /* PA0 = push button. Button should connect PA0 to GND when pressed. */
    g.Pin = GPIO_PIN_0;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &g);

    /* PC13 onboard LED is active-low. */
    g.Pin = GPIO_PIN_13;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &g);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
}

static const char *read_algorithm_name(void)
{
    bool sw0 = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_RESET);
    bool sw1 = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_RESET);

    if (!sw0 && !sw1) return "FLOOD FILL";
    if ( sw0 && !sw1) return "LEFT WALL";
    if (!sw0 &&  sw1) return "RIGHT WALL";
    return "A STAR";
}

static void update_oled(void)
{
    oled_show_algorithm(read_algorithm_name(), false);
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    ui_gpio_init();

    /* Give the OLED supply time to settle on cold power-up. */
    HAL_Delay(1000);
    MX_I2C1_Init();
    oled_init();
    update_oled();

    uint8_t old_dip = 0xFF;

    while (1) {
        /* Read the physical PB4/PB3 pins directly for this diagnostic.
           This deliberately bypasses algorithm.c so no other module can
           affect the result. */
        uint8_t dip = 0;
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_RESET) dip |= 1U;
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_RESET) dip |= 2U;

        if (dip != old_dip) {
            old_dip = dip;
            update_oled();
        }

        /* Diagnostic mode: PC13 directly follows PA0.
           PA0 released = LED OFF; PA0 pressed to GND = LED ON. */
        GPIO_PinState key = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,
                          (key == GPIO_PIN_RESET) ? GPIO_PIN_RESET : GPIO_PIN_SET);

        HAL_Delay(20);
    }
}

void Error_Handler(void)
{
    while (1) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(250);
    }
}
