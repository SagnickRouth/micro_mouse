#ifndef HARDWARE_TEST_H
#define HARDWARE_TEST_H

#include <stdint.h>
#include <stdbool.h>

/*
 * Hardware diagnostic for:
 * MCU/main loop, SSD1306, PA0 button, PB3/PB4 DIP switches,
 * TIM1 PA8/PA9 left encoder and TIM4 PB6/PB7 right encoder.
 *
 * Call hardware_test_init() after CubeMX MX_GPIO_Init(), MX_I2C1_Init(),
 * MX_TIM1_Init() and MX_TIM4_Init(), then call hardware_test_run() in
 * the main loop.
 */
void hardware_test_init(void);
void hardware_test_run(void);

#endif
