/**
 * @file oled.h
 * @brief Minimal SSD1306 128x64 I2C interface used by the micromouse UI.
 */
#ifndef OLED_H
#define OLED_H

#include <stdbool.h>
#include "config.h"

void oled_init(void);
void oled_clear(void);
void oled_show_algorithm(const char *name, bool running);
void oled_show_message(const char *line1, const char *line2);
void oled_show_hardware_test(bool button_pressed, uint8_t dip,
                             const char *algorithm,
                             const char *left_count,
                             const char *right_count);

#endif
