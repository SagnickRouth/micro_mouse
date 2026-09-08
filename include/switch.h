/**
 * @file    switch.h
 * @brief   Physical switch and button reading with debounce.
 *
 * Provides debounced reading of:
 *   - Start button (PB8)
 *   - Mode button (PB9)
 *   - Optional DIP switches for algorithm selection (PA2, PA3)
 *   - LED feedback for current algorithm
 */

#ifndef SWITCH_H
#define SWITCH_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* ── DIP Switch Pins (optional, for algorithm select) ───── */
/* If you don't have DIP switches, use button cycling instead.
 * Wire two small switches to PA2 and PA3 with pull-ups. */
#define SW1_PORT        GPIOA
#define SW1_PIN         GPIO_PIN_2   /* Algorithm bit 0 */
#define SW2_PORT        GPIOA
#define SW2_PIN         GPIO_PIN_3   /* Algorithm bit 1 */

/* ── Functions ──────────────────────────────────────────── */

/** Initialize all switch/button GPIO pins. */
void switch_init(void);

/** Read start button (debounced). Returns true on press. */
bool switch_start_pressed(void);

/** Read mode button (debounced). Returns true if held. */
bool switch_mode_held(void);

/** Read DIP switch 1. */
bool switch_sw1(void);

/** Read DIP switch 2. */
bool switch_sw2(void);

/**
 * Wait for algorithm selection at boot.
 * User can either:
 *   a) Set DIP switches and press START, or
 *   b) Press MODE repeatedly to cycle algorithms, then START
 *
 * LED blinks N times to indicate current algorithm:
 *   1 blink  = Flood Fill
 *   2 blinks = Left Wall
 *   3 blinks = Right Wall
 *   4 blinks = Dead-End Fill
 *
 * Returns when START is pressed.
 */
void switch_wait_for_selection(void);

/**
 * Blink LED N times to indicate algorithm number.
 */
void switch_blink_algorithm(uint8_t alg_number);

#ifdef __cplusplus
}
#endif

#endif /* SWITCH_H */
