/**
 * @file    switch.c
 * @brief   Physical switch/button reading with debounce.
 * @author  Sagnick Routh
 * @date    2026-09-08
 */

#include "switch.h"
#include "algorithm.h"
#include "config.h"

/* ── GPIO Stubs ─────────────────────────────────────────── */

static void gpio_init_input_pullup(void *port, uint16_t pin) {
    /* TODO: Configure as input with internal pull-up
     * GPIO_InitTypeDef gpio = {0};
     * gpio.Pin = pin;
     * gpio.Mode = GPIO_MODE_INPUT;
     * gpio.Pull = GPIO_PULLUP;
     * HAL_GPIO_Init(port, &gpio);
     */
    (void)port; (void)pin;
}

static bool gpio_read(void *port, uint16_t pin) {
    /* TODO: return HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET;
     * (active low — pressed = LOW = true) */
    (void)port; (void)pin;
    return false;
}

static void led_on(void) {
    /* TODO: HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
     * (PC13 is active low on Blue Pill) */
}

static void led_off(void) {
    /* TODO: HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET); */
}

static void delay_ms(uint32_t ms) {
    /* TODO: HAL_Delay(ms); */
    for (volatile uint32_t i = 0; i < ms * 7200; i++) { __asm("nop"); }
}

/* ── Debounce ───────────────────────────────────────────── */
#define DEBOUNCE_MS     50
#define DEBOUNCE_READS  5

static bool debounced_read(void *port, uint16_t pin) {
    uint8_t count = 0;
    for (int i = 0; i < DEBOUNCE_READS; i++) {
        if (gpio_read(port, pin)) count++;
        delay_ms(DEBOUNCE_MS / DEBOUNCE_READS);
    }
    return (count > DEBOUNCE_READS / 2);
}

/* ── Initialization ─────────────────────────────────────── */
void switch_init(void) {
    gpio_init_input_pullup(BTN_START_PORT, BTN_START_PIN);
    gpio_init_input_pullup(BTN_MODE_PORT, BTN_MODE_PIN);
    gpio_init_input_pullup(SW1_PORT, SW1_PIN);
    gpio_init_input_pullup(SW2_PORT, SW2_PIN);
}

/* ── Button Reads ───────────────────────────────────────── */
bool switch_start_pressed(void) { return debounced_read(BTN_START_PORT, BTN_START_PIN); }
bool switch_mode_held(void)     { return debounced_read(BTN_MODE_PORT, BTN_MODE_PIN);   }
bool switch_sw1(void)           { return debounced_read(SW1_PORT, SW1_PIN);             }
bool switch_sw2(void)           { return debounced_read(SW2_PORT, SW2_PIN);             }

/* ── LED Feedback ───────────────────────────────────────── */
void switch_blink_algorithm(uint8_t alg_number) {
    /* Blink N+1 times to indicate algorithm (1-indexed for user) */
    for (uint8_t i = 0; i <= alg_number; i++) {
        led_on();
        delay_ms(200);
        led_off();
        delay_ms(200);
    }
    delay_ms(500);  /* Pause after blink sequence */
}

/* ── Boot Selection Loop ────────────────────────────────── */
void switch_wait_for_selection(void) {
    AlgorithmConfig cfg = algorithm_read_switches();

    /* Show current algorithm via LED blinks */
    switch_blink_algorithm((uint8_t)cfg.algorithm);

    /* Wait for START or MODE press */
    while (1) {
        if (switch_start_pressed()) {
            /* Confirm selection — 3 rapid blinks */
            for (int i = 0; i < 3; i++) {
                led_on(); delay_ms(100);
                led_off(); delay_ms(100);
            }
            return;
        }

        if (switch_mode_held()) {
            /* Cycle to next algorithm */
            cfg = algorithm_cycle_next();
            switch_blink_algorithm((uint8_t)cfg.algorithm);

            /* Wait for button release */
            while (switch_mode_held()) { delay_ms(10); }
            delay_ms(200);  /* Extra debounce */
        }

        delay_ms(50);
    }
}
