/**
 * @file    encoder.c
 * @brief   Quadrature encoder driver — hardware timer encoder mode.
 * @author  Sagnick Routh
 * @date    2026-09-06
 *
 * LEFT:  TIM2 CH1/CH2 → PA0/PA1
 * RIGHT: TIM3 CH1/CH2 → PA6/PA7
 *
 * Fix: Previous version mapped both encoders to PA6/PA7.
 */

#include "encoder.h"
#include "config.h"

/* ── Encoder State ──────────────────────────────────────── */
static int32_t count_left;
static int32_t count_right;
static int32_t last_raw_left;
static int32_t last_raw_right;
static float   speed_left;   /* mm/s */
static float   speed_right;  /* mm/s */

/* ── Timer Stubs (replace with real HAL) ────────────────── */

static uint16_t timer_read_left(void) {
    /* TODO: return __HAL_TIM_GET_COUNTER(&htim2); */
    return 0;
}

static uint16_t timer_read_right(void) {
    /* TODO: return __HAL_TIM_GET_COUNTER(&htim3); */
    return 0;
}

/* ── Initialization ─────────────────────────────────────── */
void encoder_init(void) {
    /* TODO: Configure TIM2 in encoder mode on PA0/PA1
     *       Configure TIM3 in encoder mode on PA6/PA7
     *
     * Example (HAL):
     *   htim2.Instance = TIM2;
     *   htim2.Init.Period = 0xFFFF;
     *   HAL_TIM_Encoder_Init(&htim2, &sConfig);
     *   HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
     *   (same for htim3/TIM3)
     */
    encoder_reset();
}

/* ── Update ─────────────────────────────────────────────── */
void encoder_update(void) {
    int32_t raw_left  = (int32_t)timer_read_left();
    int32_t raw_right = (int32_t)timer_read_right();

    /* Compute delta (handle 16-bit overflow) */
    int16_t delta_l = (int16_t)(raw_left  - last_raw_left);
    int16_t delta_r = (int16_t)(raw_right - last_raw_right);

    count_left  += delta_l;
    count_right += delta_r;

    /* Speed in mm/s */
    speed_left  = encoder_ticks_to_mm(delta_l) / CONTROL_DT;
    speed_right = encoder_ticks_to_mm(delta_r) / CONTROL_DT;

    last_raw_left  = raw_left;
    last_raw_right = raw_right;
}

/* ── Reset ──────────────────────────────────────────────── */
void encoder_reset(void) {
    count_left  = 0;
    count_right = 0;
    last_raw_left  = 0;
    last_raw_right = 0;
    speed_left  = 0.0f;
    speed_right = 0.0f;
    /* TODO: __HAL_TIM_SET_COUNTER(&htim2, 0);
     *       __HAL_TIM_SET_COUNTER(&htim3, 0);
     */
}

/* ── Getters ────────────────────────────────────────────── */
int32_t encoder_get_left_count(void)   { return count_left;  }
int32_t encoder_get_right_count(void)  { return count_right; }
float   encoder_get_left_speed(void)   { return speed_left;  }
float   encoder_get_right_speed(void)  { return speed_right; }

/* ── Tick to MM Conversion ──────────────────────────────── */
float encoder_ticks_to_mm(int32_t ticks) {
    return (float)ticks * MM_PER_TICK;
}
