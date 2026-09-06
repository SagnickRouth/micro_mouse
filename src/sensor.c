/**
 * @file    sensor.c
 * @brief   Digital IR wall sensor driver — GPIO reads with debounce.
 * @author  Sagnick Routh
 * @date    2026-09-06
 *
 * Reads 5 digital IR obstacle avoidance modules (FC-51 type).
 * Each module outputs HIGH or LOW based on its onboard comparator.
 * Calibration is done physically via each module's potentiometer.
 *
 * No ADC, no ambient compensation, no analog thresholds — all handled
 * on-board by the modules.
 */

#include "sensor.h"
#include "config.h"

/* ── Global Sensor Data ─────────────────────────────────── */
SensorData sensor_data;

/* ── GPIO Stubs (replace with real HAL) ─────────────────── */

static void gpio_init_input(void *port, uint16_t pin) {
    /* TODO: Configure pin as GPIO input with pull-up
     * Example (HAL):
     *   GPIO_InitTypeDef gpio = {0};
     *   gpio.Pin = pin;
     *   gpio.Mode = GPIO_MODE_INPUT;
     *   gpio.Pull = GPIO_PULLUP;
     *   HAL_GPIO_Init(port, &gpio);
     */
    (void)port; (void)pin;
}

static bool gpio_read(void *port, uint16_t pin) {
    /* TODO: Replace with real HAL read
     * return HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET;
     */
    (void)port; (void)pin;
    return false;
}

/* ── Sensor Pin Table ───────────────────────────────────── */
typedef struct {
    void    *port;
    uint16_t pin;
} SensorPin;

static const SensorPin sensor_pins[SENSOR_COUNT] = {
    { (void*)IR_LEFT_PORT,         IR_LEFT_PIN         },  /* SENSOR_LEFT */
    { (void*)IR_FRONT_LEFT_PORT,   IR_FRONT_LEFT_PIN   },  /* SENSOR_FRONT_LEFT */
    { (void*)IR_FRONT_CENTER_PORT, IR_FRONT_CENTER_PIN },  /* SENSOR_FRONT_CENTER */
    { (void*)IR_FRONT_RIGHT_PORT,  IR_FRONT_RIGHT_PIN  },  /* SENSOR_FRONT_RIGHT */
    { (void*)IR_RIGHT_PORT,        IR_RIGHT_PIN        },  /* SENSOR_RIGHT */
};

/* ── Initialization ─────────────────────────────────────── */
void sensor_init(void) {
    for (int i = 0; i < SENSOR_COUNT; i++) {
        gpio_init_input(sensor_pins[i].port, sensor_pins[i].pin);
        sensor_data.wall[i] = false;
    }
    sensor_data.wall_left  = false;
    sensor_data.wall_right = false;
    sensor_data.wall_front = false;
}

/* ── Read Single Sensor with Debounce ───────────────────── */
static bool sensor_read_single(SensorPosition pos) {
    uint8_t count = 0;
    for (int i = 0; i < IR_DEBOUNCE_SAMPLES; i++) {
        bool raw = gpio_read(sensor_pins[pos].port, sensor_pins[pos].pin);
        #if IR_ACTIVE_LOW
            if (!raw) count++;   /* LOW = wall detected */
        #else
            if (raw)  count++;   /* HIGH = wall detected */
        #endif
    }
    /* Majority vote */
    return (count > (IR_DEBOUNCE_SAMPLES / 2));
}

/* ── Read All Sensors ───────────────────────────────────── */
void sensor_read_all(void) {
    for (int i = 0; i < SENSOR_COUNT; i++) {
        sensor_data.wall[i] = sensor_read_single((SensorPosition)i);
    }

    /* Convenience flags */
    sensor_data.wall_left  = sensor_data.wall[SENSOR_LEFT];
    sensor_data.wall_right = sensor_data.wall[SENSOR_RIGHT];

    /* Front wall = any front sensor triggered */
    sensor_data.wall_front = sensor_data.wall[SENSOR_FRONT_LEFT]  ||
                             sensor_data.wall[SENSOR_FRONT_CENTER] ||
                             sensor_data.wall[SENSOR_FRONT_RIGHT];
}

/* ── Convenience Getters ────────────────────────────────── */
const SensorData* sensor_get_data(void) {
    return &sensor_data;
}

bool sensor_front_wall(void) { return sensor_data.wall_front; }
bool sensor_left_wall(void)  { return sensor_data.wall_left;  }
bool sensor_right_wall(void) { return sensor_data.wall_right; }

/* ── Convert Relative Walls to Absolute ─────────────────── */
uint8_t sensor_to_absolute_walls(Direction facing) {
    uint8_t walls = 0;

    /* Map relative L/F/R to absolute N/E/S/W based on heading */
    switch (facing) {
        case DIR_NORTH:
            if (sensor_data.wall_front) walls |= WALL_NORTH;
            if (sensor_data.wall_left)  walls |= WALL_WEST;
            if (sensor_data.wall_right) walls |= WALL_EAST;
            break;
        case DIR_EAST:
            if (sensor_data.wall_front) walls |= WALL_EAST;
            if (sensor_data.wall_left)  walls |= WALL_NORTH;
            if (sensor_data.wall_right) walls |= WALL_SOUTH;
            break;
        case DIR_SOUTH:
            if (sensor_data.wall_front) walls |= WALL_SOUTH;
            if (sensor_data.wall_left)  walls |= WALL_EAST;
            if (sensor_data.wall_right) walls |= WALL_WEST;
            break;
        case DIR_WEST:
            if (sensor_data.wall_front) walls |= WALL_WEST;
            if (sensor_data.wall_left)  walls |= WALL_SOUTH;
            if (sensor_data.wall_right) walls |= WALL_NORTH;
            break;
    }
    return walls;
}
