#ifndef SENSOR_H
#define SENSOR_H

#include <stdbool.h>
#include <stdint.h>
#include "config.h"

/* Wall distances in millimetres. */
typedef struct {
    uint16_t distance_mm[SENSOR_COUNT];
    bool wall_left;
    bool wall_front_left;
    bool wall_front_right;
    bool wall_right;
    bool wall_front;
} SensorData;

bool sensor_init(void);
bool sensor_read_all(void);
const SensorData *sensor_get_data(void);
bool sensor_front_wall(void);
bool sensor_left_wall(void);
bool sensor_right_wall(void);
uint8_t sensor_to_absolute_walls(Direction facing);

/* VL53L0X address setup. Call after power-up with all XSHUT low. */
bool sensor_assign_addresses(void);

extern SensorData sensor_data;

#endif
