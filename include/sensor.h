/**
 * @file    sensor.h
 * @brief   Digital IR wall sensor driver (GPIO, binary output).
 *
 * ARCHITECTURE: Sensors provide ONLY wall presence (true/false).
 * No analog distance data. Navigation uses gyro + encoders instead.
 */

#ifndef SENSOR_H
#define SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/** Sensor positions */
typedef enum {
    SENSOR_LEFT         = 0,
    SENSOR_FRONT_LEFT   = 1,
    SENSOR_FRONT_CENTER = 2,
    SENSOR_FRONT_RIGHT  = 3,
    SENSOR_RIGHT        = 4,
    SENSOR_COUNT        = 5
} SensorPosition;

/** Sensor data (all binary) */
typedef struct {
    bool wall[SENSOR_COUNT];     /* true = wall detected */
    bool wall_left;              /* Convenience: left wall */
    bool wall_right;             /* Convenience: right wall */
    bool wall_front;             /* Convenience: front wall (any front sensor) */
} SensorData;

/** Initialize GPIO pins for all 5 IR sensor modules. */
void sensor_init(void);

/** Read all 5 sensors with debouncing. Updates global sensor_data. */
void sensor_read_all(void);

/** Get current sensor data (last read). */
const SensorData* sensor_get_data(void);

/** Individual wall queries (use after sensor_read_all). */
bool sensor_front_wall(void);
bool sensor_left_wall(void);
bool sensor_right_wall(void);

/** Convert relative wall flags (L/F/R) to absolute walls based on heading. */
uint8_t sensor_to_absolute_walls(Direction facing);

/* Global sensor data */
extern SensorData sensor_data;

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_H */
