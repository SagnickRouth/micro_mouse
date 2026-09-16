#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>
#include "config.h"

typedef struct {
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
    float yaw_rate;
    float yaw_angle;
    float gyro_z_bias;
    int16_t temperature;
    uint8_t who_am_i;
} ImuData;

bool imu_init(void);
void imu_read(void);
void imu_calibrate(void);
float imu_get_yaw(void);
void imu_reset_yaw(void);
void imu_set_yaw(float yaw_deg);
const ImuData *imu_get_data(void);
extern ImuData imu_data;

#endif
