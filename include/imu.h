/**
 * @file    imu.h
 * @brief   MPU6050 IMU driver (I2C) — gyro yaw integration.
 *
 * Provides yaw angle for heading-hold and turn control.
 * This is the PRIMARY navigation sensor in the gyro-primary architecture.
 */

#ifndef IMU_H
#define IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/** IMU data structure */
typedef struct {
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x,  gyro_y,  gyro_z;
    float   yaw_rate;     /* Degrees/second (bias-corrected) */
    float   yaw_angle;    /* Integrated yaw (degrees) */
    float   gyro_z_bias;  /* Calibrated bias */
    int16_t temperature;
} ImuData;

/** Initialize MPU6050 over I2C. Returns true on success. */
bool imu_init(void);

/** Read accelerometer + gyroscope data, integrate yaw. */
void imu_read(void);

/** Calibrate gyro bias (robot must be stationary). */
void imu_calibrate(void);

/** Get current integrated yaw angle (degrees). */
float imu_get_yaw(void);

/** Reset yaw angle to zero. */
void imu_reset_yaw(void);

/** Set yaw to a specific value (for front-wall squaring). */
void imu_set_yaw(float yaw_deg);

/** Get raw IMU data. */
const ImuData* imu_get_data(void);

/* Global IMU data */
extern ImuData imu_data;

#ifdef __cplusplus
}
#endif

#endif /* IMU_H */
