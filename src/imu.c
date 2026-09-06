/**
 * @file    imu.c
 * @brief   MPU6050 IMU driver over I2C — yaw integration with bias calibration.
 * @author  Sagnick Routh
 * @date    2026-09-06
 *
 * PRIMARY NAVIGATION SENSOR in the gyro-primary architecture.
 * Provides continuous yaw angle for heading-hold PID and turn control.
 *
 * Key features:
 *   - Startup bias calibration (mandatory, robot stationary)
 *   - Digital low-pass filter (DLPF) enabled to reduce vibration noise
 *   - yaw_set() for front-wall squaring drift reset
 */

#include "imu.h"
#include "config.h"

/* ── Global IMU Data ────────────────────────────────────── */
ImuData imu_data;

/* ── I2C Stubs (replace with real HAL) ──────────────────── */

static bool i2c_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t value) {
    /* TODO: Implement real I2C write
     * HAL_I2C_Mem_Write(&hi2c1, dev_addr << 1, reg,
     *                   I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
     */
    (void)dev_addr; (void)reg; (void)value;
    return true;
}

static bool i2c_read_reg(uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint16_t len) {
    /* TODO: Implement real I2C read
     * HAL_I2C_Mem_Read(&hi2c1, dev_addr << 1, reg,
     *                  I2C_MEMADD_SIZE_8BIT, buf, len, 100);
     */
    (void)dev_addr; (void)reg; (void)buf; (void)len;
    return true;
}

static void delay_ms(uint32_t ms) {
    /* TODO: Replace with HAL_Delay(ms) or SysTick-based delay */
    for (volatile uint32_t i = 0; i < ms * 7200; i++) { __asm("nop"); }
}

/* ── Initialization ─────────────────────────────────────── */
bool imu_init(void) {
    /* Check WHO_AM_I */
    uint8_t who = 0;
    i2c_read_reg(MPU6050_ADDR, MPU6050_WHO_AM_I, &who, 1);
    if (who != 0x68 && who != 0x72) {
        return false;  /* MPU6050 not found */
    }

    /* Wake up (clear SLEEP bit) */
    i2c_write_reg(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x00);
    delay_ms(100);

    /* Set gyro range: ±500°/s */
    i2c_write_reg(MPU6050_ADDR, MPU6050_GYRO_CONFIG, GYRO_RANGE_500DPS);

    /* Set accel range: ±2g */
    i2c_write_reg(MPU6050_ADDR, MPU6050_ACCEL_CONFIG, 0x00);

    /* Enable DLPF: ~44Hz bandwidth, reduces motor vibration noise */
    i2c_write_reg(MPU6050_ADDR, MPU6050_CONFIG, IMU_DLPF_CFG);

    /* Initialize data */
    imu_data.yaw_angle  = 0.0f;
    imu_data.yaw_rate   = 0.0f;
    imu_data.gyro_z_bias = 0.0f;

    return true;
}

/* ── Read All Axes + Integrate Yaw ──────────────────────── */
void imu_read(void) {
    uint8_t buf[14];
    i2c_read_reg(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, buf, 14);

    /* Parse 14-byte block: accel(6) + temp(2) + gyro(6) */
    imu_data.accel_x = (int16_t)((buf[0]  << 8) | buf[1]);
    imu_data.accel_y = (int16_t)((buf[2]  << 8) | buf[3]);
    imu_data.accel_z = (int16_t)((buf[4]  << 8) | buf[5]);
    imu_data.temperature = (int16_t)((buf[6] << 8) | buf[7]);
    imu_data.gyro_x  = (int16_t)((buf[8]  << 8) | buf[9]);
    imu_data.gyro_y  = (int16_t)((buf[10] << 8) | buf[11]);
    imu_data.gyro_z  = (int16_t)((buf[12] << 8) | buf[13]);

    /* Compute yaw rate (°/s) with bias correction */
    imu_data.yaw_rate = ((float)imu_data.gyro_z - imu_data.gyro_z_bias)
                        / GYRO_SENSITIVITY_500;

    /* Integrate yaw angle */
    imu_data.yaw_angle += imu_data.yaw_rate * CONTROL_DT;
}

/* ── Calibrate Gyro Bias ────────────────────────────────── */
void imu_calibrate(void) {
    float sum = 0.0f;

    for (int i = 0; i < IMU_CALIBRATION_SAMPLES; i++) {
        uint8_t buf[2];
        i2c_read_reg(MPU6050_ADDR, 0x47, buf, 2);  /* GYRO_ZOUT_H/L */
        int16_t raw_z = (int16_t)((buf[0] << 8) | buf[1]);
        sum += (float)raw_z;
        delay_ms(1);
    }

    imu_data.gyro_z_bias = sum / (float)IMU_CALIBRATION_SAMPLES;
    imu_data.yaw_angle = 0.0f;
}

/* ── Getters / Setters ──────────────────────────────────── */
float imu_get_yaw(void)           { return imu_data.yaw_angle;   }
void  imu_reset_yaw(void)         { imu_data.yaw_angle = 0.0f;   }
void  imu_set_yaw(float yaw_deg)  { imu_data.yaw_angle = yaw_deg; }
const ImuData* imu_get_data(void)  { return &imu_data;             }
