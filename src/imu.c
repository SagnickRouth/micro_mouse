#include "imu.h"
#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;
ImuData imu_data;
static uint8_t imu_addr = IMU_I2C_ADDR;

static bool wr(uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(&hi2c1, imu_addr, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 100) == HAL_OK;
}
static bool rd(uint8_t reg, uint8_t *buf, uint16_t n)
{
    return HAL_I2C_Mem_Read(&hi2c1, imu_addr, reg, I2C_MEMADD_SIZE_8BIT, buf, n, 100) == HAL_OK;
}

bool imu_init(void)
{
    uint8_t who = 0;
    if (!rd(IMU_WHO_AM_I, &who, 1)) return false;
    if (who != MPU6500_WHO_AM_I_VALUE && who != MPU9250_WHO_AM_I_VALUE && who != MPU9255_WHO_AM_I_VALUE)
        return false;

    imu_data.who_am_i = who;
    if (!wr(IMU_PWR_MGMT_1, 0x00)) return false;
    HAL_Delay(100);
    wr(IMU_CONFIG, IMU_DLPF_CFG);
    wr(IMU_GYRO_CONFIG, GYRO_RANGE_500DPS);
    wr(IMU_ACCEL_CONFIG, 0x00); /* ±2 g */
    imu_data.yaw_angle = 0.0f;
    imu_data.gyro_z_bias = 0.0f;
    return true;
}

void imu_read(void)
{
    uint8_t b[14];
    if (!rd(IMU_ACCEL_XOUT_H, b, 14)) return;
    imu_data.accel_x = (int16_t)((b[0]<<8)|b[1]);
    imu_data.accel_y = (int16_t)((b[2]<<8)|b[3]);
    imu_data.accel_z = (int16_t)((b[4]<<8)|b[5]);
    imu_data.temperature = (int16_t)((b[6]<<8)|b[7]);
    imu_data.gyro_x = (int16_t)((b[8]<<8)|b[9]);
    imu_data.gyro_y = (int16_t)((b[10]<<8)|b[11]);
    imu_data.gyro_z = (int16_t)((b[12]<<8)|b[13]);
    imu_data.yaw_rate = ((float)imu_data.gyro_z - imu_data.gyro_z_bias) / GYRO_SENSITIVITY_500;
    imu_data.yaw_angle += imu_data.yaw_rate * CONTROL_DT;
}

void imu_calibrate(void)
{
    float sum = 0.0f;
    uint8_t b[2];
    for (uint32_t i=0; i<IMU_CALIBRATION_SAMPLES; ++i) {
        if (rd(IMU_GYRO_ZOUT_H,b,2)) sum += (float)(int16_t)((b[0]<<8)|b[1]);
        HAL_Delay(1);
    }
    imu_data.gyro_z_bias = sum / (float)IMU_CALIBRATION_SAMPLES;
    imu_data.yaw_angle = 0.0f;
}
float imu_get_yaw(void){ return imu_data.yaw_angle; }
void imu_reset_yaw(void){ imu_data.yaw_angle=0.0f; }
void imu_set_yaw(float y){ imu_data.yaw_angle=y; }
const ImuData* imu_get_data(void){ return &imu_data; }
