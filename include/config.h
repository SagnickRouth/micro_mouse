#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

/* ===================== Micromouse ===================== */
#define MAZE_SIZE        16
#define CELL_SIZE_MM     180
#define GOAL_MIN         7
#define GOAL_MAX         8
#define WALL_NORTH       0x01
#define WALL_EAST        0x02
#define WALL_SOUTH       0x04
#define WALL_WEST        0x08
#define CELL_VISITED     0x10

typedef enum { DIR_NORTH=0, DIR_EAST=1, DIR_SOUTH=2, DIR_WEST=3 } Direction;

typedef struct {
    uint8_t x, y;
    Direction facing;
    float yaw;
    float target_yaw;
} RobotPose;

/* ===================== TB6612FNG ===================== */
/* TIM3 CH1/CH2 PWM */
#define MOTOR_PWM_TIMER       TIM3
#define MOTOR_PWMA_PORT       GPIOA
#define MOTOR_PWMA_PIN        GPIO_PIN_6       /* PA6 TIM3_CH1 */
#define MOTOR_PWMB_PORT       GPIOA
#define MOTOR_PWMB_PIN        GPIO_PIN_7       /* PA7 TIM3_CH2 */
#define MOTOR_AIN1_PORT       GPIOB
#define MOTOR_AIN1_PIN        GPIO_PIN_12
#define MOTOR_AIN2_PORT       GPIOB
#define MOTOR_AIN2_PIN        GPIO_PIN_13
#define MOTOR_BIN1_PORT       GPIOB
#define MOTOR_BIN1_PIN        GPIO_PIN_14
#define MOTOR_BIN2_PORT       GPIOB
#define MOTOR_BIN2_PIN        GPIO_PIN_15
#define MOTOR_STBY_PORT       GPIOC
#define MOTOR_STBY_PIN        GPIO_PIN_0
#define MOTOR_PWM_MAX         999
#define MOTOR_PWM_FREQ        20000

/* ===================== N20 Encoders ===================== */
/* Left: TIM2 CH1/CH2 */
#define ENC_LEFT_TIMER        TIM2
#define ENC_LEFT_A_PORT       GPIOA
#define ENC_LEFT_A_PIN        GPIO_PIN_0
#define ENC_LEFT_B_PORT       GPIOA
#define ENC_LEFT_B_PIN        GPIO_PIN_1
/* Right: TIM4 CH1/CH2 */
#define ENC_RIGHT_TIMER       TIM4
#define ENC_RIGHT_A_PORT      GPIOB
#define ENC_RIGHT_A_PIN       GPIO_PIN_6
#define ENC_RIGHT_B_PORT      GPIOB
#define ENC_RIGHT_B_PIN       GPIO_PIN_7
#define ENCODER_CPR           12
#define ENCODER_GEAR_RATIO    100
#define ENCODER_TICKS_PER_REV (ENCODER_CPR * ENCODER_GEAR_RATIO)
#define WHEEL_DIAMETER_MM     25.0f
#define WHEEL_TRACK_MM        75.0f
#define MM_PER_TICK           ((3.14159265f * WHEEL_DIAMETER_MM) / ENCODER_TICKS_PER_REV)

/* ===================== I2C BUS ===================== */
/* OLED + MPU9250/MPU6500 share I2C1 */
#define I2C1_SCL_PORT         GPIOB
#define I2C1_SCL_PIN          GPIO_PIN_8
#define I2C1_SDA_PORT         GPIOB
#define I2C1_SDA_PIN          GPIO_PIN_9
#define I2C_SPEED_HZ          400000U
#define OLED_I2C_ADDR         (0x3C << 1)
#define IMU_I2C_ADDR          (0x68 << 1)

/* ===================== VL53L0X x4 ===================== */
/* Four sensors share SDA/SCL. XSHUT gives each a unique address. */
#define VL53_COUNT            4
#define VL53_LEFT_XSHUT_PORT  GPIOA
#define VL53_LEFT_XSHUT_PIN   GPIO_PIN_8
#define VL53_FL_XSHUT_PORT    GPIOA
#define VL53_FL_XSHUT_PIN     GPIO_PIN_9
#define VL53_FR_XSHUT_PORT    GPIOA
#define VL53_FR_XSHUT_PIN     GPIO_PIN_10
#define VL53_RIGHT_XSHUT_PORT GPIOA
#define VL53_RIGHT_XSHUT_PIN  GPIO_PIN_15
#define VL53_ADDR_LEFT        0x30
#define VL53_ADDR_FL          0x31
#define VL53_ADDR_FR          0x32
#define VL53_ADDR_RIGHT       0x33
#define VL53_DEFAULT_ADDR     0x29

typedef enum {
    SENSOR_LEFT = 0,
    SENSOR_FRONT_LEFT = 1,
    SENSOR_FRONT_RIGHT = 2,
    SENSOR_RIGHT = 3,
    SENSOR_COUNT = 4
} SensorPosition;

#define VL53_WALL_THRESHOLD_MM  100U
#define VL53_FRONT_THRESHOLD_MM 100U

/* ===================== MPU9250 / MPU6500 ===================== */
#define IMU_WHO_AM_I            0x75
#define IMU_PWR_MGMT_1          0x6B
#define IMU_PWR_MGMT_2          0x6C
#define IMU_CONFIG               0x1A
#define IMU_GYRO_CONFIG          0x1B
#define IMU_ACCEL_CONFIG         0x1C
#define IMU_ACCEL_XOUT_H         0x3B
#define IMU_GYRO_ZOUT_H          0x47
#define MPU6500_WHO_AM_I_VALUE   0x70
#define MPU9250_WHO_AM_I_VALUE   0x71
#define MPU9255_WHO_AM_I_VALUE   0x73
#define GYRO_RANGE_500DPS        0x08
#define GYRO_SENSITIVITY_500     65.5f
#define IMU_DLPF_CFG              0x03
#define IMU_CALIBRATION_SAMPLES   500
#define CONTROL_DT                0.001f

/* ===================== KEY + 2 DIP ===================== */
#define KEY_PORT                 GPIOC
#define KEY_PIN                  GPIO_PIN_13
#define DIP_ALG0_PORT            GPIOB
#define DIP_ALG0_PIN             GPIO_PIN_2
#define DIP_ALG1_PORT            GPIOB
#define DIP_ALG1_PIN             GPIO_PIN_3
/* OFF/OFF=Flood Fill, ON/OFF=Left Wall, OFF/ON=Right Wall, ON/ON=A* */

/* ===================== Motion/PID ===================== */
#define KP_SPEED                 2.0f
#define KI_SPEED                 0.1f
#define KD_SPEED                 0.5f
#define PID_SPEED_MIN           -1000
#define PID_SPEED_MAX            1000
#define KP_HEADING               5.0f
#define KI_HEADING               0.05f
#define KD_HEADING               1.5f
#define PID_HEADING_MIN         -500
#define PID_HEADING_MAX          500
#define KP_TURN                  4.0f
#define KI_TURN                  0.0f
#define KD_TURN                  1.5f
#define PID_TURN_MIN            -600
#define PID_TURN_MAX             600
#define MAX_SPEED_MMPS           500
#define SEARCH_SPEED_MMPS        200
#define TURN_SPEED_MMPS          150
#define ACCEL_MMPS2              1000
#define DECEL_MMPS2              1000
#define TURN_ANGLE_90            90.0f
#define TURN_ANGLE_180           180.0f
#define TURN_DEADBAND_DEG        2.0f
#define TURN_TIMEOUT_MS          3000

/* Onboard LED */
#define LED_PORT                 GPIOC
#define LED_PIN                  GPIO_PIN_13

#endif
