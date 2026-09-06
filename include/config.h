/**
 * @file    config.h
 * @brief   Global configuration, pin definitions, and tuning constants.
 * @author  Sagnick Routh (adapted for digital IR + gyro-primary architecture)
 * @date    2026-09-06
 *
 * HARDWARE:
 *   - STM32F103C8T6 (Blue Pill) — 72MHz
 *   - TB6612FNG motor driver
 *   - 2x N20 gear motors with quadrature encoders
 *   - 5x digital IR obstacle avoidance modules (GPIO, HIGH/LOW)
 *   - MPU6050 IMU (I2C)
 *   - Misc: switches, LED, slide switch
 *
 * ARCHITECTURE: Gyro-primary navigation
 *   - MPU6050 gyro for heading hold (straight-line + turns)
 *   - Encoders for distance measurement
 *   - Digital IR sensors for binary wall detection (maze mapping only)
 */

#ifndef CONFIG_H
#define CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ── Maze Constants ─────────────────────────────────────── */
#define MAZE_SIZE           16
#define CELL_SIZE_MM        180
#define GOAL_MIN            7
#define GOAL_MAX            8

/* Wall bitmask encoding */
#define WALL_NORTH          0x01
#define WALL_EAST           0x02
#define WALL_SOUTH          0x04
#define WALL_WEST           0x08
#define CELL_VISITED        0x10

/* ── Direction Enum ─────────────────────────────────────── */
typedef enum {
    DIR_NORTH = 0,
    DIR_EAST  = 1,
    DIR_SOUTH = 2,
    DIR_WEST  = 3
} Direction;

/* ── Robot State ────────────────────────────────────────── */
typedef enum {
    STATE_IDLE = 0,
    STATE_CALIBRATE,
    STATE_SEARCH_RUN,
    STATE_RETURN_START,
    STATE_SPEED_RUN,
    STATE_ERROR
} RobotState;

/* ── Motor Pins (TB6612FNG) ─────────────────────────────── */
/* PWM via TIM1 */
#define MOTOR_PWMA_PORT     GPIOA
#define MOTOR_PWMA_PIN      GPIO_PIN_8       /* PA8  = TIM1_CH1 (Left) */
#define MOTOR_PWMB_PORT     GPIOA
#define MOTOR_PWMB_PIN      GPIO_PIN_11      /* PA11 = TIM1_CH4 (Right) */

/* Direction control */
#define MOTOR_AIN1_PORT     GPIOB
#define MOTOR_AIN1_PIN      GPIO_PIN_12      /* PB12 */
#define MOTOR_AIN2_PORT     GPIOB
#define MOTOR_AIN2_PIN      GPIO_PIN_13      /* PB13 */
#define MOTOR_BIN1_PORT     GPIOB
#define MOTOR_BIN1_PIN      GPIO_PIN_14      /* PB14 */
#define MOTOR_BIN2_PORT     GPIOB
#define MOTOR_BIN2_PIN      GPIO_PIN_15      /* PB15 */

/* Standby */
#define MOTOR_STBY_PORT     GPIOA
#define MOTOR_STBY_PIN      GPIO_PIN_15      /* PA15 */

/* Motor constants */
#define MOTOR_PWM_MAX       999
#define MOTOR_PWM_FREQ      20000            /* 20kHz (above audible) */

/* ── Encoder Pins (Hardware Timer Encoder Mode) ─────────── */
/* LEFT encoder: TIM2 CH1/CH2 */
#define ENC_LEFT_A_PORT     GPIOA
#define ENC_LEFT_A_PIN      GPIO_PIN_0       /* PA0 = TIM2_CH1 */
#define ENC_LEFT_B_PORT     GPIOA
#define ENC_LEFT_B_PIN      GPIO_PIN_1       /* PA1 = TIM2_CH2 */

/* RIGHT encoder: TIM3 CH1/CH2 */
#define ENC_RIGHT_A_PORT    GPIOA
#define ENC_RIGHT_A_PIN     GPIO_PIN_6       /* PA6 = TIM3_CH1 */
#define ENC_RIGHT_B_PORT    GPIOA
#define ENC_RIGHT_B_PIN     GPIO_PIN_7       /* PA7 = TIM3_CH2 */

/* Encoder physical constants */
#define ENCODER_CPR             12           /* Counts per revolution (motor shaft) */
#define ENCODER_GEAR_RATIO      100          /* Gear reduction */
#define ENCODER_TICKS_PER_REV   (ENCODER_CPR * ENCODER_GEAR_RATIO) /* 1200 */
#define WHEEL_DIAMETER_MM       25.0f
#define WHEEL_TRACK_MM          75.0f        /* Distance between wheels */
#define MM_PER_TICK             ((3.14159f * WHEEL_DIAMETER_MM) / ENCODER_TICKS_PER_REV)

/* ── IR Sensors (5x DIGITAL GPIO — NO ADC) ──────────────── */
/*
 * ARCHITECTURE NOTE:
 *   These sensors output binary HIGH/LOW only.
 *   They are used ONLY for wall-presence detection (maze mapping).
 *   Straight-line control uses the MPU6050 gyro instead.
 *
 * FC-51 modules typically output LOW when obstacle detected.
 * Set IR_ACTIVE_LOW to 1 if your modules are active-low.
 */
#define IR_ACTIVE_LOW           1            /* 1 = LOW means wall, 0 = HIGH means wall */

#define IR_LEFT_PORT            GPIOB
#define IR_LEFT_PIN             GPIO_PIN_0   /* PB0 */
#define IR_FRONT_LEFT_PORT      GPIOB
#define IR_FRONT_LEFT_PIN       GPIO_PIN_1   /* PB1 */
#define IR_FRONT_CENTER_PORT    GPIOB
#define IR_FRONT_CENTER_PIN     GPIO_PIN_3   /* PB3 */
#define IR_FRONT_RIGHT_PORT     GPIOB
#define IR_FRONT_RIGHT_PIN      GPIO_PIN_4   /* PB4 */
#define IR_RIGHT_PORT           GPIOB
#define IR_RIGHT_PIN            GPIO_PIN_5   /* PB5 */

#define IR_DEBOUNCE_SAMPLES     5            /* Majority vote for noise rejection */

/* ── MPU6050 IMU (I2C1) ─────────────────────────────────── */
#define IMU_I2C_PORT            GPIOB
#define IMU_SCL_PIN             GPIO_PIN_6   /* PB6 = I2C1_SCL */
#define IMU_SDA_PIN             GPIO_PIN_7   /* PB7 = I2C1_SDA */
#define MPU6050_ADDR            0x68         /* AD0 → GND */

/* MPU6050 registers */
#define MPU6050_WHO_AM_I        0x75
#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_GYRO_CONFIG     0x1B
#define MPU6050_ACCEL_CONFIG    0x1C
#define MPU6050_CONFIG          0x1A
#define MPU6050_ACCEL_XOUT_H    0x3B

/* Gyro configuration */
#define GYRO_RANGE_500DPS       0x08         /* ±500°/s */
#define GYRO_SENSITIVITY_500    65.5f        /* LSB/(°/s) at ±500°/s */
#define IMU_CALIBRATION_SAMPLES 500          /* Samples for bias calibration */
#define IMU_DLPF_CFG            0x03         /* ~44Hz bandwidth, 4.9ms delay */

/* ── Battery Monitoring (ADC) ───────────────────────────── */
#define BATTERY_ADC_PORT        GPIOA
#define BATTERY_ADC_PIN         GPIO_PIN_4   /* PA4 = ADC1_CH4 */
#define BATTERY_R1              10000        /* Upper resistor (ohms) */
#define BATTERY_R2              10000        /* Lower resistor (ohms) */
#define BATTERY_LOW_MV          3200         /* Low battery threshold */

/* ── UART Debug (USART1) ────────────────────────────────── */
#define UART_TX_PORT            GPIOA
#define UART_TX_PIN             GPIO_PIN_9   /* PA9 */
#define UART_RX_PORT            GPIOA
#define UART_RX_PIN             GPIO_PIN_10  /* PA10 */
#define UART_BAUD               115200

/* ── UI / Misc ──────────────────────────────────────────── */
#define LED_PORT                GPIOC
#define LED_PIN                 GPIO_PIN_13  /* PC13 (onboard) */
#define BTN_START_PORT          GPIOB
#define BTN_START_PIN           GPIO_PIN_8   /* PB8 */
#define BTN_MODE_PORT           GPIOB
#define BTN_MODE_PIN            GPIO_PIN_9   /* PB9 */

/* ── PID Tuning Constants ───────────────────────────────── */
/*
 * GYRO-PRIMARY ARCHITECTURE:
 *   - Speed PID: per-wheel speed control (encoder feedback)
 *   - Heading PID: yaw-hold for straight-line driving (gyro feedback)
 *   - Turn PID: heading-based turning (gyro feedback)
 *
 * NOTE: Wall-follow PID is REMOVED. Binary sensors cannot
 *       provide proportional error for PID.
 */

/* Per-wheel speed PID */
#define KP_SPEED                2.0f
#define KI_SPEED                0.1f
#define KD_SPEED                0.5f
#define PID_SPEED_MIN          -1000
#define PID_SPEED_MAX           1000

/* Heading-hold PID (gyro yaw error → differential motor correction) */
#define KP_HEADING              5.0f
#define KI_HEADING              0.05f
#define KD_HEADING              1.5f
#define PID_HEADING_MIN        -500
#define PID_HEADING_MAX         500

/* Turn PID (gyro yaw error for 90° turns) */
#define KP_TURN                 4.0f
#define KI_TURN                 0.0f
#define KD_TURN                 1.5f
#define PID_TURN_MIN           -600
#define PID_TURN_MAX            600

/* ── Motion Profile Constants ───────────────────────────── */
#define MAX_SPEED_MMPS          500          /* mm/s */
#define SEARCH_SPEED_MMPS       200          /* mm/s (cautious search) */
#define TURN_SPEED_MMPS         150          /* mm/s */
#define ACCEL_MMPS2             1000         /* mm/s² */
#define DECEL_MMPS2             1000         /* mm/s² */

/* Turn parameters */
#define TURN_ANGLE_90           90.0f        /* degrees */
#define TURN_ANGLE_180          180.0f
#define TURN_DEADBAND_DEG       2.0f         /* acceptable error for turn completion */
#define TURN_TIMEOUT_MS         3000         /* max time for a turn */

/* ── Control Loop ───────────────────────────────────────── */
#define CONTROL_FREQ_HZ         1000
#define CONTROL_DT              0.001f       /* 1ms */

/* ── Robot Pose ─────────────────────────────────────────── */
typedef struct {
    uint8_t   x;              /* Cell column (0-15) */
    uint8_t   y;              /* Cell row (0-15) */
    Direction facing;         /* Current heading */
    float     yaw;            /* Current gyro yaw angle (degrees) */
    float     target_yaw;     /* Target yaw for heading hold */
} RobotPose;

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_H */
