/**
 * @file main.c
 * @brief Micromouse entry point and run control.
 *
 * STM32F401CCU6 Black Pill UI:
 *   PC13 = onboard KEY/USER button, active-low, START/STOP
 *   PB2 = DIP1, PB3 = DIP2, PB4 = DIP3, PB5 = DIP4
 *   PB8 = I2C1 SCL, PB9 = I2C1 SDA for SSD1306 OLED
 *
 * DIP algorithm selection:
 *   SW1 -> Flood Fill
 *   SW2 -> Left Wall
 *   SW3 -> Right Wall
 *   SW4 -> A*
 */

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "config.h"
#include "motor.h"
#include "encoder.h"
#include "sensor.h"
#include "imu.h"
#include "pid.h"
#include "maze.h"
#include "motion.h"
#include "battery.h"
#include "algorithm.h"
#include "oled.h"

typedef enum {
    STATE_IDLE,
    STATE_CALIBRATION,
    STATE_SEARCH_RUN,
    STATE_RETURN_TO_START,
    STATE_SPEED_RUN,
    STATE_ERROR,
    STATE_FINISHED
} RobotState;

static volatile RobotState robot_state = STATE_IDLE;
static Pose robot_pose;
static SensorData sensor_data;
static ImuData imu_data;
static PidController pid_left_speed, pid_right_speed, pid_wall;
static AlgorithmConfig selected_algorithm;
static bool run_requested = false;

static void system_init(void);
static void ui_gpio_init(void);
static void state_calibration(void);
static void state_search_run(void);
static void state_return_to_start(void);
static void state_speed_run(void);
static void state_error(void);
static bool button_pressed(void);
static void stop_run(void);
static void delay_ms(uint32_t ms);

int main(void)
{
    system_init();

    while (1) {
        /* The onboard KEY is the single start/stop control. */
        if (button_pressed()) {
            if (!run_requested && robot_state == STATE_IDLE) {
                selected_algorithm = algorithm_read_switches();
                run_requested = true;
                oled_show_algorithm(selected_algorithm.alg_name, false);
                robot_state = STATE_CALIBRATION;
            } else if (run_requested && robot_state != STATE_ERROR) {
                stop_run();
            }
        }

        if (battery_is_low() && robot_state != STATE_IDLE) {
            stop_run();
            robot_state = STATE_ERROR;
        }

        switch (robot_state) {
            case STATE_IDLE: {
                /* Read the DIP switches while stopped so the OLED follows them. */
                static AlgorithmType shown_alg = 0xFF;
                AlgorithmConfig cfg = algorithm_read_switches();
                if (cfg.algorithm != shown_alg) {
                    shown_alg = cfg.algorithm;
                    oled_show_algorithm(cfg.alg_name, false);
                }
                delay_ms(20);
                break;
            }
            case STATE_CALIBRATION: state_calibration(); break;
            case STATE_SEARCH_RUN: state_search_run(); break;
            case STATE_RETURN_TO_START: state_return_to_start(); break;
            case STATE_SPEED_RUN: state_speed_run(); break;
            case STATE_FINISHED:
                motor_disable();
                run_requested = false;
                oled_show_message("FINISHED", selected_algorithm.alg_name);
                robot_state = STATE_IDLE;
                delay_ms(500);
                break;
            case STATE_ERROR: state_error(); break;
        }
    }
}

static void system_init(void)
{
    HAL_Init();
    SystemClock_Config();
    ui_gpio_init();
    oled_init();

    motor_init();
    encoder_init();
    sensor_init();
    battery_init();
    motion_init();
    maze_init();

    pid_init(&pid_left_speed, KP_SPEED, KI_SPEED, KD_SPEED, PID_OUTPUT_MIN, PID_OUTPUT_MAX);
    pid_init(&pid_right_speed, KP_SPEED, KI_SPEED, KD_SPEED, PID_OUTPUT_MIN, PID_OUTPUT_MAX);
    pid_init(&pid_wall, KP_WALL, KI_WALL, KD_WALL, PID_OUTPUT_MIN, PID_OUTPUT_MAX);

    (void)imu_init();

    robot_pose.x = 0;
    robot_pose.y = 0;
    robot_pose.dir = DIR_NORTH;
    selected_algorithm = algorithm_read_switches();
    oled_show_algorithm(selected_algorithm.alg_name, false);
}

static void ui_gpio_init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};

    /* Four algorithm DIP switches, active-low. */
    g.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &g);

    /* STM32F401 Black Pill onboard KEY/USER button, active-low. */
    g.Pin = GPIO_PIN_13;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &g);
}

static void state_calibration(void)
{
    oled_show_message("CALIBRATING", selected_algorithm.alg_name);
    sensor_calibrate();
    imu_calibrate();
    encoder_reset();
    imu_reset_yaw();
    pid_reset(&pid_left_speed);
    pid_reset(&pid_right_speed);
    pid_reset(&pid_wall);
    delay_ms(500);
    motor_enable();
    oled_show_algorithm(selected_algorithm.alg_name, true);
    robot_state = STATE_SEARCH_RUN;
}

static void state_search_run(void)
{
    sensor_read_all(&sensor_data);
    imu_read(&imu_data);

    bool front = sensor_front_wall(&sensor_data);
    bool left  = sensor_left_wall(&sensor_data);
    bool right = sensor_right_wall(&sensor_data);

    maze_update_walls(robot_pose.x, robot_pose.y, robot_pose.dir,
                      front, left, right);
    maze_mark_visited(robot_pose.x, robot_pose.y);

    if (maze_is_goal(robot_pose.x, robot_pose.y)) {
        motion_stop();
        robot_state = STATE_RETURN_TO_START;
        return;
    }

    Direction next_dir = algorithm_next_direction(
        robot_pose.x, robot_pose.y, robot_pose.dir,
        left, front, right);

    motion_execute_direction(&robot_pose, next_dir);
}

static void state_return_to_start(void)
{
    if (!run_requested) return;

    maze_flood_fill_to(0, 0);
    sensor_read_all(&sensor_data);
    imu_read(&imu_data);

    bool front = sensor_front_wall(&sensor_data);
    bool left  = sensor_left_wall(&sensor_data);
    bool right = sensor_right_wall(&sensor_data);

    maze_update_walls(robot_pose.x, robot_pose.y, robot_pose.dir,
                      front, left, right);

    if (robot_pose.x == 0 && robot_pose.y == 0) {
        motion_stop();
        /* One KEY press starts the complete run; do not require another press. */
        robot_state = STATE_SPEED_RUN;
        oled_show_algorithm(selected_algorithm.alg_name, true);
        return;
    }

    Direction next_dir = maze_best_direction(robot_pose.x, robot_pose.y, robot_pose.dir);
    motion_execute_direction(&robot_pose, next_dir);
}

static void state_speed_run(void)
{
    if (!run_requested) return;

    sensor_read_all(&sensor_data);
    imu_read(&imu_data);

    if (maze_is_goal(robot_pose.x, robot_pose.y)) {
        motion_stop();
        robot_state = STATE_FINISHED;
        return;
    }

    Direction next_dir = algorithm_next_direction(
        robot_pose.x, robot_pose.y, robot_pose.dir,
        sensor_left_wall(&sensor_data),
        sensor_front_wall(&sensor_data),
        sensor_right_wall(&sensor_data));

    motion_execute_direction(&robot_pose, next_dir);
}

static void stop_run(void)
{
    run_requested = false;
    motion_stop();
    motor_disable();
    robot_state = STATE_IDLE;
    oled_show_algorithm(selected_algorithm.alg_name, false);
}

static void state_error(void)
{
    motor_disable();
    oled_show_message("ERROR", "LOW BATTERY");
    while (1) {
        delay_ms(500);
    }
}

static bool button_pressed(void)
{
    static GPIO_PinState last_raw = GPIO_PIN_SET;
    static GPIO_PinState stable = GPIO_PIN_SET;
    static uint32_t changed_at = 0;
    GPIO_PinState raw = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
    uint32_t now = HAL_GetTick();

    if (raw != last_raw) {
        last_raw = raw;
        changed_at = now;
    }

    if ((now - changed_at) >= 30U && raw != stable) {
        GPIO_PinState old = stable;
        stable = raw;
        return (old == GPIO_PIN_SET && stable == GPIO_PIN_RESET);
    }
    return false;
}

static void delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}
