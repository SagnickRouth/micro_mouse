#include "stm32f4xx_hal.h"
#include <stdbool.h>
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

RobotPose robot_pose;
SensorData sensor_data;
ImuData imu_data;

static PidController pid_left_speed;
static PidController pid_right_speed;
static PidController pid_wall;
static AlgorithmConfig selected_algorithm;
static bool run_requested = false;

typedef enum { STATE_IDLE, STATE_CALIBRATION, STATE_SEARCH_RUN,
               STATE_RETURN_TO_START, STATE_SPEED_RUN, STATE_ERROR,
               STATE_FINISHED } RobotState;
static RobotState robot_state = STATE_IDLE;

void SystemClock_Config(void);
static void ui_gpio_init(void);
static bool key_pressed(void);
static void stop_run(void);
static void state_calibration(void);
static void state_search_run(void);
static void state_return_to_start(void);
static void state_speed_run(void);
static void state_error(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    ui_gpio_init();
    oled_init();
    motor_init();
    encoder_init();
    sensor_init();
    battery_init();
    maze_init();
    motion_init();
    (void)imu_init();

    pid_init(&pid_left_speed, KP_SPEED, KI_SPEED, KD_SPEED, PID_SPEED_MIN, PID_SPEED_MAX);
    pid_init(&pid_right_speed, KP_SPEED, KI_SPEED, KD_SPEED, PID_SPEED_MIN, PID_SPEED_MAX);
    pid_init(&pid_wall, KP_HEADING, KI_HEADING, KD_HEADING, PID_HEADING_MIN, PID_HEADING_MAX);

    robot_pose.x = 0;
    robot_pose.y = 0;
    robot_pose.facing = DIR_NORTH;
    robot_pose.yaw = 0.0f;
    robot_pose.target_yaw = 0.0f;

    selected_algorithm = algorithm_read_switches();
    oled_show_algorithm(selected_algorithm.alg_name, false);

    while (1) {
        if (key_pressed()) {
            if (!run_requested && robot_state == STATE_IDLE) {
                selected_algorithm = algorithm_read_switches();
                run_requested = true;
                robot_state = STATE_CALIBRATION;
            } else if (run_requested) {
                stop_run();
            }
        }

        if (battery_is_low() && robot_state != STATE_IDLE) {
            stop_run();
            robot_state = STATE_ERROR;
        }

        switch (robot_state) {
        case STATE_IDLE:
            selected_algorithm = algorithm_read_switches();
            oled_show_algorithm(selected_algorithm.alg_name, false);
            HAL_Delay(50);
            break;
        case STATE_CALIBRATION: state_calibration(); break;
        case STATE_SEARCH_RUN: state_search_run(); break;
        case STATE_RETURN_TO_START: state_return_to_start(); break;
        case STATE_SPEED_RUN: state_speed_run(); break;
        case STATE_FINISHED:
            motor_disable(); run_requested = false;
            oled_show_message("FINISHED", selected_algorithm.alg_name);
            robot_state = STATE_IDLE;
            break;
        case STATE_ERROR: state_error(); break;
        }
    }
}

static void ui_gpio_init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};

    /* 2-bit algorithm selector, active-low. */
    g.Pin = DIP_ALG0_PIN | DIP_ALG1_PIN;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &g);

    /* Black Pill KEY/USER button, active-low. */
    g.Pin = KEY_PIN;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(KEY_PORT, &g);
}

static bool key_pressed(void)
{
    static GPIO_PinState last = GPIO_PIN_SET;
    static uint32_t changed = 0;
    GPIO_PinState now = HAL_GPIO_ReadPin(KEY_PORT, KEY_PIN);
    uint32_t t = HAL_GetTick();
    if (now != last) { last = now; changed = t; }
    if (now == GPIO_PIN_RESET && (t - changed) >= 30U) {
        while (HAL_GPIO_ReadPin(KEY_PORT, KEY_PIN) == GPIO_PIN_RESET) HAL_Delay(5);
        return true;
    }
    return false;
}

static void state_calibration(void)
{
    oled_show_message("CALIBRATING", selected_algorithm.alg_name);
    motor_disable();
    sensor_init();
    imu_calibrate();
    encoder_reset();
    imu_reset_yaw();
    pid_reset(&pid_left_speed);
    pid_reset(&pid_right_speed);
    pid_reset(&pid_wall);
    maze_init();
    HAL_Delay(300);
    motor_enable();
    oled_show_algorithm(selected_algorithm.alg_name, true);
    robot_state = STATE_SEARCH_RUN;
}

static void state_search_run(void)
{
    sensor_read_all();
    imu_read();

    maze_update_walls(robot_pose.x, robot_pose.y, robot_pose.facing,
                      sensor_to_absolute_walls(robot_pose.facing));
    maze_mark_visited(robot_pose.x, robot_pose.y);

    if (maze_is_goal(robot_pose.x, robot_pose.y)) {
        motion_stop();
        robot_state = STATE_RETURN_TO_START;
        return;
    }

    Direction next = algorithm_next_direction(robot_pose.x, robot_pose.y,
                                               robot_pose.facing,
                                               sensor_left_wall(),
                                               sensor_front_wall(),
                                               sensor_right_wall());
    motion_execute_direction(next);
}

static void state_return_to_start(void)
{
    maze_flood_fill_to(0, 0);
    if (robot_pose.x == 0 && robot_pose.y == 0) {
        motion_stop();
        oled_show_message("RETURNED", "SPEED RUN");
        HAL_Delay(500);
        robot_state = STATE_SPEED_RUN;
        return;
    }
    Direction next = maze_best_direction(robot_pose.x, robot_pose.y, robot_pose.facing);
    motion_execute_direction(next);
}

static void state_speed_run(void)
{
    if (maze_is_goal(robot_pose.x, robot_pose.y)) {
        motion_stop();
        robot_state = STATE_FINISHED;
        return;
    }

    sensor_read_all();
    imu_read();
    Direction next = algorithm_next_direction(robot_pose.x, robot_pose.y,
                                               robot_pose.facing,
                                               sensor_left_wall(),
                                               sensor_front_wall(),
                                               sensor_right_wall());
    motion_execute_direction(next);
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
    while (1) HAL_Delay(500);
}
