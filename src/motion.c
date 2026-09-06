/**
 * @file    motion.c
 * @brief   Motion profiling and navigation — gyro-primary architecture.
 * @author  Sagnick Routh
 * @date    2026-09-06
 *
 * GYRO-PRIMARY CONTROL:
 *   Straight-line driving uses MPU6050 yaw as the primary error source
 *   for heading hold. Binary IR sensors cannot provide proportional
 *   wall-distance feedback, so the gyro replaces wall-follow PID.
 *
 *   Turns use gyro yaw target: rotate until yaw matches target ± deadband.
 *
 *   Front-wall squaring re-zeros the gyro to correct accumulated drift.
 */

#include "motion.h"
#include "motor.h"
#include "encoder.h"
#include "imu.h"
#include "sensor.h"
#include "pid.h"
#include "config.h"

/* ── PID Instances ──────────────────────────────────────── */
static PID pid_speed_l;       /* Left wheel speed PID */
static PID pid_speed_r;       /* Right wheel speed PID */
static PID pid_heading;       /* Heading-hold PID (gyro yaw) */
static PID pid_turn;          /* Turn PID (gyro yaw for in-place turns) */

/* ── Motion State ───────────────────────────────────────── */
static bool     motion_complete;
static float    current_speed;
static float    target_distance;

/* ── Global Robot Pose ──────────────────────────────────── */
extern RobotPose robot_pose;

/* ── Initialization ─────────────────────────────────────── */
void motion_init(void) {
    pid_init(&pid_speed_l, KP_SPEED, KI_SPEED, KD_SPEED,
             PID_SPEED_MIN, PID_SPEED_MAX);
    pid_init(&pid_speed_r, KP_SPEED, KI_SPEED, KD_SPEED,
             PID_SPEED_MIN, PID_SPEED_MAX);
    pid_init(&pid_heading, KP_HEADING, KI_HEADING, KD_HEADING,
             PID_HEADING_MIN, PID_HEADING_MAX);
    pid_init(&pid_turn, KP_TURN, KI_TURN, KD_TURN,
             PID_TURN_MIN, PID_TURN_MAX);

    motion_complete = true;
    current_speed   = 0.0f;
    target_distance = 0.0f;
}

/* ── Straight-Line Move (Gyro + Encoder) ────────────────── */
void motion_move(float distance_mm, float end_speed) {
    encoder_reset();
    pid_reset(&pid_heading);
    pid_reset(&pid_speed_l);
    pid_reset(&pid_speed_r);
    motion_complete = false;
    target_distance = distance_mm;
    current_speed   = 0.0f;

    /* Set heading target to current yaw (hold this heading) */
    float heading_target = imu_get_yaw();

    while (!motion_complete) {
        /* Read sensors */
        encoder_update();
        imu_read();

        /* Distance covered (average of both wheels) */
        float dist_l = encoder_ticks_to_mm(encoder_get_left_count());
        float dist_r = encoder_ticks_to_mm(encoder_get_right_count());
        float distance_covered = (dist_l + dist_r) / 2.0f;

        /* Trapezoidal velocity profile */
        float remaining = target_distance - distance_covered;
        float decel_dist = (current_speed * current_speed - end_speed * end_speed)
                           / (2.0f * DECEL_MMPS2);

        if (remaining <= 0.0f) {
            motion_complete = true;
            motor_brake();
            break;
        }

        if (remaining <= decel_dist) {
            /* Decelerate */
            current_speed -= DECEL_MMPS2 * CONTROL_DT;
            if (current_speed < end_speed) current_speed = end_speed;
        } else if (current_speed < SEARCH_SPEED_MMPS) {
            /* Accelerate */
            current_speed += ACCEL_MMPS2 * CONTROL_DT;
            if (current_speed > SEARCH_SPEED_MMPS) current_speed = SEARCH_SPEED_MMPS;
        }

        /* Heading-hold PID: error = current_yaw - target_yaw */
        float yaw_error = imu_get_yaw() - heading_target;
        float heading_correction = pid_compute(&pid_heading, yaw_error);

        /* Apply differential correction */
        float left_target  = current_speed - heading_correction;
        float right_target = current_speed + heading_correction;

        /* Per-wheel speed PID */
        float left_speed  = encoder_get_left_speed();
        float right_speed = encoder_get_right_speed();

        float left_pwm  = pid_compute(&pid_speed_l, left_target - left_speed);
        float right_pwm = pid_compute(&pid_speed_r, right_target - right_speed);

        motor_set_left((int16_t)left_pwm);
        motor_set_right((int16_t)right_pwm);

        /* Delay ~1ms (control period) */
        /* TODO: Replace with proper timer-based control tick */
        for (volatile int d = 0; d < 7200; d++) { __asm("nop"); }
    }
}

void motion_move_cell(void) {
    motion_move(CELL_SIZE_MM, 0.0f);
}

/* ── Gyro-Based Turn ────────────────────────────────────── */
void motion_turn(float angle_deg) {
    pid_reset(&pid_turn);
    imu_read();

    float start_yaw = imu_get_yaw();
    float target_yaw = start_yaw + angle_deg;
    uint32_t timeout = 0;

    while (timeout < (TURN_TIMEOUT_MS / 1)) {
        imu_read();

        float yaw_error = target_yaw - imu_get_yaw();

        /* Check completion */
        if (yaw_error > -TURN_DEADBAND_DEG && yaw_error < TURN_DEADBAND_DEG) {
            motor_brake();
            /* Update robot heading target */
            robot_pose.target_yaw = target_yaw;
            return;
        }

        float turn_output = pid_compute(&pid_turn, yaw_error);

        /* Spin in place: left and right motors in opposite directions */
        motor_set_left((int16_t)(-turn_output));
        motor_set_right((int16_t)(turn_output));

        timeout++;
        /* TODO: Replace with proper 1ms delay */
        for (volatile int d = 0; d < 7200; d++) { __asm("nop"); }
    }

    /* Timeout — emergency stop */
    motor_brake();
}

void motion_turn_left(void)  { motion_turn(-TURN_ANGLE_90); }
void motion_turn_right(void) { motion_turn(TURN_ANGLE_90);  }
void motion_turn_180(void)   { motion_turn(TURN_ANGLE_180); }

/* ── Direction Execution ────────────────────────────────── */
void motion_execute_direction(Direction target_dir) {
    int diff = (int)target_dir - (int)robot_pose.facing;

    /* Normalize to -2..+2 */
    if (diff > 2)  diff -= 4;
    if (diff < -2) diff += 4;

    /* Turn as needed */
    switch (diff) {
        case  0: break;                           /* Already facing right way */
        case  1: motion_turn_right(); break;      /* Turn right 90° */
        case -1: motion_turn_left();  break;      /* Turn left 90° */
        case  2: case -2: motion_turn_180(); break; /* About-face */
    }

    /* Move one cell forward */
    motion_move_cell();

    /* Update pose */
    robot_pose.facing = target_dir;
    switch (target_dir) {
        case DIR_NORTH: robot_pose.y++; break;
        case DIR_SOUTH: robot_pose.y--; break;
        case DIR_EAST:  robot_pose.x++; break;
        case DIR_WEST:  robot_pose.x--; break;
    }
}

/* ── Front-Wall Squaring (Drift Reset) ──────────────────── */
void motion_square_up(void) {
    /*
     * When both front-left and front-right sensors detect a wall,
     * the robot is facing a wall squarely. Use this to re-zero
     * the gyro yaw to a known cardinal direction, correcting drift.
     *
     * With digital sensors we can't measure distance symmetry,
     * so we simply re-zero the yaw to the nearest 90° multiple.
     */
    if (sensor_front_wall()) {
        float yaw = imu_get_yaw();
        /* Snap to nearest 90° */
        float snapped = ((int)((yaw + 45.0f) / 90.0f)) * 90.0f;
        imu_set_yaw(snapped);
        robot_pose.target_yaw = snapped;
    }
}

/* ── Update (for ISR-based control) ─────────────────────── */
void motion_update(void) {
    /* Reserved for future ISR-driven control loop */
}

bool motion_is_complete(void) { return motion_complete; }

void motion_stop(void) {
    motor_brake();
    motion_complete = true;
    current_speed = 0.0f;
}
