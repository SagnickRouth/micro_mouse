/**
 * @file    competition.c
 * @brief   Competition run state management — CELESTA'26.
 * @date    2026-09-08
 */

#include "competition.h"

void run_state_init(RunState *rs, uint8_t run_number) {
    rs->current_run       = run_number;
    rs->run_start_time_ms = 0;
    rs->elapsed_ms        = 0;
    rs->restart_count     = 0;
    rs->penalty_ms        = 0;
    rs->run_complete      = false;
    rs->timed_out         = false;
    rs->last_checkpoint_x = 0;
    rs->last_checkpoint_y = 0;
    rs->checkpoint_reached = false;
}

bool run_state_update(RunState *rs, uint32_t current_millis) {
    if (rs->run_start_time_ms == 0) {
        rs->run_start_time_ms = current_millis;
    }
    rs->elapsed_ms = current_millis - rs->run_start_time_ms;

    if (rs->elapsed_ms >= RUN_TIME_LIMIT_MS) {
        rs->timed_out = true;
        return true;
    }
    return false;
}

void run_state_add_restart(RunState *rs) {
    rs->restart_count++;
    rs->penalty_ms += RESTART_PENALTY_MS;
}

void run_state_set_checkpoint(RunState *rs, uint8_t x, uint8_t y) {
    rs->last_checkpoint_x = x;
    rs->last_checkpoint_y = y;
    rs->checkpoint_reached = true;
}

uint32_t run_state_total_time(const RunState *rs) {
    return rs->elapsed_ms + rs->penalty_ms;
}

bool run_state_timed_out(const RunState *rs) {
    return rs->timed_out;
}
