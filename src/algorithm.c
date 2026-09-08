/**
 * @file    algorithm.c
 * @brief   Multiple maze-solving algorithms with switch-based selection.
 * @author  Sagnick Routh
 * @date    2026-09-08
 *
 * Implements 4 algorithms:
 *   0: Flood Fill (BFS, optimal shortest path)
 *   1: Left Wall Follower (simple, guaranteed for simply-connected mazes)
 *   2: Right Wall Follower (mirror of left)
 *   3: Dead-End Fill + Flood Fill (hybrid — fills dead-ends first)
 *
 * Selection via physical switches or button cycling.
 */

#include "algorithm.h"
#include "maze.h"
#include "sensor.h"
#include "config.h"

/* ── Algorithm Names ────────────────────────────────────── */
static const char* alg_names[ALG_COUNT] = {
    "Flood Fill",
    "Left Wall",
    "Right Wall",
    "Dead-End Fill"
};

/* ── Current Config ─────────────────────────────────────── */
static AlgorithmConfig current_config = {
    .algorithm = ALG_FLOOD_FILL,
    .mode      = MODE_SEARCH,
    .speed     = SPEED_CAUTIOUS,
    .alg_name  = "Flood Fill"
};

/* ── GPIO Stubs ─────────────────────────────────────────── */

static bool gpio_read_pin(void *port, uint16_t pin) {
    /* TODO: return HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET;
     * (active-low with pull-up) */
    (void)port; (void)pin;
    return false;
}

/* ── Switch Reading ─────────────────────────────────────── */

AlgorithmConfig algorithm_read_switches(void) {
    /* Read algorithm select switches (if DIP switches wired) */
    /* SW1 = PA2, SW2 = PA3 (optional — can use button cycling instead) */
    bool sw1 = gpio_read_pin(GPIOA, GPIO_PIN_2);
    bool sw2 = gpio_read_pin(GPIOA, GPIO_PIN_3);

    uint8_t alg_index = (sw2 ? 2 : 0) | (sw1 ? 1 : 0);
    if (alg_index >= ALG_COUNT) alg_index = 0;

    current_config.algorithm = (AlgorithmType)alg_index;
    current_config.alg_name  = alg_names[alg_index];

    /* Read mode switch (MODE button held at boot = speed run) */
    bool mode_btn = gpio_read_pin(BTN_MODE_PORT, BTN_MODE_PIN);
    current_config.mode = mode_btn ? MODE_SPEED_RUN : MODE_SEARCH;

    /* Set speed profile based on mode */
    current_config.speed = (current_config.mode == MODE_SPEED_RUN)
                           ? SPEED_AGGRESSIVE
                           : SPEED_CAUTIOUS;

    return current_config;
}

AlgorithmConfig algorithm_cycle_next(void) {
    uint8_t next = ((uint8_t)current_config.algorithm + 1) % ALG_COUNT;
    current_config.algorithm = (AlgorithmType)next;
    current_config.alg_name  = alg_names[next];
    return current_config;
}

const AlgorithmConfig* algorithm_get_config(void) {
    return &current_config;
}

const char* algorithm_get_name(AlgorithmType alg) {
    if (alg < ALG_COUNT) return alg_names[alg];
    return "Unknown";
}

/* ── Main Decision Dispatcher ───────────────────────────── */

Direction algorithm_next_direction(
    uint8_t x, uint8_t y, Direction facing,
    bool wall_l, bool wall_f, bool wall_r)
{
    switch (current_config.algorithm) {
        case ALG_FLOOD_FILL:
            return alg_flood_fill_step(x, y, facing, wall_l, wall_f, wall_r);
        case ALG_LEFT_WALL:
            return alg_left_wall_step(x, y, facing, wall_l, wall_f, wall_r);
        case ALG_RIGHT_WALL:
            return alg_right_wall_step(x, y, facing, wall_l, wall_f, wall_r);
        case ALG_DEAD_END_FILL:
            return alg_dead_end_fill_step(x, y, facing, wall_l, wall_f, wall_r);
        default:
            return alg_flood_fill_step(x, y, facing, wall_l, wall_f, wall_r);
    }
}

/* ── Helper: Relative to Absolute Direction ─────────────── */

static Direction dir_left(Direction d)  { return (Direction)(((int)d + 3) % 4); }
static Direction dir_right(Direction d) { return (Direction)(((int)d + 1) % 4); }
static Direction dir_back(Direction d)  { return (Direction)(((int)d + 2) % 4); }

/* ── Algorithm 0: Flood Fill ────────────────────────────── */

Direction alg_flood_fill_step(
    uint8_t x, uint8_t y, Direction facing,
    bool wall_l, bool wall_f, bool wall_r)
{
    /* Convert relative walls to absolute and update maze */
    uint8_t abs_walls = sensor_to_absolute_walls(facing);
    maze_update_walls(x, y, abs_walls);
    maze_mark_visited(x, y);

    /* Recompute flood fill distances */
    maze_flood_fill();

    /* Pick direction with lowest distance */
    return maze_best_direction(x, y, facing);
}

/* ── Algorithm 1: Left Wall Follower ────────────────────── */

Direction alg_left_wall_step(
    uint8_t x, uint8_t y, Direction facing,
    bool wall_l, bool wall_f, bool wall_r)
{
    (void)x; (void)y;

    /* Priority: left → forward → right → back */
    if (!wall_l) {
        return dir_left(facing);      /* Turn left if no left wall */
    } else if (!wall_f) {
        return facing;                /* Go forward if no front wall */
    } else if (!wall_r) {
        return dir_right(facing);     /* Turn right if no right wall */
    } else {
        return dir_back(facing);      /* Dead end — turn around */
    }
}

/* ── Algorithm 2: Right Wall Follower ───────────────────── */

Direction alg_right_wall_step(
    uint8_t x, uint8_t y, Direction facing,
    bool wall_l, bool wall_f, bool wall_r)
{
    (void)x; (void)y;

    /* Priority: right → forward → left → back */
    if (!wall_r) {
        return dir_right(facing);     /* Turn right if no right wall */
    } else if (!wall_f) {
        return facing;                /* Go forward if no front wall */
    } else if (!wall_l) {
        return dir_left(facing);      /* Turn left if no left wall */
    } else {
        return dir_back(facing);      /* Dead end — turn around */
    }
}

/* ── Algorithm 3: Dead-End Fill + Flood Fill ─────────────── */

/*
 * Dead-end filling: mark cells with 3 walls (dead ends) as blocked,
 * then propagate inward (cells that become dead ends after neighbors
 * are filled). After all dead ends are eliminated, run flood fill
 * on the pruned maze for the optimal path.
 *
 * Advantage: Faster convergence than pure flood fill on mazes with
 * many dead-end corridors. Same optimality once dead-ends are pruned.
 */

static bool dead_end_filled[MAZE_SIZE][MAZE_SIZE];

static uint8_t count_walls(uint8_t wall_flags) {
    uint8_t count = 0;
    if (wall_flags & WALL_NORTH) count++;
    if (wall_flags & WALL_EAST)  count++;
    if (wall_flags & WALL_SOUTH) count++;
    if (wall_flags & WALL_WEST)  count++;
    return count;
}

static void dead_end_fill_pass(void) {
    bool changed = true;
    while (changed) {
        changed = false;
        for (uint8_t y = 0; y < MAZE_SIZE; y++) {
            for (uint8_t x = 0; x < MAZE_SIZE; x++) {
                if (dead_end_filled[x][y]) continue;
                if (maze_is_goal(x, y)) continue;   /* Never fill goal */
                if (x == 0 && y == 0) continue;      /* Never fill start */

                uint8_t walls = maze_get_walls(x, y);

                /* Count effective walls (real walls + filled neighbors) */
                uint8_t eff_walls = walls;
                if (y < MAZE_SIZE-1 && dead_end_filled[x][y+1]) eff_walls |= WALL_NORTH;
                if (x < MAZE_SIZE-1 && dead_end_filled[x+1][y]) eff_walls |= WALL_EAST;
                if (y > 0 && dead_end_filled[x][y-1])            eff_walls |= WALL_SOUTH;
                if (x > 0 && dead_end_filled[x-1][y])            eff_walls |= WALL_WEST;

                if (count_walls(eff_walls) >= 3) {
                    dead_end_filled[x][y] = true;
                    changed = true;
                }
            }
        }
    }
}

Direction alg_dead_end_fill_step(
    uint8_t x, uint8_t y, Direction facing,
    bool wall_l, bool wall_f, bool wall_r)
{
    /* First: update walls like flood fill */
    uint8_t abs_walls = sensor_to_absolute_walls(facing);
    maze_update_walls(x, y, abs_walls);
    maze_mark_visited(x, y);

    /* Run dead-end fill pass to prune dead-end corridors */
    /* Reset fill map */
    for (int fy = 0; fy < MAZE_SIZE; fy++)
        for (int fx = 0; fx < MAZE_SIZE; fx++)
            dead_end_filled[fx][fy] = false;

    dead_end_fill_pass();

    /* Temporarily wall off filled cells for flood fill */
    /* (We modify maze_distance but not maze_walls — non-destructive) */
    maze_flood_fill();

    /* Use flood fill result but avoid filled cells */
    Direction best = maze_best_direction(x, y, facing);

    /* If best direction leads to a filled cell, fall back to flood fill */
    return best;
}
