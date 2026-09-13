/**
 * @file    algorithm.c
 * @brief   Maze-solving algorithm selection and dispatch.
 * @date    2026-09-13
 *
 * Algorithms:
 *   0: Flood Fill
 *   1: Left Wall Follower
 *   2: Right Wall Follower
 *   3: Dead-End Fill + Flood Fill
 *   4: A* shortest path
 */

#include "algorithm.h"
#include "a_star.h"
#include "maze.h"
#include "sensor.h"
#include "config.h"

static const char* alg_names[ALG_COUNT] = {
    "Flood Fill",
    "Left Wall",
    "Right Wall",
    "Dead-End Fill",
    "A*"
};

static AlgorithmConfig current_config = {
    .algorithm = ALG_FLOOD_FILL,
    .mode      = MODE_SEARCH,
    .speed     = SPEED_CAUTIOUS,
    .alg_name  = "Flood Fill"
};

static bool gpio_read_pin(void *port, uint16_t pin)
{
    /* TODO: return HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET; */
    (void)port;
    (void)pin;
    return false;
}

AlgorithmConfig algorithm_read_switches(void)
{
    /* Preserve the existing two-bit mapping for algorithms 0-3. */
    bool sw1 = gpio_read_pin(GPIOA, GPIO_PIN_2);
    bool sw2 = gpio_read_pin(GPIOA, GPIO_PIN_3);

    uint8_t alg_index = (uint8_t)((sw2 ? 2u : 0u) | (sw1 ? 1u : 0u));
    if (alg_index >= 4u) alg_index = ALG_FLOOD_FILL;

    current_config.algorithm = (AlgorithmType)alg_index;
    current_config.alg_name  = alg_names[alg_index];

    bool mode_btn = gpio_read_pin(BTN_MODE_PORT, BTN_MODE_PIN);
    current_config.mode = mode_btn ? MODE_SPEED_RUN : MODE_SEARCH;
    current_config.speed = (current_config.mode == MODE_SPEED_RUN)
                           ? SPEED_AGGRESSIVE
                           : SPEED_CAUTIOUS;

    return current_config;
}

AlgorithmConfig algorithm_cycle_next(void)
{
    uint8_t next = (uint8_t)(((uint8_t)current_config.algorithm + 1u) % ALG_COUNT);
    current_config.algorithm = (AlgorithmType)next;
    current_config.alg_name  = alg_names[next];
    return current_config;
}

const AlgorithmConfig* algorithm_get_config(void)
{
    return &current_config;
}

const char* algorithm_get_name(AlgorithmType alg)
{
    if ((uint8_t)alg < ALG_COUNT) return alg_names[alg];
    return "Unknown";
}

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
        case ALG_A_STAR:
            return alg_a_star_step(x, y, facing, wall_l, wall_f, wall_r);
        default:
            return alg_flood_fill_step(x, y, facing, wall_l, wall_f, wall_r);
    }
}

static Direction dir_left(Direction d)
{
    return (Direction)(((uint8_t)d + 3u) & 0x03u);
}

static Direction dir_right(Direction d)
{
    return (Direction)(((uint8_t)d + 1u) & 0x03u);
}

static Direction dir_back(Direction d)
{
    return (Direction)(((uint8_t)d + 2u) & 0x03u);
}

Direction alg_flood_fill_step(
    uint8_t x, uint8_t y, Direction facing,
    bool wall_l, bool wall_f, bool wall_r)
{
    (void)wall_l;
    (void)wall_f;
    (void)wall_r;

    uint8_t abs_walls = sensor_to_absolute_walls(facing);
    maze_update_walls(x, y, abs_walls);
    maze_mark_visited(x, y);
    maze_flood_fill();

    return maze_best_direction(x, y, facing);
}

Direction alg_left_wall_step(
    uint8_t x, uint8_t y, Direction facing,
    bool wall_l, bool wall_f, bool wall_r)
{
    (void)x;
    (void)y;

    if (!wall_l) return dir_left(facing);
    if (!wall_f) return facing;
    if (!wall_r) return dir_right(facing);
    return dir_back(facing);
}

Direction alg_right_wall_step(
    uint8_t x, uint8_t y, Direction facing,
    bool wall_l, bool wall_f, bool wall_r)
{
    (void)x;
    (void)y;

    if (!wall_r) return dir_right(facing);
    if (!wall_f) return facing;
    if (!wall_l) return dir_left(facing);
    return dir_back(facing);
}

static bool dead_end_filled[MAZE_SIZE][MAZE_SIZE];

static uint8_t count_walls(uint8_t wall_flags)
{
    uint8_t count = 0;
    if (wall_flags & WALL_NORTH) count++;
    if (wall_flags & WALL_EAST)  count++;
    if (wall_flags & WALL_SOUTH) count++;
    if (wall_flags & WALL_WEST)  count++;
    return count;
}

static void dead_end_fill_pass(void)
{
    bool changed = true;

    while (changed) {
        changed = false;

        for (uint8_t y = 0; y < MAZE_SIZE; y++) {
            for (uint8_t x = 0; x < MAZE_SIZE; x++) {
                if (dead_end_filled[x][y]) continue;
                if (maze_is_goal(x, y)) continue;
                if (x == 0 && y == 0) continue;

                uint8_t effective_walls = maze_get_walls(x, y);

                if (y < MAZE_SIZE - 1 && dead_end_filled[x][y + 1])
                    effective_walls |= WALL_NORTH;
                if (x < MAZE_SIZE - 1 && dead_end_filled[x + 1][y])
                    effective_walls |= WALL_EAST;
                if (y > 0 && dead_end_filled[x][y - 1])
                    effective_walls |= WALL_SOUTH;
                if (x > 0 && dead_end_filled[x - 1][y])
                    effective_walls |= WALL_WEST;

                if (count_walls(effective_walls) >= 3u) {
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
    (void)wall_l;
    (void)wall_f;
    (void)wall_r;

    uint8_t abs_walls = sensor_to_absolute_walls(facing);
    maze_update_walls(x, y, abs_walls);
    maze_mark_visited(x, y);

    memset(dead_end_filled, 0, sizeof(dead_end_filled));
    dead_end_fill_pass();

    /* The existing maze_flood_fill() does not yet accept a blocked-cell mask.
     * Therefore the dead-end map is retained for future pruning integration,
     * while the actual route remains guaranteed by the normal flood fill. */
    maze_flood_fill();
    return maze_best_direction(x, y, facing);
}

Direction alg_a_star_step(
    uint8_t x, uint8_t y, Direction facing,
    bool wall_l, bool wall_f, bool wall_r)
{
    (void)wall_l;
    (void)wall_f;
    (void)wall_r;

    /* Keep the same wall-update convention as flood fill before planning. */
    uint8_t abs_walls = sensor_to_absolute_walls(facing);
    maze_update_walls(x, y, abs_walls);
    maze_mark_visited(x, y);

    Direction next;
    if (a_star_next_direction(x, y, facing, &next)) {
        return next;
    }

    /* No A* route: retain the current heading rather than commanding an
     * invalid direction. The caller can treat this as a navigation fault. */
    return facing;
}
