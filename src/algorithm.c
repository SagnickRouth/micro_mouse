#include "algorithm.h"
#include "a_star.h"
#include "maze.h"
#include "sensor.h"
#include "config.h"
#include "stm32f4xx_hal.h"
#include <string.h>

static const char *alg_names[ALG_COUNT] = {
    "Flood Fill", "Left Wall", "Right Wall", "A*", "Dead-End Fill"
};

static AlgorithmConfig current_config = {
    ALG_FLOOD_FILL, MODE_SEARCH, SPEED_CAUTIOUS, "Flood Fill"
};

static bool gpio_read_active_low(GPIO_TypeDef *port, uint16_t pin)
{
    return HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET;
}

/*
 * Two physical DIP switches select one of four algorithms.
 * Both switches are active-low and use the pins defined in config.h:
 *
 *   PB4 (ALG0)   PB3 (ALG1)   Algorithm
 *   ------------------------------------
 *     OFF          OFF        Flood Fill
 *     ON           OFF        Left Wall
 *     OFF          ON         Right Wall
 *     ON           ON         A*
 *
 * If this function is called while a switch is changing, the resulting
 * combination is simply read again by the caller on its next iteration.
 */
AlgorithmConfig algorithm_read_switches(void)
{
    bool sw0 = gpio_read_active_low(DIP_ALG0_PORT, DIP_ALG0_PIN);
    bool sw1 = gpio_read_active_low(DIP_ALG1_PORT, DIP_ALG1_PIN);

    uint8_t alg;

    if (!sw0 && !sw1) {
        alg = ALG_FLOOD_FILL;
    } else if (sw0 && !sw1) {
        alg = ALG_LEFT_WALL;
    } else if (!sw0 && sw1) {
        alg = ALG_RIGHT_WALL;
    } else {
        alg = ALG_A_STAR;
    }

    current_config.algorithm = (AlgorithmType)alg;
    current_config.alg_name = alg_names[alg];
    current_config.mode = MODE_SEARCH;
    current_config.speed = SPEED_CAUTIOUS;

    return current_config;
}

AlgorithmConfig algorithm_cycle_next(void)
{
    uint8_t next = ((uint8_t)current_config.algorithm + 1u) % 4u;
    current_config.algorithm = (AlgorithmType)next;
    current_config.alg_name = alg_names[next];
    return current_config;
}

const AlgorithmConfig* algorithm_get_config(void)
{
    return &current_config;
}

const char* algorithm_get_name(AlgorithmType alg)
{
    return ((uint8_t)alg < ALG_COUNT) ? alg_names[alg] : "Unknown";
}

static Direction dir_left(Direction d)
{
    return (Direction)(((uint8_t)d + 3u) & 3u);
}

static Direction dir_right(Direction d)
{
    return (Direction)(((uint8_t)d + 1u) & 3u);
}

static Direction dir_back(Direction d)
{
    return (Direction)(((uint8_t)d + 2u) & 3u);
}

Direction algorithm_next_direction(uint8_t x,uint8_t y,Direction facing,
                                   bool wall_l,bool wall_f,bool wall_r)
{
    switch (current_config.algorithm) {
        case ALG_FLOOD_FILL:
            return alg_flood_fill_step(x,y,facing,wall_l,wall_f,wall_r);
        case ALG_LEFT_WALL:
            return alg_left_wall_step(x,y,facing,wall_l,wall_f,wall_r);
        case ALG_RIGHT_WALL:
            return alg_right_wall_step(x,y,facing,wall_l,wall_f,wall_r);
        case ALG_A_STAR:
            return alg_a_star_step(x,y,facing,wall_l,wall_f,wall_r);
        case ALG_DEAD_END_FILL:
            return alg_dead_end_fill_step(x,y,facing,wall_l,wall_f,wall_r);
        default:
            return facing;
    }
}

Direction alg_flood_fill_step(uint8_t x,uint8_t y,Direction facing,bool l,bool f,bool r)
{
    (void)l;
    (void)f;
    (void)r;
    maze_update_walls(x,y,sensor_to_absolute_walls(facing));
    maze_mark_visited(x,y);
    maze_flood_fill();
    return maze_best_direction(x,y,facing);
}

Direction alg_left_wall_step(uint8_t x,uint8_t y,Direction facing,bool l,bool f,bool r)
{
    (void)x;
    (void)y;
    if (!l) return dir_left(facing);
    if (!f) return facing;
    if (!r) return dir_right(facing);
    return dir_back(facing);
}

Direction alg_right_wall_step(uint8_t x,uint8_t y,Direction facing,bool l,bool f,bool r)
{
    (void)x;
    (void)y;
    if (!r) return dir_right(facing);
    if (!f) return facing;
    if (!l) return dir_left(facing);
    return dir_back(facing);
}

static bool dead_end_filled[MAZE_SIZE][MAZE_SIZE];

static uint8_t count_walls(uint8_t w)
{
    return (uint8_t)(((w & WALL_NORTH) != 0) +
                     ((w & WALL_EAST)  != 0) +
                     ((w & WALL_SOUTH) != 0) +
                     ((w & WALL_WEST)  != 0));
}

static void dead_end_fill_pass(void)
{
    bool changed = true;

    while (changed) {
        changed = false;

        for (uint8_t y = 0; y < MAZE_SIZE; y++) {
            for (uint8_t x = 0; x < MAZE_SIZE; x++) {
                if (dead_end_filled[x][y] || maze_is_goal(x,y) || (x == 0 && y == 0))
                    continue;

                uint8_t w = maze_get_walls(x,y);

                if (y < MAZE_SIZE-1 && dead_end_filled[x][y+1]) w |= WALL_NORTH;
                if (x < MAZE_SIZE-1 && dead_end_filled[x+1][y]) w |= WALL_EAST;
                if (y > 0 && dead_end_filled[x][y-1]) w |= WALL_SOUTH;
                if (x > 0 && dead_end_filled[x-1][y]) w |= WALL_WEST;

                if (count_walls(w) >= 3u) {
                    dead_end_filled[x][y] = true;
                    changed = true;
                }
            }
        }
    }
}

Direction alg_dead_end_fill_step(uint8_t x,uint8_t y,Direction facing,bool l,bool f,bool r)
{
    (void)l;
    (void)f;
    (void)r;
    maze_update_walls(x,y,sensor_to_absolute_walls(facing));
    maze_mark_visited(x,y);
    memset(dead_end_filled,0,sizeof(dead_end_filled));
    dead_end_fill_pass();
    maze_flood_fill();
    return maze_best_direction(x,y,facing);
}

Direction alg_a_star_step(uint8_t x,uint8_t y,Direction facing,bool l,bool f,bool r)
{
    a_star_update_walls(x,y,facing,l,f,r);
    maze_mark_visited(x,y);
    Direction next;
    return a_star_next_direction(x,y,facing,&next) ? next : facing;
}
