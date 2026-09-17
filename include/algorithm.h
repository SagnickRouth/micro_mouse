/**
 * @file algorithm.h
 * @brief Maze-solving algorithm selection and dispatch.
 */
#ifndef ALGORITHM_H
#define ALGORITHM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

typedef enum {
    ALG_FLOOD_FILL    = 0,
    ALG_LEFT_WALL     = 1,
    ALG_RIGHT_WALL    = 2,
    ALG_A_STAR        = 3,
    ALG_DEAD_END_FILL = 4,
    ALG_COUNT         = 5
} AlgorithmType;

typedef enum { MODE_SEARCH = 0, MODE_SPEED_RUN = 1 } RunMode;
typedef enum { SPEED_CAUTIOUS = 0, SPEED_NORMAL = 1, SPEED_AGGRESSIVE = 2 } SpeedProfile;

typedef struct {
    AlgorithmType algorithm;
    RunMode mode;
    SpeedProfile speed;
    const char *alg_name;
} AlgorithmConfig;

/*
 * Two-switch algorithm selection using PB4 and PB3.
 * Active-low switches:
 *   OFF/OFF = Flood Fill
 *   ON/OFF  = Left Wall
 *   OFF/ON  = Right Wall
 *   ON/ON   = A*
 */
AlgorithmConfig algorithm_read_switches(void);
AlgorithmConfig algorithm_cycle_next(void);
const AlgorithmConfig* algorithm_get_config(void);
const char* algorithm_get_name(AlgorithmType alg);

Direction algorithm_next_direction(uint8_t x, uint8_t y, Direction facing,
                                   bool wall_l, bool wall_f, bool wall_r);
Direction alg_flood_fill_step(uint8_t x,uint8_t y,Direction facing,bool wall_l,bool wall_f,bool wall_r);
Direction alg_left_wall_step(uint8_t x,uint8_t y,Direction facing,bool wall_l,bool wall_f,bool wall_r);
Direction alg_right_wall_step(uint8_t x,uint8_t y,Direction facing,bool wall_l,bool wall_f,bool wall_r);
Direction alg_dead_end_fill_step(uint8_t x,uint8_t y,Direction facing,bool wall_l,bool wall_f,bool wall_r);
Direction alg_a_star_step(uint8_t x,uint8_t y,Direction facing,bool wall_l,bool wall_f,bool wall_r);

#ifdef __cplusplus
}
#endif
#endif
