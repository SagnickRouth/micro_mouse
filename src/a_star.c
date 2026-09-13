/**
 * @file    a_star.c
 * @brief   Heading-aware A* planner for the micromouse maze.
 *
 * Reference: simulations/mms/Main.py
 *
 * The MMS implementation uses Dijkstra over (x, y, facing) with:
 *   MOVE_COST     = 2
 *   TURN_COST_90  = 3
 *   TURN_COST_180 = 6
 *
 * Here the same state graph and edge costs are used, but A* is used with the
 * admissible heuristic 2 * ManhattanDistanceToNearestGoal. Turn penalties
 * are non-negative, so ignoring them in the heuristic cannot overestimate.
 *
 * Unknown maze edges are treated as open, matching Main.py's search behavior.
 * Walls discovered by the L/F/R sensors are mirrored into the neighboring
 * cell before every planning call. The planner itself never drives motors;
 * motion.c remains responsible for gyro-based turns and forward motion.
 */

#include "a_star.h"
#include "maze.h"
#include <stddef.h>
#include <string.h>

#define ASTAR_STATE_COUNT (MAZE_SIZE * MAZE_SIZE * 4u)
#define ASTAR_INF         0xFFFFu
#define ASTAR_NONE        0xFFFFu

static const int8_t dx[4] = { 0, 1, 0, -1 }; /* N, E, S, W */
static const int8_t dy[4] = { 1, 0, -1, 0 };
static const uint8_t wall_bit[4] = {
    WALL_NORTH, WALL_EAST, WALL_SOUTH, WALL_WEST
};
static const uint8_t opposite_dir[4] = { DIR_SOUTH, DIR_WEST, DIR_NORTH, DIR_EAST };

/* A* state arrays: state = ((y * MAZE_SIZE) + x) * 4 + facing. */
static uint16_t g_score[ASTAR_STATE_COUNT];
static uint16_t f_score[ASTAR_STATE_COUNT];
static uint16_t came_from[ASTAR_STATE_COUNT];
static uint8_t  came_dir[ASTAR_STATE_COUNT];
static uint16_t heap[ASTAR_STATE_COUNT];
static uint16_t heap_pos[ASTAR_STATE_COUNT];
static bool     closed[ASTAR_STATE_COUNT];
static uint16_t heap_size;

static uint16_t cell_index(uint8_t x, uint8_t y)
{
    return (uint16_t)(y * MAZE_SIZE + x);
}

static uint16_t state_index(uint8_t x, uint8_t y, Direction facing)
{
    return (uint16_t)(cell_index(x, y) * 4u + (uint8_t)facing);
}

static void state_xyf(uint16_t state, uint8_t *x, uint8_t *y,
                      Direction *facing)
{
    uint16_t cell = (uint16_t)(state / 4u);
    *facing = (Direction)(state & 0x03u);
    *x = (uint8_t)(cell % MAZE_SIZE);
    *y = (uint8_t)(cell / MAZE_SIZE);
}

static bool heap_less(uint16_t a, uint16_t b)
{
    if (f_score[a] != f_score[b]) return f_score[a] < f_score[b];
    if (g_score[a] != g_score[b]) return g_score[a] < g_score[b];
    return a < b;
}

static void heap_swap(uint16_t a, uint16_t b)
{
    uint16_t sa = heap[a];
    uint16_t sb = heap[b];
    heap[a] = sb;
    heap[b] = sa;
    heap_pos[sa] = b;
    heap_pos[sb] = a;
}

static void heap_sift_up(uint16_t pos)
{
    while (pos > 0u) {
        uint16_t parent = (uint16_t)((pos - 1u) / 2u);
        if (!heap_less(heap[pos], heap[parent])) break;
        heap_swap(pos, parent);
        pos = parent;
    }
}

static void heap_sift_down(uint16_t pos)
{
    for (;;) {
        uint16_t left = (uint16_t)(2u * pos + 1u);
        uint16_t right = (uint16_t)(left + 1u);
        uint16_t smallest = pos;

        if (left < heap_size && heap_less(heap[left], heap[smallest])) {
            smallest = left;
        }
        if (right < heap_size && heap_less(heap[right], heap[smallest])) {
            smallest = right;
        }
        if (smallest == pos) break;
        heap_swap(pos, smallest);
        pos = smallest;
    }
}

static void heap_clear(void)
{
    heap_size = 0u;
    for (uint16_t i = 0; i < ASTAR_STATE_COUNT; i++) {
        heap_pos[i] = ASTAR_NONE;
    }
}

static bool heap_push_or_decrease(uint16_t state)
{
    uint16_t pos = heap_pos[state];

    if (pos != ASTAR_NONE) {
        heap_sift_up(pos);
        heap_sift_down(heap_pos[state]);
        return true;
    }

    if (heap_size >= ASTAR_STATE_COUNT) return false;

    heap[heap_size] = state;
    heap_pos[state] = heap_size;
    heap_size++;
    heap_sift_up((uint16_t)(heap_size - 1u));
    return true;
}

static uint16_t heap_pop(void)
{
    uint16_t result = heap[0];
    heap_pos[result] = ASTAR_NONE;

    heap_size--;
    if (heap_size != 0u) {
        heap[0] = heap[heap_size];
        heap_pos[heap[0]] = 0u;
        heap_sift_down(0u);
    }
    return result;
}

static uint16_t heuristic(uint8_t x, uint8_t y)
{
    uint16_t best = ASTAR_INF;

    /* Main.py uses the nearest of the four center goal cells. */
    for (uint8_t gy = 0; gy < MAZE_SIZE; gy++) {
        for (uint8_t gx = 0; gx < MAZE_SIZE; gx++) {
            if (!maze_is_goal(gx, gy)) continue;

            uint16_t ax = (x > gx) ? (uint16_t)(x - gx) : (uint16_t)(gx - x);
            uint16_t ay = (y > gy) ? (uint16_t)(y - gy) : (uint16_t)(gy - y);
            uint16_t h = (uint16_t)(A_STAR_MOVE_COST * (ax + ay));
            if (h < best) best = h;
        }
    }
    return best;
}

static bool passable(uint8_t x, uint8_t y, Direction direction,
                     uint8_t *nx, uint8_t *ny)
{
    if (maze_walls[x][y] & wall_bit[direction]) return false;

    int16_t tx = (int16_t)x + dx[direction];
    int16_t ty = (int16_t)y + dy[direction];
    if (tx < 0 || tx >= MAZE_SIZE || ty < 0 || ty >= MAZE_SIZE) {
        return false;
    }

    /* Match the simulator's mirrored-wall map semantics. */
    if (maze_walls[(uint8_t)tx][(uint8_t)ty] &
        wall_bit[opposite_dir[direction]]) {
        return false;
    }

    *nx = (uint8_t)tx;
    *ny = (uint8_t)ty;
    return true;
}

static uint16_t edge_cost(Direction from, Direction to)
{
    uint8_t diff = (uint8_t)(((uint8_t)to - (uint8_t)from) & 0x03u);

    if (diff == 0u) return A_STAR_MOVE_COST;
    if (diff == 1u || diff == 3u) {
        return (uint16_t)(A_STAR_MOVE_COST + A_STAR_TURN_COST_90);
    }
    return (uint16_t)(A_STAR_MOVE_COST + A_STAR_TURN_COST_180);
}

static void reconstruct_path(uint16_t start_state, uint16_t goal_state,
                             AStarPath *path)
{
    Direction reverse[A_STAR_MAX_PATH];
    uint16_t length = 0u;
    uint16_t state = goal_state;

    while (state != start_state && length < A_STAR_MAX_PATH) {
        reverse[length++] = (Direction)came_dir[state];
        state = came_from[state];
    }

    path->length = length;
    for (uint16_t i = 0u; i < length; i++) {
        path->directions[i] = reverse[length - 1u - i];
    }
}

void a_star_update_walls(uint8_t x, uint8_t y, Direction facing,
                         bool wall_l, bool wall_f, bool wall_r)
{
    if (x >= MAZE_SIZE || y >= MAZE_SIZE) return;

    bool sensed[3] = { wall_l, wall_f, wall_r };
    Direction relative_dir[3] = {
        (Direction)(((uint8_t)facing + 3u) & 0x03u),
        facing,
        (Direction)(((uint8_t)facing + 1u) & 0x03u)
    };

    for (uint8_t i = 0u; i < 3u; i++) {
        if (!sensed[i]) continue;

        Direction d = relative_dir[i];
        maze_walls[x][y] |= wall_bit[d];

        int16_t nx = (int16_t)x + dx[d];
        int16_t ny = (int16_t)y + dy[d];
        if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) {
            maze_walls[(uint8_t)nx][(uint8_t)ny] |= wall_bit[opposite_dir[d]];
        }
    }
}

void a_star_reset(void)
{
    memset(g_score, 0xFF, sizeof(g_score));
    memset(f_score, 0xFF, sizeof(f_score));
    memset(came_from, 0xFF, sizeof(came_from));
    memset(came_dir, 0xFF, sizeof(came_dir));
    memset(closed, 0, sizeof(closed));
    heap_clear();
}

bool a_star_find_path(uint8_t start_x, uint8_t start_y,
                      Direction facing, AStarPath *path)
{
    if (path == NULL || start_x >= MAZE_SIZE || start_y >= MAZE_SIZE ||
        (uint8_t)facing > DIR_WEST) {
        return false;
    }

    path->length = 0u;
    path->cost = 0u;
    a_star_reset();

    uint16_t start = state_index(start_x, start_y, facing);
    uint16_t start_h = heuristic(start_x, start_y);
    if (start_h == ASTAR_INF) return false;

    g_score[start] = 0u;
    f_score[start] = start_h;
    if (!heap_push_or_decrease(start)) return false;

    while (heap_size != 0u) {
        uint16_t current = heap_pop();
        uint8_t cx, cy;
        Direction cf;
        state_xyf(current, &cx, &cy, &cf);

        if (maze_is_goal(cx, cy)) {
            reconstruct_path(start, current, path);
            path->cost = g_score[current];
            return true;
        }

        closed[current] = true;

        for (Direction next = DIR_NORTH; next <= DIR_WEST; next++) {
            uint8_t nx, ny;
            if (!passable(cx, cy, next, &nx, &ny)) continue;

            uint16_t neighbor = state_index(nx, ny, next);
            if (closed[neighbor]) continue;

            uint16_t step = edge_cost(cf, next);
            uint16_t tentative_g = (uint16_t)(g_score[current] + step);
            if (tentative_g >= g_score[neighbor]) continue;

            came_from[neighbor] = current;
            came_dir[neighbor] = (uint8_t)next;
            g_score[neighbor] = tentative_g;

            uint16_t h = heuristic(nx, ny);
            if (h == ASTAR_INF) continue;
            f_score[neighbor] = (uint16_t)(tentative_g + h);

            if (!heap_push_or_decrease(neighbor)) return false;
        }
    }

    return false;
}

bool a_star_next_direction(uint8_t start_x, uint8_t start_y,
                           Direction facing, Direction *next_direction)
{
    if (next_direction == NULL) return false;

    AStarPath path;
    if (!a_star_find_path(start_x, start_y, facing, &path) ||
        path.length == 0u) {
        return false;
    }

    *next_direction = path.directions[0];
    return true;
}
