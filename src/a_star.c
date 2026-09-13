/**
 * @file    a_star.c
 * @brief   A* shortest-path planner for the micromouse maze.
 *
 * Design notes:
 *   - 16x16 grid, fixed-size storage: no malloc/free on STM32.
 *   - Four configured goal cells are supported through maze_is_goal().
 *   - Manhattan distance to the nearest goal is the heuristic.
 *   - The heuristic is admissible and consistent for unit-cost cell moves.
 *   - A binary min-heap with decrease-key keeps the planner deterministic
 *     without allocating a large duplicate-entry priority queue.
 *   - Walls are read from maze_walls, so the planner uses the same map as
 *     the existing flood-fill implementation.
 *
 * Unknown cells are intentionally treated as open. During the search run,
 * newly discovered walls are written into maze_walls before A* is called;
 * this makes the result conservative only with respect to known walls, which
 * is the same assumption made by the current flood-fill implementation.
 */

#include "a_star.h"
#include "maze.h"
#include <string.h>

#define ASTAR_NODE_COUNT (MAZE_SIZE * MAZE_SIZE)
#define ASTAR_INF        0xFFFFu
#define ASTAR_NONE       0xFFu

static const int8_t dx[4] = { 0, 1, 0, -1 }; /* N, E, S, W */
static const int8_t dy[4] = { 1, 0, -1, 0 };
static const uint8_t dir_wall[4] = {
    WALL_NORTH, WALL_EAST, WALL_SOUTH, WALL_WEST
};

/* Per-plan state. Index = y * MAZE_SIZE + x. */
static uint16_t g_score[ASTAR_NODE_COUNT];
static uint16_t f_score[ASTAR_NODE_COUNT];
static uint8_t  came_from[ASTAR_NODE_COUNT];
static uint8_t  came_dir[ASTAR_NODE_COUNT];
static uint8_t  heap[ASTAR_NODE_COUNT];
static uint8_t  heap_pos[ASTAR_NODE_COUNT];
static bool     closed[ASTAR_NODE_COUNT];
static uint16_t heap_size;

static uint8_t node_index(uint8_t x, uint8_t y)
{
    return (uint8_t)(y * MAZE_SIZE + x);
}

static void node_xy(uint8_t node, uint8_t *x, uint8_t *y)
{
    *x = (uint8_t)(node % MAZE_SIZE);
    *y = (uint8_t)(node / MAZE_SIZE);
}

static uint8_t popcount4(uint8_t value)
{
    value &= 0x0F;
    value = (uint8_t)(value - ((value >> 1) & 0x55));
    value = (uint8_t)((value & 0x33) + ((value >> 2) & 0x33));
    return (uint8_t)((value + (value >> 4)) & 0x0F);
}

/* Lower f first; on equal f prefer lower g, then lower node index. */
static bool heap_less(uint8_t a, uint8_t b)
{
    if (f_score[a] != f_score[b]) return f_score[a] < f_score[b];
    if (g_score[a] != g_score[b]) return g_score[a] < g_score[b];
    return a < b;
}

static void heap_swap(uint16_t a, uint16_t b)
{
    uint8_t na = heap[a];
    uint8_t nb = heap[b];
    heap[a] = nb;
    heap[b] = na;
    heap_pos[na] = (uint8_t)b;
    heap_pos[nb] = (uint8_t)a;
}

static void heap_sift_up(uint16_t pos)
{
    while (pos > 0) {
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
    heap_size = 0;
    for (uint16_t i = 0; i < ASTAR_NODE_COUNT; i++) {
        heap_pos[i] = ASTAR_NONE;
    }
}

static bool heap_push_or_decrease(uint8_t node)
{
    uint8_t pos = heap_pos[node];

    if (pos != ASTAR_NONE) {
        heap_sift_up(pos);
        heap_sift_down(heap_pos[node]);
        return true;
    }

    if (heap_size >= ASTAR_NODE_COUNT) return false;

    heap[heap_size] = node;
    heap_pos[node] = (uint8_t)heap_size;
    heap_size++;
    heap_sift_up((uint16_t)(heap_size - 1u));
    return true;
}

static uint8_t heap_pop(void)
{
    uint8_t result = heap[0];
    heap_pos[result] = ASTAR_NONE;

    heap_size--;
    if (heap_size != 0) {
        heap[0] = heap[heap_size];
        heap_pos[heap[0]] = 0;
        heap_sift_down(0);
    }

    return result;
}

static uint16_t heuristic(uint8_t x, uint8_t y)
{
    uint16_t best = ASTAR_INF;

    /* Goal count is at most four, so this is cheap on the STM32. */
    for (uint8_t gy = 0; gy < MAZE_SIZE; gy++) {
        for (uint8_t gx = 0; gx < MAZE_SIZE; gx++) {
            if (!maze_is_goal(gx, gy)) continue;

            uint16_t dx_abs = (x > gx) ? (uint16_t)(x - gx)
                                       : (uint16_t)(gx - x);
            uint16_t dy_abs = (y > gy) ? (uint16_t)(y - gy)
                                       : (uint16_t)(gy - y);
            uint16_t h = (uint16_t)(dx_abs + dy_abs);
            if (h < best) best = h;
        }
    }

    /* No configured goal: make the node unreachable rather than guessing. */
    return best;
}

static bool passable(uint8_t x, uint8_t y, Direction d,
                     uint8_t *nx, uint8_t *ny)
{
    if (maze_walls[x][y] & dir_wall[d]) return false;

    int16_t tx = (int16_t)x + dx[d];
    int16_t ty = (int16_t)y + dy[d];

    if (tx < 0 || tx >= MAZE_SIZE || ty < 0 || ty >= MAZE_SIZE) {
        return false;
    }

    /* Require the neighbor to agree that the shared edge is open. */
    Direction opposite = (Direction)(((uint8_t)d + 2u) & 0x03u);
    if (maze_walls[(uint8_t)tx][(uint8_t)ty] & dir_wall[opposite]) {
        return false;
    }

    *nx = (uint8_t)tx;
    *ny = (uint8_t)ty;
    return true;
}

static void reconstruct_path(uint8_t start_node, uint8_t goal_node,
                             AStarPath *path)
{
    Direction reverse_dirs[A_STAR_MAX_PATH];
    uint16_t reverse_len = 0;
    uint8_t node = goal_node;

    while (node != start_node && reverse_len < A_STAR_MAX_PATH) {
        reverse_dirs[reverse_len++] = (Direction)came_dir[node];
        node = came_from[node];
    }

    path->length = reverse_len;
    for (uint16_t i = 0; i < reverse_len; i++) {
        path->directions[i] = reverse_dirs[reverse_len - 1u - i];
    }
}

void a_star_reset(void)
{
    memset(g_score, 0xFF, sizeof(g_score));
    memset(f_score, 0xFF, sizeof(f_score));
    memset(came_from, ASTAR_NONE, sizeof(came_from));
    memset(came_dir, ASTAR_NONE, sizeof(came_dir));
    memset(closed, 0, sizeof(closed));
    heap_clear();
}

bool a_star_find_path(uint8_t start_x, uint8_t start_y, AStarPath *path)
{
    if (path == NULL || start_x >= MAZE_SIZE || start_y >= MAZE_SIZE) {
        return false;
    }

    path->length = 0;
    a_star_reset();

    uint16_t start_h = heuristic(start_x, start_y);
    if (start_h == ASTAR_INF) return false;

    uint8_t start = node_index(start_x, start_y);
    g_score[start] = 0;
    f_score[start] = start_h;

    if (!heap_push_or_decrease(start)) return false;

    while (heap_size != 0) {
        uint8_t current = heap_pop();
        uint8_t cx, cy;
        node_xy(current, &cx, &cy);

        if (maze_is_goal(cx, cy)) {
            reconstruct_path(start, current, path);
            return true;
        }

        closed[current] = true;

        for (Direction d = DIR_NORTH; d <= DIR_WEST; d++) {
            uint8_t nx, ny;
            if (!passable(cx, cy, d, &nx, &ny)) continue;

            uint8_t neighbor = node_index(nx, ny);
            if (closed[neighbor]) continue;

            uint16_t tentative_g = (uint16_t)(g_score[current] + 1u);
            if (tentative_g >= g_score[neighbor]) continue;

            came_from[neighbor] = current;
            came_dir[neighbor] = (uint8_t)d;
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
    if (!a_star_find_path(start_x, start_y, &path) || path.length == 0) {
        return false;
    }

    /* Prefer the A* path. Facing is intentionally not part of the graph
     * cost: A* finds the shortest cell path, while motion.c decides whether
     * the transition is a straight, 90-degree, or 180-degree maneuver. */
    (void)facing;
    *next_direction = path.directions[0];
    return true;
}
