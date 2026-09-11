"""
Wall follower algorithms for MMS simulator.

NOTE: CELESTA rules state wall-hugging will NOT find the destination.
Use flood_fill for competition. These are for testing only.
"""

import API

NORTH, EAST, SOUTH, WEST = 0, 1, 2, 3
DX = [0, 1, 0, -1]
DY = [1, 0, -1, 0]


class LeftWallSolver:
    """Always try left first, then forward, then right, then back."""

    def __init__(self, width, height):
        self.w = width
        self.h = height
        self.x = 0
        self.y = 0
        self.facing = NORTH
        self.step_count = 0
        API.setColor(0, 0, "g")

    def step(self, wall_l, wall_f, wall_r):
        self.step_count += 1
        API.setText(self.x, self.y, str(self.step_count))
        API.setColor(self.x, self.y, "c")

        if not wall_l:
            target = (self.facing + 3) % 4
            self.facing = target
            self.x += DX[target]
            self.y += DY[target]
            return "left"
        elif not wall_f:
            self.x += DX[self.facing]
            self.y += DY[self.facing]
            return "forward"
        elif not wall_r:
            target = (self.facing + 1) % 4
            self.facing = target
            self.x += DX[target]
            self.y += DY[target]
            return "right"
        else:
            target = (self.facing + 2) % 4
            self.facing = target
            self.x += DX[target]
            self.y += DY[target]
            return "turn_around"


class RightWallSolver:
    """Always try right first, then forward, then left, then back."""

    def __init__(self, width, height):
        self.w = width
        self.h = height
        self.x = 0
        self.y = 0
        self.facing = NORTH
        self.step_count = 0
        API.setColor(0, 0, "g")

    def step(self, wall_l, wall_f, wall_r):
        self.step_count += 1
        API.setText(self.x, self.y, str(self.step_count))
        API.setColor(self.x, self.y, "m")

        if not wall_r:
            target = (self.facing + 1) % 4
            self.facing = target
            self.x += DX[target]
            self.y += DY[target]
            return "right"
        elif not wall_f:
            self.x += DX[self.facing]
            self.y += DY[self.facing]
            return "forward"
        elif not wall_l:
            target = (self.facing + 3) % 4
            self.facing = target
            self.x += DX[target]
            self.y += DY[target]
            return "left"
        else:
            target = (self.facing + 2) % 4
            self.facing = target
            self.x += DX[target]
            self.y += DY[target]
            return "turn_around"
