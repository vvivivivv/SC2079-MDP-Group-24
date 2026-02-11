import math
from enum import Enum


class Direction(int, Enum):
    NORTH = 0
    EAST = 2
    SOUTH = 4
    WEST = 6
    SKIP = 8

    def __int__(self):
        return self.value

    @staticmethod
    def rotation_cost(d1, d2):
        diff = abs(d1 - d2)
        return min(diff, 8 - diff)

    @staticmethod
    def to_angle_deg(d):
        """Convert Direction enum to angle in degrees (math convention: EAST=0, CCW positive)"""
        mapping = {
            0: 90.0,    # NORTH
            2: 0.0,     # EAST
            4: 270.0,   # SOUTH
            6: 180.0,   # WEST
        }
        return mapping[int(d)]

    @staticmethod
    def to_angle_rad(d):
        """Convert Direction enum to angle in radians"""
        return math.radians(Direction.to_angle_deg(d))


MOVE_DIRECTION = [
    (1, 0, Direction.EAST),
    (-1, 0, Direction.WEST),
    (0, 1, Direction.NORTH),
    (0, -1, Direction.SOUTH),
]

# ============================================================================
# TURN_FACTOR: How expensive a 90° turn is relative to moving 1 cell (10cm).
#
# The A* cost formula is: rotation_cost * TURN_FACTOR + 1 + safe_cost
#   - Straight move:  cost = 0*TF + 1 = 1
#   - 90° turn move:  cost = 2*TF + 1
#
# TUNING GUIDE (measure your real robot):
#   1. Time how long 10cm straight takes (e.g., 0.5s)
#   2. Time how long a 90° turn takes (e.g., 2.0s)
#   3. TURN_FACTOR = (turn_seconds / straight_seconds - 1) / 2
#      Example: (2.0/0.5 - 1) / 2 = 1.5
#
# Values:
#   TF=1:   turn costs 3x a straight (fast turns)
#   TF=2:   turn costs 5x (moderate)
#   TF=3:   turn costs 7x (default - balanced)
#   TF=4+:  turn costs 9x+ (very slow turns, avoids them heavily)
# ============================================================================
TURN_FACTOR = 3

EXPANDED_CELL = 1

WIDTH = 20
HEIGHT = 20

ITERATIONS = 2000
TURN_RADIUS = 1

SAFE_COST = 1000
SCREENSHOT_COST = 50

# ============================================================================
# FOV-based capture constants (Pi Camera v2.1)
# ============================================================================
CAMERA_HFOV_DEG = 62.2
CAMERA_HFOV_RAD = math.radians(CAMERA_HFOV_DEG)
CAMERA_HALF_FOV_DEG = CAMERA_HFOV_DEG / 2.0
CAMERA_HALF_FOV_RAD = CAMERA_HFOV_RAD / 2.0

# Max capture distance in grid cells (30cm / 10cm = 3 cells)
CAMERA_MAX_DIST_CELLS = 3

# Penalties based on micro-turn angle needed (must be low vs movement costs)
FOV_PENALTY_TINY_TURN = 0    # < 10 degrees
FOV_PENALTY_SMALL_TURN = 1   # 10-20 degrees
FOV_PENALTY_MEDIUM_TURN = 3  # 20-80 degrees