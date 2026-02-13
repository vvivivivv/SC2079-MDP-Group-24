import math
from enum import Enum

# ============================================================================
# 1. PHYSICAL ROBOT CONSTANTS (MEASURE THESE ON REAL BOT!)
# ============================================================================
# These are the "Golden Numbers". If you change hardware, change ONLY here.

# Distance from center of left wheel to center of right wheel (cm)
# Used for: Differential drive kinematics in simulator & Hybrid A*
ROBOT_AXLE_TRACK_CM = 16.0  

ROBOT_WHEELBASE_CM = 20.0  # Distance from front axle to rear axle (Measure this!)

# Minimum turning radius (cm)
# Used for: Reeds-Shepp paths & Hybrid A* collision checking
# Calculate as: ROBOT_AXLE_TRACK_CM / 2 * (MaxSpeed + MinSpeed) / (MaxSpeed - MinSpeed)
# OR just measure it empirically by driving in a circle.
ROBOT_TURN_RADIUS_CM = 25

# Robot Speed (cm/s)
# Used for: Physics integration step size
ROBOT_SPEED_CM_S = 30.0

# Robot Dimensions (cm)
# Used for: Collision checking & safety buffers
ROBOT_LENGTH_CM = 30.0
ROBOT_WIDTH_CM = 30.0
# Distance from robot center to farthest corner (hypotenuse of 15x15)
ROBOT_RADIUS_CM = math.sqrt((ROBOT_LENGTH_CM/2)**2 + (ROBOT_WIDTH_CM/2)**2) # ~21.21cm

# ============================================================================
# 2. ARENA & GRID CONSTANTS
# ============================================================================
WIDTH = 20
HEIGHT = 20
CELL_SIZE_CM = 10  # Size of one grid cell in cm

# A* Search limits
ITERATIONS = 2000 # Limit for Hybrid A* expansions
EXPANDED_CELL = 1 # Grid inflation (integer cells)

# Cost Weights
SAFE_COST = 1000
SCREENSHOT_COST = 50

# Turn Cost Factor
# How expensive is a turn vs a straight? 
# 1.0 = equal cost, 3.0 = turn is 3x more "expensive" than straight
TURN_FACTOR = 3.0

# ============================================================================
# 3. CAMERA & FOV PARAMETERS (Pi Camera v2.1)
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


# ============================================================================
# 4. ENUMS & HELPERS
# ============================================================================
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