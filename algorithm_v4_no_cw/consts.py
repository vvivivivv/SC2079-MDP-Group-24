import math
from enum import Enum

# ============================================================================
# 1. PHYSICAL ROBOT CONSTANTS (MEASURE THESE ON REAL BOT!)
# ============================================================================

ROBOT_AXLE_TRACK_CM = 16.0
ROBOT_WHEELBASE_CM = 20.0

# Asymmetric turning radii — MEASURE ALL FOUR on the real robot!
# Forward and backward arcs can have different effective radii due to
# tire friction, weight distribution, motor behaviour, and mechanical slop.
# Measure each by driving a full circle and computing R = circumference / (2*pi).
#
# NOTE on naming: "BL" means the FRONT of the car swings LEFT while reversing.
#   FL = forward, front swings left    (left steering, forward gear)
#   FR = forward, front swings right   (right steering, forward gear)
#   BL = backward, front swings left   (right steering, backward gear)
#   BR = backward, front swings right  (left steering, backward gear)
ROBOT_TURN_RADIUS_FL_CM = 29.4 #29.4   # Forward-Left arc radius  (tune this!)
ROBOT_TURN_RADIUS_FR_CM = 30.6  #30.6   # Forward-Right arc radius (tune this!)
ROBOT_TURN_RADIUS_BL_CM = 29.5 #30.2   # Back-Left arc radius     (tune this!)
ROBOT_TURN_RADIUS_BR_CM = 28.6 #28.3   # Back-Right arc radius    (tune this!)

# Derived values used throughout the planner:
#   MAX: conservative — used for RS direct paths & collision checking
#        (guarantees the robot fits through gaps)
#   MIN: aggressive — used for RS heuristic (admissible lower bound)
#        (the robot CAN turn this tight in at least one direction)
_ALL_RADII = [ROBOT_TURN_RADIUS_FL_CM, ROBOT_TURN_RADIUS_FR_CM,
              ROBOT_TURN_RADIUS_BL_CM, ROBOT_TURN_RADIUS_BR_CM]
ROBOT_TURN_RADIUS_MAX_CM = max(_ALL_RADII)
ROBOT_TURN_RADIUS_MIN_CM = min(_ALL_RADII)

# ============================================================================
# 1a. EMPIRICAL DISTANCE CALIBRATION
# ============================================================================
# The planner computes distances at the robot CENTER in cm.  The real robot's
# encoder + motor + braking behaviour means the actual distance traveled
# differs from what is commanded.  These constants correct for that.
#
# HOW TO CALIBRATE (see calibration_guide.html):
#   1. Send each command type at 2-3 different distances
#   2. Measure actual vs expected distance/position
#   3. If the error is constant (same cm short every time):
#        set OFFSET_xx = shortfall in mm  (e.g. 18 = 1.8cm)
#   4. If the error is proportional (same % short every time):
#        set SCALE_xx = expected / actual  (e.g. 1.035 = 3.5% short)
#   5. If both: set both.
#
# Formula applied:  commanded_mm = center_arc_mm * SCALE + OFFSET
#
# Set SCALE=1.0 and OFFSET=0 to disable correction (uncalibrated default).

SCALE_FW = 1.0;  OFFSET_FW = 0   # Forward straight
SCALE_BW = 1.0;  OFFSET_BW = 0   # Backward straight
SCALE_FL = 1.0;  OFFSET_FL = 0   # Forward-Left arc
SCALE_FR = 1.0;  OFFSET_FR = 0   # Forward-Right arc
SCALE_BL = 1.0;  OFFSET_BL = 0   # Backward-Left arc
SCALE_BR = 1.0;  OFFSET_BR = 0   # Backward-Right arc

ROBOT_SPEED_CM_S = 30.0
ROBOT_LENGTH_CM = 30.0
ROBOT_WIDTH_CM = 30.0
ROBOT_RADIUS_CM = math.sqrt((ROBOT_LENGTH_CM/2)**2 + (ROBOT_WIDTH_CM/2)**2)

# ============================================================================
# 1b. CAMERA SNAP CONSTRAINTS
# ============================================================================
# Minimum face-to-camera distance for a readable image.
# Below this the face is too close / blurry / partially out of frame.
SNAP_MIN_DIST_CM = 17.5

# Maximum face-to-camera distance (beyond this, resolution is too low)
SNAP_MAX_DIST_CM = 50.0

# Pi Camera v2.1: 62.2 deg HFOV.  With tighter approach angles (±20°),
# the face is always well-centered in frame and clearly readable.
# 25° max incidence gives a good balance between safety and flexibility.
SNAP_MAX_INCIDENCE_DEG = 25.0

# ============================================================================
# 2. PIVOT-ON-BACK-LEFT-WHEEL GEOMETRY
# ============================================================================
# When spinning CW, the robot pivots on its back-left wheel, NOT its center.
# This means every spin also MOVES the robot center along a circular arc.
#
# Back-left wheel offset from robot center in ROBOT FRAME:
#   robot-x = forward, robot-y = left
#   back = -WHEELBASE/2, left = +AXLE_TRACK/2
#
# Pivot radius = distance from center to back-left wheel
#   = sqrt((WHEELBASE/2)^2 + (AXLE_TRACK/2)^2)

PIVOT_OFFSET_X = -ROBOT_WHEELBASE_CM / 2.0   # -10cm (behind center)
PIVOT_OFFSET_Y = +ROBOT_AXLE_TRACK_CM / 2.0  # +8cm  (left of center)
PIVOT_RADIUS_CM = math.sqrt(PIVOT_OFFSET_X**2 + PIVOT_OFFSET_Y**2)  # ~12.81cm

# Angle from robot forward axis to the pivot point (in robot frame)
PIVOT_ANGLE_RAD = math.atan2(PIVOT_OFFSET_Y, PIVOT_OFFSET_X)  # ~141.3 deg

# ============================================================================
# 3. ARENA & GRID CONSTANTS
# ============================================================================
WIDTH = 20
HEIGHT = 20
CELL_SIZE_CM = 10

ITERATIONS = 2000
EXPANDED_CELL = 1

SAFE_COST = 1000
SCREENSHOT_COST = 50
TURN_FACTOR = 3.0

# ============================================================================
# 4. CAMERA & FOV PARAMETERS (Pi Camera v2.1)
# ============================================================================
CAMERA_HFOV_DEG = 62.2
CAMERA_HFOV_RAD = math.radians(CAMERA_HFOV_DEG)
CAMERA_HALF_FOV_DEG = CAMERA_HFOV_DEG / 2.0
CAMERA_HALF_FOV_RAD = CAMERA_HFOV_RAD / 2.0
CAMERA_MAX_DIST_CELLS = 3

FOV_PENALTY_TINY_TURN = 0
FOV_PENALTY_SMALL_TURN = 1
FOV_PENALTY_MEDIUM_TURN = 3

# ============================================================================
# 5. SPOT TURN PHYSICS
# ============================================================================
# Only CW spins are used (robot is unreliable CCW).
SPIN_SPEED_DEG_S = 120.0

# Cost of stopping + starting (acceleration overhead per spin) in cm-equivalent.
# Empirically: ~0.3-0.5s per stop-start * 30cm/s = 9-15cm equivalent
STOP_START_PENALTY_CM = 12.0

# ============================================================================
# 6. ENUMS & HELPERS
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
        mapping = {0: 90.0, 2: 0.0, 4: 270.0, 6: 180.0}
        return mapping[int(d)]

    @staticmethod
    def to_angle_rad(d):
        return math.radians(Direction.to_angle_deg(d))


MOVE_DIRECTION = [
    (1, 0, Direction.EAST),
    (-1, 0, Direction.WEST),
    (0, 1, Direction.NORTH),
    (0, -1, Direction.SOUTH),
]