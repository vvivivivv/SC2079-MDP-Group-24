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

# Legacy aliases for code that only needs left/right (e.g. simulator steering)
ROBOT_TURN_RADIUS_LEFT_CM = ROBOT_TURN_RADIUS_FL_CM
ROBOT_TURN_RADIUS_RIGHT_CM = ROBOT_TURN_RADIUS_FR_CM

# Derived values used throughout the planner:
#   MAX: conservative — used for RS direct paths & collision checking
#        (guarantees the robot fits through gaps)
#   MIN: aggressive — used for RS heuristic (admissible lower bound)
#        (the robot CAN turn this tight in at least one direction)
_ALL_RADII = [ROBOT_TURN_RADIUS_FL_CM, ROBOT_TURN_RADIUS_FR_CM,
              ROBOT_TURN_RADIUS_BL_CM, ROBOT_TURN_RADIUS_BR_CM]
ROBOT_TURN_RADIUS_MAX_CM = max(_ALL_RADII)
ROBOT_TURN_RADIUS_MIN_CM = min(_ALL_RADII)
# Legacy alias (use specific FL/FR/BL/BR/max/min where possible)
ROBOT_TURN_RADIUS_CM = ROBOT_TURN_RADIUS_MAX_CM

# ============================================================================
# 1a. ENCODER CORRECTION FOR ARC COMMANDS
# ============================================================================
# The planner computes arc distances at the robot CENTER, but the real robot
# stops based on its encoder which measures at a WHEEL — not the center.
#
# During a turn, the outer wheel travels farther and the inner wheel travels
# shorter than the center arc.  The ratio is:
#   encoder_dist = center_dist × (R ± track/2) / R
#     + for outer wheel, − for inner wheel
#
# HOW TO CALIBRATE:
#   1. Command the robot to do FR with a large distance (e.g. FR1800 = 180cm)
#   2. Measure the actual angle turned (should be ~360° for a full circle)
#   3. Read the encoder value when the robot completes 360°
#   4. The scale factor = encoder_reading / commanded_distance
#   5. Put that value below for the corresponding direction
#
# Set to 1.0 to disable correction (encoder = center arc).
_HALF_TRACK = ROBOT_AXLE_TRACK_CM / 2.0  # 8.0cm

# Default: assume encoder on LEFT rear wheel
#   - FR/BR (right turn): left wheel is OUTER → factor > 1
#   - FL/BL (left turn):  left wheel is INNER → factor < 1
ENCODER_SCALE_FL = (ROBOT_TURN_RADIUS_FL_CM - _HALF_TRACK) / ROBOT_TURN_RADIUS_FL_CM
ENCODER_SCALE_FR = (ROBOT_TURN_RADIUS_FR_CM + _HALF_TRACK) / ROBOT_TURN_RADIUS_FR_CM
ENCODER_SCALE_BL = (ROBOT_TURN_RADIUS_BL_CM - _HALF_TRACK) / ROBOT_TURN_RADIUS_BL_CM
ENCODER_SCALE_BR = (ROBOT_TURN_RADIUS_BR_CM + _HALF_TRACK) / ROBOT_TURN_RADIUS_BR_CM
# FW/BW: no correction (both wheels travel the same distance)
ENCODER_SCALE_FW = 1.0
ENCODER_SCALE_BW = 1.0

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

# Pi Camera v2.1: 62.2 deg HFOV.  The face just needs to be within the
# camera's field of view.  Since the camera always points at the face,
# the limiting factor is how foreshortened the face becomes at extreme
# incidence angles — but the wide FOV means we can approach from steep
# angles and still capture the whole face.
# Practical max incidence: ~75 deg (face is ~26% projected width, still
# readable by the classifier for large block digits/arrows).
SNAP_MAX_INCIDENCE_DEG = 35.0

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