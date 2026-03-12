import math
from enum import Enum

# ============================================================================
# 1. PHYSICAL ROBOT CONSTANTS (MEASURE THESE ON REAL BOT!)
# ============================================================================

ROBOT_AXLE_TRACK_CM = 16.0
ROBOT_WHEELBASE_CM = 20.0

# 90-DEGREE TURN DISPLACEMENT MODEL
# Each 90-degree turn has a fixed (DX, DY) displacement in the robot's
# ORIGINAL frame (before the turn).
#   DX = displacement along the original heading (positive = forward)
#   DY = displacement perpendicular (positive = left of original heading)
#
# Measure by commanding each turn on the real robot and measuring the
# center-to-center displacement.
#
# Heading changes:
#   FL90: +90 deg (CCW)    FR90: -90 deg (CW)
#   BL90: +90 deg (CCW)    BR90: -90 deg (CW)

TURN_FL90_DX_CM = 38.5    # forward displacement during FL90 (tune!)
TURN_FL90_DY_CM = 24.5    # leftward displacement during FL90 (tune!)

TURN_FR90_DX_CM = 39    # forward displacement during FR90 (tune!)
TURN_FR90_DY_CM = -25.5   # rightward displacement during FR90 (negative = right)

TURN_BL90_DX_CM = -39   # backward displacement during BL90 (tune!)
TURN_BL90_DY_CM = 25.0    # leftward displacement during BL90 (tune!)

TURN_BR90_DX_CM = -38.5   # backward displacement during BR90 (tune!)
TURN_BR90_DY_CM = -24   # rightward displacement during BR90 (tune!)

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
SNAP_MAX_DIST_CM = 40.0

# Pi Camera v2.1: 62.2 deg HFOV.  With tighter approach angles (±20°),
# the face is always well-centered in frame and clearly readable.
# 20° max incidence keeps the face well-centered and reduces risk of
# missing the image due to positioning drift on the real robot.
SNAP_MAX_INCIDENCE_DEG = 20.0

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