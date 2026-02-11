"""
Utility functions shared by Entity.py and algo.py.
The main pipeline (TSP + Hybrid A* + command generation) is in algo.py.
"""

import math
from consts import WIDTH, HEIGHT, Direction, EXPANDED_CELL


def is_valid(center_x: int, center_y: int):
    """Check if grid position is within arena bounds (with robot clearance)."""
    return center_x > 0 and center_y > 0 and center_x < WIDTH - 1 and center_y < HEIGHT - 1


def compute_micro_turn_angle(robot_x, robot_y, robot_dir, ob_x, ob_y, ob_dir):
    """Compute the micro-turn angle from robot heading to obstacle face.

    Returns:
        (abs_angle_degrees, signed_degrees, signal)
        signed_degrees: positive = turn left, negative = turn right
        signal: 'L', 'C', or 'R'
    """
    face_offsets = {
        0: (0, 0.5 + EXPANDED_CELL * 0.5),    # NORTH
        2: (0.5 + EXPANDED_CELL * 0.5, 0),     # EAST
        4: (0, -0.5 - EXPANDED_CELL * 0.5),    # SOUTH
        6: (-0.5 - EXPANDED_CELL * 0.5, 0),    # WEST
    }
    fx_off, fy_off = face_offsets.get(ob_dir, (0, 0))
    face_x = ob_x + fx_off
    face_y = ob_y + fy_off

    dx = face_x - robot_x
    dy = face_y - robot_y

    angle_to_face = math.atan2(dy, dx)
    robot_angle = Direction.to_angle_rad(robot_dir)

    diff_rad = angle_to_face - robot_angle
    diff_rad = (diff_rad + math.pi) % (2 * math.pi) - math.pi
    diff_deg = math.degrees(diff_rad)
    abs_angle = abs(diff_deg)

    if abs_angle < 3.0:
        signal = 'C'
    elif diff_deg > 0:
        signal = 'L'
    else:
        signal = 'R'

    return abs_angle, diff_deg, signal