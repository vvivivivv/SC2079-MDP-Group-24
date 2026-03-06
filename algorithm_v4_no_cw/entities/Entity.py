"""
Entity.py - Arena objects: CellState, Obstacle, Grid.

View Cone Generation (updated for wide-angle capture):
  - Camera HFOV: 62.2 deg (Pi Camera v2.1)
  - Max incidence angle: 45 deg (camera must face obstacle fairly directly)
  - Detection range: 15-50cm face-to-camera
  - Robot footprint: 30x30cm (virtual), camera at front center
  - Obstacle: 10x10cm block, virtual obstacle 40x40cm (slide 36)
  - Virtual obstacle radius: 20cm from obstacle center

Wide-angle capture is valuable because:
  - More candidate positions => better TSP solutions
  - Positions near walls/corners become reachable
  - Robot can approach from many angles, reducing detour distance
"""

import math
from typing import List
from consts import (
    Direction, SNAP_MIN_DIST_CM, SNAP_MAX_DIST_CM,
    SNAP_MAX_INCIDENCE_DEG
)
from helper import is_valid


# =============================================================================
# VIEW CONE PARAMETERS
# =============================================================================

# Face-to-CAMERA distances (cm).
# The camera is at the front center of the robot, 15cm ahead of robot center.
# Robot center is placed face_dist + CAMERA_OFFSET further from the face.
#   dist=15cm => center 30cm from face => 35cm from obs center
#   dist=20cm => center 35cm from face => 40cm from obs center
#   dist=30cm => center 45cm from face => 50cm from obs center
FACE_DISTANCES_CM = [15, 20, 25, 30]

# Camera offset from robot center (half the robot length)
CAMERA_OFFSET_CM = 15.0

# Approach angle offsets from the face normal (degrees).
# With 75 deg max incidence and wide HFOV, we can approach from steep angles.
APPROACH_ANGLES_DEG = [-35, -30, -15, 0, 15, 30, 35]

# Safety: virtual obstacle radius
VIRTUAL_OBS_HALF_CM = 21.0

# Penalty weights (lower = preferred)
PENALTY_CENTER = 0
PENALTY_MILD_ANGLE = 1      # 0-15 deg (excellent)
PENALTY_MODERATE_ANGLE = 2   # 15-30 deg (good)
PENALTY_STEEP_ANGLE = 4      # 30-45 deg (angled but readable)
PENALTY_FAR = 3
PENALTY_CLOSE = 0
PENALTY_VERY_CLOSE = 2       # < 18cm (tight but works)

# Wall proximity penalty
PENALTY_WALL_CRITICAL = 15
PENALTY_WALL_TIGHT = 8
PENALTY_WALL_SNUG = 3


class CellState:
    """Base class for all objects on the arena."""

    def __init__(self, x, y, direction=None, screenshot_id=-1,
                 penalty=0, theta_rad=None):
        if direction is None:
            direction = Direction.NORTH
        self.x = x
        self.y = y
        self.direction = direction
        self.screenshot_id = screenshot_id
        self.penalty = penalty
        self.theta_rad = theta_rad

    def cmp_position(self, x, y):
        return self.x == x and self.y == y

    def is_eq(self, x, y, direction):
        return self.x == x and self.y == y and self.direction == direction

    def __repr__(self):
        return (f"x: {self.x:.1f}, y: {self.y:.1f}, d: {self.direction}, "
                f"screenshot: {self.screenshot_id}")

    def set_screenshot(self, screenshot_id):
        self.screenshot_id = screenshot_id

    def get_dict(self):
        return {
            'x': self.x, 'y': self.y, 'd': int(self.direction),
            's': self.screenshot_id,
            'theta': self.theta_rad if hasattr(self, 'theta_rad') and self.theta_rad is not None else None
        }


class Obstacle(CellState):
    """Obstacle with view cone capture position generation."""

    def __init__(self, x, y, direction, obstacle_id):
        super().__init__(x, y, direction)
        self.obstacle_id = obstacle_id

    def __eq__(self, other):
        return (self.x == other.x and self.y == other.y and
                self.direction == other.direction)

    def _get_face_center_cm(self):
        cx = self.x * 10 + 5
        cy = self.y * 10 + 5
        if self.direction == Direction.NORTH:
            return cx, cy + 5
        elif self.direction == Direction.SOUTH:
            return cx, cy - 5
        elif self.direction == Direction.EAST:
            return cx + 5, cy
        elif self.direction == Direction.WEST:
            return cx - 5, cy
        return cx, cy

    def _get_face_normal_rad(self):
        if self.direction == Direction.NORTH:
            return math.pi / 2
        elif self.direction == Direction.SOUTH:
            return -math.pi / 2
        elif self.direction == Direction.EAST:
            return 0.0
        elif self.direction == Direction.WEST:
            return math.pi
        return 0.0

    def get_view_state(self, retrying):
        """Generate view cone capture candidates.

        With Pi Camera v2.1 wide 62.2 deg HFOV, the robot can capture
        images at steep incidence angles (up to 75 deg). Combined with
        a 15cm minimum distance, this gives many more candidate positions.
        """
        cells = []
        face_x, face_y = self._get_face_center_cm()
        normal_rad = self._get_face_normal_rad()
        obs_cx = self.x * 10 + 5
        obs_cy = self.y * 10 + 5

        distances = FACE_DISTANCES_CM
        angles = APPROACH_ANGLES_DEG

        if retrying:
            distances = [15, 20, 25, 30, 40]
            angles = [-35, -25, -15, -5, 0, 5, 15, 25, 35]

        for face_dist in distances:
            if face_dist < SNAP_MIN_DIST_CM or face_dist > SNAP_MAX_DIST_CM:
                continue

            for angle_off in angles:
                angle_off_rad = math.radians(angle_off)

                # 1. Camera position at face_dist from face
                approach_rad = normal_rad + angle_off_rad
                cam_x = face_x + face_dist * math.cos(approach_rad)
                cam_y = face_y + face_dist * math.sin(approach_rad)

                # 2. Robot heading: camera toward face
                heading_rad = math.atan2(face_y - cam_y, face_x - cam_x)

                # 3. Robot CENTER is 15cm behind camera along heading
                robot_x_cm = cam_x - CAMERA_OFFSET_CM * math.cos(heading_rad)
                robot_y_cm = cam_y - CAMERA_OFFSET_CM * math.sin(heading_rad)

                # 4. Incidence angle check
                if abs(angle_off) > SNAP_MAX_INCIDENCE_DEG:
                    continue

                # 5. Distance from own obstacle center
                d_to_self = math.sqrt((robot_x_cm - obs_cx)**2 +
                                       (robot_y_cm - obs_cy)**2)
                if d_to_self < VIRTUAL_OBS_HALF_CM:
                    continue

                # 6. Arena bounds
                ROBOT_HALF_CM = 18.0
                ARENA_CM = 200.0
                if (robot_x_cm < ROBOT_HALF_CM or robot_x_cm > ARENA_CM - ROBOT_HALF_CM or
                    robot_y_cm < ROBOT_HALF_CM or robot_y_cm > ARENA_CM - ROBOT_HALF_CM):
                    continue

                # Grid coordinates
                robot_gx = robot_x_cm / 10.0
                robot_gy = robot_y_cm / 10.0

                # 7. Cardinal direction + exact theta
                heading_deg = math.degrees(heading_rad) % 360
                if 45 <= heading_deg < 135:
                    cardinal = Direction.NORTH
                elif 135 <= heading_deg < 225:
                    cardinal = Direction.WEST
                elif 225 <= heading_deg < 315:
                    cardinal = Direction.SOUTH
                else:
                    cardinal = Direction.EAST

                # 8. Penalty
                penalty = 0
                abs_angle = abs(angle_off)
                if abs_angle > 30:
                    penalty += PENALTY_STEEP_ANGLE
                elif abs_angle > 15:
                    penalty += PENALTY_MODERATE_ANGLE
                elif abs_angle > 5:
                    penalty += PENALTY_MILD_ANGLE
                else:
                    penalty += PENALTY_CENTER

                if face_dist > 30:
                    penalty += PENALTY_FAR
                elif face_dist < 18:
                    penalty += PENALTY_VERY_CLOSE
                elif face_dist < 22:
                    penalty += PENALTY_CLOSE

                # 9. Wall proximity penalty
                wall_margin = min(
                    robot_x_cm - ROBOT_HALF_CM,
                    ARENA_CM - ROBOT_HALF_CM - robot_x_cm,
                    robot_y_cm - ROBOT_HALF_CM,
                    ARENA_CM - ROBOT_HALF_CM - robot_y_cm
                )
                if wall_margin < 10:
                    penalty += PENALTY_WALL_CRITICAL
                elif wall_margin < 20:
                    penalty += PENALTY_WALL_TIGHT
                elif wall_margin < 30:
                    penalty += PENALTY_WALL_SNUG

                cell = CellState(
                    robot_gx, robot_gy, cardinal,
                    self.obstacle_id, penalty,
                    theta_rad=heading_rad
                )
                cells.append(cell)

        cells.sort(key=lambda c: c.penalty)

        # More candidates (wider angles), but cap for TSP
        MAX_CANDIDATES = 15
        if len(cells) > MAX_CANDIDATES:
            cells = cells[:MAX_CANDIDATES]

        return cells


class Grid:
    """Arena grid with obstacles."""

    def __init__(self, size_x, size_y):
        self.size_x = size_x
        self.size_y = size_y
        self.obstacles = []

    def add_obstacle(self, obstacle):
        for ob in self.obstacles:
            if ob == obstacle:
                return
        self.obstacles.append(obstacle)

    def reset_obstacles(self):
        self.obstacles = []

    def get_obstacles(self):
        return self.obstacles

    def is_valid_coord(self, x, y):
        return (1 <= x <= self.size_x - 2 and 1 <= y <= self.size_y - 2)

    def get_view_obstacle_positions(self, retrying):
        """Get valid capture candidates for each obstacle."""
        all_positions = []

        for obstacle in self.obstacles:
            if obstacle.direction == Direction.SKIP:
                continue

            candidates = obstacle.get_view_state(retrying)

            # Filter against OTHER obstacles safety zones
            valid = []
            for cand in candidates:
                blocked = False
                for other in self.obstacles:
                    if other.obstacle_id == obstacle.obstacle_id:
                        continue

                    other_cx = other.x * 10 + 5
                    other_cy = other.y * 10 + 5
                    robot_cx = cand.x * 10
                    robot_cy = cand.y * 10

                    dist = math.sqrt((robot_cx - other_cx)**2 +
                                     (robot_cy - other_cy)**2)

                    if dist < VIRTUAL_OBS_HALF_CM + 5:
                        blocked = True
                        break

                if not blocked:
                    valid.append(cand)

            all_positions.append(valid)

        return all_positions