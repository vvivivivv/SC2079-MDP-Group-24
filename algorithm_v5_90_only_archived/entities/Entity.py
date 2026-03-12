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
# Must stay within SNAP_MAX_INCIDENCE_DEG (25 deg) to ensure the obstacle
# face is clearly visible and readable.  Tighter angles give more direct
# line-of-sight to the face at the cost of fewer candidate positions.
APPROACH_ANGLES_DEG = [-20, -10, 0, 10, 20]

# Safety: virtual obstacle radius
# Conservative value (23cm) keeps the robot body well clear of obstacles
# during capture positioning.  Tighter values (e.g. 21cm) give more
# candidate positions but risk clipping obstacles with positioning error.
VIRTUAL_OBS_HALF_CM = 23.0

# Penalty weights (lower = preferred)
PENALTY_CENTER = 0
PENALTY_MILD_ANGLE = 2      # 0-15 deg (excellent)
PENALTY_MODERATE_ANGLE = 12  # 15-30 deg (risky — TSP should prefer direct)
PENALTY_STEEP_ANGLE = 25     # 30-45 deg (last resort only)
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
        """Generate view cone capture candidates with cardinal headings only.

        The robot can only face 4 cardinal directions. For each obstacle face,
        the robot faces the direction matching the face normal (camera looks
        at the face from the approach side).

        Lateral offsets provide position variety while keeping heading fixed.
        """
        cells = []
        face_x, face_y = self._get_face_center_cm()
        normal_rad = self._get_face_normal_rad()
        obs_cx = self.x * 10 + 5
        obs_cy = self.y * 10 + 5

        # Robot heading = TOWARD the face (opposite of face normal)
        # Face normal points outward; robot faces inward toward the face
        heading_rad = normal_rad + math.pi
        # Normalize to [-pi, pi]
        heading_rad = (heading_rad + math.pi) % (2 * math.pi) - math.pi

        # Cardinal direction for the robot heading (opposite of face direction)
        if self.direction == Direction.NORTH:
            cardinal = Direction.SOUTH   # face North → robot faces South
        elif self.direction == Direction.SOUTH:
            cardinal = Direction.NORTH   # face South → robot faces North
        elif self.direction == Direction.EAST:
            cardinal = Direction.WEST    # face East → robot faces West
        elif self.direction == Direction.WEST:
            cardinal = Direction.EAST    # face West → robot faces East
        else:
            return cells

        distances = FACE_DISTANCES_CM
        lateral_offsets_cm = [-20, -10, 0, 10, 20]

        if retrying:
            distances = [15, 20, 25, 30, 40]
            lateral_offsets_cm = [-25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25]

        # Approach direction = face normal (outward, away from obstacle)
        cos_n = math.cos(normal_rad)
        sin_n = math.sin(normal_rad)
        cos_h = math.cos(heading_rad)
        sin_h = math.sin(heading_rad)

        for face_dist in distances:
            if face_dist < SNAP_MIN_DIST_CM or face_dist > SNAP_MAX_DIST_CM:
                continue

            for lat_offset in lateral_offsets_cm:
                # Camera position: face_dist along the normal (outward from face),
                # plus lateral offset perpendicular to the approach direction
                cam_x = face_x + face_dist * cos_n + lat_offset * (-sin_n)
                cam_y = face_y + face_dist * sin_n + lat_offset * cos_n

                # Robot CENTER is 15cm behind camera along heading
                robot_x_cm = cam_x - CAMERA_OFFSET_CM * cos_h
                robot_y_cm = cam_y - CAMERA_OFFSET_CM * sin_h

                # Incidence angle from lateral offset
                incidence_rad = math.atan2(abs(lat_offset), face_dist)
                incidence_deg = math.degrees(incidence_rad)

                if incidence_deg > SNAP_MAX_INCIDENCE_DEG:
                    continue

                # Distance from own obstacle center
                d_to_self = math.sqrt((robot_x_cm - obs_cx)**2 +
                                       (robot_y_cm - obs_cy)**2)
                if d_to_self < VIRTUAL_OBS_HALF_CM:
                    continue

                # Arena bounds
                ROBOT_HALF_CM = 18.0
                ARENA_CM = 200.0
                if (robot_x_cm < ROBOT_HALF_CM or robot_x_cm > ARENA_CM - ROBOT_HALF_CM or
                    robot_y_cm < ROBOT_HALF_CM or robot_y_cm > ARENA_CM - ROBOT_HALF_CM):
                    continue

                # Grid coordinates
                robot_gx = robot_x_cm / 10.0
                robot_gy = robot_y_cm / 10.0

                # Penalty
                penalty = 0
                if incidence_deg > 30:
                    penalty += PENALTY_STEEP_ANGLE
                elif incidence_deg > 15:
                    penalty += PENALTY_MODERATE_ANGLE
                elif incidence_deg > 5:
                    penalty += PENALTY_MILD_ANGLE
                else:
                    penalty += PENALTY_CENTER

                if face_dist > 30:
                    penalty += PENALTY_FAR
                elif face_dist < 18:
                    penalty += PENALTY_VERY_CLOSE
                elif face_dist < 22:
                    penalty += PENALTY_CLOSE

                # Wall proximity penalty
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
        """Get valid capture candidates for each obstacle.
        
        Filters include:
          1. Robot body must not overlap other obstacles' virtual zones
          2. Camera must have clear line-of-sight to the target face
             (no other obstacle's physical block in the way)
        """
        all_positions = []

        for obstacle in self.obstacles:
            if obstacle.direction == Direction.SKIP:
                continue

            candidates = obstacle.get_view_state(retrying)
            face_x, face_y = obstacle._get_face_center_cm()

            # Filter against OTHER obstacles
            valid = []
            for cand in candidates:
                blocked = False

                # Camera position: 15cm ahead of robot center along heading
                cam_x = cand.x * 10 + CAMERA_OFFSET_CM * math.cos(cand.theta_rad)
                cam_y = cand.y * 10 + CAMERA_OFFSET_CM * math.sin(cand.theta_rad)

                for other in self.obstacles:
                    if other.obstacle_id == obstacle.obstacle_id:
                        continue

                    other_cx = other.x * 10 + 5
                    other_cy = other.y * 10 + 5
                    robot_cx = cand.x * 10
                    robot_cy = cand.y * 10

                    # Check 1: robot body vs other obstacle virtual zone
                    # Conservative clearance (+9cm) prevents the robot from
                    # squeezing between obstacles where positioning error
                    # could cause a collision.
                    dist = math.sqrt((robot_cx - other_cx)**2 +
                                     (robot_cy - other_cy)**2)
                    if dist < VIRTUAL_OBS_HALF_CM + 9:
                        blocked = True
                        break

                    # Check 2: line-of-sight from camera to target face
                    # Must not pass through other obstacle's physical block.
                    # The physical obstacle is a 10x10cm square centered at
                    # (other_cx, other_cy).  We check if the line segment
                    # from camera to face passes within OBS_BLOCK_RADIUS
                    # of the other obstacle center.
                    OBS_BLOCK_RADIUS = 8.0  # ~half-diagonal of 10x10cm block
                    if _line_segment_circle_intersect(
                            cam_x, cam_y, face_x, face_y,
                            other_cx, other_cy, OBS_BLOCK_RADIUS):
                        blocked = True
                        break

                if not blocked:
                    valid.append(cand)

            all_positions.append(valid)

        return all_positions


def _line_segment_circle_intersect(x1, y1, x2, y2, cx, cy, r):
    """Check if line segment (x1,y1)-(x2,y2) passes within radius r of (cx,cy).
    
    Uses closest-point-on-segment to circle center.
    Returns True if the line of sight is blocked.
    """
    dx = x2 - x1
    dy = y2 - y1
    fx = x1 - cx
    fy = y1 - cy

    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq < 1e-9:
        # Degenerate segment (camera on top of face)
        return (fx * fx + fy * fy) < r * r

    # Parameter t of closest point on the infinite line
    t = -(fx * dx + fy * dy) / seg_len_sq
    # Clamp to segment [0, 1]
    t = max(0.0, min(1.0, t))

    closest_x = x1 + t * dx
    closest_y = y1 + t * dy
    dist_sq = (closest_x - cx) ** 2 + (closest_y - cy) ** 2

    return dist_sq < r * r