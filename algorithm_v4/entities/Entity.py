"""
Entity.py — Arena objects: CellState, Obstacle, Grid.

View Cone Generation (from PDF briefing + hardware specs):
  - Camera HFOV: 62.2° (Pi Camera v2.1)
  - Max incidence angle: 45° (camera angle to face normal)
  - Detection range: 20–50cm face-to-face
  - Robot footprint: 30x30cm (virtual), camera at front center
  - Obstacle: 10x10cm block, virtual obstacle 40x40cm (slide 36)
  - Virtual obstacle radius: 20cm from obstacle center (slide 36)

For each obstacle, the image face defines a view cone. We generate
candidates at multiple approach angles (left, center, right) and
distances so the TSP solver can pick the one that minimizes Dubins
path length from the previous node.
"""

import math
from typing import List
from consts import Direction
from helper import is_valid


# =============================================================================
# VIEW CONE PARAMETERS
# =============================================================================

# Face-to-camera distances (cm). Slide 4: "best position 20cm away".
# We measure from the IMAGE FACE to the camera (robot front center).
# Robot center is ~15cm behind camera, so robot-center-to-face = dist + 15.
# In grid coords, obstacle center to robot center = (dist + 5) / 10 grid units.
#   dist=20cm => robot center is 25cm from face => 30cm from obs center => 3.0 grid
#   dist=25cm => robot center is 30cm from face => 35cm from obs center => 3.5 grid
#   dist=35cm => robot center is 40cm from face => 45cm from obs center => 4.5 grid
FACE_DISTANCES_CM = [20, 25, 35]

# Approach angle offsets from the face normal (degrees).
# 0° = dead center, ±30° = angled, ±45° = max incidence angle.
# Wider angles give more room when obstacle faces a nearby wall.
APPROACH_ANGLES_DEG = [-45, -30, 0, 30, 45]

# Safety: virtual obstacle radius (slide 36: 40x40cm => 20cm from center)
# We check robot center is at least this far from ALL obstacle centers.
VIRTUAL_OBS_HALF_CM = 21.0  # cm (min distance from own obstacle center)

# Penalty weights for TSP cost (lower = preferred)
PENALTY_CENTER = 0       # dead-center shot
PENALTY_ANGLED = 2       # 30° off-center (still very readable)
PENALTY_WIDE_ANGLE = 4   # 45° off-center (max incidence, still works)
PENALTY_FAR = 3           # further distance
PENALTY_CLOSE = 1         # 20cm (at minimum spec distance, slight risk)

# Wall proximity penalty — positions near walls cause zigzagging
# because the robot (R=25cm) can't make smooth arcs near walls.
# Minimum to complete ANY turn: 15(clearance) + 25(radius) = 40cm from wall.
# Below 30cm: heavy penalty. Below 40cm: moderate penalty.
PENALTY_WALL_CRITICAL = 15  # < 25cm from wall — almost unreachable
PENALTY_WALL_TIGHT = 8      # 25-35cm from wall — forces awkward approach
PENALTY_WALL_SNUG = 3       # 35-45cm from wall — limited approach angles


class CellState:
    """Base class for all objects on the arena.
    
    Coordinates are in GRID units (0–19 for a 20x20 grid).
    x, y can be floats for sub-cell precision.
    """

    def __init__(self, x, y, direction: Direction = Direction.NORTH,
                 screenshot_id=-1, penalty=0, theta_rad=None):
        self.x = x
        self.y = y
        self.direction = direction
        self.screenshot_id = screenshot_id
        self.penalty = penalty
        # Optional: continuous heading for Hybrid A* (radians, math convention)
        self.theta_rad = theta_rad

    def cmp_position(self, x, y) -> bool:
        return self.x == x and self.y == y

    def is_eq(self, x, y, direction):
        return self.x == x and self.y == y and self.direction == direction

    def __repr__(self):
        return (f"x: {self.x:.1f}, y: {self.y:.1f}, d: {self.direction}, "
                f"screenshot: {self.screenshot_id}")

    def set_screenshot(self, screenshot_id):
        self.screenshot_id = screenshot_id

    def get_dict(self):
        return {'x': self.x, 'y': self.y, 'd': int(self.direction),
                's': self.screenshot_id,
                'theta': self.theta_rad if hasattr(self, 'theta_rad') and self.theta_rad is not None else None}


class Obstacle(CellState):
    """Obstacle with view cone capture position generation."""

    def __init__(self, x: int, y: int, direction: Direction, obstacle_id: int):
        super().__init__(x, y, direction)
        self.obstacle_id = obstacle_id

    def __eq__(self, other):
        return (self.x == other.x and self.y == other.y and
                self.direction == other.direction)

    def _get_face_center_cm(self):
        """Get face center position in cm.
        
        Obstacle occupies a 10x10cm cell. Its center is at
        (x*10 + 5, y*10 + 5) cm. The image face is on one side,
        so the face center is 5cm outward from the obstacle center.
        """
        cx = self.x * 10 + 5  # obstacle center in cm
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
        """Get the outward normal of the image face (radians, math convention)."""
        if self.direction == Direction.NORTH:
            return math.pi / 2      # pointing up
        elif self.direction == Direction.SOUTH:
            return -math.pi / 2     # pointing down
        elif self.direction == Direction.EAST:
            return 0.0              # pointing right
        elif self.direction == Direction.WEST:
            return math.pi          # pointing left
        return 0.0

    def get_view_state(self, retrying) -> List['CellState']:
        """Generate view cone capture candidates.
        
        For each (distance, angle) combination:
        1. Position the robot along a ray from the face center
        2. Point the robot heading BACK toward the face
        3. Verify the incidence angle is ≤ 45°
        4. Verify distance from all obstacles is safe
        5. Verify position is within arena bounds
        
        Returns a list of CellState sorted by penalty (best first).
        """
        cells = []
        face_x, face_y = self._get_face_center_cm()
        normal_rad = self._get_face_normal_rad()
        obs_cx = self.x * 10 + 5  # obstacle center in cm
        obs_cy = self.y * 10 + 5
        
        distances = FACE_DISTANCES_CM
        angles = APPROACH_ANGLES_DEG
        
        if retrying:
            # Wider search on retry
            distances = [20, 25, 35, 45]
            angles = [-40, -20, 0, 20, 40]
        
        for face_dist in distances:
            for angle_off in angles:
                angle_off_rad = math.radians(angle_off)
                
                # 1. Robot position in cm
                # Place robot at distance face_dist from face center,
                # along the face normal rotated by angle_off
                robot_x_cm = face_x + face_dist * math.cos(normal_rad + angle_off_rad)
                robot_y_cm = face_y + face_dist * math.sin(normal_rad + angle_off_rad)
                
                # 2. Robot heading: point camera TOWARD the face
                # Camera is at front of robot, so heading = direction from robot to face
                heading_rad = math.atan2(face_y - robot_y_cm, face_x - robot_x_cm)
                
                # 3. Incidence angle check (should be ≤ 45°)
                # Incidence = angle between the view ray and face normal
                # For our parameterization, incidence ≈ |angle_off|
                if abs(angle_off) > 45:
                    continue
                
                # 4. Distance from THIS obstacle's center (safety check)
                d_to_self = math.sqrt((robot_x_cm - obs_cx)**2 + 
                                       (robot_y_cm - obs_cy)**2)
                if d_to_self < VIRTUAL_OBS_HALF_CM:
                    continue
                
                # 5. Arena bounds check (robot center must be ≥ 15cm from walls)
                ROBOT_HALF_CM = 15.0
                ARENA_CM = 200.0
                if (robot_x_cm < ROBOT_HALF_CM or robot_x_cm > ARENA_CM - ROBOT_HALF_CM or
                    robot_y_cm < ROBOT_HALF_CM or robot_y_cm > ARENA_CM - ROBOT_HALF_CM):
                    continue
                
                # Convert to grid coordinates for CellState
                robot_gx = robot_x_cm / 10.0
                robot_gy = robot_y_cm / 10.0
                
                # 6. Snap heading to nearest cardinal Direction for grid compatibility
                # AND store the exact theta for Hybrid A*
                heading_deg = math.degrees(heading_rad)
                heading_deg = heading_deg % 360
                
                # Map to nearest cardinal direction
                if 45 <= heading_deg < 135:
                    cardinal = Direction.NORTH
                elif 135 <= heading_deg < 225:
                    cardinal = Direction.WEST
                elif 225 <= heading_deg < 315:
                    cardinal = Direction.SOUTH
                else:
                    cardinal = Direction.EAST
                
                # 7. Penalty: center is best, angled slightly worse, far slightly worse
                penalty = 0
                if abs(angle_off) > 35:
                    penalty += PENALTY_WIDE_ANGLE
                elif abs(angle_off) > 5:
                    penalty += PENALTY_ANGLED
                if face_dist > 30:
                    penalty += PENALTY_FAR
                elif face_dist < 22:
                    penalty += PENALTY_CLOSE
                
                # 8. Wall proximity penalty — the big one for path quality.
                # A car with R=25cm turning radius needs space to maneuver.
                # Positions near walls force the A* into zigzag paths.
                wall_margin = min(
                    robot_x_cm - ROBOT_HALF_CM,           # left wall
                    ARENA_CM - ROBOT_HALF_CM - robot_x_cm, # right wall  
                    robot_y_cm - ROBOT_HALF_CM,            # bottom wall
                    ARENA_CM - ROBOT_HALF_CM - robot_y_cm  # top wall
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
        
        # Limit to best candidates to keep TSP tractable
        MAX_CANDIDATES = 10
        if len(cells) > MAX_CANDIDATES:
            cells = cells[:MAX_CANDIDATES]
        
        return cells


class Grid:
    """Arena grid with obstacles."""
    
    def __init__(self, size_x: int, size_y: int):
        self.size_x = size_x
        self.size_y = size_y
        self.obstacles: List[Obstacle] = []

    def add_obstacle(self, obstacle: Obstacle):
        for ob in self.obstacles:
            if ob == obstacle:
                return
        self.obstacles.append(obstacle)

    def reset_obstacles(self):
        self.obstacles = []

    def get_obstacles(self):
        return self.obstacles

    def is_valid_coord(self, x, y) -> bool:
        return (1 <= x <= self.size_x - 2 and 1 <= y <= self.size_y - 2)

    def get_view_obstacle_positions(self, retrying) -> List[List[CellState]]:
        """Get valid capture candidates for each obstacle.
        
        Filters out candidates that are:
        - Inside another obstacle's virtual safety zone (40x40cm = 2 grid units)
        - Outside arena bounds
        """
        all_positions = []
        
        for obstacle in self.obstacles:
            if obstacle.direction == Direction.SKIP:
                continue
            
            candidates = obstacle.get_view_state(retrying)
            
            # Filter against OTHER obstacles' safety zones
            valid = []
            for cand in candidates:
                cand_x_cm = cand.x * 10  # grid to cm (left edge)
                cand_y_cm = cand.y * 10
                
                blocked = False
                for other in self.obstacles:
                    if other.obstacle_id == obstacle.obstacle_id:
                        continue
                    
                    # Other obstacle center in cm
                    other_cx = other.x * 10 + 5
                    other_cy = other.y * 10 + 5
                    
                    # Robot center in cm (cand.x/y are already center in grid)
                    robot_cx = cand.x * 10
                    robot_cy = cand.y * 10
                    
                    dist = math.sqrt((robot_cx - other_cx)**2 + 
                                     (robot_cy - other_cy)**2)
                    
                    if dist < VIRTUAL_OBS_HALF_CM + 5:  # 26cm minimum clearance from other obstacles
                        blocked = True
                        break
                
                if not blocked:
                    valid.append(cand)
            
            all_positions.append(valid)
        
        return all_positions