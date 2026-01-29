"""
MDP Path Planning Module - Simplified Version
Implements Hamiltonian path planning with Dubins path calculations
Based on the MDP Algorithm Briefing (SC2079)

Key simplifications:
1. Use discrete sampling for path following instead of complex Dubins math
2. Focus on getting collision-free paths that work
3. Simplified path tracing
"""

import math
from typing import List, Tuple, Dict, Optional
from dataclasses import dataclass, field


# =============================================================================
# CONFIGURATION
# =============================================================================

ARENA_SIZE = 200  # cm
OBSTACLE_SIZE = 10  # cm
ROBOT_SIZE = 30  # cm
TURNING_RADIUS = 25  # cm
CAMERA_DISTANCE = 20  # cm from obstacle for image recognition

# Virtual obstacle size for collision detection
# Robot (30cm) must not overlap with obstacle (10cm)
# So robot center must be at least 15cm (robot radius) + 5cm (obstacle radius) = 20cm from obstacle center
COLLISION_DISTANCE = 20  # cm from obstacle center to robot center

# Path sampling
PATH_SAMPLE_DISTANCE = 3  # cm between collision check samples


# =============================================================================
# DATA STRUCTURES
# =============================================================================

@dataclass
class RobotConfig:
    """Robot configuration: position (x, y) and orientation theta"""
    x: float  # cm (center of robot)
    y: float  # cm (center of robot)
    theta: float  # radians (-π to π, 0 = East facing)

    def __repr__(self):
        return f"Config(x={self.x:.1f}, y={self.y:.1f}, θ={math.degrees(self.theta):.1f}°)"


@dataclass
class Obstacle:
    """Obstacle with position and image direction"""
    id: int
    x: int  # grid cell column (0-19)
    y: int  # grid cell row (0-19)
    direction: int  # 0=North, 90=East, 180=South, 270=West

    def get_center_cm(self) -> Tuple[float, float]:
        """Get center position in cm"""
        return (self.x * 10 + 5, self.y * 10 + 5)

    def get_target_config(self) -> RobotConfig:
        """
        Calculate target robot configuration to recognize image.
        Robot faces TOWARD the obstacle from the side with the image.
        """
        cx, cy = self.get_center_cm()
        
        # Distance from obstacle center to robot center
        # = half_obstacle + camera_dist + half_robot = 5 + 20 + 15 = 40cm
        offset = 5 + CAMERA_DISTANCE + ROBOT_SIZE / 2

        if self.direction == 0:  # NORTH - image on top
            # Robot above obstacle, facing South (down)
            return RobotConfig(cx, cy + offset, -math.pi/2)
        elif self.direction == 90:  # EAST - image on right
            # Robot to the right, facing West (left)
            return RobotConfig(cx + offset, cy, math.pi)
        elif self.direction == 180:  # SOUTH - image on bottom
            # Robot below obstacle, facing North (up)
            return RobotConfig(cx, cy - offset, math.pi/2)
        else:  # 270 WEST - image on left
            # Robot to the left, facing East (right)
            return RobotConfig(cx - offset, cy, 0)


@dataclass
class DubinsPath:
    """A Dubins path with waypoints for navigation"""
    path_type: str
    length: float
    waypoints: List[Tuple[float, float, float]] = field(default_factory=list)
    has_collision: bool = False

    def __repr__(self):
        coll = " [COLL]" if self.has_collision else ""
        return f"Path({self.path_type}, {self.length:.1f}cm{coll})"


# =============================================================================
# COLLISION DETECTION
# =============================================================================

def normalize_angle(angle: float) -> float:
    """Normalize angle to [-π, π]"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def point_in_bounds(x: float, y: float) -> bool:
    """Check if point keeps robot within arena"""
    margin = ROBOT_SIZE / 2
    return margin <= x <= ARENA_SIZE - margin and margin <= y <= ARENA_SIZE - margin


def point_hits_obstacle(x: float, y: float, obstacles: List[Obstacle], 
                       exclude_id: Optional[int] = None) -> bool:
    """Check if robot center at (x,y) collides with any obstacle"""
    for obs in obstacles:
        if exclude_id is not None and obs.id == exclude_id:
            continue
        
        cx, cy = obs.get_center_cm()
        # Use circular collision check (simpler and more forgiving)
        dist = math.sqrt((x - cx)**2 + (y - cy)**2)
        if dist < COLLISION_DISTANCE:
            return True
    
    return False


def check_path_clear(waypoints: List[Tuple[float, float, float]], 
                    obstacles: List[Obstacle],
                    target_id: Optional[int] = None) -> bool:
    """Check if all waypoints are collision-free"""
    for x, y, theta in waypoints:
        if not point_in_bounds(x, y):
            return False
        if point_hits_obstacle(x, y, obstacles, target_id):
            return False
    return True


# =============================================================================
# SIMPLE DUBINS PATH GENERATION
# =============================================================================

def generate_arc_waypoints(cx: float, cy: float, start_angle: float, 
                          arc_angle: float, radius: float, 
                          turn_left: bool) -> List[Tuple[float, float, float]]:
    """
    Generate waypoints along a circular arc.
    
    cx, cy: center of circle
    start_angle: angle from center to start point
    arc_angle: total angle to travel (positive for CCW, negative for CW)
    radius: turning radius
    turn_left: True for counterclockwise (left turn), False for clockwise (right turn)
    """
    waypoints = []
    arc_length = abs(arc_angle) * radius
    num_steps = max(2, int(arc_length / PATH_SAMPLE_DISTANCE))
    
    for i in range(num_steps + 1):
        t = i / num_steps
        
        # Calculate angle from center at this point
        angle = start_angle + t * arc_angle
        
        # Position on circle
        x = cx + radius * math.cos(angle)
        y = cy + radius * math.sin(angle)
        
        # Heading is tangent to circle
        # For left turn (CCW): heading is 90° behind the radius angle
        # For right turn (CW): heading is 90° ahead of the radius angle
        if turn_left:
            heading = angle - math.pi/2
        else:
            heading = angle + math.pi/2
        
        waypoints.append((x, y, normalize_angle(heading)))
    
    return waypoints


def generate_straight_waypoints(x1: float, y1: float, x2: float, y2: float, 
                               theta: float) -> List[Tuple[float, float, float]]:
    """Generate waypoints along a straight line"""
    waypoints = []
    dist = math.sqrt((x2-x1)**2 + (y2-y1)**2)
    num_steps = max(2, int(dist / PATH_SAMPLE_DISTANCE))
    
    for i in range(num_steps + 1):
        t = i / num_steps
        x = x1 + t * (x2 - x1)
        y = y1 + t * (y2 - y1)
        waypoints.append((x, y, theta))
    
    return waypoints


def compute_rsr_path(start: RobotConfig, end: RobotConfig) -> Optional[DubinsPath]:
    """Compute Right-Straight-Right Dubins path"""
    r = TURNING_RADIUS
    
    # Centers of right-turning circles
    c1_x = start.x + r * math.cos(start.theta - math.pi/2)
    c1_y = start.y + r * math.sin(start.theta - math.pi/2)
    c2_x = end.x + r * math.cos(end.theta - math.pi/2)
    c2_y = end.y + r * math.sin(end.theta - math.pi/2)
    
    # Vector between centers
    dx = c2_x - c1_x
    dy = c2_y - c1_y
    d = math.sqrt(dx*dx + dy*dy)
    
    if d < 0.1:
        return None
    
    # Angle of tangent line (outer tangent for same-side circles)
    theta_line = math.atan2(dy, dx)
    
    # Tangent points
    pt1_x = c1_x + r * math.cos(theta_line + math.pi/2)
    pt1_y = c1_y + r * math.sin(theta_line + math.pi/2)
    pt2_x = c2_x + r * math.cos(theta_line + math.pi/2)
    pt2_y = c2_y + r * math.sin(theta_line + math.pi/2)
    
    # Arc angles
    start_angle_1 = math.atan2(start.y - c1_y, start.x - c1_x)
    end_angle_1 = math.atan2(pt1_y - c1_y, pt1_x - c1_x)
    arc1 = end_angle_1 - start_angle_1
    while arc1 > 0: arc1 -= 2*math.pi
    while arc1 < -2*math.pi: arc1 += 2*math.pi
    
    start_angle_2 = math.atan2(pt2_y - c2_y, pt2_x - c2_x)
    end_angle_2 = math.atan2(end.y - c2_y, end.x - c2_x)
    arc2 = end_angle_2 - start_angle_2
    while arc2 > 0: arc2 -= 2*math.pi
    while arc2 < -2*math.pi: arc2 += 2*math.pi
    
    # Generate waypoints
    waypoints = []
    waypoints.extend(generate_arc_waypoints(c1_x, c1_y, start_angle_1, arc1, r, False))
    waypoints.extend(generate_straight_waypoints(pt1_x, pt1_y, pt2_x, pt2_y, theta_line))
    waypoints.extend(generate_arc_waypoints(c2_x, c2_y, start_angle_2, arc2, r, False))
    
    length = abs(arc1)*r + d + abs(arc2)*r
    
    return DubinsPath("RSR", length, waypoints)


def compute_lsl_path(start: RobotConfig, end: RobotConfig) -> Optional[DubinsPath]:
    """Compute Left-Straight-Left Dubins path"""
    r = TURNING_RADIUS
    
    # Centers of left-turning circles
    c1_x = start.x + r * math.cos(start.theta + math.pi/2)
    c1_y = start.y + r * math.sin(start.theta + math.pi/2)
    c2_x = end.x + r * math.cos(end.theta + math.pi/2)
    c2_y = end.y + r * math.sin(end.theta + math.pi/2)
    
    dx = c2_x - c1_x
    dy = c2_y - c1_y
    d = math.sqrt(dx*dx + dy*dy)
    
    if d < 0.1:
        return None
    
    theta_line = math.atan2(dy, dx)
    
    # Tangent points (outer tangent)
    pt1_x = c1_x + r * math.cos(theta_line - math.pi/2)
    pt1_y = c1_y + r * math.sin(theta_line - math.pi/2)
    pt2_x = c2_x + r * math.cos(theta_line - math.pi/2)
    pt2_y = c2_y + r * math.sin(theta_line - math.pi/2)
    
    # Arc angles (counterclockwise = positive)
    start_angle_1 = math.atan2(start.y - c1_y, start.x - c1_x)
    end_angle_1 = math.atan2(pt1_y - c1_y, pt1_x - c1_x)
    arc1 = end_angle_1 - start_angle_1
    while arc1 < 0: arc1 += 2*math.pi
    while arc1 > 2*math.pi: arc1 -= 2*math.pi
    
    start_angle_2 = math.atan2(pt2_y - c2_y, pt2_x - c2_x)
    end_angle_2 = math.atan2(end.y - c2_y, end.x - c2_x)
    arc2 = end_angle_2 - start_angle_2
    while arc2 < 0: arc2 += 2*math.pi
    while arc2 > 2*math.pi: arc2 -= 2*math.pi
    
    waypoints = []
    waypoints.extend(generate_arc_waypoints(c1_x, c1_y, start_angle_1, arc1, r, True))
    waypoints.extend(generate_straight_waypoints(pt1_x, pt1_y, pt2_x, pt2_y, theta_line))
    waypoints.extend(generate_arc_waypoints(c2_x, c2_y, start_angle_2, arc2, r, True))
    
    length = arc1*r + d + arc2*r
    
    return DubinsPath("LSL", length, waypoints)


def compute_rsl_path(start: RobotConfig, end: RobotConfig) -> Optional[DubinsPath]:
    """Compute Right-Straight-Left Dubins path"""
    r = TURNING_RADIUS
    
    # Right circle for start, left circle for end
    c1_x = start.x + r * math.cos(start.theta - math.pi/2)
    c1_y = start.y + r * math.sin(start.theta - math.pi/2)
    c2_x = end.x + r * math.cos(end.theta + math.pi/2)
    c2_y = end.y + r * math.sin(end.theta + math.pi/2)
    
    dx = c2_x - c1_x
    dy = c2_y - c1_y
    d = math.sqrt(dx*dx + dy*dy)
    
    if d < 2*r:
        return None  # Circles overlap, no tangent exists
    
    # Angle between centers
    theta = math.atan2(dy, dx)
    
    # Cross tangent calculation
    alpha = math.acos(2*r / d)
    
    # Tangent angle at first circle (exit direction)
    tangent_angle_1 = theta - alpha
    # Tangent angle at second circle (entry direction)
    tangent_angle_2 = theta - alpha + math.pi
    
    # Tangent points
    pt1_x = c1_x + r * math.cos(tangent_angle_1 + math.pi/2)
    pt1_y = c1_y + r * math.sin(tangent_angle_1 + math.pi/2)
    pt2_x = c2_x + r * math.cos(tangent_angle_2 - math.pi/2)
    pt2_y = c2_y + r * math.sin(tangent_angle_2 - math.pi/2)
    
    # Arc angles
    start_angle_1 = math.atan2(start.y - c1_y, start.x - c1_x)
    end_angle_1 = math.atan2(pt1_y - c1_y, pt1_x - c1_x)
    arc1 = end_angle_1 - start_angle_1
    while arc1 > 0: arc1 -= 2*math.pi
    
    start_angle_2 = math.atan2(pt2_y - c2_y, pt2_x - c2_x)
    end_angle_2 = math.atan2(end.y - c2_y, end.x - c2_x)
    arc2 = end_angle_2 - start_angle_2
    while arc2 < 0: arc2 += 2*math.pi
    
    # Straight segment length
    straight_len = math.sqrt(d*d - 4*r*r)
    
    waypoints = []
    waypoints.extend(generate_arc_waypoints(c1_x, c1_y, start_angle_1, arc1, r, False))
    waypoints.extend(generate_straight_waypoints(pt1_x, pt1_y, pt2_x, pt2_y, tangent_angle_1))
    waypoints.extend(generate_arc_waypoints(c2_x, c2_y, start_angle_2, arc2, r, True))
    
    length = abs(arc1)*r + straight_len + arc2*r
    
    return DubinsPath("RSL", length, waypoints)


def compute_lsr_path(start: RobotConfig, end: RobotConfig) -> Optional[DubinsPath]:
    """Compute Left-Straight-Right Dubins path"""
    r = TURNING_RADIUS
    
    # Left circle for start, right circle for end
    c1_x = start.x + r * math.cos(start.theta + math.pi/2)
    c1_y = start.y + r * math.sin(start.theta + math.pi/2)
    c2_x = end.x + r * math.cos(end.theta - math.pi/2)
    c2_y = end.y + r * math.sin(end.theta - math.pi/2)
    
    dx = c2_x - c1_x
    dy = c2_y - c1_y
    d = math.sqrt(dx*dx + dy*dy)
    
    if d < 2*r:
        return None
    
    theta = math.atan2(dy, dx)
    alpha = math.acos(2*r / d)
    
    tangent_angle_1 = theta + alpha
    tangent_angle_2 = theta + alpha - math.pi
    
    pt1_x = c1_x + r * math.cos(tangent_angle_1 - math.pi/2)
    pt1_y = c1_y + r * math.sin(tangent_angle_1 - math.pi/2)
    pt2_x = c2_x + r * math.cos(tangent_angle_2 + math.pi/2)
    pt2_y = c2_y + r * math.sin(tangent_angle_2 + math.pi/2)
    
    start_angle_1 = math.atan2(start.y - c1_y, start.x - c1_x)
    end_angle_1 = math.atan2(pt1_y - c1_y, pt1_x - c1_x)
    arc1 = end_angle_1 - start_angle_1
    while arc1 < 0: arc1 += 2*math.pi
    
    start_angle_2 = math.atan2(pt2_y - c2_y, pt2_x - c2_x)
    end_angle_2 = math.atan2(end.y - c2_y, end.x - c2_x)
    arc2 = end_angle_2 - start_angle_2
    while arc2 > 0: arc2 -= 2*math.pi
    
    straight_len = math.sqrt(d*d - 4*r*r)
    
    waypoints = []
    waypoints.extend(generate_arc_waypoints(c1_x, c1_y, start_angle_1, arc1, r, True))
    waypoints.extend(generate_straight_waypoints(pt1_x, pt1_y, pt2_x, pt2_y, tangent_angle_1))
    waypoints.extend(generate_arc_waypoints(c2_x, c2_y, start_angle_2, arc2, r, False))
    
    length = arc1*r + straight_len + abs(arc2)*r
    
    return DubinsPath("LSR", length, waypoints)


def find_best_path(start: RobotConfig, end: RobotConfig, 
                  obstacles: List[Obstacle], 
                  target_id: Optional[int] = None) -> Optional[DubinsPath]:
    """
    Find the shortest collision-free Dubins path.
    """
    path_funcs = [compute_rsr_path, compute_lsl_path, compute_rsl_path, compute_lsr_path]
    
    valid_paths = []
    
    for func in path_funcs:
        path = func(start, end)
        if path is None:
            continue
        
        # Check for collisions
        if check_path_clear(path.waypoints, obstacles, target_id):
            valid_paths.append(path)
        else:
            path.has_collision = True
    
    if not valid_paths:
        return None
    
    # Return shortest valid path
    return min(valid_paths, key=lambda p: p.length)


# =============================================================================
# PATH PLANNING ALGORITHMS
# =============================================================================

def exhaustive_search(start: RobotConfig, obstacles: List[Obstacle]) -> Tuple[Optional[List[int]], float, List]:
    """
    Find optimal Hamiltonian path by trying all permutations.
    """
    from itertools import permutations
    
    best_order = None
    best_length = float('inf')
    best_segments = []
    
    for perm in permutations(obstacles):
        current = start
        total_length = 0
        valid = True
        segments = []
        
        for obs in perm:
            target = obs.get_target_config()
            path = find_best_path(current, target, obstacles, obs.id)
            
            if path is None:
                valid = False
                break
            
            total_length += path.length
            segments.append({
                'obstacle_id': obs.id,
                'path': path,
                'waypoints': path.waypoints
            })
            
            if total_length >= best_length:
                valid = False
                break
            
            current = target
        
        if valid and total_length < best_length:
            best_length = total_length
            best_order = [obs.id for obs in perm]
            best_segments = segments
    
    return best_order, best_length, best_segments


def nearest_neighbor(start: RobotConfig, obstacles: List[Obstacle]) -> Tuple[Optional[List[int]], float, List]:
    """
    Greedy nearest-neighbor path planning.
    """
    current = start
    visited = set()
    order = []
    total_length = 0
    segments = []
    
    for _ in range(len(obstacles)):
        best_obs = None
        best_path = None
        best_dist = float('inf')
        
        for obs in obstacles:
            if obs.id in visited:
                continue
            
            target = obs.get_target_config()
            path = find_best_path(current, target, obstacles, obs.id)
            
            if path and path.length < best_dist:
                best_dist = path.length
                best_obs = obs
                best_path = path
        
        if best_obs is None:
            break
        
        visited.add(best_obs.id)
        order.append(best_obs.id)
        total_length += best_dist
        segments.append({
            'obstacle_id': best_obs.id,
            'path': best_path,
            'waypoints': best_path.waypoints
        })
        current = best_obs.get_target_config()
    
    if len(order) != len(obstacles):
        return None, 0, []
    
    return order, total_length, segments


def plan_path(start: RobotConfig, obstacles: List[Obstacle], 
              use_exhaustive: bool = True) -> Dict:
    """
    Main path planning function.
    """
    print(f"\n{'='*50}")
    print("PATH PLANNING")
    print(f"{'='*50}")
    print(f"Start: {start}")
    print(f"Obstacles: {[o.id for o in obstacles]}")
    
    # Validate targets
    print("\nTarget positions:")
    for obs in obstacles:
        target = obs.get_target_config()
        in_bounds = point_in_bounds(target.x, target.y)
        status = "✓" if in_bounds else "✗ OUT OF BOUNDS"
        print(f"  Obs {obs.id}: ({target.x:.0f}, {target.y:.0f}) {status}")
    
    # Find path
    if use_exhaustive and len(obstacles) <= 6:
        print(f"\nUsing exhaustive search ({math.factorial(len(obstacles))} permutations)...")
        order, length, segments = exhaustive_search(start, obstacles)
    else:
        print("\nUsing nearest neighbor...")
        order, length, segments = nearest_neighbor(start, obstacles)
    
    if order is None:
        print("✗ No valid path found!")
        return {'order': [], 'total_length': 0, 'segments': [], 'error': 'No path found'}
    
    print(f"\n✓ Path found: {order}")
    print(f"  Total length: {length:.1f} cm")
    
    return {
        'order': order,
        'total_length': length,
        'segments': segments
    }


def load_obstacles_from_json(data: Dict) -> List[Obstacle]:
    """Load obstacles from JSON data."""
    return [Obstacle(o['id'], o['x'], o['y'], o['direction']) 
            for o in data.get('obstacles', [])]


# =============================================================================
# TEST
# =============================================================================

if __name__ == "__main__":
    # Test with well-spaced obstacles
    # Key points for obstacle placement:
    # 1. Image direction determines where robot must position itself
    # 2. Robot needs 40cm clearance from obstacle center to target position
    # 3. Obstacles near arena edges should have images facing INWARD
    # 4. Obstacles should be far enough apart that Dubins paths don't cross them
    
    obstacles = [
        Obstacle(1, 4, 4, 90),     # bottom-left, East face
        Obstacle(2, 14, 4, 270),   # bottom-right, West face
        Obstacle(3, 4, 14, 90),    # top-left, East face
        Obstacle(4, 14, 14, 270),  # top-right, West face
        Obstacle(5, 9, 9, 0),      # center, North face
    ]
    
    # Robot starts in START zone facing East (0°)
    # Starting at (20, 20) gives room to turn in any direction
    start = RobotConfig(20, 20, 0)
    
    result = plan_path(start, obstacles, use_exhaustive=True)
    
    if 'error' not in result:
        print(f"\nPath details:")
        for seg in result['segments']:
            print(f"  -> Obs {seg['obstacle_id']}: {seg['path']}")
