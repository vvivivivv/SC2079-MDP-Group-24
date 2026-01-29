
"""
MDP Path Planning Module with Collision Avoidance
Implements Hamiltonian path planning with Dubins path calculations
Based on the MDP Algorithm Briefing (SC2079)
"""

import math
from typing import List, Tuple, Dict, Optional
from dataclasses import dataclass
from enum import Enum


# =============================================================================
# CONFIGURATION
# =============================================================================

# Arena and Robot Settings
ARENA_SIZE = 200  # cm
OBSTACLE_SIZE = 10  # cm
ROBOT_SIZE = 30  # cm
TURNING_RADIUS = 25  # cm (adjust based on your actual robot)
CAMERA_DISTANCE = 20  # cm from obstacle for image recognition

# Collision Settings
VIRTUAL_OBSTACLE_SIZE = 40  # cm (10cm obstacle + 15cm each side)
PATH_SAMPLE_DISTANCE = 5  # cm between collision check samples


# =============================================================================
# DATA STRUCTURES
# =============================================================================

@dataclass
class RobotConfig:
    """Robot configuration: position (x, y) and orientation theta"""
    x: float  # cm (center of robot)
    y: float  # cm (center of robot)
    theta: float  # radians (-π to π, 0 = East)

    def __repr__(self):
        return f"RobotConfig(x={self.x:.2f}, y={self.y:.2f}, θ={math.degrees(self.theta):.1f}°)"


@dataclass
class Obstacle:
    """Obstacle with position and image direction"""
    id: int
    x: int  # bottom-left corner (grid cell)
    y: int  # bottom-left corner (grid cell)
    direction: int  # 0=North, 90=East, 180=South, 270=West

    def get_center(self) -> Tuple[float, float]:
        """Get center position of obstacle in cm"""
        center_x = self.x * 10 + 5
        center_y = self.y * 10 + 5
        return center_x, center_y

    def get_virtual_bounds(self) -> Tuple[float, float, float, float]:
        """Get virtual obstacle boundaries (left, right, bottom, top)"""
        center_x, center_y = self.get_center()
        half_size = VIRTUAL_OBSTACLE_SIZE / 2
        return (
            center_x - half_size,  # left
            center_x + half_size,  # right
            center_y - half_size,  # bottom
            center_y + half_size   # top
        )

    def get_target_config(self) -> RobotConfig:
        """
        Calculate target robot configuration to recognize image.
        Robot center should be positioned so camera is 20cm from obstacle face.
        """
        obs_x, obs_y = self.get_center()

        # Robot is 30cm, so its center is 15cm from its edge
        # Camera is at robot's front center
        # Need: camera 20cm from obstacle face
        # So: robot_center = obstacle_face + 20cm + 15cm = obstacle_face + 35cm

        if self.direction == 0:  # North - image on top
            target_x = obs_x
            target_y = obs_y + 5 + 35
            target_theta = math.radians(90)

        elif self.direction == 90:  # East - image on right
            target_x = obs_x + 5 + 35
            target_y = obs_y
            target_theta = math.radians(0)

        elif self.direction == 180:  # South - image on bottom
            target_x = obs_x
            target_y = obs_y - 5 - 35
            target_theta = math.radians(-90)

        else:  # 270, West - image on left
            target_x = obs_x - 5 - 35
            target_y = obs_y
            target_theta = math.radians(180)

        return RobotConfig(target_x, target_y, target_theta)


@dataclass
class DubinsPath:
    """A Dubins path consisting of up to 3 segments"""
    path_type: str  # e.g., "LSL", "RSR", "LSR", etc.
    length: float  # total path length in cm
    segments: List[Tuple[str, float]]  # List of (segment_type, length/angle)
    has_collision: bool = False  # NEW: Track if path has collision

    def __repr__(self):
        collision_str = " [COLLISION]" if self.has_collision else ""
        return f"DubinsPath({self.path_type}, length={self.length:.2f}cm{collision_str})"


# =============================================================================
# COLLISION DETECTION
# =============================================================================

def point_collides_with_obstacle(x: float, y: float, obstacle: Obstacle) -> bool:
    """
    Check if a point (x, y) is too close to an obstacle's virtual boundary.
    Robot needs clearance equal to its radius (15cm).
    """
    left, right, bottom, top = obstacle.get_virtual_bounds()
    robot_radius = ROBOT_SIZE / 2  # 15cm

    # Find closest point on virtual obstacle box to (x, y)
    closest_x = max(left, min(x, right))
    closest_y = max(bottom, min(y, top))

    # Calculate distance from (x, y) to closest point
    dist_x = x - closest_x
    dist_y = y - closest_y
    distance = math.sqrt(dist_x*dist_x + dist_y*dist_y)

    # Collision if distance is less than robot radius
    return distance < robot_radius


def check_path_collision(start: RobotConfig, end: RobotConfig, 
                         path_length: float, obstacles: List[Obstacle],
                         exclude_obstacle: Optional[Obstacle] = None) -> bool:
    """
    Sample points along a path and check for collisions.

    Args:
        start: Starting robot configuration
        end: Ending robot configuration
        path_length: Length of the path
        obstacles: List of all obstacles
        exclude_obstacle: Obstacle to exclude from collision check (typically the target)

    Returns:
        True if collision detected, False if path is clear
    """
    # Sample points along the path
    num_samples = max(10, int(path_length / PATH_SAMPLE_DISTANCE))

    for i in range(num_samples + 1):
        t = i / num_samples

        # Linear interpolation (simplified - actual Dubins path would curve)
        # For more accuracy, you'd trace the actual curved path
        x = start.x + t * (end.x - start.x)
        y = start.y + t * (end.y - start.y)

        # Check collision with all obstacles except target
        for obs in obstacles:
            if exclude_obstacle and obs.id == exclude_obstacle.id:
                continue  # Don't check collision with target obstacle

            if point_collides_with_obstacle(x, y, obs):
                return True  # Collision found!

    return False  # Path is clear


def check_arena_bounds(config: RobotConfig) -> bool:
    """Check if robot configuration is within arena bounds"""
    robot_radius = ROBOT_SIZE / 2

    if config.x - robot_radius < 0 or config.x + robot_radius > ARENA_SIZE:
        return False
    if config.y - robot_radius < 0 or config.y + robot_radius > ARENA_SIZE:
        return False

    return True


# =============================================================================
# DUBINS PATH CALCULATIONS
# =============================================================================

def normalize_angle(angle: float) -> float:
    """Normalize angle to [-π, π]"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def compute_turning_center(config: RobotConfig, turn_left: bool) -> Tuple[float, float]:
    """Compute the center of the turning circle."""
    if turn_left:
        angle = config.theta + math.pi / 2
    else:
        angle = config.theta - math.pi / 2

    cx = config.x + TURNING_RADIUS * math.cos(angle)
    cy = config.y + TURNING_RADIUS * math.sin(angle)

    return cx, cy


def compute_arc_angle(start_angle: float, end_angle: float, turn_left: bool) -> float:
    """Compute the arc angle considering turn direction."""
    angle = normalize_angle(end_angle - start_angle)

    if turn_left and angle < 0:
        angle += 2 * math.pi
    elif not turn_left and angle > 0:
        angle -= 2 * math.pi

    return abs(angle)


def dubins_lsl(start: RobotConfig, end: RobotConfig) -> Optional[DubinsPath]:
    """Left-Straight-Left path"""
    c1 = compute_turning_center(start, turn_left=True)
    c2 = compute_turning_center(end, turn_left=True)

    dx = c2[0] - c1[0]
    dy = c2[1] - c1[1]
    d = math.sqrt(dx*dx + dy*dy)

    if d < 0.01:
        return None

    tangent_angle = math.atan2(dy, dx)
    arc1_angle = compute_arc_angle(start.theta, tangent_angle, turn_left=True)
    arc2_angle = compute_arc_angle(tangent_angle, end.theta, turn_left=True)

    straight_length = d
    total_length = (arc1_angle + arc2_angle) * TURNING_RADIUS + straight_length

    return DubinsPath(
        path_type="LSL",
        length=total_length,
        segments=[
            ("L", arc1_angle),
            ("S", straight_length),
            ("L", arc2_angle)
        ]
    )


def dubins_rsr(start: RobotConfig, end: RobotConfig) -> Optional[DubinsPath]:
    """Right-Straight-Right path"""
    c1 = compute_turning_center(start, turn_left=False)
    c2 = compute_turning_center(end, turn_left=False)

    dx = c2[0] - c1[0]
    dy = c2[1] - c1[1]
    d = math.sqrt(dx*dx + dy*dy)

    if d < 0.01:
        return None

    tangent_angle = math.atan2(dy, dx)
    arc1_angle = compute_arc_angle(start.theta, tangent_angle, turn_left=False)
    arc2_angle = compute_arc_angle(tangent_angle, end.theta, turn_left=False)

    straight_length = d
    total_length = (arc1_angle + arc2_angle) * TURNING_RADIUS + straight_length

    return DubinsPath(
        path_type="RSR",
        length=total_length,
        segments=[
            ("R", arc1_angle),
            ("S", straight_length),
            ("R", arc2_angle)
        ]
    )


def dubins_lsr(start: RobotConfig, end: RobotConfig) -> Optional[DubinsPath]:
    """Left-Straight-Right path"""
    c1 = compute_turning_center(start, turn_left=True)
    c2 = compute_turning_center(end, turn_left=False)

    dx = c2[0] - c1[0]
    dy = c2[1] - c1[1]
    d = math.sqrt(dx*dx + dy*dy)

    if d < 2 * TURNING_RADIUS:
        return None

    angle_between = math.atan2(dy, dx)
    alpha = math.asin(2 * TURNING_RADIUS / d)

    tangent_angle_start = angle_between + alpha
    tangent_angle_end = angle_between + alpha + math.pi

    arc1_angle = compute_arc_angle(start.theta, tangent_angle_start, turn_left=True)
    arc2_angle = compute_arc_angle(tangent_angle_end, end.theta, turn_left=False)

    straight_length = math.sqrt(d*d - 4*TURNING_RADIUS*TURNING_RADIUS)
    total_length = (arc1_angle + arc2_angle) * TURNING_RADIUS + straight_length

    return DubinsPath(
        path_type="LSR",
        length=total_length,
        segments=[
            ("L", arc1_angle),
            ("S", straight_length),
            ("R", arc2_angle)
        ]
    )


def dubins_rsl(start: RobotConfig, end: RobotConfig) -> Optional[DubinsPath]:
    """Right-Straight-Left path"""
    c1 = compute_turning_center(start, turn_left=False)
    c2 = compute_turning_center(end, turn_left=True)

    dx = c2[0] - c1[0]
    dy = c2[1] - c1[1]
    d = math.sqrt(dx*dx + dy*dy)

    if d < 2 * TURNING_RADIUS:
        return None

    angle_between = math.atan2(dy, dx)
    alpha = math.asin(2 * TURNING_RADIUS / d)

    tangent_angle_start = angle_between - alpha
    tangent_angle_end = angle_between - alpha + math.pi

    arc1_angle = compute_arc_angle(start.theta, tangent_angle_start, turn_left=False)
    arc2_angle = compute_arc_angle(tangent_angle_end, end.theta, turn_left=True)

    straight_length = math.sqrt(d*d - 4*TURNING_RADIUS*TURNING_RADIUS)
    total_length = (arc1_angle + arc2_angle) * TURNING_RADIUS + straight_length

    return DubinsPath(
        path_type="RSL",
        length=total_length,
        segments=[
            ("R", arc1_angle),
            ("S", straight_length),
            ("L", arc2_angle)
        ]
    )


def compute_shortest_dubins_path(start: RobotConfig, end: RobotConfig, 
                                 obstacles: List[Obstacle],
                                 target_obstacle: Optional[Obstacle] = None) -> Optional[DubinsPath]:
    """
    Compute the shortest COLLISION-FREE Dubins path between two configurations.
    Tries all 4 CSC path types and returns the shortest one that doesn't collide.

    Args:
        start: Starting configuration
        end: Target configuration
        obstacles: List of all obstacles
        target_obstacle: The obstacle we're traveling to (excluded from collision check)
    """
    paths = []

    # Try all CSC paths
    for path_func in [dubins_lsl, dubins_rsr, dubins_lsr, dubins_rsl]:
        path = path_func(start, end)
        if path is not None:
            # Check for collisions
            has_collision = check_path_collision(start, end, path.length, obstacles, target_obstacle)
            path.has_collision = has_collision

            # Only consider collision-free paths
            if not has_collision:
                paths.append(path)

    if not paths:
        print(f"⚠ Warning: No collision-free path found from {start} to {end}")
        return None

    # Return shortest collision-free path
    return min(paths, key=lambda p: p.length)


# =============================================================================
# PATH PLANNING ALGORITHMS
# =============================================================================

def nearest_neighbor_path(start: RobotConfig, obstacles: List[Obstacle]) -> List[int]:
    """Greedy nearest-neighbor algorithm with collision avoidance"""
    current_config = start
    visited = set()
    path = []

    for _ in range(len(obstacles)):
        best_obstacle = None
        best_distance = float('inf')

        for obs in obstacles:
            if obs.id in visited:
                continue

            target_config = obs.get_target_config()
            dubins = compute_shortest_dubins_path(current_config, target_config, obstacles, obs)

            if dubins and dubins.length < best_distance:
                best_distance = dubins.length
                best_obstacle = obs

        if best_obstacle is None:
            print(f"⚠ Cannot find collision-free path to remaining obstacles!")
            break

        visited.add(best_obstacle.id)
        path.append(best_obstacle.id)
        current_config = best_obstacle.get_target_config()

    return path


def exhaustive_search_path(start: RobotConfig, obstacles: List[Obstacle]) -> Tuple[List[int], float]:
    """Exhaustive search with collision avoidance"""
    from itertools import permutations

    best_path = None
    best_length = float('inf')

    for perm in permutations(obstacles):
        current_config = start
        total_length = 0
        valid = True

        for obs in perm:
            target_config = obs.get_target_config()
            dubins = compute_shortest_dubins_path(current_config, target_config, obstacles, obs)

            if dubins is None:
                valid = False
                break

            total_length += dubins.length
            current_config = target_config

        if valid and total_length < best_length:
            best_length = total_length
            best_path = [obs.id for obs in perm]

    if best_path is None:
        print("⚠ No valid collision-free path found for any permutation!")

    return best_path, best_length


def plan_path(start_config: RobotConfig, obstacles: List[Obstacle], 
              use_exhaustive: bool = True) -> Dict:
    """
    Main path planning function with collision avoidance.
    """
    if use_exhaustive:
        order, total_length = exhaustive_search_path(start_config, obstacles)
    else:
        order = nearest_neighbor_path(start_config, obstacles)
        total_length = 0

    if order is None:
        return {
            'order': [],
            'total_length': 0,
            'segments': [],
            'error': 'No collision-free path found'
        }

    # Compute detailed segments
    segments = []
    current_config = start_config

    obs_dict = {obs.id: obs for obs in obstacles}

    for obs_id in order:
        obs = obs_dict[obs_id]
        target_config = obs.get_target_config()
        dubins = compute_shortest_dubins_path(current_config, target_config, obstacles, obs)

        if dubins:
            segments.append({
                'from': current_config,
                'to': target_config,
                'obstacle_id': obs_id,
                'path': dubins
            })

            if not use_exhaustive:
                total_length += dubins.length

            current_config = target_config

    return {
        'order': order,
        'total_length': total_length,
        'segments': segments
    }


# =============================================================================
# UTILITY FUNCTIONS
# =============================================================================

def load_obstacles_from_json(data: Dict) -> List[Obstacle]:
    """Load obstacles from JSON data structure."""
    obstacles = []
    for obs_data in data.get('obstacles', []):
        obstacles.append(Obstacle(
            id=obs_data['id'],
            x=obs_data['x'],
            y=obs_data['y'],
            direction=obs_data['direction']
        ))
    return obstacles


if __name__ == "__main__":
    import json

    obstacles_json = {
        "obstacles": [
            {"id": 1, "x": 5, "y": 5, "direction": 0},
            {"id": 2, "x": 10, "y": 8, "direction": 90},
            {"id": 3, "x": 15, "y": 15, "direction": 180},
            {"id": 4, "x": 7, "y": 12, "direction": 270},
            {"id": 5, "x": 12, "y": 3, "direction": 0}
        ]
    }

    obstacles = load_obstacles_from_json(obstacles_json)
    start = RobotConfig(x=15, y=15, theta=math.radians(90))

    print("Computing collision-free path using exhaustive search...")
    result = plan_path(start, obstacles, use_exhaustive=True)

    if 'error' in result:
        print(f"\nError: {result['error']}")
    else:
        print(f"\nBest path order: {result['order']}")
        print(f"Total path length: {result['total_length']:.2f} cm")
        print(f"\nSegments:")
        for i, seg in enumerate(result['segments'], 1):
            print(f"  {i}. To obstacle {seg['obstacle_id']}: {seg['path']}")
