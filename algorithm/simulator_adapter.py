
"""
Simulator Adapter - Bridges simulator.py to original pathfinding code
DOES NOT modify original pathfinding files, just translates formats
"""

from __future__ import annotations
from dataclasses import dataclass
import math
from typing import Optional

# Import from YOUR original pathfinding modules
from pathfinding.search.search import search, Segment
from pathfinding.world.objective import generate_objectives
from pathfinding.world.primitives import Direction, Point, Vector
from pathfinding.world.world import Obstacle as PathObstacle, Robot, World

# Grid configuration matching original design
GRID_SIZE = 20  # 20x20 grid
CELL_SIZE = 10  # 10cm per cell

# Simulator's data classes (for compatibility)
@dataclass
class RobotConfig:
    """Robot configuration for simulator compatibility"""
    x: float  # cm
    y: float  # cm
    theta: float  # radians


@dataclass
class Obstacle:
    """Obstacle for simulator compatibility"""
    id: int
    x: int  # grid coordinate (0-19)
    y: int  # grid coordinate (0-19)
    direction: int  # degrees (0, 90, 180, 270)


def plan_path(start_config: RobotConfig, obstacles: list[Obstacle], use_exhaustive: bool = True) -> dict:
    """
    Main entry point for simulator.py
    Converts simulator format → your pathfinding format → runs search → converts back

    Args:
        start_config: RobotConfig with x, y (cm), theta (radians)
        obstacles: List of Obstacle objects with grid coordinates
        use_exhaustive: Ignored (your search algorithm is used as-is)

    Returns:
        Dict compatible with simulator expectations
    """

    try:
        print(f"\n{'='*60}")
        print(f"🧠 PATHFINDING: Starting...")
        print(f"   Robot: ({start_config.x:.1f}cm, {start_config.y:.1f}cm) @ {math.degrees(start_config.theta):.0f}°")
        print(f"   Obstacles: {len(obstacles)}")
        print(f"{'='*60}\n")

        # Convert simulator format to your World format
        world = _create_world(start_config, obstacles)

        # Generate objectives using YOUR original function
        objectives = generate_objectives(world)

        if not objectives:
            return {
                'order': [],
                'total_length': 0,
                'segments': [],
                'error': 'Could not generate viewing positions for obstacles'
            }

        print(f"✓ Generated {len(objectives)} objectives")

        # Run YOUR original search algorithm
        segments = search(world, objectives)

        if not segments:
            return {
                'order': [],
                'total_length': 0,
                'segments': [],
                'error': 'No valid path found'
            }

        print(f"✓ Found path with {len(segments)} segments\n")

        # Convert back to simulator format
        order = [seg.image_id for seg in segments]
        total_length = sum(seg.cost for seg in segments)

        sim_segments = []
        for seg in segments:
            if seg.vectors:
                start_vec = seg.vectors[0]
                end_vec = seg.vectors[-1]
                
                # Vectors are in grid coordinates, convert to cm
                from_x = start_vec.x * CELL_SIZE  # Grid to cm
                from_y = start_vec.y * CELL_SIZE
                from_theta = _direction_to_radians(start_vec.direction)

                to_x = end_vec.x * CELL_SIZE
                to_y = end_vec.y * CELL_SIZE
                to_theta = _direction_to_radians(end_vec.direction)

                sim_segments.append({
                    'from': RobotConfig(from_x, from_y, from_theta),
                    'to': RobotConfig(to_x, to_y, to_theta),
                    'obstacle_id': seg.image_id,
                    'path': seg.vectors,
                    'instructions': seg.instructions
                })

        print(f"{'='*60}")
        print(f"✓ Path planning complete!")
        print(f"   Order: {order}")
        print(f"   Total: {total_length:.1f}cm")
        print(f"{'='*60}\n")

        return {
            'order': order,
            'total_length': total_length,
            'segments': sim_segments
        }

    except Exception as e:
        import traceback
        print(f"\n❌ Error: {e}")
        traceback.print_exc()
        return {
            'order': [],
            'total_length': 0,
            'segments': [],
            'error': str(e)
        }


def _create_world(start_config: RobotConfig, obstacles: list[Obstacle]) -> World:
    """
    Convert simulator format to your World format.
    Uses grid size of 20 with 10cm cells (not 200 with 1cm cells!)
    """
    
    # Use 20x20 grid with 10cm cells to match your original design
    grid_size = 20
    cell_size = 10  # cm per cell
    
    # Convert robot position from cm to grid coordinates
    robot_grid_x = int(start_config.x / cell_size)
    robot_grid_y = int(start_config.y / cell_size)
    robot_grid_size = 30 // cell_size  # 30cm = 3 grid cells
    robot_half = robot_grid_size // 2
    
    # Convert theta to Direction
    theta_deg = math.degrees(start_config.theta) % 360
    if 315 <= theta_deg or theta_deg < 45:
        robot_dir = Direction.EAST
    elif 45 <= theta_deg < 135:
        robot_dir = Direction.NORTH
    elif 135 <= theta_deg < 225:
        robot_dir = Direction.WEST
    else:
        robot_dir = Direction.SOUTH
    
    # Create robot (ensure bounds)
    robot = Robot(
        direction=robot_dir,
        south_west=Point(
            max(0, robot_grid_x - robot_half),
            max(0, robot_grid_y - robot_half)
        ),
        north_east=Point(
            min(grid_size - 1, robot_grid_x + robot_half),
            min(grid_size - 1, robot_grid_y + robot_half)
        )
    )
    
    # Convert obstacles - they're already in grid coordinates (0-19)!
    obstacle_entities = []
    for obs in obstacles:
        # obs.x and obs.y are ALREADY grid coordinates (0-19)
        # No conversion needed!
        
        # Convert direction
        if obs.direction == 0:
            obs_dir = Direction.NORTH
        elif obs.direction == 90:
            obs_dir = Direction.EAST
        elif obs.direction == 180:
            obs_dir = Direction.SOUTH
        else:  # 270
            obs_dir = Direction.WEST
        
        # Obstacles are 1 grid cell (10cm)
        obstacle_entities.append(PathObstacle(
            direction=obs_dir,
            south_west=Point(obs.x, obs.y),
            north_east=Point(obs.x, obs.y),  # Single cell
            image_id=obs.id
        ))
    
    # Create world with correct grid size
    world = World(
        size=grid_size,  # 20, not 200!
        robot=robot,
        obstacles=obstacle_entities
    )
    
    return world



def _direction_to_radians(direction: Direction) -> float:
    """Convert Direction enum to radians"""
    match direction:
        case Direction.NORTH:
            return math.pi / 2  # 90 degrees
        case Direction.EAST:
            return 0  # 0 degrees
        case Direction.SOUTH:
            return -math.pi / 2  # -90 degrees
        case Direction.WEST:
            return math.pi  # 180 degrees
        case _:
            return 0
