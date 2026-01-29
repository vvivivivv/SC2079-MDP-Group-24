
"""
MDP Path Planning Module - Grid-Based A* with TSP Optimization
Adapted from MazeSolver for simulator compatibility
"""

import heapq
import math
from typing import List, Tuple, Dict, Optional
from dataclasses import dataclass
from enum import IntEnum
import numpy as np

# Try to import TSP solver, fallback to brute force if not available
try:
    from python_tsp.exact import solve_tsp_dynamic_programming
    HAS_TSP_SOLVER = True
except ImportError:
    HAS_TSP_SOLVER = False
    print("⚠ python-tsp not installed. Using brute force optimization.")
    print("  Install with: pip install python-tsp")


# =============================================================================
# CONFIGURATION
# =============================================================================

# Arena Settings
ARENA_SIZE = 200  # cm
GRID_SIZE = 20    # 20x20 grid (each cell = 10cm)
CELL_SIZE = 10    # cm per grid cell

# Robot Settings
ROBOT_SIZE = 30   # cm (3x3 cells)
ROBOT_RADIUS = 15 # cm

# Movement Settings
TURN_RADIUS = 25  # cm
TURN_FACTOR = 10  # Cost multiplier for turns
SAFE_COST = 20    # Penalty for being near obstacles
BIG_TURN = 0      # 0 = 3-1 turn, 1 = 4-2 turn

# Turn patterns: [bigger_change, smaller_change]
TURN_PATTERNS = [
    [3, 1],  # 3-1 turn
    [4, 2]   # 4-2 turn
]


# =============================================================================
# ENUMS
# =============================================================================

class Direction(IntEnum):
    """Cardinal directions"""
    NORTH = 0
    EAST = 1
    SOUTH = 2
    WEST = 3

    @staticmethod
    def rotation_cost(from_dir: 'Direction', to_dir: 'Direction') -> int:
        """Calculate cost of rotating from one direction to another"""
        diff = abs(from_dir - to_dir)
        return min(diff, 4 - diff)  # Shortest rotation


# Direction vectors for straight movement: (dx, dy, direction)
MOVE_DIRECTIONS = [
    (0, 1, Direction.NORTH),   # North: +y
    (1, 0, Direction.EAST),    # East: +x
    (0, -1, Direction.SOUTH),  # South: -y
    (-1, 0, Direction.WEST)    # West: -x
]


# =============================================================================
# DATA STRUCTURES
# =============================================================================

@dataclass
class CellState:
    """Represents a state: position + direction"""
    x: int  # Grid x coordinate
    y: int  # Grid y coordinate
    direction: Direction
    screenshot_id: Optional[int] = None  # Obstacle ID if taking photo
    penalty: int = 0  # Penalty cost for this position

    def is_eq(self, x: int, y: int, direction: Direction) -> bool:
        return self.x == x and self.y == y and self.direction == direction

    def set_screenshot(self, obstacle_id: int):
        self.screenshot_id = obstacle_id

    def __hash__(self):
        return hash((self.x, self.y, self.direction))

    def __eq__(self, other):
        if not isinstance(other, CellState):
            return False
        return self.x == other.x and self.y == other.y and self.direction == other.direction

    def __repr__(self):
        dir_str = ['N', 'E', 'S', 'W'][self.direction]
        return f"CellState({self.x}, {self.y}, {dir_str})"


@dataclass
class Obstacle:
    """Obstacle with position and image direction"""
    id: int
    x: int  # Grid x coordinate (0-19)
    y: int  # Grid y coordinate (0-19)
    direction: int  # 0=North, 90=East, 180=South, 270=West

    def get_direction_enum(self) -> Direction:
        """Convert degrees to Direction enum"""
        return Direction(self.direction // 90)


# =============================================================================
# GRID CLASS
# =============================================================================

class Grid:
    """Grid representation of the arena"""

    def __init__(self, size_x: int, size_y: int):
        self.size_x = size_x
        self.size_y = size_y
        self.obstacles: List[Obstacle] = []
        # Create grid of booleans (True = blocked)
        self.cells = [[False for _ in range(size_x)] for _ in range(size_y)]

    def add_obstacle(self, obstacle: Obstacle):
        """Add obstacle and mark surrounding cells as blocked"""
        self.obstacles.append(obstacle)
        # Mark 3x3 area as blocked (obstacle + robot safety margin)
        for dy in range(-1, 2):
            for dx in range(-1, 2):
                nx, ny = obstacle.x + dx, obstacle.y + dy
                if 0 <= nx < self.size_x and 0 <= ny < self.size_y:
                    self.cells[ny][nx] = True

    def reset_obstacles(self):
        """Clear all obstacles"""
        self.obstacles.clear()
        self.cells = [[False for _ in range(self.size_x)] for _ in range(self.size_y)]

    def reachable(self, x: int, y: int, turn=False, preTurn=False) -> bool:
        """Check if position is reachable (not blocked and within bounds)"""
        # Check bounds
        if x < 0 or x >= self.size_x or y < 0 or y >= self.size_y:
            return False

        # Check if blocked by obstacle
        if self.cells[y][x]:
            return False

        # Additional check for turns: robot needs 3x3 space
        if turn or preTurn:
            for dy in range(-1, 2):
                for dx in range(-1, 2):
                    nx, ny = x + dx, y + dy
                    if nx < 0 or nx >= self.size_x or ny < 0 or ny >= self.size_y:
                        return False
                    if self.cells[ny][nx]:
                        return False

        return True

    def get_view_obstacle_positions(self, retrying=False) -> List[List[CellState]]:
        """
        Get all valid positions to view each obstacle.
        Returns list of lists: one list of positions per obstacle.
        """
        all_positions = []

        for obs in self.obstacles:
            positions = []

            # Calculate target position based on image direction
            # Robot needs to be 4 cells away (40cm = camera distance + robot offset)
            if obs.direction == 0:  # North - image on top
                target_x, target_y = obs.x, obs.y + 4
                target_dir = Direction.NORTH
            elif obs.direction == 90:  # East - image on right
                target_x, target_y = obs.x + 4, obs.y
                target_dir = Direction.EAST
            elif obs.direction == 180:  # South - image on bottom
                target_x, target_y = obs.x, obs.y - 4
                target_dir = Direction.SOUTH
            else:  # 270, West - image on left
                target_x, target_y = obs.x - 4, obs.y
                target_dir = Direction.WEST

            # Check if target position is valid
            if self.reachable(target_x, target_y):
                state = CellState(target_x, target_y, target_dir, obs.id, penalty=0)
                positions.append(state)

            # If retrying and no valid position, try alternatives
            if not positions and retrying:
                # Try adjacent positions
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    alt_x, alt_y = target_x + dx, target_y + dy
                    if self.reachable(alt_x, alt_y):
                        state = CellState(alt_x, alt_y, target_dir, obs.id, penalty=50)
                        positions.append(state)

            all_positions.append(positions)

        return all_positions


# =============================================================================
# PATH PLANNING
# =============================================================================

class MazeSolver:
    """A* based path planner with TSP optimization"""

    def __init__(self, big_turn: int = BIG_TURN):
        self.grid = Grid(GRID_SIZE, GRID_SIZE)
        self.big_turn = big_turn
        self.path_table: Dict[Tuple[CellState, CellState], List] = {}
        self.cost_table: Dict[Tuple[CellState, CellState], float] = {}

    def add_obstacle(self, obstacle: Obstacle):
        """Add obstacle to grid"""
        self.grid.add_obstacle(obstacle)

    def get_safe_cost(self, x: int, y: int) -> int:
        """
        Get penalty cost if position is dangerously close to obstacles.
        Penalize positions that are exactly 2 units away in both x and y.
        """
        for obs in self.grid.obstacles:
            dx, dy = abs(obs.x - x), abs(obs.y - y)
            if (dx == 2 and dy == 2) or (dx == 1 and dy == 2) or (dx == 2 and dy == 1):
                return SAFE_COST
        return 0

    def get_neighbors(self, x: int, y: int, direction: Direction) -> List[Tuple[int, int, Direction, int]]:
        """
        Get valid neighbor positions from current state.
        Returns list of (new_x, new_y, new_direction, safe_cost)
        """
        neighbors = []
        bigger, smaller = TURN_PATTERNS[self.big_turn]

        # Straight movements
        for dx, dy, md in MOVE_DIRECTIONS:
            if md == direction:
                # Forward
                if self.grid.reachable(x + dx, y + dy):
                    safe_cost = self.get_safe_cost(x + dx, y + dy)
                    neighbors.append((x + dx, y + dy, md, safe_cost))
                # Backward
                if self.grid.reachable(x - dx, y - dy):
                    safe_cost = self.get_safe_cost(x - dx, y - dy)
                    neighbors.append((x - dx, y - dy, md, safe_cost))

        # Turn movements (8 cases: 4 directions × 2 turn types)
        turn_moves = [
            # (from_dir, to_dir, [(dx1, dy1), (dx2, dy2)])
            (Direction.NORTH, Direction.EAST, [(bigger, smaller), (-smaller, -bigger)]),
            (Direction.EAST, Direction.NORTH, [(smaller, bigger), (-bigger, -smaller)]),
            (Direction.EAST, Direction.SOUTH, [(smaller, -bigger), (-bigger, smaller)]),
            (Direction.SOUTH, Direction.EAST, [(bigger, -smaller), (-smaller, bigger)]),
            (Direction.SOUTH, Direction.WEST, [(-bigger, -smaller), (smaller, bigger)]),
            (Direction.WEST, Direction.SOUTH, [(-smaller, -bigger), (bigger, smaller)]),
            (Direction.WEST, Direction.NORTH, [(-smaller, bigger), (bigger, -smaller)]),
            (Direction.NORTH, Direction.WEST, [(smaller, -bigger), (-bigger, smaller)]),
        ]

        for from_dir, to_dir, moves in turn_moves:
            if direction == from_dir:
                for dx, dy in moves:
                    nx, ny = x + dx, y + dy
                    if self.grid.reachable(nx, ny, turn=True) and self.grid.reachable(x, y, preTurn=True):
                        safe_cost = self.get_safe_cost(nx, ny)
                        neighbors.append((nx, ny, to_dir, safe_cost + 10))  # +10 for turn

        return neighbors

    def astar_search(self, start: CellState, end: CellState):
        """A* search from start to end state"""
        # Skip if already computed
        if (start, end) in self.path_table:
            return

        g_distance = {(start.x, start.y, start.direction): 0}
        heap = [(0, start.x, start.y, start.direction)]
        parent = {}
        visited = set()

        while heap:
            _, cur_x, cur_y, cur_dir = heapq.heappop(heap)

            if (cur_x, cur_y, cur_dir) in visited:
                continue

            # Check if reached goal
            if end.is_eq(cur_x, cur_y, cur_dir):
                self._record_path(start, end, parent, g_distance[(cur_x, cur_y, cur_dir)])
                return

            visited.add((cur_x, cur_y, cur_dir))
            cur_distance = g_distance[(cur_x, cur_y, cur_dir)]

            # Explore neighbors
            for next_x, next_y, new_dir, safe_cost in self.get_neighbors(cur_x, cur_y, cur_dir):
                if (next_x, next_y, new_dir) in visited:
                    continue

                # Calculate movement cost
                move_cost = Direction.rotation_cost(new_dir, cur_dir) * TURN_FACTOR + 1 + safe_cost
                new_g = cur_distance + move_cost

                # Heuristic: Manhattan distance
                h = abs(next_x - end.x) + abs(next_y - end.y)
                f = new_g + h

                if (next_x, next_y, new_dir) not in g_distance or g_distance[(next_x, next_y, new_dir)] > new_g:
                    g_distance[(next_x, next_y, new_dir)] = new_g
                    parent[(next_x, next_y, new_dir)] = (cur_x, cur_y, cur_dir)
                    heapq.heappush(heap, (f, next_x, next_y, new_dir))

    def _record_path(self, start: CellState, end: CellState, parent: dict, cost: float):
        """Record computed path in tables"""
        self.cost_table[(start, end)] = cost
        self.cost_table[(end, start)] = cost

        path = []
        cursor = (end.x, end.y, end.direction)

        while cursor in parent:
            path.append(cursor)
            cursor = parent[cursor]
        path.append(cursor)

        self.path_table[(start, end)] = path[::-1]
        self.path_table[(end, start)] = path

    def path_cost_generator(self, states: List[CellState]):
        """Generate paths between all state pairs using A*"""
        for i in range(len(states) - 1):
            for j in range(i + 1, len(states)):
                self.astar_search(states[i], states[j])

    def get_optimal_order(self, start_state: CellState, retrying=False) -> Tuple[List[CellState], float]:
        """
        Find optimal visiting order using TSP solver or brute force.
        Returns (path, total_distance)
        """
        # Get all viewing positions
        all_view_positions = self.grid.get_view_obstacle_positions(retrying)

        if not all(positions for positions in all_view_positions):
            print("⚠ Some obstacles have no valid viewing positions!")
            return [], float('inf')

        # For simplicity, use first viewing position for each obstacle
        # (Full implementation would try combinations)
        items = [start_state]
        view_positions = []

        for positions in all_view_positions:
            items.append(positions[0])
            view_positions.append(positions[0])

        # Generate path costs
        self.path_cost_generator(items)

        # Build cost matrix
        n = len(items)
        cost_matrix = np.full((n, n), 1e9)

        for i in range(n):
            for j in range(n):
                if i == j:
                    cost_matrix[i][j] = 0
                elif (items[i], items[j]) in self.cost_table:
                    cost_matrix[i][j] = self.cost_table[(items[i], items[j])]

        # Make start position have 0 cost to return to
        cost_matrix[:, 0] = 0

        # Solve TSP
        if HAS_TSP_SOLVER:
            permutation, distance = solve_tsp_dynamic_programming(cost_matrix)
        else:
            # Fallback: nearest neighbor
            permutation, distance = self._nearest_neighbor_tsp(cost_matrix)

        print(f"TSP Permutation: {permutation}")
        print(f"Items mapping: {[f'Start' if i == 0 else f'Obs{items[i].screenshot_id}' for i in range(len(items))]}")

        # Build optimal path by following the permutation
        optimal_path = []
        total_distance = 0

        for i in range(len(permutation) - 1):
            from_idx = permutation[i]
            to_idx = permutation[i + 1]

            from_state = items[from_idx]
            to_state = items[to_idx]

            print(f"Segment {i+1}: {from_idx} -> {to_idx}", end="")

            # Skip returning to start (last segment in TSP)
            if to_idx == 0:
                print(" (skipping return to start)")
                break

            if (from_state, to_state) not in self.path_table:
                print(f" ⚠ NO PATH FOUND!")
                continue

            path_segment = self.path_table[(from_state, to_state)]
            segment_cost = self.cost_table[(from_state, to_state)]
            total_distance += segment_cost

            print(f" (cost: {segment_cost:.1f}, steps: {len(path_segment)})")

            # Add all steps in this segment
            for j, step in enumerate(path_segment):
                state = CellState(step[0], step[1], step[2])

                # Mark screenshot at the END of segment (at obstacle)
                if j == len(path_segment) - 1 and to_idx > 0:
                    state.set_screenshot(items[to_idx].screenshot_id)
                    print(f"  → Taking photo of obstacle {items[to_idx].screenshot_id} at ({state.x}, {state.y})")

                optimal_path.append(state)

        print(f"\nTotal path length: {len(optimal_path)} steps")
        print(f"Obstacles visited: {[s.screenshot_id for s in optimal_path if s.screenshot_id is not None]}")

        return optimal_path, total_distance

    def _nearest_neighbor_tsp(self, cost_matrix):
        """Simple nearest neighbor TSP heuristic"""
        n = len(cost_matrix)
        visited = {0}
        tour = [0]
        total_cost = 0

        current = 0
        while len(visited) < n:
            best_next = None
            best_cost = float('inf')

            for next_node in range(n):
                if next_node not in visited and cost_matrix[current][next_node] < best_cost:
                    best_cost = cost_matrix[current][next_node]
                    best_next = next_node

            if best_next is not None:
                tour.append(best_next)
                visited.add(best_next)
                total_cost += best_cost
                current = best_next

        return tour, total_cost


# =============================================================================
# PUBLIC API (Compatible with simulator.py)
# =============================================================================


def plan_path(start_config, obstacles: List[Obstacle], use_exhaustive: bool = True) -> Dict:
    """
    Main path planning function - compatible with simulator interface.

    Args:
        start_config: RobotConfig with x, y (in cm), theta (in radians)
        obstacles: List of Obstacle objects
        use_exhaustive: Whether to use optimal TSP solver (ignored, always optimal)

    Returns:
        Dict with 'order', 'total_length', 'segments'
    """
    # Convert continuous coordinates to grid
    start_x = int(start_config.x / CELL_SIZE)
    start_y = int(start_config.y / CELL_SIZE)
    start_theta_deg = math.degrees(start_config.theta) % 360
    start_dir = Direction(int((start_theta_deg + 45) / 90) % 4)

    # Create solver
    solver = MazeSolver()

    # Add obstacles
    for obs in obstacles:
        solver.add_obstacle(obs)

    # Get optimal path
    start_state = CellState(start_x, start_y, start_dir)
    path, total_cost = solver.get_optimal_order(start_state, retrying=False)

    if not path:
        return {
            'order': [],
            'total_length': 0,
            'segments': [],
            'error': 'No valid path found'
        }

    # Extract visiting order and build segments
    order = []
    segments = []
    segment_start_idx = 0

    print(f"\nBuilding segments from {len(path)} path steps...")

    for i, state in enumerate(path):
        # Check if this is a photo point (obstacle visit)
        if state.screenshot_id is not None:
            obstacle_id = state.screenshot_id

            # Only add to order if not already there
            if obstacle_id not in order:
                order.append(obstacle_id)

            # Get start and end states for this segment
            start_state = path[segment_start_idx]
            end_state = state

            # Convert to cm coordinates for visualization
            from_x = start_state.x * CELL_SIZE + CELL_SIZE / 2
            from_y = start_state.y * CELL_SIZE + CELL_SIZE / 2
            from_theta = start_state.direction * math.pi / 2

            to_x = end_state.x * CELL_SIZE + CELL_SIZE / 2
            to_y = end_state.y * CELL_SIZE + CELL_SIZE / 2
            to_theta = end_state.direction * math.pi / 2

            # Create RobotConfig objects for compatibility
            from dataclasses import dataclass
            @dataclass
            class FakeConfig:
                x: float
                y: float
                theta: float

            segment = {
                'from': FakeConfig(from_x, from_y, from_theta),
                'to': FakeConfig(to_x, to_y, to_theta),
                'obstacle_id': obstacle_id,
                'path_steps': path[segment_start_idx:i+1]  # Include discrete steps
            }

            segments.append(segment)
            print(f"  Segment {len(segments)}: Obstacle {obstacle_id} at ({to_x:.1f}, {to_y:.1f})")

            # Next segment starts from here
            segment_start_idx = i

    # Convert cost to approximate cm
    total_length = total_cost * CELL_SIZE

    print(f"\nCreated {len(segments)} segments for {len(order)} obstacles")
    print(f"Order: {order}")

    return {
        'order': order,
        'total_length': total_length,
        'segments': segments,
        'full_path': path  # Full discrete path for detailed visualization
    }



def load_obstacles_from_json(data: Dict) -> List[Obstacle]:
    """Load obstacles from JSON"""
    obstacles = []
    for obs_data in data.get('obstacles', []):
        obstacles.append(Obstacle(
            id=obs_data['id'],
            x=obs_data['x'],
            y=obs_data['y'],
            direction=obs_data['direction']
        ))
    return obstacles


# Backwards compatibility
class RobotConfig:
    """Dummy class for compatibility"""
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta


if __name__ == "__main__":
    print("MazeSolver-based path planning module loaded")
    print(f"TSP Solver available: {HAS_TSP_SOLVER}")
