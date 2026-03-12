"""
MazeSolver — v1-style exhaustive pathfinding.

Architecture (same as v1, with accurate physics):
  1. Generate capture candidates for each obstacle          (Entity.py)
  2. Run grid A* between ALL candidate pairs                (built-in)
  3. Build EXACT cost matrix from real A* distances          (built-in)
  4. Solve TSP with dynamic programming for optimal order    (python_tsp)
  5. Reconstruct full path from stored A* paths              (built-in)
  6. Generate commands from the path states                  (built-in)

Grid: 5cm cells (40×40), 4 cardinal headings.
Turn displacements: computed from real measurements in consts.py.
Max rounding error per turn: ~1.5cm (vs v1's ~15-29cm with 3-1 model).

Command set:
  FW{mm}/BW{mm}   — straight drive
  FL90/FR90/BL90/BR90 — 90° turns
  SNAP{id}         — capture image
  FIN              — mission complete
"""

import math
import heapq
import time
from typing import List, Tuple
import numpy as np

from entities.Robot import Robot
from entities.Entity import Obstacle, CellState, Grid
from consts import (
    Direction, SAFE_COST, TURN_FACTOR, ITERATIONS,
    SCALE_FW, OFFSET_FW, SCALE_BW, OFFSET_BW,
    TURN_FL90_DX_CM, TURN_FL90_DY_CM,
    TURN_FR90_DX_CM, TURN_FR90_DY_CM,
    TURN_BL90_DX_CM, TURN_BL90_DY_CM,
    TURN_BR90_DX_CM, TURN_BR90_DY_CM,
)
from python_tsp.exact import solve_tsp_dynamic_programming


# =============================================================================
# GRID CONSTANTS
# =============================================================================

GRID_CM = 5                              # 5cm per cell
ARENA_CM = 200.0
ARENA_CELLS = int(ARENA_CM / GRID_CM)    # 40

ROBOT_CLEARANCE_CM = 15.0
_MIN_CELL = math.ceil(ROBOT_CLEARANCE_CM / GRID_CM)   # 3
_MAX_CELL = ARENA_CELLS - 1 - _MIN_CELL               # 36

# Obstacle clearance (in cm)
OBS_CLEAR_MANHATTAN = 35.0
OBS_CLEAR_MAX = 18.0          # for straight moves
OBS_CLEAR_TURN = 25.0         # for turn endpoints
OBS_SAFE_DIST = 22.0          # triggers SAFE_COST

MAX_A_STAR_ITERATIONS = 200000


# =============================================================================
# DIRECTION HELPERS
# =============================================================================

_DIR_TO_RAD = {
    Direction.NORTH: math.pi / 2,
    Direction.EAST:  0.0,
    Direction.SOUTH: -math.pi / 2,
    Direction.WEST:  math.pi,
}

_STRAIGHT_MOVES = [
    (1, 0, Direction.EAST),
    (-1, 0, Direction.WEST),
    (0, 1, Direction.NORTH),
    (0, -1, Direction.SOUTH),
]


def _rad_to_direction(theta):
    t = theta % (2 * math.pi)
    bucket = int(round(t / (math.pi / 2))) % 4
    return [Direction.EAST, Direction.NORTH, Direction.WEST, Direction.SOUTH][bucket]


# =============================================================================
# TURN DISPLACEMENT TABLE — computed once from consts.py
# =============================================================================

def _build_turn_table():
    """Precompute grid displacements for every (heading, turn) combination.

    Uses measured DX/DY from consts.py, rounded to 5cm grid.
    Heading rules (car physics):
        FL90: CCW (+90°)    FR90: CW (-90°)
        BL90: CW (-90°)     BR90: CCW (+90°)
    """
    turn_defs = {
        'FL90': (TURN_FL90_DX_CM, TURN_FL90_DY_CM, +math.pi / 2),
        'FR90': (TURN_FR90_DX_CM, TURN_FR90_DY_CM, -math.pi / 2),
        'BL90': (TURN_BL90_DX_CM, TURN_BL90_DY_CM, -math.pi / 2),
        'BR90': (TURN_BR90_DX_CM, TURN_BR90_DY_CM, +math.pi / 2),
    }

    table = {}
    for from_dir in [Direction.NORTH, Direction.EAST, Direction.SOUTH, Direction.WEST]:
        theta = _DIR_TO_RAD[from_dir]
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)

        for name, (ldx, ldy, dtheta) in turn_defs.items():
            wx = ldx * cos_t - ldy * sin_t
            wy = ldx * sin_t + ldy * cos_t
            dx = round(wx / GRID_CM)
            dy = round(wy / GRID_CM)
            new_theta = (theta + dtheta) % (2 * math.pi)
            new_dir = _rad_to_direction(new_theta)
            table[(from_dir, name)] = (dx, dy, new_dir)

    return table


_TURN_TABLE = _build_turn_table()


# =============================================================================
# MAZE SOLVER
# =============================================================================

class MazeSolver:
    def __init__(self, size_x, size_y, robot_x, robot_y,
                 robot_direction: Direction, big_turn=None):
        self.grid = Grid(size_x, size_y)
        self.robot = Robot(robot_x, robot_y, robot_direction)
        self.size_x = size_x
        self.size_y = size_y

        # Pairwise A* results (keyed by CellState pairs)
        self.path_table = dict()
        self.cost_table = dict()

        # Obstacle centers in cm (for grid collision checks)
        self._obs_cm = []

    def add_obstacle(self, x, y, direction: Direction, obstacle_id: int):
        obstacle = Obstacle(x, y, direction, obstacle_id)
        self.grid.add_obstacle(obstacle)
        self._obs_cm.append((x * 10 + 5, y * 10 + 5))

    def reset_obstacles(self):
        self.grid.reset_obstacles()
        self._obs_cm = []

    # =================================================================
    # COORDINATE CONVERSION
    # =================================================================

    @staticmethod
    def _state_to_grid(state: CellState):
        """Convert CellState to (grid_x, grid_y, Direction)."""
        if hasattr(state, 'theta_rad') and state.theta_rad is not None:
            gx = int(round(state.x * 10 / GRID_CM))
            gy = int(round(state.y * 10 / GRID_CM))
        else:
            gx = int(round((state.x * 10 + 5) / GRID_CM))
            gy = int(round((state.y * 10 + 5) / GRID_CM))
        gx = max(_MIN_CELL, min(_MAX_CELL, gx))
        gy = max(_MIN_CELL, min(_MAX_CELL, gy))
        return gx, gy, state.direction

    # =================================================================
    # GRID COLLISION
    # =================================================================

    def _is_valid_cell(self, x, y):
        return _MIN_CELL <= x <= _MAX_CELL and _MIN_CELL <= y <= _MAX_CELL

    def _reachable(self, x, y, turn=False):
        """Check if grid cell is reachable (no obstacle collision)."""
        if not self._is_valid_cell(x, y):
            return False

        px = x * GRID_CM
        py = y * GRID_CM

        for ox, oy in self._obs_cm:
            dx = abs(px - ox)
            dy = abs(py - oy)

            if dx + dy >= OBS_CLEAR_MANHATTAN:
                continue

            threshold = OBS_CLEAR_TURN if turn else OBS_CLEAR_MAX
            if max(dx, dy) < threshold:
                return False

        return True

    def _get_safe_cost(self, x, y):
        """Penalty for being close to an obstacle."""
        px = x * GRID_CM
        py = y * GRID_CM

        for ox, oy in self._obs_cm:
            dx = abs(px - ox)
            dy = abs(py - oy)
            if dx <= OBS_SAFE_DIST and dy <= OBS_SAFE_DIST:
                if max(dx, dy) >= OBS_CLEAR_MAX:
                    return SAFE_COST
        return 0

    # =================================================================
    # NEIGHBOR GENERATION
    # =================================================================

    def _get_neighbors(self, x, y, direction):
        """Generate neighbors: straight moves + real turn displacements.

        Returns: [(new_x, new_y, new_dir, cost, move_name), ...]
        """
        neighbors = []

        # Straight moves (1 cell = 5cm)
        for dx, dy, md in _STRAIGHT_MOVES:
            if md == direction:
                # Forward
                nx, ny = x + dx, y + dy
                if self._reachable(nx, ny):
                    sc = self._get_safe_cost(nx, ny)
                    neighbors.append((nx, ny, md, 1 + sc, 'FW'))
                # Backward
                nx, ny = x - dx, y - dy
                if self._reachable(nx, ny):
                    sc = self._get_safe_cost(nx, ny)
                    neighbors.append((nx, ny, md, 1 + sc, 'BW'))

        # Turns
        for turn_name in ('FL90', 'FR90', 'BL90', 'BR90'):
            key = (direction, turn_name)
            if key not in _TURN_TABLE:
                continue
            dx, dy, new_dir = _TURN_TABLE[key]
            nx, ny = x + dx, y + dy
            if (self._reachable(nx, ny, turn=True) and
                    self._reachable(x, y, turn=True)):
                sc = self._get_safe_cost(nx, ny)
                turn_cost = (Direction.rotation_cost(direction, new_dir)
                             * TURN_FACTOR + 1 + sc + 10)
                neighbors.append((nx, ny, new_dir, turn_cost, turn_name))

        return neighbors

    # =================================================================
    # PAIRWISE A* (v1's approach — all pairs computed upfront)
    # =================================================================

    def _astar(self, start_state: CellState, end_state: CellState):
        """Run A* between two CellStates. Store result in path_table/cost_table.

        Skips if this pair was already computed.
        Path format: [(gx, gy, dir, move_name), ...]
          - First element has move=None (origin — no move to get here)
          - All others have the move that was executed to reach that state
        """
        if (start_state, end_state) in self.path_table:
            return

        sx, sy, sd = self._state_to_grid(start_state)
        ex, ey, ed = self._state_to_grid(end_state)

        g_dist = {(sx, sy, sd): 0}
        parent = {}
        visited = set()

        def h(x, y, d):
            return abs(x - ex) + abs(y - ey) + Direction.rotation_cost(d, ed) * TURN_FACTOR

        counter = 0
        heap = [(h(sx, sy, sd), counter, sx, sy, sd)]

        found = False
        iterations = 0

        while heap and iterations < MAX_A_STAR_ITERATIONS:
            iterations += 1
            _, _, cx, cy, cd = heapq.heappop(heap)

            if (cx, cy, cd) in visited:
                continue

            if cx == ex and cy == ey and cd == ed:
                found = True
                break

            visited.add((cx, cy, cd))
            cur_g = g_dist[(cx, cy, cd)]

            for nx, ny, nd, cost, move in self._get_neighbors(cx, cy, cd):
                if (nx, ny, nd) in visited:
                    continue
                new_g = cur_g + cost
                if (nx, ny, nd) not in g_dist or new_g < g_dist[(nx, ny, nd)]:
                    g_dist[(nx, ny, nd)] = new_g
                    parent[(nx, ny, nd)] = ((cx, cy, cd), move)
                    counter += 1
                    heapq.heappush(heap, (new_g + h(nx, ny, nd), counter, nx, ny, nd))

        if not found:
            self.cost_table[(start_state, end_state)] = 1e9
            return

        # Reconstruct
        cost = g_dist[(ex, ey, ed)]
        path = []
        cursor = (ex, ey, ed)
        while cursor in parent:
            prev_state, move = parent[cursor]
            path.append((cursor[0], cursor[1], cursor[2], move))
            cursor = prev_state
        path.append((cursor[0], cursor[1], cursor[2], None))  # origin
        path.reverse()

        self.cost_table[(start_state, end_state)] = cost
        self.path_table[(start_state, end_state)] = path

    def _path_cost_generator(self, states: List[CellState]):
        """Run A* for all directed pairs (both directions)."""
        for i in range(len(states)):
            for j in range(len(states)):
                if i != j:
                    self._astar(states[i], states[j])

    # =================================================================
    # TSP ORDERING (from v1 — exact costs)
    # =================================================================

    @staticmethod
    def _get_visit_options(n):
        """Generate all n-digit binary strings, sorted by most 1s first."""
        s = []
        l = bin(2 ** n - 1).count('1')
        for i in range(2 ** n):
            s.append(bin(i)[2:].zfill(l))
        s.sort(key=lambda x: x.count('1'), reverse=True)
        return s

    @staticmethod
    def _generate_combination(view_positions, index, current, result, iteration_left):
        if index == len(view_positions):
            result.append(current[:])
            return
        if iteration_left[0] == 0:
            return
        iteration_left[0] -= 1
        for j in range(len(view_positions[index])):
            current.append(j)
            MazeSolver._generate_combination(
                view_positions, index + 1, current, result, iteration_left)
            current.pop()

    def get_optimal_order_dp(self, retrying) -> Tuple[List[CellState], float]:
        """Find optimal visit order using exact A* costs (v1's approach).

        1. Get all candidate view positions for each obstacle
        2. For each binary mask of which obstacles to visit (most first):
           a. Collect all candidate states into `items`
           b. Run pairwise A* between all items
           c. For each combination of candidates (one per obstacle):
              - Build cost matrix from exact A* costs
              - Solve TSP with DP
              - If better than best, reconstruct full path
        3. Return the best path
        """
        all_view_positions = self.grid.get_view_obstacle_positions(retrying)
        if not all_view_positions:
            return [], 1e9

        best_distance = 1e9
        best_path = []
        start_state = self.robot.get_start_state()

        for op in self._get_visit_options(len(all_view_positions)):
            # Build items list: [start_state] + all candidates for selected obstacles
            items = [start_state]
            cur_view_positions = []

            for idx in range(len(all_view_positions)):
                if op[idx] == '1':
                    items = items + all_view_positions[idx]
                    cur_view_positions.append(all_view_positions[idx])

            if not cur_view_positions:
                continue

            # Run pairwise A* for all items
            self._path_cost_generator(items)

            # Try all candidate combinations
            combinations = []
            self._generate_combination(
                cur_view_positions, 0, [], combinations, [ITERATIONS])

            for c in combinations:
                visited_candidates = [0]  # start state
                cur_index = 1
                fixed_cost = 0

                for obs_idx, view_pos_list in enumerate(cur_view_positions):
                    selected = cur_index + c[obs_idx]
                    visited_candidates.append(selected)
                    fixed_cost += view_pos_list[c[obs_idx]].penalty
                    cur_index += len(view_pos_list)

                # Build cost matrix from exact A* costs
                n = len(visited_candidates)
                cost_np = np.zeros((n, n))
                all_reachable = True

                for s in range(n):
                    for e in range(s + 1, n):
                        u = items[visited_candidates[s]]
                        v = items[visited_candidates[e]]
                        if (u, v) in self.cost_table:
                            cost_np[s][e] = self.cost_table[(u, v)]
                            cost_np[e][s] = cost_np[s][e]
                        else:
                            cost_np[s][e] = 1e9
                            cost_np[e][s] = 1e9
                            all_reachable = False

                if not all_reachable:
                    continue

                cost_np[:, 0] = 0  # open-path TSP (no return to start)

                _permutation, _distance = solve_tsp_dynamic_programming(cost_np)

                if _distance + fixed_cost >= best_distance:
                    continue

                # Reconstruct full path from stored A* paths
                best_distance = _distance + fixed_cost
                best_path = [items[0]]  # start state as CellState

                for i in range(len(_permutation) - 1):
                    from_item = items[visited_candidates[_permutation[i]]]
                    to_item = items[visited_candidates[_permutation[i + 1]]]

                    if (from_item, to_item) in self.path_table:
                        grid_path = self.path_table[(from_item, to_item)]
                        # Append grid states — all intermediate with screenshot=-1
                        for gx, gy, gd, move in grid_path[1:]:
                            best_path.append(
                                _GridState(gx, gy, gd, move, -1))

                    # Only the LAST state of each leg gets the screenshot ID
                    if best_path and to_item.screenshot_id != -1:
                        best_path[-1].screenshot_id = to_item.screenshot_id

            if best_path:
                break  # found a valid path visiting max obstacles

        return best_path, best_distance

    # =================================================================
    # COMMAND GENERATION
    # =================================================================

    @staticmethod
    def _path_to_commands(path, obstacles_data=None):
        """Generate commands from the full path.

        Move names are embedded in _GridState objects.
        Consecutive FW/BW are merged, but merging stops at SNAP points.
        SNAP is emitted immediately after the movement that reaches the
        capture position.
        """
        if len(path) < 2:
            return ["FIN"]

        # Step 1: Extract (move, screenshot_id) pairs from path states
        steps = []  # [(move_name, screenshot_id), ...]
        for state in path:
            if isinstance(state, _GridState):
                if state.move is None:
                    continue
                steps.append((state.move, state.screenshot_id))

        if not steps:
            return ["FIN"]

        # Step 2: Merge consecutive FW/BW, break at SNAP points and turns
        commands = []
        i = 0
        while i < len(steps):
            move, snap_id = steps[i]

            if move == 'FW':
                # Merge consecutive FW — but stop if a step has a SNAP
                count = 1
                while (i + count < len(steps) and
                       steps[i + count][0] == 'FW' and
                       steps[i + count - 1][1] == -1):  # don't merge past SNAP
                    count += 1
                dist_mm = count * GRID_CM * 10
                dist_mm = max(10, round(dist_mm * SCALE_FW + OFFSET_FW))
                commands.append(f"FW{dist_mm}")
                # SNAP at the last merged step if it has a screenshot
                last_snap = steps[i + count - 1][1]
                if last_snap != -1:
                    commands.append(f"SNAP{last_snap}")
                i += count

            elif move == 'BW':
                count = 1
                while (i + count < len(steps) and
                       steps[i + count][0] == 'BW' and
                       steps[i + count - 1][1] == -1):
                    count += 1
                dist_mm = count * GRID_CM * 10
                dist_mm = max(10, round(dist_mm * SCALE_BW + OFFSET_BW))
                commands.append(f"BW{dist_mm}")
                last_snap = steps[i + count - 1][1]
                if last_snap != -1:
                    commands.append(f"SNAP{last_snap}")
                i += count

            else:
                # Turn command: FL90, FR90, BL90, BR90
                commands.append(move)
                if snap_id != -1:
                    commands.append(f"SNAP{snap_id}")
                i += 1

        commands.append("FIN")
        return commands

    # =================================================================
    # MAIN ENTRY POINT
    # =================================================================

    def plan_full_route(self, retrying=False, obstacles_data=None):
        """Full pipeline: pairwise A* → exact TSP → reconstruct → commands.

        Returns:
            (commands, path, distance)
        """
        t0 = time.time()

        path, distance = self.get_optimal_order_dp(retrying)

        elapsed = time.time() - t0
        print(f"Planning: {elapsed:.2f}s, distance={distance:.0f}")

        if not path:
            print("No valid path found!")
            return ["FIN"], [], 1e9

        for s in path:
            if hasattr(s, 'screenshot_id') and s.screenshot_id != -1:
                d_name = s.direction.name if hasattr(s.direction, 'name') else s.direction
                if isinstance(s, _GridState):
                    print(f"  Visit ob{s.screenshot_id} at grid({s.x},{s.y}) {d_name}")
                else:
                    print(f"  Visit ob{s.screenshot_id} at ({s.x:.1f},{s.y:.1f}) {d_name}")

        commands = self._path_to_commands(path, obstacles_data)

        print(f"  {len(commands)} commands generated")
        return commands, path, distance


# =============================================================================
# INTERNAL HELPER CLASSES
# =============================================================================

class _GridState:
    """Lightweight state for grid A* paths with embedded move names."""
    __slots__ = ('x', 'y', 'direction', 'move', 'screenshot_id')

    def __init__(self, x, y, direction, move='FW', screenshot_id=-1):
        self.x = x
        self.y = y
        self.direction = direction
        self.move = move
        self.screenshot_id = screenshot_id

    def __repr__(self):
        return f"G({self.x},{self.y},{self.direction},{self.move},s={self.screenshot_id})"