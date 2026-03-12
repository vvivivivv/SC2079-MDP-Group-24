"""
MazeSolver: Pure Reeds-Shepp / Hybrid A* path planning.

ARCHITECTURE (optimized for physical speed):
  1. Generate capture candidates for each obstacle (Entity.py)
     - Each candidate has an exact theta_rad: the heading needed so the
       camera (at the front of the robot) faces the obstacle.
  2. Build TSP cost matrix using REEDS-SHEPP path length (heading-aware)
  3. Solve TSP for optimal visit order
  4. Plan each leg:
     a. Try direct Reeds-Shepp path (fastest — no stopping)
     b. If obstacles block it, fall back to Hybrid A* (heading-constrained)
     c. If that fails, A* position-only (fallback)
     d. SNAP at destination

  No spot-turns (CW) are used. The robot arrives at the correct heading
  purely through Ackermann driving (FL/FR/BL/BR/FW/BW). This eliminates:
    - Pivot drift from CW spins
    - Stop/start penalties
    - The need for unreliable CCW→CW conversion

  Command set:
    FW/BW/FL/FR/BL/BR  — Ackermann drive (from RS or A*)
    SNAP{id}            — Capture image
    FIN                 — Mission complete
"""

import math
import time
from typing import List, Tuple
import numpy as np

from entities.Robot import Robot
from entities.Entity import Obstacle, CellState, Grid
from consts import (
    Direction, ITERATIONS,
    ROBOT_TURN_RADIUS_FL_CM, ROBOT_TURN_RADIUS_FR_CM,
    ROBOT_TURN_RADIUS_BL_CM, ROBOT_TURN_RADIUS_BR_CM,
    ROBOT_TURN_RADIUS_MAX_CM,
    ROBOT_SPEED_CM_S,
    ROBOT_WHEELBASE_CM,
    SCALE_FW, SCALE_BW, SCALE_FL, SCALE_FR, SCALE_BL, SCALE_BR,
    OFFSET_FW, OFFSET_BW, OFFSET_FL, OFFSET_FR, OFFSET_BL, OFFSET_BR
)
from python_tsp.exact import solve_tsp_dynamic_programming
from reeds_shepp import (
    get_optimal_path_length, get_optimal_path_segments
)

from hybrid_astar import (
    hybrid_astar_search, path_to_commands, build_obstacle_list,
    OBSTACLE_RADIUS_CM
)


# =============================================================================
# THETA NORMALIZATION
# =============================================================================

def _norm_theta(t):
    """Normalize angle to [-pi, pi]."""
    return (t + math.pi) % (2 * math.pi) - math.pi


# =============================================================================
# RS PATH COLLISION CHECKING — EULER-INTEGRATED (matches simulator exactly)
# =============================================================================

def _euler_simulate_segments(start, segments):
    """Euler-integrate RS segments using actual per-arc radii.
    
    This produces the EXACT trajectory the robot will follow when these
    segments are converted to FL/FR/BL/BR/FW/BW commands and executed.
    
    Returns: (points, end_pose) where points = [(x,y), ...] and 
             end_pose = (x, y, theta) — the actual endpoint.
    """
    L = ROBOT_WHEELBASE_CM
    speed = ROBOT_SPEED_CM_S
    dt = 1.0 / 60.0

    # Steering angle and velocity for each segment type
    _SEG_PARAMS = {
        ('L', 'F'): ( speed, +math.atan(L / ROBOT_TURN_RADIUS_FL_CM)),  # FL
        ('L', 'B'): (-speed, +math.atan(L / ROBOT_TURN_RADIUS_BR_CM)),  # BR
        ('R', 'F'): ( speed, -math.atan(L / ROBOT_TURN_RADIUS_FR_CM)),  # FR
        ('R', 'B'): (-speed, -math.atan(L / ROBOT_TURN_RADIUS_BL_CM)),  # BL
        ('S', 'F'): ( speed, 0.0),                                       # FW
        ('S', 'B'): (-speed, 0.0),                                       # BW
    }

    x, y, theta = start
    points = [(x, y)]
    SAMPLE_EVERY_CM = 3.0  # Subsample for collision checking (obstacles are 33cm radius)

    for seg_type, length_cm, gear in segments:
        if length_cm < 0.5:
            continue

        dist_cm = round(length_cm)
        v, delta = _SEG_PARAMS.get((seg_type, gear), (speed, 0.0))
        omega = (v / L) * math.tan(delta) if abs(delta) > 1e-9 else 0.0

        traveled = 0.0
        since_last_sample = 0.0
        while traveled < dist_cm:
            x += v * math.cos(theta) * dt
            y += v * math.sin(theta) * dt
            theta += omega * dt
            step = abs(v * dt)
            traveled += step
            since_last_sample += step
            if since_last_sample >= SAMPLE_EVERY_CM:
                points.append((x, y))
                since_last_sample = 0.0

        points.append((x, y))  # always include segment endpoint

    return points, (x, y, theta)


def check_rs_path_collision(start, end, radius, obstacles_expanded,
                            skip_start_idx=None, skip_goal_idx=None):
    """Check if a Reeds-Shepp path is collision-free.
    
    Because the robot has ASYMMETRIC turning radii (FL≠FR≠BL≠BR), a
    single-radius RS plan will drift when Euler-simulated with the real
    per-arc radii.  We try several candidate planning radii and pick
    the collision-free path whose Euler-simulated endpoint lands closest
    to the intended goal.  This typically reduces heading error from
    ~5-7° down to <1° at near-zero extra cost (RS is analytic).
    
    Returns:
        (is_clear, segments, total_length, actual_end_pose) or (False, None, inf, None)
    """
    ARENA = 200.0
    CLEARANCE = 15.0
    CAPTURE_CLEARANCE = 28.0

    # Candidate planning radii: the four real radii plus their average.
    # Different radii produce different RS path shapes; the one whose
    # shape best matches the mix of L/R arcs in the path will have the
    # smallest Euler drift.
    _CANDIDATE_RADII = sorted(set([
        ROBOT_TURN_RADIUS_FL_CM,
        ROBOT_TURN_RADIUS_FR_CM,
        ROBOT_TURN_RADIUS_BL_CM,
        ROBOT_TURN_RADIUS_BR_CM,
        (ROBOT_TURN_RADIUS_FL_CM + ROBOT_TURN_RADIUS_FR_CM +
         ROBOT_TURN_RADIUS_BL_CM + ROBOT_TURN_RADIUS_BR_CM) / 4.0,
        ROBOT_TURN_RADIUS_MAX_CM,
    ]))

    best_result = None
    best_score = float('inf')   # lower = better (position error + heading penalty)

    for try_radius in _CANDIDATE_RADII:
        result = get_optimal_path_segments(start, end, try_radius)
        if result is None:
            continue

        segments, total_length = result

        # Euler-integrate with the ACTUAL per-arc radii
        points, actual_end = _euler_simulate_segments(start, segments)

        # Check collisions along the Euler trajectory
        collision = False
        for px, py in points:
            if px < CLEARANCE or px > ARENA - CLEARANCE:
                collision = True
                break
            if py < CLEARANCE or py > ARENA - CLEARANCE:
                collision = True
                break
            for i, (ox, oy, orad) in enumerate(obstacles_expanded):
                r = CAPTURE_CLEARANCE if i in (skip_start_idx, skip_goal_idx) else orad
                if (px - ox)**2 + (py - oy)**2 < r**2:
                    collision = True
                    break
            if collision:
                break

        if collision:
            continue

        # Score: how close does the Euler endpoint land to the goal?
        pos_err = math.sqrt((actual_end[0] - end[0])**2 +
                            (actual_end[1] - end[1])**2)
        hdg_err = abs(((actual_end[2] - end[2] + math.pi) % (2 * math.pi)) - math.pi)
        # Heading error is weighted heavily because it compounds through
        # subsequent straight segments (~1cm lateral per degree per 50cm)
        score = pos_err + 30.0 * hdg_err

        if score < best_score:
            best_score = score
            best_result = (True, segments, total_length, actual_end)

    if best_result is not None:
        return best_result

    return False, None, float('inf'), None


def rs_segments_to_commands(segments):
    _CMD_MAP = {
        ('S', 'F'): ('FW', SCALE_FW, OFFSET_FW),
        ('S', 'B'): ('BW', SCALE_BW, OFFSET_BW),
        ('L', 'F'): ('FL', SCALE_FL, OFFSET_FL),
        ('L', 'B'): ('BR', SCALE_BR, OFFSET_BR),
        ('R', 'F'): ('FR', SCALE_FR, OFFSET_FR),
        ('R', 'B'): ('BL', SCALE_BL, OFFSET_BL),
    }

    commands = []
    for seg_type, length_cm, gear in segments:
        entry = _CMD_MAP.get((seg_type, gear))
        if entry is None:
            continue
        prefix, scale, offset = entry
        center_arc_mm = length_cm * 10
        dist_mm = max(10, round(center_arc_mm * scale + offset))
        commands.append(f"{prefix}{dist_mm}")
    return commands


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
        self.obstacle_centers = []
        self._rs_cache = {}

    def add_obstacle(self, x, y, direction: Direction, obstacle_id: int):
        obstacle = Obstacle(x, y, direction, obstacle_id)
        self.grid.add_obstacle(obstacle)
        self.obstacle_centers.append((x * 10 + 5, y * 10 + 5, obstacle_id))

    def reset_obstacles(self):
        self.grid.reset_obstacles()
        self.obstacle_centers = []

    # =================================================================
    # COORDINATE CONVERSION
    # =================================================================

    @staticmethod
    def _state_to_pose(state: CellState):
        """Convert CellState to (x_cm, y_cm, theta_rad)."""
        if hasattr(state, 'theta_rad') and state.theta_rad is not None:
            return (state.x * 10, state.y * 10, state.theta_rad)
        else:
            return (state.x * 10 + 5, state.y * 10 + 5,
                    Direction.to_angle_rad(state.direction))

    @staticmethod
    def _state_to_xy(state: CellState):
        if hasattr(state, 'theta_rad') and state.theta_rad is not None:
            return (state.x * 10, state.y * 10)
        else:
            return (state.x * 10 + 5, state.y * 10 + 5)

    # =================================================================
    # REEDS-SHEPP TSP COST MATRIX
    # =================================================================

    _AVG_RADIUS = (ROBOT_TURN_RADIUS_FL_CM + ROBOT_TURN_RADIUS_FR_CM +
                   ROBOT_TURN_RADIUS_BL_CM + ROBOT_TURN_RADIUS_BR_CM) / 4.0

    def _rs_travel_cost(self, pose_a, pose_b):
        """Reeds-Shepp path length between two poses (cached).

        Uses the average turning radius for cost estimation, since the
        actual path will use a mix of FL/FR/BL/BR arcs with different
        radii.  This gives more accurate TSP ordering than using MAX.
        """
        # Quantize to 1mm / 0.1deg to collapse near-identical poses
        key = (round(pose_a[0], 1), round(pose_a[1], 1), round(pose_a[2], 3),
               round(pose_b[0], 1), round(pose_b[1], 1), round(pose_b[2], 3))
        cached = self._rs_cache.get(key)
        if cached is not None:
            return cached
        cost = get_optimal_path_length(pose_a, pose_b, self._AVG_RADIUS)
        self._rs_cache[key] = cost
        return cost

    # =================================================================
    # VISIT ORDER (TSP)
    # =================================================================

    @staticmethod
    def _get_visit_options(n):
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
        """Find optimal visit order using Reeds-Shepp TSP costs."""
        all_view_positions = self.grid.get_view_obstacle_positions(retrying)
        if not all_view_positions:
            return [], 1e9

        best_distance = 1e9
        best_path = []

        start_state = self.robot.get_start_state()
        start_pose = self._state_to_pose(start_state)

        for op in self._get_visit_options(len(all_view_positions)):
            items_states = [start_state]
            items_poses = [start_pose]
            cur_view_positions = []

            for idx in range(len(all_view_positions)):
                if op[idx] == '1':
                    cur_view_positions.append(all_view_positions[idx])
                    for vp in all_view_positions[idx]:
                        items_states.append(vp)
                        items_poses.append(self._state_to_pose(vp))

            if not cur_view_positions:
                continue

            # Pairwise RS distances (heading-aware)
            n_total = len(items_poses)
            full_cost = np.zeros((n_total, n_total))
            for s in range(n_total):
                for e in range(n_total):
                    if s == e:
                        continue
                    full_cost[s][e] = self._rs_travel_cost(
                        items_poses[s], items_poses[e])

            # Try candidate combinations
            combinations = []
            self._generate_combination(
                cur_view_positions, 0, [], combinations, [ITERATIONS])

            for c in combinations:
                visited_indices = [0]
                cur_index = 1
                fixed_cost = 0

                for obs_idx, view_pos_list in enumerate(cur_view_positions):
                    selected = cur_index + c[obs_idx]
                    visited_indices.append(selected)
                    fixed_cost += view_pos_list[c[obs_idx]].penalty
                    cur_index += len(view_pos_list)

                n_nodes = len(visited_indices)
                cost_np = np.zeros((n_nodes, n_nodes))
                for s in range(n_nodes):
                    for e in range(n_nodes):
                        if s != e:
                            cost_np[s][e] = full_cost[
                                visited_indices[s]][visited_indices[e]]

                cost_np[:, 0] = 0  # open-path TSP

                _permutation, _distance = solve_tsp_dynamic_programming(cost_np)

                if _distance + fixed_cost >= best_distance:
                    continue

                best_distance = _distance + fixed_cost
                best_path = [items_states[0]]
                for i in range(1, len(_permutation)):
                    if _permutation[i] == 0:
                        continue
                    best_path.append(
                        items_states[visited_indices[_permutation[i]]])

            if best_path:
                break

        return best_path, best_distance

    # =================================================================
    # MAIN PIPELINE
    # =================================================================

    def plan_full_route(self, retrying=False, obstacles_data=None):
        """Full pipeline: RS-based TSP + RS/A* transit + SNAP.
        
        For each leg:
          1. Try direct Reeds-Shepp path (collision-free check)
             -> If clear: use it (fastest, no stopping)
          2. If blocked: Hybrid A* heading-constrained
          3. If that fails: Hybrid A* position-only (fallback)
          4. SNAP (robot already facing obstacle — no spin needed)
        
        Returns:
            (commands, waypoint_path, distance)
        """
        t0 = time.time()

        # Phase 1: TSP ordering
        waypoint_path, distance = self.get_optimal_order_dp(retrying)
        t_tsp = time.time() - t0
        print(f"Phase 1 (RS-based TSP): {t_tsp:.2f}s, distance={distance:.0f}cm")

        if not waypoint_path:
            print("No valid path found!")
            return ["FIN"], [], 1e9

        for s in waypoint_path:
            if s.screenshot_id != -1:
                print(f"  Visit ob{s.screenshot_id} at "
                      f"({s.x:.1f},{s.y:.1f}) {Direction(s.direction).name}")

        # Phase 2: Plan each leg
        t1 = time.time()
        obs_expanded = build_obstacle_list(obstacles_data) if obstacles_data else []
        ob_id_to_idx = {ob['id']: i for i, ob in enumerate(obstacles_data)} if obstacles_data else {}

        commands = []
        current_pose = self._state_to_pose(waypoint_path[0])

        for i in range(len(waypoint_path) - 1):
            wp_from = waypoint_path[i]
            wp_to = waypoint_path[i + 1]

            start_pose = (current_pose[0], current_pose[1], _norm_theta(current_pose[2]))
            goal_pose = self._state_to_pose(wp_to)

            # Obstacle indices for radius relaxation
            start_ob_idx = None
            goal_ob_idx = None
            if wp_from.screenshot_id != -1 and wp_from.screenshot_id in ob_id_to_idx:
                start_ob_idx = ob_id_to_idx[wp_from.screenshot_id]
            if wp_to.screenshot_id != -1 and wp_to.screenshot_id in ob_id_to_idx:
                goal_ob_idx = ob_id_to_idx[wp_to.screenshot_id]

            leg_commands = []
            leg_ok = False
            goal_xy = (goal_pose[0], goal_pose[1], goal_pose[2])

            # Find ALL obstacles whose virtual zone overlaps the start or
            # goal position. We must relax those radii so A* can find
            # a path in/out of capture positions.
            nearby_start = set()
            nearby_goal = set()
            for oi, (oox, ooy, oor) in enumerate(obs_expanded):
                ds = math.sqrt((start_pose[0]-oox)**2 + (start_pose[1]-ooy)**2)
                dg = math.sqrt((goal_pose[0]-oox)**2 + (goal_pose[1]-ooy)**2)
                if ds < oor + 2:
                    nearby_start.add(oi)
                if dg < oor + 2:
                    nearby_goal.add(oi)
            if start_ob_idx is not None:
                nearby_start.add(start_ob_idx)
            if goal_ob_idx is not None:
                nearby_goal.add(goal_ob_idx)
            skip_indices = nearby_start | nearby_goal

            # --- ATTEMPT 1: Direct Reeds-Shepp path ---
            is_clear, segments, rs_len, actual_end = check_rs_path_collision(
                start_pose, goal_pose, ROBOT_TURN_RADIUS_MAX_CM,
                obs_expanded,
                skip_start_idx=start_ob_idx,
                skip_goal_idx=goal_ob_idx)

            if is_clear and segments:
                leg_commands = rs_segments_to_commands(segments)
                # Use ACTUAL endpoint from Euler simulation (not planned goal)
                # to avoid cascading position errors with asymmetric radii.
                current_pose = (actual_end[0], actual_end[1], _norm_theta(actual_end[2]))
                leg_ok = True
                print(f"  Leg {i}: RS direct, {rs_len:.0f}cm, "
                      f"{len(segments)} segments")

            # --- ATTEMPT 2: A* heading-constrained (budget-limited) ---
            if not leg_ok:
                path, iters = hybrid_astar_search(
                    start_pose, goal_xy, obs_expanded,
                    goal_tolerance_cm=8.0,
                    heading_tolerance_rad=0.35,  # ~20 deg (within camera half-FOV of 31 deg)
                    position_only=False,
                    skip_obstacle_indices=skip_indices,
                    max_iterations=50000)

                if path:
                    leg_commands = path_to_commands(path)
                    end = path[-1]
                    current_pose = (end[0], end[1], _norm_theta(end[2]))
                    leg_ok = True
                    print(f"  Leg {i}: A* heading, {iters} iters, {len(path)} steps")

            # --- ATTEMPT 3: A* position-only ---
            if not leg_ok:
                path, iters = hybrid_astar_search(
                    start_pose, goal_xy, obs_expanded,
                    goal_tolerance_cm=5.0,
                    position_only=True,
                    skip_obstacle_indices=skip_indices)

                if path:
                    leg_commands = path_to_commands(path)
                    end = path[-1]
                    current_pose = (end[0], end[1], _norm_theta(end[2]))
                    leg_ok = True
                    print(f"  Leg {i}: A* pos-only, {iters} iters, {len(path)} steps")

            # --- ATTEMPT 4: A* relaxed tolerance ---
            if not leg_ok:
                path, iters = hybrid_astar_search(
                    start_pose, goal_xy, obs_expanded,
                    goal_tolerance_cm=15.0,
                    position_only=True,
                    skip_obstacle_indices=skip_indices)

                if path:
                    leg_commands = path_to_commands(path)
                    end = path[-1]
                    current_pose = (end[0], end[1], _norm_theta(end[2]))
                    leg_ok = True
                    print(f"  Leg {i}: A* relaxed, {iters} iters")

            if not leg_ok:
                print(f"  Leg {i}: SKIPPED (no path found)")
                continue

            commands.extend(leg_commands)

            # --- SNAP: capture image (no spin needed — RS/A* delivered
            #     the correct heading) ---
            if wp_to.screenshot_id != -1:
                commands.append(f"SNAP{wp_to.screenshot_id}")

        commands.append("FIN")
        commands = self._compress_commands(commands)
        commands = self._insert_opportunistic_snaps(commands, obstacles_data)

        t_plan = time.time() - t1
        print(f"Phase 2 (RS/A*): {t_plan:.2f}s")
        print(f"Total: {t_tsp + t_plan:.2f}s, {len(commands)} commands")

        return commands, waypoint_path, distance

    @staticmethod
    def _compress_commands(commands):
        """Merge consecutive FW/BW commands (distances in mm)."""
        if not commands:
            return commands
        compressed = [commands[0]]
        for i in range(1, len(commands)):
            cmd = commands[i]
            prev = compressed[-1]
            if (cmd.startswith("FW") and prev.startswith("FW")
                    and not cmd.startswith("FIN")):
                try:
                    pv, cv = int(prev[2:]), int(cmd[2:])
                    if pv + cv <= 900:
                        compressed[-1] = f"FW{pv + cv}"
                        continue
                except ValueError:
                    pass
            elif cmd.startswith("BW") and prev.startswith("BW"):
                try:
                    pv, cv = int(prev[2:]), int(cmd[2:])
                    if pv + cv <= 900:
                        compressed[-1] = f"BW{pv + cv}"
                        continue
                except ValueError:
                    pass
            compressed.append(cmd)
        return compressed

    @staticmethod
    def _insert_opportunistic_snaps(commands, obstacles_data):
        """Insert one extra SNAP before and one after each primary SNAP.

        For each SNAP{id}, inserts a duplicate SNAP:
          - after the move command immediately before the SNAP
          - after the move command immediately after the SNAP
        """
        if not obstacles_data:
            return commands

        # Find primary SNAP indices
        snap_info = []
        for i, cmd in enumerate(commands):
            if cmd.startswith("SNAP"):
                try:
                    ob_id = int(cmd[4:])
                    snap_info.append((i, ob_id))
                except ValueError:
                    pass

        # Collect insertion points
        insert_list = []

        for snap_idx, ob_id in snap_info:
            snap_cmd = f"SNAP{ob_id}"

            # 1 command before: after the command two steps before the SNAP
            j = snap_idx - 2
            if j >= 0 and not commands[j].startswith("SNAP") and not commands[j].startswith("FIN"):
                insert_list.append((j, snap_cmd))

            # 1 command after the SNAP
            j = snap_idx + 1
            if j < len(commands) and not commands[j].startswith("SNAP") and not commands[j].startswith("FIN"):
                insert_list.append((j, snap_cmd))

        if not insert_list:
            return commands

        # Insert from end to preserve indices
        insert_list.sort(key=lambda t: t[0], reverse=True)
        result = list(commands)
        for idx, snap_cmd in insert_list:
            result.insert(idx + 1, snap_cmd)

        return result