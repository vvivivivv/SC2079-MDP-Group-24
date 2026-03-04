"""
MazeSolver: Reeds-Shepp transit + pivot-corrected CW aim-spins.

ARCHITECTURE (optimized for physical speed):
  1. Generate capture candidates for each obstacle (Entity.py)
  2. Build TSP cost matrix using REEDS-SHEPP path length (heading-aware)
     - Includes aim-spin time cost and stop/start penalty
  3. Solve TSP for optimal visit order
  4. Plan each leg:
     a. Try direct Reeds-Shepp path (fastest — no stopping)
     b. If obstacles block it, fall back to Hybrid A* (RS heuristic)
     c. At destination: small CW aim-spin (pivot-corrected) + SNAP

  Why this is faster than spot-turn + straight:
    - Heading changes happen WHILE DRIVING (no stopping)
    - RS paths are provably shortest for car-like robots
    - The TSP sees real driving costs, not Euclidean approximations
    - Aim-spins are small (< 30 deg) because RS delivers close to right heading

  Pivot-on-back-left-wheel:
    CW spins pivot on the back-left wheel, not the center.
    This shifts the robot position. We compute the displacement and
    plan accordingly so the camera ends up aimed correctly.

  Command set:
    FW/BW/FL/FR/BL/BR  — Ackermann drive (from RS or A*)
    CW{deg}             — Spot turn clockwise (pivot on back-left wheel)
    SNAP{id}            — Capture image
    FIN                 — Mission complete
"""

import math
import math
import time
from typing import List, Tuple
import numpy as np

from entities.Robot import Robot
from entities.Entity import Obstacle, CellState, Grid
from consts import (
    Direction, ITERATIONS,
    ROBOT_TURN_RADIUS_CM,
    ROBOT_TURN_RADIUS_LEFT_CM, ROBOT_TURN_RADIUS_RIGHT_CM,
    ROBOT_TURN_RADIUS_MAX_CM, ROBOT_TURN_RADIUS_MIN_CM,
    ROBOT_SPEED_CM_S,
    PIVOT_OFFSET_X, PIVOT_OFFSET_Y,
    SPIN_SPEED_DEG_S, STOP_START_PENALTY_CM,
    ROBOT_WHEELBASE_CM
)
from python_tsp.exact import solve_tsp_dynamic_programming
from reeds_shepp import (
    get_optimal_path_length, get_optimal_path_segments,
    sample_path
)
# Max heading error (degrees) at which we skip CW spin entirely.
# Camera HFOV = 62.2 deg => face visible up to ~31 deg off-center.
# Being conservative: skip spin if error < 25 deg. This avoids
# converting tiny CCW errors into massive CW rotations (5 deg CCW
# would become 355 deg CW = 3.5cm drift for no benefit).
SPIN_SKIP_THRESHOLD_DEG = 25.0

from hybrid_astar import (
    hybrid_astar_search, path_to_commands, build_obstacle_list,
    TURN_RADIUS_CM, OBSTACLE_RADIUS_CM
)


# =============================================================================
# THETA NORMALIZATION
# =============================================================================

def _norm_theta(t):
    """Normalize angle to [-pi, pi]."""
    return (t + math.pi) % (2 * math.pi) - math.pi


# =============================================================================
# PIVOT GEOMETRY — CW spin on back-left wheel
# =============================================================================

def compute_pivot_cw(cx, cy, theta_rad, spin_deg):
    """Compute new pose after a CW spin pivoting on the back-left wheel.
    
    The back-left wheel stays fixed. The robot center traces an arc
    around it. Both position and heading change.
    
    Args:
        cx, cy: robot center position (cm)
        theta_rad: robot heading (radians, math convention)
        spin_deg: degrees to spin clockwise (positive = CW)
    
    Returns:
        (new_cx, new_cy, new_theta_rad)
    """
    if abs(spin_deg) < 0.5:
        return cx, cy, theta_rad

    alpha = math.radians(spin_deg)  # CW rotation angle

    # Back-left wheel position in world frame
    cos_t = math.cos(theta_rad)
    sin_t = math.sin(theta_rad)
    pivot_x = cx + PIVOT_OFFSET_X * cos_t - PIVOT_OFFSET_Y * sin_t
    pivot_y = cy + PIVOT_OFFSET_X * sin_t + PIVOT_OFFSET_Y * cos_t

    # Rotate center around pivot (CW = negative mathematical rotation)
    dx = cx - pivot_x
    dy = cy - pivot_y
    new_cx = pivot_x + dx * math.cos(alpha) + dy * math.sin(alpha)
    new_cy = pivot_y - dx * math.sin(alpha) + dy * math.cos(alpha)
    new_theta = theta_rad - alpha

    return new_cx, new_cy, new_theta


def compute_cw_spin_to_target(current_theta_rad, target_theta_rad):
    """Compute CW spin angle to reach target heading.
    
    Only clockwise spins allowed (robot is unreliable CCW).
    A left turn of X deg becomes CW (360-X) deg.
    
    Returns:
        spin_deg (0-360, always positive = CW), or 0 if close enough
    """
    diff = target_theta_rad - current_theta_rad
    diff = (diff + math.pi) % (2 * math.pi) - math.pi  # [-pi, pi]
    diff_deg = math.degrees(diff)

    if abs(diff_deg) < 3.0:
        return 0.0  # close enough

    if diff_deg > 0:
        # Need CCW turn -> convert to CW
        return 360.0 - abs(round(diff_deg))
    else:
        # Already CW direction
        return abs(round(diff_deg))


def compute_aim_spin_full(cx, cy, theta_rad, target_x, target_y):
    """Compute CW spin + pivot displacement to aim camera at target point.
    
    Returns:
        (spin_deg, new_cx, new_cy, new_theta)
    """
    target_theta = math.atan2(target_y - cy, target_x - cx)
    spin_deg = compute_cw_spin_to_target(theta_rad, target_theta)

    if spin_deg < 0.5:
        return 0.0, cx, cy, theta_rad

    new_cx, new_cy, new_theta = compute_pivot_cw(cx, cy, theta_rad, spin_deg)
    return spin_deg, new_cx, new_cy, new_theta


# =============================================================================
# RS PATH COLLISION CHECKING
# =============================================================================

def check_rs_path_collision(start, end, radius, obstacles_expanded,
                            skip_start_idx=None, skip_goal_idx=None):
    """Check if a Reeds-Shepp path is collision-free.
    
    Samples the path and checks each point against obstacles and walls.
    
    Returns:
        (is_clear, segments, total_length) or (False, None, inf)
    """
    ARENA = 200.0
    CLEARANCE = 15.0
    CAPTURE_CLEARANCE = 22.0

    result = get_optimal_path_segments(start, end, radius)
    if result is None:
        return False, None, float('inf')

    segments, total_length = result
    points = sample_path(start, radius, segments=segments)

    for px, py in points:
        # Arena bounds
        if px < CLEARANCE or px > ARENA - CLEARANCE:
            return False, None, float('inf')
        if py < CLEARANCE or py > ARENA - CLEARANCE:
            return False, None, float('inf')

        # Obstacle collision
        for i, (ox, oy, orad) in enumerate(obstacles_expanded):
            r = CAPTURE_CLEARANCE if i in (skip_start_idx, skip_goal_idx) else orad
            if (px - ox)**2 + (py - oy)**2 < r**2:
                return False, None, float('inf')

    return True, segments, total_length


def rs_segments_to_commands(segments):
    """Convert Reeds-Shepp segments to robot commands.
    
    Segment format: (type, length_cm, gear)
      type: 'L', 'R', 'S'
      gear: 'F' (forward), 'B' (backward)
    
    Maps to: FL, FR, FW, BW, BL, BR commands.
    """
    commands = []
    for seg_type, length_cm, gear in segments:
        dist = max(1, round(length_cm))
        if seg_type == 'S':
            cmd = f"FW{dist}" if gear == 'F' else f"BW{dist}"
        elif seg_type == 'L':
            cmd = f"FL{dist}" if gear == 'F' else f"BL{dist}"
        elif seg_type == 'R':
            cmd = f"FR{dist}" if gear == 'F' else f"BR{dist}"
        else:
            continue
        commands.append(cmd)
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

    def _rs_travel_cost(self, pose_a, pose_b):
        """Reeds-Shepp path length + estimated spin cost.
        
        RS path length gives the transit cost. We add an estimated
        spin penalty because when A* is used (RS blocked), the robot
        may arrive at a heading that differs from pose_b's heading.
        
        The penalty is based on how "natural" the approach direction
        is: if the straight-line approach from A to B already points
        near B's heading, A* will arrive close to the right heading
        and need minimal spinning. If not, large CW spins may occur.
        """
        rs_len = get_optimal_path_length(pose_a, pose_b, ROBOT_TURN_RADIUS_MAX_CM)
        
        # Estimate spin cost: what heading would a straight-line
        # approach from A to B produce?
        dx = pose_b[0] - pose_a[0]
        dy = pose_b[1] - pose_a[1]
        if abs(dx) + abs(dy) < 1.0:
            natural_heading = pose_a[2]
        else:
            natural_heading = math.atan2(dy, dx)
        
        # How far is the natural approach heading from the desired heading?
        target_heading = pose_b[2]
        h_err = abs(((natural_heading - target_heading) + math.pi) % (2 * math.pi) - math.pi)
        h_err_deg = math.degrees(h_err)
        
        # Spin penalty (in cm equivalent):
        # - Within SPIN_SKIP_THRESHOLD: free (we skip the spin)
        # - Small CW spin (<90 deg): moderate cost
        # - Large CW spin (>180 deg, i.e. small CCW forced CW): expensive
        if h_err_deg <= SPIN_SKIP_THRESHOLD_DEG:
            spin_cost = 0
        elif h_err_deg <= 90:
            spin_cost = h_err_deg * 0.3  # ~27cm max
        else:
            # Forces a near-full CW rotation = big drift risk
            spin_cost = h_err_deg * 0.6  # up to ~108cm penalty
        
        return rs_len + STOP_START_PENALTY_CM + spin_cost

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
        """Full pipeline: RS-based TSP + RS/A* transit + pivot aim-spin + SNAP.
        
        For each leg:
          1. Try direct Reeds-Shepp path (collision-free check)
             -> If clear: use it (fastest, no stopping)
          2. If blocked: Hybrid A* with RS heuristic, heading-constrained
          3. If that fails: Hybrid A* position-only (fallback)
          4. Small CW aim-spin (pivot-corrected) to face obstacle
          5. SNAP
        
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
        obstacles_dict = {ob['id']: ob for ob in obstacles_data} if obstacles_data else {}
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
            # goal position. After a large CW pivot spin, the robot center
            # can drift into a nearby obstacle's zone — we must relax those
            # radii so A* can still find a path out.
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
            is_clear, segments, rs_len = check_rs_path_collision(
                start_pose, goal_pose, ROBOT_TURN_RADIUS_MAX_CM,
                obs_expanded,
                skip_start_idx=start_ob_idx,
                skip_goal_idx=goal_ob_idx)

            if is_clear and segments:
                leg_commands = rs_segments_to_commands(segments)
                current_pose = (goal_pose[0], goal_pose[1], _norm_theta(goal_pose[2]))
                leg_ok = True
                print(f"  Leg {i}: RS direct, {rs_len:.0f}cm, "
                      f"{len(segments)} segments")

            # --- ATTEMPT 2: A* heading-constrained (Euclidean+heading heuristic) ---
            if not leg_ok:
                path, iters = hybrid_astar_search(
                    start_pose, goal_xy, obs_expanded,
                    goal_tolerance_cm=8.0,
                    heading_tolerance_rad=0.35,  # ~20 deg (within spin-skip threshold)
                    position_only=False,
                    skip_obstacle_indices=skip_indices)

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

            # --- AIM-SPIN: pivot-corrected CW to face obstacle ---
            if wp_to.screenshot_id != -1 and obstacles_dict:
                ob = obstacles_dict[wp_to.screenshot_id]

                # Obstacle face center
                face_offsets = {
                    0: (0, 5), 2: (5, 0), 4: (0, -5), 6: (-5, 0)}
                fx, fy = face_offsets.get(ob['d'], (0, 0))
                face_x = ob['x'] * 10 + 5 + fx
                face_y = ob['y'] * 10 + 5 + fy

                rx, ry, rtheta = current_pose
                rtheta = _norm_theta(rtheta)

                # Compute the SIGNED heading error to the face.
                # If it's small enough, the face is in the camera FOV
                # and we can skip the spin entirely (avoiding drift).
                angle_to_face = math.atan2(face_y - ry, face_x - rx)
                signed_err = angle_to_face - rtheta
                signed_err = (signed_err + math.pi) % (2 * math.pi) - math.pi
                signed_err_deg = math.degrees(signed_err)

                if abs(signed_err_deg) <= SPIN_SKIP_THRESHOLD_DEG:
                    # Face is within camera FOV — no spin needed
                    spin_deg = 0
                    new_rx, new_ry, new_theta = rx, ry, rtheta
                else:
                    spin_deg, new_rx, new_ry, new_theta = compute_aim_spin_full(
                        rx, ry, rtheta, face_x, face_y)

                if spin_deg > 0.5:
                    # PRE-CHECK: will the spin push us into an obstacle?
                    # If so, drive forward/backward a bit to create clearance.
                    spin_ok = True
                    for oox, ooy, oor in obs_expanded:
                        d = math.sqrt((new_rx-oox)**2 + (new_ry-ooy)**2)
                        if d < oor:
                            spin_ok = False
                            break

                    if not spin_ok:
                        # Try small forward nudge (10cm) to escape
                        for nudge_dist, nudge_cmd in [(10, "FW10"), (-10, "BW10")]:
                            test_x = rx + nudge_dist * math.cos(rtheta)
                            test_y = ry + nudge_dist * math.sin(rtheta)
                            # Check nudge destination is valid
                            nudge_valid = (15 <= test_x <= 185 and 15 <= test_y <= 185)
                            if nudge_valid:
                                for oox, ooy, oor in obs_expanded:
                                    if math.sqrt((test_x-oox)**2+(test_y-ooy)**2) < 22:
                                        nudge_valid = False
                                        break
                            if nudge_valid:
                                # Re-compute spin from nudged position
                                s2, nx2, ny2, nt2 = compute_aim_spin_full(
                                    test_x, test_y, rtheta, face_x, face_y)
                                # Check post-spin collision
                                nudge_spin_ok = True
                                for oox, ooy, oor in obs_expanded:
                                    if math.sqrt((nx2-oox)**2+(ny2-ooy)**2) < oor:
                                        nudge_spin_ok = False
                                        break
                                if nudge_spin_ok:
                                    commands.append(nudge_cmd)
                                    spin_deg, new_rx, new_ry, new_theta = s2, nx2, ny2, nt2
                                    print(f"    Nudge {nudge_cmd} to avoid spin collision")
                                    break

                    commands.append(f"CW{int(round(spin_deg))}")
                    current_pose = (new_rx, new_ry, _norm_theta(new_theta))

                # --- SNAP ---
                commands.append(f"SNAP{wp_to.screenshot_id}")

        commands.append("FIN")
        commands = self._compress_commands(commands)

        t_plan = time.time() - t1
        print(f"Phase 2 (RS/A* + spin): {t_plan:.2f}s")
        print(f"Total: {t_tsp + t_plan:.2f}s, {len(commands)} commands")

        return commands, waypoint_path, distance

    @staticmethod
    def _compress_commands(commands):
        """Merge consecutive FW/BW commands."""
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
                    if pv + cv <= 90:
                        compressed[-1] = f"FW{pv + cv}"
                        continue
                except ValueError:
                    pass
            elif cmd.startswith("BW") and prev.startswith("BW"):
                try:
                    pv, cv = int(prev[2:]), int(cmd[2:])
                    if pv + cv <= 90:
                        compressed[-1] = f"BW{pv + cv}"
                        continue
                except ValueError:
                    pass
            compressed.append(cmd)
        return compressed