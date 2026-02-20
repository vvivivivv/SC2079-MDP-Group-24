"""
MazeSolver: Obstacle visit ordering + smooth path planning + SPOT TURNS.

SPOT-TURN ARCHITECTURE:
  The robot drives like an Ackermann car between waypoints (S, L, R, B, BL, BR),
  but can spin on the spot at each waypoint to face the obstacle before capture.

  This decouples TRANSIT (get to position) from AIM (face the obstacle):
  1. Get candidate capture positions for each obstacle (Entity.py)
  2. Build TSP cost matrix using EUCLIDEAN DISTANCE (heading-free)
  3. Solve TSP for optimal visit order
  4. Plan each leg with Hybrid A* (position-only goal, Euclidean heuristic)
  5. At each waypoint: compute spin angle, insert TL/TR, then SNAP

  Command set:
    FW/BW/FL/FR/BL/BR  — Ackermann drive (unchanged)
    TL{deg}             — Spot turn left by {deg} degrees (NEW)
    TR{deg}             — Spot turn right by {deg} degrees (NEW)
    SNAP{id}            — Capture image (simplified, no L/C/R needed)
    FIN                 — Mission complete
"""

import math
import time
from typing import List, Tuple
import numpy as np

from entities.Robot import Robot
from entities.Entity import Obstacle, CellState, Grid
from consts import Direction, ITERATIONS
from python_tsp.exact import solve_tsp_dynamic_programming
from hybrid_astar import (
    hybrid_astar_search, path_to_commands, build_obstacle_list,
    TURN_RADIUS_CM, OBSTACLE_RADIUS_CM
)


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
        """Convert CellState to (x_cm, y_cm) — position only, for A* goal."""
        if hasattr(state, 'theta_rad') and state.theta_rad is not None:
            return (state.x * 10, state.y * 10)
        else:
            return (state.x * 10 + 5, state.y * 10 + 5)

    # =================================================================
    # EUCLIDEAN DISTANCE (replaces Reeds-Shepp for TSP cost matrix)
    # =================================================================

    def _euclidean_distance_obstacle_aware(self, pos_a, pos_b, skip_ids=None):
        """Euclidean distance with penalty if straight line crosses obstacles.
        
        Since heading doesn't matter (spot-turn), the cost between two
        positions is approximately the Euclidean distance. We add a penalty
        if obstacles block the direct path to encourage the TSP to avoid
        orderings that require long detours.
        """
        ax, ay = pos_a[0], pos_a[1]
        bx, by = pos_b[0], pos_b[1]
        dx, dy = bx - ax, by - ay
        base_dist = math.sqrt(dx*dx + dy*dy)
        
        if not self.obstacle_centers:
            return base_dist
        
        skip = skip_ids or set()
        line_len = base_dist
        
        penalty = 0
        for ox, oy, oid in self.obstacle_centers:
            if oid in skip:
                continue
            # Point-to-line-segment distance
            if line_len < 1e-6:
                d = math.sqrt((ox-ax)**2 + (oy-ay)**2)
            else:
                t = max(0, min(1, ((ox-ax)*dx + (oy-ay)*dy) / (line_len*line_len)))
                px, py = ax + t*dx, ay + t*dy
                d = math.sqrt((ox-px)**2 + (oy-py)**2)
            
            if d < OBSTACLE_RADIUS_CM:
                penalty = max(penalty, 80.0)
            elif d < OBSTACLE_RADIUS_CM + 15:
                penalty = max(penalty, 25.0)
        
        return base_dist + penalty

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
        """Find optimal visit order using Euclidean-based TSP."""
        all_view_positions = self.grid.get_view_obstacle_positions(retrying)
        if not all_view_positions:
            return [], 1e9

        best_distance = 1e9
        best_path = []

        start_state = self.robot.get_start_state()
        start_xy = self._state_to_xy(start_state)

        for op in self._get_visit_options(len(all_view_positions)):
            items_states = [start_state]
            items_xy = [start_xy]
            cur_view_positions = []

            for idx in range(len(all_view_positions)):
                if op[idx] == '1':
                    cur_view_positions.append(all_view_positions[idx])
                    for vp in all_view_positions[idx]:
                        items_states.append(vp)
                        items_xy.append(self._state_to_xy(vp))

            if not cur_view_positions:
                continue

            # Pairwise Euclidean distances (obstacle-aware)
            n_total = len(items_xy)
            all_ob_ids = [s.screenshot_id for s in items_states]

            full_cost = np.zeros((n_total, n_total))
            for s in range(n_total):
                for e in range(n_total):
                    if s == e:
                        continue
                    skip = set()
                    if all_ob_ids[s] != -1:
                        skip.add(all_ob_ids[s])
                    if all_ob_ids[e] != -1:
                        skip.add(all_ob_ids[e])
                    full_cost[s][e] = self._euclidean_distance_obstacle_aware(
                        items_xy[s], items_xy[e], skip_ids=skip)

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
    # SPIN COMMAND GENERATION
    # =================================================================

    @staticmethod
    def _compute_spin(current_theta_rad, face_x_cm, face_y_cm, robot_x_cm, robot_y_cm):
        """Compute the spot-turn needed to face the obstacle.
        
        Returns:
            (command_str, new_theta_rad) e.g. ("TL45", 1.57)
            or (None, current_theta_rad) if already facing it
        """
        target_theta = math.atan2(face_y_cm - robot_y_cm, face_x_cm - robot_x_cm)
        diff = target_theta - current_theta_rad
        diff = (diff + math.pi) % (2 * math.pi) - math.pi  # normalize to [-pi, pi]
        diff_deg = math.degrees(diff)

        if abs(diff_deg) < 3.0:
            return None, current_theta_rad  # close enough, no spin needed

        # Make sure deg is an integer
        deg = int(abs(round(diff_deg)))
        
        # Ensure there is no formatting like {deg:03d} here
        if diff_deg > 0:
            return f"TL{deg}", target_theta
        else:
            return f"TR{deg}", target_theta

    # =================================================================
    # MAIN PIPELINE
    # =================================================================

    def plan_full_route(self, retrying=False, obstacles_data=None):
        """Full pipeline: TSP + Hybrid A* + spot-turn + SNAP.
        
        Returns:
            (commands, waypoint_path, distance)
        """
        t0 = time.time()

        # Phase 1: TSP ordering (Euclidean cost — fast)
        waypoint_path, distance = self.get_optimal_order_dp(retrying)
        t_tsp = time.time() - t0
        print(f"Phase 1 (Euclidean TSP): {t_tsp:.2f}s, distance={distance:.0f}cm")

        if not waypoint_path:
            print("No valid path found!")
            return ["FIN"], [], 1e9

        for s in waypoint_path:
            if s.screenshot_id != -1:
                print(f"  Visit ob{s.screenshot_id} at "
                      f"({s.x:.1f},{s.y:.1f}) {Direction(s.direction).name}")

        # Phase 2: Drive + Spin + Snap for each leg
        t1 = time.time()
        obs_expanded = build_obstacle_list(obstacles_data) if obstacles_data else []
        obstacles_dict = {ob['id']: ob for ob in obstacles_data} if obstacles_data else {}
        ob_id_to_idx = {ob['id']: i for i, ob in enumerate(obstacles_data)} if obstacles_data else {}

        commands = []
        current_pose = self._state_to_pose(waypoint_path[0])

        for i in range(len(waypoint_path) - 1):
            wp_from = waypoint_path[i]
            wp_to = waypoint_path[i + 1]

            start_pose = current_pose
            goal_xy = self._state_to_xy(wp_to)  # position only!

            # Obstacle index for radius relaxation
            start_ob_idx = None
            goal_ob_idx = None
            if wp_from.screenshot_id != -1 and wp_from.screenshot_id in ob_id_to_idx:
                start_ob_idx = ob_id_to_idx[wp_from.screenshot_id]
            if wp_to.screenshot_id != -1 and wp_to.screenshot_id in ob_id_to_idx:
                goal_ob_idx = ob_id_to_idx[wp_to.screenshot_id]

            # --- DRIVE: Hybrid A* to reach position ---
            path, iters = hybrid_astar_search(
                start_pose, goal_xy, obs_expanded,
                start_obstacle_idx=start_ob_idx,
                goal_obstacle_idx=goal_ob_idx)

            if path is None:
                # Retry with relaxed tolerance
                path, iters = hybrid_astar_search(
                    start_pose, goal_xy, obs_expanded,
                    goal_tolerance_cm=12.0,
                    start_obstacle_idx=start_ob_idx,
                    goal_obstacle_idx=goal_ob_idx)

            if path:
                seg_cmds = path_to_commands(path)
                commands.extend(seg_cmds)
                end = path[-1]
                current_pose = (end[0], end[1], end[2])
                print(f"  Leg {i}: {len(path)} steps, {iters} iters")
            else:
                print(f"  Leg {i}: SKIPPED (no path found, {iters} iters)")
                continue

            # --- SPIN: rotate to face the obstacle ---
            if wp_to.screenshot_id != -1 and obstacles_dict:
                ob = obstacles_dict[wp_to.screenshot_id]

                # Compute face center
                face_offsets = {
                    0: (0, 5), 2: (5, 0), 4: (0, -5), 6: (-5, 0)}
                fx, fy = face_offsets.get(ob['d'], (0, 0))
                face_x = ob['x'] * 10 + 5 + fx
                face_y = ob['y'] * 10 + 5 + fy

                rx, ry, rtheta = current_pose
                spin_cmd, new_theta = self._compute_spin(
                    rtheta, face_x, face_y, rx, ry)

                if spin_cmd:
                    commands.append(spin_cmd)
                    current_pose = (rx, ry, new_theta)

                # --- SNAP ---
                commands.append(f"SNAP{wp_to.screenshot_id}")

        commands.append("FIN")
        commands = self._compress_commands(commands)

        t_astar = time.time() - t1
        print(f"Phase 2 (A* + spin): {t_astar:.2f}s")
        print(f"Total: {t_tsp + t_astar:.2f}s, {len(commands)} commands")

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