"""
MazeSolver: Obstacle visit ordering + smooth path planning.

CHANGES FROM DUBINS VERSION:
  - TSP cost matrix uses Reeds-Shepp path length (shorter, accounts for reverse)
  - Path sampling uses Reeds-Shepp curves for obstacle-aware cost
  - Hybrid A* uses RS heuristic + 6 motion primitives

Architecture:
  1. Get candidate capture positions for each obstacle (from Entity.py — unchanged)
  2. Build TSP cost matrix using REEDS-SHEPP PATH LENGTH
  3. Solve TSP with python_tsp for optimal visit order
  4. Plan each leg with Hybrid A* (RS heuristic, respects turning radius, free reverse)
  5. Convert paths to FW/BW/FL/FR/BL/BR motor commands
"""

import math
import time
from typing import List, Tuple, Optional
import numpy as np

from entities.Robot import Robot
from entities.Entity import Obstacle, CellState, Grid
from consts import Direction, ITERATIONS
from python_tsp.exact import solve_tsp_dynamic_programming
from reeds_shepp import (
    get_optimal_path_length as rs_path_length,
    get_optimal_path_segments as rs_path_segments,
    sample_path as rs_sample_path,
)
from hybrid_astar import (
    hybrid_astar_search, path_to_commands, build_obstacle_list,
    TURN_RADIUS_CM, OBSTACLE_RADIUS_CM
)
from helper import compute_micro_turn_angle


class MazeSolver:
    def __init__(
            self,
            size_x: int,
            size_y: int,
            robot_x: int,
            robot_y: int,
            robot_direction: Direction,
            big_turn=None
    ):
        self.grid = Grid(size_x, size_y)
        self.robot = Robot(robot_x, robot_y, robot_direction)
        self.size_x = size_x
        self.size_y = size_y
        self.obstacle_centers = []  # (x_cm, y_cm, ob_id) for TSP collision check

    def add_obstacle(self, x: int, y: int, direction: Direction, obstacle_id: int):
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
    def _grid_to_cm(gx, gy):
        """Convert integer grid cell to cm (cell center). Used for start state."""
        return gx * 10 + 5, gy * 10 + 5

    @staticmethod
    def _dir_to_rad(d):
        """Convert Direction enum to radians."""
        return Direction.to_angle_rad(d)

    @staticmethod
    def _state_to_pose(state: CellState):
        """Convert CellState to (x_cm, y_cm, theta_rad).
        
        Two coordinate conventions:
        - Start state: integer grid cell (1,1) => center at (15, 15) cm
        - View cone candidates: float grid position (3.5, 4.5) already in
          the continuous coordinate system => (35, 45) cm
        """
        if hasattr(state, 'theta_rad') and state.theta_rad is not None:
            x_cm = state.x * 10
            y_cm = state.y * 10
            theta = state.theta_rad
        else:
            x_cm = state.x * 10 + 5
            y_cm = state.y * 10 + 5
            theta = MazeSolver._dir_to_rad(state.direction)
        return (x_cm, y_cm, theta)

    # =================================================================
    # REEDS-SHEPP DISTANCE (replaces Dubins for TSP cost matrix)
    # =================================================================

    @staticmethod
    def _rs_distance(pose_a, pose_b):
        """Compute Reeds-Shepp path length between two poses (no obstacle check).
        
        Always <= Dubins distance since RS can reverse freely.
        """
        try:
            length = rs_path_length(pose_a, pose_b, TURN_RADIUS_CM)
            return length
        except:
            dx = pose_b[0] - pose_a[0]
            dy = pose_b[1] - pose_a[1]
            return math.sqrt(dx*dx + dy*dy) + 50.0

    @staticmethod
    def _sample_rs_path(pose_a, pose_b):
        """Sample points along a Reeds-Shepp path for collision checking."""
        try:
            result = rs_path_segments(pose_a, pose_b, TURN_RADIUS_CM)
            if result is None:
                return [(pose_a[0], pose_a[1]), (pose_b[0], pose_b[1])]
            segments, _ = result
            return rs_sample_path(pose_a, TURN_RADIUS_CM, segments=segments)
        except:
            return [(pose_a[0], pose_a[1]), (pose_b[0], pose_b[1])]

    def _rs_distance_obstacle_aware(self, pose_a, pose_b, obstacle_centers,
                                     skip_ids=None):
        """Reeds-Shepp distance with penalty for paths crossing obstacles.
        
        Uses a fast straight-line pre-check: only does expensive path sampling
        if the straight line passes within danger zone of an obstacle.
        """
        base_dist = self._rs_distance(pose_a, pose_b)
        if not obstacle_centers:
            return base_dist
        
        skip = skip_ids or set()
        
        # Quick check: is any non-skipped obstacle near the straight line?
        ax, ay = pose_a[0], pose_a[1]
        bx, by = pose_b[0], pose_b[1]
        dx, dy = bx - ax, by - ay
        line_len = math.sqrt(dx*dx + dy*dy)
        
        # RS paths can deviate up to ~2*R from the straight line
        CHECK_RADIUS = OBSTACLE_RADIUS_CM + 2 * TURN_RADIUS_CM
        
        needs_check = False
        for ox, oy, oid in obstacle_centers:
            if oid in skip:
                continue
            if line_len < 1e-6:
                d = math.sqrt((ox-ax)**2 + (oy-ay)**2)
            else:
                t = max(0, min(1, ((ox-ax)*dx + (oy-ay)*dy) / (line_len*line_len)))
                px, py = ax + t*dx, ay + t*dy
                d = math.sqrt((ox-px)**2 + (oy-py)**2)
            if d < CHECK_RADIUS:
                needs_check = True
                break
        
        if not needs_check:
            return base_dist
        
        # Detailed check: sample along actual RS path
        points = self._sample_rs_path(pose_a, pose_b)
        
        penalty = 0
        for px, py in points:
            for ox, oy, oid in obstacle_centers:
                if oid in skip:
                    continue
                d = math.sqrt((px - ox)**2 + (py - oy)**2)
                if d < OBSTACLE_RADIUS_CM:
                    penalty = max(penalty, 100.0)
                elif d < OBSTACLE_RADIUS_CM + 10:
                    penalty = max(penalty, 30.0)
        
        return base_dist + penalty

    # =================================================================
    # VISIT ORDER GENERATION (unchanged logic, uses RS distances)
    # =================================================================

    @staticmethod
    def _get_visit_options(n):
        """Generate all n-digit binary strings (for obstacle skip combos)."""
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
            MazeSolver._generate_combination(view_positions, index + 1, current, result, iteration_left)
            current.pop()

    # =================================================================
    # MAIN SOLVER
    # =================================================================

    def get_optimal_order_dp(self, retrying) -> Tuple[List[CellState], float]:
        """Find optimal obstacle visit order using Reeds-Shepp-based TSP.
        
        Returns:
            (optimal_path, distance) where optimal_path is a list of CellStates
        """
        # Step 1: Get candidate capture positions for each obstacle
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

            # Step 2: Pre-compute ALL pairwise RS distances (obstacle-aware)
            n_total = len(items_poses)
            
            all_ob_ids = []
            for state in items_states:
                all_ob_ids.append(state.screenshot_id)
            
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
                    full_cost[s][e] = self._rs_distance_obstacle_aware(
                        items_poses[s], items_poses[e],
                        self.obstacle_centers, skip_ids=skip
                    )

            # Step 3: Generate candidate combinations
            combinations = []
            self._generate_combination(cur_view_positions, 0, [], combinations, [ITERATIONS])

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
                            cost_np[s][e] = full_cost[visited_indices[s]][visited_indices[e]]

                # No cost to "return" to start (open-path TSP)
                cost_np[:, 0] = 0

                # Step 4: Solve TSP
                _permutation, _distance = solve_tsp_dynamic_programming(cost_np)

                if _distance + fixed_cost >= best_distance:
                    continue

                best_distance = _distance + fixed_cost
                best_path = [items_states[0]]

                for i in range(1, len(_permutation)):
                    if _permutation[i] == 0:
                        continue
                    state = items_states[visited_indices[_permutation[i]]]
                    best_path.append(state)

            if best_path:
                break

        return best_path, best_distance

    def plan_full_route(self, retrying=False, obstacles_data=None):
        """Full pipeline: TSP ordering + Hybrid A* path planning + command generation.
        
        Returns:
            (commands, waypoint_path, distance)
        """
        t0 = time.time()

        # Phase 1: Find optimal visit order using RS-based TSP
        waypoint_path, distance = self.get_optimal_order_dp(retrying)
        t_tsp = time.time() - t0
        print(f"Phase 1 (RS TSP): {t_tsp:.2f}s, distance={distance:.0f}cm")

        if not waypoint_path:
            print("No valid path found!")
            return ["FIN"], [], 1e9

        for s in waypoint_path:
            if s.screenshot_id != -1:
                print(f"  Visit ob{s.screenshot_id} at ({s.x},{s.y}) {Direction(s.direction).name}")

        # Phase 2: Plan smooth paths between waypoints using Hybrid A*
        t1 = time.time()
        obs_expanded = build_obstacle_list(obstacles_data) if obstacles_data else []
        obstacles_dict = {ob['id']: ob for ob in obstacles_data} if obstacles_data else {}
        
        commands = []
        current_pose = self._state_to_pose(waypoint_path[0])
        
        for i in range(len(waypoint_path) - 1):
            wp_from = waypoint_path[i]
            wp_to = waypoint_path[i + 1]
            
            start_pose = current_pose
            goal_pose = self._state_to_pose(wp_to)
            
            # Determine which obstacles are start/goal targets for radius relaxation
            start_ob_idx = None
            goal_ob_idx = None
            ob_id_to_idx = {ob['id']: i for i, ob in enumerate(obstacles_data)} if obstacles_data else {}

            if wp_from.screenshot_id != -1 and wp_from.screenshot_id in ob_id_to_idx:
                start_ob_idx = ob_id_to_idx[wp_from.screenshot_id]
            if wp_to.screenshot_id != -1 and wp_to.screenshot_id in ob_id_to_idx:
                goal_ob_idx = ob_id_to_idx[wp_to.screenshot_id]

            # Run Hybrid A* with RS heuristic
            path, iters = hybrid_astar_search(
                start_pose, goal_pose, obs_expanded,
                start_obstacle_idx=start_ob_idx,
                goal_obstacle_idx=goal_ob_idx
            )
            
            if path:
                seg_cmds = path_to_commands(path)
                commands.extend(seg_cmds)
                seg_dist = sum(
                    math.sqrt((path[j][0]-path[j-1][0])**2 + (path[j][1]-path[j-1][1])**2)
                    for j in range(1, len(path))
                )
                moves = ''.join(p[3][0] for p in path[1:])
                
                end = path[-1]
                current_pose = (end[0], end[1], end[2])
                
                print(f"  Leg {i}: {len(path)} steps, {seg_dist:.0f}cm, {iters} iters [{moves[:50]}]")
            else:
                # Retry with relaxed tolerance
                path2, iters2 = hybrid_astar_search(
                    start_pose, goal_pose, obs_expanded,
                    goal_tolerance_cm=15.0, goal_tolerance_deg=25.0,
                    start_obstacle_idx=start_ob_idx,
                    goal_obstacle_idx=goal_ob_idx
                )
                if path2:
                    seg_cmds = path_to_commands(path2)
                    commands.extend(seg_cmds)
                    end = path2[-1]
                    current_pose = (end[0], end[1], end[2])
                    print(f"  Leg {i}: {len(path2)} steps, {iters2} iters [RETRY-RELAXED]")
                else:
                    print(f"  Leg {i}: SKIPPED (no valid path found)")
                    continue
            
            # Insert SNAP at capture waypoint
            if wp_to.screenshot_id != -1 and obstacles_dict:
                ob = obstacles_dict[wp_to.screenshot_id]
                
                face_offsets_cm = {
                    0: (0, 5),   # NORTH face
                    2: (5, 0),   # EAST face
                    4: (0, -5),  # SOUTH face
                    6: (-5, 0),  # WEST face
                }
                fx_off, fy_off = face_offsets_cm.get(ob['d'], (0, 0))
                face_x_cm = ob['x'] * 10 + 5 + fx_off
                face_y_cm = ob['y'] * 10 + 5 + fy_off
                
                rx, ry, rtheta = current_pose
                
                angle_to_face = math.atan2(face_y_cm - ry, face_x_cm - rx)
                diff = angle_to_face - rtheta
                diff = (diff + math.pi) % (2 * math.pi) - math.pi
                diff_deg = math.degrees(diff)
                
                if abs(diff_deg) < 10:
                    signal = 'C'
                elif diff_deg > 0:
                    signal = 'L'
                else:
                    signal = 'R'
                
                commands.append(f"SNAP{wp_to.screenshot_id}_{signal}")

        commands.append("FIN")
        
        # Compress consecutive FW/BW
        commands = self._compress_commands(commands)
        
        t_hybrid = time.time() - t1
        print(f"Phase 2 (Hybrid A* + RS): {t_hybrid:.2f}s")
        print(f"Total: {t_tsp + t_hybrid:.2f}s, {len(commands)} commands")
        
        return commands, waypoint_path, distance

    @staticmethod
    def _compress_commands(commands):
        """Merge consecutive FW/BW commands for efficiency."""
        if not commands:
            return commands
        compressed = [commands[0]]
        for i in range(1, len(commands)):
            cmd = commands[i]
            prev = compressed[-1]
            if cmd.startswith("FW") and prev.startswith("FW"):
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