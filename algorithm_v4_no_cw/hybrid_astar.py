"""
Hybrid A* pathfinding — Reeds-Shepp heuristic + heading-aware goal.

TWO MODES:
  1. HEADING-CONSTRAINED (default): reach (x, y, theta) within tolerance.
     Used when Reeds-Shepp direct path is blocked by obstacles.
     Tighter heuristic => fewer expansions, shorter physical paths.

  2. POSITION-ONLY (fallback): reach (x, y), any heading.
     Used when heading-constrained search fails (very cluttered areas).
     The caller adds a small CW aim-spin afterward.

Heuristic: Reeds-Shepp optimal path length (analytic, ~0.05ms per call).
  This is a TIGHT admissible heuristic that accounts for turning radius,
  so the search expands far fewer nodes than Euclidean and finds shorter
  drivable paths — especially when large heading changes are needed.
"""

import math
import heapq
from consts import (
    ROBOT_TURN_RADIUS_CM,
    ROBOT_TURN_RADIUS_FL_CM,
    ROBOT_TURN_RADIUS_FR_CM,
    ROBOT_TURN_RADIUS_BL_CM,
    ROBOT_TURN_RADIUS_BR_CM,
    ROBOT_TURN_RADIUS_MAX_CM,
    ROBOT_TURN_RADIUS_MIN_CM,
    ROBOT_SPEED_CM_S,
    ROBOT_WHEELBASE_CM,
    ROBOT_RADIUS_CM,
    ENCODER_SCALE_FL, ENCODER_SCALE_FR,
    ENCODER_SCALE_BL, ENCODER_SCALE_BR,
    ENCODER_SCALE_FW, ENCODER_SCALE_BW
)
from reeds_shepp import get_optimal_path_length


# =============================================================================
# PARAMETERS
# =============================================================================
# Asymmetric radii: each arc primitive (FL, FR, BL, BR) uses its own
# measured turning radius. MAX is for RS planning/collision (conservative),
# MIN is for RS heuristic (admissible lower bound).
TURN_RADIUS_FL_CM = ROBOT_TURN_RADIUS_FL_CM
TURN_RADIUS_FR_CM = ROBOT_TURN_RADIUS_FR_CM
TURN_RADIUS_BL_CM = ROBOT_TURN_RADIUS_BL_CM
TURN_RADIUS_BR_CM = ROBOT_TURN_RADIUS_BR_CM
TURN_RADIUS_CM = ROBOT_TURN_RADIUS_MAX_CM
TURN_RADIUS_HEURISTIC_CM = ROBOT_TURN_RADIUS_MIN_CM
STEP_SIZE_CM = 8.0
ARENA_SIZE_CM = 200.0
ROBOT_CLEARANCE_CM = 15.0

# Discretization
CELL_SIZE_CM = 4.0
N_THETA_BUCKETS = 72
THETA_RES = 2 * math.pi / N_THETA_BUCKETS

# Search
MAX_ITERATIONS = 200000
REVERSE_PENALTY = 1.2
DIRECTION_CHANGE_PENALTY = 8.0

# Obstacle clearance — robot half-diagonal is ~21cm, plus safety margin.
# 33cm keeps the robot body ≥12cm from any obstacle edge during transit.
OBSTACLE_RADIUS_CM = 33.0


# =============================================================================
# STATE DISCRETIZATION
# =============================================================================

def _theta_bucket(theta):
    return int(((theta % (2 * math.pi)) / THETA_RES)) % N_THETA_BUCKETS

def _grid_key(x, y, theta):
    return (int(x / CELL_SIZE_CM), int(y / CELL_SIZE_CM), _theta_bucket(theta))

def _in_arena(x, y):
    return (ROBOT_CLEARANCE_CM <= x <= ARENA_SIZE_CM - ROBOT_CLEARANCE_CM and
            ROBOT_CLEARANCE_CM <= y <= ARENA_SIZE_CM - ROBOT_CLEARANCE_CM)


# =============================================================================
# COLLISION CHECKING
# =============================================================================

def _point_hits_obstacle(px, py, obstacles):
    for (ox, oy, orad) in obstacles:
        if (px - ox)**2 + (py - oy)**2 < orad**2:
            return True
    return False

# =============================================================================
# PRECOMPUTED EULER DELTAS — BICYCLE MODEL (matches simulator)
# =============================================================================
# We now store the full intermediate trajectory (in local frame) for each
# motion primitive so collision checks use the EXACT same bicycle-model
# path the simulator will execute.  This replaces the old idealized
# circular-arc collision check that could diverge by several cm.

_ARC_COLLISION_POINTS = 4    # Samples per 8cm step (~2cm spacing; sufficient for 33cm obstacle zones)

def _precompute_euler_deltas():
    """Precompute with INDEPENDENT FL/FR/BL/BR turning radii.
    
    Naming: "BL" = front of car swings LEFT while reversing.
      BL = right steering + backward gear  (front swings left)
      BR = left steering  + backward gear  (front swings right)

    Each primitive stores:
      (dx, dy, dtheta, trajectory_points)
    where trajectory_points is a list of (lx, ly) in the LOCAL frame
    (robot starts at origin heading along +x).  The caller rotates these
    into world frame for collision checking.
    """
    L = ROBOT_WHEELBASE_CM
    speed = ROBOT_SPEED_CM_S
    dt = 1.0 / 60.0
    step = STEP_SIZE_CM

    delta_fl = math.atan(L / TURN_RADIUS_FL_CM)
    delta_fr = math.atan(L / TURN_RADIUS_FR_CM)
    delta_bl = math.atan(L / TURN_RADIUS_BL_CM)
    delta_br = math.atan(L / TURN_RADIUS_BR_CM)

    deltas = {}
    primitives = [
        # (name, velocity, steering_angle)
        ('S',   speed,  0.0),
        ('L',   speed,  delta_fl),     # FL: forward, steer left
        ('R',   speed, -delta_fr),     # FR: forward, steer right
        ('B',  -speed,  0.0),
        ('BL', -speed, -delta_bl),     # BL: backward, right steer → front swings left
        ('BR', -speed,  delta_br),     # BR: backward, left steer → front swings right
    ]

    for name, v, delta in primitives:
        x, y, t = 0.0, 0.0, 0.0
        traveled = 0.0
        omega = (v / L) * math.tan(delta) if abs(delta) > 1e-9 else 0.0

        # Collect intermediate points at even intervals for collision checks
        traj_points = [(0.0, 0.0)]
        total_ticks = 0
        # First pass: count total ticks
        tmp_traveled = 0.0
        while tmp_traveled < step:
            tmp_traveled += abs(v * dt)
            total_ticks += 1
        # Interval between samples
        sample_interval = max(1, total_ticks // _ARC_COLLISION_POINTS)

        tick = 0
        while traveled < step:
            x += v * math.cos(t) * dt
            y += v * math.sin(t) * dt
            t += omega * dt
            traveled += abs(v * dt)
            tick += 1
            if tick % sample_interval == 0:
                traj_points.append((x, y))

        # Always include the endpoint
        traj_points.append((x, y))
        deltas[name] = (x, y, t, traj_points)

    return deltas

_EULER_DELTAS = _precompute_euler_deltas()


def _check_trajectory_collision(x0, y0, theta0, local_points, obstacles):
    """Check collision along precomputed bicycle-model trajectory points.
    
    local_points are in the robot's local frame (heading = +x at origin).
    We rotate them into world frame using (x0, y0, theta0).
    """
    cos_t = math.cos(theta0)
    sin_t = math.sin(theta0)
    for lx, ly in local_points:
        px = x0 + lx * cos_t - ly * sin_t
        py = y0 + lx * sin_t + ly * cos_t
        if not _in_arena(px, py):
            return True
        if _point_hits_obstacle(px, py, obstacles):
            return True
    return False


def _get_successors(x, y, theta, obstacles, prev_move=None):
    results = []
    for move, (dx, dy, dt, traj_points) in _EULER_DELTAS.items():
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        nx = x + dx * cos_t - dy * sin_t
        ny = y + dx * sin_t + dy * cos_t
        nt = theta + dt

        if not _in_arena(nx, ny):
            continue

        # Collision check using the ACTUAL bicycle-model trajectory
        if _check_trajectory_collision(x, y, theta, traj_points, obstacles):
            continue

        is_reverse = move in ('B', 'BL', 'BR')
        cost = STEP_SIZE_CM * (REVERSE_PENALTY if is_reverse else 1.0)

        if prev_move is not None:
            prev_is_reverse = prev_move in ('B', 'BL', 'BR')
            if is_reverse != prev_is_reverse:
                cost += DIRECTION_CHANGE_PENALTY

        results.append((nx, ny, nt, cost, move))

    return results


# =============================================================================
# REEDS-SHEPP HEURISTIC (tight, admissible)
# =============================================================================

def _h_reeds_shepp(x, y, theta, gx, gy, gtheta):
    """Reeds-Shepp path length as heuristic.
    
    Admissible because RS gives the shortest possible path length
    in obstacle-free space. The actual path (with obstacles) can
    only be longer.
    """
    return get_optimal_path_length(
        (x, y, theta), (gx, gy, gtheta), TURN_RADIUS_HEURISTIC_CM)

def _h_euclidean(x, y, gx, gy):
    """Euclidean heuristic (position only)."""
    return math.sqrt((x - gx)**2 + (y - gy)**2)


def _h_euclidean_with_heading(x, y, theta, gx, gy, gtheta):
    """Euclidean + lightweight heading penalty.
    
    Adds a fraction of the heading mismatch as extra cost.
    This biases A* toward paths arriving near the goal heading
    WITHOUT the 200us/call cost of full RS heuristic.
    
    The heading penalty is scaled by TURN_RADIUS so it's in cm
    (matching the distance heuristic units). This keeps it admissible
    when the weight is <= 1.0 (a heading change of X radians needs
    at least X * turn_radius cm of arc).
    """
    dist = math.sqrt((x - gx)**2 + (y - gy)**2)
    hdiff = abs((theta - gtheta + math.pi) % (2 * math.pi) - math.pi)
    # Weight: 0.5 * turn_radius * heading_diff is admissible
    # (real arc cost = turn_radius * heading_diff, so 0.5x is safe)
    heading_cost = 0.5 * TURN_RADIUS_CM * hdiff
    return dist + heading_cost


# =============================================================================
# HYBRID A* SEARCH — HEADING-CONSTRAINED (PRIMARY MODE)
# =============================================================================

def hybrid_astar_search(start, goal, obstacles_expanded,
                        goal_tolerance_cm=5.0,
                        heading_tolerance_rad=0.35,
                        position_only=False,
                        start_obstacle_idx=None,
                        goal_obstacle_idx=None,
                        skip_obstacle_indices=None,
                        max_iterations=None):
    """Find path from start pose to goal pose.
    
    PRIMARY MODE (position_only=False):
        Goal = (x, y, theta). Uses Reeds-Shepp heuristic.
        Robot arrives at the right position AND heading,
        minimizing or eliminating the aim-spin.
    
    FALLBACK MODE (position_only=True):
        Goal = (x, y). Uses Euclidean heuristic.
        Caller adds aim-spin afterward. Used when heading-constrained
        search fails (very tight spaces).
    
    Args:
        start: (x, y, theta) start pose
        goal: (x, y, theta) target pose (theta ignored if position_only)
        obstacles_expanded: list of (cx, cy, radius)
        goal_tolerance_cm: position tolerance
        heading_tolerance_rad: heading tolerance (heading mode only)
        position_only: if True, ignore goal heading
        start_obstacle_idx: reduce this obstacle's radius near start
        goal_obstacle_idx: reduce this obstacle's radius near goal
    
    Returns:
        (path, iterations) where path = [(x, y, theta, move), ...]
    """
    sx, sy, st = start
    gx, gy = goal[0], goal[1]
    gt = goal[2] if len(goal) > 2 and not position_only else st

    # Reduce radius for nearby obstacles (start, goal, or post-spin drift)
    CAPTURE_CLEARANCE = 25.0
    skip_set = set()
    if skip_obstacle_indices:
        skip_set = set(skip_obstacle_indices)
    if start_obstacle_idx is not None:
        skip_set.add(start_obstacle_idx)
    if goal_obstacle_idx is not None:
        skip_set.add(goal_obstacle_idx)

    obs_for_search = []
    for i, (ox, oy, orad) in enumerate(obstacles_expanded):
        if i in skip_set:
            obs_for_search.append((ox, oy, CAPTURE_CLEARANCE))
        else:
            obs_for_search.append((ox, oy, orad))

    counter = 0
    open_set = []
    g_cost = {}
    parent = {}

    sk = _grid_key(sx, sy, st)
    g_cost[sk] = 0

    # Use Reeds-Shepp heuristic when goal heading is known (tight, admissible)
    if not position_only:
        h_start = _h_reeds_shepp(sx, sy, st, gx, gy, gt)
    else:
        h_start = _h_euclidean(sx, sy, gx, gy)

    heapq.heappush(open_set, (h_start, counter, sx, sy, st, None))

    max_iter = max_iterations if max_iterations is not None else MAX_ITERATIONS
    iterations = 0
    while open_set and iterations < max_iter:
        iterations += 1
        f, _, cx, cy, ct, prev_move = heapq.heappop(open_set)
        ck = _grid_key(cx, cy, ct)

        cur_g = g_cost.get(ck, float('inf'))
        if not position_only:
            h_cur = _h_reeds_shepp(cx, cy, ct, gx, gy, gt)
        else:
            h_cur = _h_euclidean(cx, cy, gx, gy)

        if cur_g < f - h_cur - 1e-6:
            continue

        # Goal check
        dist = math.sqrt((cx - gx)**2 + (cy - gy)**2)
        if dist < goal_tolerance_cm:
            if position_only:
                goal_reached = True
            else:
                # Check heading too
                hdiff = abs((ct - gt + math.pi) % (2 * math.pi) - math.pi)
                goal_reached = hdiff < heading_tolerance_rad

            if goal_reached:
                path = [(cx, cy, ct, 'GOAL')]
                key = ck
                while key in parent:
                    pkey, move, px, py, pt = parent[key]
                    path.append((px, py, pt, move))
                    key = pkey
                path.reverse()
                return path, iterations

        for nx, ny, nt, cost, move in _get_successors(cx, cy, ct, obs_for_search, prev_move):
            new_g = cur_g + cost
            nk = _grid_key(nx, ny, nt)
            if nk not in g_cost or new_g < g_cost[nk]:
                g_cost[nk] = new_g
                parent[nk] = (ck, move, cx, cy, ct)
                counter += 1
                if not position_only:
                    h = _h_reeds_shepp(nx, ny, nt, gx, gy, gt)
                else:
                    h = _h_euclidean(nx, ny, gx, gy)
                heapq.heappush(open_set, (new_g + h, counter, nx, ny, nt, move))

    return None, iterations


# =============================================================================
# PATH TO DRIVE COMMANDS
# =============================================================================

def path_to_commands(path):
    """Convert Hybrid A* path to drive commands (distances in mm).

    Arc distances are scaled by encoder correction factors so the
    robot's encoder-based stopping matches the planned center arc.

    Commands:
      S  -> FW{dist_mm}   forward straight
      B  -> BW{dist_mm}   backward straight
      L  -> FL{dist_mm}   full-lock left, forward
      R  -> FR{dist_mm}   full-lock right, forward
      BL -> BL{dist_mm}   full-lock left, backward
      BR -> BR{dist_mm}   full-lock right, backward
    """
    _MOVE_MAP = {
        'S':  ('FW', ENCODER_SCALE_FW),
        'B':  ('BW', ENCODER_SCALE_BW),
        'L':  ('FL', ENCODER_SCALE_FL),
        'R':  ('FR', ENCODER_SCALE_FR),
        'BL': ('BL', ENCODER_SCALE_BL),
        'BR': ('BR', ENCODER_SCALE_BR),
    }

    if not path or len(path) < 2:
        return []

    segments = []
    for i in range(len(path) - 1):
        _, _, _, move = path[i]
        if move == 'GOAL':
            continue
        if segments and segments[-1][0] == move:
            segments[-1] = (move, segments[-1][1] + 1)
        else:
            segments.append((move, 1))

    commands = []
    for move_type, count in segments:
        entry = _MOVE_MAP.get(move_type)
        if entry is None:
            continue
        prefix, scale = entry
        dist_mm = round(STEP_SIZE_CM * count * scale * 10)
        commands.append(f"{prefix}{dist_mm}")

    return commands


# =============================================================================
# CONVENIENCE
# =============================================================================

def build_obstacle_list(obstacles):
    expanded = []
    for ob in obstacles:
        cx = ob['x'] * 10 + 5
        cy = ob['y'] * 10 + 5
        expanded.append((cx, cy, OBSTACLE_RADIUS_CM))
    return expanded