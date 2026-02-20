"""
Hybrid A* pathfinding with SPOT-TURN capability.

KEY DIFFERENCE FROM REEDS-SHEPP VERSION:
  The robot can spin on the spot at waypoints, so the A* only needs to
  reach the correct (x, y) position — heading is irrelevant at arrival.
  
  This means:
  - Heuristic: Euclidean distance (fast, admissible)
  - Goal check: position only (no heading constraint)
  - Search space: effectively 2.5D instead of 3D (heading still needed
    for motion primitives, but not for goal matching)
  - Result: dramatically faster search, shorter paths, no zigzag approaches

State: continuous (x, y, theta)
Motion primitives: S, L, R, B, BL, BR (Ackermann transit)
Goal: reach (x, y) within tolerance — any heading accepted
Spin: handled AFTER arrival by algo.py (TL/TR commands)
"""

import math
import heapq
from consts import (
    ROBOT_TURN_RADIUS_CM,
    ROBOT_SPEED_CM_S,
    ROBOT_WHEELBASE_CM,
    ROBOT_RADIUS_CM
)


# =============================================================================
# PARAMETERS
# =============================================================================
TURN_RADIUS_CM = ROBOT_TURN_RADIUS_CM
STEP_SIZE_CM = 8.0
ARENA_SIZE_CM = 200.0
ROBOT_CLEARANCE_CM = 15.0

# Discretization
CELL_SIZE_CM = 4.0
N_THETA_BUCKETS = 72          # 5 deg each (still needed for motion primitives)
THETA_RES = 2 * math.pi / N_THETA_BUCKETS

# Search
MAX_ITERATIONS = 200000
REVERSE_PENALTY = 1.2
DIRECTION_CHANGE_PENALTY = 8.0

# Obstacle clearance
OBSTACLE_RADIUS_CM = 28.0

# Arc collision check resolution
ARC_CHECK_POINTS = 10


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

def _check_line_collision(x0, y0, x1, y1, obstacles):
    for i in range(ARC_CHECK_POINTS + 1):
        t = i / ARC_CHECK_POINTS
        px = x0 + t * (x1 - x0)
        py = y0 + t * (y1 - y0)
        if not _in_arena(px, py):
            return True
        if _point_hits_obstacle(px, py, obstacles):
            return True
    return False

def _check_arc_collision(x0, y0, theta0, arc_angle, turn_radius, direction, obstacles):
    sign = 1 if direction == 'L' else -1
    cx = x0 - sign * turn_radius * math.sin(theta0)
    cy = y0 + sign * turn_radius * math.cos(theta0)
    start_a = math.atan2(y0 - cy, x0 - cx)

    for i in range(ARC_CHECK_POINTS + 1):
        t = i / ARC_CHECK_POINTS
        a = start_a + sign * arc_angle * t
        px = cx + turn_radius * math.cos(a)
        py = cy + turn_radius * math.sin(a)
        if not _in_arena(px, py):
            return True
        if _point_hits_obstacle(px, py, obstacles):
            return True
    return False


# =============================================================================
# PRECOMPUTED EULER DELTAS — BICYCLE MODEL (matches simulator)
# =============================================================================

def _precompute_euler_deltas():
    """Precompute movement deltas at heading=0 (EAST) using bicycle model."""
    R = ROBOT_TURN_RADIUS_CM
    L = ROBOT_WHEELBASE_CM
    speed = ROBOT_SPEED_CM_S
    dt = 1.0 / 60.0
    step = STEP_SIZE_CM

    delta_max = math.atan(L / R)

    deltas = {}
    primitives = [
        ('S',   speed,  0.0),
        ('L',   speed,  delta_max),
        ('R',   speed, -delta_max),
        ('B',  -speed,  0.0),
        ('BL', -speed,  delta_max),
        ('BR', -speed, -delta_max),
    ]

    for name, v, delta in primitives:
        x, y, t = 0.0, 0.0, 0.0
        traveled = 0.0
        omega = (v / L) * math.tan(delta) if abs(delta) > 1e-9 else 0.0
        while traveled < step:
            x += v * math.cos(t) * dt
            y += v * math.sin(t) * dt
            t += omega * dt
            traveled += abs(v * dt)
        deltas[name] = (x, y, t)

    return deltas

_EULER_DELTAS = _precompute_euler_deltas()


def _get_successors(x, y, theta, obstacles, prev_move=None):
    results = []
    for move, (dx, dy, dt) in _EULER_DELTAS.items():
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        nx = x + dx * cos_t - dy * sin_t
        ny = y + dx * sin_t + dy * cos_t
        nt = theta + dt

        if not _in_arena(nx, ny):
            continue

        if move in ('S', 'B'):
            if _check_line_collision(x, y, nx, ny, obstacles):
                continue
        else:
            arc_angle = abs(dt)
            direction = 'L' if move in ('L', 'BL') else 'R'
            if _check_arc_collision(x, y, theta, arc_angle, TURN_RADIUS_CM, direction, obstacles):
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
# EUCLIDEAN HEURISTIC (replaces Reeds-Shepp — much faster)
# =============================================================================

def _h_euclidean(x, y, gx, gy):
    """Euclidean distance heuristic. Admissible since the robot cannot
    reach a point faster than straight-line distance regardless of heading."""
    return math.sqrt((x - gx)**2 + (y - gy)**2)


# =============================================================================
# HYBRID A* SEARCH — POSITION-ONLY GOAL
# =============================================================================

def hybrid_astar_search(start, goal_xy, obstacles_expanded,
                        goal_tolerance_cm=5.0,
                        start_obstacle_idx=None, goal_obstacle_idx=None):
    """Find shortest path from start to goal POSITION (heading-free).
    
    The robot will spot-turn at the goal to face the obstacle, so we
    only need to reach the correct (x, y). This makes the search much
    faster and more reliable than heading-constrained search.
    
    Args:
        start: (x, y, theta) — robot's current pose
        goal_xy: (x, y) — target position (heading irrelevant)
        obstacles_expanded: list of (cx, cy, radius)
        goal_tolerance_cm: how close to goal position counts as arrival
        start_obstacle_idx: reduce this obstacle's radius near start
        goal_obstacle_idx: reduce this obstacle's radius near goal
    
    Returns:
        (path, iterations) where path = [(x, y, theta, move), ...]
        path[-1] gives the final pose (with actual arrival heading)
    """
    sx, sy, st = start
    gx, gy = goal_xy

    # Reduce radius for start/goal obstacles to allow close approach
    CAPTURE_CLEARANCE = 22.0
    obs_for_search = []
    for i, (ox, oy, orad) in enumerate(obstacles_expanded):
        if i == start_obstacle_idx or i == goal_obstacle_idx:
            obs_for_search.append((ox, oy, CAPTURE_CLEARANCE))
        else:
            obs_for_search.append((ox, oy, orad))

    counter = 0
    open_set = []
    g_cost = {}
    parent = {}

    sk = _grid_key(sx, sy, st)
    g_cost[sk] = 0
    h_start = _h_euclidean(sx, sy, gx, gy)
    heapq.heappush(open_set, (h_start, counter, sx, sy, st, None))

    iterations = 0
    while open_set and iterations < MAX_ITERATIONS:
        iterations += 1
        f, _, cx, cy, ct, prev_move = heapq.heappop(open_set)
        ck = _grid_key(cx, cy, ct)

        cur_g = g_cost.get(ck, float('inf'))
        if cur_g < f - _h_euclidean(cx, cy, gx, gy) - 1e-6:
            continue

        # POSITION-ONLY goal check (no heading constraint!)
        dist = math.sqrt((cx - gx)**2 + (cy - gy)**2)
        if dist < goal_tolerance_cm:
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
                h = _h_euclidean(nx, ny, gx, gy)
                heapq.heappush(open_set, (new_g + h, counter, nx, ny, nt, move))

    return None, iterations


# =============================================================================
# PATH TO DRIVE COMMANDS (spin is added by algo.py, not here)
# =============================================================================

def path_to_commands(path):
    """Convert Hybrid A* path to drive commands only.
    
    Spin commands (TL/TR) are NOT generated here — they are added by
    algo.py after the drive phase, since the spin angle depends on which
    obstacle face we need to photograph.
    
    Commands:
      S  -> FW{dist}   forward straight
      B  -> BW{dist}   backward straight
      L  -> FL{dist}   full-lock left, forward
      R  -> FR{dist}   full-lock right, forward
      BL -> BL{dist}   full-lock left, backward
      BR -> BR{dist}   full-lock right, backward
    """
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
        dist = round(STEP_SIZE_CM * count)
        if move_type == 'S':
            commands.append(f"FW{dist}")
        elif move_type == 'B':
            commands.append(f"BW{dist}")
        elif move_type == 'L':
            commands.append(f"FL{dist}")
        elif move_type == 'R':
            commands.append(f"FR{dist}")
        elif move_type == 'BL':
            commands.append(f"BL{dist}")
        elif move_type == 'BR':
            commands.append(f"BR{dist}")

    return commands


# =============================================================================
# CONVENIENCE
# =============================================================================

def build_obstacle_list(obstacles):
    """Convert obstacle dicts to (cx, cy, radius) tuples."""
    expanded = []
    for ob in obstacles:
        cx = ob['x'] * 10 + 5
        cy = ob['y'] * 10 + 5
        expanded.append((cx, cy, OBSTACLE_RADIUS_CM))
    return expanded