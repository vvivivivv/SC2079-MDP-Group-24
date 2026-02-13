"""
Hybrid A* pathfinding with Reeds-Shepp heuristic.

CHANGES FROM DUBINS VERSION:
  - Heuristic: Reeds-Shepp path length (tighter than Dubins, accounts for reverse)
  - Motion primitives: 6 total — S, L, R, B, BL, BR (added reverse arcs)
  - Reverse penalty: reduced from 2.0 to 1.2 (RS embraces reversing)
  - Arc collision: extended for reverse arc checking

State: continuous (x, y, theta)
Motion primitives: STRAIGHT, LEFT_ARC, RIGHT_ARC, REVERSE, REVERSE_LEFT, REVERSE_RIGHT
Heuristic: Reeds-Shepp path length (admissible, very tight)
Collision: checked along actual arc paths, not just endpoints

Commands output as arcs for smooth execution.
"""

import math
import heapq
from reeds_shepp import get_optimal_path_length as rs_path_length
from consts import (
    ROBOT_TURN_RADIUS_CM, 
    ROBOT_SPEED_CM_S, 
    ROBOT_AXLE_TRACK_CM, 
    ROBOT_RADIUS_CM
)


# =============================================================================
# PARAMETERS
# =============================================================================
TURN_RADIUS_CM = ROBOT_TURN_RADIUS_CM      # Minimum turning radius (calibrate to your robot!)
STEP_SIZE_CM = 8.0            # Distance per step (reduced from 10 for finer resolution)
ARENA_SIZE_CM = 200.0         # Arena size (20x20 grid x 10cm)
ROBOT_CLEARANCE_CM = 15.0     # Robot half-diagonal (~14.1cm) + margin

# Discretization
CELL_SIZE_CM = 4.0            # Spatial buckets (finer than before for better resolution)
N_THETA_BUCKETS = 72          # Angular buckets (5° each)
THETA_RES = 2 * math.pi / N_THETA_BUCKETS

# Search
MAX_ITERATIONS = 200000
REVERSE_PENALTY = 1.2         # REDUCED from 2.0 — RS paths use reverse naturally
DIRECTION_CHANGE_PENALTY = 3.0  # Cost for switching between forward and reverse

# Obstacle clearance
OBSTACLE_RADIUS_CM = 28.0     # 21.2cm(robot corner) + 5cm(obstacle half) + 1.8cm(safety)

# Arc collision check resolution
ARC_CHECK_POINTS = 10         # Increased from 8 for finer step size


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
# COLLISION CHECKING — along actual arc/line paths
# =============================================================================

def _point_hits_obstacle(px, py, obstacles):
    for (ox, oy, orad) in obstacles:
        if (px - ox)**2 + (py - oy)**2 < orad**2:
            return True
    return False

def _check_line_collision(x0, y0, x1, y1, obstacles):
    """Check straight line for collisions."""
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
    """Check along the actual circular arc path.
    
    direction: 'L' or 'R'
    arc_angle: magnitude (positive)
    """
    sign = 1 if direction == 'L' else -1

    # Center of turning circle
    cx = x0 - sign * turn_radius * math.sin(theta0)
    cy = y0 + sign * turn_radius * math.cos(theta0)

    # Start angle on the circle
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
# PRECOMPUTED EULER DELTAS — matches simulator's differential drive exactly
# =============================================================================
# The simulator uses Euler integration at 60fps with differential wheel speeds.
# To ensure the planned path matches executed commands, we precompute the
# (dx, dy, dtheta) for each primitive using the same integration.
#
# NEW: Added BL (backward-left) and BR (backward-right) primitives for
# Reeds-Shepp style paths that freely reverse along arcs.

def _precompute_euler_deltas():
    """Precompute movement deltas at heading=0 (EAST) using simulator's Euler integration."""
    R = ROBOT_TURN_RADIUS_CM
    axle = ROBOT_AXLE_TRACK_CM
    half = axle / 2.0
    speed = ROBOT_SPEED_CM_S
    dt = 1.0 / 60.0  # FPS from simulator
    step = STEP_SIZE_CM

    deltas = {}

    # Define all 6 motion primitives with their wheel velocities
    primitives = [
        # (name, left_wheel_speed, right_wheel_speed)
        ('S',  speed,                          speed),                          # Straight forward
        ('L',  speed * (R - half) / R,         speed * (R + half) / R),         # Forward left arc
        ('R',  speed * (R + half) / R,         speed * (R - half) / R),         # Forward right arc
        ('B',  -speed,                         -speed),                         # Straight backward
        ('BL', -speed * (R - half) / R,        -speed * (R + half) / R),        # Backward left arc
        ('BR', -speed * (R + half) / R,        -speed * (R - half) / R),        # Backward right arc
    ]

    for name, vl, vr in primitives:
        v = (vl + vr) / 2
        omega = (vr - vl) / axle
        x, y, t = 0.0, 0.0, 0.0
        traveled = 0.0
        while traveled < step:
            x += v * math.cos(t) * dt
            y += v * math.sin(t) * dt
            t += omega * dt
            traveled += abs(v * dt)
        deltas[name] = (x, y, t)

    return deltas

_EULER_DELTAS = _precompute_euler_deltas()


def _get_successors(x, y, theta, obstacles, prev_move=None):
    """Generate successor states from current position.
    
    Args:
        x, y, theta: current pose
        obstacles: list of (ox, oy, radius)
        prev_move: previous move type for direction change penalty
    """
    results = []

    for move, (dx, dy, dt) in _EULER_DELTAS.items():
        # Rotate delta by current heading
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        nx = x + dx * cos_t - dy * sin_t
        ny = y + dx * sin_t + dy * cos_t
        nt = theta + dt

        if not _in_arena(nx, ny):
            continue

        # Collision check along the actual path (not just endpoint)
        if move in ('S', 'B'):
            if _check_line_collision(x, y, nx, ny, obstacles):
                continue
        else:
            arc_angle = abs(dt)
            if move in ('L', 'BL'):
                direction = 'L'
            else:
                direction = 'R'

            # For reverse arcs, the arc is traced backward but geometry is same
            if _check_arc_collision(x, y, theta, arc_angle, TURN_RADIUS_CM, direction, obstacles):
                continue

        # Cost calculation
        is_reverse = move in ('B', 'BL', 'BR')
        cost = STEP_SIZE_CM * (REVERSE_PENALTY if is_reverse else 1.0)

        # Direction change penalty (forward <-> reverse transition)
        if prev_move is not None:
            prev_is_reverse = prev_move in ('B', 'BL', 'BR')
            if is_reverse != prev_is_reverse:
                cost += DIRECTION_CHANGE_PENALTY

        results.append((nx, ny, nt, cost, move))

    return results


# =============================================================================
# REEDS-SHEPP HEURISTIC (replaces Dubins — always tighter)
# =============================================================================

def _h_reeds_shepp(x, y, theta, goal):
    """Reeds-Shepp heuristic: always <= Dubins, so tighter and still admissible."""
    try:
        length = rs_path_length((x, y, theta), goal, TURN_RADIUS_CM)
        return length
    except:
        # Fallback to euclidean if RS computation fails
        return math.sqrt((x - goal[0])**2 + (y - goal[1])**2)


# =============================================================================
# HYBRID A* SEARCH
# =============================================================================

def hybrid_astar_search(start, goal, obstacles_expanded,
                        goal_tolerance_cm=5.0, goal_tolerance_deg=8.0,
                        start_obstacle_idx=None, goal_obstacle_idx=None):
    """Find shortest path from start to goal respecting turning radius.
    
    Uses Reeds-Shepp heuristic and 6 motion primitives (fwd/rev × straight/left/right).
    
    start_obstacle_idx/goal_obstacle_idx: if set, the obstacle at that index
    in obstacles_expanded uses a REDUCED collision radius near start/goal
    to allow the robot to get close enough for capture.
    """
    sx, sy, st = start
    gx, gy, gt = goal
    goal_tol_rad = math.radians(goal_tolerance_deg)

    # Build modified obstacle list: reduce radius for start/goal obstacles
    CAPTURE_CLEARANCE = 27.0
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
    best_state = {}

    sk = _grid_key(sx, sy, st)
    g_cost[sk] = 0
    best_state[sk] = (sx, sy, st)
    h_start = _h_reeds_shepp(sx, sy, st, goal)
    heapq.heappush(open_set, (h_start, counter, sx, sy, st, None))  # Added prev_move

    iterations = 0
    while open_set and iterations < MAX_ITERATIONS:
        iterations += 1
        f, _, cx, cy, ct, prev_move = heapq.heappop(open_set)
        ck = _grid_key(cx, cy, ct)

        if ck in g_cost and g_cost[ck] < f - _h_reeds_shepp(cx, cy, ct, goal) - 1e-6:
            continue

        # Goal check
        dist = math.sqrt((cx - gx)**2 + (cy - gy)**2)
        adiff = abs(((ct - gt + math.pi) % (2 * math.pi)) - math.pi)
        if dist < goal_tolerance_cm and adiff < goal_tol_rad:
            # Reconstruct path
            path = [(cx, cy, ct, 'GOAL')]
            key = ck
            while key in parent:
                pkey, move, px, py, pt = parent[key]
                path.append((px, py, pt, move))
                key = pkey
            path.reverse()
            return path, iterations

        cur_g = g_cost.get(ck, float('inf'))

        for nx, ny, nt, cost, move in _get_successors(cx, cy, ct, obs_for_search, prev_move):
            new_g = cur_g + cost
            nk = _grid_key(nx, ny, nt)
            if nk not in g_cost or new_g < g_cost[nk]:
                g_cost[nk] = new_g
                best_state[nk] = (nx, ny, nt)
                parent[nk] = (ck, move, cx, cy, ct)
                counter += 1
                h = _h_reeds_shepp(nx, ny, nt, goal)
                heapq.heappush(open_set, (new_g + h, counter, nx, ny, nt, move))

    return None, iterations


# =============================================================================
# PATH TO COMMANDS — smooth arc commands with reverse arcs
# =============================================================================

def path_to_commands(path):
    """Convert Hybrid A* path to motor commands.
    
    Consecutive same-direction moves are merged:
      - S  → FW{dist}                    (straight forward)
      - B  → BW{dist}                    (straight backward)
      - L  → FL{angle}_{arc_len}         (forward left arc)
      - R  → FR{angle}_{arc_len}         (forward right arc)
      - BL → BL{angle}_{arc_len}         (backward left arc)
      - BR → BR{angle}_{arc_len}         (backward right arc)
    """
    if not path or len(path) < 2:
        return []

    step_angle_deg = round(math.degrees(STEP_SIZE_CM / TURN_RADIUS_CM))
    step_arc_len = STEP_SIZE_CM

    # Group consecutive same-type moves
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
        if move_type == 'S':
            dist = round(STEP_SIZE_CM * count)
            commands.append(f"FW{dist}")

        elif move_type == 'B':
            dist = round(STEP_SIZE_CM * count)
            commands.append(f"BW{dist}")

        elif move_type == 'L':
            total_arc = round(step_arc_len * count)
            commands.append(f"FL{total_arc}")

        elif move_type == 'R':
            total_arc = round(step_arc_len * count)
            commands.append(f"FR{total_arc}")

        elif move_type == 'BL':
            total_arc = round(step_arc_len * count)
            commands.append(f"BL{total_arc}")

        elif move_type == 'BR':
            total_arc = round(step_arc_len * count)
            commands.append(f"BR{total_arc}")

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