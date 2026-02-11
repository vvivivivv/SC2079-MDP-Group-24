"""
Hybrid A* pathfinding with Dubins path heuristic.

State: continuous (x, y, theta)
Motion primitives: STRAIGHT, LEFT_ARC, RIGHT_ARC, REVERSE
Heuristic: Dubins path length (admissible, tight)
Collision: checked along actual arc paths, not just endpoints

Commands output as arcs for smooth execution.
"""

import math
import heapq
from dubins import compute_dubins_path


# =============================================================================
# PARAMETERS
# =============================================================================
TURN_RADIUS_CM = 25.0        # Minimum turning radius
STEP_SIZE_CM = 10.0           # Distance per step
ARENA_SIZE_CM = 200.0         # Arena size (20x20 grid x 10cm)
ROBOT_CLEARANCE_CM = 15.0     # Robot half-diagonal (~14.1cm) + margin

# Discretization
CELL_SIZE_CM = 5.0            # Spatial buckets
N_THETA_BUCKETS = 72          # Angular buckets (5° each)
THETA_RES = 2 * math.pi / N_THETA_BUCKETS

# Search
MAX_ITERATIONS = 200000
REVERSE_PENALTY = 2.0         # Discourages excessive reversing

# Obstacle clearance: robot center must stay this far from obstacle center.
# The grid system uses Chebyshev distance < 2 cells (20cm) for blocking.
# We add a safety buffer for the arc bulge and discretization error.
OBSTACLE_RADIUS_CM = 28.0     # 21.2cm(robot corner) + 5cm(obstacle half) + 1.8cm(safety)

# Arc collision check resolution
ARC_CHECK_POINTS = 8          # Number of points sampled along each arc


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
        if _point_hits_obstacle(px, py, obstacles):
            return True
    return False

def _check_arc_collision(x0, y0, theta0, arc_angle, turn_radius, direction, obstacles):
    """Check along the actual circular arc path (not the chord).

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
# MOTION PRIMITIVES
# =============================================================================

# =============================================================================
# PRECOMPUTED EULER DELTAS — matches simulator's differential drive exactly
# =============================================================================
# The simulator uses Euler integration at 60fps with differential wheel speeds.
# To ensure the planned path matches executed commands, we precompute the
# (dx, dy, dtheta) for each primitive using the same integration.

def _precompute_euler_deltas():
    """Precompute movement deltas at heading=0 (EAST) using simulator's Euler integration."""
    R = TURN_RADIUS_CM
    axle = 16.0  # ROBOT_AXLE_TRACK from simulator
    half = axle / 2.0
    speed = 30.0  # CURRENT_SPEED from simulator
    dt = 1.0 / 60.0  # FPS from simulator
    step = STEP_SIZE_CM

    deltas = {}
    for name, vl, vr in [
        ('S', speed, speed),
        ('L', speed * (R - half) / R, speed * (R + half) / R),
        ('R', speed * (R + half) / R, speed * (R - half) / R),
        ('B', -speed, -speed),
    ]:
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


def _get_successors(x, y, theta, obstacles):
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

        # Collision check
        if move in ('S', 'B'):
            if _check_line_collision(x, y, nx, ny, obstacles):
                continue
        else:
            arc_angle = abs(dt)
            direction = 'L' if move == 'L' else 'R'
            if _check_arc_collision(x, y, theta, arc_angle, TURN_RADIUS_CM, direction, obstacles):
                continue

        cost = STEP_SIZE_CM * (REVERSE_PENALTY if move == 'B' else 1.0)
        results.append((nx, ny, nt, cost, move))

    return results


# =============================================================================
# DUBINS HEURISTIC
# =============================================================================

def _h_dubins(x, y, theta, goal):
    result = compute_dubins_path((x, y, theta), goal, TURN_RADIUS_CM)
    if result:
        _, length = result
        return length
    return math.sqrt((x - goal[0])**2 + (y - goal[1])**2)


# =============================================================================
# HYBRID A* SEARCH
# =============================================================================

def hybrid_astar_search(start, goal, obstacles_expanded,
                        goal_tolerance_cm=5.0, goal_tolerance_deg=8.0,
                        start_obstacle_idx=None, goal_obstacle_idx=None):
    """Find shortest path from start to goal respecting turning radius.

    start_obstacle_idx/goal_obstacle_idx: if set, the obstacle at that index
    in obstacles_expanded uses a REDUCED collision radius near start/goal
    to allow the robot to get close enough for capture.
    """
    sx, sy, st = start
    gx, gy, gt = goal
    goal_tol_rad = math.radians(goal_tolerance_deg)

    # Build modified obstacle list: reduce radius for start/goal obstacles
    # Physical minimum clearance = robot_half(15) + obstacle_half(5) = 20cm
    CAPTURE_CLEARANCE = 27.0   # robot corner(21.2) + obstacle half(5) + margin(0.8)
    obs_for_search = []
    for i, (ox, oy, orad) in enumerate(obstacles_expanded):
        if i == start_obstacle_idx or i == goal_obstacle_idx:
            # Reduced radius ONLY for the specific obstacle being captured
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
    heapq.heappush(open_set, (_h_dubins(sx, sy, st, goal), counter, sx, sy, st))

    iterations = 0
    while open_set and iterations < MAX_ITERATIONS:
        iterations += 1
        f, _, cx, cy, ct = heapq.heappop(open_set)
        ck = _grid_key(cx, cy, ct)

        if ck in g_cost and g_cost[ck] < f - _h_dubins(cx, cy, ct, goal) - 1e-6:
            continue

        dist = math.sqrt((cx - gx)**2 + (cy - gy)**2)
        adiff = abs(((ct - gt + math.pi) % (2 * math.pi)) - math.pi)
        if dist < goal_tolerance_cm and adiff < goal_tol_rad:
            # Reconstruct path using continuous states stored in parent chain
            path = [(cx, cy, ct, 'GOAL')]
            key = ck
            while key in parent:
                pkey, move, px, py, pt = parent[key]
                path.append((px, py, pt, move))
                key = pkey
            path.reverse()
            return path, iterations

        cur_g = g_cost.get(ck, float('inf'))

        # Always use obs_for_search which has:
        # - Reduced radius (20cm) for the specific start/goal obstacle
        # - Full radius (25cm) for all other obstacles
        for nx, ny, nt, cost, move in _get_successors(cx, cy, ct, obs_for_search):
            new_g = cur_g + cost
            nk = _grid_key(nx, ny, nt)
            if nk not in g_cost or new_g < g_cost[nk]:
                g_cost[nk] = new_g
                best_state[nk] = (nx, ny, nt)
                # Store full continuous state of CURRENT node (the parent)
                parent[nk] = (ck, move, cx, cy, ct)
                counter += 1
                heapq.heappush(open_set, (new_g + _h_dubins(nx, ny, nt, goal), counter, nx, ny, nt))

    return None, iterations


# =============================================================================
# PATH TO COMMANDS — smooth arc commands
# =============================================================================

def path_to_commands(path):
    """Convert Hybrid A* path to motor commands.
    
    Consecutive same-direction moves are merged:
      - Straights (S) merge into longer FW commands
      - Reverses (B) merge into longer BW commands  
      - Left arcs (L) merge into larger FL arcs
      - Right arcs (R) merge into larger FR arcs
    
    Arc distances use arc length (= R × angle), which matches
    the simulator's center-of-robot distance tracking.
    """
    if not path or len(path) < 2:
        return []

    step_angle_deg = round(math.degrees(STEP_SIZE_CM / TURN_RADIUS_CM))
    step_arc_len = STEP_SIZE_CM  # arc length per step = R * (step/R) = step

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

        elif move_type == 'L':
            total_deg = round(step_angle_deg * count)
            total_arc = round(step_arc_len * count)
            commands.append(f"FL{total_deg:03d}_{total_arc:03d}")

        elif move_type == 'R':
            total_deg = round(step_angle_deg * count)
            total_arc = round(step_arc_len * count)
            commands.append(f"FR{total_deg:03d}_{total_arc:03d}")

        elif move_type == 'B':
            dist = round(STEP_SIZE_CM * count)
            commands.append(f"BW{dist}")

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