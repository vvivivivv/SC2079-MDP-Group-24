"""
Reeds-Shepp curve path planning module.

Replaces dubins.py with full forward+reverse path planning.
Computes shortest smooth paths between oriented waypoints using:
- Straight lines (S) forward or reverse
- Left circular arcs (L) forward or reverse
- Right circular arcs (R) forward or reverse

Reeds-Shepp paths are ALWAYS <= Dubins path length because they
can freely reverse. This gives dramatically shorter paths in tight
spaces (e.g., 3-point turns instead of wide U-loops).

Based on: Reeds, J.A. and Shepp, L.A., 1990.
"Optimal paths for a car that goes both forwards and backwards."
Pacific Journal of Mathematics, 145(2), pp.367-393.

12 path families x 4 symmetry variants = 48 candidate paths.
"""

import math
from dataclasses import dataclass, replace
from typing import List, Tuple, Optional


# =============================================================================
# Utility functions (inlined from former's utils module)
# =============================================================================

def _M(angle):
    """Normalize angle to [0, 2*pi)."""
    return angle % (2 * math.pi)


def _R(x, y):
    """Convert cartesian to polar coordinates."""
    r = math.sqrt(x * x + y * y)
    theta = math.atan2(y, x)
    return r, theta


def _normalize(angle):
    """Normalize angle — same as _M, used by path functions."""
    return angle % (2 * math.pi)


def _change_of_basis(p1, p2):
    """Transform p2 into p1's local coordinate frame.
    
    Args:
        p1: (x, y, theta) reference pose
        p2: (x, y, theta) target pose
    
    Returns:
        (x, y, theta) of p2 in p1's frame
    """
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    cos_t = math.cos(p1[2])
    sin_t = math.sin(p1[2])
    new_x = dx * cos_t + dy * sin_t
    new_y = -dx * sin_t + dy * cos_t
    new_theta = p2[2] - p1[2]
    return new_x, new_y, new_theta


# =============================================================================
# Path element (segment descriptor)
# =============================================================================

# Gear constants
GEAR_FORWARD = 1
GEAR_REVERSE = -1

# Steering constants
STEER_LEFT = -1
STEER_STRAIGHT = 0
STEER_RIGHT = 1


@dataclass(eq=True)
class PathElement:
    """A single segment of a Reeds-Shepp path."""
    param: float        # arc length in normalized units (multiply by radius for cm)
    steering: int       # STEER_LEFT, STEER_STRAIGHT, STEER_RIGHT
    gear: int           # GEAR_FORWARD, GEAR_REVERSE

    @classmethod
    def create(cls, param, steering, gear):
        """Create with automatic gear flip for negative params."""
        if param >= 0:
            return cls(param, steering, gear)
        else:
            return cls(-param, steering, -gear)

    def reverse_gear(self):
        return replace(self, gear=-self.gear)

    def reverse_steering(self):
        return replace(self, steering=-self.steering)

    def __repr__(self):
        s_name = {-1: 'LEFT', 0: 'STRAIGHT', 1: 'RIGHT'}[self.steering]
        g_name = {1: 'FWD', -1: 'REV'}[self.gear]
        return f"{{steer:{s_name} gear:{g_name} dist:{self.param:.2f}}}"


# =============================================================================
# Core functions
# =============================================================================

def path_length(path):
    """Total length of a Reeds-Shepp path in normalized units."""
    return sum(e.param for e in path)


def get_optimal_path_length(start, end, radius):
    """Get shortest Reeds-Shepp path length between two poses.
    
    Args:
        start: (x_cm, y_cm, theta_rad)
        end: (x_cm, y_cm, theta_rad)
        radius: minimum turning radius in cm
    
    Returns:
        Shortest path length in cm
    """
    # Normalize positions by radius
    x_0 = start[0] / radius
    y_0 = start[1] / radius
    x_f = end[0] / radius
    y_f = end[1] / radius

    opt_path = get_optimal_path(
        (x_0, y_0, start[2]),
        (x_f, y_f, end[2])
    )
    return radius * path_length(opt_path)


def get_optimal_path(start, end):
    """Return the shortest Reeds-Shepp path (in normalized coordinates)."""
    paths = get_all_paths(start, end)
    return min(paths, key=path_length)


def get_optimal_path_segments(start, end, radius):
    """Get shortest path as (segments, total_length) for command generation.
    
    Args:
        start: (x_cm, y_cm, theta_rad)
        end: (x_cm, y_cm, theta_rad)
        radius: minimum turning radius in cm
    
    Returns:
        (segments, total_length) where segments = [('L'|'R'|'S', length_cm, 'F'|'B'), ...]
        or None if no path found
    """
    x_0 = start[0] / radius
    y_0 = start[1] / radius
    x_f = end[0] / radius
    y_f = end[1] / radius

    opt_path = get_optimal_path(
        (x_0, y_0, start[2]),
        (x_f, y_f, end[2])
    )

    segments = []
    for elem in opt_path:
        length_cm = elem.param * radius
        if length_cm < 0.5:
            continue

        if elem.steering == STEER_LEFT:
            seg_type = 'L'
        elif elem.steering == STEER_RIGHT:
            seg_type = 'R'
        else:
            seg_type = 'S'

        gear = 'F' if elem.gear == GEAR_FORWARD else 'B'
        segments.append((seg_type, length_cm, gear))

    total_length = radius * path_length(opt_path)
    return segments, total_length


def get_all_paths(start, end):
    """Generate all candidate Reeds-Shepp paths between two poses.
    
    Evaluates 12 path families x 4 symmetry variants = 48 candidates.
    """
    path_fns = [
        _path1, _path2, _path3, _path4, _path5, _path6,
        _path7, _path8, _path9, _path10, _path11, _path12
    ]
    paths = []

    # Transform end into start's frame
    x, y, theta = _change_of_basis(start, end)

    for get_path in path_fns:
        # Four symmetry variants for each path type
        paths.append(get_path(x, y, theta))
        paths.append(_timeflip(get_path(-x, y, -theta)))
        paths.append(_reflect(get_path(x, -y, -theta)))
        paths.append(_reflect(_timeflip(get_path(-x, -y, theta))))

    # Remove zero-length elements
    for i in range(len(paths)):
        paths[i] = [e for e in paths[i] if e.param != 0]

    # Remove empty paths
    paths = [p for p in paths if p]

    return paths


def _timeflip(path):
    """Timeflip transform: reverse all gears."""
    return [e.reverse_gear() for e in path]


def _reflect(path):
    """Reflect transform: reverse all steering."""
    return [e.reverse_steering() for e in path]


# =============================================================================
# 12 Reeds-Shepp path families (from the 1990 paper)
# =============================================================================

def _path1(x, y, phi):
    """Formula 8.1: CSC (same turns) — L+S+L+"""
    phi = _normalize(phi)
    path = []
    u, t = _R(x - math.sin(phi), y - 1 + math.cos(phi))
    v = _M(phi - t)
    path.append(PathElement.create(t, STEER_LEFT, GEAR_FORWARD))
    path.append(PathElement.create(u, STEER_STRAIGHT, GEAR_FORWARD))
    path.append(PathElement.create(v, STEER_LEFT, GEAR_FORWARD))
    return path


def _path2(x, y, phi):
    """Formula 8.2: CSC (opposite turns) — L+S+R+"""
    phi = _M(_normalize(phi))
    path = []
    rho, t1 = _R(x + math.sin(phi), y - 1 - math.cos(phi))
    if rho * rho >= 4:
        u = math.sqrt(rho * rho - 4)
        t = _M(t1 + math.atan2(2, u))
        v = _M(t - phi)
        path.append(PathElement.create(t, STEER_LEFT, GEAR_FORWARD))
        path.append(PathElement.create(u, STEER_STRAIGHT, GEAR_FORWARD))
        path.append(PathElement.create(v, STEER_RIGHT, GEAR_FORWARD))
    return path


def _path3(x, y, phi):
    """Formula 8.3: C|C|C"""
    phi = _normalize(phi)
    path = []
    xi = x - math.sin(phi)
    eta = y - 1 + math.cos(phi)
    rho, theta = _R(xi, eta)
    if rho <= 4:
        A = math.acos(rho / 4)
        t = _M(theta + math.pi / 2 + A)
        u = _M(math.pi - 2 * A)
        v = _M(phi - t - u)
        path.append(PathElement.create(t, STEER_LEFT, GEAR_FORWARD))
        path.append(PathElement.create(u, STEER_RIGHT, GEAR_REVERSE))
        path.append(PathElement.create(v, STEER_LEFT, GEAR_FORWARD))
    return path


def _path4(x, y, phi):
    """Formula 8.4 (1): C|CC"""
    phi = _normalize(phi)
    path = []
    xi = x - math.sin(phi)
    eta = y - 1 + math.cos(phi)
    rho, theta = _R(xi, eta)
    if rho <= 4:
        A = math.acos(rho / 4)
        t = _M(theta + math.pi / 2 + A)
        u = _M(math.pi - 2 * A)
        v = _M(t + u - phi)
        path.append(PathElement.create(t, STEER_LEFT, GEAR_FORWARD))
        path.append(PathElement.create(u, STEER_RIGHT, GEAR_REVERSE))
        path.append(PathElement.create(v, STEER_LEFT, GEAR_REVERSE))
    return path


def _path5(x, y, phi):
    """Formula 8.4 (2): CC|C"""
    phi = _normalize(phi)
    path = []
    xi = x - math.sin(phi)
    eta = y - 1 + math.cos(phi)
    rho, theta = _R(xi, eta)
    if rho <= 4:
        u = math.acos(1 - rho * rho / 8)
        A = math.asin(2 * math.sin(u) / rho)
        t = _M(theta + math.pi / 2 - A)
        v = _M(t - u - phi)
        path.append(PathElement.create(t, STEER_LEFT, GEAR_FORWARD))
        path.append(PathElement.create(u, STEER_RIGHT, GEAR_FORWARD))
        path.append(PathElement.create(v, STEER_LEFT, GEAR_REVERSE))
    return path


def _path6(x, y, phi):
    """Formula 8.7: CCu|CuC"""
    phi = _normalize(phi)
    path = []
    xi = x + math.sin(phi)
    eta = y - 1 - math.cos(phi)
    rho, theta = _R(xi, eta)
    if rho <= 4:
        if rho <= 2:
            A = math.acos((rho + 2) / 4)
            t = _M(theta + math.pi / 2 + A)
            u = _M(A)
            v = _M(phi - t + 2 * u)
        else:
            A = math.acos((rho - 2) / 4)
            t = _M(theta + math.pi / 2 - A)
            u = _M(math.pi - A)
            v = _M(phi - t + 2 * u)
        path.append(PathElement.create(t, STEER_LEFT, GEAR_FORWARD))
        path.append(PathElement.create(u, STEER_RIGHT, GEAR_FORWARD))
        path.append(PathElement.create(u, STEER_LEFT, GEAR_REVERSE))
        path.append(PathElement.create(v, STEER_RIGHT, GEAR_REVERSE))
    return path


def _path7(x, y, phi):
    """Formula 8.8: C|CuCu|C"""
    phi = _normalize(phi)
    path = []
    xi = x + math.sin(phi)
    eta = y - 1 - math.cos(phi)
    rho, theta = _R(xi, eta)
    u1 = (20 - rho * rho) / 16
    if rho <= 6 and 0 <= u1 <= 1:
        u = math.acos(u1)
        A = math.asin(2 * math.sin(u) / rho)
        t = _M(theta + math.pi / 2 + A)
        v = _M(t - phi)
        path.append(PathElement.create(t, STEER_LEFT, GEAR_FORWARD))
        path.append(PathElement.create(u, STEER_RIGHT, GEAR_REVERSE))
        path.append(PathElement.create(u, STEER_LEFT, GEAR_REVERSE))
        path.append(PathElement.create(v, STEER_RIGHT, GEAR_FORWARD))
    return path


def _path8(x, y, phi):
    """Formula 8.9 (1): C|C[pi/2]SC"""
    phi = _normalize(phi)
    path = []
    xi = x - math.sin(phi)
    eta = y - 1 + math.cos(phi)
    rho, theta = _R(xi, eta)
    if rho >= 2:
        u = math.sqrt(rho * rho - 4) - 2
        A = math.atan2(2, u + 2)
        t = _M(theta + math.pi / 2 + A)
        v = _M(t - phi + math.pi / 2)
        path.append(PathElement.create(t, STEER_LEFT, GEAR_FORWARD))
        path.append(PathElement.create(math.pi / 2, STEER_RIGHT, GEAR_REVERSE))
        path.append(PathElement.create(u, STEER_STRAIGHT, GEAR_REVERSE))
        path.append(PathElement.create(v, STEER_LEFT, GEAR_REVERSE))
    return path


def _path9(x, y, phi):
    """Formula 8.9 (2): CSC[pi/2]|C"""
    phi = _normalize(phi)
    path = []
    xi = x - math.sin(phi)
    eta = y - 1 + math.cos(phi)
    rho, theta = _R(xi, eta)
    if rho >= 2:
        u = math.sqrt(rho * rho - 4) - 2
        A = math.atan2(u + 2, 2)
        t = _M(theta + math.pi / 2 - A)
        v = _M(t - phi - math.pi / 2)
        path.append(PathElement.create(t, STEER_LEFT, GEAR_FORWARD))
        path.append(PathElement.create(u, STEER_STRAIGHT, GEAR_FORWARD))
        path.append(PathElement.create(math.pi / 2, STEER_RIGHT, GEAR_FORWARD))
        path.append(PathElement.create(v, STEER_LEFT, GEAR_REVERSE))
    return path


def _path10(x, y, phi):
    """Formula 8.10 (1): C|C[pi/2]SC"""
    phi = _normalize(phi)
    path = []
    xi = x + math.sin(phi)
    eta = y - 1 - math.cos(phi)
    rho, theta = _R(xi, eta)
    if rho >= 2:
        t = _M(theta + math.pi / 2)
        u = rho - 2
        v = _M(phi - t - math.pi / 2)
        path.append(PathElement.create(t, STEER_LEFT, GEAR_FORWARD))
        path.append(PathElement.create(math.pi / 2, STEER_RIGHT, GEAR_REVERSE))
        path.append(PathElement.create(u, STEER_STRAIGHT, GEAR_REVERSE))
        path.append(PathElement.create(v, STEER_RIGHT, GEAR_REVERSE))
    return path


def _path11(x, y, phi):
    """Formula 8.10 (2): CSC[pi/2]|C"""
    phi = _normalize(phi)
    path = []
    xi = x + math.sin(phi)
    eta = y - 1 - math.cos(phi)
    rho, theta = _R(xi, eta)
    if rho >= 2:
        t = _M(theta)
        u = rho - 2
        v = _M(phi - t - math.pi / 2)
        path.append(PathElement.create(t, STEER_LEFT, GEAR_FORWARD))
        path.append(PathElement.create(u, STEER_STRAIGHT, GEAR_FORWARD))
        path.append(PathElement.create(math.pi / 2, STEER_LEFT, GEAR_FORWARD))
        path.append(PathElement.create(v, STEER_RIGHT, GEAR_REVERSE))
    return path


def _path12(x, y, phi):
    """Formula 8.11: C|C[pi/2]SC[pi/2]|C"""
    phi = _normalize(phi)
    path = []
    xi = x + math.sin(phi)
    eta = y - 1 - math.cos(phi)
    rho, theta = _R(xi, eta)
    if rho >= 4:
        u = math.sqrt(rho * rho - 4) - 4
        A = math.atan2(2, u + 4)
        t = _M(theta + math.pi / 2 + A)
        v = _M(t - phi)
        path.append(PathElement.create(t, STEER_LEFT, GEAR_FORWARD))
        path.append(PathElement.create(math.pi / 2, STEER_RIGHT, GEAR_REVERSE))
        path.append(PathElement.create(u, STEER_STRAIGHT, GEAR_REVERSE))
        path.append(PathElement.create(math.pi / 2, STEER_LEFT, GEAR_REVERSE))
        path.append(PathElement.create(v, STEER_RIGHT, GEAR_FORWARD))
    return path


# =============================================================================
# Path sampling for visualization and collision checking
# =============================================================================

def sample_path(start, radius, end=None, segments=None, num_points=50):
    """Sample points along a Reeds-Shepp path for visualization/collision checking.
    
    Args:
        start: (x_cm, y_cm, theta_rad) start pose
        radius: turning radius in cm
        end: (x_cm, y_cm, theta_rad) end pose (used if segments not provided)
        segments: list of (type, length_cm, gear) from get_optimal_path_segments
        num_points: approximate number of sample points
    
    Returns:
        List of (x, y) points in cm
    """
    if segments is None:
        if end is None:
            return [(start[0], start[1])]
        result = get_optimal_path_segments(start, end, radius)
        if result is None:
            return [(start[0], start[1]), (end[0], end[1])]
        segments, _ = result

    x, y, theta = start
    points = [(x, y)]
    R = radius

    for seg_type, length, gear in segments:
        if length < 1e-6:
            continue

        gear_sign = 1.0 if gear == 'F' else -1.0
        n_samples = max(2, int(length / 3.0))  # sample every ~3cm
        ds = length / n_samples

        for _ in range(n_samples):
            if seg_type == 'S':
                x += gear_sign * ds * math.cos(theta)
                y += gear_sign * ds * math.sin(theta)
            elif seg_type == 'L':
                dtheta = gear_sign * ds / R
                cx = x - R * math.sin(theta)
                cy = y + R * math.cos(theta)
                theta += dtheta
                x = cx + R * math.sin(theta)
                y = cy - R * math.cos(theta)
            elif seg_type == 'R':
                dtheta = gear_sign * ds / R
                cx = x + R * math.sin(theta)
                cy = y - R * math.cos(theta)
                theta -= dtheta
                x = cx - R * math.sin(theta)
                y = cy + R * math.cos(theta)
            points.append((x, y))

    return points