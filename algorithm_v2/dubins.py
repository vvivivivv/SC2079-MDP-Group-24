"""
Dubins curve path planning module.

Computes shortest smooth paths between oriented waypoints using only:
- Straight lines (S)
- Left circular arcs of minimum radius R (L)
- Right circular arcs of minimum radius R (R)

This replaces the grid-locked FR/FL/BR/BL commands with smooth curves
that respect the robot's actual minimum turning radius.
"""

import math
from typing import List, Tuple, Optional


# =============================================================================
# Dubins path computation (6 path types: LSL, RSR, LSR, RSL, RLR, LRL)
# =============================================================================

def _mod2pi(x):
    return x % (2 * math.pi)


def _LSL(alpha, beta, d):
    ca, sa = math.cos(alpha), math.sin(alpha)
    cb, sb = math.cos(beta), math.sin(beta)
    tmp = 2 + d*d - 2*(ca*cb + sa*sb - d*(sa - sb))
    if tmp < 0:
        return None
    p = math.sqrt(tmp)
    theta = math.atan2(cb - ca, d + sa - sb)
    t = _mod2pi(-alpha + theta)
    q = _mod2pi(beta - theta)
    return t, p, q


def _RSR(alpha, beta, d):
    ca, sa = math.cos(alpha), math.sin(alpha)
    cb, sb = math.cos(beta), math.sin(beta)
    tmp = 2 + d*d - 2*(ca*cb + sa*sb - d*(sb - sa))
    if tmp < 0:
        return None
    p = math.sqrt(tmp)
    theta = math.atan2(ca - cb, d - sa + sb)
    t = _mod2pi(alpha - theta)
    q = _mod2pi(-beta + theta)
    return t, p, q


def _LSR(alpha, beta, d):
    ca, sa = math.cos(alpha), math.sin(alpha)
    cb, sb = math.cos(beta), math.sin(beta)
    tmp = -2 + d*d + 2*(ca*cb + sa*sb - d*(sa + sb))
    if tmp < 0:
        return None
    p = math.sqrt(tmp)
    theta = math.atan2(-ca - cb, d - sa - sb) - math.atan2(-2, p)
    t = _mod2pi(-alpha + theta)
    q = _mod2pi(-_mod2pi(beta) + theta)
    return t, p, q


def _RSL(alpha, beta, d):
    ca, sa = math.cos(alpha), math.sin(alpha)
    cb, sb = math.cos(beta), math.sin(beta)
    tmp = -2 + d*d + 2*(ca*cb + sa*sb + d*(sa + sb))
    if tmp < 0:
        return None
    p = math.sqrt(tmp)
    theta = math.atan2(ca + cb, d + sa + sb) - math.atan2(2, p)
    t = _mod2pi(alpha - theta)
    q = _mod2pi(beta - theta)
    return t, p, q


def _RLR(alpha, beta, d):
    ca, sa = math.cos(alpha), math.sin(alpha)
    cb, sb = math.cos(beta), math.sin(beta)
    tmp = (6 - d*d + 2*(ca*cb + sa*sb) + 2*d*(sa - sb)) / 8
    if abs(tmp) > 1:
        return None
    p = _mod2pi(2*math.pi - math.acos(tmp))
    theta = math.atan2(ca - cb, d - sa + sb)
    t = _mod2pi(alpha - theta + _mod2pi(p/2))
    q = _mod2pi(alpha - beta - t + _mod2pi(p))
    return t, p, q


def _LRL(alpha, beta, d):
    ca, sa = math.cos(alpha), math.sin(alpha)
    cb, sb = math.cos(beta), math.sin(beta)
    tmp = (6 - d*d + 2*(ca*cb + sa*sb) - 2*d*(sa - sb)) / 8
    if abs(tmp) > 1:
        return None
    p = _mod2pi(2*math.pi - math.acos(tmp))
    theta = math.atan2(-ca + cb, d + sa - sb)
    t = _mod2pi(-alpha + theta + _mod2pi(p/2))
    q = _mod2pi(_mod2pi(beta) - alpha + _mod2pi(p) - t)
    return t, p, q


def compute_dubins_path(start, end, radius):
    """Compute shortest Dubins path between two oriented points.

    Args:
        start: (x, y, theta) in cm, theta in radians (math convention: 0=east, CCW+)
        end: (x, y, theta) in cm
        radius: minimum turning radius in cm

    Returns:
        (segments, total_length) or None if no path found.
        segments: [('L'|'R'|'S', length_cm), ...]
        total_length: total path length in cm
    """
    x0, y0, t0 = start
    x1, y1, t1 = end

    dx = x1 - x0
    dy = y1 - y0
    D = math.sqrt(dx*dx + dy*dy)

    if D < 1e-6:
        # Same position, just need to rotate
        angle_diff = (t1 - t0 + math.pi) % (2*math.pi) - math.pi
        if abs(angle_diff) < 1e-6:
            return [], 0
        if angle_diff > 0:
            return [('L', abs(angle_diff) * radius)], abs(angle_diff) * radius
        else:
            return [('R', abs(angle_diff) * radius)], abs(angle_diff) * radius

    d = D / radius
    theta = math.atan2(dy, dx)
    alpha = _mod2pi(t0 - theta)
    beta = _mod2pi(t1 - theta)

    candidates = []
    for name, func in [('LSL', _LSL), ('RSR', _RSR), ('LSR', _LSR),
                        ('RSL', _RSL), ('RLR', _RLR), ('LRL', _LRL)]:
        try:
            result = func(alpha, beta, d)
            if result is not None:
                t, p, q = result
                cost = abs(t) + abs(p) + abs(q)
                if cost >= 0:
                    candidates.append((name, t, p, q, cost))
        except:
            pass

    if not candidates:
        return None

    candidates.sort(key=lambda x: x[4])
    path_type, t, p, q, cost = candidates[0]

    segments = []
    for char, val in zip(path_type, [t, p, q]):
        length = abs(val) * radius
        if length < 0.5:  # skip tiny segments
            continue
        if char == 'L':
            segments.append(('L', length))
        elif char == 'R':
            segments.append(('R', length))
        elif char == 'S':
            segments.append(('S', length))

    return segments, cost * radius


def dubins_segments_to_commands(segments, radius, arc_step_deg=30):
    """Convert Dubins path segments to TL/TR/FW motor commands.

    Arcs are approximated as sequences of small turn + straight steps.
    With 15° steps and R=25cm, max deviation from true arc is ~0.2cm.

    Args:
        segments: [('L'|'R'|'S', length_cm), ...]
        radius: turning radius in cm
        arc_step_deg: angular step for arc approximation (default 15°)

    Returns:
        List of command strings: ['TL15', 'FW7', 'TL15', 'FW7', ...]
    """
    commands = []

    for seg_type, length in segments:
        if seg_type == 'S':
            dist_cm = round(length)
            if dist_cm > 0:
                commands.append(f'FW{dist_cm}')

        elif seg_type in ('L', 'R'):
            total_angle_deg = math.degrees(length / radius)
            turn_prefix = 'TL' if seg_type == 'L' else 'TR'

            if total_angle_deg < 3:
                # Tiny arc, skip
                continue

            if total_angle_deg < arc_step_deg:
                # Small arc: single turn + straight
                chord = 2 * radius * math.sin(math.radians(total_angle_deg / 2))
                commands.append(f'{turn_prefix}{round(total_angle_deg)}')
                if round(chord) > 0:
                    commands.append(f'FW{round(chord)}')
                continue

            # Break into equal steps
            n_steps = max(1, round(total_angle_deg / arc_step_deg))
            step_angle = total_angle_deg / n_steps
            chord = 2 * radius * math.sin(math.radians(step_angle / 2))
            step_int = round(step_angle)
            chord_int = round(chord)

            for _ in range(n_steps):
                if step_int > 0:
                    commands.append(f'{turn_prefix}{step_int}')
                if chord_int > 0:
                    commands.append(f'FW{chord_int}')

    return commands


def sample_dubins_path(start, segments, radius, num_points=50):
    """Sample points along a Dubins path for visualization.

    Returns list of (x, y) points in cm.
    """
    x, y, theta = start
    points = [(x, y)]

    for seg_type, length in segments:
        if seg_type == 'S':
            n = max(2, int(length / 2))
            for i in range(1, n + 1):
                t = length * i / n
                px = x + t * math.cos(theta)
                py = y + t * math.sin(theta)
                points.append((px, py))
            x = x + length * math.cos(theta)
            y = y + length * math.sin(theta)

        elif seg_type in ('L', 'R'):
            total_angle = length / radius
            sign = 1 if seg_type == 'L' else -1
            n = max(2, int(math.degrees(total_angle) / 3))

            # Center of turning circle
            cx = x - sign * radius * math.sin(theta)
            cy = y + sign * radius * math.cos(theta)

            start_angle = math.atan2(y - cy, x - cx)

            for i in range(1, n + 1):
                a = start_angle + sign * total_angle * i / n
                px = cx + radius * math.cos(a)
                py = cy + radius * math.sin(a)
                points.append((px, py))

            # Update state
            theta += sign * total_angle
            x = points[-1][0]
            y = points[-1][1]

    return points