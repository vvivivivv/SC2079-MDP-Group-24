"""
hybrid_astar.py — backward compatibility stub.

All pathfinding is now handled directly by algo.py using v1's grid-based A*.
This module exists only so imports from other files don't break.
"""

OBSTACLE_RADIUS_CM = 30


def build_obstacle_list(obstacles):
    """Convert obstacle dicts to (cx_cm, cy_cm, radius_cm) tuples."""
    expanded = []
    for ob in obstacles:
        cx = ob['x'] * 10 + 5
        cy = ob['y'] * 10 + 5
        expanded.append((cx, cy, OBSTACLE_RADIUS_CM))
    return expanded