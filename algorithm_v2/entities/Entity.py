import math
from typing import List
from consts import (
    Direction, EXPANDED_CELL, SCREENSHOT_COST,
    CAMERA_HALF_FOV_RAD, CAMERA_HALF_FOV_DEG, CAMERA_MAX_DIST_CELLS,
    FOV_PENALTY_TINY_TURN, FOV_PENALTY_SMALL_TURN, FOV_PENALTY_MEDIUM_TURN,
)
from helper import is_valid

# Maximum incidence angle (degrees) between camera direction and face normal.
# 0° = head-on, 90° = looking at the edge. 60° is generous but ensures readability.
MAX_INCIDENCE_DEG = 60.0
MAX_INCIDENCE_RAD = math.radians(MAX_INCIDENCE_DEG)

# Maximum micro-turn angle (degrees) the robot can do in place.
# Beyond this, a full 90-degree grid turn is more efficient.
# This is NOT the camera FOV — the robot physically rotates to face the target.
MAX_MICRO_TURN_DEG = 80.0
MAX_MICRO_TURN_RAD = math.radians(MAX_MICRO_TURN_DEG)


class CellState:
    """Base class for all objects on the arena"""

    def __init__(self, x, y, direction: Direction = Direction.NORTH, screenshot_id=-1, penalty=0):
        self.x = x
        self.y = y
        self.direction = direction
        self.screenshot_id = screenshot_id
        self.penalty = penalty

    def cmp_position(self, x, y) -> bool:
        return self.x == x and self.y == y

    def is_eq(self, x, y, direction):
        return self.x == x and self.y == y and self.direction == direction

    def __repr__(self):
        return "x: {}, y: {}, d: {}, screenshot: {}".format(self.x, self.y, self.direction, self.screenshot_id)

    def set_screenshot(self, screenshot_id):
        self.screenshot_id = screenshot_id

    def get_dict(self):
        return {'x': self.x, 'y': self.y, 'd': self.direction, 's': self.screenshot_id}


class Obstacle(CellState):
    """Obstacle class, inherited from CellState"""

    def __init__(self, x: int, y: int, direction: Direction, obstacle_id: int):
        super().__init__(x, y, direction)
        self.obstacle_id = obstacle_id

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and self.direction == other.direction

    def _get_obstacle_face_center(self):
        """Get the (x, y) of the center of the obstacle's image face."""
        if self.direction == Direction.NORTH:
            return (float(self.x), self.y + 0.5 + EXPANDED_CELL * 0.5)
        elif self.direction == Direction.SOUTH:
            return (float(self.x), self.y - 0.5 - EXPANDED_CELL * 0.5)
        elif self.direction == Direction.EAST:
            return (self.x + 0.5 + EXPANDED_CELL * 0.5, float(self.y))
        elif self.direction == Direction.WEST:
            return (self.x - 0.5 - EXPANDED_CELL * 0.5, float(self.y))
        return (float(self.x), float(self.y))

    @staticmethod
    def _compute_micro_turn(robot_x, robot_y, robot_dir, face_x, face_y, obs_dir):
        """Compute micro-turn angle from robot heading to obstacle face.

        The robot can turn in place by any angle up to MAX_MICRO_TURN_DEG.
        This is NOT limited by the camera FOV — the robot physically rotates
        to point the camera at the face, then the face will be at the center
        of the camera's view.

        Checks:
        1. Distance: face must be within camera range
        2. Face orientation: robot must be in front of the face (not behind it)
        3. Incidence angle: camera must look AT the face surface, not along the edge
        4. Turn magnitude: micro-turn must be ≤ MAX_MICRO_TURN_DEG (else use grid turn)

        Returns:
            (bool, float): (can_see, signed_turn_degrees)
                positive = turn left, negative = turn right
        """
        dx = face_x - robot_x
        dy = face_y - robot_y
        dist = math.sqrt(dx * dx + dy * dy)

        # CHECK 1: Distance
        if dist < 1.0 or dist > CAMERA_MAX_DIST_CELLS + 0.5:
            return False, 0.0

        # Angle from robot to face center (math convention)
        angle_to_face = math.atan2(dy, dx)
        robot_angle = Direction.to_angle_rad(robot_dir)

        # Signed angular difference: how much the robot needs to turn
        diff = angle_to_face - robot_angle
        diff = (diff + math.pi) % (2 * math.pi) - math.pi

        # CHECK 4: Micro-turn magnitude limit
        if abs(diff) > MAX_MICRO_TURN_RAD:
            return False, 0.0

        # CHECK 2: Face orientation — robot must be in front of the face
        face_normal_angle = Direction.to_angle_rad(obs_dir)
        to_robot_angle = math.atan2(robot_y - face_y, robot_x - face_x)
        face_diff = to_robot_angle - face_normal_angle
        face_diff = (face_diff + math.pi) % (2 * math.pi) - math.pi
        if abs(face_diff) > math.pi / 2:
            return False, 0.0

        # CHECK 3: Incidence angle — camera must look AT the face, not along the edge
        # After the micro-turn, the camera points at angle_to_face.
        # The face normal is face_normal_angle.
        # For the face to be readable, these should be roughly opposite.
        angle_between = angle_to_face - face_normal_angle
        angle_between = (angle_between + math.pi) % (2 * math.pi) - math.pi
        incidence = abs(abs(angle_between) - math.pi)

        if incidence > MAX_INCIDENCE_RAD:
            return False, 0.0

        return True, math.degrees(diff)

    def get_view_state(self, retrying) -> List['CellState']:
        """Generate FOV-based capture positions.

        For each nearby grid cell, tests all 4 cardinal robot directions.
        The robot can micro-turn up to ±80° to point at the face.
        Penalties scale with turn magnitude.

        Returns:
            List[CellState]: Valid capture positions sorted by penalty.
        """
        cells = []
        face_x, face_y = self._get_obstacle_face_center()
        max_dist = CAMERA_MAX_DIST_CELLS + (1 if retrying else 0)
        search_range = max_dist + 1

        for rx in range(self.x - search_range, self.x + search_range + 1):
            for ry in range(self.y - search_range, self.y + search_range + 1):
                if not is_valid(rx, ry):
                    continue

                for robot_dir in [Direction.NORTH, Direction.EAST, Direction.SOUTH, Direction.WEST]:
                    can_see, turn_deg = self._compute_micro_turn(
                        rx, ry, robot_dir, face_x, face_y, self.direction
                    )
                    if not can_see:
                        continue

                    abs_turn = abs(turn_deg)
                    if abs_turn < 10:
                        penalty = FOV_PENALTY_TINY_TURN
                    elif abs_turn < 20:
                        penalty = FOV_PENALTY_SMALL_TURN
                    else:
                        penalty = FOV_PENALTY_MEDIUM_TURN

                    cells.append(CellState(
                        rx, ry, robot_dir, self.obstacle_id, penalty
                    ))

        cells.sort(key=lambda c: c.penalty)

        MAX_CANDIDATES = 16
        if len(cells) > MAX_CANDIDATES:
            cells = cells[:MAX_CANDIDATES]

        if not cells:
            cells = self._get_legacy_view_state(retrying)

        return cells

    def _get_legacy_view_state(self, retrying) -> List['CellState']:
        """Original head-on positions as fallback."""
        cells = []
        if self.direction == Direction.NORTH:
            if not retrying:
                if is_valid(self.x, self.y + 1 + EXPANDED_CELL * 2):
                    cells.append(CellState(self.x, self.y + 1 + EXPANDED_CELL * 2, Direction.SOUTH, self.obstacle_id, 5))
                if is_valid(self.x, self.y + 2 + EXPANDED_CELL * 2):
                    cells.append(CellState(self.x, self.y + 2 + EXPANDED_CELL * 2, Direction.SOUTH, self.obstacle_id, 0))
            else:
                if is_valid(self.x, self.y + 2 + EXPANDED_CELL * 2):
                    cells.append(CellState(self.x, self.y + 2 + EXPANDED_CELL * 2, Direction.SOUTH, self.obstacle_id, 0))
                if is_valid(self.x, self.y + 3 + EXPANDED_CELL * 2):
                    cells.append(CellState(self.x, self.y + 3 + EXPANDED_CELL * 2, Direction.SOUTH, self.obstacle_id, 0))
        elif self.direction == Direction.SOUTH:
            if not retrying:
                if is_valid(self.x, self.y - 1 - EXPANDED_CELL * 2):
                    cells.append(CellState(self.x, self.y - 1 - EXPANDED_CELL * 2, Direction.NORTH, self.obstacle_id, 5))
                if is_valid(self.x, self.y - 2 - EXPANDED_CELL * 2):
                    cells.append(CellState(self.x, self.y - 2 - EXPANDED_CELL * 2, Direction.NORTH, self.obstacle_id, 0))
            else:
                if is_valid(self.x, self.y - 2 - EXPANDED_CELL * 2):
                    cells.append(CellState(self.x, self.y - 2 - EXPANDED_CELL * 2, Direction.NORTH, self.obstacle_id, 0))
                if is_valid(self.x, self.y - 3 - EXPANDED_CELL * 2):
                    cells.append(CellState(self.x, self.y - 3 - EXPANDED_CELL * 2, Direction.NORTH, self.obstacle_id, 0))
        elif self.direction == Direction.EAST:
            if not retrying:
                if is_valid(self.x + 1 + EXPANDED_CELL * 2, self.y):
                    cells.append(CellState(self.x + 1 + EXPANDED_CELL * 2, self.y, Direction.WEST, self.obstacle_id, 5))
                if is_valid(self.x + 2 + EXPANDED_CELL * 2, self.y):
                    cells.append(CellState(self.x + 2 + EXPANDED_CELL * 2, self.y, Direction.WEST, self.obstacle_id, 0))
            else:
                if is_valid(self.x + 2 + EXPANDED_CELL * 2, self.y):
                    cells.append(CellState(self.x + 2 + EXPANDED_CELL * 2, self.y, Direction.WEST, self.obstacle_id, 0))
                if is_valid(self.x + 3 + EXPANDED_CELL * 2, self.y):
                    cells.append(CellState(self.x + 3 + EXPANDED_CELL * 2, self.y, Direction.WEST, self.obstacle_id, 0))
        elif self.direction == Direction.WEST:
            if not retrying:
                if is_valid(self.x - 1 - EXPANDED_CELL * 2, self.y):
                    cells.append(CellState(self.x - 1 - EXPANDED_CELL * 2, self.y, Direction.EAST, self.obstacle_id, 5))
                if is_valid(self.x - 2 - EXPANDED_CELL * 2, self.y):
                    cells.append(CellState(self.x - 2 - EXPANDED_CELL * 2, self.y, Direction.EAST, self.obstacle_id, 0))
            else:
                if is_valid(self.x - 2 - EXPANDED_CELL * 2, self.y):
                    cells.append(CellState(self.x - 2 - EXPANDED_CELL * 2, self.y, Direction.EAST, self.obstacle_id, 0))
                if is_valid(self.x - 3 - EXPANDED_CELL * 2, self.y):
                    cells.append(CellState(self.x - 3 - EXPANDED_CELL * 2, self.y, Direction.EAST, self.obstacle_id, 0))
        return cells


class Grid:
    def __init__(self, size_x: int, size_y: int):
        self.size_x = size_x
        self.size_y = size_y
        self.obstacles: List[Obstacle] = []

    def add_obstacle(self, obstacle: Obstacle):
        to_add = True
        for ob in self.obstacles:
            if ob == obstacle:
                to_add = False
                break
        if to_add:
            self.obstacles.append(obstacle)

    def reset_obstacles(self):
        self.obstacles = []

    def get_obstacles(self):
        return self.obstacles

    def reachable(self, x: int, y: int, turn=False, preTurn=False) -> bool:
        if not self.is_valid_coord(x, y):
            return False
        for ob in self.obstacles:
            if ob.x == 4 and ob.y <= 4 and x < 4 and y < 4:
                continue
            if abs(ob.x - x) + abs(ob.y - y) >= 4:
                continue
            if turn:
                if max(abs(ob.x - x), abs(ob.y - y)) < EXPANDED_CELL * 2 + 1:
                    return False
            if preTurn:
                if max(abs(ob.x - x), abs(ob.y - y)) < EXPANDED_CELL * 2 + 1:
                    return False
            else:
                if max(abs(ob.x - x), abs(ob.y - y)) < 2:
                    return False
        return True

    def is_valid_coord(self, x: int, y: int) -> bool:
        if x < 1 or x >= self.size_x - 1 or y < 1 or y >= self.size_y - 1:
            return False
        return True

    def is_valid_cell_state(self, state: CellState) -> bool:
        return self.is_valid_coord(state.x, state.y)

    def get_view_obstacle_positions(self, retrying) -> List[List[CellState]]:
        optimal_positions = []
        for obstacle in self.obstacles:
            if obstacle.direction == 8:
                continue
            else:
                view_states = [vs for vs in obstacle.get_view_state(retrying)
                              if self.reachable(vs.x, vs.y)]
            optimal_positions.append(view_states)
        return optimal_positions