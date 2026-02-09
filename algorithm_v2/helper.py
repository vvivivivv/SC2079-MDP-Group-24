import math
from consts import WIDTH, HEIGHT, Direction, CAMERA_HALF_FOV_RAD, EXPANDED_CELL


def is_valid(center_x: int, center_y: int):
    """Checks if given position is within bounds"""
    return center_x > 0 and center_y > 0 and center_x < WIDTH - 1 and center_y < HEIGHT - 1


def compute_micro_turn_angle(robot_x, robot_y, robot_dir, ob_x, ob_y, ob_dir):
    """Compute the exact micro-turn angle from robot heading to obstacle face.

    Returns:
        (float, float, str): (abs_angle_degrees, signed_degrees, signal)
            signed_degrees: positive = left, negative = right
            signal: 'L', 'C', or 'R'
    """
    face_offsets = {
        0: (0, 0.5 + EXPANDED_CELL * 0.5),
        2: (0.5 + EXPANDED_CELL * 0.5, 0),
        4: (0, -0.5 - EXPANDED_CELL * 0.5),
        6: (-0.5 - EXPANDED_CELL * 0.5, 0),
    }
    fx_off, fy_off = face_offsets.get(ob_dir, (0, 0))
    face_x = ob_x + fx_off
    face_y = ob_y + fy_off

    dx = face_x - robot_x
    dy = face_y - robot_y

    angle_to_face = math.atan2(dy, dx)
    robot_angle = Direction.to_angle_rad(robot_dir)

    diff_rad = angle_to_face - robot_angle
    diff_rad = (diff_rad + math.pi) % (2 * math.pi) - math.pi
    diff_deg = math.degrees(diff_rad)
    abs_angle = abs(diff_deg)

    if abs_angle < 3.0:
        signal = 'C'
    elif diff_deg > 0:
        signal = 'L'
    else:
        signal = 'R'

    return abs_angle, diff_deg, signal


def command_generator(states, obstacles):
    """Generate commands with micro-turn support (TL/TR commands).

    For captures, injects: TL/TR<angle> → SNAP<id>_<signal> → TR/TL<angle>
    """
    obstacles_dict = {ob['id']: ob for ob in obstacles}
    commands = []

    for i in range(1, len(states)):
        steps = "00"

        # Same direction: straight move
        if states[i].direction == states[i - 1].direction:
            if (states[i].x > states[i - 1].x and states[i].direction == Direction.EAST) or \
               (states[i].y > states[i - 1].y and states[i].direction == Direction.NORTH):
                commands.append("FW10")
            elif (states[i].x < states[i - 1].x and states[i].direction == Direction.WEST) or \
                 (states[i].y < states[i - 1].y and states[i].direction == Direction.SOUTH):
                commands.append("FW10")
            else:
                commands.append("BW10")

            if states[i].screenshot_id != -1:
                _inject_micro_turn_snap(commands, states[i], obstacles_dict)
            continue

        # Direction change: 90° turn commands
        if states[i - 1].direction == Direction.NORTH:
            if states[i].direction == Direction.EAST:
                commands.append("FR{}".format(steps) if states[i].y > states[i - 1].y else "BL{}".format(steps))
            elif states[i].direction == Direction.WEST:
                commands.append("FL{}".format(steps) if states[i].y > states[i - 1].y else "BR{}".format(steps))
            else:
                raise Exception("Invalid turning direction")
        elif states[i - 1].direction == Direction.EAST:
            if states[i].direction == Direction.NORTH:
                commands.append("FL{}".format(steps) if states[i].y > states[i - 1].y else "BR{}".format(steps))
            elif states[i].direction == Direction.SOUTH:
                commands.append("BL{}".format(steps) if states[i].y > states[i - 1].y else "FR{}".format(steps))
            else:
                raise Exception("Invalid turning direction")
        elif states[i - 1].direction == Direction.SOUTH:
            if states[i].direction == Direction.EAST:
                commands.append("BR{}".format(steps) if states[i].y > states[i - 1].y else "FL{}".format(steps))
            elif states[i].direction == Direction.WEST:
                commands.append("BL{}".format(steps) if states[i].y > states[i - 1].y else "FR{}".format(steps))
            else:
                raise Exception("Invalid turning direction")
        elif states[i - 1].direction == Direction.WEST:
            if states[i].direction == Direction.NORTH:
                commands.append("FR{}".format(steps) if states[i].y > states[i - 1].y else "BL{}".format(steps))
            elif states[i].direction == Direction.SOUTH:
                commands.append("BR{}".format(steps) if states[i].y > states[i - 1].y else "FL{}".format(steps))
            else:
                raise Exception("Invalid turning direction")
        else:
            raise Exception("Invalid position")

        if states[i].screenshot_id != -1:
            _inject_micro_turn_snap(commands, states[i], obstacles_dict)

    commands.append("FIN")

    # Compress consecutive FW/BW
    compressed_commands = [commands[0]]
    for i in range(1, len(commands)):
        if commands[i].startswith("BW") and compressed_commands[-1].startswith("BW"):
            prev_steps = int(compressed_commands[-1][2:])
            if prev_steps != 90:
                compressed_commands[-1] = "BW{}".format(prev_steps + 10)
                continue
        elif commands[i].startswith("FW") and compressed_commands[-1].startswith("FW"):
            prev_steps = int(compressed_commands[-1][2:])
            if prev_steps != 90:
                compressed_commands[-1] = "FW{}".format(prev_steps + 10)
                continue
        compressed_commands.append(commands[i])

    return compressed_commands


def _inject_micro_turn_snap(commands, state, obstacles_dict):
    """Inject micro-turn → SNAP → reverse-turn sequence.
    If face is centered (< 3°), just SNAP directly.
    """
    ob = obstacles_dict[state.screenshot_id]
    abs_angle, signed_deg, signal = compute_micro_turn_angle(
        state.x, state.y, int(state.direction),
        ob['x'], ob['y'], ob['d']
    )
    angle_int = round(abs_angle)

    if angle_int < 3:
        commands.append(f"SNAP{state.screenshot_id}_{signal}")
    else:
        if signed_deg > 0:
            commands.append(f"TL{angle_int}")
            commands.append(f"SNAP{state.screenshot_id}_{signal}")
            commands.append(f"TR{angle_int}")
        else:
            commands.append(f"TR{angle_int}")
            commands.append(f"SNAP{state.screenshot_id}_{signal}")
            commands.append(f"TL{angle_int}")