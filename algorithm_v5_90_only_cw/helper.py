from consts import WIDTH, HEIGHT, Direction


def is_valid(center_x: int, center_y: int):
    """Checks if given position is within bounds

    Inputs
    ------
    center_x (int): x-coordinate
    center_y (int): y-coordinate

    Returns
    -------
    bool: True if valid, False otherwise
    """
    return center_x > 0 and center_y > 0 and center_x < WIDTH - 1 and center_y < HEIGHT - 1


def command_generator(states, obstacles):
    """
    This function takes in a list of states and generates a list of commands for the robot to follow
    
    Inputs
    ------
    states: list of State objects
    obstacles: list of obstacles, each obstacle is a dictionary with keys "x", "y", "d", and "id"

    Returns
    -------
    commands: list of commands for the robot to follow
    """
    
    # Initialize commands list
    commands = []

    # Iterate through each state in the list of states
    for i in range(1, len(states)):

        # If previous state and current state are the same direction,
        if states[i].direction == states[i - 1].direction:
            # Forward - Must be (east facing AND x value increased) OR (north facing AND y value increased)
            if (states[i].x > states[i - 1].x and states[i].direction == Direction.EAST) or (states[i].y > states[i - 1].y and states[i].direction == Direction.NORTH):
                commands.append("FW100")
            # Forward - Must be (west facing AND x value decreased) OR (south facing AND y value decreased)
            elif (states[i].x < states[i-1].x and states[i].direction == Direction.WEST) or (
                    states[i].y < states[i-1].y and states[i].direction == Direction.SOUTH):
                commands.append("FW100")
            # Backward - All other cases where the previous and current state is the same direction
            else:
                commands.append("BW100")

            # If any of these states has a valid screenshot ID, then add a SNAP command as well to take a picture
            if states[i].screenshot_id != -1:
                commands.append(f"SNAP{states[i].screenshot_id}")
            continue

        # If previous state and current state are not the same direction, it's a pivot turn
        # Determine which wheel was used by checking displacement direction
        # FR = left-wheel CW,  FL = left-wheel CCW
        # BR = right-wheel CW, BL = right-wheel CCW
        CW_TURNS = {
            (Direction.NORTH, Direction.EAST),
            (Direction.EAST,  Direction.SOUTH),
            (Direction.SOUTH, Direction.WEST),
            (Direction.WEST,  Direction.NORTH),
        }

        # Left-wheel pivot displacements (must match PIVOT_LEFT in algo.py)
        LEFT_PIVOT = {
            (Direction.NORTH, Direction.EAST):  (-1, -1),
            (Direction.NORTH, Direction.WEST):  (-1,  1),
            (Direction.EAST,  Direction.SOUTH): (-1,  1),
            (Direction.EAST,  Direction.NORTH): ( 1,  1),
            (Direction.SOUTH, Direction.WEST):  ( 1,  1),
            (Direction.SOUTH, Direction.EAST):  ( 1, -1),
            (Direction.WEST,  Direction.NORTH): ( 1, -1),
            (Direction.WEST,  Direction.SOUTH): (-1, -1),
        }

        prev_dir = states[i - 1].direction
        cur_dir = states[i].direction
        dx = states[i].x - states[i - 1].x
        dy = states[i].y - states[i - 1].y

        is_left_wheel = (dx, dy) == LEFT_PIVOT.get((prev_dir, cur_dir), None)
        is_cw = (prev_dir, cur_dir) in CW_TURNS

        if is_left_wheel:
            commands.append("FR00" if is_cw else "FL00")
        else:
            commands.append("BR00" if is_cw else "BL00")

        # If any of these states has a valid screenshot ID, then add a SNAP command as well to take a picture
        if states[i].screenshot_id != -1:
            commands.append(f"SNAP{states[i].screenshot_id}")

    # Final command is the stop command (FIN)
    commands.append("FIN")  

    # Compress commands if there are consecutive forward or backward commands
    compressed_commands = [commands[0]]

    for i in range(1, len(commands)):
        # If both commands are BW
        if commands[i].startswith("BW") and compressed_commands[-1].startswith("BW"):
            # Get the number of steps of previous command
            steps = int(compressed_commands[-1][2:])
            # If steps are not 900, add 100 to the steps
            if steps != 900:
                compressed_commands[-1] = "BW{}".format(steps + 100)
                continue

        # If both commands are FW
        elif commands[i].startswith("FW") and compressed_commands[-1].startswith("FW"):
            # Get the number of steps of previous command
            steps = int(compressed_commands[-1][2:])
            # If steps are not 900, add 100 to the steps
            if steps != 900:
                compressed_commands[-1] = "FW{}".format(steps + 100)
                continue
        
        # Otherwise, just add as usual
        compressed_commands.append(commands[i])

    return compressed_commands