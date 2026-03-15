import time
from algo.algo import MazeSolver
from flask import Flask, request, jsonify
from flask_cors import CORS
from model import *
import math
from consts import (
    ROBOT_TURN_RADIUS_FL_CM, ROBOT_TURN_RADIUS_FR_CM,
    ROBOT_TURN_RADIUS_BL_CM, ROBOT_TURN_RADIUS_BR_CM,
    SCALE_FW, SCALE_BW, SCALE_FL, SCALE_FR, SCALE_BL, SCALE_BR,
    OFFSET_FW, OFFSET_BW, OFFSET_FL, OFFSET_FR, OFFSET_BL, OFFSET_BR
)

def _get_cardinal_direction(theta_rad):
    heading_deg = math.degrees(theta_rad) % 360
    if 45 <= heading_deg < 135:
        return "N"
    elif 135 <= heading_deg < 225:
        return "W"
    elif 225 <= heading_deg < 315:
        return "S"
    else:
        return "E"

def _get_top_right_corner(x_center, y_center, theta):
    """Compute the top-right corner (max x, max y) of the 30x30cm robot.

    The robot is a 30x30cm square centered at (x_center, y_center) rotated
    by theta.  We compute all 4 corners in world coordinates and return the
    one with the largest x and largest y (i.e. the top-right in arena frame,
    regardless of robot heading).

    Returns: (corner_x_cm, corner_y_cm)
    """
    half = 15.0  # 30cm / 2
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    # 4 corners in robot-local frame: (±half, ±half)
    # Rotated to world: (x + dx*cos - dy*sin, y + dx*sin + dy*cos)
    corners_x = []
    corners_y = []
    for dx, dy in [(-half, -half), (-half, half), (half, -half), (half, half)]:
        wx = x_center + dx * cos_t - dy * sin_t
        wy = y_center + dx * sin_t + dy * cos_t
        corners_x.append(wx)
        corners_y.append(wy)
    return max(corners_x), max(corners_y)


def calculate_positions(start_pose, commands):
    x, y, theta = start_pose
    positions = []

    for cmd in commands:
        if cmd.startswith("SNAP") or cmd == "FIN":
            # Top-right corner of robot footprint in arena frame
            tr_x, tr_y = _get_top_right_corner(x, y, theta)
            grid_x = min(19, max(0, int(tr_x / 10.0)))
            grid_y = min(19, max(0, int(tr_y / 10.0)))

            positions.append({
                "x": grid_x,
                "y": grid_y,
                "direction": _get_cardinal_direction(theta)
            })
            continue

        prefix = cmd[:2]
        try:
            dist_mm = float(cmd[2:])
        except ValueError:
            dist_mm = 0.0

        if prefix == "FW":
            d_cm = (dist_mm - OFFSET_FW) / SCALE_FW / 10.0
            x += d_cm * math.cos(theta)
            y += d_cm * math.sin(theta)

        elif prefix == "BW":
            d_cm = (dist_mm - OFFSET_BW) / SCALE_BW / 10.0
            x -= d_cm * math.cos(theta)
            y -= d_cm * math.sin(theta)

        elif prefix == "FL":
            angle_deg = dist_mm
            dtheta = math.radians(angle_deg)
            R = ROBOT_TURN_RADIUS_FL_CM
            cx = x - R * math.sin(theta)
            cy = y + R * math.cos(theta)
            theta += dtheta
            x = cx + R * math.sin(theta)
            y = cy - R * math.cos(theta)

        elif prefix == "FR":
            angle_deg = dist_mm
            dtheta = math.radians(angle_deg)
            R = ROBOT_TURN_RADIUS_FR_CM
            cx = x + R * math.sin(theta)
            cy = y - R * math.cos(theta)
            theta -= dtheta
            x = cx - R * math.sin(theta)
            y = cy + R * math.cos(theta)

        elif prefix == "BL":
            angle_deg = dist_mm
            dtheta = math.radians(angle_deg)
            R = ROBOT_TURN_RADIUS_BL_CM
            cx = x + R * math.sin(theta)
            cy = y - R * math.cos(theta)
            theta += dtheta
            x = cx - R * math.sin(theta)
            y = cy + R * math.cos(theta)

        elif prefix == "BR":
            angle_deg = dist_mm
            dtheta = math.radians(angle_deg)
            R = ROBOT_TURN_RADIUS_BR_CM
            cx = x - R * math.sin(theta)
            cy = y + R * math.cos(theta)
            theta -= dtheta
            x = cx + R * math.sin(theta)
            y = cy - R * math.cos(theta)

        theta = (theta + math.pi) % (2 * math.pi) - math.pi

        # Top-right corner of robot footprint in arena frame
        tr_x, tr_y = _get_top_right_corner(x, y, theta)
        grid_x = min(19, max(0, int(tr_x / 10.0)))
        grid_y = min(19, max(0, int(tr_y / 10.0)))

        positions.append({
            "x": grid_x,
            "y": grid_y,
            "direction": _get_cardinal_direction(theta)
        })

    return positions

app = Flask(__name__)
CORS(app)
model = None
SAVE_DIR = "./received_images"
os.makedirs(SAVE_DIR, exist_ok=True)

@app.route('/status', methods=['GET'])
def status():
    return jsonify({"result": "ok"})


@app.route('/path', methods=['POST'])
def path_finding():
    content = request.json
    obstacles = content['obstacles']
    retrying = content['retrying']
    robot_x, robot_y = content['robot_x'], content['robot_y']
    robot_direction = int(content['robot_dir'])

    maze_solver = MazeSolver(20, 20, robot_x, robot_y, robot_direction)

    for ob in obstacles:
        maze_solver.add_obstacle(ob['x'], ob['y'], ob['d'], ob['id'])

    # Single unified call: Dubins TSP + Hybrid A* + command generation
    commands, waypoint_path, distance = maze_solver.plan_full_route(
        retrying=retrying,
        obstacles_data=obstacles
    )

    path_results = [s.get_dict() for s in waypoint_path]

    # --- NEW CODE BLOCK ---
    # Extract the starting (x, y, theta) using the static method from MazeSolver
    start_pose = MazeSolver._state_to_pose(waypoint_path[0])
    
    # Generate the parallel list of coordinates
    positions = calculate_positions(start_pose, commands)
    # ----------------------

    # print({
    #     "data": {
    #         'distance': distance,
    #         'path': path_results,
    #         'commands': commands,       # Original string commands remain unchanged
    #         'positions': positions      # New parallel array containing coordinates
    #     },
    #     "error": None
    # })

    return jsonify({
        "data": {
            'distance': distance,
            'path': path_results,
            'commands': commands,       # Original string commands remain unchanged
            'positions': positions      # New parallel array containing coordinates
        },
        "error": None
    })

@app.route("/upload", methods=["POST"])
def upload_image():
    f = request.files["file"]
    save_path = os.path.join(SAVE_DIR, f.filename)
    f.save(save_path)
    return jsonify({"ok": True, "saved_as": f.filename})


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)