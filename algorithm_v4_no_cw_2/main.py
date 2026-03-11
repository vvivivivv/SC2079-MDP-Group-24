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

def calculate_positions(start_pose, commands):
    x, y, theta = start_pose
    positions = []
    
    for cmd in commands:
        if cmd.startswith("SNAP") or cmd == "FIN":
            # Divide by 10 to get the base grid index (24cm -> 2)
            grid_x = min(19, max(0, int(x / 10.0)))
            grid_y = min(19, max(0, int(y / 10.0)))
            
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
            d_cm = (dist_mm - OFFSET_FL) / SCALE_FL / 10.0
            R = ROBOT_TURN_RADIUS_FL_CM
            dtheta = d_cm / R
            cx = x - R * math.sin(theta)
            cy = y + R * math.cos(theta)
            theta += dtheta
            x = cx + R * math.sin(theta)
            y = cy - R * math.cos(theta)
            
        elif prefix == "FR":
            d_cm = (dist_mm - OFFSET_FR) / SCALE_FR / 10.0
            R = ROBOT_TURN_RADIUS_FR_CM
            dtheta = d_cm / R
            cx = x + R * math.sin(theta)
            cy = y - R * math.cos(theta)
            theta -= dtheta
            x = cx - R * math.sin(theta)
            y = cy + R * math.cos(theta)
            
        elif prefix == "BL":
            d_cm = (dist_mm - OFFSET_BL) / SCALE_BL / 10.0
            R = ROBOT_TURN_RADIUS_BL_CM
            dtheta = d_cm / R
            cx = x + R * math.sin(theta)
            cy = y - R * math.cos(theta)
            theta += dtheta
            x = cx - R * math.sin(theta)
            y = cy + R * math.cos(theta)
            
        elif prefix == "BR":
            d_cm = (dist_mm - OFFSET_BR) / SCALE_BR / 10.0
            R = ROBOT_TURN_RADIUS_BR_CM
            dtheta = d_cm / R
            cx = x - R * math.sin(theta)
            cy = y + R * math.cos(theta)
            theta -= dtheta
            x = cx + R * math.sin(theta)
            y = cy - R * math.cos(theta)

        theta = (theta + math.pi) % (2 * math.pi) - math.pi

        # Divide by 10 to get the base grid index (24cm -> 2)
        grid_x = min(19, max(0, int(x / 10.0)))
        grid_y = min(19, max(0, int(y / 10.0)))

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