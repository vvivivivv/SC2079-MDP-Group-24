import time
from algo.algo import MazeSolver
from flask import Flask, request, jsonify
from flask_cors import CORS
from model import *

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

    return jsonify({
        "data": {
            'distance': distance,
            'path': path_results,
            'commands': commands
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