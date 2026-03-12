import pygame
import math
import sys
import json
import time
import requests
import tkinter
from tkinter import filedialog, simpledialog
from consts import (
    ROBOT_SPEED_CM_S, ROBOT_AXLE_TRACK_CM,
    ROBOT_WHEELBASE_CM, PIVOT_OFFSET_X, PIVOT_OFFSET_Y,
    SCALE_FW, SCALE_BW, OFFSET_FW, OFFSET_BW,
    TURN_FL90_DX_CM, TURN_FL90_DY_CM,
    TURN_FR90_DX_CM, TURN_FR90_DY_CM,
    TURN_BL90_DX_CM, TURN_BL90_DY_CM,
    TURN_BR90_DX_CM, TURN_BR90_DY_CM,
)

# =============================================================================
# 1. LIVE TUNING VARIABLES (GLOBALS)
# =============================================================================
CURRENT_SPEED = ROBOT_SPEED_CM_S
TURN_SPEED_DEG_S = 120           # Degrees per second for spot turns

# =============================================================================
# 2. CONFIGURATION & CONSTANTS
# =============================================================================
API_URL = "http://localhost:5000/path"

WINDOW_WIDTH = 1000
WINDOW_HEIGHT = 800
ARENA_WIDTH = 800
FPS = 60

GRID_SIZE = 200
CELL_SIZE_CM = 10
GRID_COLS = 20
GRID_ROWS = 20

ROBOT_SIZE_CM = 30
# Visual trapezoid dimensions (cm) — fits inside 30x30 bounding box
ROBOT_FRONT_WIDTH_CM = 18.6   # north/front edge width
ROBOT_BACK_WIDTH_CM = 18.8    # south/back edge width
ROBOT_SIDE_LENGTH_CM = 23.0   # east/west side length
ROBOT_AXLE_TRACK = ROBOT_AXLE_TRACK_CM
SCAN_DURATION = 1.0

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRAY = (200, 200, 200)
LIGHT_GRAY = (220, 220, 220)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
RED = (255, 0, 0)
YELLOW = (255, 255, 0)
PURPLE = (128, 0, 128)
GHOST_COLOR = (200, 200, 255)
FOV_COLOR = (255, 255, 0, 60)

# =============================================================================
# 3. HELPER FUNCTIONS
# =============================================================================
def grid_to_pixel(x_cm, y_cm):
    scale = ARENA_WIDTH / GRID_SIZE
    px_x = x_cm * scale
    px_y = WINDOW_HEIGHT - (y_cm * scale)
    return int(px_x), int(px_y)

def pixel_to_grid(px_x, px_y):
    scale = ARENA_WIDTH / GRID_SIZE
    if px_x > ARENA_WIDTH: return None
    col = int(px_x / (CELL_SIZE_CM * scale))
    pixel_from_bottom = WINDOW_HEIGHT - px_y
    row = int(pixel_from_bottom / (CELL_SIZE_CM * scale))
    return (col, row)

def load_obstacles_from_json(filepath):
    try:
        with open(filepath, 'r') as f:
            data = json.load(f)
        obstacles = []
        max_id = 0
        for obs_data in data.get('obstacles', []):
            x = obs_data.get('x', 0)
            y = obs_data.get('y', 0)
            direction = obs_data.get('direction', obs_data.get('d', 0))
            if direction <= 6:
                if direction == 0: direction = 0
                elif direction == 2: direction = 90
                elif direction == 4: direction = 180
                elif direction == 6: direction = 270
            obs_id = obs_data.get('id', None)
            obstacles.append(Obstacle(x, y, direction, obs_id))
            if obs_id and obs_id > max_id:
                max_id = obs_id
        if max_id > 0:
            Obstacle._id_counter = max_id + 1
        return obstacles
    except Exception as e:
        print(f"Error loading: {e}")
        return []

def save_obstacles_to_json(obstacles, filepath):
    data = { "obstacles": [] }
    for obs in obstacles:
        d_val = 0
        if obs.direction == 0: d_val = 0
        elif obs.direction == 90: d_val = 2
        elif obs.direction == 180: d_val = 4
        elif obs.direction == 270: d_val = 6
        data["obstacles"].append({
            "id": obs.id, "x": obs.col, "y": obs.row, "d": d_val
        })
    try:
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=4)
        print("Saved.")
    except Exception as e:
        print(f"Error saving: {e}")

# =============================================================================
# 4. CLASSES
# =============================================================================
class Obstacle:
    _id_counter = 1
    def __init__(self, col, row, direction=0, obstacle_id=None):
        self.col = col; self.row = row; self.direction = direction
        self.is_scanned = False
        if obstacle_id is None:
            self.id = Obstacle._id_counter
            Obstacle._id_counter += 1
        else:
            self.id = obstacle_id

    def draw_safety_zone(self, surface):
        safety_margin_cm = 20
        x_cm = (self.col * CELL_SIZE_CM) - safety_margin_cm
        y_cm = ((self.row + 1) * CELL_SIZE_CM) + safety_margin_cm
        px_x, px_y = grid_to_pixel(x_cm, y_cm)
        safety_size_px = int((50 / GRID_SIZE) * ARENA_WIDTH)
        rect = pygame.Rect(px_x, px_y, safety_size_px, safety_size_px)
        pygame.draw.rect(surface, LIGHT_GRAY, rect, 1)

    def draw(self, surface):
        x_cm = self.col * CELL_SIZE_CM
        y_cm = (self.row + 1) * CELL_SIZE_CM
        px_x, px_y = grid_to_pixel(x_cm, y_cm)
        size_px = int((CELL_SIZE_CM / GRID_SIZE) * ARENA_WIDTH)
        rect = pygame.Rect(px_x, px_y, size_px, size_px)
        color = GREEN if self.is_scanned else RED
        pygame.draw.rect(surface, color, rect)
        pygame.draw.rect(surface, BLACK, rect, 1)
        font = pygame.font.SysFont(None, 20)
        text_color = BLACK if self.is_scanned else WHITE
        id_text = font.render(str(self.id), True, text_color)
        text_rect = id_text.get_rect(center=(px_x + size_px//2, px_y + size_px//2))
        surface.blit(id_text, text_rect)

        start_pos = (0,0); end_pos = (0,0)
        if self.direction == 0:
            start_pos = (px_x, px_y); end_pos = (px_x + size_px, px_y)
        elif self.direction == 90:
            start_pos = (px_x + size_px, px_y); end_pos = (px_x + size_px, px_y + size_px)
        elif self.direction == 180:
            start_pos = (px_x, px_y + size_px); end_pos = (px_x + size_px, px_y + size_px)
        else:
            start_pos = (px_x, px_y); end_pos = (px_x, px_y + size_px)
        pygame.draw.line(surface, BLACK, start_pos, end_pos, 4)


class Robot:
    def __init__(self, x, y, theta):
        self.x = x; self.y = y; self.theta = theta

        self.start_x = x; self.start_y = y; self.start_theta = theta
        self.default_x = x; self.default_y = y; self.default_theta = theta

        self.is_running = False
        self.commands = []
        self._cached_commands = []
        self.ghost_path = []
        self.path_history = [(x, y)]
        self.current_cmd_idx = 0
        self.state = "IDLE"
        self.target_val = 0; self.traveled_val = 0
        self.pause_end_time = 0

        # Step-by-Step State Management
        self.history_stack = []
        self.step_mode = False
        self.step_next = False

        # Turning variables
        self.angle_to_turn = 0
        self.angle_traveled = 0
        self.turn_direction = 0   # -1 for CW

        # Pivot-on-back-left-wheel state
        self.pivot_x = 0.0
        self.pivot_y = 0.0

        # Ackermann variables
        self.velocity = 0.0
        self.steering_angle = 0.0

        # 90-degree turn animation state
        self._turn_target_x = 0.0
        self._turn_target_y = 0.0
        self._turn_target_theta = 0.0
        self._turn_start_x = 0.0
        self._turn_start_y = 0.0
        self._turn_start_theta = 0.0
        self._turn_control_x = 0.0  # Bezier control point X
        self._turn_control_y = 0.0  # Bezier control point Y
        self._turn_frames_total = 1
        self._turn_frames_done = 0

        # Visual assets — trapezoid robot body inside 30x30 bounding box.
        scale = ARENA_WIDTH / GRID_SIZE
        size_px = int(ROBOT_SIZE_CM * scale)
        half = size_px / 2  # center in pixels (= 15cm * scale)

        sx = ROBOT_SIDE_LENGTH_CM / 2   # 11.5
        fw = ROBOT_FRONT_WIDTH_CM / 2   # 9.3
        bw = ROBOT_BACK_WIDTH_CM / 2    # 9.4

        trap_pts = [
            (half + sx * scale, half - fw * scale),  # front-left
            (half + sx * scale, half + fw * scale),  # front-right
            (half - sx * scale, half + bw * scale),  # back-right
            (half - sx * scale, half - bw * scale),  # back-left
        ]

        self.original_surf = pygame.Surface((size_px, size_px), pygame.SRCALPHA)
        pygame.draw.polygon(self.original_surf, (0, 0, 255, 160), trap_pts)
        pygame.draw.polygon(self.original_surf, BLACK, trap_pts, 2)
        cx, cy = int(half), int(half)
        front_x = int(half + sx * scale)
        pygame.draw.line(self.original_surf, YELLOW, (cx, cy), (front_x, cy), 3)
        pygame.draw.circle(self.original_surf, YELLOW, (cx, cy), 3)

    def reset_position(self):
        self.x = self.start_x; self.y = self.start_y; self.theta = self.start_theta
        self.is_running = False; self.path_history = [(self.x, self.y)]
        self.current_cmd_idx = 0; self.state = "IDLE"
        self.velocity = 0; self.steering_angle = 0
        self.history_stack.clear()
        self.step_next = False

    def reset_all(self):
        self.start_x = self.default_x; self.start_y = self.default_y; self.start_theta = self.default_theta
        self.reset_position()
        self.commands = []; self._cached_commands = []; self.ghost_path = []

    def has_cached_mission(self): return len(self._cached_commands) > 0

    def start_from_cache(self):
        self.reset_position()
        self.commands = list(self._cached_commands)
        self.is_running = True; self.current_cmd_idx = 0

    def start_mission(self, api_data):
        self.commands = api_data.get('commands', [])
        self._cached_commands = list(self.commands)
        raw_path = api_data.get('path', [])
        self.ghost_path = []
        for p in raw_path:
            px = p['x']; py = p['y']
            if isinstance(px, float): self.ghost_path.append((px * 10, py * 10))
            else: self.ghost_path.append((px * 10 + 5, py * 10 + 5))
        self.is_running = True; self.current_cmd_idx = 0
        self.history_stack.clear()

    def execute_manual_command(self, cmd_str):
        self.commands = [cmd_str]
        self.is_running = True
        self.current_cmd_idx = 0
        self.state = "IDLE"
        self.path_history = [(self.x, self.y)]
        self.history_stack.clear()
        print(f"Executing Manual Command: {cmd_str}")

    # =========================================================================
    # STEP-BY-STEP SNAPSHOT ENGINE
    # =========================================================================
    def save_snapshot(self, scanned_ids):
        self.history_stack.append({
            'x': self.x,
            'y': self.y,
            'theta': self.theta,
            'path_history': list(self.path_history),
            'cmd_idx': self.current_cmd_idx,
            'scanned_ids': list(scanned_ids)
        })

    def restore_snapshot(self):
        if not self.history_stack:
            return None
        state = self.history_stack.pop()
        self.x = state['x']
        self.y = state['y']
        self.theta = state['theta']
        self.path_history = state['path_history']
        self.current_cmd_idx = state['cmd_idx']
        
        self.state = "IDLE"
        self.velocity = 0
        self.steering_angle = 0
        self.angle_traveled = 0
        self.angle_to_turn = 0
        return state['scanned_ids']

    # =========================================================================
    # PIVOT-ON-BACK-LEFT-WHEEL HELPER
    # =========================================================================
    def _compute_pivot_point(self):
        theta_rad = math.radians(self.theta)
        cos_t = math.cos(theta_rad)
        sin_t = math.sin(theta_rad)
        self.pivot_x = self.x + PIVOT_OFFSET_X * cos_t - PIVOT_OFFSET_Y * sin_t
        self.pivot_y = self.y + PIVOT_OFFSET_X * sin_t + PIVOT_OFFSET_Y * cos_t

    def _rotate_around_pivot(self, delta_deg):
        alpha = math.radians(delta_deg)
        dx = self.x - self.pivot_x
        dy = self.y - self.pivot_y
        self.x = self.pivot_x + dx * math.cos(alpha) + dy * math.sin(alpha)
        self.y = self.pivot_y - dx * math.sin(alpha) + dy * math.cos(alpha)
        self.theta -= delta_deg

    # =========================================================================
    # PHYSICS ENGINE
    # =========================================================================
    def update(self, current_scanned_ids=None):
        if not self.is_running: return None
        dt = 1.0 / FPS

        # 1. Handle SNAP
        if self.state == "SCANNING":
            if time.time() > self.pause_end_time:
                self.state = "IDLE"
                current_cmd = self.commands[self.current_cmd_idx]
                parts = current_cmd.replace("SNAP", "").split("_")
                self.current_cmd_idx += 1
                return int(parts[0])
            else:
                return None

        # 2. Handle IDLE / NEW COMMAND FETCH
        if self.state == "IDLE":
            if self.current_cmd_idx >= len(self.commands):
                self.is_running = False
                print("Mission Complete")
                return None

            if self.step_mode and not self.step_next:
                return None
            
            self.step_next = False

            if current_scanned_ids is not None:
                self.save_snapshot(current_scanned_ids)

            cmd = self.commands[self.current_cmd_idx]
            self._decode_command(cmd)

        # 3. Handle TURNING — PIVOT ON BACK-LEFT WHEEL
        if self.state == "TURNING":
            turn_step = TURN_SPEED_DEG_S * dt
            remaining = self.angle_to_turn - self.angle_traveled

            if turn_step >= remaining:
                self._rotate_around_pivot(remaining)
                self.theta = self.theta % 360
                self.state = "IDLE"
                self.current_cmd_idx += 1
                self.angle_traveled = 0
                self.angle_to_turn = 0
                self.path_history.append((self.x, self.y))
            else:
                self._rotate_around_pivot(turn_step)
                self.angle_traveled += turn_step
                self.path_history.append((self.x, self.y))

            return None

        # 3a. Handle TURNING_90 — animated discrete 90-degree turn
        if self.state == "TURNING_90":
            self._turn_frames_done += 1
            t = min(1.0, self._turn_frames_done / self._turn_frames_total)

            # Smooth interpolation (Hermite smoothstep)
            t_smooth = t * t * (3 - 2 * t)

            # Quadratic Bezier Curve for Position (Car-like arc)
            u = 1.0 - t_smooth
            self.x = (u * u * self._turn_start_x) + (2 * u * t_smooth * self._turn_control_x) + (t_smooth * t_smooth * self._turn_target_x)
            self.y = (u * u * self._turn_start_y) + (2 * u * t_smooth * self._turn_control_y) + (t_smooth * t_smooth * self._turn_target_y)

            # Interpolate heading (handle wraparound)
            dtheta = self._turn_target_theta - self._turn_start_theta
            if dtheta > 180: dtheta -= 360
            if dtheta < -180: dtheta += 360
            self.theta = (self._turn_start_theta + t_smooth * dtheta) % 360

            self.path_history.append((self.x, self.y))

            if self._turn_frames_done >= self._turn_frames_total:
                # Snap to exact target
                self.x = self._turn_target_x
                self.y = self._turn_target_y
                self.theta = self._turn_target_theta
                self.state = "IDLE"
                self.current_cmd_idx += 1
            return None

        # 4. PHYSICS: STRAIGHT LINE
        self.x += self.velocity * math.cos(math.radians(self.theta)) * dt
        self.y += self.velocity * math.sin(math.radians(self.theta)) * dt

        if abs(self.velocity) > 0:
            self.path_history.append((self.x, self.y))

        # 5. Check Termination
        if self.state == "MOVING":
            self.traveled_val += abs(self.velocity * dt)
            if self.traveled_val >= self.target_val:
                self._stop_motors()

        return None

    def _stop_motors(self):
        self.velocity = 0
        self.steering_angle = 0
        self.state = "IDLE"; self.current_cmd_idx += 1

    def _decode_command(self, cmd):
        global CURRENT_SPEED
        print(f"Processing: {cmd}")
        self.traveled_val = 0

        # 1. 90-DEGREE TURNS (FL90, FR90, BL90, BR90) — discrete position+heading jumps
        if cmd in ("FL90", "FR90", "BL90", "BR90"):
            theta_rad = math.radians(self.theta)
            cos_t = math.cos(theta_rad)
            sin_t = math.sin(theta_rad)

            if cmd == "FL90":
                ldx, ldy, dtheta = TURN_FL90_DX_CM, TURN_FL90_DY_CM, 90.0
            elif cmd == "FR90":
                ldx, ldy, dtheta = TURN_FR90_DX_CM, TURN_FR90_DY_CM, -90.0
            elif cmd == "BL90":
                ldx, ldy, dtheta = TURN_BL90_DX_CM, TURN_BL90_DY_CM, 90.0
            elif cmd == "BR90":
                ldx, ldy, dtheta = TURN_BR90_DX_CM, TURN_BR90_DY_CM, -90.0

            # World displacement
            wx = ldx * cos_t - ldy * sin_t
            wy = ldx * sin_t + ldy * cos_t

            # Set up animation
            self._turn_start_x = self.x
            self._turn_start_y = self.y
            self._turn_start_theta = self.theta
            self._turn_target_x = self.x + wx
            self._turn_target_y = self.y + wy
            self._turn_target_theta = (self.theta + dtheta) % 360
            
            # Bezier control point placed along the initial heading
            self._turn_control_x = self.x + ldx * cos_t
            self._turn_control_y = self.y + ldx * sin_t

            # Dynamic duration based on TURN_SPEED_DEG_S (e.g. 90 deg / 120 deg/s = 0.75s)
            duration_s = 90.0 / TURN_SPEED_DEG_S
            self._turn_frames_total = max(1, int(FPS * duration_s))
            self._turn_frames_done = 0
            self.state = "TURNING_90"
            return

        # 2. STRAIGHT MOVES (FW, BW)
        if cmd.startswith("FW"):
            try: val_mm = int(cmd[2:])
            except: val_mm = 100
            self.target_val = (val_mm - OFFSET_FW) / SCALE_FW / 10.0
            self.state = "MOVING"
            self.velocity = CURRENT_SPEED
            self.steering_angle = 0.0

        elif cmd.startswith("BW"):
            try: val_mm = int(cmd[2:])
            except: val_mm = 100
            self.target_val = (val_mm - OFFSET_BW) / SCALE_BW / 10.0
            self.state = "MOVING"
            self.velocity = -CURRENT_SPEED
            self.steering_angle = 0.0

        # 3. SPOT TURNS (CW) — kept for manual use
        elif cmd.startswith("CW"):
            try:
                deg = int(cmd[2:])
                self.angle_to_turn = deg
                self.angle_traveled = 0
                self.velocity = 0
                self.steering_angle = 0
                self.turn_direction = -1

                self._compute_pivot_point()
                self.state = "TURNING"
                print(f"  Pivot CW turn -> {deg} deg (back-left wheel fixed)")
            except ValueError:
                print(f"Error parsing spot turn: {cmd}")
                self.state = "IDLE"; self.current_cmd_idx += 1

        # 4. AUX COMMANDS
        elif cmd.startswith("SNAP"):
            self.state = "SCANNING"
            self.pause_end_time = time.time() + SCAN_DURATION

        elif cmd.startswith("FIN"):
            self.state = "IDLE"; self.current_cmd_idx += 1
        else:
            self.state = "IDLE"; self.current_cmd_idx += 1

    def draw(self, surface):
        px, py = grid_to_pixel(self.x, self.y)
        rotated_surf = pygame.transform.rotate(self.original_surf, self.theta)
        rect = rotated_surf.get_rect(center=(px, py))
        surface.blit(rotated_surf, rect.topleft)

    def draw_fov_cone(self, surface):
        px, py = grid_to_pixel(self.x, self.y)
        scale = ARENA_WIDTH / GRID_SIZE
        range_px = int(30 * scale)
        theta_rad = math.radians(self.theta)
        half_fov = math.radians(31.1)
        left_angle = theta_rad + half_fov
        right_angle = theta_rad - half_fov
        cone_surface = pygame.Surface((ARENA_WIDTH, WINDOW_HEIGHT), pygame.SRCALPHA)
        points = [(px, py)]
        for i in range(21):
            angle = right_angle + (left_angle - right_angle) * i / 20
            x = px + range_px * math.cos(angle)
            y = py - range_px * math.sin(angle)
            points.append((int(x), int(y)))
        if len(points) >= 3:
            pygame.draw.polygon(cone_surface, FOV_COLOR, points)
        surface.blit(cone_surface, (0, 0))

    def draw_path(self, surface):
        if len(self.ghost_path) > 1:
             points = [grid_to_pixel(x, y) for x, y in self.ghost_path]
             pygame.draw.lines(surface, GHOST_COLOR, False, points, 2)
        if len(self.path_history) < 2: return
        points = [grid_to_pixel(x, y) for x, y in self.path_history]
        if len(points) > 1:
            pygame.draw.lines(surface, PURPLE, False, points, 2)


# =============================================================================
# 5. MAIN LOOP
# =============================================================================
def main():
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    clock = pygame.time.Clock()
    font = pygame.font.SysFont(None, 24)
    timer_font = pygame.font.SysFont(None, 40)

    robot = Robot(x=15, y=15, theta=90)
    obstacles = []

    start_time = None; elapsed_time = 0.0

    running = True
    while running:
        mouse_x, mouse_y = pygame.mouse.get_pos()
        keys = pygame.key.get_pressed()

        # UI BUTTONS
        setpos_btn = pygame.Rect(ARENA_WIDTH + 20, 390, 160, 40)
        start_btn  = pygame.Rect(ARENA_WIDTH + 20, 440, 160, 40)
        restart_btn= pygame.Rect(ARENA_WIDTH + 20, 490, 160, 40)
        reset_btn  = pygame.Rect(ARENA_WIDTH + 20, 540, 160, 40)
        manual_btn = pygame.Rect(ARENA_WIDTH + 20, 590, 160, 40)
        
        # New Step Control Buttons
        step_bwd_btn  = pygame.Rect(ARENA_WIDTH + 20, 640, 75, 40)
        step_fwd_btn  = pygame.Rect(ARENA_WIDTH + 105, 640, 75, 40)
        auto_play_btn = pygame.Rect(ARENA_WIDTH + 20, 690, 160, 40)

        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_s and keys[pygame.K_LCTRL]:
                    save_obstacles_to_json(obstacles, 'obstacles.json')

                elif event.key == pygame.K_l and keys[pygame.K_LCTRL]:
                    root = tkinter.Tk()
                    root.withdraw()
                    file_path = filedialog.askopenfilename(title="Select Obstacles JSON", filetypes=[("JSON Files", "*.json")])
                    root.destroy()
                    if file_path:
                        print(f"Loading map from: {file_path}")
                        robot.reset_all()
                        obstacles = load_obstacles_from_json(file_path)

            if event.type == pygame.MOUSEBUTTONDOWN:

                if setpos_btn.collidepoint(mouse_x, mouse_y):
                    root = tkinter.Tk()
                    root.withdraw()
                    user_input = simpledialog.askstring("Set Position", "Enter X, Y, Direction (e.g. 1.5, 1.5, 90):")
                    root.destroy()

                    if user_input:
                        try:
                            parts = [float(p.strip()) for p in user_input.split(',')]
                            if len(parts) == 3:
                                new_x, new_y, new_dir = parts

                                if new_x.is_integer() and new_y.is_integer():
                                    robot.x = new_x * 10 + 5
                                    robot.y = new_y * 10 + 5
                                else:
                                    robot.x = new_x * 10
                                    robot.y = new_y * 10

                                robot.theta = new_dir
                                robot.start_x = robot.x
                                robot.start_y = robot.y
                                robot.start_theta = robot.theta
                                robot.path_history = [(robot.x, robot.y)]
                                print(f"Robot moved to {robot.x/10}, {robot.y/10}, {robot.theta}")

                                if robot.has_cached_mission():
                                    robot.commands = []
                                    robot._cached_commands = []
                                    robot.ghost_path = []
                                    print("Cache cleared due to position change.")

                        except ValueError:
                            print("Invalid input")

                elif start_btn.collidepoint(mouse_x, mouse_y):
                    for ob in obstacles: ob.is_scanned = False
                    robot.step_mode = False  # Default to auto on fresh start

                    if robot.has_cached_mission():
                        print("Replaying cached mission...")
                        robot.start_from_cache()
                        start_time = time.time()

                    else:
                        norm_theta = int(round(robot.theta)) % 360
                        if norm_theta == 90: robot_dir_api = 0
                        elif norm_theta == 0: robot_dir_api = 2
                        elif norm_theta == 270: robot_dir_api = 4
                        elif norm_theta == 180: robot_dir_api = 6
                        else: robot_dir_api = 0

                        payload = {
                            "obstacles": [],
                            "retrying": False,
                            "robot_x": int(robot.x/10),
                            "robot_y": int(robot.y/10),
                            "robot_dir": robot_dir_api
                        }

                        for ob in obstacles:
                            d_api = 0
                            if ob.direction == 0: d_api = 0
                            elif ob.direction == 90: d_api = 2
                            elif ob.direction == 180: d_api = 4
                            elif ob.direction == 270: d_api = 6
                            payload["obstacles"].append({ "x": ob.col, "y": ob.row, "id": ob.id, "d": d_api })

                        print("Sending request...")
                        try:
                            r = requests.post(API_URL, json=payload)
                            if r.status_code == 200:
                                data = r.json()
                                if data.get('error'): print(f"Error: {data['error']}")
                                else:
                                    robot.start_mission(data['data'])
                                    start_time = time.time()
                            else: print(f"Status {r.status_code}")
                        except Exception as e: print(f"Failed: {e}")

                elif restart_btn.collidepoint(mouse_x, mouse_y):
                    robot.reset_position()
                    start_time = None; elapsed_time = 0.0
                    for ob in obstacles: ob.is_scanned = False
                    print("Restarted at current mission start.")

                elif reset_btn.collidepoint(mouse_x, mouse_y):
                    robot.reset_all(); obstacles.clear(); Obstacle._id_counter = 1
                    start_time = None; elapsed_time = 0.0
                    print("Full Reset.")

                elif pixel_to_grid(mouse_x, mouse_y):
                    c, r = pixel_to_grid(mouse_x, mouse_y)
                    existing = next((o for o in obstacles if o.col == c and o.row == r), None)
                    if existing:
                        if event.button == 1: existing.direction = (existing.direction + 90) % 360
                        elif event.button == 3: obstacles.remove(existing)
                    else:
                        if event.button == 1: obstacles.append(Obstacle(c, r))

                    if robot.has_cached_mission():
                        robot.reset_all()

                elif manual_btn.collidepoint(mouse_x, mouse_y):
                    root = tkinter.Tk()
                    root.withdraw()
                    cmd_input = simpledialog.askstring("Manual Command", "Enter Command (e.g. FW20, FL39, SNAP1):")
                    root.destroy()

                    if cmd_input:
                        clean_cmd = cmd_input.strip().upper().replace(" ", "")
                        robot.execute_manual_command(clean_cmd)

                # ================= STEP CONTROLS =================
                elif step_fwd_btn.collidepoint(mouse_x, mouse_y):
                    robot.step_mode = True
                    robot.step_next = True
                    # If robot isn't running but has cache, kickstart it
                    if not robot.is_running and robot.has_cached_mission() and robot.current_cmd_idx == 0:
                        robot.start_from_cache()
                        robot.step_mode = True
                        robot.step_next = True

                elif step_bwd_btn.collidepoint(mouse_x, mouse_y):
                    robot.step_mode = True
                    robot.step_next = False
                    restored_scanned = robot.restore_snapshot()
                    
                    if restored_scanned is not None:
                        # Revert scanned obstacles to their old state
                        for ob in obstacles:
                            ob.is_scanned = (ob.id in restored_scanned)
                        
                        # Re-activate the robot logic so it draws and waits
                        if not robot.is_running:
                            robot.is_running = True

                elif auto_play_btn.collidepoint(mouse_x, mouse_y):
                    robot.step_mode = not robot.step_mode # Toggle auto
                    if not robot.step_mode and not robot.is_running and robot.has_cached_mission() and robot.current_cmd_idx < len(robot.commands):
                        robot.is_running = True

        # Extract current scanned obstacle IDs to pass to the snapshot engine
        current_scanned = [ob.id for ob in obstacles if ob.is_scanned]

        if robot.is_running:
            scanned_id = robot.update(current_scanned)
            if start_time: elapsed_time = time.time() - start_time
            if scanned_id is not None:
                for ob in obstacles:
                    if ob.id == scanned_id: ob.is_scanned = True

        # Draw
        screen.fill(WHITE)
        for i in range(GRID_COLS + 1):
            s = grid_to_pixel(i * 10, 0); e = grid_to_pixel(i * 10, GRID_SIZE)
            pygame.draw.line(screen, GRAY, s, e)
            s = grid_to_pixel(0, i * 10); e = grid_to_pixel(GRID_SIZE, i * 10)
            pygame.draw.line(screen, GRAY, s, e)

        # Start Zone
        box_px = int((40 / GRID_SIZE) * ARENA_WIDTH)
        s = pygame.Surface((box_px, box_px)); s.set_alpha(100); s.fill(GREEN)
        screen.blit(s, (0, WINDOW_HEIGHT - box_px))

        for obs in obstacles: obs.draw_safety_zone(screen); obs.draw(screen)
        robot.draw_path(screen); robot.draw_fov_cone(screen); robot.draw(screen)

        # UI Panel
        pygame.draw.rect(screen, (255, 250, 250), (ARENA_WIDTH, 0, WINDOW_WIDTH-ARENA_WIDTH, WINDOW_HEIGHT))
        screen.blit(timer_font.render(f"{elapsed_time:.2f}s", True, RED), (ARENA_WIDTH + 40, 20))
        screen.blit(font.render("Group 24 SIMULATOR", True, BLUE), (ARENA_WIDTH + 10, 80))

        # Robot coordinates & heading (always visible)
        r_gx = robot.x / 10.0
        r_gy = robot.y / 10.0
        r_hdg = robot.theta % 360
        screen.blit(font.render(f"X: {r_gx:.2f}  Y: {r_gy:.2f}", True, BLACK), (ARENA_WIDTH + 10, 110))
        screen.blit(font.render(f"Heading: {r_hdg:.1f} deg", True, BLACK), (ARENA_WIDTH + 10, 135))
        # Also show cm coordinates
        screen.blit(font.render(f"({robot.x:.1f}, {robot.y:.1f}) cm", True, GRAY), (ARENA_WIDTH + 10, 160))

        if robot.has_cached_mission():
            screen.blit(font.render("Mission CACHED", True, (0, 128, 0)), (ARENA_WIDTH + 20, 220))
            
            # HUD: Show Current / Next Instruction
            if robot.current_cmd_idx < len(robot.commands):
                cmd_str = robot.commands[robot.current_cmd_idx]
                screen.blit(font.render(f"Next: {cmd_str}", True, PURPLE), (ARENA_WIDTH + 20, 260))
                screen.blit(font.render(f"Step: {robot.current_cmd_idx + 1} / {len(robot.commands)}", True, PURPLE), (ARENA_WIDTH + 20, 290))
            else:
                screen.blit(font.render("Mission Complete", True, GREEN), (ARENA_WIDTH + 20, 260))
        else:
            screen.blit(font.render("No cached mission", True, GRAY), (ARENA_WIDTH + 20, 220))

        # Core Buttons
        pygame.draw.rect(screen, (0, 200, 255), setpos_btn)
        pygame.draw.rect(screen, BLACK, setpos_btn, 2)
        screen.blit(font.render("SET POS", True, BLACK), (ARENA_WIDTH + 50, 400))

        btn_color = (100, 255, 100) if robot.has_cached_mission() else (GREEN if len(obstacles)>=1 else LIGHT_GRAY)
        pygame.draw.rect(screen, btn_color, start_btn)
        pygame.draw.rect(screen, BLACK, start_btn, 2)
        start_label = "GO (cached)" if robot.has_cached_mission() else "START"
        screen.blit(font.render(start_label, True, BLACK), (ARENA_WIDTH + 40, 450))

        pygame.draw.rect(screen, YELLOW, restart_btn)
        pygame.draw.rect(screen, BLACK, restart_btn, 2)
        screen.blit(font.render("RESTART", True, BLACK), (ARENA_WIDTH + 45, 500))

        pygame.draw.rect(screen, RED, reset_btn)
        pygame.draw.rect(screen, BLACK, reset_btn, 2)
        screen.blit(font.render("RESET", True, WHITE), (ARENA_WIDTH + 55, 550))

        pygame.draw.rect(screen, PURPLE, manual_btn)
        pygame.draw.rect(screen, BLACK, manual_btn, 2)
        screen.blit(font.render("MANUAL CMD", True, WHITE), (ARENA_WIDTH + 30, 600))

        # Step Engine Buttons
        pygame.draw.rect(screen, (255, 165, 0), step_bwd_btn)  # Orange
        pygame.draw.rect(screen, BLACK, step_bwd_btn, 2)
        screen.blit(font.render("< REV", True, BLACK), (ARENA_WIDTH + 32, 652))

        pygame.draw.rect(screen, (0, 255, 127), step_fwd_btn)  # Spring Green
        pygame.draw.rect(screen, BLACK, step_fwd_btn, 2)
        screen.blit(font.render("FWD >", True, BLACK), (ARENA_WIDTH + 115, 652))

        auto_bg = GRAY if robot.step_mode else GREEN
        pygame.draw.rect(screen, auto_bg, auto_play_btn)
        pygame.draw.rect(screen, BLACK, auto_play_btn, 2)
        auto_text = "AUTO: OFF" if robot.step_mode else "AUTO: ON"
        screen.blit(font.render(auto_text, True, BLACK), (ARENA_WIDTH + 40, 702))

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()