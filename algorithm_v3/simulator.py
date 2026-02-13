import pygame
import math
import sys
import json
import time
import requests
import tkinter
from tkinter import filedialog, simpledialog
from consts import ROBOT_SPEED_CM_S, ROBOT_AXLE_TRACK_CM, ROBOT_TURN_RADIUS_CM, ROBOT_WHEELBASE_CM

# =============================================================================
# 1. LIVE TUNING VARIABLES (GLOBALS)
# =============================================================================
CURRENT_SPEED = ROBOT_SPEED_CM_S # cm/s

# =============================================================================
# 2. CONFIGURATION & CONSTANTS
# =============================================================================
API_URL = "http://localhost:5000/path"

WINDOW_WIDTH = 1000
WINDOW_HEIGHT = 800
ARENA_WIDTH = 800
FPS = 60

# Arena Settings
GRID_SIZE = 200
CELL_SIZE_CM = 10
GRID_COLS = 20
GRID_ROWS = 20

# Robot Settings
ROBOT_SIZE_CM = 30
ROBOT_AXLE_TRACK = ROBOT_AXLE_TRACK_CM  # Distance between wheels
SCAN_DURATION = 1.0      # Time to stop for scanning

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
            # Normalize direction
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
        
        # Direction Indicator
        start_pos = (0,0); end_pos = (0,0)
        if self.direction == 0:   # North
            start_pos = (px_x, px_y); end_pos = (px_x + size_px, px_y)
        elif self.direction == 90: # East
            start_pos = (px_x + size_px, px_y); end_pos = (px_x + size_px, px_y + size_px)
        elif self.direction == 180: # South
            start_pos = (px_x, px_y + size_px); end_pos = (px_x + size_px, px_y + size_px)
        else: # West
            start_pos = (px_x, px_y); end_pos = (px_x, px_y + size_px)
        pygame.draw.line(surface, BLACK, start_pos, end_pos, 4)

class Robot:
    def __init__(self, x, y, theta):
        self.x = x; self.y = y; self.theta = theta 
        
        # Save start/default positions for Reset/Restart functionality
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
        
        # --- ACKERMANN VARIABLES ---
        self.velocity = 0.0        # cm/s (Single speed for the whole car)
        self.steering_angle = 0.0  # radians (Positive = Left, Negative = Right)
        
        # Visual assets
        scale = ARENA_WIDTH / GRID_SIZE
        size_px = int(ROBOT_SIZE_CM * scale)
        self.original_surf = pygame.Surface((size_px, size_px), pygame.SRCALPHA)
        self.original_surf.fill(BLUE)
        w, h = size_px, size_px
        pygame.draw.line(self.original_surf, YELLOW, (w/2, h/2), (w, h/2), 3)

    # (Keep reset_position, reset_all, has_cached_mission, start_from_cache, start_mission exactly as they were)
    def reset_position(self):
        self.x = self.start_x; self.y = self.start_y; self.theta = self.start_theta
        self.is_running = False; self.path_history = [(self.x, self.y)]
        self.current_cmd_idx = 0; self.state = "IDLE"
        self.velocity = 0; self.steering_angle = 0

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

    def execute_manual_command(self, cmd_str):
        """Injects a single command and executes it immediately."""
        self.commands = [cmd_str]
        self.is_running = True
        self.current_cmd_idx = 0
        self.state = "IDLE" # Will trigger _decode_command on next update loop
        self.path_history = [(self.x, self.y)] # Optional: Reset path or keep it
        print(f"Executing Manual Command: {cmd_str}")

    # =========================================================================
    # NEW: ACKERMANN PHYSICS ENGINE
    # =========================================================================
    def update(self):
        if not self.is_running: return None
        dt = 1.0 / FPS 

        # 1. Handle SNAP (Scanning)
        if self.state == "SCANNING":
            if time.time() > self.pause_end_time:
                self.state = "IDLE"
                current_cmd = self.commands[self.current_cmd_idx]
                parts = current_cmd.replace("SNAP", "").split("_")
                self.current_cmd_idx += 1
                return int(parts[0]) 
            else:
                return None

        # 2. Handle IDLE (Fetch next command)
        if self.state == "IDLE":
            if self.current_cmd_idx >= len(self.commands):
                self.is_running = False
                print("Mission Complete")
                return None
            cmd = self.commands[self.current_cmd_idx]
            self._decode_command(cmd)

        # 3. PHYSICS: BICYCLE MODEL
        # x_dot = v * cos(theta)
        # y_dot = v * sin(theta)
        # theta_dot = (v / L) * tan(delta)
        
        self.x += self.velocity * math.cos(math.radians(self.theta)) * dt
        self.y += self.velocity * math.sin(math.radians(self.theta)) * dt
        
        if abs(self.steering_angle) > 0.001:
            # Calculate angular velocity (degrees per second)
            angular_velocity = (self.velocity / ROBOT_WHEELBASE_CM) * math.tan(self.steering_angle)
            self.theta += math.degrees(angular_velocity * dt)

        # Update History
        if abs(self.velocity) > 0:
            self.path_history.append((self.x, self.y))

        # 4. Check Termination (Distance Traveled)
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
        
        # Calculate Steering Angle required for the fixed Turn Radius
        # tan(delta) = Wheelbase / Radius
        # delta = atan(Wheelbase / Radius)
        req_steering_rad = math.atan(ROBOT_WHEELBASE_CM / ROBOT_TURN_RADIUS_CM)
        
        # ==========================
        # 1. ARC MOVES (FL, FR, BL, BR)
        if cmd[:2] in ["FR", "FL", "BR", "BL"]:
            try:
                # === NEW PARSING LOGIC ===
                # Extract everything after the first 2 letters
                raw_suffix = cmd[2:]
                
                if '_' in raw_suffix:
                    # Old Format (e.g., "FL090_039") -> Split and take the second part
                    dist_cm = int(raw_suffix.split('_')[1])
                else:
                    # New Format (e.g., "FL39") -> The whole suffix is the distance
                    dist_cm = int(raw_suffix)
                # =========================

                self.target_val = dist_cm
                self.state = "MOVING"

                # Forward Left: Positive Speed, Positive Steer (Left)
                if cmd.startswith("FL"):
                    self.velocity = CURRENT_SPEED
                    self.steering_angle = req_steering_rad 

                # Forward Right: Positive Speed, Negative Steer (Right)
                elif cmd.startswith("FR"):
                    self.velocity = CURRENT_SPEED
                    self.steering_angle = -req_steering_rad

                # Backward Left: Negative Speed, Positive Steer (Front wheels point Left)
                elif cmd.startswith("BL"):
                    self.velocity = -CURRENT_SPEED
                    self.steering_angle = req_steering_rad

                # Backward Right: Negative Speed, Negative Steer (Front wheels point Right)
                elif cmd.startswith("BR"):
                    self.velocity = -CURRENT_SPEED
                    self.steering_angle = -req_steering_rad
                    
            except ValueError:
                print(f"Error parsing command: {cmd}")
                self.state = "IDLE"; self.current_cmd_idx += 1
            return

        # ==========================
        # 2. STRAIGHT MOVES (FW, BW)
        # ==========================
        elif cmd.startswith("FW"):
            try: val = int(cmd[2:])
            except: val = 10
            self.target_val = val; self.state = "MOVING"
            self.velocity = CURRENT_SPEED
            self.steering_angle = 0.0

        elif cmd.startswith("BW"):
            try: val = int(cmd[2:])
            except: val = 10
            self.target_val = val; self.state = "MOVING"
            self.velocity = -CURRENT_SPEED
            self.steering_angle = 0.0

        # ==========================
        # 3. UNSUPPORTED MOVES
        # ==========================
        elif cmd.startswith("TL") or cmd.startswith("TR"):
            print(f"WARNING: Ackermann steering cannot Spot Turn ({cmd}). Skipping.")
            self.state = "IDLE"; self.current_cmd_idx += 1

        # ==========================
        # 4. AUX COMMANDS
        # ==========================
        elif cmd.startswith("SNAP"):
            self.state = "SCANNING"
            self.pause_end_time = time.time() + SCAN_DURATION
        
        elif cmd.startswith("FIN"):
            self.state = "IDLE"; self.current_cmd_idx += 1
        else:
            self.state = "IDLE"; self.current_cmd_idx += 1

    # (Keep draw, draw_fov_cone, draw_path exactly as they were)
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

    # NOTE: Set initial position to match your API request (e.g., 1, 1, North)
    robot = Robot(x=15, y=15, theta=90) 
    obstacles = []
    
    start_time = None; elapsed_time = 0.0

    running = True
    while running:
        mouse_x, mouse_y = pygame.mouse.get_pos()
        keys = pygame.key.get_pressed()

        # UI BUTTONS
        setpos_btn = pygame.Rect(ARENA_WIDTH + 20, 400, 160, 40)
        start_btn = pygame.Rect(ARENA_WIDTH + 20, 450, 160, 40)
        restart_btn = pygame.Rect(ARENA_WIDTH + 20, 500, 160, 40)
        reset_btn = pygame.Rect(ARENA_WIDTH + 20, 550, 160, 40)
        manual_btn  = pygame.Rect(ARENA_WIDTH + 20, 600, 160, 40)

        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False
            
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_s and keys[pygame.K_LCTRL]:
                    save_obstacles_to_json(obstacles, 'obstacles.json')
                
                # LOAD JSON WITH TKINTER
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

                # ========== SET POSITION BUTTON ==========
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
                                
                                # LOGIC: If input is integer (1.0), center it (+5).
                                # If input is decimal (1.2), use exact.
                                if new_x.is_integer() and new_y.is_integer():
                                    robot.x = new_x * 10 + 5
                                    robot.y = new_y * 10 + 5
                                else:
                                    robot.x = new_x * 10
                                    robot.y = new_y * 10

                                robot.theta = new_dir
                                
                                # Update START position (for Restart button)
                                robot.start_x = robot.x
                                robot.start_y = robot.y
                                robot.start_theta = robot.theta
                                
                                # Clear history
                                robot.path_history = [(robot.x, robot.y)]
                                print(f"Robot moved to {robot.x/10}, {robot.y/10}, {robot.theta}")
                                
                                # Invalidate cache if position changes manually
                                if robot.has_cached_mission():
                                    robot.commands = []
                                    robot._cached_commands = []
                                    robot.ghost_path = []
                                    print("Cache cleared due to position change.")

                        except ValueError:
                            print("Invalid input")

                # ========== START BUTTON ==========
                elif start_btn.collidepoint(mouse_x, mouse_y):
                    for ob in obstacles: ob.is_scanned = False
                    
                    # 1. Try Cached Mission
                    if robot.has_cached_mission():
                        print("Replaying cached mission...")
                        robot.start_from_cache()
                        start_time = time.time()
                    
                    # 2. Fetch New Mission
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
                
                # ========== RESTART BUTTON ==========
                elif restart_btn.collidepoint(mouse_x, mouse_y):
                    robot.reset_position()
                    start_time = None; elapsed_time = 0.0
                    for ob in obstacles: ob.is_scanned = False
                    print("Restarted at current mission start.")

                # ========== RESET BUTTON ==========
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
                        robot.reset_all() # Clear cache if obstacles change

                # ========== MANUAL CMD BUTTON ==========
                elif manual_btn.collidepoint(mouse_x, mouse_y):
                    root = tkinter.Tk()
                    root.withdraw()
                    # Ask user for input (e.g., "FW20" or "FL39")
                    cmd_input = simpledialog.askstring("Manual Command", "Enter Command (e.g. FW20, FL39, SNAP1_C):")
                    root.destroy()

                    if cmd_input:
                        # Clean up input: Uppercase and remove spaces
                        clean_cmd = cmd_input.strip().upper().replace(" ", "")
                        robot.execute_manual_command(clean_cmd)

        if robot.is_running:
            scanned_id = robot.update()
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
        screen.blit(font.render("Group 24 SIMULATOR 2", True, BLUE), (ARENA_WIDTH + 10, 80))

        # Cache Status
        if robot.has_cached_mission():
            screen.blit(font.render("Mission CACHED", True, (0, 128, 0)), (ARENA_WIDTH + 20, 240))
        else:
            screen.blit(font.render("No cached mission", True, GRAY), (ARENA_WIDTH + 20, 240))

        # Buttons
        pygame.draw.rect(screen, (0, 200, 255), setpos_btn)
        pygame.draw.rect(screen, BLACK, setpos_btn, 2)
        screen.blit(font.render("SET POS", True, BLACK), (ARENA_WIDTH + 50, 410))

        btn_color = (100, 255, 100) if robot.has_cached_mission() else (GREEN if len(obstacles)>=1 else LIGHT_GRAY)
        pygame.draw.rect(screen, btn_color, start_btn)
        pygame.draw.rect(screen, BLACK, start_btn, 2)
        start_label = "GO (cached)" if robot.has_cached_mission() else "START"
        screen.blit(font.render(start_label, True, BLACK), (ARENA_WIDTH + 40, 460))
        
        pygame.draw.rect(screen, YELLOW, restart_btn)
        pygame.draw.rect(screen, BLACK, restart_btn, 2)
        screen.blit(font.render("RESTART", True, BLACK), (ARENA_WIDTH + 45, 510))

        pygame.draw.rect(screen, RED, reset_btn)
        pygame.draw.rect(screen, BLACK, reset_btn, 2)
        screen.blit(font.render("RESET", True, WHITE), (ARENA_WIDTH + 55, 560))

        # Draw Manual Button
        pygame.draw.rect(screen, PURPLE, manual_btn)
        pygame.draw.rect(screen, BLACK, manual_btn, 2)
        screen.blit(font.render("MANUAL CMD", True, WHITE), (ARENA_WIDTH + 30, 610))

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()