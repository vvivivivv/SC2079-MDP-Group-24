import pygame
import math
import sys
import json
import time
import requests
import tkinter
from tkinter import filedialog

# =============================================================================
# 1. LIVE TUNING VARIABLES (GLOBALS)
# =============================================================================
CURRENT_FWD_RATIO = 0.38
CURRENT_REV_RATIO = 0.50
CURRENT_SPEED = 30.0

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
ROBOT_AXLE_TRACK = 16.0
SCAN_DURATION = 3.0

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRAY = (200, 200, 200)
LIGHT_BLUE = (173, 216, 230)
LIGHT_GRAY = (192, 192, 192)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
RED = (255, 0, 0)
YELLOW = (255, 255, 0)
PURPLE = (128, 0, 128)
GHOST_COLOR = (200, 200, 255)

# FOV Cone settings (Pi Camera v2.1)
FOV_HALF_ANGLE = 31.1
FOV_RANGE_CM = 30
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
        self.is_running = False; self.commands = []
        self.ghost_path = []; self.path_history = [(x, y)]
        self.current_cmd_idx = 0
        self.state = "IDLE"
        self.target_val = 0; self.traveled_val = 0
        self.pause_end_time = 0
        self.vl = 0.0; self.vr = 0.0

        scale = ARENA_WIDTH / GRID_SIZE
        size_px = int(ROBOT_SIZE_CM * scale)
        self.original_surf = pygame.Surface((size_px, size_px), pygame.SRCALPHA)
        self.original_surf.fill(BLUE)
        w, h = size_px, size_px
        pygame.draw.line(self.original_surf, YELLOW, (w/2, h/2), (w, h/2), 3)

    def reset_position(self):
        """Reset robot to start position. Does NOT clear commands/ghost_path (cache kept)."""
        self.x = self.start_x; self.y = self.start_y; self.theta = self.start_theta
        self.is_running = False
        self.path_history = [(self.start_x, self.start_y)]
        self.current_cmd_idx = 0; self.state = "IDLE"
        self.vl = 0; self.vr = 0

    def reset_all(self):
        """Full reset: clear position, commands, ghost path, everything."""
        self.reset_position()
        self.commands = []
        self.ghost_path = []

    def has_cached_mission(self):
        """Check if there's a cached mission ready to replay."""
        return len(self.commands) > 0

    def start_from_cache(self):
        """Start mission using cached commands (no API call needed)."""
        self.reset_position()
        # Rebuild commands from cache - we need a fresh copy since
        # FR/BL splits insert into the list during execution
        self.commands = list(self._cached_commands)
        self.is_running = True
        self.current_cmd_idx = 0
        print(f"Starting from cache ({len(self.commands)} commands)")

    def start_mission(self, api_data):
        """Start mission from fresh API data and cache it."""
        self.commands = api_data.get('commands', [])
        # Store a pristine copy of commands before execution modifies the list
        self._cached_commands = list(self.commands)
        raw_path = api_data.get('path', [])
        self.ghost_path = []
        for p in raw_path:
            self.ghost_path.append((p['x'] * 10 + 5, p['y'] * 10 + 5))
        self.is_running = True; self.current_cmd_idx = 0

    def update(self):
        if not self.is_running: return None
        dt = 1.0 / FPS

        if self.state == "SCANNING":
            if time.time() > self.pause_end_time:
                self.state = "IDLE"
                current_cmd = self.commands[self.current_cmd_idx]
                parts = current_cmd.replace("SNAP", "").split("_")
                self.current_cmd_idx += 1
                return int(parts[0])
            else:
                return None

        if self.state == "IDLE":
            if self.current_cmd_idx >= len(self.commands):
                self.is_running = False
                print("Mission Complete")
                return None
            cmd = self.commands[self.current_cmd_idx]
            self._decode_command(cmd)

        # Physics
        v = (self.vl + self.vr) / 2.0
        omega = (self.vr - self.vl) / ROBOT_AXLE_TRACK

        self.x += v * math.cos(math.radians(self.theta)) * dt
        self.y += v * math.sin(math.radians(self.theta)) * dt
        self.theta += math.degrees(omega * dt)

        if abs(v) > 0 or abs(omega) > 0:
            self.path_history.append((self.x, self.y))

        if self.state == "MOVING":
            self.traveled_val += abs(v * dt)
            if self.traveled_val >= self.target_val:
                self._stop_motors()
        elif self.state == "TURNING":
            self.traveled_val += abs(math.degrees(omega * dt))
            if self.traveled_val >= self.target_val:
                # Only snap to 90 for grid turns, NOT micro-turns (TL/TR)
                is_micro_turn = False
                if self.current_cmd_idx < len(self.commands):
                    cur = self.commands[self.current_cmd_idx]
                    if cur.startswith("TL") or cur.startswith("TR"):
                        is_micro_turn = True
                self._stop_motors()
                if not is_micro_turn:
                    self.theta = round(self.theta / 90) * 90
        return None

    def _stop_motors(self):
        self.vl = 0; self.vr = 0
        self.state = "IDLE"; self.current_cmd_idx += 1

    def _decode_command(self, cmd):
            global CURRENT_SPEED, CURRENT_FWD_RATIO, CURRENT_REV_RATIO
            print(f"Processing: {cmd}")
            self.traveled_val = 0

            # 1. HELPER: EXTENSION MOVES
            if cmd == "FW20":
                self.target_val = 20; self.state = "MOVING"
                self.vl = CURRENT_SPEED; self.vr = CURRENT_SPEED
                return

            elif cmd == "BW20":
                self.target_val = 20; self.state = "MOVING"
                self.vl = -CURRENT_SPEED; self.vr = -CURRENT_SPEED
                return

            # 2. HELPER: TIGHT TURNS
            elif cmd == "FR_TIGHT":
                self.target_val = 90; self.state = "TURNING"
                self.vl = CURRENT_SPEED; self.vr = CURRENT_SPEED * 0.10
                return

            elif cmd == "FL_TIGHT":
                self.target_val = 90; self.state = "TURNING"
                self.vl = CURRENT_SPEED * 0.10; self.vr = CURRENT_SPEED
                return

            elif cmd == "BR_TIGHT":
                self.target_val = 90; self.state = "TURNING"
                self.vl = -CURRENT_SPEED; self.vr = -CURRENT_SPEED * 0.10
                return

            elif cmd == "BL_TIGHT":
                self.target_val = 90; self.state = "TURNING"
                self.vl = -CURRENT_SPEED * 0.10; self.vr = -CURRENT_SPEED
                return

            # 3. STANDARD COMMANDS
            elif cmd.startswith("FR"):
                print("Splitting FR -> FR_TIGHT + FW20")
                self.target_val = 90; self.state = "TURNING"
                self.vl = CURRENT_SPEED; self.vr = CURRENT_SPEED * 0.10
                self.commands.insert(self.current_cmd_idx + 1, "FW20")

            elif cmd.startswith("FL"):
                print("Splitting FL -> FL_TIGHT + FW20")
                self.target_val = 90; self.state = "TURNING"
                self.vl = CURRENT_SPEED * 0.10; self.vr = CURRENT_SPEED
                self.commands.insert(self.current_cmd_idx + 1, "FW20")

            elif cmd.startswith("BR"):
                print("Splitting BR -> BW20 + BR_TIGHT")
                self.target_val = 20; self.state = "MOVING"
                self.vl = -CURRENT_SPEED; self.vr = -CURRENT_SPEED
                self.commands.insert(self.current_cmd_idx + 1, "BR_TIGHT")

            elif cmd.startswith("BL"):
                print("Splitting BL -> BW20 + BL_TIGHT")
                self.target_val = 20; self.state = "MOVING"
                self.vl = -CURRENT_SPEED; self.vr = -CURRENT_SPEED
                self.commands.insert(self.current_cmd_idx + 1, "BL_TIGHT")

            # 4. BASIC MOVES
            elif cmd.startswith("FW"):
                try: val = int(cmd[2:])
                except: val = 10
                self.target_val = val; self.state = "MOVING"
                self.vl = CURRENT_SPEED; self.vr = CURRENT_SPEED

            elif cmd.startswith("BW"):
                try: val = int(cmd[2:])
                except: val = 10
                self.target_val = val; self.state = "MOVING"
                self.vl = -CURRENT_SPEED; self.vr = -CURRENT_SPEED

            elif cmd.startswith("SNAP"):
                self.state = "SCANNING"
                self.pause_end_time = time.time() + SCAN_DURATION

            # 5. MICRO-TURN COMMANDS
            elif cmd.startswith("TL"):
                try: val = int(cmd[2:])
                except: val = 0
                if val > 0:
                    self.target_val = val; self.state = "TURNING"
                    self.vl = -CURRENT_SPEED * 0.5; self.vr = CURRENT_SPEED * 0.5
                else:
                    self.state = "IDLE"; self.current_cmd_idx += 1

            elif cmd.startswith("TR"):
                try: val = int(cmd[2:])
                except: val = 0
                if val > 0:
                    self.target_val = val; self.state = "TURNING"
                    self.vl = CURRENT_SPEED * 0.5; self.vr = -CURRENT_SPEED * 0.5
                else:
                    self.state = "IDLE"; self.current_cmd_idx += 1

            elif cmd.startswith("FIN"):
                self.state = "IDLE"; self.current_cmd_idx += 1

            else:
                print(f"Warning: UNKNOWN COMMAND: {cmd} - Skipping")
                self.state = "IDLE"; self.current_cmd_idx += 1

    def draw(self, surface):
        px, py = grid_to_pixel(self.x, self.y)
        rotated_surf = pygame.transform.rotate(self.original_surf, self.theta)
        rect = rotated_surf.get_rect(center=(px, py))
        surface.blit(rotated_surf, rect.topleft)

    def draw_fov_cone(self, surface):
        px, py = grid_to_pixel(self.x, self.y)
        scale = ARENA_WIDTH / GRID_SIZE
        range_px = int(FOV_RANGE_CM * scale)

        theta_rad = math.radians(self.theta)
        half_fov_rad = math.radians(FOV_HALF_ANGLE)

        left_angle = theta_rad + half_fov_rad
        right_angle = theta_rad - half_fov_rad

        cone_surface = pygame.Surface((ARENA_WIDTH, WINDOW_HEIGHT), pygame.SRCALPHA)
        num_segments = 20
        points = [(px, py)]
        for i in range(num_segments + 1):
            angle = right_angle + (left_angle - right_angle) * i / num_segments
            x = px + range_px * math.cos(angle)
            y = py - range_px * math.sin(angle)
            points.append((int(x), int(y)))

        if len(points) >= 3:
            pygame.draw.polygon(cone_surface, FOV_COLOR, points)
            pygame.draw.polygon(cone_surface, (255, 200, 0, 120), points, 2)

        surface.blit(cone_surface, (0, 0))

    def draw_path(self, surface):
        if len(self.ghost_path) > 1:
             points = [grid_to_pixel(x, y) for x, y in self.ghost_path]
             pygame.draw.lines(surface, GHOST_COLOR, False, points, 1)
        if len(self.path_history) < 2: return
        points = [grid_to_pixel(x, y) for x, y in self.path_history]
        if len(points) > 1:
            pygame.draw.lines(surface, PURPLE, False, points, 2)

# =============================================================================
# 5. MAIN LOOP
# =============================================================================
def main():
    global CURRENT_FWD_RATIO, CURRENT_REV_RATIO, CURRENT_SPEED
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

        start_btn = pygame.Rect(ARENA_WIDTH + 20, 450, 160, 40)
        restart_btn = pygame.Rect(ARENA_WIDTH + 20, 500, 160, 40)
        reset_btn = pygame.Rect(ARENA_WIDTH + 20, 550, 160, 40)

        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_s and keys[pygame.K_LCTRL]:
                    save_obstacles_to_json(obstacles, 'obstacles.json')
                elif event.key == pygame.K_l and keys[pygame.K_LCTRL]:
                    # 1. Initialize Tkinter and hide the main window (we only want the popup)
                    root = tkinter.Tk()
                    root.withdraw() 

                    # 2. Open the file dialog
                    file_path = filedialog.askopenfilename(
                        title="Select Obstacles JSON",
                        filetypes=[("JSON Files", "*.json"), ("All Files", "*.*")]
                    )
                    
                    # 3. Clean up Tkinter
                    root.destroy()

                    # 4. If the user selected a file (didn't press Cancel)
                    if file_path:
                        print(f"Loading map from: {file_path}")
                        # Clear existing obstacles/cache to prevent conflicts
                        robot.reset_all() 
                        obstacles = load_obstacles_from_json(file_path)

            if event.type == pygame.MOUSEBUTTONDOWN:
                # ========== START BUTTON ==========
                if start_btn.collidepoint(mouse_x, mouse_y):
                    for ob in obstacles: ob.is_scanned = False

                    # If we have a cached mission, replay it without API call
                    if robot.has_cached_mission():
                        print("Replaying cached mission...")
                        robot.start_from_cache()
                        start_time = time.time()
                    else:
                        # No cache — fetch from API
                        robot_dir_api = 0; norm_theta = int(robot.theta) % 360
                        if norm_theta == 90: robot_dir_api = 0
                        elif norm_theta == 0: robot_dir_api = 2
                        elif norm_theta == 270: robot_dir_api = 4
                        elif norm_theta == 180: robot_dir_api = 6

                        payload = { "obstacles": [], "retrying": False, "robot_x": 1, "robot_y": 1, "robot_dir": robot_dir_api }
                        for ob in obstacles:
                            d_api = 0
                            if ob.direction == 0: d_api = 0
                            elif ob.direction == 90: d_api = 2
                            elif ob.direction == 180: d_api = 4
                            elif ob.direction == 270: d_api = 6
                            payload["obstacles"].append({ "x": ob.col, "y": ob.row, "id": ob.id, "d": d_api })

                        print("Sending request to API...")
                        try:
                            r = requests.post(API_URL, json=payload)
                            print(payload)
                            if r.status_code == 200:
                                data = r.json()
                                if data.get('error'): print(f"Error: {data['error']}")
                                else:
                                    robot.start_mission(data['data'])
                                    start_time = time.time()
                                    print(f"Mission cached ({len(robot.commands)} commands)")
                            else: print(f"Status {r.status_code}")
                        except Exception as e: print(f"Failed: {e}")

                # ========== RESTART BUTTON ==========
                # Reset robot to start position, keep cached mission & obstacles
                elif restart_btn.collidepoint(mouse_x, mouse_y):
                    robot.reset_position()
                    start_time = None; elapsed_time = 0.0
                    for ob in obstacles: ob.is_scanned = False
                    if robot.has_cached_mission():
                        print("Robot reset to start. Press START to replay cached mission.")
                    else:
                        print("Robot reset to start. No cached mission — press START to compute.")

                # ========== RESET BUTTON ==========
                # Full reset: clear obstacles, path, cache, everything
                elif reset_btn.collidepoint(mouse_x, mouse_y):
                    robot.reset_all()
                    obstacles.clear(); Obstacle._id_counter = 1
                    start_time = None; elapsed_time = 0.0
                    print("Full reset: obstacles, cache, and paths cleared.")

                elif pixel_to_grid(mouse_x, mouse_y):
                    c, r = pixel_to_grid(mouse_x, mouse_y)
                    existing = next((o for o in obstacles if o.col == c and o.row == r), None)
                    if existing:
                        if event.button == 1: existing.direction = (existing.direction + 90) % 360
                        elif event.button == 3: obstacles.remove(existing)
                    else:
                        if event.button == 1: obstacles.append(Obstacle(c, r))
                    # Invalidate cache when obstacles change
                    if robot.has_cached_mission():
                        robot.reset_all()
                        print("Obstacles changed — cache cleared. Press START to recompute.")

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

        screen.blit(font.render("Group 24 SIMULATOR", True, BLUE), (ARENA_WIDTH + 10, 80))

        pygame.draw.line(screen, GHOST_COLOR, (ARENA_WIDTH+20, 170), (ARENA_WIDTH+50, 170), 2)
        screen.blit(font.render("Ideal Path", True, BLACK), (ARENA_WIDTH + 60, 165))
        pygame.draw.line(screen, PURPLE, (ARENA_WIDTH+20, 200), (ARENA_WIDTH+50, 200), 2)
        screen.blit(font.render("Virtual Robot", True, BLACK), (ARENA_WIDTH + 60, 195))

        # Cache status indicator
        if robot.has_cached_mission():
            screen.blit(font.render("Mission CACHED", True, (0, 128, 0)), (ARENA_WIDTH + 20, 240))
        else:
            screen.blit(font.render("No cached mission", True, GRAY), (ARENA_WIDTH + 20, 240))

        # Buttons - START shows different color when cached
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

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()