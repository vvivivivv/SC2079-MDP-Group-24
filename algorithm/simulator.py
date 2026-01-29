import pygame
import math
import sys
import json
import time

# Import path planning module
from simulator_adapter import plan_path, RobotConfig
from simulator_adapter import Obstacle as PathObstacle


# =============================================================================
# 1. CONFIGURATION & CONSTANTS
# =============================================================================

# Visual Settings
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
AUTO_MOVE_SPEED = 2  # Speed when following computed path

# Collision Settings
VIRTUAL_OBSTACLE_SIZE = 40  # cm (obstacle + safety margin)

# Colors (R, G, B)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRAY = (200, 200, 200)
LIGHT_GRAY = (180, 180, 180)
DARK_GRAY = (100, 100, 100)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
RED = (255, 0, 0)
YELLOW = (255, 255, 0)
ORANGE = (255, 165, 0)
PURPLE = (128, 0, 128)


# =============================================================================
# 2. HELPER FUNCTIONS (Coordinate Math)
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


# =============================================================================
# 3. JSON LOAD/SAVE FUNCTIONS
# =============================================================================

def load_obstacles_from_json(filepath):
    """Load obstacles from a JSON file (for simulation UI)"""
    try:
        with open(filepath, 'r') as f:
            data = json.load(f)

        obstacles = []
        max_id = 0
        for obs_data in data.get('obstacles', []):
            x = obs_data.get('x', 0)
            y = obs_data.get('y', 0)
            direction = obs_data.get('direction', 0)
            obs_id = obs_data.get('id', None)

            obstacles.append(Obstacle(x, y, direction, obs_id))

            if obs_id and obs_id > max_id:
                max_id = obs_id

        if max_id > 0:
            Obstacle._id_counter = max_id + 1

        print(f"✓ Loaded {len(obstacles)} obstacles from {filepath}")
        return obstacles

    except FileNotFoundError:
        print(f"✗ File not found: {filepath}")
        return []
    except json.JSONDecodeError as e:
        print(f"✗ Error parsing JSON: {e}")
        return []

def save_obstacles_to_json(obstacles, filepath):
    """Save current obstacles to a JSON file"""
    data = {
        "obstacles": [
            {
                "id": obs.id,
                "x": obs.col,
                "y": obs.row,
                "direction": obs.direction
            }
            for obs in obstacles
        ]
    }

    try:
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)
        print(f"✓ Saved {len(obstacles)} obstacles to {filepath}")
    except Exception as e:
        print(f"✗ Error saving obstacles: {e}")

def reset_simulation(robot, obstacles):
    """Reset robot to default position and clear all obstacles"""
    robot.x = 15
    robot.y = 15
    robot.theta = 90
    robot.is_auto = False
    robot.path_segments = []
    robot.current_segment = 0
    robot.path_history = [(15, 15)]
    obstacles.clear()
    Obstacle._id_counter = 1
    print("✓ Simulation reset")


# =============================================================================
# 4. CLASSES
# =============================================================================

class Obstacle:
    _id_counter = 1

    def __init__(self, col, row, direction=0, obstacle_id=None):
        self.col = col
        self.row = row
        self.direction = direction

        if obstacle_id is None:
            self.id = Obstacle._id_counter
            Obstacle._id_counter += 1
        else:
            self.id = obstacle_id

    def draw_safety_zone(self, surface):
        """Draw a 20cm camera positioning zone around the obstacle"""
        safety_margin_cm = 20
        x_cm = (self.col * CELL_SIZE_CM) - safety_margin_cm
        y_cm = ((self.row + 1) * CELL_SIZE_CM) + safety_margin_cm

        px_x, px_y = grid_to_pixel(x_cm, y_cm)

        scale = ARENA_WIDTH / GRID_SIZE
        safety_size_px = int((50 / GRID_SIZE) * ARENA_WIDTH)

        rect = pygame.Rect(px_x, px_y, safety_size_px, safety_size_px)

        dash_length = 5
        gap_length = 3

        self._draw_dashed_line(surface, (rect.left, rect.top), (rect.right, rect.top), 
                            LIGHT_GRAY, dash_length, gap_length)
        self._draw_dashed_line(surface, (rect.right, rect.top), (rect.right, rect.bottom), 
                            LIGHT_GRAY, dash_length, gap_length)
        self._draw_dashed_line(surface, (rect.right, rect.bottom), (rect.left, rect.bottom), 
                            LIGHT_GRAY, dash_length, gap_length)
        self._draw_dashed_line(surface, (rect.left, rect.bottom), (rect.left, rect.top), 
                            LIGHT_GRAY, dash_length, gap_length)

    def draw_virtual_obstacle(self, surface):
        """Draw the virtual obstacle zone (40cm x 40cm) for collision visualization"""
        obs_center_x = self.col * 10 + 5
        obs_center_y = self.row * 10 + 5

        virtual_half = VIRTUAL_OBSTACLE_SIZE / 2

        x_cm = obs_center_x - virtual_half
        y_cm = obs_center_y + virtual_half

        px_x, px_y = grid_to_pixel(x_cm, y_cm)

        scale = ARENA_WIDTH / GRID_SIZE
        virtual_size_px = int((VIRTUAL_OBSTACLE_SIZE / GRID_SIZE) * ARENA_WIDTH)

        rect = pygame.Rect(px_x, px_y, virtual_size_px, virtual_size_px)

        # Draw semi-transparent red overlay
        s = pygame.Surface((virtual_size_px, virtual_size_px))
        s.set_alpha(30)
        s.fill(RED)
        surface.blit(s, (px_x, px_y))

        # Draw border
        pygame.draw.rect(surface, RED, rect, 1)

    def _draw_dashed_line(self, surface, start, end, color, dash_length, gap_length):
        """Helper to draw a dashed line"""
        x1, y1 = start
        x2, y2 = end

        dx = x2 - x1
        dy = y2 - y1
        distance = math.sqrt(dx*dx + dy*dy)

        if distance == 0:
            return

        dx = dx / distance
        dy = dy / distance

        pos = 0
        while pos < distance:
            start_x = x1 + dx * pos
            start_y = y1 + dy * pos
            end_pos = min(pos + dash_length, distance)
            end_x = x1 + dx * end_pos
            end_y = y1 + dy * end_pos

            pygame.draw.line(surface, color, (start_x, start_y), (end_x, end_y), 2)
            pos += dash_length + gap_length

    def draw(self, surface):
        x_cm = self.col * CELL_SIZE_CM
        y_cm = (self.row + 1) * CELL_SIZE_CM 
        px_x, px_y = grid_to_pixel(x_cm, y_cm)

        size_px = int((CELL_SIZE_CM / GRID_SIZE) * ARENA_WIDTH)

        rect = pygame.Rect(px_x, px_y, size_px, size_px)
        pygame.draw.rect(surface, RED, rect)
        pygame.draw.rect(surface, BLACK, rect, 1)

        # Draw ID in center
        font = pygame.font.SysFont(None, 20)
        id_text = font.render(str(self.id), True, WHITE)
        text_rect = id_text.get_rect(center=(px_x + size_px//2, px_y + size_px//2))
        surface.blit(id_text, text_rect)

        # Draw "Image Face" indicator
        if self.direction == 0:
            start_pos = (px_x, px_y)
            end_pos = (px_x + size_px, px_y)
        elif self.direction == 90:
            start_pos = (px_x + size_px, px_y)
            end_pos = (px_x + size_px, px_y + size_px)
        elif self.direction == 180:
            start_pos = (px_x, px_y + size_px)
            end_pos = (px_x + size_px, px_y + size_px)
        else:
            start_pos = (px_x, px_y)
            end_pos = (px_x, px_y + size_px)

        pygame.draw.line(surface, BLACK, start_pos, end_pos, 4)


class Robot:
    def __init__(self, x, y, theta):
        self.x = x  # Center position
        self.y = y  # Center position
        self.theta = theta  # Degrees

        # Path following
        self.is_auto = False
        self.path_segments = []
        self.current_segment = 0
        self.target_config = None
        self.path_history = [(x, y)]  # Track path for visualization

        scale = ARENA_WIDTH / GRID_SIZE
        size_px = int(ROBOT_SIZE_CM * scale)
        self.original_surf = pygame.Surface((size_px, size_px), pygame.SRCALPHA)
        self.original_surf.fill(BLUE)

        w, h = size_px, size_px
        pygame.draw.line(self.original_surf, YELLOW, (w/2, h/2), (w, h/2), 3)

    def start_auto_path(self, path_result):
        """Start automatic path following"""
        self.is_auto = True
        self.path_segments = path_result['segments']
        self.current_segment = 0
        self.path_history = [(self.x, self.y)]

        if self.path_segments:
            self.target_config = self.path_segments[0]['to']
            print(f"Starting path: {path_result['order']}")
            print(f"Total distance: {path_result['total_length']:.2f} cm")

    def update_auto(self):
        """
        Update automatic movement.
        Path is already collision-free from planning phase, so just follow it.
        """
        if not self.is_auto or self.current_segment >= len(self.path_segments):
            self.is_auto = False
            return False

        target = self.target_config

        # Calculate distance and angle to target
        dx = target.x - self.x
        dy = target.y - self.y
        distance = math.sqrt(dx*dx + dy*dy)

        # Check if reached target
        if distance < 2:  # Within 2cm
            self.current_segment += 1
            if self.current_segment < len(self.path_segments):
                self.target_config = self.path_segments[self.current_segment]['to']
                print(f"✓ Reached obstacle {self.path_segments[self.current_segment-1]['obstacle_id']}")
            else:
                self.is_auto = False
                print("✓ Path complete!")
            return True

        # Calculate movement
        angle_to_target = math.atan2(dy, dx)
        target_theta_deg = math.degrees(angle_to_target)
        angle_diff = target_theta_deg - self.theta

        # Normalize angle difference
        while angle_diff > 180:
            angle_diff -= 360
        while angle_diff < -180:
            angle_diff += 360

        # Rotate towards target
        if abs(angle_diff) > 2:
            rotation_step = math.copysign(min(5, abs(angle_diff)), angle_diff)
            self.theta += rotation_step

        # Move forward if aligned
        # Path is already collision-free, so we can trust it!
        if abs(angle_diff) < 30:
            rad = math.radians(self.theta)
            move_dist = min(AUTO_MOVE_SPEED, distance)
            self.x += move_dist * math.cos(rad)
            self.y += move_dist * math.sin(rad)
            self.path_history.append((self.x, self.y))

        return True

    def draw(self, surface):
        px, py = grid_to_pixel(self.x, self.y)
        rotated_surf = pygame.transform.rotate(self.original_surf, self.theta)
        rect = rotated_surf.get_rect(center=(px, py))
        surface.blit(rotated_surf, rect.topleft)

    def draw_path(self, surface):
        """Draw the path history"""
        if len(self.path_history) < 2:
            return

        # Draw path line
        points = [grid_to_pixel(x, y) for x, y in self.path_history]
        if len(points) > 1:
            pygame.draw.lines(surface, PURPLE, False, points, 2)


# =============================================================================
# 5. MAIN LOOP
# =============================================================================

def main():
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption("MDP Group 14 Simulator")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont(None, 24)

    # Init State
    robot = Robot(x=15, y=15, theta=90)
    obstacles = []

    # Timer
    start_time = None
    elapsed_time = 0.0

    # Visualization toggle
    show_virtual_obstacles = False

    running = True
    while running:
        keys = pygame.key.get_pressed()

        # --- 1. EVENTS ---
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            # KEYBOARD SHORTCUTS
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_s and keys[pygame.K_LCTRL]:
                    save_obstacles_to_json(obstacles, 'obstacles.json')
                elif event.key == pygame.K_l and keys[pygame.K_LCTRL]:
                    obstacles = load_obstacles_from_json('obstacles.json')
                elif event.key == pygame.K_r and keys[pygame.K_LCTRL]:
                    reset_simulation(robot, obstacles)
                    start_time = None
                    elapsed_time = 0.0
                elif event.key == pygame.K_v:
                    # Toggle virtual obstacle visualization
                    show_virtual_obstacles = not show_virtual_obstacles
                    print(f"Virtual obstacles: {'ON' if show_virtual_obstacles else 'OFF'}")

            # MOUSE EVENTS
            if event.type == pygame.MOUSEBUTTONDOWN:
                mouse_x, mouse_y = pygame.mouse.get_pos()

                # Check if START button was clicked
                start_button_rect = pygame.Rect(ARENA_WIDTH + 20, 450, 160, 40)
                if start_button_rect.collidepoint(mouse_x, mouse_y) and len(obstacles) >= 5:
                    # Convert obstacles to path planning format
                    path_obstacles = []
                    for obs in obstacles:
                        path_obstacles.append(PathObstacle(  # ✅ Use PathObstacle
                            id=obs.id,
                            x=obs.col,
                            y=obs.row,
                            direction=obs.direction
                        ))

                    # Compute collision-free path
                    start_config = RobotConfig(
                        x=robot.x,
                        y=robot.y,
                        theta=math.radians(robot.theta)
                    )

                    print("\n" + "="*50)
                    print("COMPUTING COLLISION-FREE PATH...")
                    path_result = plan_path(start_config, path_obstacles, use_exhaustive=True)

                    if 'error' in path_result:
                        print(f"✗ {path_result['error']}")
                        print("="*50 + "\n")
                        continue

                    print("✓ Path computed successfully!")
                    print("="*50 + "\n")

                    # Start robot on path
                    robot.start_auto_path(path_result)
                    start_time = time.time()
                    continue

                # Check if Reset button was clicked
                reset_button_rect = pygame.Rect(ARENA_WIDTH + 20, 500, 160, 40)
                if reset_button_rect.collidepoint(mouse_x, mouse_y):
                    reset_simulation(robot, obstacles)
                    start_time = None
                    elapsed_time = 0.0
                    continue

                # Handle obstacle placement
                grid_pos = pixel_to_grid(mouse_x, mouse_y)

                if grid_pos:
                    c, r = grid_pos
                    existing = next((o for o in obstacles if o.col == c and o.row == r), None)

                    if existing:
                        if event.button == 1:
                            existing.direction = (existing.direction + 90) % 360
                        elif event.button == 3:
                            obstacles.remove(existing)
                    else:
                        if event.button == 1:
                            obstacles.append(Obstacle(c, r))

        # --- 2. UPDATE ---
        if robot.is_auto:
            robot.update_auto()

        # Update timer
        if start_time is not None and robot.is_auto:
            elapsed_time = time.time() - start_time

        # --- 3. DRAW ---
        screen.fill(WHITE)

        # A. Draw Grid Lines
        for i in range(GRID_COLS + 1):
            start = grid_to_pixel(i * 10, 0)
            end = grid_to_pixel(i * 10, GRID_SIZE)
            pygame.draw.line(screen, GRAY, start, end)
            start = grid_to_pixel(0, i * 10)
            end = grid_to_pixel(GRID_SIZE, i * 10)
            pygame.draw.line(screen, GRAY, start, end)

        # B. Draw Start Zone
        sz_scale = ARENA_WIDTH / GRID_SIZE
        pygame.draw.rect(screen, GREEN, 
                        (0, WINDOW_HEIGHT - (30 * sz_scale), 30 * sz_scale, 30 * sz_scale))

        # C. Draw Obstacles
        for obs in obstacles:
            if show_virtual_obstacles:
                obs.draw_virtual_obstacle(screen)
            obs.draw_safety_zone(screen)
            obs.draw(screen)

        # D. Draw Robot Path
        robot.draw_path(screen)

        # E. Draw Robot
        robot.draw(screen)

        # F. Draw Dashboard
        pygame.draw.rect(screen, GRAY, (ARENA_WIDTH, 0, WINDOW_WIDTH-ARENA_WIDTH, WINDOW_HEIGHT))

        # Robot State
        text_x = f"X: {robot.x:.1f} cm"
        text_y = f"Y: {robot.y:.1f} cm"
        text_t = f"Angle: {int(robot.theta)} deg"

        screen.blit(font.render("ROBOT STATE", True, BLACK), (ARENA_WIDTH + 20, 20))
        screen.blit(font.render(text_x, True, BLACK), (ARENA_WIDTH + 20, 50))
        screen.blit(font.render(text_y, True, BLACK), (ARENA_WIDTH + 20, 80))
        screen.blit(font.render(text_t, True, BLACK), (ARENA_WIDTH + 20, 110))
        screen.blit(font.render(f"Obstacles: {len(obstacles)}", True, BLACK), (ARENA_WIDTH + 20, 140))

        # Timer
        timer_text = f"Time: {elapsed_time:.2f}s"
        screen.blit(font.render(timer_text, True, BLACK), (ARENA_WIDTH + 20, 170))

        # Status
        if robot.is_auto:
            status_color = ORANGE
            screen.blit(font.render("Status: RUNNING", True, status_color), (ARENA_WIDTH + 20, 200))
        else:
            screen.blit(font.render("Status: READY", True, DARK_GRAY), (ARENA_WIDTH + 20, 200))

        # Controls
        screen.blit(font.render("CONTROLS:", True, BLACK), (ARENA_WIDTH + 20, 250))
        screen.blit(font.render("L-Click: Add/Rotate Obs", True, BLACK), (ARENA_WIDTH + 20, 280))
        screen.blit(font.render("R-Click: Delete Obs", True, BLACK), (ARENA_WIDTH + 20, 310))
        screen.blit(font.render("Ctrl+S: Save to JSON", True, BLACK), (ARENA_WIDTH + 20, 340))
        screen.blit(font.render("Ctrl+L: Load from JSON", True, BLACK), (ARENA_WIDTH + 20, 370))
        screen.blit(font.render("V: Toggle Collision View", True, BLACK), (ARENA_WIDTH + 20, 400))

        # START Button
        start_button_rect = pygame.Rect(ARENA_WIDTH + 20, 450, 160, 40)
        button_color = GREEN if len(obstacles) >= 5 else LIGHT_GRAY
        pygame.draw.rect(screen, button_color, start_button_rect)
        pygame.draw.rect(screen, BLACK, start_button_rect, 2)
        start_text = font.render("START", True, BLACK)
        text_rect = start_text.get_rect(center=start_button_rect.center)
        screen.blit(start_text, text_rect)

        if len(obstacles) < 5:
            hint_font = pygame.font.SysFont(None, 18)
            hint = hint_font.render("(Need 5 obstacles)", True, RED)
            screen.blit(hint, (ARENA_WIDTH + 25, 495))

        # Reset Button
        reset_button_rect = pygame.Rect(ARENA_WIDTH + 20, 500, 160, 40)
        pygame.draw.rect(screen, RED, reset_button_rect)
        pygame.draw.rect(screen, BLACK, reset_button_rect, 2)
        reset_text = font.render("RESET (Ctrl+R)", True, WHITE)
        text_rect = reset_text.get_rect(center=reset_button_rect.center)
        screen.blit(reset_text, text_rect)

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
