"""
MDP Simulator - Path Planning Visualization
Displays robot movement area, obstacles, and path planning results.
"""

import pygame
import math
import sys
import json
import time

from path_planning import (
    RobotConfig, 
    Obstacle as PathObstacle,
    plan_path,
    load_obstacles_from_json as load_path_obstacles,
    ARENA_SIZE,
    ROBOT_SIZE,
    COLLISION_DISTANCE,
    TURNING_RADIUS
)


# =============================================================================
# CONFIGURATION
# =============================================================================

WINDOW_WIDTH = 1000
WINDOW_HEIGHT = 800
ARENA_PIXELS = 800
FPS = 60

GRID_SIZE = 200  # cm
CELL_SIZE_CM = 10
GRID_COLS = 20
GRID_ROWS = 20

ROBOT_SIZE_CM = 30
AUTO_MOVE_SPEED = 4  # pixels per frame

SCALE = ARENA_PIXELS / GRID_SIZE  # pixels per cm

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRAY = (200, 200, 200)
LIGHT_GRAY = (180, 180, 180)
DARK_GRAY = (100, 100, 100)
GREEN = (0, 255, 0)
LIGHT_GREEN = (144, 238, 144)
BLUE = (0, 100, 255)
RED = (255, 0, 0)
LIGHT_RED = (255, 200, 200)
YELLOW = (255, 255, 0)
ORANGE = (255, 165, 0)
PURPLE = (128, 0, 128)
CYAN = (0, 255, 255)


# =============================================================================
# COORDINATE CONVERSION
# =============================================================================

def cm_to_pixel(x_cm, y_cm):
    """Convert cm to pixels (Y flipped)"""
    return int(x_cm * SCALE), int(WINDOW_HEIGHT - y_cm * SCALE)


def pixel_to_grid(px_x, px_y):
    """Convert pixels to grid cell"""
    if px_x > ARENA_PIXELS:
        return None
    x_cm = px_x / SCALE
    y_cm = (WINDOW_HEIGHT - px_y) / SCALE
    col = int(x_cm / CELL_SIZE_CM)
    row = int(y_cm / CELL_SIZE_CM)
    if 0 <= col < GRID_COLS and 0 <= row < GRID_ROWS:
        return (col, row)
    return None


# =============================================================================
# JSON FUNCTIONS
# =============================================================================

def load_obstacles_from_file(filepath):
    try:
        with open(filepath, 'r') as f:
            data = json.load(f)
        obstacles = []
        max_id = 0
        for obs_data in data.get('obstacles', []):
            obstacles.append(SimObstacle(
                obs_data['x'], obs_data['y'], 
                obs_data.get('direction', 0), 
                obs_data.get('id')
            ))
            if obs_data.get('id', 0) > max_id:
                max_id = obs_data['id']
        if max_id > 0:
            SimObstacle._id_counter = max_id + 1
        print(f"✓ Loaded {len(obstacles)} obstacles")
        return obstacles
    except Exception as e:
        print(f"✗ Error loading: {e}")
        return []


def save_obstacles_to_file(obstacles, filepath):
    data = {"obstacles": [
        {"id": o.id, "x": o.col, "y": o.row, "direction": o.direction}
        for o in obstacles
    ]}
    try:
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)
        print(f"✓ Saved {len(obstacles)} obstacles")
    except Exception as e:
        print(f"✗ Error saving: {e}")


# =============================================================================
# OBSTACLE CLASS
# =============================================================================

class SimObstacle:
    _id_counter = 1

    def __init__(self, col, row, direction=0, obs_id=None):
        self.col = col
        self.row = row
        self.direction = direction
        self.id = obs_id if obs_id else SimObstacle._id_counter
        if obs_id is None:
            SimObstacle._id_counter += 1

    def get_center_cm(self):
        return (self.col * 10 + 5, self.row * 10 + 5)

    def draw_collision_zone(self, surface):
        cx, cy = self.get_center_cm()
        px, py = cm_to_pixel(cx, cy)
        radius_px = int(COLLISION_DISTANCE * SCALE)
        
        # Draw semi-transparent circle
        s = pygame.Surface((radius_px*2, radius_px*2), pygame.SRCALPHA)
        pygame.draw.circle(s, (255, 0, 0, 40), (radius_px, radius_px), radius_px)
        surface.blit(s, (px - radius_px, py - radius_px))
        pygame.draw.circle(surface, LIGHT_RED, (px, py), radius_px, 1)

    def draw_target(self, surface):
        path_obs = PathObstacle(self.id, self.col, self.row, self.direction)
        target = path_obs.get_target_config()
        px, py = cm_to_pixel(target.x, target.y)
        
        # Cross marker
        pygame.draw.line(surface, CYAN, (px-6, py), (px+6, py), 2)
        pygame.draw.line(surface, CYAN, (px, py-6), (px, py+6), 2)
        
        # Direction arrow
        end_x = px + 12 * math.cos(-target.theta)
        end_y = py + 12 * math.sin(-target.theta)
        pygame.draw.line(surface, CYAN, (px, py), (int(end_x), int(end_y)), 2)

    def draw(self, surface):
        x_cm = self.col * CELL_SIZE_CM
        y_cm = (self.row + 1) * CELL_SIZE_CM
        px, py = cm_to_pixel(x_cm, y_cm)
        size_px = int(CELL_SIZE_CM * SCALE)

        # Obstacle box
        rect = pygame.Rect(px, py, size_px, size_px)
        pygame.draw.rect(surface, RED, rect)
        pygame.draw.rect(surface, BLACK, rect, 2)

        # ID text
        font = pygame.font.SysFont(None, 22)
        text = font.render(str(self.id), True, WHITE)
        text_rect = text.get_rect(center=(px + size_px//2, py + size_px//2))
        surface.blit(text, text_rect)

        # Image face indicator
        if self.direction == 0:  # North
            pts = [(px, py), (px + size_px, py)]
        elif self.direction == 90:  # East
            pts = [(px + size_px, py), (px + size_px, py + size_px)]
        elif self.direction == 180:  # South
            pts = [(px, py + size_px), (px + size_px, py + size_px)]
        else:  # West
            pts = [(px, py), (px, py + size_px)]
        pygame.draw.line(surface, YELLOW, pts[0], pts[1], 4)


# =============================================================================
# ROBOT CLASS
# =============================================================================

class Robot:
    def __init__(self, x_cm, y_cm, theta_deg):
        self.x = x_cm
        self.y = y_cm
        self.theta = theta_deg
        
        self.is_auto = False
        self.waypoints = []
        self.wp_idx = 0
        self.path_history = [(x_cm, y_cm)]

        size_px = int(ROBOT_SIZE_CM * SCALE)
        self.surf = pygame.Surface((size_px, size_px), pygame.SRCALPHA)
        self.surf.fill(BLUE)
        
        # Direction indicator
        w = size_px
        pygame.draw.polygon(self.surf, YELLOW, [
            (w*0.7, w*0.5), (w*0.4, w*0.3), (w*0.4, w*0.7)
        ])

    def start_path(self, path_result):
        self.is_auto = True
        self.waypoints = []
        for seg in path_result['segments']:
            self.waypoints.extend(seg['waypoints'])
        self.wp_idx = 0
        self.path_history = [(self.x, self.y)]
        print(f"✓ Following path: {path_result['order']}")
        print(f"  {len(self.waypoints)} waypoints")

    def update(self):
        if not self.is_auto or self.wp_idx >= len(self.waypoints):
            self.is_auto = False
            return
        
        tx, ty, tt = self.waypoints[self.wp_idx]
        dx, dy = tx - self.x, ty - self.y
        dist = math.sqrt(dx*dx + dy*dy)
        
        if dist < 2:
            self.wp_idx += 1
            if self.wp_idx >= len(self.waypoints):
                self.is_auto = False
                print("✓ Path complete!")
            return
        
        speed = AUTO_MOVE_SPEED / SCALE
        move = min(speed, dist)
        angle = math.atan2(dy, dx)
        
        self.x += move * math.cos(angle)
        self.y += move * math.sin(angle)
        self.theta = math.degrees(angle)
        self.path_history.append((self.x, self.y))

    def draw_history(self, surface):
        if len(self.path_history) > 1:
            pts = [cm_to_pixel(x, y) for x, y in self.path_history]
            pygame.draw.lines(surface, PURPLE, False, pts, 2)

    def draw_planned(self, surface, segments):
        for seg in segments:
            wps = seg.get('waypoints', [])
            if len(wps) > 1:
                pts = [cm_to_pixel(w[0], w[1]) for w in wps]
                pygame.draw.lines(surface, LIGHT_GREEN, False, pts, 1)

    def draw(self, surface):
        px, py = cm_to_pixel(self.x, self.y)
        rotated = pygame.transform.rotate(self.surf, self.theta)
        rect = rotated.get_rect(center=(px, py))
        surface.blit(rotated, rect.topleft)


# =============================================================================
# MAIN
# =============================================================================

def main():
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption("MDP Path Planning Simulator")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont(None, 24)
    small_font = pygame.font.SysFont(None, 18)

    robot = Robot(20, 20, 0)
    obstacles = []
    path_result = None
    
    start_time = None
    elapsed = 0.0
    
    show_collision = True
    show_targets = True
    show_path = True

    running = True
    while running:
        keys = pygame.key.get_pressed()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_s and keys[pygame.K_LCTRL]:
                    save_obstacles_to_file(obstacles, 'obstacles.json')
                elif event.key == pygame.K_l and keys[pygame.K_LCTRL]:
                    obstacles = load_obstacles_from_file('obstacles.json')
                    path_result = None
                elif event.key == pygame.K_r and keys[pygame.K_LCTRL]:
                    robot = Robot(20, 20, 0)
                    obstacles.clear()
                    SimObstacle._id_counter = 1
                    path_result = None
                    start_time = None
                    elapsed = 0.0
                    print("✓ Reset")
                elif event.key == pygame.K_v:
                    show_collision = not show_collision
                elif event.key == pygame.K_t:
                    show_targets = not show_targets
                elif event.key == pygame.K_p:
                    show_path = not show_path

            if event.type == pygame.MOUSEBUTTONDOWN:
                mx, my = pygame.mouse.get_pos()

                # START button
                start_btn = pygame.Rect(ARENA_PIXELS + 20, 450, 160, 40)
                if start_btn.collidepoint(mx, my) and obstacles:
                    path_obs = [PathObstacle(o.id, o.col, o.row, o.direction) for o in obstacles]
                    start_cfg = RobotConfig(robot.x, robot.y, math.radians(robot.theta))
                    
                    path_result = plan_path(start_cfg, path_obs, use_exhaustive=len(obstacles) <= 6)
                    
                    if 'error' not in path_result:
                        robot.start_path(path_result)
                        start_time = time.time()
                    continue

                # RESET button
                reset_btn = pygame.Rect(ARENA_PIXELS + 20, 500, 160, 40)
                if reset_btn.collidepoint(mx, my):
                    robot = Robot(20, 20, 0)
                    obstacles.clear()
                    SimObstacle._id_counter = 1
                    path_result = None
                    start_time = None
                    elapsed = 0.0
                    continue

                # Obstacle placement
                grid = pixel_to_grid(mx, my)
                if grid:
                    c, r = grid
                    existing = next((o for o in obstacles if o.col == c and o.row == r), None)
                    if existing:
                        if event.button == 1:
                            existing.direction = (existing.direction + 90) % 360
                        elif event.button == 3:
                            obstacles.remove(existing)
                    elif event.button == 1:
                        obstacles.append(SimObstacle(c, r))
                    path_result = None

        # Update
        if robot.is_auto:
            robot.update()
        if start_time and robot.is_auto:
            elapsed = time.time() - start_time

        # Draw
        screen.fill(WHITE)

        # Grid
        for i in range(GRID_COLS + 1):
            x = int(i * CELL_SIZE_CM * SCALE)
            pygame.draw.line(screen, GRAY, (x, 0), (x, WINDOW_HEIGHT))
        for i in range(GRID_ROWS + 1):
            y = int(WINDOW_HEIGHT - i * CELL_SIZE_CM * SCALE)
            pygame.draw.line(screen, GRAY, (0, y), (ARENA_PIXELS, y))

        # Start zone
        sz_px = int(40 * SCALE)
        pygame.draw.rect(screen, LIGHT_GREEN, (0, WINDOW_HEIGHT - sz_px, sz_px, sz_px))
        pygame.draw.rect(screen, GREEN, (0, WINDOW_HEIGHT - sz_px, sz_px, sz_px), 2)

        # Obstacles
        for obs in obstacles:
            if show_collision:
                obs.draw_collision_zone(screen)
            if show_targets:
                obs.draw_target(screen)
            obs.draw(screen)

        # Paths
        if show_path and path_result and 'segments' in path_result:
            robot.draw_planned(screen, path_result['segments'])
        robot.draw_history(screen)
        robot.draw(screen)

        # Dashboard
        pygame.draw.rect(screen, GRAY, (ARENA_PIXELS, 0, WINDOW_WIDTH - ARENA_PIXELS, WINDOW_HEIGHT))
        
        screen.blit(font.render("ROBOT", True, BLACK), (ARENA_PIXELS + 20, 20))
        screen.blit(font.render(f"X: {robot.x:.1f} cm", True, BLACK), (ARENA_PIXELS + 20, 50))
        screen.blit(font.render(f"Y: {robot.y:.1f} cm", True, BLACK), (ARENA_PIXELS + 20, 80))
        screen.blit(font.render(f"θ: {robot.theta:.1f}°", True, BLACK), (ARENA_PIXELS + 20, 110))
        screen.blit(font.render(f"Obstacles: {len(obstacles)}", True, BLACK), (ARENA_PIXELS + 20, 140))
        screen.blit(font.render(f"Time: {elapsed:.1f}s", True, BLACK), (ARENA_PIXELS + 20, 170))
        
        status = "RUNNING" if robot.is_auto else "READY"
        color = ORANGE if robot.is_auto else DARK_GRAY
        screen.blit(font.render(f"Status: {status}", True, color), (ARENA_PIXELS + 20, 200))
        
        if path_result and 'order' in path_result:
            screen.blit(font.render(f"Path: {path_result['order']}", True, BLACK), (ARENA_PIXELS + 20, 230))

        # Controls
        screen.blit(font.render("CONTROLS", True, BLACK), (ARENA_PIXELS + 20, 280))
        screen.blit(small_font.render("L-Click: Add/Rotate", True, BLACK), (ARENA_PIXELS + 20, 305))
        screen.blit(small_font.render("R-Click: Delete", True, BLACK), (ARENA_PIXELS + 20, 325))
        screen.blit(small_font.render("Ctrl+S: Save", True, BLACK), (ARENA_PIXELS + 20, 345))
        screen.blit(small_font.render("Ctrl+L: Load", True, BLACK), (ARENA_PIXELS + 20, 365))
        screen.blit(small_font.render("V: Collision zones", True, BLACK), (ARENA_PIXELS + 20, 385))
        screen.blit(small_font.render("T: Target positions", True, BLACK), (ARENA_PIXELS + 20, 405))
        screen.blit(small_font.render("P: Planned path", True, BLACK), (ARENA_PIXELS + 20, 425))

        # Buttons
        start_btn = pygame.Rect(ARENA_PIXELS + 20, 450, 160, 40)
        btn_color = GREEN if obstacles else LIGHT_GRAY
        pygame.draw.rect(screen, btn_color, start_btn)
        pygame.draw.rect(screen, BLACK, start_btn, 2)
        text = font.render("START", True, BLACK)
        screen.blit(text, text.get_rect(center=start_btn.center))

        reset_btn = pygame.Rect(ARENA_PIXELS + 20, 500, 160, 40)
        pygame.draw.rect(screen, RED, reset_btn)
        pygame.draw.rect(screen, BLACK, reset_btn, 2)
        text = font.render("RESET", True, WHITE)
        screen.blit(text, text.get_rect(center=reset_btn.center))

        # Legend
        screen.blit(font.render("LEGEND", True, BLACK), (ARENA_PIXELS + 20, 560))
        pygame.draw.circle(screen, LIGHT_RED, (ARENA_PIXELS + 30, 590), 8)
        screen.blit(small_font.render("Collision zone", True, BLACK), (ARENA_PIXELS + 45, 585))
        pygame.draw.line(screen, CYAN, (ARENA_PIXELS + 22, 610), (ARENA_PIXELS + 38, 610), 2)
        screen.blit(small_font.render("Target position", True, BLACK), (ARENA_PIXELS + 45, 605))
        pygame.draw.line(screen, LIGHT_GREEN, (ARENA_PIXELS + 22, 630), (ARENA_PIXELS + 38, 630), 2)
        screen.blit(small_font.render("Planned path", True, BLACK), (ARENA_PIXELS + 45, 625))
        pygame.draw.line(screen, PURPLE, (ARENA_PIXELS + 22, 650), (ARENA_PIXELS + 38, 650), 2)
        screen.blit(small_font.render("Traveled path", True, BLACK), (ARENA_PIXELS + 45, 645))

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
