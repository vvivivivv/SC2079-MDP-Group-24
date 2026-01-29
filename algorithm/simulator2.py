import pygame
import math
import sys
import json


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
TURN_SPEED = 5
MOVE_SPEED = 1


# Colors (R, G, B)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRAY = (200, 200, 200)
LIGHT_GRAY = (180, 180, 180)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
RED = (255, 0, 0)
YELLOW = (255, 255, 0)


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
    """
    Load obstacles from a JSON file.
    Expected JSON format:
    {
        "obstacles": [
            {"id": 1, "x": 5, "y": 5, "direction": 0},
            {"id": 2, "x": 10, "y": 8, "direction": 90}
        ]
    }
    """
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
        
        # Update ID counter to avoid duplicates
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
    obstacles.clear()
    Obstacle._id_counter = 1
    print("✓ Simulation reset")


# =============================================================================
# 4. CLASSES
# =============================================================================


class Obstacle:
    _id_counter = 1  # Class variable to auto-increment IDs
    
    def __init__(self, col, row, direction=0, obstacle_id=None):
        self.col = col
        self.row = row
        self.direction = direction
        
        # Auto-assign ID if not provided
        if obstacle_id is None:
            self.id = Obstacle._id_counter
            Obstacle._id_counter += 1
        else:
            self.id = obstacle_id


    def draw_safety_zone(self, surface):
        """Draw a 20cm camera positioning zone around the obstacle (dashed grey outline)"""
        safety_margin_cm = 20  # Changed from 10 to 20
        x_cm = (self.col * CELL_SIZE_CM) - safety_margin_cm
        y_cm = ((self.row + 1) * CELL_SIZE_CM) + safety_margin_cm
        
        px_x, px_y = grid_to_pixel(x_cm, y_cm)
        
        scale = ARENA_WIDTH / GRID_SIZE
        # Safety zone is now 50cm x 50cm (10cm obstacle + 20cm padding on each side)
        safety_size_px = int((50 / GRID_SIZE) * ARENA_WIDTH)  # Changed from 30 to 50
        
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
        if self.direction == 0:   # North
            start_pos = (px_x, px_y)
            end_pos = (px_x + size_px, px_y)
        elif self.direction == 90: # East
            start_pos = (px_x + size_px, px_y)
            end_pos = (px_x + size_px, px_y + size_px)
        elif self.direction == 180: # South
            start_pos = (px_x, px_y + size_px)
            end_pos = (px_x + size_px, px_y + size_px)
        else: # 270, West
            start_pos = (px_x, px_y)
            end_pos = (px_x, px_y + size_px)
            
        pygame.draw.line(surface, BLACK, start_pos, end_pos, 4)


class Robot:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
        
        scale = ARENA_WIDTH / GRID_SIZE
        size_px = int(ROBOT_SIZE_CM * scale)
        self.original_surf = pygame.Surface((size_px, size_px), pygame.SRCALPHA)
        self.original_surf.fill(BLUE)
        
        w, h = size_px, size_px
        pygame.draw.line(self.original_surf, YELLOW, (w/2, h/2), (w, h/2), 3)


    def move(self, keys):
        if keys[pygame.K_LEFT]:
            self.theta += TURN_SPEED
        if keys[pygame.K_RIGHT]:
            self.theta -= TURN_SPEED
        
        if keys[pygame.K_UP]:
            rad = math.radians(self.theta)
            self.x += MOVE_SPEED * math.cos(rad)
            self.y += MOVE_SPEED * math.sin(rad)
            
        if keys[pygame.K_DOWN]:
            rad = math.radians(self.theta)
            self.x -= MOVE_SPEED * math.cos(rad)
            self.y -= MOVE_SPEED * math.sin(rad)
            
        self.x = max(0, min(self.x, GRID_SIZE))
        self.y = max(0, min(self.y, GRID_SIZE))


    def draw(self, surface):
        px, py = grid_to_pixel(self.x, self.y)
        rotated_surf = pygame.transform.rotate(self.original_surf, self.theta)
        rect = rotated_surf.get_rect(center=(px, py))
        surface.blit(rotated_surf, rect.topleft)


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
                
            # MOUSE EVENTS
            if event.type == pygame.MOUSEBUTTONDOWN:
                mouse_x, mouse_y = pygame.mouse.get_pos()
                
                # Check if Reset button was clicked
                reset_button_rect = pygame.Rect(ARENA_WIDTH + 20, 400, 160, 40)
                if reset_button_rect.collidepoint(mouse_x, mouse_y):
                    reset_simulation(robot, obstacles)
                    continue
                
                # Handle obstacle placement
                grid_pos = pixel_to_grid(mouse_x, mouse_y)
                
                if grid_pos:
                    c, r = grid_pos
                    existing = next((o for o in obstacles if o.col == c and o.row == r), None)
                    
                    if existing:
                        if event.button == 1:  # Left Click: Rotate
                            existing.direction = (existing.direction + 90) % 360
                        elif event.button == 3:  # Right Click: Remove
                            obstacles.remove(existing)
                    else:
                        if event.button == 1:  # Left Click: Add
                            obstacles.append(Obstacle(c, r))


        # --- 2. UPDATE ---
        robot.move(keys)


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
            obs.draw_safety_zone(screen)
            obs.draw(screen)


        # D. Draw Robot
        robot.draw(screen)


        # E. Draw Dashboard
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
        
        # Controls
        screen.blit(font.render("CONTROLS:", True, BLACK), (ARENA_WIDTH + 20, 200))
        screen.blit(font.render("Arrow Keys: Move", True, BLACK), (ARENA_WIDTH + 20, 230))
        screen.blit(font.render("L-Click: Add/Rotate Obs", True, BLACK), (ARENA_WIDTH + 20, 260))
        screen.blit(font.render("R-Click: Delete Obs", True, BLACK), (ARENA_WIDTH + 20, 290))
        screen.blit(font.render("Ctrl+S: Save to JSON", True, BLACK), (ARENA_WIDTH + 20, 320))
        screen.blit(font.render("Ctrl+L: Load from JSON", True, BLACK), (ARENA_WIDTH + 20, 350))

        # Reset Button
        reset_button_rect = pygame.Rect(ARENA_WIDTH + 20, 400, 160, 40)
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
