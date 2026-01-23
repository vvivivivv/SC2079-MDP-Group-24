import pygame
import math
import sys

# =============================================================================
# 1. CONFIGURATION & CONSTANTS
# =============================================================================

# Visual Settings
WINDOW_WIDTH = 1000  # Total window width (Arena + Dashboard)
WINDOW_HEIGHT = 800  # Total window height
ARENA_WIDTH = 800    # Pixel width of the 200cm arena
FPS = 60

# Arena Settings
GRID_SIZE = 200      # 200 cm x 200 cm real world size
CELL_SIZE_CM = 10    # Each grid cell is 10cm x 10cm
GRID_COLS = 20       # 20 columns
GRID_ROWS = 20       # 20 rows

# Robot Settings
ROBOT_SIZE_CM = 30   # 30cm x 30cm
TURN_SPEED = 5       # Degrees per frame
MOVE_SPEED = 1       # cm per frame

# Colors (R, G, B)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRAY = (200, 200, 200)
GREEN = (0, 255, 0)      # Start Zone
BLUE = (0, 0, 255)       # Robot
RED = (255, 0, 0)        # Obstacle
YELLOW = (255, 255, 0)   # Robot Head / Selection

# =============================================================================
# 2. HELPER FUNCTIONS (Coordinate Math)
# =============================================================================

def grid_to_pixel(x_cm, y_cm):
    """
    Converts logical coordinates (cm) to Pygame screen coordinates (px).
    Logic: (0,0) is Bottom-Left. Pygame: (0,0) is Top-Left.
    """
    scale = ARENA_WIDTH / GRID_SIZE
    
    px_x = x_cm * scale
    # Flip Y-axis: Height - y
    px_y = WINDOW_HEIGHT - (y_cm * scale)
    
    return int(px_x), int(px_y)

def pixel_to_grid(px_x, px_y):
    """
    Converts mouse clicks (px) to logical grid index (col, row).
    """
    scale = ARENA_WIDTH / GRID_SIZE
    
    # Check if click is outside arena
    if px_x > ARENA_WIDTH: return None
    
    # Calculate column
    col = int(px_x / (CELL_SIZE_CM * scale))
    
    # Calculate row (Flip Y-axis logic)
    # Pygame Y increases downwards, Grid Row increases upwards
    pixel_from_bottom = WINDOW_HEIGHT - px_y
    row = int(pixel_from_bottom / (CELL_SIZE_CM * scale))
    
    return (col, row)

# =============================================================================
# 3. CLASSES
# =============================================================================

class Obstacle:
    def __init__(self, col, row, direction=0):
        self.col = col
        self.row = row
        self.direction = direction # 0=N, 90=E, 180=S, 270=W
        self.id = 1 # Placeholder ID

    def draw(self, surface):
        # 1. Calculate pixel position of the cell (Top-Left of the cell for drawing)
        # Note: We need the Top-Left for pygame.rect, so we use (row + 1) logic for Y
        x_cm = self.col * CELL_SIZE_CM
        y_cm = (self.row + 1) * CELL_SIZE_CM 
        px_x, px_y = grid_to_pixel(x_cm, y_cm)
        
        size_px = int((CELL_SIZE_CM / GRID_SIZE) * ARENA_WIDTH)
        
        # 2. Draw the Red Block
        rect = pygame.Rect(px_x, px_y, size_px, size_px)
        pygame.draw.rect(surface, RED, rect)
        pygame.draw.rect(surface, BLACK, rect, 1) # Border

        # 3. Draw the "Image Face" (The thick black line)
        # Calculate center of block
        cx, cy = px_x + size_px//2, px_y + size_px//2
        offset = size_px // 2
        
        start_pos, end_pos = (cx, cy), (cx, cy)
        
        if self.direction == 0:   # North (Up)
            start_pos = (px_x, px_y)
            end_pos = (px_x + size_px, px_y)
        elif self.direction == 90: # East (Right)
            start_pos = (px_x + size_px, px_y)
            end_pos = (px_x + size_px, px_y + size_px)
        elif self.direction == 180: # South (Down)
            start_pos = (px_x, px_y + size_px)
            end_pos = (px_x + size_px, px_y + size_px)
        elif self.direction == 270: # West (Left)
            start_pos = (px_x, px_y)
            end_pos = (px_x, px_y + size_px)
            
        pygame.draw.line(surface, BLACK, start_pos, end_pos, 4)

class Robot:
    def __init__(self, x, y, theta):
        self.x = x           # cm
        self.y = y           # cm
        self.theta = theta   # degrees
        
        # Create the Robot Image Surface (Square)
        scale = ARENA_WIDTH / GRID_SIZE
        size_px = int(ROBOT_SIZE_CM * scale)
        self.original_surf = pygame.Surface((size_px, size_px), pygame.SRCALPHA)
        self.original_surf.fill(BLUE)
        
        # Draw "Head" (Yellow line at the top to indicate front)
        # In Pygame unrotated surface, "Up" is negative Y, so top of surface is y=0
        # But we will rotate this, so let's draw it on the right side (0 degrees = East) 
        # or top side depending on how you define 0.
        # Let's assume 0 degrees = East (Right).
        w, h = size_px, size_px
        pygame.draw.line(self.original_surf, YELLOW, (w/2, h/2), (w, h/2), 3) # Line to right

    def move(self, keys):
        # MANUAL CONTROLS (Tank Style)
        if keys[pygame.K_LEFT]:
            self.theta += TURN_SPEED
        if keys[pygame.K_RIGHT]:
            self.theta -= TURN_SPEED
        
        if keys[pygame.K_UP]:
            # Convert degrees to radians for math
            rad = math.radians(self.theta)
            self.x += MOVE_SPEED * math.cos(rad)
            self.y += MOVE_SPEED * math.sin(rad)
            
        if keys[pygame.K_DOWN]:
            rad = math.radians(self.theta)
            self.x -= MOVE_SPEED * math.cos(rad)
            self.y -= MOVE_SPEED * math.sin(rad)
            
        # Bound checks (Keep robot inside arena)
        self.x = max(0, min(self.x, GRID_SIZE))
        self.y = max(0, min(self.y, GRID_SIZE))

    def draw(self, surface):
        px, py = grid_to_pixel(self.x, self.y)
        
        # Rotate image (Pygame rotates CCW, so we pass angle directly)
        rotated_surf = pygame.transform.rotate(self.original_surf, self.theta)
        
        # Center the image on the x,y coordinates
        rect = rotated_surf.get_rect(center=(px, py))
        
        surface.blit(rotated_surf, rect.topleft)

# =============================================================================
# 4. MAIN LOOP
# =============================================================================

def main():
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption("MDP Group 14 Simulator")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont(None, 24)

    # Init State
    robot = Robot(x=15, y=15, theta=90) # Start in middle of start zone
    obstacles = [] # List of Obstacle objects

    running = True
    while running:
        # --- 1. EVENTS ---
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                
            # MOUSE: Place/Edit Obstacles
            if event.type == pygame.MOUSEBUTTONDOWN:
                mouse_x, mouse_y = pygame.mouse.get_pos()
                grid_pos = pixel_to_grid(mouse_x, mouse_y)
                
                if grid_pos: # Click was inside arena
                    c, r = grid_pos
                    
                    # Check if obstacle exists there
                    existing = next((o for o in obstacles if o.col == c and o.row == r), None)
                    
                    if existing:
                        if event.button == 1: # Left Click: Rotate Face
                            existing.direction = (existing.direction + 90) % 360
                        elif event.button == 3: # Right Click: Remove
                            obstacles.remove(existing)
                    else:
                        if event.button == 1: # Left Click: Add
                            obstacles.append(Obstacle(c, r))

        # --- 2. UPDATE ---
        keys = pygame.key.get_pressed()
        robot.move(keys)

        # --- 3. DRAW ---
        screen.fill(WHITE)

        # A. Draw Grid Lines
        for i in range(GRID_COLS + 1):
            # Vertical
            start = grid_to_pixel(i * 10, 0)
            end = grid_to_pixel(i * 10, GRID_SIZE)
            pygame.draw.line(screen, GRAY, start, end)
            # Horizontal
            start = grid_to_pixel(0, i * 10)
            end = grid_to_pixel(GRID_SIZE, i * 10)
            pygame.draw.line(screen, GRAY, start, end)
            
        # B. Draw Start Zone (30x30 cm box at 0,0)
        start_px_bl = grid_to_pixel(0, 0) # Bottom Left of arena
        # Rect needs (left, top, width, height). 
        # Since grid_to_pixel returns bottom-left logic, we need to adjust 'top' for rect
        sz_scale = ARENA_WIDTH / GRID_SIZE
        pygame.draw.rect(screen, GREEN, (0, WINDOW_HEIGHT - (30 * sz_scale), 30 * sz_scale, 30 * sz_scale))

        # C. Draw Obstacles
        for obs in obstacles:
            obs.draw(screen)

        # D. Draw Robot
        robot.draw(screen)

        # E. Draw Dashboard (Right Side)
        # Draw a divider
        pygame.draw.rect(screen, GRAY, (ARENA_WIDTH, 0, WINDOW_WIDTH-ARENA_WIDTH, WINDOW_HEIGHT))
        
        # Display Stats
        text_x = f"X: {robot.x:.1f} cm"
        text_y = f"Y: {robot.y:.1f} cm"
        text_t = f"Angle: {int(robot.theta)} deg"
        
        screen.blit(font.render("ROBOT STATE", True, BLACK), (ARENA_WIDTH + 20, 20))
        screen.blit(font.render(text_x, True, BLACK), (ARENA_WIDTH + 20, 50))
        screen.blit(font.render(text_y, True, BLACK), (ARENA_WIDTH + 20, 80))
        screen.blit(font.render(text_t, True, BLACK), (ARENA_WIDTH + 20, 110))
        
        screen.blit(font.render("CONTROLS:", True, BLACK), (ARENA_WIDTH + 20, 200))
        screen.blit(font.render("Arrow Keys: Move", True, BLACK), (ARENA_WIDTH + 20, 230))
        screen.blit(font.render("L-Click: Add/Rotate Obs", True, BLACK), (ARENA_WIDTH + 20, 260))
        screen.blit(font.render("R-Click: Delete Obs", True, BLACK), (ARENA_WIDTH + 20, 290))

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()