# ----------- grid_utils.py (Modified) -----------
import pygame
import settings as s
import math # Import math

def create_grid(width, height):
    """Creates an empty grid."""
    return [[s.EMPTY for _ in range(width)] for _ in range(height)]

def is_valid(row, col):
    """Checks if a cell coordinate is within grid bounds."""
    return 0 <= row < s.GRID_HEIGHT_CELLS and 0 <= col < s.GRID_WIDTH_CELLS

def get_neighbors(grid, row, col, allow_diagonal=True): # Default to allowing diagonal
    """Gets valid, non-obstacle neighbor coordinates."""
    neighbors = []
    potential_neighbors = []
    # 4-directional
    potential_neighbors.extend([(row - 1, col), (row + 1, col), (row, col - 1), (row, col + 1)])
    if allow_diagonal:
         # Diagonal
        potential_neighbors.extend([(row - 1, col - 1), (row - 1, col + 1), (row + 1, col - 1), (row + 1, col + 1)])

    for r_new, c_new in potential_neighbors:
        # Only check if the target neighbor itself is valid and not an obstacle
        # Corner cutting checks will be done by the cost function in the algorithms
        if is_valid(r_new, c_new) and grid[r_new][c_new] != s.OBSTACLE:
            neighbors.append((r_new, c_new)) # Return just the coordinate tuple
    return neighbors

def reconstruct_path(came_from, start_pos, goal_pos):
    """Builds the path list from the came_from dictionary."""
    path = []
    current = goal_pos
    if current not in came_from and current != start_pos: # Goal not reached
         return None
    # Check came_from[current] isn't None before entering loop if start == goal
    while current != start_pos and came_from.get(current) is not None:
        path.append(current)
        current = came_from.get(current)
        # Safety break for potential cycles if came_from is malformed
        if len(path) > s.GRID_WIDTH_CELLS * s.GRID_HEIGHT_CELLS * 2:
            print("Error: Path reconstruction seems to be in a loop.")
            return None # Indicate failure
    if current == start_pos: # Add start only if loop finished properly
        path.append(start_pos)
        path.reverse()
        return path
    else: # Path didn't lead back to start
        # This might happen if goal was unreachable but added to came_from somehow
        print("Warning: Path reconstruction failed to reach start.")
        return None


def draw_grid_and_elements(screen, grid, start_pos, goal_pos, visited=None, path=None):
    """Draws the grid, obstacles, start/goal markers, visited cells, and path."""
    screen.fill(s.WHITE)
    if visited is None: visited = set()

    # 1. Draw Visited Cells (Inset, Soft Pastel)
    inset = s.CELL_SIZE // 5  # Slight inset for better visual distinction
    for r_vis, c_vis in visited:
        rect = pygame.Rect(
            c_vis * s.CELL_SIZE + inset,
            r_vis * s.CELL_SIZE + inset,
            s.CELL_SIZE - 2 * inset,
            s.CELL_SIZE - 2 * inset
        )
        pygame.draw.rect(screen, s.PASTEL_GRAY, rect)  # Use a soft pastel gray

    # 2. Draw Obstacles and Thin Grid Lines
    for r in range(s.GRID_HEIGHT_CELLS):
        for c in range(s.GRID_WIDTH_CELLS):
            rect = pygame.Rect(c * s.CELL_SIZE, r * s.CELL_SIZE, s.CELL_SIZE, s.CELL_SIZE)
            cell_value = grid[r][c]
            color_to_draw = None # Determine color first

            if cell_value == s.OBSTACLE:
                color_to_draw = s.PASTEL_BLACK
            elif cell_value == s.DYNAMIC_OBSTACLE: # <<<--- ADDED CHECK
                 color_to_draw = s.ORANGE

            if color_to_draw:
                 pygame.draw.rect(screen, color_to_draw, rect)

            pygame.draw.rect(screen, s.PASTEL_OUTLINE, rect, 1) # Draw cell border over everything

    # 3. Draw Path (Smooth Pastel Red Line)
    if path and len(path) > 1:
        path_points = [
            (c * s.CELL_SIZE + s.CELL_SIZE // 2, r * s.CELL_SIZE + s.CELL_SIZE // 2)
            for r, c in path
        ]
        pygame.draw.lines(screen, s.PASTEL_RED, False, path_points, 3)

    # 4. Draw Start and Goal (Pastel Circles with Outline)
    start_center = (
        start_pos[1] * s.CELL_SIZE + s.CELL_SIZE // 2,
        start_pos[0] * s.CELL_SIZE + s.CELL_SIZE // 2
    )
    pygame.draw.circle(screen, s.PASTEL_BLUE, start_center, s.CELL_SIZE // 3)
    pygame.draw.circle(screen, s.PASTEL_OUTLINE, start_center, s.CELL_SIZE // 3, 2)

    goal_center = (
        goal_pos[1] * s.CELL_SIZE + s.CELL_SIZE // 2,
        goal_pos[0] * s.CELL_SIZE + s.CELL_SIZE // 2
    )
    pygame.draw.circle(screen, s.PASTEL_GREEN, goal_center, s.CELL_SIZE // 3)
    pygame.draw.circle(screen, s.PASTEL_OUTLINE, goal_center, s.CELL_SIZE // 3, 2)

    pygame.display.flip()


    # Keep this update outside the main draw function, call it explicitly in main loop
    # pygame.display.flip()