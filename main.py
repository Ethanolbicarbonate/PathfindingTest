# ----------- main.py (Modified for Scenarios) -----------
import pygame
import sys
import time
import random # For potential random obstacle placement
import settings as s
from grid_utils import create_grid, draw_grid_and_elements, is_valid
from algorithms import dijkstra, a_star, DStarLite

class Visualizer:
    """Handles Pygame window and drawing updates."""
    def __init__(self, screen, start_pos, goal_pos):
        self.screen = screen
        self.grid = None # Grid will be updated per scenario
        self.start_pos = start_pos
        self.goal_pos = goal_pos
        self.font = pygame.font.SysFont(None, 24)

    def update_grid_reference(self, new_grid):
        """Update the grid the visualizer uses."""
        self.grid = new_grid

    def update_search_view(self, visited=None, path=None, message=""):
        """Redraws the grid with current search state."""
        if self.grid is None: return # Don't draw if no grid is set

        draw_grid_and_elements(self.screen, self.grid, self.start_pos,
                               self.goal_pos, visited, path)
        if message:
            text_surf = self.font.render(message, True, s.BLACK)
            # Simple background for text readability
            text_rect = text_surf.get_rect(topleft=(10, 10))
            pygame.draw.rect(self.screen, s.WHITE, text_rect.inflate(4, 4))
            self.screen.blit(text_surf, text_rect)
        pygame.display.flip()

    def handle_events(self):
        """Process Pygame events to keep window responsive."""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

    def wait_for_quit(self):
        """Loop indefinitely until the user quits."""
        while True:
            self.handle_events()
            pygame.time.wait(50)

# --- Scenario Setup Functions ---

def setup_scenario_simple(grid):
    """Clears the grid (no obstacles)."""
    for r in range(len(grid)):
        for c in range(len(grid[0])):
            grid[r][c] = s.EMPTY
    # Ensure start/goal aren't accidentally obstacles if grid was reused
    # (Though create_grid handles this initially)

def setup_scenario_complex(grid):
    """Adds predefined complex obstacles."""
    setup_scenario_simple(grid) # Start with a clear grid
    # Vertical wall with a gap
    wall_c = s.GRID_WIDTH_CELLS // 2
    gap_r_start = s.GRID_HEIGHT_CELLS // 2 - 2
    gap_r_end = s.GRID_HEIGHT_CELLS // 2 + 2
    for r in range(s.GRID_HEIGHT_CELLS):
        if not (gap_r_start <= r <= gap_r_end):
            if is_valid(r, wall_c): grid[r][wall_c] = s.OBSTACLE

    # # Horizontal wall with a gap
    # wall_r = s.GRID_HEIGHT_CELLS * 3 // 4
    # gap_c_start = s.GRID_WIDTH_CELLS // 3
    # gap_c_end = s.GRID_WIDTH_CELLS // 3 + 5
    # for c in range(s.GRID_WIDTH_CELLS):
    #      if not (gap_c_start <= c <= gap_c_end):
    #           if is_valid(wall_r, c): grid[wall_r][c] = s.OBSTACLE

    # Some blocks creating tighter passages
    blocks = [(5, 5), (5, 6), (6, 5), (6,6),
              (15, s.GRID_WIDTH_CELLS - 7), (14, s.GRID_WIDTH_CELLS - 7),
              (15, s.GRID_WIDTH_CELLS - 8),(14, s.GRID_WIDTH_CELLS - 8)]
    for r, c in blocks:
        if is_valid(r, c): grid[r][c] = s.OBSTACLE

def apply_dynamic_change_minor(grid, reference_path):
    """Adds ONE obstacle on the reference path."""
    if not reference_path or len(reference_path) <= 2:
        print("Warning: Cannot apply minor change, reference path too short or None.")
        return None # Indicate no change applied

    # Try to place obstacle somewhere in the middle, avoiding start/goal neighbors
    potential_indices = list(range(1, len(reference_path) - 1))
    random.shuffle(potential_indices)

    for idx in potential_indices:
        obstacle_pos = reference_path[idx]
        # Basic check: don't block start/goal directly if path is very short
        if obstacle_pos != start_pos and obstacle_pos != goal_pos:
             # Check if cell is currently empty
             if is_valid(obstacle_pos[0], obstacle_pos[1]) and grid[obstacle_pos[0]][obstacle_pos[1]] == s.EMPTY:
                 grid[obstacle_pos[0]][obstacle_pos[1]] = s.OBSTACLE
                 print(f"Applying MINOR dynamic change: Obstacle added at {obstacle_pos}")
                 return [obstacle_pos] # Return list of changed cells

    print("Warning: Could not find suitable place for minor change on path.")
    return None

def apply_dynamic_change_major(grid, reference_path):
    """Adds MULTIPLE obstacles, potentially blocking path significantly."""
    if not reference_path or len(reference_path) <= 4:
         print("Warning: Cannot apply major change, reference path too short or None.")
         return None

    changed_cells = []
    num_obstacles = random.randint(3, 6) # Add 3 to 6 obstacles

    # Try placing some near the middle of the path
    mid_point_index = len(reference_path) // 2
    indices_to_try = list(range(max(1, mid_point_index - num_obstacles // 2),
                                min(len(reference_path) - 1, mid_point_index + num_obstacles // 2 + 1)))
    random.shuffle(indices_to_try)

    count = 0
    for idx in indices_to_try:
         if count >= num_obstacles: break
         obstacle_pos = reference_path[idx]
         if obstacle_pos != start_pos and obstacle_pos != goal_pos:
            if is_valid(obstacle_pos[0], obstacle_pos[1]) and grid[obstacle_pos[0]][obstacle_pos[1]] == s.EMPTY:
                grid[obstacle_pos[0]][obstacle_pos[1]] = s.OBSTACLE
                changed_cells.append(obstacle_pos)
                count += 1

    # Maybe add one or two random obstacles near the path too
    # ... (More complex logic could go here if needed) ...

    if not changed_cells:
         print("Warning: Could not apply major change.")
         return None
    else:
        print(f"Applying MAJOR dynamic change: {len(changed_cells)} obstacles added near path: {changed_cells}")
        return changed_cells





# ----------- Add this near the other scenario functions in main.py -----------

def setup_scenario_from_list(grid, obstacle_list):
    """Sets up obstacles based on a provided list of coordinates."""
    # Start with a clear grid
    for r in range(len(grid)):
        for c in range(len(grid[0])):
            grid[r][c] = s.EMPTY

    # Add obstacles from the list
    for r_obs, c_obs in obstacle_list:
        if is_valid(r_obs, c_obs):
            grid[r_obs][c_obs] = s.OBSTACLE
        else:
            print(f"Warning: Obstacle coordinate { (r_obs, c_obs) } is out of bounds.")

def apply_dynamic_change_minor(grid, reference_path, target_cells=None):
    """Adds ONE obstacle. Uses target_cells if provided, otherwise places on path."""
    if target_cells: # User specified exact cell(s)
        if isinstance(target_cells, list) and len(target_cells) > 0:
            obstacle_pos = target_cells[0] # Use the first one for minor change
            if is_valid(obstacle_pos[0], obstacle_pos[1]) and grid[obstacle_pos[0]][obstacle_pos[1]] == s.EMPTY:
                grid[obstacle_pos[0]][obstacle_pos[1]] = s.OBSTACLE
                print(f"Applying CUSTOM MINOR dynamic change: Obstacle added at {obstacle_pos}")
                return [obstacle_pos]
            else:
                 print(f"Warning: Cannot apply custom minor change at {obstacle_pos} (invalid or occupied).")
                 return None
        else:
            print("Warning: Invalid target_cells format for minor change.")
            return None

    # --- Fallback to path-based logic if target_cells not provided ---
    if not reference_path or len(reference_path) <= 2:
        print("Warning: Cannot apply path-based minor change, reference path too short or None.")
        return None

    potential_indices = list(range(1, len(reference_path) - 1))
    random.shuffle(potential_indices)

    for idx in potential_indices:
        obstacle_pos = reference_path[idx]
        if obstacle_pos != start_pos and obstacle_pos != goal_pos:
             if is_valid(obstacle_pos[0], obstacle_pos[1]) and grid[obstacle_pos[0]][obstacle_pos[1]] == s.EMPTY:
                 grid[obstacle_pos[0]][obstacle_pos[1]] = s.OBSTACLE
                 print(f"Applying PATH-BASED MINOR dynamic change: Obstacle added at {obstacle_pos}")
                 return [obstacle_pos]

    print("Warning: Could not find suitable place for path-based minor change.")
    return None

def apply_dynamic_change_major(grid, reference_path, target_cells=None):
    """Adds MULTIPLE obstacles. Uses target_cells if provided, otherwise places near path."""
    changed_cells = []
    if target_cells: # User specified exact cells
        if isinstance(target_cells, list):
            count = 0
            for obstacle_pos in target_cells:
                 if is_valid(obstacle_pos[0], obstacle_pos[1]) and grid[obstacle_pos[0]][obstacle_pos[1]] == s.EMPTY:
                     grid[obstacle_pos[0]][obstacle_pos[1]] = s.OBSTACLE
                     changed_cells.append(obstacle_pos)
                     count +=1
                 else:
                      print(f"Warning: Cannot apply custom major change at {obstacle_pos} (invalid or occupied).")

            if count > 0:
                print(f"Applying CUSTOM MAJOR dynamic change: {count} obstacles added at specified locations: {changed_cells}")
                return changed_cells
            else:
                print("Warning: No valid custom major changes applied from target_cells.")
                return None
        else:
             print("Warning: Invalid target_cells format for major change.")
             return None

    # --- Fallback to path-based logic if target_cells not provided ---
    if not reference_path or len(reference_path) <= 4:
         print("Warning: Cannot apply path-based major change, reference path too short or None.")
         return None

    num_obstacles = random.randint(3, 6)
    mid_point_index = len(reference_path) // 2
    indices_to_try = list(range(max(1, mid_point_index - num_obstacles // 2),
                                min(len(reference_path) - 1, mid_point_index + num_obstacles // 2 + 1)))
    random.shuffle(indices_to_try)

    count = 0
    for idx in indices_to_try:
         if count >= num_obstacles: break
         obstacle_pos = reference_path[idx]
         if obstacle_pos != start_pos and obstacle_pos != goal_pos:
            if is_valid(obstacle_pos[0], obstacle_pos[1]) and grid[obstacle_pos[0]][obstacle_pos[1]] == s.EMPTY:
                grid[obstacle_pos[0]][obstacle_pos[1]] = s.OBSTACLE
                changed_cells.append(obstacle_pos)
                count += 1

    if not changed_cells:
         print("Warning: Could not apply path-based major change.")
         return None
    else:
        print(f"Applying PATH-BASED MAJOR dynamic change: {len(changed_cells)} obstacles added near path: {changed_cells}")
        return changed_cells







# --- Main Comparison Logic ---
if __name__ == "__main__":
    # Initialize Pygame & Visualizer
    pygame.init()
    screen = pygame.display.set_mode((s.WINDOW_WIDTH, s.WINDOW_HEIGHT))
    pygame.display.set_caption("Pathfinding Comparison")

    # Define fixed start/goal for consistency across scenarios
    start_pos = (2, 2)
    # Adjust goal if grid size changes significantly
    goal_pos = (s.GRID_HEIGHT_CELLS - 3, s.GRID_WIDTH_CELLS - 3)

    vis = Visualizer(screen, start_pos, goal_pos)

    algorithms_to_run = {
        "Dijkstra": dijkstra,
        "A*": a_star,
        "D* Lite": "dstar_lite_runner"
    }

    # ==============================================================
    # ========== CUSTOMIZE YOUR SCENARIOS HERE =====================
    # ==============================================================

    # --- Static Complex Obstacles ---
    # Define a list of (row, col) tuples for your complex static map
    custom_complex_obstacle_list = [
        # Example: A wall with a gap
        (r, s.GRID_WIDTH_CELLS // 2) for r in range(s.GRID_HEIGHT_CELLS) if r not in range(8, 12)
    ] + [
        # Example: Another wall
        (s.GRID_HEIGHT_CELLS // 2, c) for c in range(s.GRID_WIDTH_CELLS // 3, s.GRID_WIDTH_CELLS * 2 // 3)
    ] + [
        # Example: Some blocks
        (5,5), (5,6), (6,5), (15, 20), (15, 21)
    ]

    # --- Dynamic Minor Change Targets ---
    # Define a list with ONE (row, col) tuple.
    # If empty or None, it will try to place obstacle on the calculated path.
    custom_minor_change_targets = [(10, 15)] # Example: Force obstacle at (10, 15)
    # custom_minor_change_targets = None # Example: Use path-based placement

    # --- Dynamic Major Change Targets ---
    # Define a list with MULTIPLE (row, col) tuples.
    # If empty or None, it will try to place multiple obstacles near the path.
    custom_major_change_targets = [(12, 16), (13, 16), (14, 16), (13, 17)] # Example: Block passage
    # custom_major_change_targets = None # Example: Use path-based placement

    # ==============================================================
    # ==============================================================


    scenarios = {
        "Static Simple": {
            "setup": setup_scenario_simple,
            "setup_args": [], # No arguments needed
            "dynamic": None,
            "dynamic_args": []
        },
        "Static Complex (Custom)": {
            "setup": setup_scenario_from_list,
            "setup_args": [custom_complex_obstacle_list], # Pass the custom list
            "dynamic": None,
            "dynamic_args": []
        },
        "Dynamic Minor Change (Custom/Path)": {
            "setup": setup_scenario_from_list, # Base it on the custom complex map
            "setup_args": [custom_complex_obstacle_list],
            "dynamic": apply_dynamic_change_minor,
            "dynamic_args": [custom_minor_change_targets] # Pass custom targets
        },
        "Dynamic Major Change (Custom/Path)": {
            "setup": setup_scenario_from_list, # Base it on the custom complex map
            "setup_args": [custom_complex_obstacle_list],
            "dynamic": apply_dynamic_change_major,
            "dynamic_args": [custom_major_change_targets] # Pass custom targets
        },
    }

    all_results = {}

    for scenario_name, config in scenarios.items():
        print(f"\n{'='*10} SCENARIO: {scenario_name} {'='*10}")
        all_results[scenario_name] = {}
        results = all_results[scenario_name]

        grid = create_grid(s.GRID_WIDTH_CELLS, s.GRID_HEIGHT_CELLS)
        # Call setup function with its specific arguments
        config["setup"](grid, *config["setup_args"]) # Unpack arguments using *
        vis.update_grid_reference(grid)

        # Ensure start/goal are clear after setup
        if is_valid(*start_pos) and grid[start_pos[0]][start_pos[1]] == s.OBSTACLE: grid[start_pos[0]][start_pos[1]] = s.EMPTY
        if is_valid(*goal_pos) and grid[goal_pos[0]][goal_pos[1]] == s.OBSTACLE: grid[goal_pos[0]][goal_pos[1]] = s.EMPTY

        vis.update_search_view(message=f"Scenario: {scenario_name}")
        time.sleep(1.5)

        # --- Initial Planning ---
        print("\n--- Initial Planning ---")
        initial_paths = {}
        dstar_instances = {}

        for name, func in algorithms_to_run.items():
            print(f"\nRunning {name} (Initial)...")
            current_grid_for_algo = [row[:] for row in grid]
            vis.update_grid_reference(current_grid_for_algo)
            path, time_taken, nodes_exp, visited = None, 0, 0, set()

            if name == "D* Lite":
                dstar_instance = DStarLite(current_grid_for_algo, start_pos, goal_pos, vis)
                dstar_instances[name] = dstar_instance
                path, time_taken, nodes_exp, visited = dstar_instance.compute_shortest_path()
            else:
                path, time_taken, nodes_exp, visited = func(current_grid_for_algo, start_pos, goal_pos, vis)

            initial_paths[name] = path
            results[name] = results.get(name, {})
            results[name].update({
                'initial_path': path, 'initial_time': time_taken,
                'initial_nodes': nodes_exp, 'initial_visited': visited
            })
            path_len = len(path) - 1 if path else "N/A"
            print(f"{name} Initial Results: Time={time_taken:.6f}s, Nodes={nodes_exp}, PathLen={path_len}")
            vis.update_search_view(visited=visited, path=path, message=f"{name} Initial - Path: {path_len} nodes: {nodes_exp}")
            time.sleep(1)

        # --- Dynamic Change Simulation (If applicable) ---
        dynamic_change_func = config["dynamic"]
        changed_cells = None
        replan_start_pos = start_pos

        if dynamic_change_func:
            print("\n--- Applying Dynamic Change ---")
            ref_path = initial_paths.get("A*")
            dynamic_args = config["dynamic_args"] # Get the custom targets list
            # Apply changes to the ORIGINAL grid
            changed_cells = dynamic_change_func(grid, ref_path, *dynamic_args) # Pass targets
            vis.update_grid_reference(grid)

            if changed_cells:
                 vis.update_search_view(message=f"{len(changed_cells)} obstacle(s) added. Press Enter.")
                 input("Press Enter to continue to replanning...")

                 # --- Replanning Setup ---
                 if ref_path:
                     # Find node before the *first* obstacle for Dijkstra/A* start
                     # This part might need adjustment if target_cells were used
                     first_obstacle = changed_cells[0]
                     try:
                         obstacle_index = ref_path.index(first_obstacle)
                         if obstacle_index > 0:
                             replan_start_pos = ref_path[obstacle_index - 1]
                             print(f"Replanning for Dijkstra/A* starting from: {replan_start_pos}")
                         else:
                             replan_start_pos = start_pos # Obstacle was first step
                     except ValueError: # Obstacle might not have been on A* path if custom targets used
                         print(f"Obstacle {first_obstacle} not on A* initial path. Replanning from original start for Dijkstra/A*.")
                         replan_start_pos = start_pos
                 else:
                     replan_start_pos = start_pos

            else:
                 print("No dynamic change applied.")

            # --- Replanning ---
            if changed_cells:
                 print("\n--- Replanning After Change ---")
                 for name, func in algorithms_to_run.items():
                    print(f"\nRunning {name} (Replan)...")
                    current_grid_for_algo = grid
                    vis.update_grid_reference(current_grid_for_algo)
                    path, time_taken, nodes_exp, visited = None, 0, 0, set()

                    if name == "D* Lite":
                        dstar_instance = dstar_instances.get(name)
                        if dstar_instance:
                             dstar_instance.grid = current_grid_for_algo
                             dstar_instance.notify_change(changed_cells)
                             path, time_taken, nodes_exp, visited = dstar_instance.compute_shortest_path()
                        else: continue
                    else:
                        path, time_taken, nodes_exp, visited = func(current_grid_for_algo, replan_start_pos, goal_pos, vis)

                    results[name]['replan_path'] = path
                    results[name]['replan_time'] = time_taken
                    results[name]['replan_nodes'] = nodes_exp
                    results[name]['replan_visited'] = visited
                    path_len = len(path) - 1 if path else "N/A"
                    print(f"{name} Replan Results (from {replan_start_pos if name != 'D* Lite' else 'internal state'}): Time={time_taken:.6f}s, Nodes={nodes_exp}, PathLen(seg/full)={path_len}")
                    vis.update_search_view(visited=visited, path=path, message=f"{name} Replan - Path: {path_len} nodes: {nodes_exp}")
                    time.sleep(1)

        print(f"\n--- Scenario {scenario_name} Finished ---")
        time.sleep(1.5)


    # --- Final Summary ---
    print("\n\n" + "="*20 + " FINAL SUMMARY " + "="*20)
    for scenario_name, scenario_results in all_results.items():
         print(f"\n--- SCENARIO: {scenario_name} ---")
         # Determine replan_start_pos used in this scenario for accurate summary (bit repetitive)
         replan_start_pos = start_pos # Default
         if "Dynamic" in scenario_name:
             ref_path = scenario_results.get("A*", {}).get("initial_path")
             # Logic to find first changed cell (needs access to changed_cells if stored)
             # Simplified: Assume it was calculated correctly during the run
             # For exact summary, you might need to store changed_cells and replan_start_pos per scenario
             # Using A*'s result dictionary to retrieve path length info
             a_star_results = scenario_results.get("A*", {})
             if a_star_results.get("replan_path"): # Check if replan happened
                 # Re-calculate replan_start_pos based on A*'s initial path for consistency in summary logic
                 a_star_initial_path = a_star_results.get("initial_path")
                 # Need to know the *actual* first obstacle placed in *this* scenario run... this gets complex.
                 # Simplification: Assume the replan_start_pos logged during the run was correct.
                 # We will use the path lengths stored in results directly.

         for name, data in scenario_results.items():
             print(f"\n  Algorithm: {name}")
             init_len = len(data['initial_path']) - 1 if data.get('initial_path') else "N/A"
             if isinstance(init_len, int) and init_len < 0 : init_len = "N/A"
             print(f"    Initial: Time={data['initial_time']:.6f}s, Nodes={data['initial_nodes']}, PathLen={init_len}")

             if 'replan_time' in data:
                 replan_len_raw = len(data['replan_path']) - 1 if data.get('replan_path') else 0
                 if replan_len_raw < 0: replan_len_raw = 0 # Treat None path as 0 length for calculation

                 total_len_after_replan = "N/A" # Initialize

                 if name == "D* Lite":
                     total_len_after_replan = replan_len_raw if isinstance(replan_len_raw, int) and replan_len_raw >= 0 else "N/A"
                     path_label = "New Path Length (full, start->goal)"
                     print(f"    Replan:  Time={data['replan_time']:.6f}s, Nodes={data['replan_nodes']}, {path_label}={total_len_after_replan}")

                 else: # Dijkstra or A*
                     path_label = "New Path Length (replan_start->goal)"
                     initial_segment_len = "N/A"
                     # Need the replan_start_pos used *specifically* for this algorithm's replan run...
                     # Let's try to reconstruct the total length using the initial path data if possible
                     if data.get('initial_path') and data.get('replan_path'):
                          # Find where the initial path diverges / stops being relevant
                          # This requires knowing the actual replan_start_pos used.
                          # Simplified calculation (may not be perfect if replan_start_pos logic failed):
                          # Try finding the start of the replan segment in the initial path
                          replan_seg_start_node = data['replan_path'][0]
                          try:
                               replan_start_index = data['initial_path'].index(replan_seg_start_node)
                               initial_segment_len = replan_start_index
                          except (ValueError, IndexError):
                               initial_segment_len = "N/A" # Couldn't find node or paths empty
                     else:
                          initial_segment_len = "N/A"


                     if initial_segment_len != "N/A" and isinstance(replan_len_raw, int) and replan_len_raw >= 0:
                          total_len_after_replan = initial_segment_len + replan_len_raw
                     else:
                          total_len_after_replan = "N/A" # Mark as N/A if components missing

                     # Ensure N/A prints correctly if calculation failed
                     if total_len_after_replan == 0 and not data['replan_path']:
                         total_len_after_replan = "N/A"

                     replan_seg_len_str = str(replan_len_raw) if isinstance(replan_len_raw, int) and replan_len_raw >= 0 else "N/A"
                     print(f"    Replan:  Time={data['replan_time']:.6f}s, Nodes={data['replan_nodes']}, {path_label}={replan_seg_len_str} (Total Est. Length: {total_len_after_replan})")
             else:
                 print("    Replan:  Not performed for this scenario.")


    print("\nComparison finished. Close the window to exit.")
    vis.wait_for_quit()