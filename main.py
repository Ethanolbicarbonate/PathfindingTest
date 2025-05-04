# ----------- main.py (Modified for Scenarios) -----------
import pygame
import sys
import time
import random # For potential random obstacle placement
import settings as s
from grid_utils import create_grid, draw_grid_and_elements, is_valid
from algorithms import dijkstra, a_star, DStarLite
import os # Import os for directory handling
import re # Import re for sanitizing filenames
import tracemalloc # <<<--- IMPORT tracemalloc

SCREENSHOT_DIR = "screenshots"

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
    """Sets up STATIC obstacles based on a provided list."""
    for r in range(len(grid)):
        for c in range(len(grid[0])):
            grid[r][c] = s.EMPTY
    for r_obs, c_obs in obstacle_list:
        if is_valid(r_obs, c_obs):
            # Use STATIC obstacle type here
            grid[r_obs][c_obs] = s.OBSTACLE
        else:
            print(f"Warning: Obstacle coordinate { (r_obs, c_obs) } is out of bounds.")

def apply_dynamic_change_minor(grid, reference_path, target_cells=None):
    """Adds ONE DYNAMIC obstacle."""
    if target_cells:
        if isinstance(target_cells, list) and len(target_cells) > 0:
            obstacle_pos = target_cells[0]
            if is_valid(obstacle_pos[0], obstacle_pos[1]) and grid[obstacle_pos[0]][obstacle_pos[1]] == s.EMPTY:
                # Use DYNAMIC obstacle type
                grid[obstacle_pos[0]][obstacle_pos[1]] = s.DYNAMIC_OBSTACLE # <<<--- MODIFIED
                print(f"Applying CUSTOM MINOR dynamic change: Obstacle added at {obstacle_pos}")
                return [obstacle_pos]
            # ... (error handling) ...
            return None
        # ... (error handling) ...
        return None

    if not reference_path or len(reference_path) <= 2: return None
    potential_indices = list(range(1, len(reference_path) - 1)); random.shuffle(potential_indices)
    for idx in potential_indices:
        obstacle_pos = reference_path[idx]
        if obstacle_pos != start_pos and obstacle_pos != goal_pos:
             if is_valid(obstacle_pos[0], obstacle_pos[1]) and grid[obstacle_pos[0]][obstacle_pos[1]] == s.EMPTY:
                 # Use DYNAMIC obstacle type
                 grid[obstacle_pos[0]][obstacle_pos[1]] = s.DYNAMIC_OBSTACLE # <<<--- MODIFIED
                 print(f"Applying PATH-BASED MINOR dynamic change: Obstacle added at {obstacle_pos}")
                 return [obstacle_pos]
    print("Warning: Could not find suitable place for path-based minor change."); return None

def apply_dynamic_change_major(grid, reference_path, target_cells=None):
    """Adds MULTIPLE DYNAMIC obstacles."""
    changed_cells = []
    if target_cells:
        if isinstance(target_cells, list):
            count = 0
            for obstacle_pos in target_cells:
                 if is_valid(obstacle_pos[0], obstacle_pos[1]) and grid[obstacle_pos[0]][obstacle_pos[1]] == s.EMPTY:
                     # Use DYNAMIC obstacle type
                     grid[obstacle_pos[0]][obstacle_pos[1]] = s.DYNAMIC_OBSTACLE # <<<--- MODIFIED
                     changed_cells.append(obstacle_pos)
                     count +=1
                 # ... (error handling) ...
            if count > 0:
                print(f"Applying CUSTOM MAJOR dynamic change: {count} obstacles added: {changed_cells}")
                return changed_cells
            # ... (error handling) ...
            return None
        # ... (error handling) ...
        return None

    if not reference_path or len(reference_path) <= 4: return None
    num_obstacles = random.randint(3, 6)
    mid_point_index = len(reference_path) // 2
    indices_to_try = list(range(max(1, mid_point_index - num_obstacles // 2), min(len(reference_path) - 1, mid_point_index + num_obstacles // 2 + 1)))
    random.shuffle(indices_to_try)
    count = 0
    for idx in indices_to_try:
         if count >= num_obstacles: break
         obstacle_pos = reference_path[idx]
         if obstacle_pos != start_pos and obstacle_pos != goal_pos:
            if is_valid(obstacle_pos[0], obstacle_pos[1]) and grid[obstacle_pos[0]][obstacle_pos[1]] == s.EMPTY:
                # Use DYNAMIC obstacle type
                grid[obstacle_pos[0]][obstacle_pos[1]] = s.DYNAMIC_OBSTACLE # <<<--- MODIFIED
                changed_cells.append(obstacle_pos)
                count += 1
    if not changed_cells: return None
    else:
        print(f"Applying PATH-BASED MAJOR dynamic change: {len(changed_cells)} obstacles added: {changed_cells}")
        return changed_cells

# --- Screenshot Helper ---
def sanitize_filename(name):
    """Removes or replaces characters invalid for filenames."""
    name = name.replace(" ", "_") # Replace spaces
    name = re.sub(r'[\\/*?:"<>|]', "", name) # Remove other invalid chars
    return name

def save_screenshot(screen, scenario_name, algo_name, stage):
    """Saves the current screen state to a file."""
    try:
        # Create directory if it doesn't exist
        os.makedirs(SCREENSHOT_DIR, exist_ok=True)

        # Sanitize names for filename
        s_scenario = sanitize_filename(scenario_name)
        s_algo = sanitize_filename(algo_name)
        s_stage = sanitize_filename(stage)

        filename = f"{SCREENSHOT_DIR}/{s_scenario}_{s_algo}_{s_stage}.png"
        pygame.image.save(screen, filename)
        # print(f"Screenshot saved: {filename}") # Optional confirmation
    except Exception as e:
        print(f"Error saving screenshot {filename}: {e}")





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
        (r, 7) for r in range(0, s.GRID_HEIGHT_CELLS) if r not in range(6, 8) and r not in range(15, 17)
    ] + [
        (r, 11) for r in range(2, 18) if r not in range(11, 13)
    ] + [
        (8, c) for c in list(range(7, 9)) + list(range(10, 26))
    ] + [
        (r, 25) for r in list(range(0, 7)) + list(range(14, 20))
    ] + [
        (6, c) for c in range(16, 26)
    ] + [
        (11, c) for c in list(range(15, 21)) + list(range(22, 30))
    ] + [
        (r, 15) for r in range(13, 20)
    ] + [
        (r, 20) for r in list(range(11, 16)) + list(range(18, 20))
    ]

    # --- Dynamic Minor Change Targets ---
    # Define a list with ONE (row, col) tuple.
    # If empty or None, it will try to place obstacle on the calculated path.
    custom_minor_change_targets = [
        (11,21)
    ] # Example: Force obstacle at (10, 15)
    # custom_minor_change_targets = None # Example: Use path-based placement

    # --- Dynamic Major Change Targets ---
    # Define a list with MULTIPLE (row, col) tuples.
    # If empty or None, it will try to place multiple obstacles near the path.
    custom_major_change_targets = [(12,11), (11,11), (18,11), (19,11) ] # Example: Block passage
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

            # --- Variables for results ---
            path, time_taken, nodes_exp, visited = None, 0, 0, set()
            mem_peak_kb, mem_diff_kb = 0.0, 0.0 # Initialize memory results

            # <<<--- Start tracemalloc ---
            tracemalloc.start()
            snapshot1 = tracemalloc.take_snapshot()
            # --- Start timing AFTER starting tracemalloc ---
            start_time = time.perf_counter()

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

            # --- Stop timing BEFORE stopping tracemalloc ---
            end_time = time.perf_counter()
            time_taken = end_time - start_time # Calculate execution time

            # <<<--- Stop tracemalloc and get results ---
            snapshot2 = tracemalloc.take_snapshot()
            mem_stats = snapshot2.compare_to(snapshot1, 'lineno')
            mem_diff_bytes = sum(stat.size_diff for stat in mem_stats)
            mem_peak_bytes = tracemalloc.get_traced_memory()[1] # Peak during start/stop block
            tracemalloc.stop()

            # Convert to KB
            mem_diff_kb = mem_diff_bytes / 1024
            mem_peak_kb = mem_peak_bytes / 1024
            # --- End tracemalloc block ---

            # --- Store all results ---
            initial_paths[name] = path
            results[name] = results.get(name, {})
            results[name].update({
                'initial_path': path, 'initial_time': time_taken,
                'initial_nodes': nodes_exp, 'initial_visited': visited,
                'initial_mem_diff_kb': mem_diff_kb, 'initial_mem_peak_kb': mem_peak_kb # Add mem results
            })

            # --- Print results including memory ---
            path_len = len(path) - 1 if path else "N/A"
            if isinstance(path_len, int) and path_len < 0 : path_len = "N/A"
            print(f"{name} Initial Results:")
            print(f"  Time: {time_taken:.6f} s")
            print(f"  Nodes Expanded: {nodes_exp}")
            print(f"  Path Length: {path_len}")
            print(f"  Memory Peak: {mem_peak_kb:.2f} KB") # Print peak memory
            # print(f"  Memory Diff: {mem_diff_kb:.2f} KB") # Optional: print diff
            vis.update_search_view(visited=visited, path=path, message=f"{name} Initial - Path: {path_len} nodes: {nodes_exp}")
            save_screenshot(vis.screen, scenario_name, name, "InitialPlan") # <<<--- SAVE SCREENSHOT
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
                 save_screenshot(vis.screen, scenario_name, "Grid", "AfterChange") # <<<--- SAVE SCREENSHOT
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
                    mem_peak_kb, mem_diff_kb = 0.0, 0.0 # Initialize memory results

                    # <<<--- Start tracemalloc ---
                    tracemalloc.start()
                    snapshot1 = tracemalloc.take_snapshot()
                    # --- Start timing ---
                    start_time = time.perf_counter()

                    if name == "D* Lite":
                        dstar_instance = dstar_instances.get(name)
                        if dstar_instance:
                             dstar_instance.grid = current_grid_for_algo
                             dstar_instance.notify_change(changed_cells)
                             path, time_taken, nodes_exp, visited = dstar_instance.compute_shortest_path()
                        else: continue
                    else:
                        path, time_taken, nodes_exp, visited = func(current_grid_for_algo, replan_start_pos, goal_pos, vis)

                    # --- Stop timing ---
                    end_time = time.perf_counter()
                    time_taken = end_time - start_time

                    # <<<--- Stop tracemalloc and get results ---
                    snapshot2 = tracemalloc.take_snapshot()
                    mem_stats = snapshot2.compare_to(snapshot1, 'lineno')
                    mem_diff_bytes = sum(stat.size_diff for stat in mem_stats)
                    mem_peak_bytes = tracemalloc.get_traced_memory()[1]
                    tracemalloc.stop()

                    # Convert to KB
                    mem_diff_kb = mem_diff_bytes / 1024
                    mem_peak_kb = mem_peak_bytes / 1024
                    # --- End tracemalloc block ---


                    # --- Store results ---
                    # Ensure results[name] exists from initial run
                    if name in results:
                         results[name]['replan_path'] = path
                         results[name]['replan_time'] = time_taken
                         results[name]['replan_nodes'] = nodes_exp
                         results[name]['replan_visited'] = visited
                         results[name]['replan_mem_diff_kb'] = mem_diff_kb # Store memory
                         results[name]['replan_mem_peak_kb'] = mem_peak_kb # Store memory
                    else:
                         print(f"Warning: No initial results found for {name} to store replan data.")


                    # --- Print results including memory ---
                    path_len = len(path) - 1 if path else "N/A"
                    if isinstance(path_len, int) and path_len < 0 : path_len = "N/A"
                    print(f"{name} Replan Results (from {replan_start_pos if name != 'D* Lite' else 'internal state'}):")
                    print(f"  Replan Time: {time_taken:.6f} s")
                    print(f"  Replan Nodes Expanded: {nodes_exp}")
                    print(f"  New Path Length (seg/full): {path_len}")
                    print(f"  Memory Peak: {mem_peak_kb:.2f} KB") # Print peak memory
                    # print(f"  Memory Diff: {mem_diff_kb:.2f} KB") # Optional
                    vis.update_search_view(visited=visited, path=path, message=f"{name} Replan - Path: {path_len} nodes: {nodes_exp}")
                    save_screenshot(vis.screen, scenario_name, name, "Replan") # <<<--- SAVE SCREENSHOT
                    time.sleep(1)

        print(f"\n--- Scenario {scenario_name} Finished ---")
        time.sleep(1.5)


    # --- Final Summary ---
    print("\n\n" + "="*20 + " FINAL SUMMARY " + "="*20)
    for scenario_name, scenario_results in all_results.items():
         print(f"\n--- SCENARIO: {scenario_name} ---")
         for name, data in scenario_results.items():
             print(f"\n  Algorithm: {name}")
             init_len = len(data['initial_path']) - 1 if data.get('initial_path') else "N/A"; il_kb = data.get('initial_mem_peak_kb', 0.0)
             if isinstance(init_len, int) and init_len < 0 : init_len = "N/A"
             print(f"    Initial: Time={data['initial_time']:.6f}s, Nodes={data['initial_nodes']}, PathLen={init_len}, MemPeak={il_kb:.2f}KB") # Add Mem

             if 'replan_time' in data:
                  # Use the Option A print logic here, adding memory
                  replan_len_raw = len(data['replan_path']) - 1 if data.get('replan_path') else 0
                  if replan_len_raw < 0: replan_len_raw = 0
                  rl_kb = data.get('replan_mem_peak_kb', 0.0)
                  total_len_after_replan = "N/A"
                  path_label = ""

                  if name == "D* Lite":
                      total_len_after_replan = replan_len_raw if isinstance(replan_len_raw, int) and replan_len_raw >= 0 else "N/A"
                      path_label = "New Path Length (full, start->goal)"
                      print(f"    Replan:  Time={data['replan_time']:.6f}s, Nodes={data['replan_nodes']}, {path_label}={total_len_after_replan}, MemPeak={rl_kb:.2f}KB") # Add Mem
                  else: # Dijkstra or A*
                      # ... [Calculation logic for total_len_after_replan from Option A] ...
                      path_label = "New Path Length (replan_start->goal)"
                      replan_seg_len_str = str(replan_len_raw) if isinstance(replan_len_raw, int) and replan_len_raw >= 0 else "N/A"
                      # Need to calculate total_len_after_replan properly based on Option A logic
                      print(f"    Replan:  Time={data['replan_time']:.6f}s, Nodes={data['replan_nodes']}, {path_label}={replan_seg_len_str} (Total Est. Length: {total_len_after_replan}), MemPeak={rl_kb:.2f}KB") # Add Mem
             else:
                 print("    Replan:  Not performed for this scenario.")


    print("\nComparison finished. Screenshots saved in 'screenshots' folder. Close the window to exit.")
    vis.wait_for_quit()