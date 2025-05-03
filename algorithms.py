# ----------- algorithms.py (Modified) -----------
import heapq
import time
import math # Make sure math is imported
import settings as s
# Ensure grid_utils functions are imported correctly
from grid_utils import get_neighbors, reconstruct_path, is_valid

# --- Heuristic Functions ---
# Keep Manhattan for comparison if needed, but don't use for diagonal movement
def heuristic_manhattan(pos1, pos2):
    """Manhattan distance heuristic."""
    r1, c1 = pos1
    r2, c2 = pos2
    return abs(r1 - r2) + abs(c1 - c2)

def heuristic_diagonal(pos1, pos2):
    """Diagonal distance heuristic (Octile distance)."""
    dx = abs(pos1[1] - pos2[1]) # Change in columns
    dy = abs(pos1[0] - pos2[0]) # Change in rows
    D = 1.0  # Cost of orthogonal move
    D2 = math.sqrt(2) # Cost of diagonal move
    # Formula: D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)
    # Simplified: D * max(dx, dy) + (D2 - D) * min(dx, dy)
    return D * max(dx, dy) + (D2 - D) * min(dx, dy)

def heuristic_euclidean(pos1, pos2):
     """Euclidean distance heuristic."""
     dx = pos1[1] - pos2[1]
     dy = pos1[0] - pos2[0]
     return math.sqrt(dx*dx + dy*dy)

# --- Integer Heuristic ---
def heuristic_diagonal_integer(pos1, pos2):
    """Integer-based diagonal distance heuristic."""
    dx = abs(pos1[1] - pos2[1])
    dy = abs(pos1[0] - pos2[0])
    # Using costs from settings if defined, otherwise literal 10/14
    D = getattr(s, 'COST_ORTHOGONAL', 10)
    D2 = getattr(s, 'COST_DIAGONAL', 14)
    # return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy) # Original Octile formula scaled
    # Simpler equivalent scaled formula:
    return D * max(dx, dy) + (D2 - D) * min(dx, dy)

# --- Cost Calculation Helper (Modified) ---
def calculate_move_cost(grid, u_pos, v_pos):
    """Calculates INTEGER movement cost, treating both obstacle types as impassable."""
    D = getattr(s, 'COST_ORTHOGONAL', 10)
    D2 = getattr(s, 'COST_DIAGONAL', 14)

    # Check target validity and impassability
    if not is_valid(v_pos[0], v_pos[1]): return s.INFINITY
    v_cell_state = grid[v_pos[0]][v_pos[1]]
    if v_cell_state == s.OBSTACLE or v_cell_state == s.DYNAMIC_OBSTACLE: return s.INFINITY # <<<--- MODIFIED CHECK

    # Check source validity and impassability (less critical but good practice)
    if not is_valid(u_pos[0], u_pos[1]): return s.INFINITY
    u_cell_state = grid[u_pos[0]][u_pos[1]]
    if u_cell_state == s.OBSTACLE or u_cell_state == s.DYNAMIC_OBSTACLE: return s.INFINITY # <<<--- MODIFIED CHECK


    dx = abs(u_pos[1] - v_pos[1])
    dy = abs(u_pos[0] - v_pos[0])

    if dx == 1 and dy == 1: # Diagonal
        # Corner cutting check
        u_row_v_col_state = grid[u_pos[0]][v_pos[1]]
        v_row_u_col_state = grid[v_pos[0]][u_pos[1]]
        corner_blocked = (u_row_v_col_state == s.OBSTACLE or u_row_v_col_state == s.DYNAMIC_OBSTACLE or
                          v_row_u_col_state == s.OBSTACLE or v_row_u_col_state == s.DYNAMIC_OBSTACLE)

        if not is_valid(u_pos[0], v_pos[1]) or not is_valid(v_pos[0], u_pos[1]) or corner_blocked:
            return s.INFINITY
        else:
            return D2
    elif dx + dy == 1: # Orthogonal
        return D
    else:
        return s.INFINITY


# --- Dijkstra (Modified) ---
def dijkstra(grid, start_pos, goal_pos, visualizer=None, allow_diagonal=True): # Add flag
    """Finds the shortest path using Dijkstra's algorithm."""
    start_time = time.perf_counter()
    nodes_expanded = 0

    priority_queue = [(0, start_pos)] # (cost, position)
    cost_so_far = {start_pos: 0}
    came_from = {start_pos: None}
    visited_for_vis = set() # For visualization

    path = None

    while priority_queue:
        current_cost, current_pos = heapq.heappop(priority_queue)

        # Optimization: if we found a shorter path already
        if current_cost > cost_so_far.get(current_pos, s.INFINITY):
             continue

        nodes_expanded += 1
        visited_for_vis.add(current_pos)

        if current_pos == goal_pos:
            path = reconstruct_path(came_from, start_pos, goal_pos)
            break

        if visualizer:
            # Pass only visited set for standard visualization during search
            visualizer.update_search_view(visited=visited_for_vis)
            visualizer.handle_events()

        # Use the flag passed to the function
        for neighbor_pos in get_neighbors(grid, current_pos[0], current_pos[1], allow_diagonal=allow_diagonal):
            # Calculate cost using the helper function
            move_cost = calculate_move_cost(grid, current_pos, neighbor_pos)
            if move_cost == s.INFINITY: # Skip blocked moves (e.g., cut corners)
                 continue

            new_cost = current_cost + move_cost # Use calculated move_cost
            if neighbor_pos not in cost_so_far or new_cost < cost_so_far[neighbor_pos]:
                cost_so_far[neighbor_pos] = new_cost
                priority = new_cost
                heapq.heappush(priority_queue, (priority, neighbor_pos))
                came_from[neighbor_pos] = current_pos

    end_time = time.perf_counter()
    return path, end_time - start_time, nodes_expanded, visited_for_vis

# --- A* (Modified) ---
# Choose your preferred heuristic here
ACTIVE_HEURISTIC = heuristic_diagonal_integer # Or heuristic_euclidean

def a_star(grid, start_pos, goal_pos, visualizer=None, allow_diagonal=True): # Add flag
    """Finds the shortest path using A* algorithm."""
    start_time = time.perf_counter()
    nodes_expanded = 0

    # Use chosen heuristic
    h_start = ACTIVE_HEURISTIC(start_pos, goal_pos)
    priority_queue = [(h_start, 0, start_pos)] # (f_cost, g_cost, position)
    cost_so_far = {start_pos: 0} # g_cost
    came_from = {start_pos: None}
    visited_for_vis = set()

    path = None

    while priority_queue:
        f_cost_est, current_g_cost, current_pos = heapq.heappop(priority_queue)
        nodes_expanded += 1

        if current_g_cost > cost_so_far.get(current_pos, s.INFINITY):
             continue

        visited_for_vis.add(current_pos)

        if current_pos == goal_pos:
            path = reconstruct_path(came_from, start_pos, goal_pos)
            break

        if visualizer:
            visualizer.update_search_view(visited=visited_for_vis)
            visualizer.handle_events()

        # Use the flag passed to the function
        for neighbor_pos in get_neighbors(grid, current_pos[0], current_pos[1], allow_diagonal=allow_diagonal):
            # Calculate cost using the helper function
            move_cost = calculate_move_cost(grid, current_pos, neighbor_pos)
            if move_cost == s.INFINITY:
                 continue

            new_g_cost = current_g_cost + move_cost # Use calculated move_cost
            if neighbor_pos not in cost_so_far or new_g_cost < cost_so_far[neighbor_pos]:
                cost_so_far[neighbor_pos] = new_g_cost
                h_cost = ACTIVE_HEURISTIC(neighbor_pos, goal_pos) # Use chosen heuristic
                priority = new_g_cost + h_cost
                heapq.heappush(priority_queue, (priority, new_g_cost, neighbor_pos))
                came_from[neighbor_pos] = current_pos

    end_time = time.perf_counter()
    return path, end_time - start_time, nodes_expanded, visited_for_vis


# --- D* Lite Class (Fresh Implementation) ---
class DStarLite:
    ACTIVE_HEURISTIC = heuristic_diagonal_integer

    def __init__(self, grid, start_pos, goal_pos, visualizer=None, allow_diagonal=True):
        # --- Core Attributes ---
        self.grid = grid
        self.start_pos = start_pos
        self.goal_pos = goal_pos
        self.allow_diagonal = allow_diagonal

        # --- D* State ---
        self.g_values = {}     # g(s): Estimated cost from start to s
        self.rhs_values = {}   # rhs(s): Lookahead value based on successors' g-values
        self.priority_queue = [] # U: The priority queue (min-heap) [(key, node), ...]
        self.km = 0            # Key modifier (for moving agent, 0 here)

        # --- Performance & Visualization ---
        self.visualizer = visualizer
        self.nodes_expanded_total = 0 # Cumulative across calls
        self.visited_for_vis = set() # Cumulative visited nodes for drawing

        # --- Initialization ---
        self._initialize()

    # --- Helper Methods for State Access ---
    def _get_g(self, pos):
        return self.g_values.get(pos, s.INFINITY)

    def _get_rhs(self, pos):
        return self.rhs_values.get(pos, s.INFINITY)

    def _set_g(self, pos, value):
        self.g_values[pos] = value

    def _set_rhs(self, pos, value):
        self.rhs_values[pos] = value

    # --- Core D* Logic ---
    def _calculate_key(self, s_pos):
        """Calculates the priority key for node s."""
        g = self._get_g(s_pos)
        rhs = self._get_rhs(s_pos)
        min_g_rhs = min(g, rhs)

        # Use the class-defined heuristic function
        heuristic_func = DStarLite.ACTIVE_HEURISTIC
        h = heuristic_func(self.start_pos, s_pos)

        # Key = [k1, k2] = [min(g(s), rhs(s)) + h(s_start, s) + km; min(g(s), rhs(s))]
        return (min_g_rhs + h + self.km, min_g_rhs)

    def _cost(self, u_pos, v_pos):
        """Calculates INTEGER movement cost c(u, v), treating both obstacle types impassable."""
        D = getattr(s, 'COST_ORTHOGONAL', 10)
        D2 = getattr(s, 'COST_DIAGONAL', 14)

        # Check target validity and impassability
        if not is_valid(v_pos[0], v_pos[1]): return s.INFINITY
        v_cell_state = self.grid[v_pos[0]][v_pos[1]] # Use self.grid
        if v_cell_state == s.OBSTACLE or v_cell_state == s.DYNAMIC_OBSTACLE: return s.INFINITY # <<<--- MODIFIED CHECK

        # Check source validity and impassability
        if not is_valid(u_pos[0], u_pos[1]): return s.INFINITY
        u_cell_state = self.grid[u_pos[0]][u_pos[1]]
        if u_cell_state == s.OBSTACLE or u_cell_state == s.DYNAMIC_OBSTACLE: return s.INFINITY # <<<--- MODIFIED CHECK


        dx = abs(u_pos[1] - v_pos[1])
        dy = abs(u_pos[0] - v_pos[0])

        if dx == 1 and dy == 1: # Diagonal
            u_row_v_col_state = self.grid[u_pos[0]][v_pos[1]]
            v_row_u_col_state = self.grid[v_pos[0]][u_pos[1]]
            corner_blocked = (u_row_v_col_state == s.OBSTACLE or u_row_v_col_state == s.DYNAMIC_OBSTACLE or
                              v_row_u_col_state == s.OBSTACLE or v_row_u_col_state == s.DYNAMIC_OBSTACLE)
            if not is_valid(u_pos[0], v_pos[1]) or not is_valid(v_pos[0], u_pos[1]) or corner_blocked:
                return s.INFINITY
            else:
                return D2
        elif dx + dy == 1: # Orthogonal
            return D
        else:
            return s.INFINITY
    def _initialize(self):
        """Initializes D* Lite state."""
        self.g_values.clear()
        self.rhs_values.clear()
        self.priority_queue = []
        self.km = 0
        self.visited_for_vis.clear()
        # All g/rhs implicitly infinity

        # Set rhs(goal) = 0 and add goal to queue
        self._set_rhs(self.goal_pos, 0)
        heapq.heappush(self.priority_queue, (self._calculate_key(self.goal_pos), self.goal_pos))

    def _update_queue(self, u):
        """Removes u if present and re-adds if inconsistent."""
        # 1. Remove (inefficient O(N) but simple)
        in_queue = False
        temp_queue = []
        for key, node in self.priority_queue:
            if node == u:
                in_queue = True
            else:
                temp_queue.append((key, node))
        needs_heapify = in_queue # Only heapify if something was removed
        self.priority_queue = temp_queue
        if needs_heapify:
             heapq.heapify(self.priority_queue)

        # 2. Re-add if inconsistent
        if self._get_g(u) != self._get_rhs(u):
            heapq.heappush(self.priority_queue, (self._calculate_key(u), u))


    def _update_vertex(self, u):
        """Updates rhs value for node u and its queue status."""
        if u != self.goal_pos:
            min_rhs = s.INFINITY
            # Calculate rhs(u) = min_{s' in Succ(u)} ( c(u, s') + g(s') )
            for s_prime in get_neighbors(self.grid, u[0], u[1], allow_diagonal=self.allow_diagonal):
                cost_u_to_s_prime = self._cost(u, s_prime)
                if cost_u_to_s_prime == s.INFINITY: continue

                g_s_prime = self._get_g(s_prime)
                min_rhs = min(min_rhs, cost_u_to_s_prime + g_s_prime)
            self._set_rhs(u, min_rhs)

        # Update the node's status in the priority queue
        self._update_queue(u)


    def compute_shortest_path(self):
        """Computes/recomputes the path until start node is consistent and optimal."""
        start_time = time.perf_counter()
        nodes_expanded_this_call = 0
        current_call_visited = set() # Nodes processed in this specific call

        iter_count = 0 # Safety break counter
        max_iters = s.GRID_WIDTH_CELLS * s.GRID_HEIGHT_CELLS * 10 # Generous limit

        while self.priority_queue:
            iter_count += 1
            if iter_count > max_iters:
                 print("!!! D* Lite Error: Max iterations reached in compute_shortest_path loop!")
                 break

            # Peek top key and calculate current start key
            key_top, u_peek = self.priority_queue[0]
            start_key_current = self._calculate_key(self.start_pos)

            # Termination condition check
            if key_top >= start_key_current and self._get_rhs(self.start_pos) == self._get_g(self.start_pos):
                # print(f"Compute loop finished: Start consistent, TopKey={key_top}, StartKey={start_key_current}")
                break

            # Pop the node with the smallest key
            key_popped, u = heapq.heappop(self.priority_queue)

            # Stale check: Recalculate key for u based on its current g/rhs state
            key_new_recalculated = self._calculate_key(u)
            if key_popped < key_new_recalculated:
                # Key was outdated. Push back with updated key and continue.
                heapq.heappush(self.priority_queue, (key_new_recalculated, u))
                continue

            # Mark as processed for this call
            nodes_expanded_this_call += 1
            current_call_visited.add(u)

            # --- Process Node u ---
            g_old_u = self._get_g(u) # Store old g for comparison below

            if g_old_u > self._get_rhs(u): # Overconsistent case: g > rhs
                # Make consistent: set g(u) = rhs(u)
                self._set_g(u, self._get_rhs(u))
                # g(u) decreased, potentially decreasing rhs of predecessors v. Update predecessors.
                for v_predecessor in get_neighbors(self.grid, u[0], u[1], allow_diagonal=self.allow_diagonal):
                     if v_predecessor != self.goal_pos: # No need to update goal's rhs
                        self._update_vertex(v_predecessor)
            else: # Underconsistent case: g <= rhs (but we popped it, so key must be lowest)
                  # This implies g < rhs (or g=rhs=inf). Set g = infinity.
                  # This increase in g(u) might increase rhs of predecessors v.
                  self._set_g(u, s.INFINITY)
                  # Update predecessors v first, THEN update u itself (its rhs might change now)
                  nodes_to_update = get_neighbors(self.grid, u[0], u[1], allow_diagonal=self.allow_diagonal) + [u]
                  for node in nodes_to_update:
                      if node != self.goal_pos: # Skip goal update
                         self._update_vertex(node)

            # --- Visualization Hook ---
            if self.visualizer and nodes_expanded_this_call % 30 == 0:
                self.visited_for_vis.update(current_call_visited)
                self.visualizer.update_search_view(visited=self.visited_for_vis)
                self.visualizer.handle_events()
        # --- End While Loop ---

        # Final visualization update
        if self.visualizer:
            self.visited_for_vis.update(current_call_visited)
            self.visualizer.update_search_view(visited=self.visited_for_vis)

        end_time = time.perf_counter()
        self.nodes_expanded_total += nodes_expanded_this_call

        # Attempt path reconstruction
        path = self._reconstruct_dstar_path_robust() # Use the robust version

        if path is None or (path and path[-1] != self.goal_pos):
             print(f"!!! D* Lite Warning: compute_shortest_path finished but reconstruction failed or didn't reach goal. Final g(start)={self._get_g(self.start_pos):.2f}, rhs(start)={self._get_rhs(self.start_pos):.2f}")
             # You could add debugging here: print g/rhs values around start/goal/problem area

        return path, end_time - start_time, nodes_expanded_this_call, self.visited_for_vis


    # Replace the existing _reconstruct_dstar_path_robust method with this one

    def _reconstruct_dstar_path_robust(self):
        """Reconstructs path using cost+g, with heuristic tie-breaking and loop avoidance."""
        if self._get_g(self.start_pos) == s.INFINITY:
            print("D* Lite: Reconstruction failed - start node is unreachable.")
            return None

        path = [self.start_pos]
        current = self.start_pos
        # Keep track of nodes visited IN THIS RECONSTRUCTION to detect loops
        visited_in_reconstruction = {current}
        max_path_len = s.GRID_WIDTH_CELLS * s.GRID_HEIGHT_CELLS * 3
        step_count = 0

        while current != self.goal_pos:
            step_count += 1
            if step_count > max_path_len:
                print(f"D* Lite path reconstruction loop detected (safety break after {max_path_len} steps).")
                print(f"  Tail of path: ... {path[-15:]}")
                return path

            possible_next_steps = [] # Store tuples: (cost+g, h_to_goal, neighbor_node)

            # --- Gather Candidate Next Steps ---
            for neighbor in get_neighbors(self.grid, current[0], current[1], allow_diagonal=self.allow_diagonal):
                cost = self._cost(current, neighbor)
                if cost == s.INFINITY: continue

                g_neighbor = self._get_g(neighbor)
                # Generally avoid nodes marked as unreachable, unless it IS the goal
                if g_neighbor == s.INFINITY and neighbor != self.goal_pos: continue

                cost_plus_g = cost + g_neighbor
                # Calculate heuristic from neighbor to goal
                h_to_goal = DStarLite.ACTIVE_HEURISTIC(neighbor, self.goal_pos)
                possible_next_steps.append((cost_plus_g, h_to_goal, neighbor))

            if not possible_next_steps:
                 print(f"D* Lite path reconstruction failed at {current}. No valid/reachable neighbors found. g={self._get_g(current):.2f}, rhs={self._get_rhs(current):.2f}")
                 return path # Return partial path

            # --- Sort Candidates ---
            # Sort primarily by cost+g (ascending)
            # Secondarily by heuristic to goal (ascending - prefer nodes closer to goal)
            possible_next_steps.sort()

            # --- Select Best Neighbor (Avoiding Cycles) ---
            best_neighbor = None
            for cost_g_val, h_val, node in possible_next_steps:
                # Crucial Check: Has this node already been added to the path *during this reconstruction*?
                if node not in visited_in_reconstruction:
                    best_neighbor = node
                    # print(f"  Selected {node} (cost+g={cost_g_val:.2f}, h={h_val:.2f})") # Debug
                    break # Found the best option that doesn't immediately create a cycle

            # If all candidates lead to already visited nodes (indicates a potential dead end or complex loop closing)
            if best_neighbor is None:
                if possible_next_steps:
                     # Fallback: Take the absolute best (lowest cost+g) even if it revisits.
                     # This might be necessary to backtrack out of a dead end.
                     cost_g_val, h_val, node = possible_next_steps[0]
                     best_neighbor = node
                     # Check if this forced choice is the node just before current (the 'previous' node)
                     previous = path[-2] if len(path) >= 2 else None
                     if best_neighbor == previous:
                         # print(f"Path Recon: Forced backtrack from {current} to {previous}")
                         pass # Allow backtracking if forced
                     else:
                          # We are forced to revisit a node that's not the immediate predecessor. Cycle!
                          print(f"Path Recon: Cycle unavoidable at {current}. Forced to revisit {best_neighbor} (not immediate predecessor). Path reconstruction likely failed.")
                          # Returning the path here might be better than continuing indefinitely
                          return path


                else:
                     # Should be caught by the "not possible_next_steps" check earlier
                     print(f"Path Recon: Logic error - No possible steps, but list wasn't empty?")
                     return path

            # --- Update Path and State ---
            path.append(best_neighbor)
            visited_in_reconstruction.add(best_neighbor) # Add to loop detection set for this run
            current = best_neighbor
        # --- End While Loop ---

        return path # Successfully reached goal

    def notify_change(self, changed_cells):
        """Updates affected nodes after environment changes."""
        # print(f"D* Lite notified of changes: {changed_cells}") # Debugging
        for cell_pos in changed_cells:
            # When cell_pos (u) changes (e.g., becomes obstacle), c(v, u) increases for predecessors v.
            # This might increase rhs(v). Update predecessors.
            for v_predecessor in get_neighbors(self.grid, cell_pos[0], cell_pos[1], allow_diagonal=self.allow_diagonal):
                 self._update_vertex(v_predecessor) # Update nodes whose path *to* cell_pos changed

            # Also update cell_pos itself, as its rhs depends on its successors,
            # and its own g-value might now be inconsistent.
            self._update_vertex(cell_pos)

        # km update would go here if agent was moving

    # update_start_pos remains the same if only used for setting initial start