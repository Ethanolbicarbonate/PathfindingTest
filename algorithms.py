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

# --- Cost Calculation Helper (Used by Dijkstra and A*) ---
def calculate_move_cost(grid, u_pos, v_pos):
    """Calculates movement cost, handling diagonals and preventing corner cutting."""
    # Check target cell just in case (should be okay if get_neighbors worked)
    if not is_valid(v_pos[0], v_pos[1]) or grid[v_pos[0]][v_pos[1]] == s.OBSTACLE:
        return s.INFINITY

    dx = abs(u_pos[1] - v_pos[1])
    dy = abs(u_pos[0] - v_pos[0])

    if dx == 1 and dy == 1: # Diagonal move
        # Prevent cutting corners: Check orthogonal neighbours needed for diagonal
        # Check grid[u_row][v_col] and grid[v_row][u_col]
        if (not is_valid(u_pos[0], v_pos[1]) or grid[u_pos[0]][v_pos[1]] == s.OBSTACLE or
            not is_valid(v_pos[0], u_pos[1]) or grid[v_pos[0]][u_pos[1]] == s.OBSTACLE):
            return s.INFINITY # Corner is blocked
        else:
            return math.sqrt(2) # Diagonal cost
    elif dx + dy == 1: # Orthogonal move
        return 1.0 # Orthogonal cost
    else:
        # Should not happen if u_pos and v_pos are direct neighbours
        print(f"Warning: Cost calculation between non-neighbors? {u_pos} -> {v_pos}")
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
ACTIVE_HEURISTIC = heuristic_diagonal # Or heuristic_euclidean

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


# --- D* Lite (Modified) ---
class DStarLite:
    # Use chosen heuristic
    ACTIVE_HEURISTIC = heuristic_diagonal # Or heuristic_euclidean

    def __init__(self, grid, start_pos, goal_pos, visualizer=None, allow_diagonal=True): # Add flag
        self.grid = grid
        self.start_pos = start_pos
        self.goal_pos = goal_pos
        self.visualizer = visualizer
        self.allow_diagonal = allow_diagonal # Store flag

        self.g_values = {}
        self.rhs_values = {}
        self.priority_queue = []
        self.km = 0
        self.nodes_expanded_total = 0
        self.visited_for_vis = set()

        self._initialize()

    def _get_g(self, pos):
        return self.g_values.get(pos, s.INFINITY)

    def _get_rhs(self, pos):
        return self.rhs_values.get(pos, s.INFINITY)

    def _set_g(self, pos, value):
        self.g_values[pos] = value

    def _set_rhs(self, pos, value):
        self.rhs_values[pos] = value

    def _calculate_key(self, pos):
        g = self._get_g(pos)
        rhs = self._get_rhs(pos)
        min_g_rhs = min(g, rhs)
        # Use chosen heuristic
        heuristic_func = DStarLite.ACTIVE_HEURISTIC
        h = heuristic_func(self.start_pos, pos)
        return (min_g_rhs + h + self.km, min_g_rhs) # Full key needed if agent moves

    def _cost(self, u_pos, v_pos):
        """Cost of moving from u to v. Handles diagonals and corner cutting."""
        # Replicate logic from calculate_move_cost directly here
        if not is_valid(v_pos[0], v_pos[1]) or self.grid[v_pos[0]][v_pos[1]] == s.OBSTACLE:
            return s.INFINITY
        # Check source too, though usually okay
        if not is_valid(u_pos[0], u_pos[1]) or self.grid[u_pos[0]][u_pos[1]] == s.OBSTACLE:
             return s.INFINITY

        dx = abs(u_pos[1] - v_pos[1])
        dy = abs(u_pos[0] - v_pos[0])

        if dx == 1 and dy == 1: # Diagonal move
            # Prevent cutting corners
            if (not is_valid(u_pos[0], v_pos[1]) or self.grid[u_pos[0]][v_pos[1]] == s.OBSTACLE or
                not is_valid(v_pos[0], u_pos[1]) or self.grid[v_pos[0]][u_pos[1]] == s.OBSTACLE):
                return s.INFINITY # Corner is blocked
            else:
                return math.sqrt(2) # Diagonal cost
        elif dx + dy == 1: # Orthogonal move
            return 1.0 # Orthogonal cost
        else:
            # Should not happen for neighbors
            return s.INFINITY


    def _initialize(self):
        self.g_values = {}
        self.rhs_values = {}
        self.priority_queue = []
        self.km = 0
        self._set_rhs(self.goal_pos, 0)
        heapq.heappush(self.priority_queue, (self._calculate_key(self.goal_pos), self.goal_pos))

    def _update_vertex(self, pos):
        if pos != self.goal_pos:
            min_rhs = s.INFINITY
            # Pass allow_diagonal flag
            for neighbor in get_neighbors(self.grid, pos[0], pos[1], allow_diagonal=self.allow_diagonal):
                 # Calculate cost from neighbor to pos
                 cost_neighbor_to_pos = self._cost(neighbor, pos)
                 g_neighbor = self._get_g(neighbor)
                 # D* Lite looks forward from successors: rhs(u) = min_{s' in Succ(u)}( c(u, s') + g(s') )
                 # So we need cost(pos, neighbor) + g(neighbor)
                 cost_pos_to_neighbor = self._cost(pos, neighbor) # Calculate cost
                 min_rhs = min(min_rhs, cost_pos_to_neighbor + g_neighbor)
            self._set_rhs(pos, min_rhs)

        # Remove node from queue (inefficient method)
        self.priority_queue = [(key, node) for key, node in self.priority_queue if node != pos]
        heapq.heapify(self.priority_queue)

        if self._get_g(pos) != self._get_rhs(pos):
            heapq.heappush(self.priority_queue, (self._calculate_key(pos), pos))

    def compute_shortest_path(self):
        """Computes or recomputes the path after initialization or changes. (Refined Loop)"""
        start_time = time.perf_counter()
        nodes_expanded_this_call = 0
        current_call_visited = set()

        # Ensure start key calculation uses current state
        # No real change needed here, calculation is dynamic

        while self.priority_queue: # Continue as long as there are inconsistent nodes to process
            # Peek at the top key/node without popping yet
            key_top, u_top = self.priority_queue[0]

            # Recalculate the start node's key based on its CURRENT g/rhs
            start_key_current = self._calculate_key(self.start_pos)

            # Termination condition: Top key is >= start's key AND start is consistent
            if key_top >= start_key_current and self._get_rhs(self.start_pos) == self._get_g(self.start_pos):
                break # Stop when start is consistent and optimal according to PQ

            # --- Pop the node with the smallest key ---
            key_old_popped, u = heapq.heappop(self.priority_queue)

            # Recalculate the key for the popped node 'u' based on its current g/rhs
            key_new_recalculated = self._calculate_key(u)

            # Check if the key used for popping (key_old_popped) is stale compared to its current state
            if key_old_popped < key_new_recalculated:
                # Key was outdated (node's g/rhs improved since it was added/updated)
                # Push it back with the up-to-date key
                heapq.heappush(self.priority_queue, (key_new_recalculated, u))
                continue # Process next node in PQ

            # Process the node 'u'
            nodes_expanded_this_call += 1
            current_call_visited.add(u)

            if self._get_g(u) > self._get_rhs(u): # Overconsistent -> make consistent
                self._set_g(u, self._get_rhs(u))
                # Update predecessors (neighbors whose rhs depends on u's g)
                for neighbor in get_neighbors(self.grid, u[0], u[1], allow_diagonal=self.allow_diagonal):
                    self._update_vertex(neighbor)
            else: # Underconsistent -> make consistent (g(u) was infinity or too low)
                 self._set_g(u, s.INFINITY)
                 # Update itself first, then predecessors
                 nodes_to_update = [u] + get_neighbors(self.grid, u[0], u[1], allow_diagonal=self.allow_diagonal)
                 for node_to_update in nodes_to_update:
                     self._update_vertex(node_to_update)

            # Visualization Hook (Optional)
            if self.visualizer and nodes_expanded_this_call % 20 == 0: # Update less frequently
                self.visited_for_vis.update(current_call_visited)
                self.visualizer.update_search_view(visited=self.visited_for_vis)
                self.visualizer.handle_events()

        # Final visualization update
        if self.visualizer:
            self.visited_for_vis.update(current_call_visited)
            self.visualizer.update_search_view(visited=self.visited_for_vis)

        end_time = time.perf_counter()
        self.nodes_expanded_total += nodes_expanded_this_call

        path = self._reconstruct_dstar_path() # Attempt reconstruction

        # --- Add a check AFTER reconstruction ---
        if path is None or (path and path[-1] != self.goal_pos):
             print(f"!!! D* Lite Warning: compute_shortest_path finished but reconstruction failed or didn't reach goal. Final g(start)={self._get_g(self.start_pos)}")
             # Could potentially trigger more processing here if needed, but indicates an issue.

        return path, end_time - start_time, nodes_expanded_this_call, self.visited_for_vis

    def _reconstruct_dstar_path(self):
        """Reconstructs path by following lowest cost neighbors from start."""
        if self._get_g(self.start_pos) == s.INFINITY:
            print("D* Lite: Reconstruction failed, start node is unreachable.")
            return None

        path = [self.start_pos]
        current = self.start_pos
        while current != self.goal_pos:
            best_neighbor = None
            min_cost_plus_g = s.INFINITY
            found_next = False

            # Pass allow_diagonal flag
            for neighbor in get_neighbors(self.grid, current[0], current[1], allow_diagonal=self.allow_diagonal):
                cost = self._cost(current, neighbor) # Get cost from current to neighbor
                if cost == s.INFINITY: continue # Skip blocked moves

                cost_plus_g = cost + self._get_g(neighbor)

                # Tie-breaking can be added here if needed
                if cost_plus_g < min_cost_plus_g:
                    min_cost_plus_g = cost_plus_g
                    best_neighbor = neighbor
                    found_next = True

            if not found_next or best_neighbor is None:
                 print(f"D* Lite path reconstruction failed at {current}, g={self._get_g(current)}")
                 # Print neighbor costs for debugging
                 # for n in get_neighbors(self.grid, current[0], current[1], allow_diagonal=self.allow_diagonal):
                 #    print(f"  Neighbor {n}: cost={self._cost(current, n)}, g={self._get_g(n)}")
                 return path # Return partial path found so far? Or None?

            # Check for direct cycle back to current node (shouldn't happen with proper costs)
            if best_neighbor == current:
                 print(f"D* Lite path reconstruction cycle detected at {current}.")
                 return path # Return partial path

            path.append(best_neighbor)
            current = best_neighbor

            if len(path) > s.GRID_WIDTH_CELLS * s.GRID_HEIGHT_CELLS * 1.5: # Increased safety break
                 print("D* Lite path reconstruction loop detected (long path).")
                 return path # Return potentially incomplete path

        return path

    def notify_change(self, changed_cells):
        """Call this when obstacles are added/removed."""
        # The core logic remains the same - update affected vertices
        for cell_pos in changed_cells:
            self._update_vertex(cell_pos)
            # Pass allow_diagonal flag
            for neighbor in get_neighbors(self.grid, cell_pos[0], cell_pos[1], allow_diagonal=self.allow_diagonal):
                self._update_vertex(neighbor)
        # Update km if agent moved (not implemented here)

    def update_start_pos(self, new_start):
        """If the agent moves, update the start and km."""
        # self.km += self.ACTIVE_HEURISTIC(self.start_pos, new_start) # If agent moved
        self.start_pos = new_start
        # Note: For pure replanning without agent movement, only notify_change is needed