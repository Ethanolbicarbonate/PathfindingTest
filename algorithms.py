# ----------- algorithms.py -----------
import heapq
import time
import math
import settings as s
from grid_utils import get_neighbors, reconstruct_path

# --- Heuristic Function ---
def heuristic(pos1, pos2):
    """Manhattan distance heuristic."""
    r1, c1 = pos1
    r2, c2 = pos2
    return abs(r1 - r2) + abs(c1 - c2)

# --- Dijkstra ---
def dijkstra(grid, start_pos, goal_pos, visualizer=None):
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
        nodes_expanded += 1
        visited_for_vis.add(current_pos)

        if current_pos == goal_pos:
            path = reconstruct_path(came_from, start_pos, goal_pos)
            break

        if visualizer: # Update visualization during search
            visualizer.update_search_view(visited=visited_for_vis)
            # Allow Pygame events to be processed to keep window responsive
            visualizer.handle_events()

        for neighbor_pos in get_neighbors(grid, current_pos[0], current_pos[1]):
            # Assume cost of 1 for adjacent non-diagonal move
            new_cost = current_cost + 1
            if neighbor_pos not in cost_so_far or new_cost < cost_so_far[neighbor_pos]:
                cost_so_far[neighbor_pos] = new_cost
                priority = new_cost # Dijkstra's priority is just the cost
                heapq.heappush(priority_queue, (priority, neighbor_pos))
                came_from[neighbor_pos] = current_pos

    end_time = time.perf_counter()
    return path, end_time - start_time, nodes_expanded, visited_for_vis

# --- A* ---
def a_star(grid, start_pos, goal_pos, visualizer=None):
    """Finds the shortest path using A* algorithm."""
    start_time = time.perf_counter()
    nodes_expanded = 0

    priority_queue = [(0 + heuristic(start_pos, goal_pos), 0, start_pos)] # (f_cost, g_cost, position)
    cost_so_far = {start_pos: 0} # g_cost
    came_from = {start_pos: None}
    visited_for_vis = set() # For visualization

    path = None

    while priority_queue:
        f_cost, current_g_cost, current_pos = heapq.heappop(priority_queue)
        nodes_expanded += 1

        # Optimization: If we already found a cheaper path to here, skip
        if current_g_cost > cost_so_far.get(current_pos, s.INFINITY):
             continue

        visited_for_vis.add(current_pos)

        if current_pos == goal_pos:
            path = reconstruct_path(came_from, start_pos, goal_pos)
            break

        if visualizer: # Update visualization during search
            visualizer.update_search_view(visited=visited_for_vis)
            visualizer.handle_events()

        for neighbor_pos in get_neighbors(grid, current_pos[0], current_pos[1]):
            # Assume cost of 1 for adjacent non-diagonal move
            new_g_cost = current_g_cost + 1
            if neighbor_pos not in cost_so_far or new_g_cost < cost_so_far[neighbor_pos]:
                cost_so_far[neighbor_pos] = new_g_cost
                h_cost = heuristic(neighbor_pos, goal_pos)
                priority = new_g_cost + h_cost # f_cost = g_cost + h_cost
                # Store g_cost in tuple for the optimization check above
                heapq.heappush(priority_queue, (priority, new_g_cost, neighbor_pos))
                came_from[neighbor_pos] = neighbor_pos # Typo fixed: should be current_pos
                came_from[neighbor_pos] = current_pos # Corrected line

    end_time = time.perf_counter()
    return path, end_time - start_time, nodes_expanded, visited_for_vis


# --- D* Lite ---
class DStarLite:
    def __init__(self, grid, start_pos, goal_pos, visualizer=None):
        self.grid = grid # Reference to the grid
        self.start_pos = start_pos
        self.goal_pos = goal_pos
        self.visualizer = visualizer

        self.g_values = {}
        self.rhs_values = {}
        self.priority_queue = [] # Min-heap (priority queue U)
        self.km = 0 # Key modifier for agent movement (simplified for static changes)
        self.nodes_expanded_total = 0 # Track nodes across calls
        self.visited_for_vis = set() # For visualization

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
        # Simplified key for non-moving agent reacting to static changes
        # The full key includes h(start, pos) + km
        return (min_g_rhs + heuristic(self.start_pos, pos), min_g_rhs)
        # Full key if agent moves:
        # return (min_g_rhs + heuristic(self.start_pos, pos) + self.km, min_g_rhs)

    def _cost(self, u, v):
        """Cost of moving from u to v."""
        if self.grid[v[0]][v[1]] == s.OBSTACLE or self.grid[u[0]][u[1]] == s.OBSTACLE:
            return s.INFINITY
        # Simple adjacent cost = 1
        return 1

    def _initialize(self):
        self.g_values = {}
        self.rhs_values = {}
        self.priority_queue = []
        self.km = 0
        # All g/rhs are implicitly infinity
        self._set_rhs(self.goal_pos, 0)
        heapq.heappush(self.priority_queue, (self._calculate_key(self.goal_pos), self.goal_pos))

    def _update_vertex(self, pos):
        if pos != self.goal_pos:
            min_rhs = s.INFINITY
            for neighbor in get_neighbors(self.grid, pos[0], pos[1]):
                 cost = self._cost(pos, neighbor) # Cost from pos to neighbor
                 g_neighbor = self._get_g(neighbor)
                 min_rhs = min(min_rhs, cost + g_neighbor)
            self._set_rhs(pos, min_rhs)

        # Remove node from queue if it's already there (heapq doesn't support efficient removal/update)
        # This is inefficient but simple. A better implementation uses a dict or flags lazy deletion.
        self.priority_queue = [(key, node) for key, node in self.priority_queue if node != pos]
        heapq.heapify(self.priority_queue) # Re-heapify after potential removals

        if self._get_g(pos) != self._get_rhs(pos):
            heapq.heappush(self.priority_queue, (self._calculate_key(pos), pos))

    def compute_shortest_path(self):
        """Computes or recomputes the path after initialization or changes."""
        start_time = time.perf_counter()
        nodes_expanded_this_call = 0
        self.visited_for_vis.clear() # Clear visualization for this call

        # Loop while the start node's key is less than the top key in PQ
        # or while the start node is inconsistent (g != rhs)
        while (not self.priority_queue or
               self.priority_queue[0][0] < self._calculate_key(self.start_pos) or
               self._get_rhs(self.start_pos) != self._get_g(self.start_pos)):

            if not self.priority_queue: # Path blocked
                 print("D* Lite: Path appears blocked.")
                 break

            key_old, u = heapq.heappop(self.priority_queue)
            nodes_expanded_this_call += 1
            self.visited_for_vis.add(u)

            key_new = self._calculate_key(u)
            if key_old < key_new: # Stale entry in PQ, push back with updated key
                heapq.heappush(self.priority_queue, (key_new, u))
            elif self._get_g(u) > self._get_rhs(u): # Overconsistent -> make consistent
                self._set_g(u, self._get_rhs(u))
                for neighbor in get_neighbors(self.grid, u[0], u[1]):
                    self._update_vertex(neighbor)
            else: # Underconsistent -> make consistent
                 self._set_g(u, s.INFINITY)
                 # Update itself first
                 self._update_vertex(u)
                 # Then update neighbors
                 for neighbor in get_neighbors(self.grid, u[0], u[1]):
                     self._update_vertex(neighbor)

            if self.visualizer: # Update visualization during search
                self.visualizer.update_search_view(visited=self.visited_for_vis)
                self.visualizer.handle_events()

        end_time = time.perf_counter()
        self.nodes_expanded_total += nodes_expanded_this_call

        # Reconstruct path after computation
        path = self._reconstruct_dstar_path()

        # Return path, time *for this specific call*, nodes *for this specific call*
        return path, end_time - start_time, nodes_expanded_this_call, self.visited_for_vis

    def _reconstruct_dstar_path(self):
        """Reconstructs path by following lowest cost neighbors from start."""
        if self._get_g(self.start_pos) == s.INFINITY:
            return None # No path found

        path = [self.start_pos]
        current = self.start_pos
        while current != self.goal_pos:
            best_neighbor = None
            min_cost_heuristic = s.INFINITY

            found_next = False
            for neighbor in get_neighbors(self.grid, current[0], current[1]):
                cost = self._cost(current, neighbor) + self._get_g(neighbor)
                if cost < min_cost_heuristic:
                    min_cost_heuristic = cost
                    best_neighbor = neighbor
                    found_next = True

            if not found_next or best_neighbor is None:
                 print("D* Lite path reconstruction failed")
                 return None # Should not happen if g(start) is finite

            path.append(best_neighbor)
            current = best_neighbor
            if len(path) > s.GRID_WIDTH_CELLS * s.GRID_HEIGHT_CELLS: # Safety break
                 print("D* Lite path reconstruction loop detected")
                 return None
        return path

    def notify_change(self, changed_cells):
        """Call this when obstacles are added/removed."""
        # This simplified version assumes costs change from 1 to INFINITY or vice-versa
        # A full implementation might handle varying edge costs.
        for cell_pos in changed_cells:
             # We need to update the vertices *around* the change
             # If an obstacle is ADDED at cell_pos, the cost TO cell_pos from neighbors increases.
             # If an obstacle is REMOVED at cell_pos, the cost TO cell_pos from neighbors may decrease.
             # D* Lite updates based on rhs calculation, which looks at cost(neighbor, changed_cell) + g(changed_cell)
             # The key is to update vertices whose RHS value might change.
             self._update_vertex(cell_pos) # Update the changed cell itself
             for neighbor in get_neighbors(self.grid, cell_pos[0], cell_pos[1]):
                 # Update neighbors as their RHS might depend on the changed cell's G value
                 self._update_vertex(neighbor)
        # self.km += heuristic(self.last_pos, self.start_pos) # If agent moved
        # self.last_pos = self.start_pos # Update last known position if agent moved

    def update_start_pos(self, new_start):
        """If the agent moves, update the start and km."""
        # Simple update for static replan - just set new start
        # For moving agent:
        # self.km += heuristic(self.start_pos, new_start)
        self.start_pos = new_start