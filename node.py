# ----------- node.py (Optional but can clarify) -----------
class Node:
    def __init__(self, row, col):
        self.row = row
        self.col = col
        # Add other node-specific attributes if needed later

    def get_pos(self):
        return (self.row, self.col)

    # Make nodes hashable for use in sets/dictionaries
    def __eq__(self, other):
        return isinstance(other, Node) and self.row == other.row and self.col == other.col

    def __hash__(self):
        return hash((self.row, self.col))

    def __lt__(self, other):
        # Basic comparison for priority queue if needed, though usually tuples handle priority
        return False # Or implement based on some criteria if nodes directly go in PQ