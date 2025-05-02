# ----------- settings.py -----------
import pygame

# --- Colors ---
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)       # Obstacle
GREEN = (0, 255, 0)     # Goal
BLUE = (0, 0, 255)      # Start
RED = (255, 0, 0)       # Path Line
LIGHT_GRAY = (211, 211, 211) # Visited / Closed Set
DARK_GRAY = (169, 169, 169)  # Grid Lines
YELLOW = (255, 255, 0)  # Frontier / Open Set (Optional visual)
ORANGE = (255, 165, 0) # Agent Position (Optional visual)
PASTEL_GRAY = (220, 220, 235)
PASTEL_OUTLINE = (180, 180, 200)
PASTEL_RED = (255, 140, 140)
PASTEL_GREEN = (140, 230, 140)
PASTEL_BLUE = (140, 170, 255)
PASTEL_BLACK = (60, 60, 60)

# --- Grid Dimensions & Cell Size ---
GRID_WIDTH_CELLS = 30
GRID_HEIGHT_CELLS = 20
CELL_SIZE = 25 # Pixels
WINDOW_WIDTH = GRID_WIDTH_CELLS * CELL_SIZE
WINDOW_HEIGHT = GRID_HEIGHT_CELLS * CELL_SIZE

# --- Cell States (Internal Representation) ---
EMPTY = 0
OBSTACLE = 1
# Note: Start/Goal are stored by coordinate, not changing cell value usually

# --- Algorithm Constants ---
INFINITY = float('inf')