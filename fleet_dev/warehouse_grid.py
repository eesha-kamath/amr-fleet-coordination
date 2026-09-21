"""
warehouse_grid.py

Grid built from REAL shelf geometry: poses from small_warehouse.sdf
(all yaw=0, confirmed via grep) combined with actual collision mesh
bounding boxes (extracted from the .DAE files via pycollada, unit-scaled
0.01 per each file's <asset><unit> tag). No padding-guesses.

Two shelf walls run nearly the full warehouse length:
  - ShelfD/E row at x~4.7 (6 units, confirmed contiguous -- one wall)
  - ShelfF at x~-5.8 (single model, but itself an 18m-long shelf run)
Open central aisle sits between them, roughly -4.79 < x < 4.28.
"""

CELL_SIZE = 0.5
GRID_WIDTH = 100
GRID_HEIGHT = 100
ORIGIN_X = -20.0
ORIGIN_Y = -20.0

# Rectangular obstacle regions in GRID cells: (col_min, row_min, col_max, row_max)
# Derived from real mesh bounding boxes + real world poses (see module docstring)
SHELF_REGIONS = [
    (47, 20, 51, 44),   # ShelfD/E row, world x:[4.28,5.16] y:[-9.95,1.91]
    (25, 20, 31, 57),   # ShelfF, world x:[-6.89,-4.79] y:[-9.95,8.10]
]

def _build_obstacle_set():
    obstacles = set()
    for (c0, r0, c1, r1) in SHELF_REGIONS:
        for c in range(c0, c1 + 1):
            for r in range(r0, r1 + 1):
                obstacles.add((c, r))
    return obstacles

OBSTACLES = _build_obstacle_set()

def world_to_grid(x, y):
    col = int((x - ORIGIN_X) / CELL_SIZE)
    row = int((y - ORIGIN_Y) / CELL_SIZE)
    return (col, row)

def grid_to_world(cell):
    col, row = cell
    x = ORIGIN_X + (col + 0.5) * CELL_SIZE
    y = ORIGIN_Y + (row + 0.5) * CELL_SIZE
    return (x, y)

def in_bounds(cell):
    col, row = cell
    return 0 <= col < GRID_WIDTH and 0 <= row < GRID_HEIGHT

def is_walkable(cell):
    return in_bounds(cell) and cell not in OBSTACLES

def neighbors(cell):
    col, row = cell
    candidates = [(col + 1, row), (col - 1, row), (col, row + 1), (col, row - 1)]
    return [c for c in candidates if is_walkable(c)]
