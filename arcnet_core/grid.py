from dataclasses import dataclass


@dataclass
class Grid:
    width: int
    height: int
    cell_size: float

    def validate(self):
        if self.width <= 0 or self.height <= 0:
            raise ValueError("Grid dimensions must be positive")

        if self.cell_size <= 0:
            raise ValueError("Cell size must be positive")

    def to_cell(self, x, y):
        return int(x / self.cell_size), int(y / self.cell_size)

    def to_position(self, x, y):
        return (
            (x + 0.5) * self.cell_size,
            (y + 0.5) * self.cell_size
        )