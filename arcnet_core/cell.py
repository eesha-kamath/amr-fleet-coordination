from dataclasses import dataclass


@dataclass(frozen=True)
class Cell:
    x: int
    y: int

    def multicast_group(self):
        return f"239.1.{self.y % 256}.{self.x % 256}"

    def neighbors(self):
        cells = []

        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                cells.append(
                    Cell(
                        self.x + dx,
                        self.y + dy
                    )
                )

        return cells

    def is_neighbor(self, other):
        return (
            abs(self.x - other.x) <= 1
            and abs(self.y - other.y) <= 1
        )


def cell_from_position(x, y, cell_size):
    return Cell(
        int(x / cell_size),
        int(y / cell_size)
    )