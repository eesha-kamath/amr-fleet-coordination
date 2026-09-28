from .cell import Cell, cell_from_position


class SpatialManager:

    def __init__(self, cell_size):
        self.cell_size = cell_size
        self.current_cells = {}

    def get_cell(self, x, y):
        return cell_from_position(
            x,
            y,
            self.cell_size
        )

    def set_robot_position(self, robot_id, x, y):
        old = self.current_cells.get(robot_id)
        new = self.get_cell(x, y)

        self.current_cells[robot_id] = new

        if old is None:
            return {
                "event": "CELL_ENTER",
                "robot_id": robot_id,
                "cell": new
            }

        if old == new:
            return None

        return {
            "event": "CELL_CHANGE",
            "robot_id": robot_id,
            "old_cell": old,
            "new_cell": new
        }

    def remove_robot(self, robot_id):
        self.current_cells.pop(robot_id, None)

    def get_cell_for_robot(self, robot_id):
        return self.current_cells.get(robot_id)

    def get_subscriptions(self, robot_id):
        cell = self.current_cells.get(robot_id)

        if not cell:
            return []

        return cell.neighbors()