import numpy as np
from math import floor

from mapper_py.data_structures.grid import Point, Cell

class PointRobotState:
    """A simple point size robot for which the orientation doesn't matter.
    """
    def __init__(self, pos: Point, cell: Cell, resolution: float) -> None:
        self.res = resolution
        self.c = cell
        self.p = pos

    @classmethod
    def from_cell(cls, cell: Cell, resolution: float):
        p = Point()
        PointRobotState.pos_from_cell(cell, resolution, p)
        return cls(p, cell, resolution)

    @classmethod
    def from_point(cls, point : Point, resolution: float):
        cell = Cell()
        PointRobotState.cell_from_pos(point, resolution, cell)
        return cls(point, cell, resolution)

    def update(self, cell: Cell) -> None:
        self.c = cell
        PointRobotState.pos_from_cell(self.c, self.res, self.p)

    def update_by(self, cell: Cell) -> None:
        self.c += cell
        PointRobotState.pos_from_cell(self.c, self.res, self.p)

    @staticmethod
    def cell_from_pos(pos: Point, res: float, cell: Cell) -> None:
        cell.row = floor(pos.y / res)
        cell.col = floor(pos.x / res)

    @staticmethod
    def pos_from_cell(cell: Cell, res: float, pos: Point) -> None:
        pos.x = (float(cell.col) + 0.5) * res
        pos.y = (float(cell.row) + 0.5) * res

    def to_numpy(self):
        return np.array([self.p.x, self.p.y])