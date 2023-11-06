import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator
from matplotlib.collections import LineCollection

from mapper_py.data_structures.grid import Grid2D, Cell, Point


class Visualizer2D:
    def __init__(self) -> None:
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.ax.xaxis.set_major_locator(MultipleLocator(1.0))
        self.ax.yaxis.set_major_locator(MultipleLocator(1.0))
        self.ax.grid(which='major', axis='both', linestyle='-')
        self.ax.grid(which='minor', axis='both', linestyle='-')
        self.ax.set_xlabel('Cell Space: Cols, Point Space: X (meters)')
        self.ax.set_ylabel('Cell Space: Rows, Point Space: Y (meters)')
        self.ax.set_aspect('equal')

        self.prim_lib_vis = []
        self.prim_lib_objs = LineCollection(
            self.prim_lib_vis, color='black', alpha=0.5)
        self.ax.add_collection(self.prim_lib_objs)

        self.grid_viz = None
        self.grid_min, self.grid_max = None, None
        self.hl = None

        self.rob = plt.Circle((0.0, 0.0), 0.1, color='r')
        self.ax.add_artist(self.rob)

        self.rob_history_plot = None
        self.rob_history = []

    def reset(self):
        self.prim_lib_vis = []
        self.prim_lib_objs = LineCollection(
            self.prim_lib_vis, color='black', alpha=0.5)
        self.ax.add_collection(self.prim_lib_objs)

        self.grid_viz = None
        self.grid_min, self.grid_max = None, None
        self.hl = None

        self.rob = plt.Circle((0.0, 0.0), 0.1, color='r')
        self.ax.add_artist(self.rob)

        self.rob_history = []
        for l in self.rob_history_plot:
            l.remove()

    def draw_robot(self, pos: Point):
        self.rob.center = pos.x, pos.y
        self.rob_history.append(np.array([pos.x, pos.y]))

    def draw_prim_lib(self, prim_lib_gen):
        segs = []
        for l in prim_lib_gen.library:
            segs.append(
                np.array([[l.start.p.x, l.start.p.y], [l.end(3).p.x, l.end(3).p.y]]))
        self.prim_lib_objs.set_segments(segs)

    def draw_path(self):
        self.rob_history = np.array(self.rob_history)
        self.rob_history_plot = self.ax.plot(self.rob_history[:, 0], self.rob_history[:, 1], 'o-', color='r')

    def initialize_grid(self, grid, vmin=0.0, vmax=1.0):
        self.ax.xaxis.set_minor_locator(MultipleLocator(grid.resolution))
        self.ax.yaxis.set_minor_locator(MultipleLocator(grid.resolution))
        self.ax.set_xlim([0.0, grid.resolution * grid.width])
        self.ax.set_ylim([0.0, grid.resolution * grid.height])
        self.grid_min = grid.cell_to_point_row_col(0, 0)
        self.grid_max = grid.cell_to_point_row_col(grid.height, grid.width)

        self.grid_viz = self.ax.imshow(grid.to_numpy(), cmap='Greys', origin='lower', vmin=vmin,
                                        vmax=vmax, extent=(self.grid_min.x, self.grid_max.x,
                                                            self.grid_min.y, self.grid_max.y))

    def update_grid(self, grid):
        self.grid_viz.set_data(grid.to_numpy())
        self.hl = self.fig.colorbar(self.grid_viz, ax=self.ax)

    def save_frame(self, fname):
        self.fig.savefig(fname)