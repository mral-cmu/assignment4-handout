import argparse
import numpy as np
import matplotlib.pyplot as plt
from cprint import cprint
from time import time

from visualizer import Visualizer2D
from robot import PointRobotState
from mapper_py.data_structures.grid import Grid2D, Cell
from mapper_py.data_structures.sensor import Sensor
from mapper_py.data_structures.observer import Observer
from mapper_py.mapper import Mapper

from exploration import ExplorationPlanner, FrontierPlanner, MIPlanner
from mapper_py.utils import png_to_grid2d


class ExploreTest:
    def __init__(self, max_time: int = 200, vis_on: bool = True):
        self.vis_on = vis_on
        if self.vis_on:
            self.vis = Visualizer2D()

        self.grid_res = 0.2
        self.grid_size = (40, 40)
        self.grid_clamps = (0.001, 0.999)
        self.Tmax = max_time
        self.gt_grid = Grid2D(self.grid_res,
                              self.grid_size[0],
                              self.grid_size[1],
                              self.grid_clamps[0],
                              self.grid_clamps[1])

        # Initialize the map to be explored
        # Initially all probabilities are 0.5
        self.exp_grid = Grid2D(self.grid_res,
                               self.grid_size[0],
                               self.grid_size[1],
                               self.grid_clamps[0],
                               self.grid_clamps[1])

    def reset(self):
        if self.vis_on:
            self.vis.reset()

        self.gt_grid = Grid2D(self.grid_res,
                              self.grid_size[0],
                              self.grid_size[1],
                              self.grid_clamps[0],
                              self.grid_clamps[1])
        self.exp_grid = Grid2D(self.grid_res,
                               self.grid_size[0],
                               self.grid_size[1],
                               self.grid_clamps[0],
                               self.grid_clamps[1])

    def check_collision(self, state):
        if self.gt_grid.freeQ(state.c):
            return False
        else:
            return True

    def run(self,
            scenario='simple-obstacle',
            planner=ExplorationPlanner,
            initial_cell=Cell(10, 10)):
        # Initialize the ground truth map
        png_map_path = f'test_data/{scenario}.png'
        self.gt_grid = png_to_grid2d(self.gt_grid, f'{png_map_path}')

        # Objects developed in Assignment 2 for sensing and mapping
        observer_obj = Observer(self.gt_grid)
        sensor_obj = Sensor(max_range=2.0, num_rays=200)
        mapper_obj = Mapper(self.exp_grid, sensor_obj, observer_obj)

        # Initial robot state
        robot_state = PointRobotState.from_cell(
            initial_cell, self.exp_grid.resolution)

        # Planner for exploration
        planner_obj = planner(self.exp_grid, robot_state)

        if self.vis_on:
            self.vis.initialize_grid(self.exp_grid)
        entropies = np.zeros(self.Tmax)
        for i in range(self.Tmax):
            if self.vis_on:
                self.vis.draw_robot(planner_obj.state.p)
                self.vis.draw_prim_lib(planner_obj.mpl)

            mapper_obj.add_obs(planner_obj.state.p)
            planner_obj.update_map(mapper_obj.grid)
            planner_obj.take_action()

            # The robot must not collide after taking an action
            if self.check_collision(planner_obj.state):
                cprint.err("The robot has collided. Please check your collision avoidance implementation.",
                           interrupt=False)
                raise RuntimeError

            entropies[i] = planner_obj.map.map_entropy()

            if self.vis_on:
                self.vis.update_grid(self.exp_grid)
                # self.vis.save_frame(f'output/{i:06}.png')
                plt.draw()
                plt.pause(0.01)
                self.vis.hl.remove()

        if self.vis_on:
            self.vis.draw_path()
            self.vis.fig.colorbar(self.vis.grid_viz, ax=self.vis.ax)
            plt.draw()

        return entropies


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-planner_type", help="type of exploration planner to run", type=str, default="random")
    parser.add_argument("-env", help="environment",
                        type=str, default="simple-obstacle")
    parser.add_argument(
        "-max_time", help="maximum duration of exploration", type=int, default=200)
    args = parser.parse_args()

    E = np.zeros(args.max_time)

    planner = ExplorationPlanner
    if args.planner_type == 'frontier':
        planner = FrontierPlanner
    if args.planner_type == 'mi':
        planner = MIPlanner

    test_obj = ExploreTest(args.max_time)
    E = test_obj.run(scenario=args.env,
                     planner=planner)

    ent_fig, ent_ax = plt.subplots()
    ent_ax.plot(np.arange(args.max_time), E)

    if test_obj.vis_on:
        plt.show()
