from visualizer import Visualizer2D
from robot import PointRobotState
from exploration import ExplorationPlanner, SimplePrimitive
from mapper_py.utils import png_to_grid2d
from mapper_py.data_structures.grid import Grid2D, Cell
import matplotlib.pyplot as plt
from cprint import cprint

grid = Grid2D(0.2, 40, 40, 0.001, 0.999)
grid = png_to_grid2d(grid, 'test_data/simple-obstacle.png')

initial_cell=Cell(25, 15)
robot_state = PointRobotState.from_cell(initial_cell, grid.resolution)

planner_obj = ExplorationPlanner(grid, robot_state)

result = planner_obj.is_feasible(SimplePrimitive(robot_state, Cell(0, -1)))
assert(result == True)

result = planner_obj.is_feasible(SimplePrimitive(robot_state, Cell(1, -1)))
assert(result == False)

result = planner_obj.is_feasible(SimplePrimitive(robot_state, Cell(1, 0)))
assert(result == False)

result = planner_obj.is_feasible(SimplePrimitive(robot_state, Cell(-1, -1)))
assert(result == True)

# If you want visualization during debugging, uncomment the following:
# vis = Visualizer2D()
# vis.initialize_grid(grid)
# vis.draw_robot(robot_state.p)
# plt.show()

cprint.ok("[Task 2.1]: Full Credit.")