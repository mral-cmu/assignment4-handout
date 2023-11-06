from cprint import cprint

from mapper_py.utils import png_to_grid2d
from robot import PointRobotState
from mapper_py.data_structures.grid import Grid2D, Cell
from mapper_py.data_structures.sensor import Sensor
from mapper_py.data_structures.observer import Observer
from mapper_py.mapper import Mapper

def score_entropy():
    gt_grid = Grid2D(0.2, 40, 40, 0.001, 0.999)
    gt_grid = png_to_grid2d(gt_grid, 'test_data/simple-obstacle.png')
    ent1 = gt_grid.map_entropy()
    assert(abs(ent1 - 18.253) < 1e-2)

    exp_grid = Grid2D(0.2, 40, 40, 0.001, 0.999)
    ent2 = exp_grid.map_entropy()
    assert(abs(ent2 - 1600) < 1e-2)

    observer_obj = Observer(gt_grid)
    sensor_obj = Sensor(max_range=2.0, num_rays=50)
    mapper_obj = Mapper(exp_grid, sensor_obj, observer_obj)

    robot_state = PointRobotState.from_cell(Cell(10, 10), exp_grid.resolution)
    mapper_obj.add_obs(robot_state.p)
    ent3 = mapper_obj.grid.map_entropy()
    assert(ent3 < ent2)
    mapper_obj.add_obs(robot_state.p)
    ent4 = mapper_obj.grid.map_entropy()
    assert(ent4 < ent2 and ent4 < ent3)

    cprint.ok("[Task 0.2]: Full Credit.")

if __name__ == "__main__":
    score_entropy()