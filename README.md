# 16-362 Assignment 4: Exploration

Goal: In this assignment, you will implement data structures and algorithms for
exploration using occupancy grid maps.

### Academic Integrity
1. Do not publicly share your solution (using GitHub or otherwise)
2. Collaboration is encouraged but you should write the final code on your own.

### Setup
This repository uses Git LFS. Perform the following in a terminal on your computer.

```bash
git clone git@github.com:mral-cmu/assignment4-handout.git
cd assignment2-handout
git lfs install
git lfs pull
```

Now, create a python virtual environment.
```bash
python3.8 -m .venv venv
```
Source the environment
```bash
source .venv/bin/activate
```
You will need to install the following dependencies.
```bash
pip install cprint numpy matplotlib opencv-python scipy scikit-learn
```

We assume a point-shape robot with a size equal to one cell in the occupancy grid map. The state of
this robot is given by `PointRobotState` in `robot.py`. As in the mapping assignment, we assume that
the robot is equipped with a 360 degree field-of-view range sensor (e.g. 2D LiDAR) of a maximum usable
range `2.0m` and an angular resolution of `50` rays per scan. The robot can explore various environments
provided in the `test_data/` folder.

To set up this functionality correctly, we will rely on the solution to the mapping assignment (Assignment 2).
Please complete the following task as part of the setup.

> [!IMPORTANT]
> **Task 0.1 (0 points)**: Please copy your solutions for all functions from `mapper_py/data_structures/grid.py` in
> Assignment 2 into the file `mapper_py/data_structures/grid.py`. Similarly, copy solutions for
> `mapper_py/data_structures/sensor.py` and `mapper_py/mapper.py` in the respective files within this assignment.
> If you created some helper functions for your Assignment 2 solutions, make sure those are copied too.

> [!WARNING]
> Please do not directly copy and replace the files for the task above. Some helper functions have changed in
> the `mapper_py` folder provided within this assignment compared to the previous assignment.

> [!NOTE]
> If you could not get `traverse` function to work in Assignment 2,
> you are free to use any function from standard libraries. One idea can be to use
> [`skimage.draw.line`](https://scikit-image.org/docs/stable/api/skimage.draw.html#skimage.draw.line) from
> the `scikit-image` library. However, make sure you pass the `test_traversal` test from Assignment 2
> after fixing your `traverse` implementation (either on your own or through standard library functions).

## A Simple Motion Primitive Library
> [!NOTE]
> No implementation required in this section. However, this information is important to read and it is
> crucial to understand when working on implementing the motion planners.

Let us equip our point robot with a simple library of motion primitives that enables the robot to move
in an 8-connected grid. For an illustration, consider this scenario:

![connected](https://github.com/mral-cmu/assignment4-handout/assets/7077226/cc6059ab-01df-43b1-90e6-54cd346e8944)

Here, the robot is in the position shown in red in the occupancy grid. The motion primitives, shown by
red arrow, contain the information about the starting position and the direction in which the robot can move.
In this assignment, we constrain the robot to be able to move at most one cell along these directions.

This motion primitive library has been implemented for through the classes `SimplePrimitive`
and `PrimitiveLibraryGenerator` in `exploration.py`. Please read through the docstrings of these classes
to understand the provided functionality.

## Take Random Actions and Avoid Collisions
Using the motion primitive library, let us create a simple motion planning setup for exploration.
Take a look at the class `ExplorationPlanner` in `exploration.py` and read through the docstrings.

The first task is to ensure safety by avoiding actions that may result in collision with the environment.

> [!IMPORTANT]
> **Task 2.1 (5 points)** Implement `is_feasible` function in `ExplorationPlanner`.

> [!NOTE]
> To check your implementation for Task 2.1, you can use `collision_test.py`. If the implementation is correct,
> you will see the output:
> ```
> Collision tests passed.
> ```

Now that we have the capability to avoid collisions, let us enable the robot to take random actions while
avoiding collisions. Intuitively, this is the most "naive" way in which the robot can explore its surroundings.

> [!IMPORTANT]
> **Task 2.2 (5 points)** Implement `selection_policy` function in `ExplorationPlanner`.

> [!NOTE]
> It is difficult to quantitatively test this function since the selection policy is random. We will describe
> how this problem is graded later in the Frontier-based Exploration section.

You should see a confused robot trying to explore

https://github.com/mral-cmu/assignment4-handout/assets/7077226/2f327114-a62f-482c-b587-b78b0181fd32

and the final trajectory may look like

<img src="./assets/random-traj.png" width="400" height="400"/>

You will also see a plot for entropy of the explored map over time.

Clearly this is not a good exploration strategy. However, we now have infrastructure to test out
exploration planning algorithms. Let us start with implementing the frontier-based exploration method.

## Frontier-based Exploration
Take a look at the class `ClosestPointFrontierPlanner`, which is derived from the `ExplorationPlanner` class.
We will override the method `selection_policy` to implement a new one.

> [!IMPORTANT]
> **Task 3.1 (20 points)**

https://github.com/mral-cmu/assignment4-handout/assets/7077226/700a7f5f-dcd1-4937-9d71-b615e79e3153

<img src="./assets/closest-point-frontier-traj.png" width="400" height="400"/>

## Information-Theoretic Exploration

## Grading with AutoLab
Assuming you are in this assignment directory, run this command after completing your solutions:

```
tar -C . -cvf handin.tar mapper_py explore_py
```

Submit `handin.tar` on Autolab.

Autolab will run tests on each function you implement and you will
receive a score out of 100. You may upload as many times as you like.
Note that we may regrade submissions after the deadline passes.

## References

## Author(s)
Kshitij Goel, Wennie Tabib
