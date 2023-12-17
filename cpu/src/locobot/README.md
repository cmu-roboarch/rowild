# LoCoBot Planning
This directory provides the implementation of sampling-based planning on
[LoCoBot](http://www.locobot.org).

## Description
The program uses sampling-based planners (e.g., PRM, RRT) to find an efficient,
collision-free path from a start point to a goal point in the environment shown
below. The visualization of the setup is done using
[CoppeliaSim](https://www.coppeliarobotics.com).

<p align="center">
  <img
    width="400"
    height="300"
    src="../../../.images/locobot.gif"
  >
</p>

The output of the program is a set of angles in N-dimensional state space that
guides the robot's end-effector to reach the goal.

## Code & Data
The planning algorithm is taken as an input. For example, running the program
with `--planner RRT` will search for the path using RRT algorithm.

`input-maps/` includes the map of the environments.

`run_all.sh` sweeps different execution parameters (e.g., number of samples)
and runs an experiment per configuration.
