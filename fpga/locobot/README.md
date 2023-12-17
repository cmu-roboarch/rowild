# LoCoBot Planning
This directory provides the implementation of RRT planning on
[LoCoBot](http://www.locobot.org) in Verilog.

## Description
The program uses the RRT sampling-based planner to find an efficient,
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
