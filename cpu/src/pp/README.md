# Pure Pursuit
This directory provides the implementation of the Pure Pursuit (PP) algorithm.

## Description
The Pure Pursuit algorithm is a path-tracking technique used in robotics.
Given a set of path points, the algorithm selects a "look-ahead" point on the
path based on the current position of the vehicle and computes the necessary
steering command to drive the vehicle toward that look-ahead point.

<p align="center">
  <img
    width="600"
    height="150"
    src="../../../.images/pp.png"
  >
</p>

## Code & Data
`input-paths/` includes files containing pairs of (x, y) that represent the
robot's path.

`run_all.sh` sweeps different execution parameters (e.g., lookahead distance)
and runs an experiment per configuration.
