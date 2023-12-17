# Dynamic Movement Primitives
This directory provides the implementation of Dynamic Movement Primitives (DMP)
algorithm in Verilog.

## Description
The program generates a smooth trajectory based on the path computed by the
robot's path planner. It uses Gaussian bias functions and shape parameters to
define the overall trajectory shape. These parameters are often acquired
through imitation learning and linear regression, typically through a single
demonstration.

The figure below shows a reference trajectory depicted in orange. The black
lines in the figure show the trajectory (left) and velocity (right) generated
by DMP.

<p align="center">
  <img
    width="800"
    height="250"
    src="../../../.images/dmp.png"
  >
</p>
