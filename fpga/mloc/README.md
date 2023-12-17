# Monte Carlo Localization
This directory provides an implementation of Monte Carlo Localization (MCL) in
Verilog.

## Description
The program models a robot operating in a 2D environment, as shown in the
figure below. The animation shows how the algorithm localizes the robot over
time in the modeled environment.

<p float="center">
  <img src="../../../.images/mcl.png" width="500" />
  <img src="../../../.images/mcl.gif" width="450" />
</p>

The state of the robot is defined by (x, y, $\theta$), where (x, y) is its
location and $\theta$ is its orientation. The robot is equipped with an
odometer and a laser rangefinder.
