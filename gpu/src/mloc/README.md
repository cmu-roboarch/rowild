# Monte Carlo Localization
This directory provides a CUDA implementation of Monte Carlo Localization
(MCL). The program heavily utilizes
[Thrust](https://docs.nvidia.com/cuda/thrust/index.html) for
GPU-parallelization of different operations.

## Description
The program models a wheeled robot operating in a 2D environment, as shown in
the figure below. The animation shows how the algorithm localizes the robot
over time in the modeled environment.

<p float="center">
  <img src="../../../.images/mcl.png" width="500" />
  <img src="../../../.images/mcl.gif" width="450" />
</p>

The state of the robot is defined by (x, y, $\theta$), where (x, y) is its
location and $\theta$ is its orientation. The robot is equipped with an
odometer and a laser rangefinder.

## Code & Data
`input-maps/` includes the map of the environment. We have included the map of
CMU's Wean Hall (courtesy of Michael Montemerlo). The numbers in the map file
represent the "occupation probability." E.g., cells whose value is zero are
definitely free, and the cells whose value is one are definitely occupied.
Cells with a probability of -1 are unknown: the robot has no clue about their
occupancy status. Observing the provided map's template and norms, any other
maps can be smoothly used.

`input-measurements/` includes sensory data. Every line includes 3 + 180
numbers. The first three are odometry measurements, and the rest are laser
readings in cm. The 180 readings span 180 degrees starting from the right and
going left. We have included the measurements of only one scenario. You can
download further logs from
[here](https://cmu.box.com/s/fshj3cjsn0qbutn3osk9l4z83w63w7co).

`kidnapped-log/` includes sensory data for a kidnapped robot scenario.

`run_all.sh` sweeps different execution parameters (e.g., the number of
particles) and runs an experiment per configuration.
