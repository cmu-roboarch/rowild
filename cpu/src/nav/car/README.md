# Self-Driving Car Navigation
This directory provides an implementation of (x, y, $\theta$) Path Planning.

## Description
The program uses A\* graph search to find an efficient (short), collision-free
path from a start point to a goal point in the 2D environment shown in the
figure below (right). The program resembles a self-driving car operating in a
city.

<p align="center">
  <img
    width="900"
    height="300"
    src="../../../../.images/navcar.png"
  >
</p>

## Code & Data
`input-maps/` includes the map of Boston city, taken from [Moving
AI](https://www.movingai.com). All other [city
maps](https://movingai.com/benchmarks/street/index.html) can be readily used.

`run_all.sh` sweeps different execution parameters (e.g., search heuristic) and
runs an experiment per configuration.
