# POM
This directory provides the implementation of Probabilistic Occupancy Maps
(POM).

## Description
POM creates a spatial representation of an environment based on uncertain
sensor data. The environment is discretized into grid cells, where each cell
represents a probability indicating the likelihood that it is occupied by an
obstacle.

### Key Features
**Initial Belief**:
Every grid cell starts with an initial probability, typically representing
complete uncertainty (e.g., 50% occupied).

**Sensor Integration**:
As sensors (like Lidar or sonar) provide distance measurements to obstacles,
the probabilities in the grid cells are updated. If a sensor indicates a clear
path to a certain distance, cells along that path increase their probability of
being free. Conversely, the cell where an obstacle is detected increases its
probability of being occupied.

**Bayesian Update**:
The core of the algorithm leverages Bayes' theorem to continuously update
the occupancy probability of each cell as new sensor readings are integrated.
This allows the map to refine and correct itself over time, integrating
multiple measurements.

**Handling Uncertainty**:
Since sensors are prone to noise and errors, the probabilistic framework of
this algorithm is particularly valuable. It doesn't assert an absolute occupied
or free state but instead provides a likelihood, making it robust against
occasional sensor inaccuracies.

The resulting occupancy grid map provides robots with a probabilistic
understanding of their environment, facilitating safer navigation and
decision-making.

## Code & Data
`input-measurements/` includes a script for generating random measurements, and
an output of it.

`run_all.sh` sweeps different execution parameters (e.g., occupancy
probability) and runs an experiment per configuration.
