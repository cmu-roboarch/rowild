# Model Predictive Control
This directory contains CUDA code that demonstrates a Model Predictive Control
(MPC) algorithm for a drone navigating in 3D space. MPC is a control method
that utilizes a system model to forecast future states and control the system
to reduce a defined cost over a prediction horizon.

<p align="center">
  <img
    width="500"
    height="350"
    src="../../../.images/mpc-drone.png"
  >
</p>

## Key Components

### 1. Constants and Parameters
- **`gGravity`**: Denotes the acceleration due to gravity.
- **`gMaxThrust`** and **`gMaxVel`**: Set the bounds for maximum thrust and velocity.
- **`goal`**: A fixed array indicating the target 3D position for the drone.

### 2. Drone Dynamics
- **`droneDynamics` Function**: Portrays how the drone's state changes over
  time, influenced by the present state and control actions. The state
encompasses its position (`X, Y, Z`) and velocities (`Vx, Vy, Vz`).

### 3. Cost Function
- **`computeCost` Function**: Calculates the Euclidean distance separating the
  drone's present position from the goal. The intention is to decrease this
value, guiding the drone towards its target.

### 4. MPC Kernel
- **`mpcKernel` Function**: The central function where MPC calculations are
  performed. For every thread (symbolizing a potential control sequence), it
predicts the drone's trajectory over a defined prediction horizon employing the
drone dynamics. It subsequently computes the total cost over this horizon.
- A rudimentary gradient descent is executed to adjust the control actions with
  the aim of diminishing the overall cost across the prediction horizon. The
initial control of the best sequence is then stored as the desired control
instruction.

### 5. Main Execution
- Initializes states and control instructions.
- Allocates GPU memory and conducts data transfers between the host and the device.
- Invokes the `mpcKernel` function.
- Retrieves the determined controls from the GPU and displays the first control
  as a demonstration.
