# ORB-SLAM
This directory provides the implementation of ORB-SLAM.

## Description
ORB-SLAM (Oriented FAST and Rotated BRIEF SLAM) is a state-of-the-art visual
SLAM system that combines the ORB feature detector and descriptor with a robust
backend optimization to perform SLAM in real-time with a single camera.

For more information, please refer to ORB-SLAM project's
[webpage](https://webdiis.unizar.es/~raulmur/orbslam/).

## Code & Data
`download.sh` script creates a directory named `input-data/`, including images
used as the input of the ORB algorithm.

`run_all.sh` runs the ORB algorithm on different input images located in
`input-data/`.
