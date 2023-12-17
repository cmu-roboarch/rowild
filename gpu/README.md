# RoWild on GPU
This directory includes the implementation of the GPU kernels.

## Content
`include/` includes the implementation of functions that are not specific for
any kernel. Like timing different operations, common CUDA libraries, etc.

`src/` includes the implementation of the kernels.

`build.sh/clean.sh` is used to build/clean all CPU kernels.

`makefile.rules` includes building rules that are shared across all kernels.

## Parameter Tuning
In this released version, the GPU kernels are tuned to offer the highest
performance on our machine, which uses NVIDIA Titan X GPU. To get the best
performance on other GPU platforms, some parameters might need to be changed.
Particularly, consider changing the number of blocks and threads-per-block to
tune the kernels for your machine.
