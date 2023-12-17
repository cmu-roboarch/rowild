# RoWild on CPU
This directory includes the implementation of the CPU kernels.

## Content
`include/` includes the implementation of functions that are not specific for
any kernel. Like parsing input arguments, reading JSON files, etc.

`src/` includes the implementation of the kernels.

`build.sh/clean.sh` is used to build/clean all CPU kernels.

`makefile.rules` includes building rules that are shared across all kernels.

## Parameter Tuning
In this released version, the CPU kernels are tuned to offer the highest
performance on our machine, which uses Intel Xeon Gold 5218R processors. To get
the best performance on other CPU platforms, some parameters might need to be
changed.  Particularly, consider playing with `num_threads` in OpenMP
directives to tune the kernels for your machine.
