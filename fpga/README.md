# RoWild on FPGA
This directory includes the implementation of the FPGA kernels.

## Content
`src/` includes the implementation of the kernels.

## Methodology
Our typical approach for FPGA kernel development involves initially crafting
HLS-friendly C++ code, which includes strategies like decomposing kernels into
a series of pure functions that smoothly map into combinational logic.
Following this, we utilize Vivado HLS to transform the C++ code into Verilog,
and subsequently, we undertake manual optimizations to enhance performance.

## Parameter Tuning
In this released version, the FPGA kernels have been specifically tailored for
the Xilinx Zynq-7000 family of FPGAs. This means that every aspect, from
parameter selection and loop unrolling to function inlining and dataflow
analysis, has been finely tuned for optimal performance on this FPGA family.
If you intend to use the same code for a different FPGA platform, it's
essential to take it with a huge grain of salt when considering the selected
parameters and configurations.
