/*
 * MIT License
 *
 * Copyright (c) 2023 Carnegie Mellon University
 *
 * This file is part of RoWild.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/

// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2023.2 (64-bit)
// Tool Version Limit: 2023.10
// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
// 
// ==============================================================

`timescale 1ns / 1ps

module main_mux_4_2_32_1_1 #(
    parameter ID         = 0,
              NUM_STAGE  = 1,
              din0_WIDTH = 32,
              din1_WIDTH = 32,
              din2_WIDTH = 32,
              din3_WIDTH = 32,
              din4_WIDTH = 32,
              dout_WIDTH = 32
) (
    input  [31 : 0] din0,
    input  [31 : 0] din1,
    input  [31 : 0] din2,
    input  [31 : 0] din3,
    input  [ 1 : 0] din4,
    output [31 : 0] dout
);

    // puts internal signals
    wire [ 1 : 0] sel;
    // level 1 signals
    wire [31 : 0] mux_1_0;
    wire [31 : 0] mux_1_1;
    // level 2 signals
    wire [31 : 0] mux_2_0;

    assign sel = din4;

    // Generate level 1 logic
    assign mux_1_0 = (sel[0] == 0) ? din0 : din1;
    assign mux_1_1 = (sel[0] == 0) ? din2 : din3;

    // Generate level 2 logic
    assign mux_2_0 = (sel[1] == 0) ? mux_1_0 : mux_1_1;

    // output logic
    assign dout = mux_2_0;

endmodule
