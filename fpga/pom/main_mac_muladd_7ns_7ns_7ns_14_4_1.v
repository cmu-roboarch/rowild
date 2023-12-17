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
`timescale 1 ns / 1 ps

(* use_dsp = "yes" *) module main_mac_muladd_7ns_7ns_7ns_14_4_1_DSP48_0 (
    input clk,
    input rst,
    input ce,
    input [7 - 1:0] in0,
    input [7 - 1:0] in1,
    input [7 - 1:0] in2,
    output [14 - 1:0] dout
);

    wire signed [25 - 1:0] a;
    wire signed [18 - 1:0] b;
    wire signed [48 - 1:0] c;
    wire signed [43 - 1:0] m;
    wire signed [48 - 1:0] p;
    reg signed  [43 - 1:0] m_reg;
    reg signed  [25 - 1:0] a_reg;
    reg signed  [18 - 1:0] b_reg;
    reg signed  [48 - 1:0] p_reg;

    assign a = $unsigned(in0);
    assign b = $unsigned(in1);
    assign c = $unsigned(in2);

    assign m = a_reg * b_reg;
    assign p = m_reg + c;

    always @(posedge clk) begin
        if (ce) begin
            m_reg <= m;
            a_reg <= a;
            b_reg <= b;
            p_reg <= p;
        end
    end

    assign dout = p_reg;

endmodule
`timescale 1 ns / 1 ps
module main_mac_muladd_7ns_7ns_7ns_14_4_1 (
    clk,
    reset,
    ce,
    din0,
    din1,
    din2,
    dout
);

    parameter ID = 32'd1;
    parameter NUM_STAGE = 32'd1;
    parameter din0_WIDTH = 32'd1;
    parameter din1_WIDTH = 32'd1;
    parameter din2_WIDTH = 32'd1;
    parameter dout_WIDTH = 32'd1;
    input clk;
    input reset;
    input ce;
    input [din0_WIDTH - 1:0] din0;
    input [din1_WIDTH - 1:0] din1;
    input [din2_WIDTH - 1:0] din2;
    output [dout_WIDTH - 1:0] dout;



    main_mac_muladd_7ns_7ns_7ns_14_4_1_DSP48_0 main_mac_muladd_7ns_7ns_7ns_14_4_1_DSP48_0_U (
        .clk (clk),
        .rst (reset),
        .ce  (ce),
        .in0 (din0),
        .in1 (din1),
        .in2 (din2),
        .dout(dout)
    );

endmodule

