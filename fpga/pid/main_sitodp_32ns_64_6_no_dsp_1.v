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
// Generated by Vitis HLS v2023.2
// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
// ==============================================================

`timescale 1ns / 1ps

module main_sitodp_32ns_64_6_no_dsp_1 #(
    parameter ID         = 1,
              NUM_STAGE  = 3,
              din0_WIDTH = 32,
              dout_WIDTH = 32
) (
    input  wire                  clk,
    input  wire                  reset,
    input  wire                  ce,
    input  wire [din0_WIDTH-1:0] din0,
    output wire [dout_WIDTH-1:0] dout
);
    //------------------------Local signal-------------------
    wire                  aclk;
    wire                  aclken;
    wire                  a_tvalid;
    wire [din0_WIDTH-1:0] a_tdata;
    wire                  r_tvalid;
    wire [dout_WIDTH-1:0] r_tdata;
    reg  [din0_WIDTH-1:0] din0_buf1;
    reg                   ce_r;
    wire [dout_WIDTH-1:0] dout_i;
    reg  [dout_WIDTH-1:0] dout_r;
    //------------------------Instantiation------------------
    main_sitodp_32ns_64_6_no_dsp_1_ip main_sitodp_32ns_64_6_no_dsp_1_ip_u (
        .aclk                (aclk),
        .aclken              (aclken),
        .s_axis_a_tvalid     (a_tvalid),
        .s_axis_a_tdata      (a_tdata),
        .m_axis_result_tvalid(r_tvalid),
        .m_axis_result_tdata (r_tdata)
    );
    //------------------------Body---------------------------
    assign aclk     = clk;
    assign aclken   = ce_r;
    assign a_tvalid = 1'b1;
    assign a_tdata  = din0_buf1;
    assign dout_i   = r_tdata;

    always @(posedge clk) begin
        if (ce) begin
            din0_buf1 <= din0;
        end
    end

    always @(posedge clk) begin
        ce_r <= ce;
    end

    always @(posedge clk) begin
        if (ce_r) begin
            dout_r <= dout_i;
        end
    end

    assign dout = ce_r ? dout_i : dout_r;
endmodule
