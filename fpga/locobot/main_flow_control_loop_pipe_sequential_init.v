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

module main_flow_control_loop_pipe_sequential_init (
    ap_clk,
    ap_rst,
    ap_start,
    ap_ready,
    ap_done,
    ap_start_int,
    ap_ready_int,
    ap_done_int,
    ap_continue_int,
    ap_loop_init,
    ap_loop_exit_ready,
    ap_loop_exit_done
);

    input ap_clk;
    input ap_rst;

    //Block level handshake with outside loop
    input ap_start;
    output ap_ready;
    output ap_done;

    //Block level handshake with loop body
    output ap_start_int;
    input ap_ready_int;
    input ap_done_int;
    output ap_continue_int;

    //Init live in variables
    output ap_loop_init;
    wire ap_loop_init;
    reg  ap_loop_init_int;
    reg  ap_done;
    reg  ap_done_cache;

    //Exit signal from loop body
    input ap_loop_exit_ready;
    input ap_loop_exit_done;

    // power-on initialization
    initial begin
        #0 ap_loop_init_int = 1'b1;
        #0 ap_done_cache = 1'b0;
    end

    assign ap_start_int = ap_start;

    assign ap_continue_int = 1'b1;

    assign ap_ready = ap_loop_exit_ready;

    //ap_loop_init is valid for the first II
    //of the first loop run so as to enable
    //the init block ops which are pushed into
    //the first state of the pipeline region
    always @(posedge ap_clk) begin
        if (ap_rst == 1'b1) begin
            ap_loop_init_int <= 1'b1;
        end else if (ap_loop_exit_done == 1'b1) begin
            ap_loop_init_int <= 1'b1;
        end else if (ap_ready_int == 1'b1) begin
            ap_loop_init_int <= 1'b0;
        end
    end

    assign ap_loop_init = ap_loop_init_int & ap_start;

    // if no ap_continue port and current module is not top module, 
    // ap_done handshakes with ap_start. Internally, flow control sends out 
    // ap_conintue_int = 1'b1 so the ap_done_int is asserted high for 1 clock cycle.
    // ap_done_cache is used to record ap_done_int, and de-assert if ap_start_int
    // is asserted, so DUT can start the next run
    always @(posedge ap_clk) begin
        if (ap_rst == 1'b1) begin
            ap_done_cache <= 1'b0;
        end else if (ap_done_int == 1'b1) begin
            ap_done_cache <= 1'b1;
        end else if (ap_start_int == 1'b1) begin
            ap_done_cache <= 1'b0;
        end
    end

    // if no ap_continue port and current module is not top module, ap_done handshakes with ap_start
    always @(*) begin
        if ((ap_done_int == 1'b1) || ((ap_done_cache == 1'b1) && (ap_start_int == 1'b0))) begin
            ap_done = 1'b1;
        end else begin
            ap_done = 1'b0;
        end
    end

endmodule

