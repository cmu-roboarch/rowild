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
`timescale 1 ns / 1 ps
module main_env_axesObs_RAM_AUTO_1R1W (

    address0,
    ce0,
    d0,
    we0,
    q0,

    address1,
    ce1,
    d1,
    we1,
    q1,

    reset,
    clk
);

    parameter DataWidth = 64;
    parameter AddressWidth = 7;
    parameter AddressRange = 72;

    input [AddressWidth-1:0] address0;
    input ce0;
    input [DataWidth-1:0] d0;
    input we0;
    output reg [DataWidth-1:0] q0;

    input [AddressWidth-1:0] address1;
    input ce1;
    input [DataWidth-1:0] d1;
    input we1;
    output reg [DataWidth-1:0] q1;

    input reset;
    input clk;

    (* ram_style = "auto"  *) reg [DataWidth-1:0] ram[0:AddressRange-1];








    //read first
    always @(posedge clk) begin
        if (ce0) begin
            if (we0) ram[address0] <= d0;
            q0 <= ram[address0];

        end
    end







    //read first
    always @(posedge clk) begin
        if (ce1) begin
            if (we1) ram[address1] <= d1;
            q1 <= ram[address1];

        end
    end



endmodule

