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

// 67d7842dbbe25473c3c32b93c0da8047785f30d78e8a024de1b57352245f9689

`timescale 1 ns / 1 ps

module main_mul_64s_63ns_126_5_1 (
    clk,
    ce,
    reset,
    din0,
    din1,
    dout
);
    parameter ID = 1;
    parameter NUM_STAGE = 0;
    parameter din0_WIDTH = 14;
    parameter din1_WIDTH = 12;
    parameter dout_WIDTH = 26;

    input clk;
    input ce;
    input reset;

    input [din0_WIDTH - 1 : 0] din0;
    input [din1_WIDTH - 1 : 0] din1;
    output [dout_WIDTH - 1 : 0] dout;

    wire signed [dout_WIDTH - 1 : 0] tmp_product;


    reg signed [dout_WIDTH - 1 : 0] buff0;


    reg [din0_WIDTH - 1 : 0] din0_reg;
    reg [din1_WIDTH - 1 : 0] din1_reg;


    reg signed [dout_WIDTH - 1 : 0] buff1;


    reg signed [dout_WIDTH - 1 : 0] buff2;










    assign tmp_product = $signed(din0_reg) * $signed({1'b0, din1_reg});





    always @(posedge clk) begin
        if (ce) begin
            buff0 <= tmp_product;

            din0_reg <= din0;
            din1_reg <= din1;


            buff1 <= buff0;


            buff2 <= buff1;



        end
    end







    assign dout = buff2;



endmodule
