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

module main_mul_42ns_33ns_75_2_1 (
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


    reg signed  [dout_WIDTH - 1 : 0] buff0;









    assign tmp_product = $signed({1'b0, din0}) * $signed({1'b0, din1});









    always @(posedge clk) begin
        if (ce) begin
            buff0 <= tmp_product;





        end
    end




    assign dout = buff0;






endmodule
