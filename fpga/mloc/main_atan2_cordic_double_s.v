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

module main_atan2_cordic_double_s (
    ap_clk,
    ap_rst,
    ap_start,
    ap_done,
    ap_idle,
    ap_ready,
    y_in,
    x_in,
    ap_return,
    grp_fu_323_p_din0,
    grp_fu_323_p_din1,
    grp_fu_323_p_opcode,
    grp_fu_323_p_dout0,
    grp_fu_323_p_ce,
    grp_fu_361_p_din0,
    grp_fu_361_p_din1,
    grp_fu_361_p_opcode,
    grp_fu_361_p_dout0,
    grp_fu_361_p_ce
);

    parameter ap_ST_fsm_state1 = 30'd1;
    parameter ap_ST_fsm_state2 = 30'd2;
    parameter ap_ST_fsm_state3 = 30'd4;
    parameter ap_ST_fsm_state4 = 30'd8;
    parameter ap_ST_fsm_state5 = 30'd16;
    parameter ap_ST_fsm_state6 = 30'd32;
    parameter ap_ST_fsm_state7 = 30'd64;
    parameter ap_ST_fsm_state8 = 30'd128;
    parameter ap_ST_fsm_state9 = 30'd256;
    parameter ap_ST_fsm_state10 = 30'd512;
    parameter ap_ST_fsm_state11 = 30'd1024;
    parameter ap_ST_fsm_state12 = 30'd2048;
    parameter ap_ST_fsm_state13 = 30'd4096;
    parameter ap_ST_fsm_state14 = 30'd8192;
    parameter ap_ST_fsm_state15 = 30'd16384;
    parameter ap_ST_fsm_state16 = 30'd32768;
    parameter ap_ST_fsm_state17 = 30'd65536;
    parameter ap_ST_fsm_state18 = 30'd131072;
    parameter ap_ST_fsm_state19 = 30'd262144;
    parameter ap_ST_fsm_state20 = 30'd524288;
    parameter ap_ST_fsm_state21 = 30'd1048576;
    parameter ap_ST_fsm_state22 = 30'd2097152;
    parameter ap_ST_fsm_state23 = 30'd4194304;
    parameter ap_ST_fsm_state24 = 30'd8388608;
    parameter ap_ST_fsm_state25 = 30'd16777216;
    parameter ap_ST_fsm_state26 = 30'd33554432;
    parameter ap_ST_fsm_state27 = 30'd67108864;
    parameter ap_ST_fsm_state28 = 30'd134217728;
    parameter ap_ST_fsm_state29 = 30'd268435456;
    parameter ap_ST_fsm_state30 = 30'd536870912;

    input ap_clk;
    input ap_rst;
    input ap_start;
    output ap_done;
    output ap_idle;
    output ap_ready;
    input [63:0] y_in;
    input [63:0] x_in;
    output [63:0] ap_return;
    output [63:0] grp_fu_323_p_din0;
    output [63:0] grp_fu_323_p_din1;
    output [0:0] grp_fu_323_p_opcode;
    input [63:0] grp_fu_323_p_dout0;
    output grp_fu_323_p_ce;
    output [63:0] grp_fu_361_p_din0;
    output [63:0] grp_fu_361_p_din1;
    output [4:0] grp_fu_361_p_opcode;
    input [0:0] grp_fu_361_p_dout0;
    output grp_fu_361_p_ce;

    reg ap_done;
    reg ap_idle;
    reg ap_ready;
    reg[63:0] ap_return;

    (* fsm_encoding = "none" *) reg   [29:0] ap_CS_fsm;
    wire    ap_CS_fsm_state1;
    wire   [63:0] grp_atan2_generic_double_s_fu_160_ap_return;
    reg   [63:0] reg_179;
    wire    ap_CS_fsm_state4;
    wire    ap_CS_fsm_state7;
    wire   [0:0] icmp_ln18_1_fu_235_p2;
    reg   [0:0] icmp_ln18_1_reg_506;
    wire   [0:0] and_ln18_fu_247_p2;
    reg   [0:0] and_ln18_reg_511;
    wire   [0:0] ys_sign_fu_253_p3;
    reg   [0:0] ys_sign_reg_515;
    wire   [0:0] icmp_ln18_3_fu_267_p2;
    reg   [0:0] icmp_ln18_3_reg_526;
    wire   [0:0] and_ln18_1_fu_279_p2;
    reg   [0:0] and_ln18_1_reg_531;
    wire   [0:0] icmp_ln733_fu_285_p2;
    reg   [0:0] icmp_ln733_reg_535;
    wire   [0:0] and_ln733_fu_291_p2;
    reg   [0:0] and_ln733_reg_539;
    wire   [0:0] icmp_ln737_fu_297_p2;
    reg   [0:0] icmp_ln737_reg_543;
    wire   [0:0] and_ln737_fu_303_p2;
    reg   [0:0] and_ln737_reg_547;
    wire   [1:0] m_fu_309_p3;
    reg   [1:0] m_reg_551;
    wire   [0:0] and_ln18_2_fu_317_p2;
    reg   [0:0] and_ln18_2_reg_555;
    wire   [0:0] and_ln18_3_fu_323_p2;
    reg   [0:0] and_ln18_3_reg_559;
    wire   [63:0] a_fu_341_p1;
    reg   [63:0] a_reg_563;
    wire   [63:0] b_fu_358_p1;
    reg   [63:0] b_reg_570;
    wire   [0:0] icmp_ln717_fu_369_p2;
    reg   [0:0] icmp_ln717_reg_580;
    wire   [0:0] icmp_ln717_1_fu_375_p2;
    reg   [0:0] icmp_ln717_1_reg_585;
    reg   [63:0] sub_i_reg_593;
    wire    ap_CS_fsm_state14;
    reg   [63:0] d_reg_598;
    wire    ap_CS_fsm_state22;
    wire   [63:0] select_ln780_fu_430_p3;
    wire    ap_CS_fsm_state24;
    wire   [63:0] bitcast_ln497_5_fu_444_p1;
    wire    ap_CS_fsm_state25;
    wire   [63:0] bitcast_ln497_4_fu_455_p1;
    wire    ap_CS_fsm_state26;
    wire   [63:0] bitcast_ln497_3_fu_466_p1;
    wire    ap_CS_fsm_state27;
    wire   [63:0] bitcast_ln497_2_fu_477_p1;
    wire    ap_CS_fsm_state28;
    wire   [63:0] bitcast_ln497_1_fu_488_p1;
    wire    ap_CS_fsm_state29;
    wire   [63:0] bitcast_ln497_fu_499_p1;
    wire    ap_CS_fsm_state30;
    wire    grp_atan2_generic_double_s_fu_160_ap_start;
    wire    grp_atan2_generic_double_s_fu_160_ap_done;
    wire    grp_atan2_generic_double_s_fu_160_ap_idle;
    wire    grp_atan2_generic_double_s_fu_160_ap_ready;
    reg   [63:0] grp_atan2_generic_double_s_fu_160_y_in;
    reg   [63:0] grp_atan2_generic_double_s_fu_160_x_in;
    reg   [63:0] c_reg_82;
    wire    ap_CS_fsm_state15;
    wire    ap_CS_fsm_state5;
    wire   [0:0] icmp_ln715_fu_363_p2;
    reg   [63:0] ap_phi_mux_UnifiedRetVal_phi_fu_108_p40;
    reg   [63:0] UnifiedRetVal_reg_96;
    wire   [63:0] bitcast_ln797_1_fu_425_p1;
    wire    ap_CS_fsm_state23;
    wire   [63:0] bitcast_ln793_1_fu_411_p1;
    reg    grp_atan2_generic_double_s_fu_160_ap_start_reg;
    wire    ap_CS_fsm_state3;
    wire    ap_CS_fsm_state6;
    reg   [63:0] grp_fu_168_p0;
    reg   [63:0] grp_fu_168_p1;
    wire    ap_CS_fsm_state8;
    wire    ap_CS_fsm_state16;
    wire   [63:0] data_fu_185_p1;
    wire   [63:0] data_6_fu_211_p1;
    wire   [10:0] fpsy_exp_fu_215_p4;
    wire   [51:0] fpsy_sig_fu_225_p1;
    wire   [0:0] icmp_ln18_fu_229_p2;
    wire   [0:0] xor_ln18_fu_241_p2;
    wire   [10:0] fpsx_exp_fu_197_p4;
    wire   [51:0] fpsx_sig_fu_207_p1;
    wire   [0:0] icmp_ln18_2_fu_261_p2;
    wire   [0:0] xor_ln18_1_fu_273_p2;
    wire   [0:0] fpsx_sign_fu_189_p3;
    wire   [62:0] trunc_ln479_fu_329_p1;
    wire   [63:0] t_10_fu_333_p3;
    wire   [62:0] trunc_ln479_3_fu_346_p1;
    wire   [63:0] t_11_fu_350_p3;
    wire    ap_CS_fsm_state2;
    wire   [0:0] or_ln717_fu_381_p2;
    wire   [0:0] or_ln717_1_fu_385_p2;
    wire   [0:0] and_ln717_fu_389_p2;
    wire   [63:0] bitcast_ln793_fu_401_p1;
    wire   [63:0] xor_ln793_fu_405_p2;
    wire   [63:0] bitcast_ln797_fu_416_p1;
    wire   [63:0] xor_ln797_fu_419_p2;
    wire   [63:0] t_9_fu_437_p3;
    wire   [63:0] t_8_fu_448_p3;
    wire   [63:0] t_7_fu_459_p3;
    wire   [63:0] t_6_fu_470_p3;
    wire   [63:0] t_5_fu_481_p3;
    wire   [63:0] t_fu_492_p3;
    wire   [0:0] and_ln717_1_fu_395_p2;
    reg   [63:0] ap_return_preg;
    reg   [29:0] ap_NS_fsm;
    reg    ap_ST_fsm_state1_blk;
    wire    ap_ST_fsm_state2_blk;
    wire    ap_ST_fsm_state3_blk;
    reg    ap_ST_fsm_state4_blk;
    wire    ap_ST_fsm_state5_blk;
    wire    ap_ST_fsm_state6_blk;
    reg    ap_ST_fsm_state7_blk;
    wire    ap_ST_fsm_state8_blk;
    wire    ap_ST_fsm_state9_blk;
    wire    ap_ST_fsm_state10_blk;
    wire    ap_ST_fsm_state11_blk;
    wire    ap_ST_fsm_state12_blk;
    wire    ap_ST_fsm_state13_blk;
    wire    ap_ST_fsm_state14_blk;
    wire    ap_ST_fsm_state15_blk;
    wire    ap_ST_fsm_state16_blk;
    wire    ap_ST_fsm_state17_blk;
    wire    ap_ST_fsm_state18_blk;
    wire    ap_ST_fsm_state19_blk;
    wire    ap_ST_fsm_state20_blk;
    wire    ap_ST_fsm_state21_blk;
    wire    ap_ST_fsm_state22_blk;
    wire    ap_ST_fsm_state23_blk;
    wire    ap_ST_fsm_state24_blk;
    wire    ap_ST_fsm_state25_blk;
    wire    ap_ST_fsm_state26_blk;
    wire    ap_ST_fsm_state27_blk;
    wire    ap_ST_fsm_state28_blk;
    wire    ap_ST_fsm_state29_blk;
    wire    ap_ST_fsm_state30_blk;
    reg    ap_condition_706;
    wire    ap_ce_reg;

    // power-on initialization
    initial begin
        #0 ap_CS_fsm = 30'd1;
        #0 grp_atan2_generic_double_s_fu_160_ap_start_reg = 1'b0;
        #0 ap_return_preg = 64'd0;
    end

    main_atan2_generic_double_s grp_atan2_generic_double_s_fu_160 (
        .ap_clk(ap_clk),
        .ap_rst(ap_rst),
        .ap_start(grp_atan2_generic_double_s_fu_160_ap_start),
        .ap_done(grp_atan2_generic_double_s_fu_160_ap_done),
        .ap_idle(grp_atan2_generic_double_s_fu_160_ap_idle),
        .ap_ready(grp_atan2_generic_double_s_fu_160_ap_ready),
        .y_in(grp_atan2_generic_double_s_fu_160_y_in),
        .x_in(grp_atan2_generic_double_s_fu_160_x_in),
        .ap_return(grp_atan2_generic_double_s_fu_160_ap_return)
    );

    always @(posedge ap_clk) begin
        if (ap_rst == 1'b1) begin
            ap_CS_fsm <= ap_ST_fsm_state1;
        end else begin
            ap_CS_fsm <= ap_NS_fsm;
        end
    end

    always @(posedge ap_clk) begin
        if (ap_rst == 1'b1) begin
            ap_return_preg <= 64'd0;
        end else begin
            if ((1'b1 == ap_CS_fsm_state23)) begin
                ap_return_preg <= ap_phi_mux_UnifiedRetVal_phi_fu_108_p40;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (ap_rst == 1'b1) begin
            grp_atan2_generic_double_s_fu_160_ap_start_reg <= 1'b0;
        end else begin
            if (((1'b1 == ap_CS_fsm_state6) | (1'b1 == ap_CS_fsm_state3))) begin
                grp_atan2_generic_double_s_fu_160_ap_start_reg <= 1'b1;
            end else if ((grp_atan2_generic_double_s_fu_160_ap_ready == 1'b1)) begin
                grp_atan2_generic_double_s_fu_160_ap_start_reg <= 1'b0;
            end
        end
    end

    always @(posedge ap_clk) begin
        if ((1'b1 == ap_CS_fsm_state25)) begin
            UnifiedRetVal_reg_96 <= bitcast_ln497_5_fu_444_p1;
        end else if ((1'b1 == ap_CS_fsm_state26)) begin
            UnifiedRetVal_reg_96 <= bitcast_ln497_4_fu_455_p1;
        end else if ((1'b1 == ap_CS_fsm_state27)) begin
            UnifiedRetVal_reg_96 <= bitcast_ln497_3_fu_466_p1;
        end else if ((1'b1 == ap_CS_fsm_state28)) begin
            UnifiedRetVal_reg_96 <= bitcast_ln497_2_fu_477_p1;
        end else if ((1'b1 == ap_CS_fsm_state29)) begin
            UnifiedRetVal_reg_96 <= bitcast_ln497_1_fu_488_p1;
        end else if ((1'b1 == ap_CS_fsm_state30)) begin
            UnifiedRetVal_reg_96 <= bitcast_ln497_fu_499_p1;
        end else if (((ap_start == 1'b1) & (1'b1 == ap_CS_fsm_state1) & ((1'd1 == and_ln18_1_fu_279_p2) | (1'd1 == and_ln18_fu_247_p2)))) begin
            UnifiedRetVal_reg_96 <= 64'd9223372036854775807;
        end else if ((1'b1 == ap_CS_fsm_state24)) begin
            UnifiedRetVal_reg_96 <= select_ln780_fu_430_p3;
        end else if (((m_fu_309_p3 == 2'd0) & (icmp_ln737_fu_297_p2 == 1'd0) & (icmp_ln733_fu_285_p2 == 1'd0) & (ap_start == 1'b1) & (1'd1 == and_ln18_2_fu_317_p2) & (1'b1 == ap_CS_fsm_state1) & (1'd0 == and_ln18_3_fu_323_p2) & (1'd0 == and_ln737_fu_303_p2) & (1'd0 == and_ln733_fu_291_p2) & (1'd0 == and_ln18_1_fu_279_p2) & (1'd0 == and_ln18_fu_247_p2))) begin
            UnifiedRetVal_reg_96 <= 64'd0;
        end else if (((m_fu_309_p3 == 2'd0) & (icmp_ln737_fu_297_p2 == 1'd0) & (icmp_ln733_fu_285_p2 == 1'd0) & (ap_start == 1'b1) & (1'd1 == and_ln18_3_fu_323_p2) & (1'd1 == and_ln18_2_fu_317_p2) & (1'b1 == ap_CS_fsm_state1) & (1'd0 == and_ln737_fu_303_p2) & (1'd0 == and_ln733_fu_291_p2) & (1'd0 == and_ln18_1_fu_279_p2) & (1'd0 == and_ln18_fu_247_p2))) begin
            UnifiedRetVal_reg_96 <= 64'd4605249457297304856;
        end else if (((m_reg_551 == 2'd2) & (icmp_ln737_reg_543 == 1'd0) & (icmp_ln733_reg_535 == 1'd0) & (1'b1 == ap_CS_fsm_state23) & (1'd0 == and_ln18_3_reg_559) & (1'd0 == and_ln18_2_reg_555) & (1'd0 == and_ln737_reg_547) & (1'd0 == and_ln733_reg_539) & (1'd0 == and_ln18_1_reg_531) & (1'd0 == and_ln18_reg_511))) begin
            UnifiedRetVal_reg_96 <= d_reg_598;
        end else if (((m_reg_551 == 2'd0) & (icmp_ln737_reg_543 == 1'd0) & (icmp_ln733_reg_535 == 1'd0) & (1'b1 == ap_CS_fsm_state23) & (1'd0 == and_ln18_3_reg_559) & (1'd0 == and_ln18_2_reg_555) & (1'd0 == and_ln737_reg_547) & (1'd0 == and_ln733_reg_539) & (1'd0 == and_ln18_1_reg_531) & (1'd0 == and_ln18_reg_511))) begin
            UnifiedRetVal_reg_96 <= c_reg_82;
        end else if (((m_reg_551 == 2'd1) & (icmp_ln737_reg_543 == 1'd0) & (icmp_ln733_reg_535 == 1'd0) & (1'b1 == ap_CS_fsm_state23) & (1'd0 == and_ln18_3_reg_559) & (1'd0 == and_ln18_2_reg_555) & (1'd0 == and_ln737_reg_547) & (1'd0 == and_ln733_reg_539) & (1'd0 == and_ln18_1_reg_531) & (1'd0 == and_ln18_reg_511))) begin
            UnifiedRetVal_reg_96 <= bitcast_ln793_1_fu_411_p1;
        end else if (((m_reg_551 == 2'd3) & (icmp_ln737_reg_543 == 1'd0) & (icmp_ln733_reg_535 == 1'd0) & (1'b1 == ap_CS_fsm_state23) & (1'd0 == and_ln18_3_reg_559) & (1'd0 == and_ln18_2_reg_555) & (1'd0 == and_ln737_reg_547) & (1'd0 == and_ln733_reg_539) & (1'd0 == and_ln18_1_reg_531) & (1'd0 == and_ln18_reg_511))) begin
            UnifiedRetVal_reg_96 <= bitcast_ln797_1_fu_425_p1;
        end else if (((m_fu_309_p3 == 2'd1) & (icmp_ln737_fu_297_p2 == 1'd0) & (icmp_ln733_fu_285_p2 == 1'd0) & (ap_start == 1'b1) & (1'd1 == and_ln18_2_fu_317_p2) & (1'b1 == ap_CS_fsm_state1) & (1'd0 == and_ln18_3_fu_323_p2) & (1'd0 == and_ln737_fu_303_p2) & (1'd0 == and_ln733_fu_291_p2) & (1'd0 == and_ln18_1_fu_279_p2) & (1'd0 == and_ln18_fu_247_p2))) begin
            UnifiedRetVal_reg_96 <= 64'd9223372036854775808;
        end else if (((m_fu_309_p3 == 2'd2) & (icmp_ln737_fu_297_p2 == 1'd0) & (icmp_ln733_fu_285_p2 == 1'd0) & (ap_start == 1'b1) & (1'd1 == and_ln18_2_fu_317_p2) & (1'b1 == ap_CS_fsm_state1) & (1'd0 == and_ln18_3_fu_323_p2) & (1'd0 == and_ln737_fu_303_p2) & (1'd0 == and_ln733_fu_291_p2) & (1'd0 == and_ln18_1_fu_279_p2) & (1'd0 == and_ln18_fu_247_p2))) begin
            UnifiedRetVal_reg_96 <= 64'd4614256656552045848;
        end else if (((m_fu_309_p3 == 2'd3) & (icmp_ln737_fu_297_p2 == 1'd0) & (icmp_ln733_fu_285_p2 == 1'd0) & (ap_start == 1'b1) & (1'd1 == and_ln18_2_fu_317_p2) & (1'b1 == ap_CS_fsm_state1) & (1'd0 == and_ln18_3_fu_323_p2) & (1'd0 == and_ln737_fu_303_p2) & (1'd0 == and_ln733_fu_291_p2) & (1'd0 == and_ln18_1_fu_279_p2) & (1'd0 == and_ln18_fu_247_p2))) begin
            UnifiedRetVal_reg_96 <= 64'd13837628693406821656;
        end else if (((m_fu_309_p3 == 2'd1) & (icmp_ln737_fu_297_p2 == 1'd0) & (icmp_ln733_fu_285_p2 == 1'd0) & (ap_start == 1'b1) & (1'd1 == and_ln18_3_fu_323_p2) & (1'd1 == and_ln18_2_fu_317_p2) & (1'b1 == ap_CS_fsm_state1) & (1'd0 == and_ln737_fu_303_p2) & (1'd0 == and_ln733_fu_291_p2) & (1'd0 == and_ln18_1_fu_279_p2) & (1'd0 == and_ln18_fu_247_p2))) begin
            UnifiedRetVal_reg_96 <= 64'd13828621494152080664;
        end else if (((m_fu_309_p3 == 2'd2) & (icmp_ln737_fu_297_p2 == 1'd0) & (icmp_ln733_fu_285_p2 == 1'd0) & (ap_start == 1'b1) & (1'd1 == and_ln18_3_fu_323_p2) & (1'd1 == and_ln18_2_fu_317_p2) & (1'b1 == ap_CS_fsm_state1) & (1'd0 == and_ln737_fu_303_p2) & (1'd0 == and_ln733_fu_291_p2) & (1'd0 == and_ln18_1_fu_279_p2) & (1'd0 == and_ln18_fu_247_p2))) begin
            UnifiedRetVal_reg_96 <= 64'd4612488097114038738;
        end else if (((m_fu_309_p3 == 2'd3) & (icmp_ln737_fu_297_p2 == 1'd0) & (icmp_ln733_fu_285_p2 == 1'd0) & (ap_start == 1'b1) & (1'd1 == and_ln18_3_fu_323_p2) & (1'd1 == and_ln18_2_fu_317_p2) & (1'b1 == ap_CS_fsm_state1) & (1'd0 == and_ln737_fu_303_p2) & (1'd0 == and_ln733_fu_291_p2) & (1'd0 == and_ln18_1_fu_279_p2) & (1'd0 == and_ln18_fu_247_p2))) begin
            UnifiedRetVal_reg_96 <= 64'd13835860133968814546;
        end
    end

    always @(posedge ap_clk) begin
        if (((icmp_ln737_fu_297_p2 == 1'd0) & (icmp_ln733_fu_285_p2 == 1'd0) & (ap_start == 1'b1) & (1'b1 == ap_CS_fsm_state1) & (icmp_ln715_fu_363_p2 == 1'd1) & (1'd0 == and_ln18_3_fu_323_p2) & (1'd0 == and_ln18_2_fu_317_p2) & (1'd0 == and_ln737_fu_303_p2) & (1'd0 == and_ln733_fu_291_p2) & (1'd0 == and_ln18_1_fu_279_p2) & (1'd0 == and_ln18_fu_247_p2))) begin
            c_reg_82 <= 64'd4605249457297304856;
        end else if ((1'b1 == ap_CS_fsm_state5)) begin
            c_reg_82 <= reg_179;
        end else if ((1'b1 == ap_CS_fsm_state15)) begin
            c_reg_82 <= sub_i_reg_593;
        end
    end

    always @(posedge ap_clk) begin
        if ((1'b1 == ap_CS_fsm_state1)) begin
            a_reg_563[62 : 0] <= a_fu_341_p1[62 : 0];
            and_ln18_1_reg_531 <= and_ln18_1_fu_279_p2;
            and_ln18_2_reg_555 <= and_ln18_2_fu_317_p2;
            and_ln18_3_reg_559 <= and_ln18_3_fu_323_p2;
            and_ln18_reg_511 <= and_ln18_fu_247_p2;
            and_ln733_reg_539 <= and_ln733_fu_291_p2;
            and_ln737_reg_547 <= and_ln737_fu_303_p2;
            b_reg_570[62 : 0] <= b_fu_358_p1[62 : 0];
            icmp_ln18_1_reg_506 <= icmp_ln18_1_fu_235_p2;
            icmp_ln18_3_reg_526 <= icmp_ln18_3_fu_267_p2;
            icmp_ln717_1_reg_585 <= icmp_ln717_1_fu_375_p2;
            icmp_ln717_reg_580 <= icmp_ln717_fu_369_p2;
            icmp_ln733_reg_535 <= icmp_ln733_fu_285_p2;
            icmp_ln737_reg_543 <= icmp_ln737_fu_297_p2;
            m_reg_551 <= m_fu_309_p3;
            ys_sign_reg_515 <= data_6_fu_211_p1[32'd63];
        end
    end

    always @(posedge ap_clk) begin
        if ((1'b1 == ap_CS_fsm_state22)) begin
            d_reg_598 <= grp_fu_323_p_dout0;
        end
    end

    always @(posedge ap_clk) begin
        if (((1'b1 == ap_CS_fsm_state7) | (1'b1 == ap_CS_fsm_state4))) begin
            reg_179 <= grp_atan2_generic_double_s_fu_160_ap_return;
        end
    end

    always @(posedge ap_clk) begin
        if ((1'b1 == ap_CS_fsm_state14)) begin
            sub_i_reg_593 <= grp_fu_323_p_dout0;
        end
    end

    assign ap_ST_fsm_state10_blk = 1'b0;

    assign ap_ST_fsm_state11_blk = 1'b0;

    assign ap_ST_fsm_state12_blk = 1'b0;

    assign ap_ST_fsm_state13_blk = 1'b0;

    assign ap_ST_fsm_state14_blk = 1'b0;

    assign ap_ST_fsm_state15_blk = 1'b0;

    assign ap_ST_fsm_state16_blk = 1'b0;

    assign ap_ST_fsm_state17_blk = 1'b0;

    assign ap_ST_fsm_state18_blk = 1'b0;

    assign ap_ST_fsm_state19_blk = 1'b0;

    always @(*) begin
        if ((ap_start == 1'b0)) begin
            ap_ST_fsm_state1_blk = 1'b1;
        end else begin
            ap_ST_fsm_state1_blk = 1'b0;
        end
    end

    assign ap_ST_fsm_state20_blk = 1'b0;

    assign ap_ST_fsm_state21_blk = 1'b0;

    assign ap_ST_fsm_state22_blk = 1'b0;

    assign ap_ST_fsm_state23_blk = 1'b0;

    assign ap_ST_fsm_state24_blk = 1'b0;

    assign ap_ST_fsm_state25_blk = 1'b0;

    assign ap_ST_fsm_state26_blk = 1'b0;

    assign ap_ST_fsm_state27_blk = 1'b0;

    assign ap_ST_fsm_state28_blk = 1'b0;

    assign ap_ST_fsm_state29_blk = 1'b0;

    assign ap_ST_fsm_state2_blk  = 1'b0;

    assign ap_ST_fsm_state30_blk = 1'b0;

    assign ap_ST_fsm_state3_blk  = 1'b0;

    always @(*) begin
        if ((grp_atan2_generic_double_s_fu_160_ap_done == 1'b0)) begin
            ap_ST_fsm_state4_blk = 1'b1;
        end else begin
            ap_ST_fsm_state4_blk = 1'b0;
        end
    end

    assign ap_ST_fsm_state5_blk = 1'b0;

    assign ap_ST_fsm_state6_blk = 1'b0;

    always @(*) begin
        if ((grp_atan2_generic_double_s_fu_160_ap_done == 1'b0)) begin
            ap_ST_fsm_state7_blk = 1'b1;
        end else begin
            ap_ST_fsm_state7_blk = 1'b0;
        end
    end

    assign ap_ST_fsm_state8_blk = 1'b0;

    assign ap_ST_fsm_state9_blk = 1'b0;

    always @(*) begin
        if (((1'b1 == ap_CS_fsm_state23) | ((ap_start == 1'b0) & (1'b1 == ap_CS_fsm_state1)))) begin
            ap_done = 1'b1;
        end else begin
            ap_done = 1'b0;
        end
    end

    always @(*) begin
        if (((ap_start == 1'b0) & (1'b1 == ap_CS_fsm_state1))) begin
            ap_idle = 1'b1;
        end else begin
            ap_idle = 1'b0;
        end
    end

    always @(*) begin
        if ((1'b1 == ap_condition_706)) begin
            if ((m_reg_551 == 2'd2)) begin
                ap_phi_mux_UnifiedRetVal_phi_fu_108_p40 = d_reg_598;
            end else if ((m_reg_551 == 2'd0)) begin
                ap_phi_mux_UnifiedRetVal_phi_fu_108_p40 = c_reg_82;
            end else if ((m_reg_551 == 2'd1)) begin
                ap_phi_mux_UnifiedRetVal_phi_fu_108_p40 = bitcast_ln793_1_fu_411_p1;
            end else if ((m_reg_551 == 2'd3)) begin
                ap_phi_mux_UnifiedRetVal_phi_fu_108_p40 = bitcast_ln797_1_fu_425_p1;
            end else begin
                ap_phi_mux_UnifiedRetVal_phi_fu_108_p40 = UnifiedRetVal_reg_96;
            end
        end else begin
            ap_phi_mux_UnifiedRetVal_phi_fu_108_p40 = UnifiedRetVal_reg_96;
        end
    end

    always @(*) begin
        if ((1'b1 == ap_CS_fsm_state23)) begin
            ap_ready = 1'b1;
        end else begin
            ap_ready = 1'b0;
        end
    end

    always @(*) begin
        if ((1'b1 == ap_CS_fsm_state23)) begin
            ap_return = ap_phi_mux_UnifiedRetVal_phi_fu_108_p40;
        end else begin
            ap_return = ap_return_preg;
        end
    end

    always @(*) begin
        if ((1'b1 == ap_CS_fsm_state7)) begin
            grp_atan2_generic_double_s_fu_160_x_in = a_reg_563;
        end else if ((1'b1 == ap_CS_fsm_state4)) begin
            grp_atan2_generic_double_s_fu_160_x_in = b_reg_570;
        end else begin
            grp_atan2_generic_double_s_fu_160_x_in = 'bx;
        end
    end

    always @(*) begin
        if ((1'b1 == ap_CS_fsm_state7)) begin
            grp_atan2_generic_double_s_fu_160_y_in = b_reg_570;
        end else if ((1'b1 == ap_CS_fsm_state4)) begin
            grp_atan2_generic_double_s_fu_160_y_in = a_reg_563;
        end else begin
            grp_atan2_generic_double_s_fu_160_y_in = 'bx;
        end
    end

    always @(*) begin
        if ((1'b1 == ap_CS_fsm_state16)) begin
            grp_fu_168_p0 = 64'd4614256656552045848;
        end else if ((1'b1 == ap_CS_fsm_state8)) begin
            grp_fu_168_p0 = 64'd4609753056924675352;
        end else begin
            grp_fu_168_p0 = 'bx;
        end
    end

    always @(*) begin
        if ((1'b1 == ap_CS_fsm_state16)) begin
            grp_fu_168_p1 = c_reg_82;
        end else if ((1'b1 == ap_CS_fsm_state8)) begin
            grp_fu_168_p1 = reg_179;
        end else begin
            grp_fu_168_p1 = 'bx;
        end
    end

    always @(*) begin
        case (ap_CS_fsm)
            ap_ST_fsm_state1: begin
                if (((fpsx_sign_fu_189_p3 == 1'd1) & (ap_start == 1'b1) & (1'd1 == and_ln733_fu_291_p2) & (1'b1 == ap_CS_fsm_state1) & (1'd0 == and_ln18_1_fu_279_p2) & (1'd0 == and_ln18_fu_247_p2))) begin
                    ap_NS_fsm = ap_ST_fsm_state30;
                end else if (((fpsx_sign_fu_189_p3 == 1'd0) & (ap_start == 1'b1) & (1'd1 == and_ln733_fu_291_p2) & (1'b1 == ap_CS_fsm_state1) & (1'd0 == and_ln18_1_fu_279_p2) & (1'd0 == and_ln18_fu_247_p2))) begin
                    ap_NS_fsm = ap_ST_fsm_state29;
                end else if (((icmp_ln733_fu_285_p2 == 1'd1) & (fpsx_sign_fu_189_p3 == 1'd1) & (ap_start == 1'b1) & (1'b1 == ap_CS_fsm_state1) & (1'd0 == and_ln737_fu_303_p2) & (1'd0 == and_ln733_fu_291_p2) & (1'd0 == and_ln18_1_fu_279_p2) & (1'd0 == and_ln18_fu_247_p2))) begin
                    ap_NS_fsm = ap_ST_fsm_state27;
                end else if (((icmp_ln733_fu_285_p2 == 1'd1) & (fpsx_sign_fu_189_p3 == 1'd0) & (ap_start == 1'b1) & (1'b1 == ap_CS_fsm_state1) & (1'd0 == and_ln737_fu_303_p2) & (1'd0 == and_ln733_fu_291_p2) & (1'd0 == and_ln18_1_fu_279_p2) & (1'd0 == and_ln18_fu_247_p2))) begin
                    ap_NS_fsm = ap_ST_fsm_state26;
                end else if (((ap_start == 1'b1) & (1'b1 == ap_CS_fsm_state1) & ((1'd1 == and_ln18_fu_247_p2) | ((1'd1 == and_ln18_1_fu_279_p2) | ((icmp_ln737_fu_297_p2 == 1'd0) & (icmp_ln733_fu_285_p2 == 1'd0) & (1'd1 == and_ln18_2_fu_317_p2) & (1'd0 == and_ln737_fu_303_p2) & (1'd0 == and_ln733_fu_291_p2)))))) begin
                    ap_NS_fsm = ap_ST_fsm_state23;
                end else if (((icmp_ln737_fu_297_p2 == 1'd0) & (icmp_ln733_fu_285_p2 == 1'd0) & (ap_start == 1'b1) & (1'b1 == ap_CS_fsm_state1) & (icmp_ln715_fu_363_p2 == 1'd1) & (1'd0 == and_ln18_3_fu_323_p2) & (1'd0 == and_ln18_2_fu_317_p2) & (1'd0 == and_ln737_fu_303_p2) & (1'd0 == and_ln733_fu_291_p2) & (1'd0 == and_ln18_1_fu_279_p2) & (1'd0 == and_ln18_fu_247_p2))) begin
                    ap_NS_fsm = ap_ST_fsm_state16;
                end else if (((icmp_ln737_fu_297_p2 == 1'd0) & (icmp_ln733_fu_285_p2 == 1'd0) & (ap_start == 1'b1) & (1'b1 == ap_CS_fsm_state1) & (icmp_ln715_fu_363_p2 == 1'd0) & (1'd0 == and_ln18_3_fu_323_p2) & (1'd0 == and_ln18_2_fu_317_p2) & (1'd0 == and_ln737_fu_303_p2) & (1'd0 == and_ln733_fu_291_p2) & (1'd0 == and_ln18_1_fu_279_p2) & (1'd0 == and_ln18_fu_247_p2))) begin
                    ap_NS_fsm = ap_ST_fsm_state2;
                end else if (((icmp_ln737_fu_297_p2 == 1'd0) & (icmp_ln733_fu_285_p2 == 1'd0) & (ap_start == 1'b1) & (1'd1 == and_ln18_3_fu_323_p2) & (1'b1 == ap_CS_fsm_state1) & (1'd0 == and_ln18_2_fu_317_p2) & (1'd0 == and_ln737_fu_303_p2) & (1'd0 == and_ln733_fu_291_p2) & (1'd0 == and_ln18_1_fu_279_p2) & (1'd0 == and_ln18_fu_247_p2))) begin
                    ap_NS_fsm = ap_ST_fsm_state24;
                end else if (((icmp_ln737_fu_297_p2 == 1'd1) & (icmp_ln733_fu_285_p2 == 1'd0) & (ap_start == 1'b1) & (1'b1 == ap_CS_fsm_state1) & (1'd0 == and_ln737_fu_303_p2) & (1'd0 == and_ln733_fu_291_p2) & (1'd0 == and_ln18_1_fu_279_p2) & (1'd0 == and_ln18_fu_247_p2))) begin
                    ap_NS_fsm = ap_ST_fsm_state25;
                end else if (((ap_start == 1'b1) & (1'd1 == and_ln737_fu_303_p2) & (1'b1 == ap_CS_fsm_state1) & (1'd0 == and_ln733_fu_291_p2) & (1'd0 == and_ln18_1_fu_279_p2) & (1'd0 == and_ln18_fu_247_p2))) begin
                    ap_NS_fsm = ap_ST_fsm_state28;
                end else begin
                    ap_NS_fsm = ap_ST_fsm_state1;
                end
            end
            ap_ST_fsm_state2: begin
                if (((1'd1 == and_ln717_1_fu_395_p2) & (1'b1 == ap_CS_fsm_state2))) begin
                    ap_NS_fsm = ap_ST_fsm_state6;
                end else begin
                    ap_NS_fsm = ap_ST_fsm_state3;
                end
            end
            ap_ST_fsm_state3: begin
                ap_NS_fsm = ap_ST_fsm_state4;
            end
            ap_ST_fsm_state4: begin
                if (((1'b1 == ap_CS_fsm_state4) & (grp_atan2_generic_double_s_fu_160_ap_done == 1'b1))) begin
                    ap_NS_fsm = ap_ST_fsm_state5;
                end else begin
                    ap_NS_fsm = ap_ST_fsm_state4;
                end
            end
            ap_ST_fsm_state5: begin
                ap_NS_fsm = ap_ST_fsm_state16;
            end
            ap_ST_fsm_state6: begin
                ap_NS_fsm = ap_ST_fsm_state7;
            end
            ap_ST_fsm_state7: begin
                if (((1'b1 == ap_CS_fsm_state7) & (grp_atan2_generic_double_s_fu_160_ap_done == 1'b1))) begin
                    ap_NS_fsm = ap_ST_fsm_state8;
                end else begin
                    ap_NS_fsm = ap_ST_fsm_state7;
                end
            end
            ap_ST_fsm_state8: begin
                ap_NS_fsm = ap_ST_fsm_state9;
            end
            ap_ST_fsm_state9: begin
                ap_NS_fsm = ap_ST_fsm_state10;
            end
            ap_ST_fsm_state10: begin
                ap_NS_fsm = ap_ST_fsm_state11;
            end
            ap_ST_fsm_state11: begin
                ap_NS_fsm = ap_ST_fsm_state12;
            end
            ap_ST_fsm_state12: begin
                ap_NS_fsm = ap_ST_fsm_state13;
            end
            ap_ST_fsm_state13: begin
                ap_NS_fsm = ap_ST_fsm_state14;
            end
            ap_ST_fsm_state14: begin
                ap_NS_fsm = ap_ST_fsm_state15;
            end
            ap_ST_fsm_state15: begin
                ap_NS_fsm = ap_ST_fsm_state16;
            end
            ap_ST_fsm_state16: begin
                ap_NS_fsm = ap_ST_fsm_state17;
            end
            ap_ST_fsm_state17: begin
                ap_NS_fsm = ap_ST_fsm_state18;
            end
            ap_ST_fsm_state18: begin
                ap_NS_fsm = ap_ST_fsm_state19;
            end
            ap_ST_fsm_state19: begin
                ap_NS_fsm = ap_ST_fsm_state20;
            end
            ap_ST_fsm_state20: begin
                ap_NS_fsm = ap_ST_fsm_state21;
            end
            ap_ST_fsm_state21: begin
                ap_NS_fsm = ap_ST_fsm_state22;
            end
            ap_ST_fsm_state22: begin
                ap_NS_fsm = ap_ST_fsm_state23;
            end
            ap_ST_fsm_state23: begin
                ap_NS_fsm = ap_ST_fsm_state1;
            end
            ap_ST_fsm_state24: begin
                ap_NS_fsm = ap_ST_fsm_state23;
            end
            ap_ST_fsm_state25: begin
                ap_NS_fsm = ap_ST_fsm_state23;
            end
            ap_ST_fsm_state26: begin
                ap_NS_fsm = ap_ST_fsm_state23;
            end
            ap_ST_fsm_state27: begin
                ap_NS_fsm = ap_ST_fsm_state23;
            end
            ap_ST_fsm_state28: begin
                ap_NS_fsm = ap_ST_fsm_state23;
            end
            ap_ST_fsm_state29: begin
                ap_NS_fsm = ap_ST_fsm_state23;
            end
            ap_ST_fsm_state30: begin
                ap_NS_fsm = ap_ST_fsm_state23;
            end
            default: begin
                ap_NS_fsm = 'bx;
            end
        endcase
    end

    assign a_fu_341_p1 = t_10_fu_333_p3;

    assign and_ln18_1_fu_279_p2 = (xor_ln18_1_fu_273_p2 & icmp_ln18_2_fu_261_p2);

    assign and_ln18_2_fu_317_p2 = (icmp_ln18_3_fu_267_p2 & icmp_ln18_2_fu_261_p2);

    assign and_ln18_3_fu_323_p2 = (icmp_ln18_fu_229_p2 & icmp_ln18_1_fu_235_p2);

    assign and_ln18_fu_247_p2 = (xor_ln18_fu_241_p2 & icmp_ln18_fu_229_p2);

    assign and_ln717_1_fu_395_p2 = (grp_fu_361_p_dout0 & and_ln717_fu_389_p2);

    assign and_ln717_fu_389_p2 = (or_ln717_fu_381_p2 & or_ln717_1_fu_385_p2);

    assign and_ln733_fu_291_p2 = (icmp_ln733_fu_285_p2 & icmp_ln18_1_fu_235_p2);

    assign and_ln737_fu_303_p2 = (icmp_ln737_fu_297_p2 & icmp_ln18_3_fu_267_p2);

    assign ap_CS_fsm_state1 = ap_CS_fsm[32'd0];

    assign ap_CS_fsm_state14 = ap_CS_fsm[32'd13];

    assign ap_CS_fsm_state15 = ap_CS_fsm[32'd14];

    assign ap_CS_fsm_state16 = ap_CS_fsm[32'd15];

    assign ap_CS_fsm_state2 = ap_CS_fsm[32'd1];

    assign ap_CS_fsm_state22 = ap_CS_fsm[32'd21];

    assign ap_CS_fsm_state23 = ap_CS_fsm[32'd22];

    assign ap_CS_fsm_state24 = ap_CS_fsm[32'd23];

    assign ap_CS_fsm_state25 = ap_CS_fsm[32'd24];

    assign ap_CS_fsm_state26 = ap_CS_fsm[32'd25];

    assign ap_CS_fsm_state27 = ap_CS_fsm[32'd26];

    assign ap_CS_fsm_state28 = ap_CS_fsm[32'd27];

    assign ap_CS_fsm_state29 = ap_CS_fsm[32'd28];

    assign ap_CS_fsm_state3 = ap_CS_fsm[32'd2];

    assign ap_CS_fsm_state30 = ap_CS_fsm[32'd29];

    assign ap_CS_fsm_state4 = ap_CS_fsm[32'd3];

    assign ap_CS_fsm_state5 = ap_CS_fsm[32'd4];

    assign ap_CS_fsm_state6 = ap_CS_fsm[32'd5];

    assign ap_CS_fsm_state7 = ap_CS_fsm[32'd6];

    assign ap_CS_fsm_state8 = ap_CS_fsm[32'd7];

    always @(*) begin
        ap_condition_706 = ((icmp_ln737_reg_543 == 1'd0) & (icmp_ln733_reg_535 == 1'd0) & (1'b1 == ap_CS_fsm_state23) & (1'd0 == and_ln18_3_reg_559) & (1'd0 == and_ln18_2_reg_555) & (1'd0 == and_ln737_reg_547) & (1'd0 == and_ln733_reg_539) & (1'd0 == and_ln18_1_reg_531) & (1'd0 == and_ln18_reg_511));
    end

    assign b_fu_358_p1 = t_11_fu_350_p3;

    assign bitcast_ln497_1_fu_488_p1 = t_5_fu_481_p3;

    assign bitcast_ln497_2_fu_477_p1 = t_6_fu_470_p3;

    assign bitcast_ln497_3_fu_466_p1 = t_7_fu_459_p3;

    assign bitcast_ln497_4_fu_455_p1 = t_8_fu_448_p3;

    assign bitcast_ln497_5_fu_444_p1 = t_9_fu_437_p3;

    assign bitcast_ln497_fu_499_p1 = t_fu_492_p3;

    assign bitcast_ln793_1_fu_411_p1 = xor_ln793_fu_405_p2;

    assign bitcast_ln793_fu_401_p1 = c_reg_82;

    assign bitcast_ln797_1_fu_425_p1 = xor_ln797_fu_419_p2;

    assign bitcast_ln797_fu_416_p1 = d_reg_598;

    assign data_6_fu_211_p1 = y_in;

    assign data_fu_185_p1 = x_in;

    assign fpsx_exp_fu_197_p4 = {{data_fu_185_p1[62:52]}};

    assign fpsx_sig_fu_207_p1 = data_fu_185_p1[51:0];

    assign fpsx_sign_fu_189_p3 = data_fu_185_p1[32'd63];

    assign fpsy_exp_fu_215_p4 = {{data_6_fu_211_p1[62:52]}};

    assign fpsy_sig_fu_225_p1 = data_6_fu_211_p1[51:0];

    assign grp_atan2_generic_double_s_fu_160_ap_start = grp_atan2_generic_double_s_fu_160_ap_start_reg;

    assign grp_fu_323_p_ce = 1'b1;

    assign grp_fu_323_p_din0 = grp_fu_168_p0;

    assign grp_fu_323_p_din1 = grp_fu_168_p1;

    assign grp_fu_323_p_opcode = 2'd1;

    assign grp_fu_361_p_ce = 1'b1;

    assign grp_fu_361_p_din0 = a_fu_341_p1;

    assign grp_fu_361_p_din1 = b_fu_358_p1;

    assign grp_fu_361_p_opcode = 5'd2;

    assign icmp_ln18_1_fu_235_p2 = ((fpsy_sig_fu_225_p1 == 52'd0) ? 1'b1 : 1'b0);

    assign icmp_ln18_2_fu_261_p2 = ((fpsx_exp_fu_197_p4 == 11'd2047) ? 1'b1 : 1'b0);

    assign icmp_ln18_3_fu_267_p2 = ((fpsx_sig_fu_207_p1 == 52'd0) ? 1'b1 : 1'b0);

    assign icmp_ln18_fu_229_p2 = ((fpsy_exp_fu_215_p4 == 11'd2047) ? 1'b1 : 1'b0);

    assign icmp_ln715_fu_363_p2 = ((t_11_fu_350_p3 == t_10_fu_333_p3) ? 1'b1 : 1'b0);

    assign icmp_ln717_1_fu_375_p2 = ((fpsx_exp_fu_197_p4 != 11'd2047) ? 1'b1 : 1'b0);

    assign icmp_ln717_fu_369_p2 = ((fpsy_exp_fu_215_p4 != 11'd2047) ? 1'b1 : 1'b0);

    assign icmp_ln733_fu_285_p2 = ((fpsy_exp_fu_215_p4 == 11'd0) ? 1'b1 : 1'b0);

    assign icmp_ln737_fu_297_p2 = ((fpsx_exp_fu_197_p4 == 11'd0) ? 1'b1 : 1'b0);

    assign m_fu_309_p3 = {{fpsx_sign_fu_189_p3}, {ys_sign_fu_253_p3}};

    assign or_ln717_1_fu_385_p2 = (icmp_ln717_1_reg_585 | icmp_ln18_3_reg_526);

    assign or_ln717_fu_381_p2 = (icmp_ln717_reg_580 | icmp_ln18_1_reg_506);

    assign select_ln780_fu_430_p3 = ((ys_sign_reg_515[0:0] == 1'b1) ? 64'd13833125093779451160 : 64'd4609753056924675352);

    assign t_10_fu_333_p3 = {{1'd0}, {trunc_ln479_fu_329_p1}};

    assign t_11_fu_350_p3 = {{1'd0}, {trunc_ln479_3_fu_346_p1}};

    assign t_5_fu_481_p3 = {{ys_sign_reg_515}, {63'd0}};

    assign t_6_fu_470_p3 = {{ys_sign_reg_515}, {63'd4609753056924675352}};

    assign t_7_fu_459_p3 = {{ys_sign_reg_515}, {63'd4614256656552045848}};

    assign t_8_fu_448_p3 = {{ys_sign_reg_515}, {63'd0}};

    assign t_9_fu_437_p3 = {{ys_sign_reg_515}, {63'd4609753056924675352}};

    assign t_fu_492_p3 = {{ys_sign_reg_515}, {63'd4614256656552045848}};

    assign trunc_ln479_3_fu_346_p1 = data_fu_185_p1[62:0];

    assign trunc_ln479_fu_329_p1 = data_6_fu_211_p1[62:0];

    assign xor_ln18_1_fu_273_p2 = (icmp_ln18_3_fu_267_p2 ^ 1'd1);

    assign xor_ln18_fu_241_p2 = (icmp_ln18_1_fu_235_p2 ^ 1'd1);

    assign xor_ln793_fu_405_p2 = (bitcast_ln793_fu_401_p1 ^ 64'd9223372036854775808);

    assign xor_ln797_fu_419_p2 = (bitcast_ln797_fu_416_p1 ^ 64'd9223372036854775808);

    assign ys_sign_fu_253_p3 = data_6_fu_211_p1[32'd63];

    always @(posedge ap_clk) begin
        a_reg_563[63] <= 1'b0;
        b_reg_570[63] <= 1'b0;
    end

endmodule  //main_atan2_cordic_double_s
