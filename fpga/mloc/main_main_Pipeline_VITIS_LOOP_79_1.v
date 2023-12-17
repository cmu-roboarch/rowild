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

module main_main_Pipeline_VITIS_LOOP_79_1 (
    ap_clk,
    ap_rst,
    ap_start,
    ap_done,
    ap_idle,
    ap_ready,
    pf_address0,
    pf_ce0,
    pf_we0,
    pf_d0
);

    parameter ap_ST_fsm_pp0_stage0 = 1'd1;

    input ap_clk;
    input ap_rst;
    input ap_start;
    output ap_done;
    output ap_idle;
    output ap_ready;
    output [16:0] pf_address0;
    output pf_ce0;
    output [31:0] pf_we0;
    output [255:0] pf_d0;

    reg ap_idle;
    reg pf_ce0;
    reg[31:0] pf_we0;

    (* fsm_encoding = "none" *) reg   [0:0] ap_CS_fsm;
    wire    ap_CS_fsm_pp0_stage0;
    wire    ap_enable_reg_pp0_iter0;
    reg    ap_enable_reg_pp0_iter1;
    reg    ap_enable_reg_pp0_iter2;
    reg    ap_enable_reg_pp0_iter3;
    reg    ap_enable_reg_pp0_iter4;
    reg    ap_enable_reg_pp0_iter5;
    reg    ap_enable_reg_pp0_iter6;
    reg    ap_enable_reg_pp0_iter7;
    reg    ap_enable_reg_pp0_iter8;
    reg    ap_enable_reg_pp0_iter9;
    reg    ap_enable_reg_pp0_iter10;
    reg    ap_enable_reg_pp0_iter11;
    reg    ap_enable_reg_pp0_iter12;
    reg    ap_enable_reg_pp0_iter13;
    reg    ap_enable_reg_pp0_iter14;
    reg    ap_enable_reg_pp0_iter15;
    reg    ap_enable_reg_pp0_iter16;
    reg    ap_enable_reg_pp0_iter17;
    reg    ap_enable_reg_pp0_iter18;
    reg    ap_enable_reg_pp0_iter19;
    reg    ap_enable_reg_pp0_iter20;
    reg    ap_enable_reg_pp0_iter21;
    reg    ap_enable_reg_pp0_iter22;
    reg    ap_enable_reg_pp0_iter23;
    reg    ap_enable_reg_pp0_iter24;
    reg    ap_enable_reg_pp0_iter25;
    reg    ap_enable_reg_pp0_iter26;
    reg    ap_enable_reg_pp0_iter27;
    reg    ap_enable_reg_pp0_iter28;
    reg    ap_enable_reg_pp0_iter29;
    reg    ap_enable_reg_pp0_iter30;
    reg    ap_enable_reg_pp0_iter31;
    reg    ap_enable_reg_pp0_iter32;
    reg    ap_idle_pp0;
    wire    ap_block_pp0_stage0_subdone;
    wire   [0:0] icmp_ln79_fu_110_p2;
    reg    ap_condition_exit_pp0_iter0_stage0;
    wire    ap_loop_exit_ready;
    reg    ap_ready_int;
    wire    ap_block_pp0_stage0_11001;
    reg   [8:0] i_6_reg_270;
    reg   [8:0] i_6_reg_270_pp0_iter1_reg;
    reg   [8:0] i_6_reg_270_pp0_iter2_reg;
    reg   [8:0] i_6_reg_270_pp0_iter3_reg;
    reg   [8:0] i_6_reg_270_pp0_iter4_reg;
    reg   [8:0] i_6_reg_270_pp0_iter5_reg;
    reg   [8:0] i_6_reg_270_pp0_iter6_reg;
    reg   [8:0] i_6_reg_270_pp0_iter7_reg;
    reg   [8:0] i_6_reg_270_pp0_iter8_reg;
    reg   [8:0] i_6_reg_270_pp0_iter9_reg;
    reg   [8:0] i_6_reg_270_pp0_iter10_reg;
    reg   [8:0] i_6_reg_270_pp0_iter11_reg;
    reg   [8:0] i_6_reg_270_pp0_iter12_reg;
    reg   [8:0] i_6_reg_270_pp0_iter13_reg;
    reg   [8:0] i_6_reg_270_pp0_iter14_reg;
    reg   [8:0] i_6_reg_270_pp0_iter15_reg;
    reg   [8:0] i_6_reg_270_pp0_iter16_reg;
    reg   [8:0] i_6_reg_270_pp0_iter17_reg;
    reg   [8:0] i_6_reg_270_pp0_iter18_reg;
    reg   [8:0] i_6_reg_270_pp0_iter19_reg;
    reg   [8:0] i_6_reg_270_pp0_iter20_reg;
    reg   [8:0] i_6_reg_270_pp0_iter21_reg;
    reg   [8:0] i_6_reg_270_pp0_iter22_reg;
    reg   [8:0] i_6_reg_270_pp0_iter23_reg;
    reg   [8:0] i_6_reg_270_pp0_iter24_reg;
    reg   [8:0] i_6_reg_270_pp0_iter25_reg;
    reg   [8:0] i_6_reg_270_pp0_iter26_reg;
    reg   [8:0] i_6_reg_270_pp0_iter27_reg;
    reg   [8:0] i_6_reg_270_pp0_iter28_reg;
    reg   [8:0] i_6_reg_270_pp0_iter29_reg;
    reg   [8:0] i_6_reg_270_pp0_iter30_reg;
    reg   [8:0] i_6_reg_270_pp0_iter31_reg;
    wire   [19:0] grp_fu_156_p2;
    reg   [19:0] rem_urem_i_i_reg_289;
    wire   [21:0] grp_fu_131_p2;
    reg   [21:0] rem3_urem_i_i_reg_294;
    wire   [63:0] grp_fu_78_p1;
    reg   [63:0] x_reg_314;
    wire   [63:0] grp_fu_81_p1;
    reg   [63:0] y_reg_319;
    wire   [63:0] grp_fu_84_p1;
    reg   [63:0] theta_reg_324;
    wire   [63:0] zext_ln79_fu_208_p1;
    wire    ap_block_pp0_stage0;
    reg   [8:0] phi_urem_fu_50;
    wire   [8:0] select_ln79_fu_182_p3;
    wire    ap_loop_init;
    reg   [21:0] phi_mul1_fu_54;
    wire   [21:0] add_ln81_fu_125_p2;
    reg   [21:0] ap_sig_allocacmp_phi_mul1_load;
    reg   [19:0] phi_mul_fu_58;
    wire   [19:0] add_ln80_fu_150_p2;
    reg   [8:0] i_fu_62;
    wire   [8:0] add_ln79_fu_116_p2;
    reg   [8:0] ap_sig_allocacmp_i_6;
    wire   [31:0] grp_fu_78_p0;
    wire   [31:0] grp_fu_81_p0;
    wire   [31:0] grp_fu_84_p0;
    wire   [10:0] grp_fu_131_p1;
    wire   [10:0] grp_fu_156_p1;
    wire   [8:0] add_ln79_1_fu_170_p2;
    wire   [0:0] icmp_ln79_1_fu_176_p2;
    wire   [63:0] bitcast_ln87_fu_218_p1;
    wire   [63:0] bitcast_ln86_fu_215_p1;
    wire   [63:0] bitcast_ln85_fu_212_p1;
    wire   [248:0] or_ln_fu_221_p5;
    wire  signed [253:0] sext_ln88_fu_233_p1;
    reg    ap_done_reg;
    wire    ap_continue_int;
    reg    ap_done_int;
    reg    ap_loop_exit_ready_pp0_iter1_reg;
    reg    ap_loop_exit_ready_pp0_iter2_reg;
    reg    ap_loop_exit_ready_pp0_iter3_reg;
    reg    ap_loop_exit_ready_pp0_iter4_reg;
    reg    ap_loop_exit_ready_pp0_iter5_reg;
    reg    ap_loop_exit_ready_pp0_iter6_reg;
    reg    ap_loop_exit_ready_pp0_iter7_reg;
    reg    ap_loop_exit_ready_pp0_iter8_reg;
    reg    ap_loop_exit_ready_pp0_iter9_reg;
    reg    ap_loop_exit_ready_pp0_iter10_reg;
    reg    ap_loop_exit_ready_pp0_iter11_reg;
    reg    ap_loop_exit_ready_pp0_iter12_reg;
    reg    ap_loop_exit_ready_pp0_iter13_reg;
    reg    ap_loop_exit_ready_pp0_iter14_reg;
    reg    ap_loop_exit_ready_pp0_iter15_reg;
    reg    ap_loop_exit_ready_pp0_iter16_reg;
    reg    ap_loop_exit_ready_pp0_iter17_reg;
    reg    ap_loop_exit_ready_pp0_iter18_reg;
    reg    ap_loop_exit_ready_pp0_iter19_reg;
    reg    ap_loop_exit_ready_pp0_iter20_reg;
    reg    ap_loop_exit_ready_pp0_iter21_reg;
    reg    ap_loop_exit_ready_pp0_iter22_reg;
    reg    ap_loop_exit_ready_pp0_iter23_reg;
    reg    ap_loop_exit_ready_pp0_iter24_reg;
    reg    ap_loop_exit_ready_pp0_iter25_reg;
    reg    ap_loop_exit_ready_pp0_iter26_reg;
    reg    ap_loop_exit_ready_pp0_iter27_reg;
    reg    ap_loop_exit_ready_pp0_iter28_reg;
    reg    ap_loop_exit_ready_pp0_iter29_reg;
    reg    ap_loop_exit_ready_pp0_iter30_reg;
    reg    ap_loop_exit_ready_pp0_iter31_reg;
    reg   [0:0] ap_NS_fsm;
    wire    ap_enable_pp0;
    wire    ap_start_int;
    wire    ap_ce_reg;

    // power-on initialization
    initial begin
        #0 ap_CS_fsm = 1'd1;
        #0 ap_enable_reg_pp0_iter1 = 1'b0;
        #0 ap_enable_reg_pp0_iter2 = 1'b0;
        #0 ap_enable_reg_pp0_iter3 = 1'b0;
        #0 ap_enable_reg_pp0_iter4 = 1'b0;
        #0 ap_enable_reg_pp0_iter5 = 1'b0;
        #0 ap_enable_reg_pp0_iter6 = 1'b0;
        #0 ap_enable_reg_pp0_iter7 = 1'b0;
        #0 ap_enable_reg_pp0_iter8 = 1'b0;
        #0 ap_enable_reg_pp0_iter9 = 1'b0;
        #0 ap_enable_reg_pp0_iter10 = 1'b0;
        #0 ap_enable_reg_pp0_iter11 = 1'b0;
        #0 ap_enable_reg_pp0_iter12 = 1'b0;
        #0 ap_enable_reg_pp0_iter13 = 1'b0;
        #0 ap_enable_reg_pp0_iter14 = 1'b0;
        #0 ap_enable_reg_pp0_iter15 = 1'b0;
        #0 ap_enable_reg_pp0_iter16 = 1'b0;
        #0 ap_enable_reg_pp0_iter17 = 1'b0;
        #0 ap_enable_reg_pp0_iter18 = 1'b0;
        #0 ap_enable_reg_pp0_iter19 = 1'b0;
        #0 ap_enable_reg_pp0_iter20 = 1'b0;
        #0 ap_enable_reg_pp0_iter21 = 1'b0;
        #0 ap_enable_reg_pp0_iter22 = 1'b0;
        #0 ap_enable_reg_pp0_iter23 = 1'b0;
        #0 ap_enable_reg_pp0_iter24 = 1'b0;
        #0 ap_enable_reg_pp0_iter25 = 1'b0;
        #0 ap_enable_reg_pp0_iter26 = 1'b0;
        #0 ap_enable_reg_pp0_iter27 = 1'b0;
        #0 ap_enable_reg_pp0_iter28 = 1'b0;
        #0 ap_enable_reg_pp0_iter29 = 1'b0;
        #0 ap_enable_reg_pp0_iter30 = 1'b0;
        #0 ap_enable_reg_pp0_iter31 = 1'b0;
        #0 ap_enable_reg_pp0_iter32 = 1'b0;
        #0 phi_urem_fu_50 = 9'd0;
        #0 phi_mul1_fu_54 = 22'd0;
        #0 phi_mul_fu_58 = 20'd0;
        #0 i_fu_62 = 9'd0;
        #0 ap_done_reg = 1'b0;
    end

    main_sitodp_32ns_64_6_no_dsp_1 #(
        .ID(1),
        .NUM_STAGE(6),
        .din0_WIDTH(32),
        .dout_WIDTH(64)
    ) sitodp_32ns_64_6_no_dsp_1_U2 (
        .clk(ap_clk),
        .reset(ap_rst),
        .din0(grp_fu_78_p0),
        .ce(1'b1),
        .dout(grp_fu_78_p1)
    );

    main_sitodp_32ns_64_6_no_dsp_1 #(
        .ID(1),
        .NUM_STAGE(6),
        .din0_WIDTH(32),
        .dout_WIDTH(64)
    ) sitodp_32ns_64_6_no_dsp_1_U3 (
        .clk(ap_clk),
        .reset(ap_rst),
        .din0(grp_fu_81_p0),
        .ce(1'b1),
        .dout(grp_fu_81_p1)
    );

    main_sitodp_32ns_64_6_no_dsp_1 #(
        .ID(1),
        .NUM_STAGE(6),
        .din0_WIDTH(32),
        .dout_WIDTH(64)
    ) sitodp_32ns_64_6_no_dsp_1_U4 (
        .clk(ap_clk),
        .reset(ap_rst),
        .din0(grp_fu_84_p0),
        .ce(1'b1),
        .dout(grp_fu_84_p1)
    );

    main_urem_22ns_11ns_22_26_1 #(
        .ID(1),
        .NUM_STAGE(26),
        .din0_WIDTH(22),
        .din1_WIDTH(11),
        .dout_WIDTH(22)
    ) urem_22ns_11ns_22_26_1_U5 (
        .clk(ap_clk),
        .reset(ap_rst),
        .din0(ap_sig_allocacmp_phi_mul1_load),
        .din1(grp_fu_131_p1),
        .ce(1'b1),
        .dout(grp_fu_131_p2)
    );

    main_urem_20ns_11ns_20_24_1 #(
        .ID(1),
        .NUM_STAGE(24),
        .din0_WIDTH(20),
        .din1_WIDTH(11),
        .dout_WIDTH(20)
    ) urem_20ns_11ns_20_24_1_U6 (
        .clk(ap_clk),
        .reset(ap_rst),
        .din0(phi_mul_fu_58),
        .din1(grp_fu_156_p1),
        .ce(1'b1),
        .dout(grp_fu_156_p2)
    );

    main_flow_control_loop_pipe_sequential_init flow_control_loop_pipe_sequential_init_U (
        .ap_clk(ap_clk),
        .ap_rst(ap_rst),
        .ap_start(ap_start),
        .ap_ready(ap_ready),
        .ap_done(ap_done),
        .ap_start_int(ap_start_int),
        .ap_loop_init(ap_loop_init),
        .ap_ready_int(ap_ready_int),
        .ap_loop_exit_ready(ap_condition_exit_pp0_iter0_stage0),
        .ap_loop_exit_done(ap_done_int),
        .ap_continue_int(ap_continue_int),
        .ap_done_int(ap_done_int)
    );

    always @(posedge ap_clk) begin
        if (ap_rst == 1'b1) begin
            ap_CS_fsm <= ap_ST_fsm_pp0_stage0;
        end else begin
            ap_CS_fsm <= ap_NS_fsm;
        end
    end

    always @(posedge ap_clk) begin
        if (ap_rst == 1'b1) begin
            ap_done_reg <= 1'b0;
        end else begin
            if ((ap_continue_int == 1'b1)) begin
                ap_done_reg <= 1'b0;
            end else if (((ap_loop_exit_ready_pp0_iter31_reg == 1'b1) & (1'b0 == ap_block_pp0_stage0_subdone))) begin
                ap_done_reg <= 1'b1;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (ap_rst == 1'b1) begin
            ap_enable_reg_pp0_iter1 <= 1'b0;
        end else begin
            if ((1'b1 == ap_condition_exit_pp0_iter0_stage0)) begin
                ap_enable_reg_pp0_iter1 <= 1'b0;
            end else if (((1'b1 == ap_CS_fsm_pp0_stage0) & (1'b0 == ap_block_pp0_stage0_subdone))) begin
                ap_enable_reg_pp0_iter1 <= ap_start_int;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (ap_rst == 1'b1) begin
            ap_enable_reg_pp0_iter10 <= 1'b0;
        end else begin
            if ((1'b0 == ap_block_pp0_stage0_subdone)) begin
                ap_enable_reg_pp0_iter10 <= ap_enable_reg_pp0_iter9;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (ap_rst == 1'b1) begin
            ap_enable_reg_pp0_iter11 <= 1'b0;
        end else begin
            if ((1'b0 == ap_block_pp0_stage0_subdone)) begin
                ap_enable_reg_pp0_iter11 <= ap_enable_reg_pp0_iter10;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (ap_rst == 1'b1) begin
            ap_enable_reg_pp0_iter12 <= 1'b0;
        end else begin
            if ((1'b0 == ap_block_pp0_stage0_subdone)) begin
                ap_enable_reg_pp0_iter12 <= ap_enable_reg_pp0_iter11;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (ap_rst == 1'b1) begin
            ap_enable_reg_pp0_iter13 <= 1'b0;
        end else begin
            if ((1'b0 == ap_block_pp0_stage0_subdone)) begin
                ap_enable_reg_pp0_iter13 <= ap_enable_reg_pp0_iter12;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (ap_rst == 1'b1) begin
            ap_enable_reg_pp0_iter14 <= 1'b0;
        end else begin
            if ((1'b0 == ap_block_pp0_stage0_subdone)) begin
                ap_enable_reg_pp0_iter14 <= ap_enable_reg_pp0_iter13;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (ap_rst == 1'b1) begin
            ap_enable_reg_pp0_iter15 <= 1'b0;
        end else begin
            if ((1'b0 == ap_block_pp0_stage0_subdone)) begin
                ap_enable_reg_pp0_iter15 <= ap_enable_reg_pp0_iter14;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (ap_rst == 1'b1) begin
            ap_enable_reg_pp0_iter16 <= 1'b0;
        end else begin
            if ((1'b0 == ap_block_pp0_stage0_subdone)) begin
                ap_enable_reg_pp0_iter16 <= ap_enable_reg_pp0_iter15;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (ap_rst == 1'b1) begin
            ap_enable_reg_pp0_iter17 <= 1'b0;
        end else begin
            if ((1'b0 == ap_block_pp0_stage0_subdone)) begin
                ap_enable_reg_pp0_iter17 <= ap_enable_reg_pp0_iter16;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (ap_rst == 1'b1) begin
            ap_enable_reg_pp0_iter18 <= 1'b0;
        end else begin
            if ((1'b0 == ap_block_pp0_stage0_subdone)) begin
                ap_enable_reg_pp0_iter18 <= ap_enable_reg_pp0_iter17;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (ap_rst == 1'b1) begin
            ap_enable_reg_pp0_iter19 <= 1'b0;
        end else begin
            if ((1'b0 == ap_block_pp0_stage0_subdone)) begin
                ap_enable_reg_pp0_iter19 <= ap_enable_reg_pp0_iter18;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (ap_rst == 1'b1) begin
            ap_enable_reg_pp0_iter2 <= 1'b0;
        end else begin
            if ((1'b0 == ap_block_pp0_stage0_subdone)) begin
                ap_enable_reg_pp0_iter2 <= ap_enable_reg_pp0_iter1;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (ap_rst == 1'b1) begin
            ap_enable_reg_pp0_iter20 <= 1'b0;
        end else begin
            if ((1'b0 == ap_block_pp0_stage0_subdone)) begin
                ap_enable_reg_pp0_iter20 <= ap_enable_reg_pp0_iter19;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (ap_rst == 1'b1) begin
            ap_enable_reg_pp0_iter21 <= 1'b0;
        end else begin
            if ((1'b0 == ap_block_pp0_stage0_subdone)) begin
                ap_enable_reg_pp0_iter21 <= ap_enable_reg_pp0_iter20;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (ap_rst == 1'b1) begin
            ap_enable_reg_pp0_iter22 <= 1'b0;
        end else begin
            if ((1'b0 == ap_block_pp0_stage0_subdone)) begin
                ap_enable_reg_pp0_iter22 <= ap_enable_reg_pp0_iter21;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (ap_rst == 1'b1) begin
            ap_enable_reg_pp0_iter23 <= 1'b0;
        end else begin
            if ((1'b0 == ap_block_pp0_stage0_subdone)) begin
                ap_enable_reg_pp0_iter23 <= ap_enable_reg_pp0_iter22;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (ap_rst == 1'b1) begin
            ap_enable_reg_pp0_iter24 <= 1'b0;
        end else begin
            if ((1'b0 == ap_block_pp0_stage0_subdone)) begin
                ap_enable_reg_pp0_iter24 <= ap_enable_reg_pp0_iter23;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (ap_rst == 1'b1) begin
            ap_enable_reg_pp0_iter25 <= 1'b0;
        end else begin
            if ((1'b0 == ap_block_pp0_stage0_subdone)) begin
                ap_enable_reg_pp0_iter25 <= ap_enable_reg_pp0_iter24;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (ap_rst == 1'b1) begin
            ap_enable_reg_pp0_iter26 <= 1'b0;
        end else begin
            if ((1'b0 == ap_block_pp0_stage0_subdone)) begin
                ap_enable_reg_pp0_iter26 <= ap_enable_reg_pp0_iter25;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (ap_rst == 1'b1) begin
            ap_enable_reg_pp0_iter27 <= 1'b0;
        end else begin
            if ((1'b0 == ap_block_pp0_stage0_subdone)) begin
                ap_enable_reg_pp0_iter27 <= ap_enable_reg_pp0_iter26;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (ap_rst == 1'b1) begin
            ap_enable_reg_pp0_iter28 <= 1'b0;
        end else begin
            if ((1'b0 == ap_block_pp0_stage0_subdone)) begin
                ap_enable_reg_pp0_iter28 <= ap_enable_reg_pp0_iter27;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (ap_rst == 1'b1) begin
            ap_enable_reg_pp0_iter29 <= 1'b0;
        end else begin
            if ((1'b0 == ap_block_pp0_stage0_subdone)) begin
                ap_enable_reg_pp0_iter29 <= ap_enable_reg_pp0_iter28;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (ap_rst == 1'b1) begin
            ap_enable_reg_pp0_iter3 <= 1'b0;
        end else begin
            if ((1'b0 == ap_block_pp0_stage0_subdone)) begin
                ap_enable_reg_pp0_iter3 <= ap_enable_reg_pp0_iter2;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (ap_rst == 1'b1) begin
            ap_enable_reg_pp0_iter30 <= 1'b0;
        end else begin
            if ((1'b0 == ap_block_pp0_stage0_subdone)) begin
                ap_enable_reg_pp0_iter30 <= ap_enable_reg_pp0_iter29;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (ap_rst == 1'b1) begin
            ap_enable_reg_pp0_iter31 <= 1'b0;
        end else begin
            if ((1'b0 == ap_block_pp0_stage0_subdone)) begin
                ap_enable_reg_pp0_iter31 <= ap_enable_reg_pp0_iter30;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (ap_rst == 1'b1) begin
            ap_enable_reg_pp0_iter32 <= 1'b0;
        end else begin
            if ((1'b0 == ap_block_pp0_stage0_subdone)) begin
                ap_enable_reg_pp0_iter32 <= ap_enable_reg_pp0_iter31;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (ap_rst == 1'b1) begin
            ap_enable_reg_pp0_iter4 <= 1'b0;
        end else begin
            if ((1'b0 == ap_block_pp0_stage0_subdone)) begin
                ap_enable_reg_pp0_iter4 <= ap_enable_reg_pp0_iter3;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (ap_rst == 1'b1) begin
            ap_enable_reg_pp0_iter5 <= 1'b0;
        end else begin
            if ((1'b0 == ap_block_pp0_stage0_subdone)) begin
                ap_enable_reg_pp0_iter5 <= ap_enable_reg_pp0_iter4;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (ap_rst == 1'b1) begin
            ap_enable_reg_pp0_iter6 <= 1'b0;
        end else begin
            if ((1'b0 == ap_block_pp0_stage0_subdone)) begin
                ap_enable_reg_pp0_iter6 <= ap_enable_reg_pp0_iter5;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (ap_rst == 1'b1) begin
            ap_enable_reg_pp0_iter7 <= 1'b0;
        end else begin
            if ((1'b0 == ap_block_pp0_stage0_subdone)) begin
                ap_enable_reg_pp0_iter7 <= ap_enable_reg_pp0_iter6;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (ap_rst == 1'b1) begin
            ap_enable_reg_pp0_iter8 <= 1'b0;
        end else begin
            if ((1'b0 == ap_block_pp0_stage0_subdone)) begin
                ap_enable_reg_pp0_iter8 <= ap_enable_reg_pp0_iter7;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (ap_rst == 1'b1) begin
            ap_enable_reg_pp0_iter9 <= 1'b0;
        end else begin
            if ((1'b0 == ap_block_pp0_stage0_subdone)) begin
                ap_enable_reg_pp0_iter9 <= ap_enable_reg_pp0_iter8;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (((1'b1 == ap_CS_fsm_pp0_stage0) & (1'b0 == ap_block_pp0_stage0_11001))) begin
            if (((ap_enable_reg_pp0_iter0 == 1'b1) & (icmp_ln79_fu_110_p2 == 1'd0))) begin
                i_fu_62 <= add_ln79_fu_116_p2;
            end else if ((ap_loop_init == 1'b1)) begin
                i_fu_62 <= 9'd0;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (((1'b1 == ap_CS_fsm_pp0_stage0) & (1'b0 == ap_block_pp0_stage0_11001))) begin
            if (((ap_enable_reg_pp0_iter0 == 1'b1) & (icmp_ln79_fu_110_p2 == 1'd0))) begin
                phi_mul1_fu_54 <= add_ln81_fu_125_p2;
            end else if ((ap_loop_init == 1'b1)) begin
                phi_mul1_fu_54 <= 22'd0;
            end
        end
    end

    always @(posedge ap_clk) begin
        if ((1'b0 == ap_block_pp0_stage0_11001)) begin
            if (((ap_loop_init == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
                phi_mul_fu_58 <= 20'd0;
            end else if ((ap_enable_reg_pp0_iter2 == 1'b1)) begin
                phi_mul_fu_58 <= add_ln80_fu_150_p2;
            end
        end
    end

    always @(posedge ap_clk) begin
        if ((1'b0 == ap_block_pp0_stage0_11001)) begin
            if (((ap_loop_init == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
                phi_urem_fu_50 <= 9'd0;
            end else if ((ap_enable_reg_pp0_iter26 == 1'b1)) begin
                phi_urem_fu_50 <= select_ln79_fu_182_p3;
            end
        end
    end

    always @(posedge ap_clk) begin
        if ((1'b0 == ap_block_pp0_stage0_11001)) begin
            ap_loop_exit_ready_pp0_iter10_reg <= ap_loop_exit_ready_pp0_iter9_reg;
            ap_loop_exit_ready_pp0_iter11_reg <= ap_loop_exit_ready_pp0_iter10_reg;
            ap_loop_exit_ready_pp0_iter12_reg <= ap_loop_exit_ready_pp0_iter11_reg;
            ap_loop_exit_ready_pp0_iter13_reg <= ap_loop_exit_ready_pp0_iter12_reg;
            ap_loop_exit_ready_pp0_iter14_reg <= ap_loop_exit_ready_pp0_iter13_reg;
            ap_loop_exit_ready_pp0_iter15_reg <= ap_loop_exit_ready_pp0_iter14_reg;
            ap_loop_exit_ready_pp0_iter16_reg <= ap_loop_exit_ready_pp0_iter15_reg;
            ap_loop_exit_ready_pp0_iter17_reg <= ap_loop_exit_ready_pp0_iter16_reg;
            ap_loop_exit_ready_pp0_iter18_reg <= ap_loop_exit_ready_pp0_iter17_reg;
            ap_loop_exit_ready_pp0_iter19_reg <= ap_loop_exit_ready_pp0_iter18_reg;
            ap_loop_exit_ready_pp0_iter20_reg <= ap_loop_exit_ready_pp0_iter19_reg;
            ap_loop_exit_ready_pp0_iter21_reg <= ap_loop_exit_ready_pp0_iter20_reg;
            ap_loop_exit_ready_pp0_iter22_reg <= ap_loop_exit_ready_pp0_iter21_reg;
            ap_loop_exit_ready_pp0_iter23_reg <= ap_loop_exit_ready_pp0_iter22_reg;
            ap_loop_exit_ready_pp0_iter24_reg <= ap_loop_exit_ready_pp0_iter23_reg;
            ap_loop_exit_ready_pp0_iter25_reg <= ap_loop_exit_ready_pp0_iter24_reg;
            ap_loop_exit_ready_pp0_iter26_reg <= ap_loop_exit_ready_pp0_iter25_reg;
            ap_loop_exit_ready_pp0_iter27_reg <= ap_loop_exit_ready_pp0_iter26_reg;
            ap_loop_exit_ready_pp0_iter28_reg <= ap_loop_exit_ready_pp0_iter27_reg;
            ap_loop_exit_ready_pp0_iter29_reg <= ap_loop_exit_ready_pp0_iter28_reg;
            ap_loop_exit_ready_pp0_iter30_reg <= ap_loop_exit_ready_pp0_iter29_reg;
            ap_loop_exit_ready_pp0_iter31_reg <= ap_loop_exit_ready_pp0_iter30_reg;
            ap_loop_exit_ready_pp0_iter3_reg <= ap_loop_exit_ready_pp0_iter2_reg;
            ap_loop_exit_ready_pp0_iter4_reg <= ap_loop_exit_ready_pp0_iter3_reg;
            ap_loop_exit_ready_pp0_iter5_reg <= ap_loop_exit_ready_pp0_iter4_reg;
            ap_loop_exit_ready_pp0_iter6_reg <= ap_loop_exit_ready_pp0_iter5_reg;
            ap_loop_exit_ready_pp0_iter7_reg <= ap_loop_exit_ready_pp0_iter6_reg;
            ap_loop_exit_ready_pp0_iter8_reg <= ap_loop_exit_ready_pp0_iter7_reg;
            ap_loop_exit_ready_pp0_iter9_reg <= ap_loop_exit_ready_pp0_iter8_reg;
            i_6_reg_270_pp0_iter10_reg <= i_6_reg_270_pp0_iter9_reg;
            i_6_reg_270_pp0_iter11_reg <= i_6_reg_270_pp0_iter10_reg;
            i_6_reg_270_pp0_iter12_reg <= i_6_reg_270_pp0_iter11_reg;
            i_6_reg_270_pp0_iter13_reg <= i_6_reg_270_pp0_iter12_reg;
            i_6_reg_270_pp0_iter14_reg <= i_6_reg_270_pp0_iter13_reg;
            i_6_reg_270_pp0_iter15_reg <= i_6_reg_270_pp0_iter14_reg;
            i_6_reg_270_pp0_iter16_reg <= i_6_reg_270_pp0_iter15_reg;
            i_6_reg_270_pp0_iter17_reg <= i_6_reg_270_pp0_iter16_reg;
            i_6_reg_270_pp0_iter18_reg <= i_6_reg_270_pp0_iter17_reg;
            i_6_reg_270_pp0_iter19_reg <= i_6_reg_270_pp0_iter18_reg;
            i_6_reg_270_pp0_iter20_reg <= i_6_reg_270_pp0_iter19_reg;
            i_6_reg_270_pp0_iter21_reg <= i_6_reg_270_pp0_iter20_reg;
            i_6_reg_270_pp0_iter22_reg <= i_6_reg_270_pp0_iter21_reg;
            i_6_reg_270_pp0_iter23_reg <= i_6_reg_270_pp0_iter22_reg;
            i_6_reg_270_pp0_iter24_reg <= i_6_reg_270_pp0_iter23_reg;
            i_6_reg_270_pp0_iter25_reg <= i_6_reg_270_pp0_iter24_reg;
            i_6_reg_270_pp0_iter26_reg <= i_6_reg_270_pp0_iter25_reg;
            i_6_reg_270_pp0_iter27_reg <= i_6_reg_270_pp0_iter26_reg;
            i_6_reg_270_pp0_iter28_reg <= i_6_reg_270_pp0_iter27_reg;
            i_6_reg_270_pp0_iter29_reg <= i_6_reg_270_pp0_iter28_reg;
            i_6_reg_270_pp0_iter2_reg <= i_6_reg_270_pp0_iter1_reg;
            i_6_reg_270_pp0_iter30_reg <= i_6_reg_270_pp0_iter29_reg;
            i_6_reg_270_pp0_iter31_reg <= i_6_reg_270_pp0_iter30_reg;
            i_6_reg_270_pp0_iter3_reg <= i_6_reg_270_pp0_iter2_reg;
            i_6_reg_270_pp0_iter4_reg <= i_6_reg_270_pp0_iter3_reg;
            i_6_reg_270_pp0_iter5_reg <= i_6_reg_270_pp0_iter4_reg;
            i_6_reg_270_pp0_iter6_reg <= i_6_reg_270_pp0_iter5_reg;
            i_6_reg_270_pp0_iter7_reg <= i_6_reg_270_pp0_iter6_reg;
            i_6_reg_270_pp0_iter8_reg <= i_6_reg_270_pp0_iter7_reg;
            i_6_reg_270_pp0_iter9_reg <= i_6_reg_270_pp0_iter8_reg;
            rem3_urem_i_i_reg_294 <= grp_fu_131_p2;
            rem_urem_i_i_reg_289 <= grp_fu_156_p2;
            theta_reg_324 <= grp_fu_84_p1;
            x_reg_314 <= grp_fu_78_p1;
            y_reg_319 <= grp_fu_81_p1;
        end
    end

    always @(posedge ap_clk) begin
        if (((1'b1 == ap_CS_fsm_pp0_stage0) & (1'b0 == ap_block_pp0_stage0_11001))) begin
            ap_loop_exit_ready_pp0_iter1_reg <= ap_loop_exit_ready;
            ap_loop_exit_ready_pp0_iter2_reg <= ap_loop_exit_ready_pp0_iter1_reg;
            i_6_reg_270 <= ap_sig_allocacmp_i_6;
            i_6_reg_270_pp0_iter1_reg <= i_6_reg_270;
        end
    end

    always @(*) begin
        if (((ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0) & (icmp_ln79_fu_110_p2 == 1'd1) & (1'b0 == ap_block_pp0_stage0_subdone))) begin
            ap_condition_exit_pp0_iter0_stage0 = 1'b1;
        end else begin
            ap_condition_exit_pp0_iter0_stage0 = 1'b0;
        end
    end

    always @(*) begin
        if (((ap_loop_exit_ready_pp0_iter31_reg == 1'b1) & (1'b0 == ap_block_pp0_stage0_subdone))) begin
            ap_done_int = 1'b1;
        end else begin
            ap_done_int = ap_done_reg;
        end
    end

    always @(*) begin
        if (((ap_start_int == 1'b0) & (1'b1 == ap_CS_fsm_pp0_stage0) & (ap_idle_pp0 == 1'b1))) begin
            ap_idle = 1'b1;
        end else begin
            ap_idle = 1'b0;
        end
    end

    always @(*) begin
        if (((ap_enable_reg_pp0_iter26 == 1'b0) & (ap_enable_reg_pp0_iter25 == 1'b0) & (ap_enable_reg_pp0_iter24 == 1'b0) & (ap_enable_reg_pp0_iter23 == 1'b0) & (ap_enable_reg_pp0_iter22 == 1'b0) & (ap_enable_reg_pp0_iter21 == 1'b0) & (ap_enable_reg_pp0_iter20 == 1'b0) & (ap_enable_reg_pp0_iter19 == 1'b0) & (ap_enable_reg_pp0_iter18 == 1'b0) & (ap_enable_reg_pp0_iter17 == 1'b0) & (ap_enable_reg_pp0_iter16 == 1'b0) & (ap_enable_reg_pp0_iter15 == 1'b0) & (ap_enable_reg_pp0_iter14 == 1'b0) & (ap_enable_reg_pp0_iter13 == 1'b0) & (ap_enable_reg_pp0_iter12 == 1'b0) & (ap_enable_reg_pp0_iter11 == 1'b0) & (ap_enable_reg_pp0_iter10 == 1'b0) & (ap_enable_reg_pp0_iter9 == 1'b0) & (ap_enable_reg_pp0_iter8 == 1'b0) & (ap_enable_reg_pp0_iter7 == 1'b0) & (ap_enable_reg_pp0_iter6 == 1'b0) & (ap_enable_reg_pp0_iter5 == 1'b0) & (ap_enable_reg_pp0_iter4 == 1'b0) & (ap_enable_reg_pp0_iter3 == 1'b0) & (ap_enable_reg_pp0_iter2 == 1'b0) & (ap_enable_reg_pp0_iter1 == 1'b0) & (ap_enable_reg_pp0_iter0 == 1'b0) & (ap_enable_reg_pp0_iter32 == 1'b0) 
    & (ap_enable_reg_pp0_iter31 == 1'b0) & (ap_enable_reg_pp0_iter30 == 1'b0) & (ap_enable_reg_pp0_iter29 == 1'b0) & (ap_enable_reg_pp0_iter28 == 1'b0) & (ap_enable_reg_pp0_iter27 == 1'b0))) begin
            ap_idle_pp0 = 1'b1;
        end else begin
            ap_idle_pp0 = 1'b0;
        end
    end

    always @(*) begin
        if (((ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0) & (1'b0 == ap_block_pp0_stage0_subdone))) begin
            ap_ready_int = 1'b1;
        end else begin
            ap_ready_int = 1'b0;
        end
    end

    always @(*) begin
        if (((ap_loop_init == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0) & (1'b0 == ap_block_pp0_stage0))) begin
            ap_sig_allocacmp_i_6 = 9'd0;
        end else begin
            ap_sig_allocacmp_i_6 = i_fu_62;
        end
    end

    always @(*) begin
        if (((ap_loop_init == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0) & (1'b0 == ap_block_pp0_stage0))) begin
            ap_sig_allocacmp_phi_mul1_load = 22'd0;
        end else begin
            ap_sig_allocacmp_phi_mul1_load = phi_mul1_fu_54;
        end
    end

    always @(*) begin
        if (((1'b0 == ap_block_pp0_stage0_11001) & (ap_enable_reg_pp0_iter32 == 1'b1))) begin
            pf_ce0 = 1'b1;
        end else begin
            pf_ce0 = 1'b0;
        end
    end

    always @(*) begin
        if (((1'b0 == ap_block_pp0_stage0_11001) & (ap_enable_reg_pp0_iter32 == 1'b1))) begin
            pf_we0 = 32'd4294967295;
        end else begin
            pf_we0 = 32'd0;
        end
    end

    always @(*) begin
        case (ap_CS_fsm)
            ap_ST_fsm_pp0_stage0: begin
                ap_NS_fsm = ap_ST_fsm_pp0_stage0;
            end
            default: begin
                ap_NS_fsm = 'bx;
            end
        endcase
    end

    assign add_ln79_1_fu_170_p2 = (phi_urem_fu_50 + 9'd1);

    assign add_ln79_fu_116_p2 = (ap_sig_allocacmp_i_6 + 9'd1);

    assign add_ln80_fu_150_p2 = (phi_mul_fu_58 + 20'd1234);

    assign add_ln81_fu_125_p2 = (ap_sig_allocacmp_phi_mul1_load + 22'd4321);

    assign ap_CS_fsm_pp0_stage0 = ap_CS_fsm[32'd0];

    assign ap_block_pp0_stage0 = ~(1'b1 == 1'b1);

    assign ap_block_pp0_stage0_11001 = ~(1'b1 == 1'b1);

    assign ap_block_pp0_stage0_subdone = ~(1'b1 == 1'b1);

    assign ap_enable_pp0 = (ap_idle_pp0 ^ 1'b1);

    assign ap_enable_reg_pp0_iter0 = ap_start_int;

    assign ap_loop_exit_ready = ap_condition_exit_pp0_iter0_stage0;

    assign bitcast_ln85_fu_212_p1 = x_reg_314;

    assign bitcast_ln86_fu_215_p1 = y_reg_319;

    assign bitcast_ln87_fu_218_p1 = theta_reg_324;

    assign grp_fu_131_p1 = 22'd800;

    assign grp_fu_156_p1 = 20'd800;

    assign grp_fu_78_p0 = rem_urem_i_i_reg_289;

    assign grp_fu_81_p0 = rem3_urem_i_i_reg_294;

    assign grp_fu_84_p0 = phi_urem_fu_50;

    assign icmp_ln79_1_fu_176_p2 = ((add_ln79_1_fu_170_p2 < 9'd3) ? 1'b1 : 1'b0);

    assign icmp_ln79_fu_110_p2 = ((ap_sig_allocacmp_i_6 == 9'd500) ? 1'b1 : 1'b0);

    assign or_ln_fu_221_p5 = {
        {{{{{57'd99187278193207804}, {bitcast_ln87_fu_218_p1}}}, {bitcast_ln86_fu_215_p1}}},
        {bitcast_ln85_fu_212_p1}
    };

    assign pf_address0 = zext_ln79_fu_208_p1;

    assign pf_d0 = $unsigned(sext_ln88_fu_233_p1);

    assign select_ln79_fu_182_p3 = ((icmp_ln79_1_fu_176_p2[0:0] == 1'b1) ? add_ln79_1_fu_170_p2 : 9'd0);

    assign sext_ln88_fu_233_p1 = $signed(or_ln_fu_221_p5);

    assign zext_ln79_fu_208_p1 = i_6_reg_270_pp0_iter31_reg;

endmodule  //main_main_Pipeline_VITIS_LOOP_79_1