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

module main_planRRT_Pipeline_VITIS_LOOP_121_6 (
    ap_clk,
    ap_rst,
    ap_start,
    ap_done,
    ap_idle,
    ap_ready,
    qNear_address0,
    qNear_ce0,
    qNear_q0,
    qRand_address0,
    qRand_ce0,
    qRand_q0,
    f,
    qConnect_address0,
    qConnect_ce0,
    qConnect_we0,
    qConnect_d0,
    grp_fu_2529_p_din0,
    grp_fu_2529_p_din1,
    grp_fu_2529_p_opcode,
    grp_fu_2529_p_dout0,
    grp_fu_2529_p_ce,
    grp_fu_2537_p_din0,
    grp_fu_2537_p_din1,
    grp_fu_2537_p_opcode,
    grp_fu_2537_p_dout0,
    grp_fu_2537_p_ce,
    grp_fu_2533_p_din0,
    grp_fu_2533_p_din1,
    grp_fu_2533_p_dout0,
    grp_fu_2533_p_ce
);

    parameter ap_ST_fsm_pp0_stage0 = 1'd1;

    input ap_clk;
    input ap_rst;
    input ap_start;
    output ap_done;
    output ap_idle;
    output ap_ready;
    output [2:0] qNear_address0;
    output qNear_ce0;
    input [63:0] qNear_q0;
    output [2:0] qRand_address0;
    output qRand_ce0;
    input [63:0] qRand_q0;
    input [63:0] f;
    output [2:0] qConnect_address0;
    output qConnect_ce0;
    output qConnect_we0;
    output [63:0] qConnect_d0;
    output [63:0] grp_fu_2529_p_din0;
    output [63:0] grp_fu_2529_p_din1;
    output [0:0] grp_fu_2529_p_opcode;
    input [63:0] grp_fu_2529_p_dout0;
    output grp_fu_2529_p_ce;
    output [63:0] grp_fu_2537_p_din0;
    output [63:0] grp_fu_2537_p_din1;
    output [0:0] grp_fu_2537_p_opcode;
    input [63:0] grp_fu_2537_p_dout0;
    output grp_fu_2537_p_ce;
    output [63:0] grp_fu_2533_p_din0;
    output [63:0] grp_fu_2533_p_din1;
    input [63:0] grp_fu_2533_p_dout0;
    output grp_fu_2533_p_ce;

    reg ap_idle;
    reg qNear_ce0;
    reg qRand_ce0;
    reg qConnect_ce0;
    reg qConnect_we0;

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
    reg    ap_idle_pp0;
    wire    ap_block_pp0_stage0_subdone;
    wire   [0:0] icmp_ln121_fu_105_p2;
    reg    ap_condition_exit_pp0_iter0_stage0;
    wire    ap_loop_exit_ready;
    reg    ap_ready_int;
    wire    ap_block_pp0_stage0_11001;
    wire   [63:0] zext_ln121_fu_117_p1;
    reg   [63:0] zext_ln121_reg_144;
    reg   [63:0] zext_ln121_reg_144_pp0_iter1_reg;
    reg   [63:0] zext_ln121_reg_144_pp0_iter2_reg;
    reg   [63:0] zext_ln121_reg_144_pp0_iter3_reg;
    reg   [63:0] zext_ln121_reg_144_pp0_iter4_reg;
    reg   [63:0] zext_ln121_reg_144_pp0_iter5_reg;
    reg   [63:0] zext_ln121_reg_144_pp0_iter6_reg;
    reg   [63:0] zext_ln121_reg_144_pp0_iter7_reg;
    reg   [63:0] zext_ln121_reg_144_pp0_iter8_reg;
    reg   [63:0] zext_ln121_reg_144_pp0_iter9_reg;
    reg   [63:0] zext_ln121_reg_144_pp0_iter10_reg;
    reg   [63:0] zext_ln121_reg_144_pp0_iter11_reg;
    reg   [63:0] zext_ln121_reg_144_pp0_iter12_reg;
    reg   [63:0] zext_ln121_reg_144_pp0_iter13_reg;
    reg   [63:0] zext_ln121_reg_144_pp0_iter14_reg;
    reg   [63:0] zext_ln121_reg_144_pp0_iter15_reg;
    reg   [63:0] zext_ln121_reg_144_pp0_iter16_reg;
    reg   [63:0] zext_ln121_reg_144_pp0_iter17_reg;
    reg   [63:0] zext_ln121_reg_144_pp0_iter18_reg;
    reg   [63:0] zext_ln121_reg_144_pp0_iter19_reg;
    reg   [63:0] zext_ln121_reg_144_pp0_iter20_reg;
    reg   [63:0] zext_ln121_reg_144_pp0_iter21_reg;
    reg   [63:0] zext_ln121_reg_144_pp0_iter22_reg;
    reg   [63:0] qNear_load_reg_159;
    reg   [63:0] qNear_load_reg_159_pp0_iter2_reg;
    reg   [63:0] qNear_load_reg_159_pp0_iter3_reg;
    reg   [63:0] qNear_load_reg_159_pp0_iter4_reg;
    reg   [63:0] qNear_load_reg_159_pp0_iter5_reg;
    reg   [63:0] qNear_load_reg_159_pp0_iter6_reg;
    reg   [63:0] qNear_load_reg_159_pp0_iter7_reg;
    reg   [63:0] qNear_load_reg_159_pp0_iter8_reg;
    reg   [63:0] qNear_load_reg_159_pp0_iter9_reg;
    reg   [63:0] qNear_load_reg_159_pp0_iter10_reg;
    reg   [63:0] qNear_load_reg_159_pp0_iter11_reg;
    reg   [63:0] qNear_load_reg_159_pp0_iter12_reg;
    reg   [63:0] qNear_load_reg_159_pp0_iter13_reg;
    reg   [63:0] qNear_load_reg_159_pp0_iter14_reg;
    reg   [63:0] qNear_load_reg_159_pp0_iter15_reg;
    reg   [63:0] qRand_load_reg_165;
    reg   [63:0] sub_reg_170;
    reg   [63:0] mul_reg_175;
    reg   [63:0] add_reg_180;
    wire    ap_block_pp0_stage0;
    reg   [2:0] i_7_fu_36;
    wire   [2:0] add_ln121_fu_111_p2;
    wire    ap_loop_init;
    reg   [2:0] ap_sig_allocacmp_i;
    wire    ap_block_pp0_stage0_00001;
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
        #0 i_7_fu_36 = 3'd0;
        #0 ap_done_reg = 1'b0;
    end

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
            end else if (((1'b0 == ap_block_pp0_stage0_subdone) & (ap_loop_exit_ready_pp0_iter22_reg == 1'b1))) begin
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
            end else if (((1'b0 == ap_block_pp0_stage0_subdone) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
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
            ap_enable_reg_pp0_iter3 <= 1'b0;
        end else begin
            if ((1'b0 == ap_block_pp0_stage0_subdone)) begin
                ap_enable_reg_pp0_iter3 <= ap_enable_reg_pp0_iter2;
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
        if (((1'b0 == ap_block_pp0_stage0_11001) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
            if (((ap_enable_reg_pp0_iter0 == 1'b1) & (icmp_ln121_fu_105_p2 == 1'd0))) begin
                i_7_fu_36 <= add_ln121_fu_111_p2;
            end else if ((ap_loop_init == 1'b1)) begin
                i_7_fu_36 <= 3'd0;
            end
        end
    end

    always @(posedge ap_clk) begin
        if ((1'b0 == ap_block_pp0_stage0_11001)) begin
            add_reg_180 <= grp_fu_2537_p_dout0;
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
            ap_loop_exit_ready_pp0_iter3_reg <= ap_loop_exit_ready_pp0_iter2_reg;
            ap_loop_exit_ready_pp0_iter4_reg <= ap_loop_exit_ready_pp0_iter3_reg;
            ap_loop_exit_ready_pp0_iter5_reg <= ap_loop_exit_ready_pp0_iter4_reg;
            ap_loop_exit_ready_pp0_iter6_reg <= ap_loop_exit_ready_pp0_iter5_reg;
            ap_loop_exit_ready_pp0_iter7_reg <= ap_loop_exit_ready_pp0_iter6_reg;
            ap_loop_exit_ready_pp0_iter8_reg <= ap_loop_exit_ready_pp0_iter7_reg;
            ap_loop_exit_ready_pp0_iter9_reg <= ap_loop_exit_ready_pp0_iter8_reg;
            mul_reg_175 <= grp_fu_2533_p_dout0;
            qNear_load_reg_159_pp0_iter10_reg <= qNear_load_reg_159_pp0_iter9_reg;
            qNear_load_reg_159_pp0_iter11_reg <= qNear_load_reg_159_pp0_iter10_reg;
            qNear_load_reg_159_pp0_iter12_reg <= qNear_load_reg_159_pp0_iter11_reg;
            qNear_load_reg_159_pp0_iter13_reg <= qNear_load_reg_159_pp0_iter12_reg;
            qNear_load_reg_159_pp0_iter14_reg <= qNear_load_reg_159_pp0_iter13_reg;
            qNear_load_reg_159_pp0_iter15_reg <= qNear_load_reg_159_pp0_iter14_reg;
            qNear_load_reg_159_pp0_iter2_reg <= qNear_load_reg_159;
            qNear_load_reg_159_pp0_iter3_reg <= qNear_load_reg_159_pp0_iter2_reg;
            qNear_load_reg_159_pp0_iter4_reg <= qNear_load_reg_159_pp0_iter3_reg;
            qNear_load_reg_159_pp0_iter5_reg <= qNear_load_reg_159_pp0_iter4_reg;
            qNear_load_reg_159_pp0_iter6_reg <= qNear_load_reg_159_pp0_iter5_reg;
            qNear_load_reg_159_pp0_iter7_reg <= qNear_load_reg_159_pp0_iter6_reg;
            qNear_load_reg_159_pp0_iter8_reg <= qNear_load_reg_159_pp0_iter7_reg;
            qNear_load_reg_159_pp0_iter9_reg <= qNear_load_reg_159_pp0_iter8_reg;
            sub_reg_170 <= grp_fu_2529_p_dout0;
            zext_ln121_reg_144_pp0_iter10_reg[2 : 0] <= zext_ln121_reg_144_pp0_iter9_reg[2 : 0];
            zext_ln121_reg_144_pp0_iter11_reg[2 : 0] <= zext_ln121_reg_144_pp0_iter10_reg[2 : 0];
            zext_ln121_reg_144_pp0_iter12_reg[2 : 0] <= zext_ln121_reg_144_pp0_iter11_reg[2 : 0];
            zext_ln121_reg_144_pp0_iter13_reg[2 : 0] <= zext_ln121_reg_144_pp0_iter12_reg[2 : 0];
            zext_ln121_reg_144_pp0_iter14_reg[2 : 0] <= zext_ln121_reg_144_pp0_iter13_reg[2 : 0];
            zext_ln121_reg_144_pp0_iter15_reg[2 : 0] <= zext_ln121_reg_144_pp0_iter14_reg[2 : 0];
            zext_ln121_reg_144_pp0_iter16_reg[2 : 0] <= zext_ln121_reg_144_pp0_iter15_reg[2 : 0];
            zext_ln121_reg_144_pp0_iter17_reg[2 : 0] <= zext_ln121_reg_144_pp0_iter16_reg[2 : 0];
            zext_ln121_reg_144_pp0_iter18_reg[2 : 0] <= zext_ln121_reg_144_pp0_iter17_reg[2 : 0];
            zext_ln121_reg_144_pp0_iter19_reg[2 : 0] <= zext_ln121_reg_144_pp0_iter18_reg[2 : 0];
            zext_ln121_reg_144_pp0_iter20_reg[2 : 0] <= zext_ln121_reg_144_pp0_iter19_reg[2 : 0];
            zext_ln121_reg_144_pp0_iter21_reg[2 : 0] <= zext_ln121_reg_144_pp0_iter20_reg[2 : 0];
            zext_ln121_reg_144_pp0_iter22_reg[2 : 0] <= zext_ln121_reg_144_pp0_iter21_reg[2 : 0];
            zext_ln121_reg_144_pp0_iter2_reg[2 : 0] <= zext_ln121_reg_144_pp0_iter1_reg[2 : 0];
            zext_ln121_reg_144_pp0_iter3_reg[2 : 0] <= zext_ln121_reg_144_pp0_iter2_reg[2 : 0];
            zext_ln121_reg_144_pp0_iter4_reg[2 : 0] <= zext_ln121_reg_144_pp0_iter3_reg[2 : 0];
            zext_ln121_reg_144_pp0_iter5_reg[2 : 0] <= zext_ln121_reg_144_pp0_iter4_reg[2 : 0];
            zext_ln121_reg_144_pp0_iter6_reg[2 : 0] <= zext_ln121_reg_144_pp0_iter5_reg[2 : 0];
            zext_ln121_reg_144_pp0_iter7_reg[2 : 0] <= zext_ln121_reg_144_pp0_iter6_reg[2 : 0];
            zext_ln121_reg_144_pp0_iter8_reg[2 : 0] <= zext_ln121_reg_144_pp0_iter7_reg[2 : 0];
            zext_ln121_reg_144_pp0_iter9_reg[2 : 0] <= zext_ln121_reg_144_pp0_iter8_reg[2 : 0];
        end
    end

    always @(posedge ap_clk) begin
        if (((1'b0 == ap_block_pp0_stage0_11001) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
            ap_loop_exit_ready_pp0_iter1_reg <= ap_loop_exit_ready;
            ap_loop_exit_ready_pp0_iter2_reg <= ap_loop_exit_ready_pp0_iter1_reg;
            qNear_load_reg_159 <= qNear_q0;
            qRand_load_reg_165 <= qRand_q0;
            zext_ln121_reg_144[2 : 0] <= zext_ln121_fu_117_p1[2 : 0];
            zext_ln121_reg_144_pp0_iter1_reg[2 : 0] <= zext_ln121_reg_144[2 : 0];
        end
    end

    always @(*) begin
        if (((1'b0 == ap_block_pp0_stage0_subdone) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0) & (icmp_ln121_fu_105_p2 == 1'd1))) begin
            ap_condition_exit_pp0_iter0_stage0 = 1'b1;
        end else begin
            ap_condition_exit_pp0_iter0_stage0 = 1'b0;
        end
    end

    always @(*) begin
        if (((1'b0 == ap_block_pp0_stage0_subdone) & (ap_loop_exit_ready_pp0_iter22_reg == 1'b1))) begin
            ap_done_int = 1'b1;
        end else begin
            ap_done_int = ap_done_reg;
        end
    end

    always @(*) begin
        if (((ap_idle_pp0 == 1'b1) & (ap_start_int == 1'b0) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
            ap_idle = 1'b1;
        end else begin
            ap_idle = 1'b0;
        end
    end

    always @(*) begin
        if (((ap_enable_reg_pp0_iter23 == 1'b0) & (ap_enable_reg_pp0_iter22 == 1'b0) & (ap_enable_reg_pp0_iter21 == 1'b0) & (ap_enable_reg_pp0_iter20 == 1'b0) & (ap_enable_reg_pp0_iter19 == 1'b0) & (ap_enable_reg_pp0_iter18 == 1'b0) & (ap_enable_reg_pp0_iter17 == 1'b0) & (ap_enable_reg_pp0_iter16 == 1'b0) & (ap_enable_reg_pp0_iter15 == 1'b0) & (ap_enable_reg_pp0_iter14 == 1'b0) & (ap_enable_reg_pp0_iter13 == 1'b0) & (ap_enable_reg_pp0_iter12 == 1'b0) & (ap_enable_reg_pp0_iter11 == 1'b0) & (ap_enable_reg_pp0_iter10 == 1'b0) & (ap_enable_reg_pp0_iter9 == 1'b0) & (ap_enable_reg_pp0_iter8 == 1'b0) & (ap_enable_reg_pp0_iter7 == 1'b0) & (ap_enable_reg_pp0_iter6 == 1'b0) & (ap_enable_reg_pp0_iter5 == 1'b0) & (ap_enable_reg_pp0_iter4 == 1'b0) & (ap_enable_reg_pp0_iter3 == 1'b0) & (ap_enable_reg_pp0_iter2 == 1'b0) & (ap_enable_reg_pp0_iter1 == 1'b0) & (ap_enable_reg_pp0_iter0 == 1'b0))) begin
            ap_idle_pp0 = 1'b1;
        end else begin
            ap_idle_pp0 = 1'b0;
        end
    end

    always @(*) begin
        if (((1'b0 == ap_block_pp0_stage0_subdone) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
            ap_ready_int = 1'b1;
        end else begin
            ap_ready_int = 1'b0;
        end
    end

    always @(*) begin
        if (((1'b0 == ap_block_pp0_stage0) & (ap_loop_init == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
            ap_sig_allocacmp_i = 3'd0;
        end else begin
            ap_sig_allocacmp_i = i_7_fu_36;
        end
    end

    always @(*) begin
        if (((1'b0 == ap_block_pp0_stage0_11001) & (ap_enable_reg_pp0_iter23 == 1'b1))) begin
            qConnect_ce0 = 1'b1;
        end else begin
            qConnect_ce0 = 1'b0;
        end
    end

    always @(*) begin
        if (((1'b0 == ap_block_pp0_stage0_11001) & (ap_enable_reg_pp0_iter23 == 1'b1))) begin
            qConnect_we0 = 1'b1;
        end else begin
            qConnect_we0 = 1'b0;
        end
    end

    always @(*) begin
        if (((1'b0 == ap_block_pp0_stage0_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
            qNear_ce0 = 1'b1;
        end else begin
            qNear_ce0 = 1'b0;
        end
    end

    always @(*) begin
        if (((1'b0 == ap_block_pp0_stage0_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
            qRand_ce0 = 1'b1;
        end else begin
            qRand_ce0 = 1'b0;
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

    assign add_ln121_fu_111_p2 = (ap_sig_allocacmp_i + 3'd1);

    assign ap_CS_fsm_pp0_stage0 = ap_CS_fsm[32'd0];

    assign ap_block_pp0_stage0 = ~(1'b1 == 1'b1);

    assign ap_block_pp0_stage0_00001 = ~(1'b1 == 1'b1);

    assign ap_block_pp0_stage0_11001 = ~(1'b1 == 1'b1);

    assign ap_block_pp0_stage0_subdone = ~(1'b1 == 1'b1);

    assign ap_enable_pp0 = (ap_idle_pp0 ^ 1'b1);

    assign ap_enable_reg_pp0_iter0 = ap_start_int;

    assign ap_loop_exit_ready = ap_condition_exit_pp0_iter0_stage0;

    assign grp_fu_2529_p_ce = 1'b1;

    assign grp_fu_2529_p_din0 = qRand_load_reg_165;

    assign grp_fu_2529_p_din1 = qNear_load_reg_159;

    assign grp_fu_2529_p_opcode = 2'd1;

    assign grp_fu_2533_p_ce = 1'b1;

    assign grp_fu_2533_p_din0 = f;

    assign grp_fu_2533_p_din1 = sub_reg_170;

    assign grp_fu_2537_p_ce = 1'b1;

    assign grp_fu_2537_p_din0 = qNear_load_reg_159_pp0_iter15_reg;

    assign grp_fu_2537_p_din1 = mul_reg_175;

    assign grp_fu_2537_p_opcode = 2'd0;

    assign icmp_ln121_fu_105_p2 = ((ap_sig_allocacmp_i == 3'd6) ? 1'b1 : 1'b0);

    assign qConnect_address0 = zext_ln121_reg_144_pp0_iter22_reg;

    assign qConnect_d0 = add_reg_180;

    assign qNear_address0 = zext_ln121_fu_117_p1;

    assign qRand_address0 = zext_ln121_fu_117_p1;

    assign zext_ln121_fu_117_p1 = ap_sig_allocacmp_i;

    always @(posedge ap_clk) begin
        zext_ln121_reg_144[63:3] <= 61'b0000000000000000000000000000000000000000000000000000000000000;
        zext_ln121_reg_144_pp0_iter1_reg[63:3] <= 61'b0000000000000000000000000000000000000000000000000000000000000;
        zext_ln121_reg_144_pp0_iter2_reg[63:3] <= 61'b0000000000000000000000000000000000000000000000000000000000000;
        zext_ln121_reg_144_pp0_iter3_reg[63:3] <= 61'b0000000000000000000000000000000000000000000000000000000000000;
        zext_ln121_reg_144_pp0_iter4_reg[63:3] <= 61'b0000000000000000000000000000000000000000000000000000000000000;
        zext_ln121_reg_144_pp0_iter5_reg[63:3] <= 61'b0000000000000000000000000000000000000000000000000000000000000;
        zext_ln121_reg_144_pp0_iter6_reg[63:3] <= 61'b0000000000000000000000000000000000000000000000000000000000000;
        zext_ln121_reg_144_pp0_iter7_reg[63:3] <= 61'b0000000000000000000000000000000000000000000000000000000000000;
        zext_ln121_reg_144_pp0_iter8_reg[63:3] <= 61'b0000000000000000000000000000000000000000000000000000000000000;
        zext_ln121_reg_144_pp0_iter9_reg[63:3] <= 61'b0000000000000000000000000000000000000000000000000000000000000;
        zext_ln121_reg_144_pp0_iter10_reg[63:3] <= 61'b0000000000000000000000000000000000000000000000000000000000000;
        zext_ln121_reg_144_pp0_iter11_reg[63:3] <= 61'b0000000000000000000000000000000000000000000000000000000000000;
        zext_ln121_reg_144_pp0_iter12_reg[63:3] <= 61'b0000000000000000000000000000000000000000000000000000000000000;
        zext_ln121_reg_144_pp0_iter13_reg[63:3] <= 61'b0000000000000000000000000000000000000000000000000000000000000;
        zext_ln121_reg_144_pp0_iter14_reg[63:3] <= 61'b0000000000000000000000000000000000000000000000000000000000000;
        zext_ln121_reg_144_pp0_iter15_reg[63:3] <= 61'b0000000000000000000000000000000000000000000000000000000000000;
        zext_ln121_reg_144_pp0_iter16_reg[63:3] <= 61'b0000000000000000000000000000000000000000000000000000000000000;
        zext_ln121_reg_144_pp0_iter17_reg[63:3] <= 61'b0000000000000000000000000000000000000000000000000000000000000;
        zext_ln121_reg_144_pp0_iter18_reg[63:3] <= 61'b0000000000000000000000000000000000000000000000000000000000000;
        zext_ln121_reg_144_pp0_iter19_reg[63:3] <= 61'b0000000000000000000000000000000000000000000000000000000000000;
        zext_ln121_reg_144_pp0_iter20_reg[63:3] <= 61'b0000000000000000000000000000000000000000000000000000000000000;
        zext_ln121_reg_144_pp0_iter21_reg[63:3] <= 61'b0000000000000000000000000000000000000000000000000000000000000;
        zext_ln121_reg_144_pp0_iter22_reg[63:3] <= 61'b0000000000000000000000000000000000000000000000000000000000000;
    end

endmodule  //main_planRRT_Pipeline_VITIS_LOOP_121_6
