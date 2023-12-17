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

module main_planRRT_Pipeline_VITIS_LOOP_293_1 (
    ap_clk,
    ap_rst,
    ap_start,
    ap_done,
    ap_idle,
    ap_ready,
    rrtVertices_address0,
    rrtVertices_ce0,
    rrtVertices_q0,
    qRand_address0,
    qRand_ce0,
    qRand_q0,
    dist_out,
    dist_out_ap_vld,
    grp_fu_2529_p_din0,
    grp_fu_2529_p_din1,
    grp_fu_2529_p_opcode,
    grp_fu_2529_p_dout0,
    grp_fu_2529_p_ce,
    grp_fu_2533_p_din0,
    grp_fu_2533_p_din1,
    grp_fu_2533_p_dout0,
    grp_fu_2533_p_ce
);

    parameter ap_ST_fsm_pp0_stage0 = 7'd1;
    parameter ap_ST_fsm_pp0_stage1 = 7'd2;
    parameter ap_ST_fsm_pp0_stage2 = 7'd4;
    parameter ap_ST_fsm_pp0_stage3 = 7'd8;
    parameter ap_ST_fsm_pp0_stage4 = 7'd16;
    parameter ap_ST_fsm_pp0_stage5 = 7'd32;
    parameter ap_ST_fsm_pp0_stage6 = 7'd64;

    input ap_clk;
    input ap_rst;
    input ap_start;
    output ap_done;
    output ap_idle;
    output ap_ready;
    output [12:0] rrtVertices_address0;
    output rrtVertices_ce0;
    input [63:0] rrtVertices_q0;
    output [2:0] qRand_address0;
    output qRand_ce0;
    input [63:0] qRand_q0;
    output [63:0] dist_out;
    output dist_out_ap_vld;
    output [63:0] grp_fu_2529_p_din0;
    output [63:0] grp_fu_2529_p_din1;
    output [1:0] grp_fu_2529_p_opcode;
    input [63:0] grp_fu_2529_p_dout0;
    output grp_fu_2529_p_ce;
    output [63:0] grp_fu_2533_p_din0;
    output [63:0] grp_fu_2533_p_din1;
    input [63:0] grp_fu_2533_p_dout0;
    output grp_fu_2533_p_ce;

    reg ap_idle;
    reg rrtVertices_ce0;
    reg qRand_ce0;
    reg dist_out_ap_vld;

    (* fsm_encoding = "none" *) reg   [6:0] ap_CS_fsm;
    wire    ap_CS_fsm_pp0_stage0;
    reg    ap_enable_reg_pp0_iter0;
    reg    ap_enable_reg_pp0_iter1;
    reg    ap_enable_reg_pp0_iter2;
    reg    ap_enable_reg_pp0_iter3;
    reg    ap_idle_pp0;
    wire    ap_CS_fsm_pp0_stage6;
    wire    ap_block_pp0_stage6_subdone;
    reg   [0:0] icmp_ln293_reg_148;
    reg    ap_condition_exit_pp0_iter0_stage6;
    wire    ap_loop_exit_ready;
    reg    ap_ready_int;
    wire    ap_block_pp0_stage0_11001;
    wire   [0:0] icmp_ln293_fu_98_p2;
    reg   [0:0] icmp_ln293_reg_148_pp0_iter1_reg;
    reg   [0:0] icmp_ln293_reg_148_pp0_iter2_reg;
    reg   [63:0] rrtVertices_load_reg_162;
    wire    ap_CS_fsm_pp0_stage1;
    wire    ap_block_pp0_stage1_11001;
    reg   [63:0] qRand_load_reg_167;
    reg   [63:0] sub_i_i_reg_172;
    reg   [63:0] mul_i_i_reg_178;
    wire    ap_CS_fsm_pp0_stage2;
    wire    ap_block_pp0_stage2_11001;
    wire    ap_CS_fsm_pp0_stage3;
    wire    ap_block_pp0_stage3_11001;
    reg   [63:0] dist_1_reg_188;
    reg    ap_enable_reg_pp0_iter0_reg;
    wire    ap_block_pp0_stage3_subdone;
    wire   [63:0] zext_ln293_fu_110_p1;
    wire    ap_block_pp0_stage0;
    reg   [63:0] dist_fu_36;
    reg   [63:0] ap_sig_allocacmp_dist_load;
    wire    ap_block_pp0_stage3;
    wire    ap_loop_init;
    reg   [2:0] i_3_fu_40;
    wire   [2:0] add_ln293_fu_104_p2;
    reg   [2:0] ap_sig_allocacmp_i;
    wire    ap_block_pp0_stage3_01001;
    reg   [63:0] grp_fu_77_p0;
    reg   [63:0] grp_fu_77_p1;
    wire    ap_block_pp0_stage2;
    reg   [1:0] grp_fu_77_opcode;
    wire    ap_block_pp0_stage3_00001;
    wire    ap_block_pp0_stage2_00001;
    reg    ap_done_reg;
    wire    ap_continue_int;
    reg    ap_done_int;
    reg    ap_loop_exit_ready_pp0_iter1_reg;
    wire    ap_block_pp0_stage6_11001;
    reg    ap_condition_exit_pp0_iter2_stage3;
    reg    ap_idle_pp0_0to1;
    reg    ap_loop_exit_ready_pp0_iter2_reg;
    reg   [6:0] ap_NS_fsm;
    wire    ap_block_pp0_stage0_subdone;
    reg    ap_idle_pp0_1to3;
    wire    ap_block_pp0_stage1_subdone;
    wire    ap_block_pp0_stage2_subdone;
    wire    ap_block_pp0_stage4_subdone;
    wire    ap_block_pp0_stage5_subdone;
    wire    ap_enable_pp0;
    wire    ap_start_int;
    wire    ap_ce_reg;

    // power-on initialization
    initial begin
        #0 ap_CS_fsm = 7'd1;
        #0 ap_enable_reg_pp0_iter1 = 1'b0;
        #0 ap_enable_reg_pp0_iter2 = 1'b0;
        #0 ap_enable_reg_pp0_iter3 = 1'b0;
        #0 ap_enable_reg_pp0_iter0_reg = 1'b0;
        #0 dist_fu_36 = 64'd0;
        #0 i_3_fu_40 = 3'd0;
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
        .ap_loop_exit_ready(ap_condition_exit_pp0_iter0_stage6),
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
            end else if (((1'b0 == ap_block_pp0_stage3_subdone) & (ap_loop_exit_ready_pp0_iter2_reg == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage3))) begin
                ap_done_reg <= 1'b1;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (ap_rst == 1'b1) begin
            ap_enable_reg_pp0_iter0_reg <= 1'b0;
        end else begin
            if ((1'b1 == ap_CS_fsm_pp0_stage0)) begin
                ap_enable_reg_pp0_iter0_reg <= ap_start_int;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (ap_rst == 1'b1) begin
            ap_enable_reg_pp0_iter1 <= 1'b0;
        end else begin
            if ((1'b1 == ap_condition_exit_pp0_iter0_stage6)) begin
                ap_enable_reg_pp0_iter1 <= 1'b0;
            end else if (((1'b0 == ap_block_pp0_stage6_subdone) & (1'b1 == ap_CS_fsm_pp0_stage6))) begin
                ap_enable_reg_pp0_iter1 <= ap_enable_reg_pp0_iter0;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (ap_rst == 1'b1) begin
            ap_enable_reg_pp0_iter2 <= 1'b0;
        end else begin
            if (((1'b0 == ap_block_pp0_stage6_subdone) & (1'b1 == ap_CS_fsm_pp0_stage6))) begin
                ap_enable_reg_pp0_iter2 <= ap_enable_reg_pp0_iter1;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (ap_rst == 1'b1) begin
            ap_enable_reg_pp0_iter3 <= 1'b0;
        end else begin
            if (((1'b0 == ap_block_pp0_stage3_subdone) & (ap_enable_reg_pp0_iter3 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage3))) begin
                ap_enable_reg_pp0_iter3 <= 1'b0;
            end else if (((1'b0 == ap_block_pp0_stage6_subdone) & (1'b1 == ap_CS_fsm_pp0_stage6))) begin
                ap_enable_reg_pp0_iter3 <= ap_enable_reg_pp0_iter2;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (((ap_idle_pp0_0to1 == 1'b1) & (1'b1 == ap_condition_exit_pp0_iter2_stage3))) begin
            ap_loop_exit_ready_pp0_iter1_reg <= 1'b0;
        end else if (((1'b0 == ap_block_pp0_stage6_11001) & (1'b1 == ap_CS_fsm_pp0_stage6))) begin
            ap_loop_exit_ready_pp0_iter1_reg <= ap_loop_exit_ready;
        end
    end

    always @(posedge ap_clk) begin
        if (((ap_idle_pp0_0to1 == 1'b1) & (1'b1 == ap_condition_exit_pp0_iter2_stage3))) begin
            ap_loop_exit_ready_pp0_iter2_reg <= 1'b0;
        end else if (((1'b0 == ap_block_pp0_stage6_11001) & (1'b1 == ap_CS_fsm_pp0_stage6))) begin
            ap_loop_exit_ready_pp0_iter2_reg <= ap_loop_exit_ready_pp0_iter1_reg;
        end
    end

    always @(posedge ap_clk) begin
        if (((1'b0 == ap_block_pp0_stage0_11001) & (1'b1 == ap_CS_fsm_pp0_stage0) & (ap_loop_init == 1'b1))) begin
            dist_fu_36 <= 64'd0;
        end else if (((1'b0 == ap_block_pp0_stage3_11001) & (ap_enable_reg_pp0_iter3 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage3))) begin
            dist_fu_36 <= dist_1_reg_188;
        end
    end

    always @(posedge ap_clk) begin
        if (((1'b0 == ap_block_pp0_stage0_11001) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
            if (((icmp_ln293_fu_98_p2 == 1'd0) & (ap_enable_reg_pp0_iter0 == 1'b1))) begin
                i_3_fu_40 <= add_ln293_fu_104_p2;
            end else if ((ap_loop_init == 1'b1)) begin
                i_3_fu_40 <= 3'd0;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (((1'b0 == ap_block_pp0_stage2_11001) & (ap_enable_reg_pp0_iter3 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage2))) begin
            dist_1_reg_188 <= grp_fu_2529_p_dout0;
        end
    end

    always @(posedge ap_clk) begin
        if (((1'b0 == ap_block_pp0_stage0_11001) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
            icmp_ln293_reg_148 <= icmp_ln293_fu_98_p2;
            icmp_ln293_reg_148_pp0_iter1_reg <= icmp_ln293_reg_148;
            icmp_ln293_reg_148_pp0_iter2_reg <= icmp_ln293_reg_148_pp0_iter1_reg;
        end
    end

    always @(posedge ap_clk) begin
        if (((1'b0 == ap_block_pp0_stage2_11001) & (1'b1 == ap_CS_fsm_pp0_stage2))) begin
            mul_i_i_reg_178 <= grp_fu_2533_p_dout0;
        end
    end

    always @(posedge ap_clk) begin
        if (((1'b0 == ap_block_pp0_stage1_11001) & (1'b1 == ap_CS_fsm_pp0_stage1))) begin
            qRand_load_reg_167 <= qRand_q0;
            rrtVertices_load_reg_162 <= rrtVertices_q0;
        end
    end

    always @(posedge ap_clk) begin
        if (((1'b0 == ap_block_pp0_stage1_11001) & (ap_enable_reg_pp0_iter1 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage1))) begin
            sub_i_i_reg_172 <= grp_fu_2529_p_dout0;
        end
    end

    always @(*) begin
        if (((icmp_ln293_reg_148 == 1'd1) & (1'b0 == ap_block_pp0_stage6_subdone) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage6))) begin
            ap_condition_exit_pp0_iter0_stage6 = 1'b1;
        end else begin
            ap_condition_exit_pp0_iter0_stage6 = 1'b0;
        end
    end

    always @(*) begin
        if (((icmp_ln293_reg_148_pp0_iter2_reg == 1'd1) & (1'b0 == ap_block_pp0_stage3_subdone) & (ap_enable_reg_pp0_iter2 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage3))) begin
            ap_condition_exit_pp0_iter2_stage3 = 1'b1;
        end else begin
            ap_condition_exit_pp0_iter2_stage3 = 1'b0;
        end
    end

    always @(*) begin
        if (((1'b0 == ap_block_pp0_stage3_subdone) & (ap_loop_exit_ready_pp0_iter2_reg == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage3))) begin
            ap_done_int = 1'b1;
        end else begin
            ap_done_int = ap_done_reg;
        end
    end

    always @(*) begin
        if ((1'b1 == ap_CS_fsm_pp0_stage0)) begin
            ap_enable_reg_pp0_iter0 = ap_start_int;
        end else begin
            ap_enable_reg_pp0_iter0 = ap_enable_reg_pp0_iter0_reg;
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
        if (((ap_enable_reg_pp0_iter3 == 1'b0) & (ap_enable_reg_pp0_iter2 == 1'b0) & (ap_enable_reg_pp0_iter1 == 1'b0) & (ap_enable_reg_pp0_iter0 == 1'b0))) begin
            ap_idle_pp0 = 1'b1;
        end else begin
            ap_idle_pp0 = 1'b0;
        end
    end

    always @(*) begin
        if (((ap_enable_reg_pp0_iter1 == 1'b0) & (ap_enable_reg_pp0_iter0 == 1'b0))) begin
            ap_idle_pp0_0to1 = 1'b1;
        end else begin
            ap_idle_pp0_0to1 = 1'b0;
        end
    end

    always @(*) begin
        if (((ap_enable_reg_pp0_iter3 == 1'b0) & (ap_enable_reg_pp0_iter2 == 1'b0) & (ap_enable_reg_pp0_iter1 == 1'b0))) begin
            ap_idle_pp0_1to3 = 1'b1;
        end else begin
            ap_idle_pp0_1to3 = 1'b0;
        end
    end

    always @(*) begin
        if (((1'b0 == ap_block_pp0_stage6_subdone) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage6))) begin
            ap_ready_int = 1'b1;
        end else begin
            ap_ready_int = 1'b0;
        end
    end

    always @(*) begin
        if (((1'b0 == ap_block_pp0_stage3) & (ap_enable_reg_pp0_iter3 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage3))) begin
            ap_sig_allocacmp_dist_load = dist_1_reg_188;
        end else begin
            ap_sig_allocacmp_dist_load = dist_fu_36;
        end
    end

    always @(*) begin
        if (((1'b0 == ap_block_pp0_stage0) & (1'b1 == ap_CS_fsm_pp0_stage0) & (ap_loop_init == 1'b1))) begin
            ap_sig_allocacmp_i = 3'd0;
        end else begin
            ap_sig_allocacmp_i = i_3_fu_40;
        end
    end

    always @(*) begin
        if (((icmp_ln293_reg_148_pp0_iter2_reg == 1'd1) & (1'b0 == ap_block_pp0_stage3_11001) & (1'b1 == ap_CS_fsm_pp0_stage3))) begin
            dist_out_ap_vld = 1'b1;
        end else begin
            dist_out_ap_vld = 1'b0;
        end
    end

    always @(*) begin
        if (((icmp_ln293_reg_148 == 1'd0) & (1'b0 == ap_block_pp0_stage2_00001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage2))) begin
            grp_fu_77_opcode = 2'd1;
        end else if (((1'b0 == ap_block_pp0_stage3_00001) & (ap_enable_reg_pp0_iter2 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage3))) begin
            grp_fu_77_opcode = 2'd0;
        end else begin
            grp_fu_77_opcode = 'bx;
        end
    end

    always @(*) begin
        if (((1'b0 == ap_block_pp0_stage3) & (ap_enable_reg_pp0_iter2 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage3))) begin
            grp_fu_77_p0 = ap_sig_allocacmp_dist_load;
        end else if (((1'b0 == ap_block_pp0_stage2) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage2))) begin
            grp_fu_77_p0 = rrtVertices_load_reg_162;
        end else begin
            grp_fu_77_p0 = 'bx;
        end
    end

    always @(*) begin
        if (((1'b0 == ap_block_pp0_stage3) & (ap_enable_reg_pp0_iter2 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage3))) begin
            grp_fu_77_p1 = mul_i_i_reg_178;
        end else if (((1'b0 == ap_block_pp0_stage2) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage2))) begin
            grp_fu_77_p1 = qRand_load_reg_167;
        end else begin
            grp_fu_77_p1 = 'bx;
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
        if (((1'b0 == ap_block_pp0_stage0_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
            rrtVertices_ce0 = 1'b1;
        end else begin
            rrtVertices_ce0 = 1'b0;
        end
    end

    always @(*) begin
        case (ap_CS_fsm)
            ap_ST_fsm_pp0_stage0: begin
                if ((~((ap_start_int == 1'b0) & (ap_idle_pp0_1to3 == 1'b1)) & (1'b0 == ap_block_pp0_stage0_subdone))) begin
                    ap_NS_fsm = ap_ST_fsm_pp0_stage1;
                end else begin
                    ap_NS_fsm = ap_ST_fsm_pp0_stage0;
                end
            end
            ap_ST_fsm_pp0_stage1: begin
                if ((1'b0 == ap_block_pp0_stage1_subdone)) begin
                    ap_NS_fsm = ap_ST_fsm_pp0_stage2;
                end else begin
                    ap_NS_fsm = ap_ST_fsm_pp0_stage1;
                end
            end
            ap_ST_fsm_pp0_stage2: begin
                if ((1'b0 == ap_block_pp0_stage2_subdone)) begin
                    ap_NS_fsm = ap_ST_fsm_pp0_stage3;
                end else begin
                    ap_NS_fsm = ap_ST_fsm_pp0_stage2;
                end
            end
            ap_ST_fsm_pp0_stage3: begin
                if (((ap_idle_pp0_0to1 == 1'b1) & (1'b1 == ap_condition_exit_pp0_iter2_stage3))) begin
                    ap_NS_fsm = ap_ST_fsm_pp0_stage0;
                end else if ((1'b0 == ap_block_pp0_stage3_subdone)) begin
                    ap_NS_fsm = ap_ST_fsm_pp0_stage4;
                end else begin
                    ap_NS_fsm = ap_ST_fsm_pp0_stage3;
                end
            end
            ap_ST_fsm_pp0_stage4: begin
                if ((1'b0 == ap_block_pp0_stage4_subdone)) begin
                    ap_NS_fsm = ap_ST_fsm_pp0_stage5;
                end else begin
                    ap_NS_fsm = ap_ST_fsm_pp0_stage4;
                end
            end
            ap_ST_fsm_pp0_stage5: begin
                if ((1'b0 == ap_block_pp0_stage5_subdone)) begin
                    ap_NS_fsm = ap_ST_fsm_pp0_stage6;
                end else begin
                    ap_NS_fsm = ap_ST_fsm_pp0_stage5;
                end
            end
            ap_ST_fsm_pp0_stage6: begin
                if ((1'b0 == ap_block_pp0_stage6_subdone)) begin
                    ap_NS_fsm = ap_ST_fsm_pp0_stage0;
                end else begin
                    ap_NS_fsm = ap_ST_fsm_pp0_stage6;
                end
            end
            default: begin
                ap_NS_fsm = 'bx;
            end
        endcase
    end

    assign add_ln293_fu_104_p2 = (ap_sig_allocacmp_i + 3'd1);

    assign ap_CS_fsm_pp0_stage0 = ap_CS_fsm[32'd0];

    assign ap_CS_fsm_pp0_stage1 = ap_CS_fsm[32'd1];

    assign ap_CS_fsm_pp0_stage2 = ap_CS_fsm[32'd2];

    assign ap_CS_fsm_pp0_stage3 = ap_CS_fsm[32'd3];

    assign ap_CS_fsm_pp0_stage6 = ap_CS_fsm[32'd6];

    assign ap_block_pp0_stage0 = ~(1'b1 == 1'b1);

    assign ap_block_pp0_stage0_11001 = ~(1'b1 == 1'b1);

    assign ap_block_pp0_stage0_subdone = ~(1'b1 == 1'b1);

    assign ap_block_pp0_stage1_11001 = ~(1'b1 == 1'b1);

    assign ap_block_pp0_stage1_subdone = ~(1'b1 == 1'b1);

    assign ap_block_pp0_stage2 = ~(1'b1 == 1'b1);

    assign ap_block_pp0_stage2_00001 = ~(1'b1 == 1'b1);

    assign ap_block_pp0_stage2_11001 = ~(1'b1 == 1'b1);

    assign ap_block_pp0_stage2_subdone = ~(1'b1 == 1'b1);

    assign ap_block_pp0_stage3 = ~(1'b1 == 1'b1);

    assign ap_block_pp0_stage3_00001 = ~(1'b1 == 1'b1);

    assign ap_block_pp0_stage3_01001 = ~(1'b1 == 1'b1);

    assign ap_block_pp0_stage3_11001 = ~(1'b1 == 1'b1);

    assign ap_block_pp0_stage3_subdone = ~(1'b1 == 1'b1);

    assign ap_block_pp0_stage4_subdone = ~(1'b1 == 1'b1);

    assign ap_block_pp0_stage5_subdone = ~(1'b1 == 1'b1);

    assign ap_block_pp0_stage6_11001 = ~(1'b1 == 1'b1);

    assign ap_block_pp0_stage6_subdone = ~(1'b1 == 1'b1);

    assign ap_enable_pp0 = (ap_idle_pp0 ^ 1'b1);

    assign ap_loop_exit_ready = ap_condition_exit_pp0_iter0_stage6;

    assign dist_out = dist_fu_36;

    assign grp_fu_2529_p_ce = 1'b1;

    assign grp_fu_2529_p_din0 = grp_fu_77_p0;

    assign grp_fu_2529_p_din1 = grp_fu_77_p1;

    assign grp_fu_2529_p_opcode = grp_fu_77_opcode;

    assign grp_fu_2533_p_ce = 1'b1;

    assign grp_fu_2533_p_din0 = sub_i_i_reg_172;

    assign grp_fu_2533_p_din1 = sub_i_i_reg_172;

    assign icmp_ln293_fu_98_p2 = ((ap_sig_allocacmp_i == 3'd6) ? 1'b1 : 1'b0);

    assign qRand_address0 = zext_ln293_fu_110_p1;

    assign rrtVertices_address0 = zext_ln293_fu_110_p1;

    assign zext_ln293_fu_110_p1 = ap_sig_allocacmp_i;

endmodule  //main_planRRT_Pipeline_VITIS_LOOP_293_1
