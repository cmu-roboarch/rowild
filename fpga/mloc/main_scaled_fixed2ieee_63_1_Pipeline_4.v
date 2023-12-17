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

module main_scaled_fixed2ieee_63_1_Pipeline_4 (
    ap_clk,
    ap_rst,
    ap_start,
    ap_done,
    ap_idle,
    ap_ready,
    in_val,
    c_0_03_reload,
    c_1_04_reload,
    c_2_05_reload,
    c_3_02_reload,
    shift_out,
    shift_out_ap_vld,
    in_shift_out,
    in_shift_out_ap_vld,
    shift_1_out,
    shift_1_out_ap_vld,
    in_shift_1_out,
    in_shift_1_out_ap_vld,
    ap_return
);

    parameter ap_ST_fsm_pp0_stage0 = 1'd1;

    input ap_clk;
    input ap_rst;
    input ap_start;
    output ap_done;
    output ap_idle;
    output ap_ready;
    input [62:0] in_val;
    input [31:0] c_0_03_reload;
    input [31:0] c_1_04_reload;
    input [31:0] c_2_05_reload;
    input [31:0] c_3_02_reload;
    output [31:0] shift_out;
    output shift_out_ap_vld;
    output [61:0] in_shift_out;
    output in_shift_out_ap_vld;
    output [31:0] shift_1_out;
    output shift_1_out_ap_vld;
    output [61:0] in_shift_1_out;
    output in_shift_1_out_ap_vld;
    output [0:0] ap_return;

    reg ap_idle;
    reg[31:0] shift_out;
    reg shift_out_ap_vld;
    reg in_shift_out_ap_vld;
    reg shift_1_out_ap_vld;
    reg in_shift_1_out_ap_vld;
    reg[0:0] ap_return;

    (* fsm_encoding = "none" *) reg   [0:0] ap_CS_fsm;
    wire    ap_CS_fsm_pp0_stage0;
    wire    ap_enable_reg_pp0_iter0;
    reg    ap_enable_reg_pp0_iter1;
    reg    ap_enable_reg_pp0_iter2;
    reg    ap_idle_pp0;
    wire    ap_block_pp0_stage0_subdone;
    wire   [0:0] icmp_ln424_fu_205_p2;
    wire   [0:0] icmp_ln421_fu_161_p2;
    reg    ap_condition_exit_pp0_iter0_stage0;
    wire    ap_loop_exit_ready;
    reg    ap_ready_int;
    wire    ap_block_pp0_stage0_11001;
    reg   [0:0] icmp_ln421_reg_302;
    wire   [31:0] tmp_2_fu_177_p6;
    reg   [31:0] tmp_2_reg_306;
    reg   [0:0] tmp_reg_312;
    wire   [31:0] sub_ln423_fu_199_p2;
    reg   [31:0] sub_ln423_reg_318;
    reg   [0:0] icmp_ln424_reg_323;
    wire   [62:0] in_shift_3_fu_255_p3;
    reg   [62:0] in_shift_3_reg_327;
    reg    ap_condition_exit_pp0_iter1_stage0;
    reg   [0:0] ap_phi_mux_UnifiedRetVal_phi_fu_135_p4;
    wire    ap_block_pp0_stage0;
    reg   [62:0] in_shift_fu_62;
    reg   [62:0] ap_sig_allocacmp_in_shift_2;
    wire    ap_loop_init;
    reg   [31:0] shift_fu_66;
    wire   [31:0] shift_1_fu_228_p2;
    wire   [31:0] shift_load_1_load_fu_276_p1;
    reg   [2:0] i_fu_70;
    wire   [2:0] add_ln421_fu_167_p2;
    reg   [2:0] ap_sig_allocacmp_i_3;
    wire    ap_block_pp0_stage0_01001;
    wire   [1:0] tmp_2_fu_177_p5;
    wire   [31:0] select_ln423_fu_234_p3;
    wire   [62:0] zext_ln423_fu_239_p1;
    wire   [62:0] ashr_ln423_fu_243_p2;
    wire   [62:0] shl_ln423_fu_249_p2;
    reg   [0:0] ap_return_preg;
    reg    ap_done_reg;
    wire    ap_continue_int;
    reg    ap_done_int;
    reg    ap_loop_exit_ready_pp0_iter1_reg;
    reg   [0:0] ap_NS_fsm;
    wire    ap_enable_pp0;
    wire    ap_start_int;
    reg    ap_condition_102;
    reg    ap_condition_258;
    reg    ap_condition_261;
    wire    ap_ce_reg;

    // power-on initialization
    initial begin
        #0 ap_CS_fsm = 1'd1;
        #0 ap_enable_reg_pp0_iter1 = 1'b0;
        #0 ap_enable_reg_pp0_iter2 = 1'b0;
        #0 in_shift_fu_62 = 63'd0;
        #0 shift_fu_66 = 32'd0;
        #0 i_fu_70 = 3'd0;
        #0 ap_return_preg = 1'd0;
        #0 ap_done_reg = 1'b0;
    end

    main_mux_4_2_32_1_1 #(
        .ID(1),
        .NUM_STAGE(1),
        .din0_WIDTH(32),
        .din1_WIDTH(32),
        .din2_WIDTH(32),
        .din3_WIDTH(32),
        .din4_WIDTH(2),
        .dout_WIDTH(32)
    ) mux_4_2_32_1_1_U43 (
        .din0(c_0_03_reload),
        .din1(c_1_04_reload),
        .din2(c_2_05_reload),
        .din3(c_3_02_reload),
        .din4(tmp_2_fu_177_p5),
        .dout(tmp_2_fu_177_p6)
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
            end else if (((1'b0 == ap_block_pp0_stage0_subdone) & (ap_loop_exit_ready_pp0_iter1_reg == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
                ap_done_reg <= 1'b1;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (ap_rst == 1'b1) begin
            ap_enable_reg_pp0_iter1 <= 1'b0;
        end else begin
            if (((1'b0 == ap_block_pp0_stage0_subdone) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
                ap_enable_reg_pp0_iter1 <= ap_start_int;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (ap_rst == 1'b1) begin
            ap_enable_reg_pp0_iter2 <= 1'b0;
        end else begin
            if ((1'b1 == ap_condition_exit_pp0_iter1_stage0)) begin
                ap_enable_reg_pp0_iter2 <= 1'b0;
            end else if ((1'b0 == ap_block_pp0_stage0_subdone)) begin
                ap_enable_reg_pp0_iter2 <= ap_enable_reg_pp0_iter1;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (ap_rst == 1'b1) begin
            ap_return_preg <= 1'd0;
        end else begin
            if (((1'b0 == ap_block_pp0_stage0_11001) & (1'b1 == ap_CS_fsm_pp0_stage0) & ((icmp_ln424_reg_323 == 1'd0) | (icmp_ln421_reg_302 == 1'd1)))) begin
                ap_return_preg <= ap_phi_mux_UnifiedRetVal_phi_fu_135_p4;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (((1'b0 == ap_block_pp0_stage0_11001) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
            if ((1'b1 == ap_condition_258)) begin
                i_fu_70 <= add_ln421_fu_167_p2;
            end else if ((ap_loop_init == 1'b1)) begin
                i_fu_70 <= 3'd0;
            end
        end
    end

    always @(posedge ap_clk) begin
        if ((1'b0 == ap_block_pp0_stage0_11001)) begin
            if (((1'b1 == ap_CS_fsm_pp0_stage0) & (ap_loop_init == 1'b1))) begin
                in_shift_fu_62 <= in_val;
            end else if ((ap_enable_reg_pp0_iter2 == 1'b1)) begin
                in_shift_fu_62 <= in_shift_3_reg_327;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (((1'b0 == ap_block_pp0_stage0_11001) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
            if ((ap_loop_init == 1'b1)) begin
                shift_fu_66 <= 32'd0;
            end else if ((1'b1 == ap_condition_261)) begin
                shift_fu_66 <= shift_1_fu_228_p2;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (((1'b0 == ap_block_pp0_stage0_11001) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
            ap_loop_exit_ready_pp0_iter1_reg <= ap_loop_exit_ready;
            icmp_ln421_reg_302 <= icmp_ln421_fu_161_p2;
            icmp_ln424_reg_323 <= icmp_ln424_fu_205_p2;
            in_shift_3_reg_327 <= in_shift_3_fu_255_p3;
            sub_ln423_reg_318 <= sub_ln423_fu_199_p2;
            tmp_2_reg_306 <= tmp_2_fu_177_p6;
            tmp_reg_312 <= tmp_2_fu_177_p6[32'd31];
        end
    end

    always @(*) begin
        if (((1'b0 == ap_block_pp0_stage0_subdone) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0) & ((icmp_ln421_fu_161_p2 == 1'd1) | (icmp_ln424_fu_205_p2 == 1'd0)))) begin
            ap_condition_exit_pp0_iter0_stage0 = 1'b1;
        end else begin
            ap_condition_exit_pp0_iter0_stage0 = 1'b0;
        end
    end

    always @(*) begin
        if (((1'b0 == ap_block_pp0_stage0_subdone) & (ap_enable_reg_pp0_iter1 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0) & ((icmp_ln424_reg_323 == 1'd0) | (icmp_ln421_reg_302 == 1'd1)))) begin
            ap_condition_exit_pp0_iter1_stage0 = 1'b1;
        end else begin
            ap_condition_exit_pp0_iter1_stage0 = 1'b0;
        end
    end

    always @(*) begin
        if (((1'b0 == ap_block_pp0_stage0_subdone) & (ap_loop_exit_ready_pp0_iter1_reg == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
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
        if (((ap_enable_reg_pp0_iter2 == 1'b0) & (ap_enable_reg_pp0_iter1 == 1'b0) & (ap_enable_reg_pp0_iter0 == 1'b0))) begin
            ap_idle_pp0 = 1'b1;
        end else begin
            ap_idle_pp0 = 1'b0;
        end
    end

    always @(*) begin
        if ((1'b1 == ap_condition_102)) begin
            if (((icmp_ln424_reg_323 == 1'd0) & (icmp_ln421_reg_302 == 1'd0))) begin
                ap_phi_mux_UnifiedRetVal_phi_fu_135_p4 = 1'd0;
            end else if ((icmp_ln421_reg_302 == 1'd1)) begin
                ap_phi_mux_UnifiedRetVal_phi_fu_135_p4 = 1'd1;
            end else begin
                ap_phi_mux_UnifiedRetVal_phi_fu_135_p4 = 'bx;
            end
        end else begin
            ap_phi_mux_UnifiedRetVal_phi_fu_135_p4 = 'bx;
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
        if (((1'b0 == ap_block_pp0_stage0_11001) & (1'b1 == ap_CS_fsm_pp0_stage0) & ((icmp_ln424_reg_323 == 1'd0) | (icmp_ln421_reg_302 == 1'd1)))) begin
            ap_return = ap_phi_mux_UnifiedRetVal_phi_fu_135_p4;
        end else begin
            ap_return = ap_return_preg;
        end
    end

    always @(*) begin
        if (((1'b0 == ap_block_pp0_stage0) & (1'b1 == ap_CS_fsm_pp0_stage0) & (ap_loop_init == 1'b1))) begin
            ap_sig_allocacmp_i_3 = 3'd0;
        end else begin
            ap_sig_allocacmp_i_3 = i_fu_70;
        end
    end

    always @(*) begin
        if (((1'b0 == ap_block_pp0_stage0) & (ap_enable_reg_pp0_iter2 == 1'b1))) begin
            ap_sig_allocacmp_in_shift_2 = in_shift_3_reg_327;
        end else begin
            ap_sig_allocacmp_in_shift_2 = in_shift_fu_62;
        end
    end

    always @(*) begin
        if (((icmp_ln424_reg_323 == 1'd0) & (icmp_ln421_reg_302 == 1'd0) & (1'b0 == ap_block_pp0_stage0_11001) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
            in_shift_1_out_ap_vld = 1'b1;
        end else begin
            in_shift_1_out_ap_vld = 1'b0;
        end
    end

    always @(*) begin
        if ((((icmp_ln421_reg_302 == 1'd1) & (1'b0 == ap_block_pp0_stage0_11001) & (1'b1 == ap_CS_fsm_pp0_stage0)) | ((icmp_ln424_reg_323 == 1'd0) & (icmp_ln421_reg_302 == 1'd0) & (1'b0 == ap_block_pp0_stage0_11001) & (1'b1 == ap_CS_fsm_pp0_stage0)))) begin
            in_shift_out_ap_vld = 1'b1;
        end else begin
            in_shift_out_ap_vld = 1'b0;
        end
    end

    always @(*) begin
        if (((icmp_ln424_reg_323 == 1'd0) & (icmp_ln421_reg_302 == 1'd0) & (1'b0 == ap_block_pp0_stage0_11001) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
            shift_1_out_ap_vld = 1'b1;
        end else begin
            shift_1_out_ap_vld = 1'b0;
        end
    end

    always @(*) begin
        if (((1'b0 == ap_block_pp0_stage0_01001) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
            if ((icmp_ln421_reg_302 == 1'd1)) begin
                shift_out = shift_load_1_load_fu_276_p1;
            end else if (((icmp_ln424_reg_323 == 1'd0) & (icmp_ln421_reg_302 == 1'd0))) begin
                shift_out = shift_fu_66;
            end else begin
                shift_out = 'bx;
            end
        end else begin
            shift_out = 'bx;
        end
    end

    always @(*) begin
        if ((((icmp_ln421_reg_302 == 1'd1) & (1'b0 == ap_block_pp0_stage0_11001) & (1'b1 == ap_CS_fsm_pp0_stage0)) | ((icmp_ln424_reg_323 == 1'd0) & (icmp_ln421_reg_302 == 1'd0) & (1'b0 == ap_block_pp0_stage0_11001) & (1'b1 == ap_CS_fsm_pp0_stage0)))) begin
            shift_out_ap_vld = 1'b1;
        end else begin
            shift_out_ap_vld = 1'b0;
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

    assign add_ln421_fu_167_p2 = (ap_sig_allocacmp_i_3 + 3'd1);

    assign ap_CS_fsm_pp0_stage0 = ap_CS_fsm[32'd0];

    assign ap_block_pp0_stage0 = ~(1'b1 == 1'b1);

    assign ap_block_pp0_stage0_01001 = ~(1'b1 == 1'b1);

    assign ap_block_pp0_stage0_11001 = ~(1'b1 == 1'b1);

    assign ap_block_pp0_stage0_subdone = ~(1'b1 == 1'b1);

    always @(*) begin
        ap_condition_102 = ((1'b0 == ap_block_pp0_stage0) & (ap_enable_reg_pp0_iter1 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0));
    end

    always @(*) begin
        ap_condition_258 = ((icmp_ln421_fu_161_p2 == 1'd0) & (icmp_ln424_fu_205_p2 == 1'd1) & (ap_enable_reg_pp0_iter0 == 1'b1));
    end

    always @(*) begin
        ap_condition_261 = ((icmp_ln424_reg_323 == 1'd1) & (icmp_ln421_reg_302 == 1'd0) & (ap_enable_reg_pp0_iter1 == 1'b1));
    end

    assign ap_enable_pp0 = (ap_idle_pp0 ^ 1'b1);

    assign ap_enable_reg_pp0_iter0 = ap_start_int;

    assign ap_loop_exit_ready = ap_condition_exit_pp0_iter0_stage0;

    assign ashr_ln423_fu_243_p2 = $signed(ap_sig_allocacmp_in_shift_2) >>> zext_ln423_fu_239_p1;

    assign icmp_ln421_fu_161_p2 = ((ap_sig_allocacmp_i_3 == 3'd4) ? 1'b1 : 1'b0);

    assign icmp_ln424_fu_205_p2 = ((tmp_2_fu_177_p6 == 32'd16) ? 1'b1 : 1'b0);

    assign in_shift_1_out = in_shift_3_fu_255_p3[61:0];

    assign in_shift_3_fu_255_p3 = ((tmp_reg_312[0:0] == 1'b1) ? ashr_ln423_fu_243_p2 : shl_ln423_fu_249_p2);

    assign in_shift_out = ap_sig_allocacmp_in_shift_2[61:0];

    assign select_ln423_fu_234_p3 = ((tmp_reg_312[0:0] == 1'b1) ? sub_ln423_reg_318 : tmp_2_reg_306);

    assign shift_1_fu_228_p2 = (tmp_2_reg_306 + shift_fu_66);

    assign shift_1_out = (tmp_2_reg_306 + shift_fu_66);

    assign shift_load_1_load_fu_276_p1 = shift_fu_66;

    assign shl_ln423_fu_249_p2 = ap_sig_allocacmp_in_shift_2 << zext_ln423_fu_239_p1;

    assign sub_ln423_fu_199_p2 = (32'd0 - tmp_2_fu_177_p6);

    assign tmp_2_fu_177_p5 = ap_sig_allocacmp_i_3[1:0];

    assign zext_ln423_fu_239_p1 = select_ln423_fu_234_p3;

endmodule  //main_scaled_fixed2ieee_63_1_Pipeline_4
