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

module main_scaled_fixed2ieee_63_1_Pipeline_3 (
    ap_clk,
    ap_rst,
    ap_start,
    ap_done,
    ap_idle,
    ap_ready,
    out_bits_0_21_reload,
    out_bits_1_2_reload,
    out_bits_2_2_reload,
    out_bits_3,
    c_2_05_out,
    c_2_05_out_ap_vld,
    c_1_04_out,
    c_1_04_out_ap_vld,
    c_0_03_out,
    c_0_03_out_ap_vld,
    c_3_02_out,
    c_3_02_out_ap_vld
);

    parameter ap_ST_fsm_state1 = 1'd1;

    input ap_clk;
    input ap_rst;
    input ap_start;
    output ap_done;
    output ap_idle;
    output ap_ready;
    input [31:0] out_bits_0_21_reload;
    input [31:0] out_bits_1_2_reload;
    input [31:0] out_bits_2_2_reload;
    input [31:0] out_bits_3;
    output [31:0] c_2_05_out;
    output c_2_05_out_ap_vld;
    output [31:0] c_1_04_out;
    output c_1_04_out_ap_vld;
    output [31:0] c_0_03_out;
    output c_0_03_out_ap_vld;
    output [31:0] c_3_02_out;
    output c_3_02_out_ap_vld;

    reg ap_idle;
    reg c_2_05_out_ap_vld;
    reg c_1_04_out_ap_vld;
    reg c_0_03_out_ap_vld;
    reg c_3_02_out_ap_vld;

    (* fsm_encoding = "none" *) reg   [0:0] ap_CS_fsm;
    wire    ap_CS_fsm_state1;
    reg    ap_block_state1_pp0_stage0_iter0;
    wire   [0:0] icmp_ln414_fu_136_p2;
    reg    ap_condition_exit_pp0_iter0_stage0;
    wire    ap_loop_exit_ready;
    reg    ap_ready_int;
    reg   [2:0] i_1_fu_56;
    wire   [2:0] add_ln414_fu_142_p2;
    wire    ap_loop_init;
    reg   [2:0] ap_sig_allocacmp_i;
    reg   [31:0] c_3_02_fu_60;
    reg   [31:0] c_1_fu_176_p3;
    wire   [1:0] trunc_ln415_fu_148_p1;
    reg   [31:0] c_0_03_fu_64;
    reg   [31:0] c_1_04_fu_68;
    reg   [31:0] c_2_05_fu_72;
    wire   [1:0] x_fu_152_p5;
    wire   [31:0] x_fu_152_p6;
    reg   [31:0] tmp_8_fu_166_p4;
    reg    ap_done_reg;
    wire    ap_continue_int;
    reg    ap_done_int;
    reg   [0:0] ap_NS_fsm;
    reg    ap_ST_fsm_state1_blk;
    wire    ap_start_int;
    wire    ap_ce_reg;

    // power-on initialization
    initial begin
        #0 ap_CS_fsm = 1'd1;
        #0 i_1_fu_56 = 3'd0;
        #0 c_3_02_fu_60 = 32'd0;
        #0 c_0_03_fu_64 = 32'd0;
        #0 c_1_04_fu_68 = 32'd0;
        #0 c_2_05_fu_72 = 32'd0;
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
    ) mux_4_2_32_1_1_U33 (
        .din0(out_bits_0_21_reload),
        .din1(out_bits_1_2_reload),
        .din2(out_bits_2_2_reload),
        .din3(out_bits_3),
        .din4(x_fu_152_p5),
        .dout(x_fu_152_p6)
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
            ap_CS_fsm <= ap_ST_fsm_state1;
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
            end else if (((ap_loop_exit_ready == 1'b1) & (1'b0 == ap_block_state1_pp0_stage0_iter0) & (1'b1 == ap_CS_fsm_state1))) begin
                ap_done_reg <= 1'b1;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (((1'b0 == ap_block_state1_pp0_stage0_iter0) & (1'b1 == ap_CS_fsm_state1))) begin
            if ((icmp_ln414_fu_136_p2 == 1'd0)) begin
                i_1_fu_56 <= add_ln414_fu_142_p2;
            end else if ((ap_loop_init == 1'b1)) begin
                i_1_fu_56 <= 3'd0;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (((trunc_ln415_fu_148_p1 == 2'd0) & (icmp_ln414_fu_136_p2 == 1'd0) & (1'b0 == ap_block_state1_pp0_stage0_iter0) & (1'b1 == ap_CS_fsm_state1))) begin
            c_0_03_fu_64 <= c_1_fu_176_p3;
        end
    end

    always @(posedge ap_clk) begin
        if (((trunc_ln415_fu_148_p1 == 2'd1) & (icmp_ln414_fu_136_p2 == 1'd0) & (1'b0 == ap_block_state1_pp0_stage0_iter0) & (1'b1 == ap_CS_fsm_state1))) begin
            c_1_04_fu_68 <= c_1_fu_176_p3;
        end
    end

    always @(posedge ap_clk) begin
        if (((trunc_ln415_fu_148_p1 == 2'd2) & (icmp_ln414_fu_136_p2 == 1'd0) & (1'b0 == ap_block_state1_pp0_stage0_iter0) & (1'b1 == ap_CS_fsm_state1))) begin
            c_2_05_fu_72 <= c_1_fu_176_p3;
        end
    end

    always @(posedge ap_clk) begin
        if (((trunc_ln415_fu_148_p1 == 2'd3) & (icmp_ln414_fu_136_p2 == 1'd0) & (1'b0 == ap_block_state1_pp0_stage0_iter0) & (1'b1 == ap_CS_fsm_state1))) begin
            c_3_02_fu_60 <= c_1_fu_176_p3;
        end
    end

    always @(*) begin
        if ((1'b1 == ap_block_state1_pp0_stage0_iter0)) begin
            ap_ST_fsm_state1_blk = 1'b1;
        end else begin
            ap_ST_fsm_state1_blk = 1'b0;
        end
    end

    always @(*) begin
        if (((icmp_ln414_fu_136_p2 == 1'd1) & (1'b0 == ap_block_state1_pp0_stage0_iter0) & (1'b1 == ap_CS_fsm_state1))) begin
            ap_condition_exit_pp0_iter0_stage0 = 1'b1;
        end else begin
            ap_condition_exit_pp0_iter0_stage0 = 1'b0;
        end
    end

    always @(*) begin
        if (((ap_loop_exit_ready == 1'b1) & (1'b0 == ap_block_state1_pp0_stage0_iter0) & (1'b1 == ap_CS_fsm_state1))) begin
            ap_done_int = 1'b1;
        end else begin
            ap_done_int = ap_done_reg;
        end
    end

    always @(*) begin
        if (((1'b1 == ap_CS_fsm_state1) & (ap_start_int == 1'b0))) begin
            ap_idle = 1'b1;
        end else begin
            ap_idle = 1'b0;
        end
    end

    always @(*) begin
        if (((1'b0 == ap_block_state1_pp0_stage0_iter0) & (1'b1 == ap_CS_fsm_state1))) begin
            ap_ready_int = 1'b1;
        end else begin
            ap_ready_int = 1'b0;
        end
    end

    always @(*) begin
        if (((ap_loop_init == 1'b1) & (1'b1 == ap_CS_fsm_state1))) begin
            ap_sig_allocacmp_i = 3'd0;
        end else begin
            ap_sig_allocacmp_i = i_1_fu_56;
        end
    end

    always @(*) begin
        if (((icmp_ln414_fu_136_p2 == 1'd1) & (1'b0 == ap_block_state1_pp0_stage0_iter0) & (1'b1 == ap_CS_fsm_state1))) begin
            c_0_03_out_ap_vld = 1'b1;
        end else begin
            c_0_03_out_ap_vld = 1'b0;
        end
    end

    always @(*) begin
        if (((icmp_ln414_fu_136_p2 == 1'd1) & (1'b0 == ap_block_state1_pp0_stage0_iter0) & (1'b1 == ap_CS_fsm_state1))) begin
            c_1_04_out_ap_vld = 1'b1;
        end else begin
            c_1_04_out_ap_vld = 1'b0;
        end
    end

    always @(*) begin
        if (((icmp_ln414_fu_136_p2 == 1'd1) & (1'b0 == ap_block_state1_pp0_stage0_iter0) & (1'b1 == ap_CS_fsm_state1))) begin
            c_2_05_out_ap_vld = 1'b1;
        end else begin
            c_2_05_out_ap_vld = 1'b0;
        end
    end

    always @(*) begin
        if (((icmp_ln414_fu_136_p2 == 1'd1) & (1'b0 == ap_block_state1_pp0_stage0_iter0) & (1'b1 == ap_CS_fsm_state1))) begin
            c_3_02_out_ap_vld = 1'b1;
        end else begin
            c_3_02_out_ap_vld = 1'b0;
        end
    end

    always @(*) begin
        case (ap_CS_fsm)
            ap_ST_fsm_state1: begin
                ap_NS_fsm = ap_ST_fsm_state1;
            end
            default: begin
                ap_NS_fsm = 'bx;
            end
        endcase
    end

    assign add_ln414_fu_142_p2 = (ap_sig_allocacmp_i + 3'd1);

    assign ap_CS_fsm_state1 = ap_CS_fsm[32'd0];

    always @(*) begin
        ap_block_state1_pp0_stage0_iter0 = (ap_start_int == 1'b0);
    end

    assign ap_loop_exit_ready = ap_condition_exit_pp0_iter0_stage0;

    assign c_0_03_out = c_0_03_fu_64;

    assign c_1_04_out = c_1_04_fu_68;


    always @(tmp_8_fu_166_p4) begin
        if (tmp_8_fu_166_p4[0] == 1'b1) begin
            c_1_fu_176_p3 = 32'd0;
        end else if (tmp_8_fu_166_p4[1] == 1'b1) begin
            c_1_fu_176_p3 = 32'd1;
        end else if (tmp_8_fu_166_p4[2] == 1'b1) begin
            c_1_fu_176_p3 = 32'd2;
        end else if (tmp_8_fu_166_p4[3] == 1'b1) begin
            c_1_fu_176_p3 = 32'd3;
        end else if (tmp_8_fu_166_p4[4] == 1'b1) begin
            c_1_fu_176_p3 = 32'd4;
        end else if (tmp_8_fu_166_p4[5] == 1'b1) begin
            c_1_fu_176_p3 = 32'd5;
        end else if (tmp_8_fu_166_p4[6] == 1'b1) begin
            c_1_fu_176_p3 = 32'd6;
        end else if (tmp_8_fu_166_p4[7] == 1'b1) begin
            c_1_fu_176_p3 = 32'd7;
        end else if (tmp_8_fu_166_p4[8] == 1'b1) begin
            c_1_fu_176_p3 = 32'd8;
        end else if (tmp_8_fu_166_p4[9] == 1'b1) begin
            c_1_fu_176_p3 = 32'd9;
        end else if (tmp_8_fu_166_p4[10] == 1'b1) begin
            c_1_fu_176_p3 = 32'd10;
        end else if (tmp_8_fu_166_p4[11] == 1'b1) begin
            c_1_fu_176_p3 = 32'd11;
        end else if (tmp_8_fu_166_p4[12] == 1'b1) begin
            c_1_fu_176_p3 = 32'd12;
        end else if (tmp_8_fu_166_p4[13] == 1'b1) begin
            c_1_fu_176_p3 = 32'd13;
        end else if (tmp_8_fu_166_p4[14] == 1'b1) begin
            c_1_fu_176_p3 = 32'd14;
        end else if (tmp_8_fu_166_p4[15] == 1'b1) begin
            c_1_fu_176_p3 = 32'd15;
        end else if (tmp_8_fu_166_p4[16] == 1'b1) begin
            c_1_fu_176_p3 = 32'd16;
        end else if (tmp_8_fu_166_p4[17] == 1'b1) begin
            c_1_fu_176_p3 = 32'd17;
        end else if (tmp_8_fu_166_p4[18] == 1'b1) begin
            c_1_fu_176_p3 = 32'd18;
        end else if (tmp_8_fu_166_p4[19] == 1'b1) begin
            c_1_fu_176_p3 = 32'd19;
        end else if (tmp_8_fu_166_p4[20] == 1'b1) begin
            c_1_fu_176_p3 = 32'd20;
        end else if (tmp_8_fu_166_p4[21] == 1'b1) begin
            c_1_fu_176_p3 = 32'd21;
        end else if (tmp_8_fu_166_p4[22] == 1'b1) begin
            c_1_fu_176_p3 = 32'd22;
        end else if (tmp_8_fu_166_p4[23] == 1'b1) begin
            c_1_fu_176_p3 = 32'd23;
        end else if (tmp_8_fu_166_p4[24] == 1'b1) begin
            c_1_fu_176_p3 = 32'd24;
        end else if (tmp_8_fu_166_p4[25] == 1'b1) begin
            c_1_fu_176_p3 = 32'd25;
        end else if (tmp_8_fu_166_p4[26] == 1'b1) begin
            c_1_fu_176_p3 = 32'd26;
        end else if (tmp_8_fu_166_p4[27] == 1'b1) begin
            c_1_fu_176_p3 = 32'd27;
        end else if (tmp_8_fu_166_p4[28] == 1'b1) begin
            c_1_fu_176_p3 = 32'd28;
        end else if (tmp_8_fu_166_p4[29] == 1'b1) begin
            c_1_fu_176_p3 = 32'd29;
        end else if (tmp_8_fu_166_p4[30] == 1'b1) begin
            c_1_fu_176_p3 = 32'd30;
        end else if (tmp_8_fu_166_p4[31] == 1'b1) begin
            c_1_fu_176_p3 = 32'd31;
        end else begin
            c_1_fu_176_p3 = 32'd32;
        end
    end

    assign c_2_05_out = c_2_05_fu_72;

    assign c_3_02_out = c_3_02_fu_60;

    assign icmp_ln414_fu_136_p2 = ((ap_sig_allocacmp_i == 3'd4) ? 1'b1 : 1'b0);

    integer ap_tvar_int_0;

    always @(x_fu_152_p6) begin
        for (ap_tvar_int_0 = 32 - 1; ap_tvar_int_0 >= 0; ap_tvar_int_0 = ap_tvar_int_0 - 1) begin
            if (ap_tvar_int_0 > 31 - 0) begin
                tmp_8_fu_166_p4[ap_tvar_int_0] = 1'b0;
            end else begin
                tmp_8_fu_166_p4[ap_tvar_int_0] = x_fu_152_p6[31-ap_tvar_int_0];
            end
        end
    end

    assign trunc_ln415_fu_148_p1 = ap_sig_allocacmp_i[1:0];

    assign x_fu_152_p5 = ap_sig_allocacmp_i[1:0];

endmodule  //main_scaled_fixed2ieee_63_1_Pipeline_3
