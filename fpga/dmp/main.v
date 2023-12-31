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

(* CORE_GENERATION_INFO="main_main,hls_ip_2023_2,{HLS_INPUT_TYPE=cxx,HLS_INPUT_FLOAT=0,HLS_INPUT_FIXED=0,HLS_INPUT_PART=xc7z010i-clg225-1L,HLS_INPUT_CLOCK=10.000000,HLS_INPUT_ARCH=others,HLS_SYN_CLOCK=7.297000,HLS_SYN_LAT=-1,HLS_SYN_TPT=none,HLS_SYN_MEM=0,HLS_SYN_DSP=0,HLS_SYN_FF=833,HLS_SYN_LUT=1325,HLS_VERSION=2023_2}" *)

module main (
    ap_clk,
    ap_rst,
    ap_start,
    ap_done,
    ap_idle,
    ap_ready,
    ap_return
);

    parameter ap_ST_fsm_pp0_stage0 = 8'd1;
    parameter ap_ST_fsm_pp0_stage1 = 8'd2;
    parameter ap_ST_fsm_pp0_stage2 = 8'd4;
    parameter ap_ST_fsm_pp0_stage3 = 8'd8;
    parameter ap_ST_fsm_pp0_stage4 = 8'd16;
    parameter ap_ST_fsm_pp0_stage5 = 8'd32;
    parameter ap_ST_fsm_pp0_stage6 = 8'd64;
    parameter ap_ST_fsm_pp0_stage7 = 8'd128;

    input ap_clk;
    input ap_rst;
    input ap_start;
    output ap_done;
    output ap_idle;
    output ap_ready;
    output [31:0] ap_return;

    reg ap_idle;

    (* fsm_encoding = "none" *) reg   [7:0] ap_CS_fsm;
    wire    ap_CS_fsm_pp0_stage0;
    reg    ap_enable_reg_pp0_iter0;
    reg    ap_enable_reg_pp0_iter1;
    reg    ap_idle_pp0;
    wire    ap_CS_fsm_pp0_stage1;
    wire    ap_block_pp0_stage1_subdone;
    wire   [0:0] and_ln58_fu_94_p2;
    reg    ap_condition_exit_pp0_iter0_stage1;
    wire    ap_loop_exit_ready;
    reg    ap_ready_int;
    wire    ap_CS_fsm_pp0_stage7;
    wire    ap_block_pp0_stage7_subdone;
    reg   [63:0] t_2_reg_111;
    wire    ap_block_pp0_stage0_11001;
    wire    ap_block_pp0_stage1_11001;
    wire   [63:0] grp_fu_40_p2;
    reg   [63:0] t_1_reg_122;
    wire    ap_block_pp0_stage7_11001;
    reg    ap_enable_reg_pp0_iter0_reg;
    wire    ap_block_pp0_stage0_subdone;
    reg   [63:0] t_fu_36;
    reg   [63:0] ap_sig_allocacmp_t_2;
    wire    ap_block_pp0_stage0;
    wire    ap_loop_init;
    wire    ap_block_pp0_stage1;
    wire   [63:0] bitcast_ln58_fu_59_p1;
    wire   [10:0] tmp_fu_62_p4;
    wire   [51:0] trunc_ln58_fu_72_p1;
    wire   [0:0] icmp_ln58_1_fu_82_p2;
    wire   [0:0] icmp_ln58_fu_76_p2;
    wire   [0:0] or_ln58_fu_88_p2;
    wire   [0:0] grp_fu_45_p2;
    wire    ap_block_pp0_stage0_00001;
    reg    ap_done_reg;
    wire    ap_continue_int;
    reg    ap_done_int;
    reg   [7:0] ap_NS_fsm;
    reg    ap_idle_pp0_1to1;
    wire    ap_block_pp0_stage2_subdone;
    wire    ap_block_pp0_stage2_11001;
    wire    ap_block_pp0_stage3_subdone;
    wire    ap_block_pp0_stage3_11001;
    wire    ap_block_pp0_stage4_subdone;
    wire    ap_block_pp0_stage4_11001;
    wire    ap_block_pp0_stage5_subdone;
    wire    ap_block_pp0_stage5_11001;
    wire    ap_block_pp0_stage6_subdone;
    wire    ap_block_pp0_stage6_11001;
    wire    ap_enable_pp0;
    wire    ap_start_int;
    wire    ap_ce_reg;

    // power-on initialization
    initial begin
        #0 ap_CS_fsm = 8'd1;
        #0 ap_enable_reg_pp0_iter1 = 1'b0;
        #0 ap_enable_reg_pp0_iter0_reg = 1'b0;
        #0 t_fu_36 = 64'd0;
        #0 ap_done_reg = 1'b0;
    end

    main_dadd_64ns_64ns_64_7_full_dsp_1 #(
        .ID(1),
        .NUM_STAGE(7),
        .din0_WIDTH(64),
        .din1_WIDTH(64),
        .dout_WIDTH(64)
    ) dadd_64ns_64ns_64_7_full_dsp_1_U1 (
        .clk(ap_clk),
        .reset(ap_rst),
        .din0(t_2_reg_111),
        .din1(64'd4562254508917369340),
        .ce(1'b1),
        .dout(grp_fu_40_p2)
    );

    main_dcmp_64ns_64ns_1_2_no_dsp_1 #(
        .ID(1),
        .NUM_STAGE(2),
        .din0_WIDTH(64),
        .din1_WIDTH(64),
        .dout_WIDTH(1)
    ) dcmp_64ns_64ns_1_2_no_dsp_1_U2 (
        .clk(ap_clk),
        .reset(ap_rst),
        .din0(ap_sig_allocacmp_t_2),
        .din1(64'd4639481672377565184),
        .ce(1'b1),
        .opcode(5'd5),
        .dout(grp_fu_45_p2)
    );

    main_flow_control_loop_pipe flow_control_loop_pipe_U (
        .ap_clk(ap_clk),
        .ap_rst(ap_rst),
        .ap_start(ap_start),
        .ap_ready(ap_ready),
        .ap_done(ap_done),
        .ap_start_int(ap_start_int),
        .ap_loop_init(ap_loop_init),
        .ap_ready_int(ap_ready_int),
        .ap_loop_exit_ready(ap_condition_exit_pp0_iter0_stage1),
        .ap_loop_exit_done(ap_done_int),
        .ap_continue_int(ap_continue_int),
        .ap_done_int(ap_done_int),
        .ap_continue(1'b1)
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
            end else if (((ap_loop_exit_ready == 1'b1) & (1'b0 == ap_block_pp0_stage1_subdone) & (1'b1 == ap_CS_fsm_pp0_stage1))) begin
                ap_done_reg <= 1'b1;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (ap_rst == 1'b1) begin
            ap_enable_reg_pp0_iter0_reg <= 1'b0;
        end else begin
            if ((1'b1 == ap_condition_exit_pp0_iter0_stage1)) begin
                ap_enable_reg_pp0_iter0_reg <= 1'b0;
            end else if ((1'b1 == ap_CS_fsm_pp0_stage0)) begin
                ap_enable_reg_pp0_iter0_reg <= ap_start_int;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (ap_rst == 1'b1) begin
            ap_enable_reg_pp0_iter1 <= 1'b0;
        end else begin
            if (((1'b0 == ap_block_pp0_stage0_subdone) & (ap_enable_reg_pp0_iter1 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
                ap_enable_reg_pp0_iter1 <= 1'b0;
            end else if (((1'b0 == ap_block_pp0_stage7_subdone) & (1'b1 == ap_CS_fsm_pp0_stage7))) begin
                ap_enable_reg_pp0_iter1 <= ap_enable_reg_pp0_iter0;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (((1'b0 == ap_block_pp0_stage0_11001) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
            if (((ap_enable_reg_pp0_iter0 == 1'b1) & (ap_loop_init == 1'b1))) begin
                t_fu_36 <= 64'd0;
            end else if ((ap_enable_reg_pp0_iter1 == 1'b1)) begin
                t_fu_36 <= t_1_reg_122;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (((1'b0 == ap_block_pp0_stage7_11001) & (1'b1 == ap_CS_fsm_pp0_stage7))) begin
            t_1_reg_122 <= grp_fu_40_p2;
        end
    end

    always @(posedge ap_clk) begin
        if (((1'b0 == ap_block_pp0_stage0_11001) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
            t_2_reg_111 <= ap_sig_allocacmp_t_2;
        end
    end

    always @(*) begin
        if (((1'd0 == and_ln58_fu_94_p2) & (1'b0 == ap_block_pp0_stage1_subdone) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage1))) begin
            ap_condition_exit_pp0_iter0_stage1 = 1'b1;
        end else begin
            ap_condition_exit_pp0_iter0_stage1 = 1'b0;
        end
    end

    always @(*) begin
        if (((ap_loop_exit_ready == 1'b1) & (1'b0 == ap_block_pp0_stage1_subdone) & (1'b1 == ap_CS_fsm_pp0_stage1))) begin
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
        if (((ap_enable_reg_pp0_iter1 == 1'b0) & (ap_enable_reg_pp0_iter0 == 1'b0))) begin
            ap_idle_pp0 = 1'b1;
        end else begin
            ap_idle_pp0 = 1'b0;
        end
    end

    always @(*) begin
        if ((ap_enable_reg_pp0_iter1 == 1'b0)) begin
            ap_idle_pp0_1to1 = 1'b1;
        end else begin
            ap_idle_pp0_1to1 = 1'b0;
        end
    end

    always @(*) begin
        if (((1'b0 == ap_block_pp0_stage7_subdone) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage7))) begin
            ap_ready_int = 1'b1;
        end else begin
            ap_ready_int = 1'b0;
        end
    end

    always @(*) begin
        if (((1'b0 == ap_block_pp0_stage0) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
            if (((ap_enable_reg_pp0_iter0 == 1'b1) & (ap_loop_init == 1'b1))) begin
                ap_sig_allocacmp_t_2 = 64'd0;
            end else if ((ap_enable_reg_pp0_iter1 == 1'b1)) begin
                ap_sig_allocacmp_t_2 = t_1_reg_122;
            end else begin
                ap_sig_allocacmp_t_2 = t_fu_36;
            end
        end else begin
            ap_sig_allocacmp_t_2 = t_fu_36;
        end
    end

    always @(*) begin
        case (ap_CS_fsm)
            ap_ST_fsm_pp0_stage0: begin
                if ((~((ap_start_int == 1'b0) & (ap_idle_pp0_1to1 == 1'b1)) & (1'b0 == ap_block_pp0_stage0_subdone))) begin
                    ap_NS_fsm = ap_ST_fsm_pp0_stage1;
                end else begin
                    ap_NS_fsm = ap_ST_fsm_pp0_stage0;
                end
            end
            ap_ST_fsm_pp0_stage1: begin
                if ((1'b1 == ap_condition_exit_pp0_iter0_stage1)) begin
                    ap_NS_fsm = ap_ST_fsm_pp0_stage0;
                end else if ((1'b0 == ap_block_pp0_stage1_subdone)) begin
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
                if ((1'b0 == ap_block_pp0_stage3_subdone)) begin
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
                    ap_NS_fsm = ap_ST_fsm_pp0_stage7;
                end else begin
                    ap_NS_fsm = ap_ST_fsm_pp0_stage6;
                end
            end
            ap_ST_fsm_pp0_stage7: begin
                if ((1'b0 == ap_block_pp0_stage7_subdone)) begin
                    ap_NS_fsm = ap_ST_fsm_pp0_stage0;
                end else begin
                    ap_NS_fsm = ap_ST_fsm_pp0_stage7;
                end
            end
            default: begin
                ap_NS_fsm = 'bx;
            end
        endcase
    end

    assign and_ln58_fu_94_p2 = (or_ln58_fu_88_p2 & grp_fu_45_p2);

    assign ap_CS_fsm_pp0_stage0 = ap_CS_fsm[32'd0];

    assign ap_CS_fsm_pp0_stage1 = ap_CS_fsm[32'd1];

    assign ap_CS_fsm_pp0_stage7 = ap_CS_fsm[32'd7];

    assign ap_block_pp0_stage0 = ~(1'b1 == 1'b1);

    assign ap_block_pp0_stage0_00001 = ~(1'b1 == 1'b1);

    assign ap_block_pp0_stage0_11001 = ~(1'b1 == 1'b1);

    assign ap_block_pp0_stage0_subdone = ~(1'b1 == 1'b1);

    assign ap_block_pp0_stage1 = ~(1'b1 == 1'b1);

    assign ap_block_pp0_stage1_11001 = ~(1'b1 == 1'b1);

    assign ap_block_pp0_stage1_subdone = ~(1'b1 == 1'b1);

    assign ap_block_pp0_stage2_11001 = ~(1'b1 == 1'b1);

    assign ap_block_pp0_stage2_subdone = ~(1'b1 == 1'b1);

    assign ap_block_pp0_stage3_11001 = ~(1'b1 == 1'b1);

    assign ap_block_pp0_stage3_subdone = ~(1'b1 == 1'b1);

    assign ap_block_pp0_stage4_11001 = ~(1'b1 == 1'b1);

    assign ap_block_pp0_stage4_subdone = ~(1'b1 == 1'b1);

    assign ap_block_pp0_stage5_11001 = ~(1'b1 == 1'b1);

    assign ap_block_pp0_stage5_subdone = ~(1'b1 == 1'b1);

    assign ap_block_pp0_stage6_11001 = ~(1'b1 == 1'b1);

    assign ap_block_pp0_stage6_subdone = ~(1'b1 == 1'b1);

    assign ap_block_pp0_stage7_11001 = ~(1'b1 == 1'b1);

    assign ap_block_pp0_stage7_subdone = ~(1'b1 == 1'b1);

    assign ap_enable_pp0 = (ap_idle_pp0 ^ 1'b1);

    assign ap_loop_exit_ready = ap_condition_exit_pp0_iter0_stage1;

    assign ap_return = 32'd0;

    assign bitcast_ln58_fu_59_p1 = t_2_reg_111;

    assign icmp_ln58_1_fu_82_p2 = ((trunc_ln58_fu_72_p1 == 52'd0) ? 1'b1 : 1'b0);

    assign icmp_ln58_fu_76_p2 = ((tmp_fu_62_p4 != 11'd2047) ? 1'b1 : 1'b0);

    assign or_ln58_fu_88_p2 = (icmp_ln58_fu_76_p2 | icmp_ln58_1_fu_82_p2);

    assign tmp_fu_62_p4 = {{bitcast_ln58_fu_59_p1[62:52]}};

    assign trunc_ln58_fu_72_p1 = bitcast_ln58_fu_59_p1[51:0];

endmodule  //main
