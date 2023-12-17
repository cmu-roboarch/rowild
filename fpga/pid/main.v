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

(* CORE_GENERATION_INFO="main_main,hls_ip_2023_2,{HLS_INPUT_TYPE=cxx,HLS_INPUT_FLOAT=0,HLS_INPUT_FIXED=0,HLS_INPUT_PART=xc7z010i-clg225-1L,HLS_INPUT_CLOCK=10.000000,HLS_INPUT_ARCH=others,HLS_SYN_CLOCK=8.607700,HLS_SYN_LAT=7118,HLS_SYN_TPT=none,HLS_SYN_MEM=0,HLS_SYN_DSP=0,HLS_SYN_FF=2248,HLS_SYN_LUT=3378,HLS_VERSION=2023_2}" *)

module main (
    ap_clk,
    ap_rst,
    ap_start,
    ap_done,
    ap_idle,
    ap_ready,
    ap_return
);

    parameter ap_ST_fsm_state1 = 4'd1;
    parameter ap_ST_fsm_state2 = 4'd2;
    parameter ap_ST_fsm_state3 = 4'd4;
    parameter ap_ST_fsm_state4 = 4'd8;

    input ap_clk;
    input ap_rst;
    input ap_start;
    output ap_done;
    output ap_idle;
    output ap_ready;
    output [31:0] ap_return;

    reg ap_done;
    reg ap_idle;
    reg ap_ready;

    (* fsm_encoding = "none" *) reg   [3:0] ap_CS_fsm;
    wire    ap_CS_fsm_state1;
    reg   [0:0] xs_sign_reg_194;
    wire    ap_CS_fsm_state3;
    wire   [0:0] tmp_fu_104_p3;
    reg   [0:0] tmp_reg_199;
    reg   [31:0] tmp_1_reg_204;
    reg   [31:0] tmp_2_reg_209;
    wire    grp_main_Pipeline_VITIS_LOOP_42_1_fu_46_ap_start;
    wire    grp_main_Pipeline_VITIS_LOOP_42_1_fu_46_ap_done;
    wire    grp_main_Pipeline_VITIS_LOOP_42_1_fu_46_ap_idle;
    wire    grp_main_Pipeline_VITIS_LOOP_42_1_fu_46_ap_ready;
    wire   [63:0] grp_main_Pipeline_VITIS_LOOP_42_1_fu_46_sum_out;
    wire    grp_main_Pipeline_VITIS_LOOP_42_1_fu_46_sum_out_ap_vld;
    reg    grp_main_Pipeline_VITIS_LOOP_42_1_fu_46_ap_start_reg;
    wire    ap_CS_fsm_state2;
    wire   [63:0] data_fu_54_p1;
    wire   [51:0] trunc_ln505_fu_76_p1;
    wire   [53:0] mantissa_fu_80_p4;
    wire   [10:0] xs_exp_fu_66_p4;
    wire   [11:0] zext_ln486_fu_94_p1;
    wire   [11:0] add_ln486_fu_98_p2;
    wire   [10:0] sub_ln18_fu_112_p2;
    wire  signed [11:0] sext_ln18_fu_118_p1;
    wire   [11:0] select_ln18_fu_122_p3;
    wire  signed [31:0] sext_ln18_1_fu_130_p1;
    wire   [136:0] zext_ln15_fu_90_p1;
    wire   [136:0] zext_ln18_fu_134_p1;
    wire   [136:0] lshr_ln18_fu_138_p2;
    wire   [136:0] shl_ln18_fu_144_p2;
    wire    ap_CS_fsm_state4;
    wire   [31:0] val_fu_170_p3;
    wire   [31:0] result_1_fu_175_p2;
    reg   [3:0] ap_NS_fsm;
    reg    ap_ST_fsm_state1_blk;
    reg    ap_ST_fsm_state2_blk;
    wire    ap_ST_fsm_state3_blk;
    wire    ap_ST_fsm_state4_blk;
    wire    ap_ce_reg;

    // power-on initialization
    initial begin
        #0 ap_CS_fsm = 4'd1;
        #0 grp_main_Pipeline_VITIS_LOOP_42_1_fu_46_ap_start_reg = 1'b0;
    end

    main_main_Pipeline_VITIS_LOOP_42_1 grp_main_Pipeline_VITIS_LOOP_42_1_fu_46 (
        .ap_clk(ap_clk),
        .ap_rst(ap_rst),
        .ap_start(grp_main_Pipeline_VITIS_LOOP_42_1_fu_46_ap_start),
        .ap_done(grp_main_Pipeline_VITIS_LOOP_42_1_fu_46_ap_done),
        .ap_idle(grp_main_Pipeline_VITIS_LOOP_42_1_fu_46_ap_idle),
        .ap_ready(grp_main_Pipeline_VITIS_LOOP_42_1_fu_46_ap_ready),
        .sum_out(grp_main_Pipeline_VITIS_LOOP_42_1_fu_46_sum_out),
        .sum_out_ap_vld(grp_main_Pipeline_VITIS_LOOP_42_1_fu_46_sum_out_ap_vld)
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
            grp_main_Pipeline_VITIS_LOOP_42_1_fu_46_ap_start_reg <= 1'b0;
        end else begin
            if (((ap_start == 1'b1) & (1'b1 == ap_CS_fsm_state1))) begin
                grp_main_Pipeline_VITIS_LOOP_42_1_fu_46_ap_start_reg <= 1'b1;
            end else if ((grp_main_Pipeline_VITIS_LOOP_42_1_fu_46_ap_ready == 1'b1)) begin
                grp_main_Pipeline_VITIS_LOOP_42_1_fu_46_ap_start_reg <= 1'b0;
            end
        end
    end

    always @(posedge ap_clk) begin
        if ((1'b1 == ap_CS_fsm_state3)) begin
            tmp_1_reg_204 <= {{lshr_ln18_fu_138_p2[84:53]}};
            tmp_2_reg_209 <= {{shl_ln18_fu_144_p2[84:53]}};
            tmp_reg_199 <= add_ln486_fu_98_p2[32'd11];
            xs_sign_reg_194 <= data_fu_54_p1[32'd63];
        end
    end

    always @(*) begin
        if ((ap_start == 1'b0)) begin
            ap_ST_fsm_state1_blk = 1'b1;
        end else begin
            ap_ST_fsm_state1_blk = 1'b0;
        end
    end

    always @(*) begin
        if ((grp_main_Pipeline_VITIS_LOOP_42_1_fu_46_ap_done == 1'b0)) begin
            ap_ST_fsm_state2_blk = 1'b1;
        end else begin
            ap_ST_fsm_state2_blk = 1'b0;
        end
    end

    assign ap_ST_fsm_state3_blk = 1'b0;

    assign ap_ST_fsm_state4_blk = 1'b0;

    always @(*) begin
        if ((1'b1 == ap_CS_fsm_state4)) begin
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
        if ((1'b1 == ap_CS_fsm_state4)) begin
            ap_ready = 1'b1;
        end else begin
            ap_ready = 1'b0;
        end
    end

    always @(*) begin
        case (ap_CS_fsm)
            ap_ST_fsm_state1: begin
                if (((ap_start == 1'b1) & (1'b1 == ap_CS_fsm_state1))) begin
                    ap_NS_fsm = ap_ST_fsm_state2;
                end else begin
                    ap_NS_fsm = ap_ST_fsm_state1;
                end
            end
            ap_ST_fsm_state2: begin
                if (((grp_main_Pipeline_VITIS_LOOP_42_1_fu_46_ap_done == 1'b1) & (1'b1 == ap_CS_fsm_state2))) begin
                    ap_NS_fsm = ap_ST_fsm_state3;
                end else begin
                    ap_NS_fsm = ap_ST_fsm_state2;
                end
            end
            ap_ST_fsm_state3: begin
                ap_NS_fsm = ap_ST_fsm_state4;
            end
            ap_ST_fsm_state4: begin
                ap_NS_fsm = ap_ST_fsm_state1;
            end
            default: begin
                ap_NS_fsm = 'bx;
            end
        endcase
    end

    assign add_ln486_fu_98_p2 = ($signed(zext_ln486_fu_94_p1) + $signed(12'd3073));

    assign ap_CS_fsm_state1 = ap_CS_fsm[32'd0];

    assign ap_CS_fsm_state2 = ap_CS_fsm[32'd1];

    assign ap_CS_fsm_state3 = ap_CS_fsm[32'd2];

    assign ap_CS_fsm_state4 = ap_CS_fsm[32'd3];

    assign ap_return = ((xs_sign_reg_194[0:0] == 1'b1) ? result_1_fu_175_p2 : val_fu_170_p3);

    assign data_fu_54_p1 = grp_main_Pipeline_VITIS_LOOP_42_1_fu_46_sum_out;

    assign grp_main_Pipeline_VITIS_LOOP_42_1_fu_46_ap_start = grp_main_Pipeline_VITIS_LOOP_42_1_fu_46_ap_start_reg;

    assign lshr_ln18_fu_138_p2 = zext_ln15_fu_90_p1 >> zext_ln18_fu_134_p1;

    assign mantissa_fu_80_p4 = {{{{1'd1}, {trunc_ln505_fu_76_p1}}}, {1'd0}};

    assign result_1_fu_175_p2 = (32'd0 - val_fu_170_p3);

    assign select_ln18_fu_122_p3 = ((tmp_fu_104_p3[0:0] == 1'b1) ? sext_ln18_fu_118_p1 : add_ln486_fu_98_p2);

    assign sext_ln18_1_fu_130_p1 = $signed(select_ln18_fu_122_p3);

    assign sext_ln18_fu_118_p1 = $signed(sub_ln18_fu_112_p2);

    assign shl_ln18_fu_144_p2 = zext_ln15_fu_90_p1 << zext_ln18_fu_134_p1;

    assign sub_ln18_fu_112_p2 = (11'd1023 - xs_exp_fu_66_p4);

    assign tmp_fu_104_p3 = add_ln486_fu_98_p2[32'd11];

    assign trunc_ln505_fu_76_p1 = data_fu_54_p1[51:0];

    assign val_fu_170_p3 = ((tmp_reg_199[0:0] == 1'b1) ? tmp_1_reg_204 : tmp_2_reg_209);

    assign xs_exp_fu_66_p4 = {{data_fu_54_p1[62:52]}};

    assign zext_ln15_fu_90_p1 = mantissa_fu_80_p4;

    assign zext_ln18_fu_134_p1 = $unsigned(sext_ln18_1_fu_130_p1);

    assign zext_ln486_fu_94_p1 = xs_exp_fu_66_p4;

endmodule  //main
