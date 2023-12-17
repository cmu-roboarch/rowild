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

module main_detectCollNode_Pipeline_VITIS_LOOP_176_3 (
    ap_clk,
    ap_rst,
    ap_start,
    ap_done,
    ap_idle,
    ap_ready,
    checks_q0,
    checks_address0,
    checks_ce0,
    ap_return
);

    parameter ap_ST_fsm_state1 = 2'd1;
    parameter ap_ST_fsm_state2 = 2'd2;

    input ap_clk;
    input ap_rst;
    input ap_start;
    output ap_done;
    output ap_idle;
    output ap_ready;
    input [0:0] checks_q0;
    output [4:0] checks_address0;
    output checks_ce0;
    output [0:0] ap_return;

    reg ap_idle;
    reg checks_ce0;
    reg[0:0] ap_return;

    (* fsm_encoding = "none" *) reg   [1:0] ap_CS_fsm;
    wire    ap_CS_fsm_state1;
    wire    ap_CS_fsm_state2;
    reg   [0:0] icmp_ln176_reg_99;
    reg    ap_condition_exit_pp0_iter0_stage1;
    wire    ap_loop_exit_ready;
    reg    ap_ready_int;
    reg   [0:0] merge_reg_51;
    wire   [0:0] icmp_ln176_fu_71_p2;
    reg    ap_block_state1_pp0_stage0_iter0;
    wire   [5:0] add_ln176_fu_77_p2;
    reg   [5:0] add_ln176_reg_103;
    reg   [0:0] ap_phi_mux_merge_phi_fu_55_p4;
    wire   [63:0] zext_ln176_fu_83_p1;
    reg   [5:0] i_fu_34;
    wire    ap_loop_init;
    reg   [5:0] ap_sig_allocacmp_i_11;
    reg   [0:0] ap_return_preg;
    reg    ap_done_reg;
    wire    ap_continue_int;
    reg    ap_done_int;
    reg   [1:0] ap_NS_fsm;
    reg    ap_ST_fsm_state1_blk;
    wire    ap_ST_fsm_state2_blk;
    wire    ap_start_int;
    wire    ap_ce_reg;

    // power-on initialization
    initial begin
        #0 ap_CS_fsm = 2'd1;
        #0 i_fu_34 = 6'd0;
        #0 ap_return_preg = 1'd0;
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
        .ap_loop_exit_ready(ap_condition_exit_pp0_iter0_stage1),
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
            end else if (((ap_loop_exit_ready == 1'b1) & (1'b1 == ap_CS_fsm_state2))) begin
                ap_done_reg <= 1'b1;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (ap_rst == 1'b1) begin
            ap_return_preg <= 1'd0;
        end else begin
            if (((1'b1 == ap_CS_fsm_state2) & ((icmp_ln176_reg_99 == 1'd1) | (checks_q0 == 1'd1)))) begin
                ap_return_preg <= ap_phi_mux_merge_phi_fu_55_p4;
            end
        end
    end

    always @(posedge ap_clk) begin
        if (((ap_loop_init == 1'b1) & (1'b0 == ap_block_state1_pp0_stage0_iter0) & (1'b1 == ap_CS_fsm_state1))) begin
            i_fu_34 <= 6'd0;
        end else if (((icmp_ln176_reg_99 == 1'd0) & (checks_q0 == 1'd0) & (1'b1 == ap_CS_fsm_state2))) begin
            i_fu_34 <= add_ln176_reg_103;
        end
    end

    always @(posedge ap_clk) begin
        if (((icmp_ln176_reg_99 == 1'd0) & (checks_q0 == 1'd1) & (1'b1 == ap_CS_fsm_state2))) begin
            merge_reg_51 <= 1'd0;
        end else if (((1'b0 == ap_block_state1_pp0_stage0_iter0) & (icmp_ln176_fu_71_p2 == 1'd1) & (1'b1 == ap_CS_fsm_state1))) begin
            merge_reg_51 <= 1'd1;
        end
    end

    always @(posedge ap_clk) begin
        if (((1'b0 == ap_block_state1_pp0_stage0_iter0) & (1'b1 == ap_CS_fsm_state1))) begin
            add_ln176_reg_103 <= add_ln176_fu_77_p2;
            icmp_ln176_reg_99 <= icmp_ln176_fu_71_p2;
        end
    end

    always @(*) begin
        if ((1'b1 == ap_block_state1_pp0_stage0_iter0)) begin
            ap_ST_fsm_state1_blk = 1'b1;
        end else begin
            ap_ST_fsm_state1_blk = 1'b0;
        end
    end

    assign ap_ST_fsm_state2_blk = 1'b0;

    always @(*) begin
        if (((1'b1 == ap_CS_fsm_state2) & ((icmp_ln176_reg_99 == 1'd1) | (checks_q0 == 1'd1)))) begin
            ap_condition_exit_pp0_iter0_stage1 = 1'b1;
        end else begin
            ap_condition_exit_pp0_iter0_stage1 = 1'b0;
        end
    end

    always @(*) begin
        if (((ap_loop_exit_ready == 1'b1) & (1'b1 == ap_CS_fsm_state2))) begin
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
        if (((icmp_ln176_reg_99 == 1'd0) & (checks_q0 == 1'd1) & (1'b1 == ap_CS_fsm_state2))) begin
            ap_phi_mux_merge_phi_fu_55_p4 = 1'd0;
        end else begin
            ap_phi_mux_merge_phi_fu_55_p4 = merge_reg_51;
        end
    end

    always @(*) begin
        if ((1'b1 == ap_CS_fsm_state2)) begin
            ap_ready_int = 1'b1;
        end else begin
            ap_ready_int = 1'b0;
        end
    end

    always @(*) begin
        if (((1'b1 == ap_CS_fsm_state2) & ((icmp_ln176_reg_99 == 1'd1) | (checks_q0 == 1'd1)))) begin
            ap_return = ap_phi_mux_merge_phi_fu_55_p4;
        end else begin
            ap_return = ap_return_preg;
        end
    end

    always @(*) begin
        if (((ap_loop_init == 1'b1) & (1'b1 == ap_CS_fsm_state1))) begin
            ap_sig_allocacmp_i_11 = 6'd0;
        end else begin
            ap_sig_allocacmp_i_11 = i_fu_34;
        end
    end

    always @(*) begin
        if (((1'b0 == ap_block_state1_pp0_stage0_iter0) & (1'b1 == ap_CS_fsm_state1))) begin
            checks_ce0 = 1'b1;
        end else begin
            checks_ce0 = 1'b0;
        end
    end

    always @(*) begin
        case (ap_CS_fsm)
            ap_ST_fsm_state1: begin
                if (((1'b0 == ap_block_state1_pp0_stage0_iter0) & (1'b1 == ap_CS_fsm_state1))) begin
                    ap_NS_fsm = ap_ST_fsm_state2;
                end else begin
                    ap_NS_fsm = ap_ST_fsm_state1;
                end
            end
            ap_ST_fsm_state2: begin
                ap_NS_fsm = ap_ST_fsm_state1;
            end
            default: begin
                ap_NS_fsm = 'bx;
            end
        endcase
    end

    assign add_ln176_fu_77_p2 = (ap_sig_allocacmp_i_11 + 6'd1);

    assign ap_CS_fsm_state1   = ap_CS_fsm[32'd0];

    assign ap_CS_fsm_state2   = ap_CS_fsm[32'd1];

    always @(*) begin
        ap_block_state1_pp0_stage0_iter0 = (ap_start_int == 1'b0);
    end

    assign ap_loop_exit_ready = ap_condition_exit_pp0_iter0_stage1;

    assign checks_address0 = zext_ln176_fu_83_p1;

    assign icmp_ln176_fu_71_p2 = ((ap_sig_allocacmp_i_11 == 6'd32) ? 1'b1 : 1'b0);

    assign zext_ln176_fu_83_p1 = ap_sig_allocacmp_i_11;

endmodule  //main_detectCollNode_Pipeline_VITIS_LOOP_176_3