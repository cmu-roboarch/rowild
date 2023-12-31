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

module main_detectCollNode_Pipeline_2 (
    ap_clk,
    ap_rst,
    ap_start,
    ap_done,
    ap_idle,
    ap_ready,
    checks_address0,
    checks_ce0,
    checks_we0,
    checks_d0
);

    parameter ap_ST_fsm_state1 = 1'd1;

    input ap_clk;
    input ap_rst;
    input ap_start;
    output ap_done;
    output ap_idle;
    output ap_ready;
    output [4:0] checks_address0;
    output checks_ce0;
    output checks_we0;
    output [0:0] checks_d0;

    reg ap_idle;
    reg checks_ce0;
    reg checks_we0;

    (* fsm_encoding = "none" *) reg   [0:0] ap_CS_fsm;
    wire    ap_CS_fsm_state1;
    reg    ap_block_state1_pp0_stage0_iter0;
    wire   [0:0] exitcond114_fu_52_p2;
    reg    ap_condition_exit_pp0_iter0_stage0;
    wire    ap_loop_exit_ready;
    reg    ap_ready_int;
    wire   [63:0] p_cast_fu_64_p1;
    reg   [5:0] empty_fu_26;
    wire   [5:0] empty_79_fu_58_p2;
    wire    ap_loop_init;
    reg   [5:0] ap_sig_allocacmp_p_load;
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
        #0 empty_fu_26 = 6'd0;
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
            if ((exitcond114_fu_52_p2 == 1'd0)) begin
                empty_fu_26 <= empty_79_fu_58_p2;
            end else if ((ap_loop_init == 1'b1)) begin
                empty_fu_26 <= 6'd0;
            end
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
        if (((exitcond114_fu_52_p2 == 1'd1) & (1'b0 == ap_block_state1_pp0_stage0_iter0) & (1'b1 == ap_CS_fsm_state1))) begin
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
        if (((ap_start_int == 1'b0) & (1'b1 == ap_CS_fsm_state1))) begin
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
            ap_sig_allocacmp_p_load = 6'd0;
        end else begin
            ap_sig_allocacmp_p_load = empty_fu_26;
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
        if (((exitcond114_fu_52_p2 == 1'd0) & (1'b0 == ap_block_state1_pp0_stage0_iter0) & (1'b1 == ap_CS_fsm_state1))) begin
            checks_we0 = 1'b1;
        end else begin
            checks_we0 = 1'b0;
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

    assign ap_CS_fsm_state1 = ap_CS_fsm[32'd0];

    always @(*) begin
        ap_block_state1_pp0_stage0_iter0 = (ap_start_int == 1'b0);
    end

    assign ap_loop_exit_ready = ap_condition_exit_pp0_iter0_stage0;

    assign checks_address0 = p_cast_fu_64_p1;

    assign checks_d0 = 1'd0;

    assign empty_79_fu_58_p2 = (ap_sig_allocacmp_p_load + 6'd1);

    assign exitcond114_fu_52_p2 = ((ap_sig_allocacmp_p_load == 6'd32) ? 1'b1 : 1'b0);

    assign p_cast_fu_64_p1 = ap_sig_allocacmp_p_load;

endmodule  //main_detectCollNode_Pipeline_2
