// SPDX-License-Identifier: GPL-3.0-only
// Copyright (c) 2026 Leonardo Capossio - bard0 design - hello@bard0.com

//--------------------------------------------------------------------------------------------------------
// Module  : tb_axi_jls_ctrl
// Type    : simulation, top
// Function: self-checking testbench for the Arty demo's AXI wrapper (example/arty_a7/rtl/axi_jls_ctrl.v).
//
//   1. WIDTH/HEIGHT validation: out-of-range sizes set dim_error and do not start a frame;
//      15-bit WIDTH/HEIGHT registers hold 16384.
//   2. One frame pushed with long AXI write bursts (wvalid held high) while a slow
//      reader with random gaps drains the output. The small input FIFO fills, so
//      wready back-pressure is exercised; every accepted pixel beat must reach the FIFO.
//   3. STATUS fields are checked against the FIFO counters every cycle, and the
//      host-side decode (same masks as run_demo.py) is used to drive the reader.
//   4. The drained stream must equal the CharLS golden byte-for-byte.
//
// Defines (run_sim.py --ctrl writes them into sim_config.vh):
//   NEAR, CTRL_IMG (input .pgm), CTRL_GOLDEN (expected .jls)
//--------------------------------------------------------------------------------------------------------

`timescale 1ns/1ps

`ifdef HAVE_SIM_CONFIG
`include "sim_config.vh"
`endif

`ifndef NEAR
`define NEAR 0
`endif
`ifndef CTRL_IMG
`define CTRL_IMG    "./images/test008.pgm"
`endif
`ifndef CTRL_GOLDEN
`define CTRL_GOLDEN "./golden/near0_w16384/test008.jls"
`endif

module tb_axi_jls_ctrl ();

localparam integer MAX_WIDTH = 256;
localparam integer IN_DEPTH  = 8;
localparam integer OUT_DEPTH = 128;
localparam integer BURST     = 32;   // > IN_DEPTH so bursts stall mid-way

localparam [31:0] REG_CTRL   = 32'h0000;
localparam [31:0] REG_WIDTH  = 32'h0004;
localparam [31:0] REG_HEIGHT = 32'h0008;
localparam [31:0] REG_STATUS = 32'h000C;
localparam [31:0] REG_MAXW   = 32'h0014;
localparam [31:0] REG_FIFOD  = 32'h0018;
localparam [31:0] PIX_IN     = 32'h1000;
localparam [31:0] OUT_BASE   = 32'h2000;

reg clk = 1'b0;
reg rst = 1'b1;
always #10 clk = ~clk;

// AXI master signals
reg  [31:0] awaddr = 0;  reg [7:0] awlen = 0;  reg awvalid = 0;  wire awready;
reg  [31:0] wdata  = 0;  reg [3:0] wstrb = 0;  reg wlast   = 0;  reg  wvalid = 0;  wire wready;
wire [1:0]  bresp;       wire bvalid;          reg bready  = 0;
reg  [31:0] araddr = 0;  reg [7:0] arlen = 0;  reg arvalid = 0;  wire arready;
wire [31:0] rdata;       wire [1:0] rresp;     wire rlast;       wire rvalid;  reg rready = 0;
wire [3:0]  leds;

axi_jls_ctrl #(
    .NEAR            ( `NEAR ),
    .MAX_WIDTH       ( MAX_WIDTH ),
    .IN_FIFO_DEPTH   ( IN_DEPTH ),
    .OUT_FIFO_DEPTH  ( OUT_DEPTH ),
    .SOF_HOLD_CYCLES ( 400 )
) dut (
    .clk(clk), .rst(rst),
    .s_awaddr(awaddr), .s_awlen(awlen), .s_awsize(3'd2), .s_awburst(2'd1), .s_awvalid(awvalid), .s_awready(awready),
    .s_wdata(wdata), .s_wstrb(wstrb), .s_wlast(wlast), .s_wvalid(wvalid), .s_wready(wready),
    .s_bresp(bresp), .s_bvalid(bvalid), .s_bready(bready),
    .s_araddr(araddr), .s_arlen(arlen), .s_arsize(3'd2), .s_arburst(2'd1), .s_arvalid(arvalid), .s_arready(arready),
    .s_rdata(rdata), .s_rresp(rresp), .s_rlast(rlast), .s_rvalid(rvalid), .s_rready(rready),
    .leds(leds)
);


// -------------------------------------------------------------------------------------------------------------------
//   continuous checks
// -------------------------------------------------------------------------------------------------------------------
integer acc_pix   = 0;   // pixel beats accepted on the AXI write channel
integer fifo_wr   = 0;   // pixels actually stored in the input FIFO
integer max_in    = 0;   // peak input FIFO occupancy

always @(posedge clk) if(!rst) begin
    if(wvalid && wready && dut.w_in_pix && wstrb[0])
        acc_pix = acc_pix + 1;
    if(dut.u_in_fifo.do_wr)
        fifo_wr = fifo_wr + 1;
    if(dut.in_count > max_in)
        max_in = dut.in_count;
    if(dut.in_wr_en && dut.in_full)
        $fatal(1, "*** accepted pixel dropped: input FIFO full at t=%0t ***", $time);
    if(dut.o_e && dut.out_full)
        $fatal(1, "*** encoder output dropped: output FIFO full at t=%0t ***", $time);
    if(dut.status_word[19:8] !== dut.in_count || dut.status_word[31:20] !== dut.out_count)
        $fatal(1, "*** STATUS count fields do not match the FIFO counters ***");
end


// -------------------------------------------------------------------------------------------------------------------
//   AXI master tasks (signals change after the clock edge; handshakes are sampled at the edge)
// -------------------------------------------------------------------------------------------------------------------
task axi_write;
    input [31:0] addr;
    input [31:0] data;
begin
    @(posedge clk);
    awaddr <= addr; awlen <= 8'd0; awvalid <= 1'b1;
    @(posedge clk); while(!awready) @(posedge clk);
    awvalid <= 1'b0;
    wdata <= data; wstrb <= 4'hF; wlast <= 1'b1; wvalid <= 1'b1;
    @(posedge clk); while(!wready) @(posedge clk);
    wvalid <= 1'b0; wlast <= 1'b0; bready <= 1'b1;
    @(posedge clk); while(!bvalid) @(posedge clk);
    bready <= 1'b0;
end
endtask

reg [31:0] rd_word;
task axi_read;
    input [31:0] addr;
begin
    @(posedge clk);
    araddr <= addr; arlen <= 8'd0; arvalid <= 1'b1;
    @(posedge clk); while(!arready) @(posedge clk);
    arvalid <= 1'b0; rready <= 1'b1;
    @(posedge clk); while(!rvalid) @(posedge clk);
    rd_word = rdata;
    rready <= 1'b0;
end
endtask


// -------------------------------------------------------------------------------------------------------------------
//   image
// -------------------------------------------------------------------------------------------------------------------
reg [7:0] img [0:1048575];
integer w = 0, h = 0;

task load_img;
    reg [256*8-1:0] line;
    integer linelen, depth, n, fp, i, c;
begin
    fp = $fopen(`CTRL_IMG, "rb");
    if(fp == 0) $fatal(1, "*** could not open %0s ***", `CTRL_IMG);
    linelen = $fgets(line, fp);
    n = $fgets(line, fp);
    n = $sscanf(line, "%d%d", w, h);
    if(n == 1) begin n = $fgets(line, fp); n = $sscanf(line, "%d", h); end
    n = $fgets(line, fp);
    n = $sscanf(line, "%d", depth);
    if(depth != 255 || w*h > 1048576) $fatal(1, "*** unsupported test image ***");
    for(i=0; i<w*h; i=i+1) begin
        c = $fgetc(fp);
        if(c < 0) $fatal(1, "*** truncated test image ***");
        img[i] = c;
    end
    $fclose(fp);
end
endtask


// -------------------------------------------------------------------------------------------------------------------
//   writer: push all pixels in BURST-beat bursts, wvalid held high within a burst
// -------------------------------------------------------------------------------------------------------------------
task push_pixels;
    integer idx, n, k;
begin
    idx = 0;
    while(idx < w*h) begin
        n = (w*h - idx < BURST) ? (w*h - idx) : BURST;
        @(posedge clk);
        awaddr <= PIX_IN; awlen <= n - 1; awvalid <= 1'b1;
        @(posedge clk); while(!awready) @(posedge clk);
        awvalid <= 1'b0;
        for(k=0; k<n; k=k+1) begin
            wdata <= {24'd0, img[idx+k]}; wstrb <= 4'h1; wlast <= (k == n-1); wvalid <= 1'b1;
            @(posedge clk); while(!wready) @(posedge clk);
        end
        wvalid <= 1'b0; wlast <= 1'b0; bready <= 1'b1;
        @(posedge clk); while(!bvalid) @(posedge clk);
        bready <= 1'b0;
        idx = idx + n;
    end
end
endtask


// -------------------------------------------------------------------------------------------------------------------
//   reader: decode STATUS like run_demo.py, pop words with random gaps, compare to golden
// -------------------------------------------------------------------------------------------------------------------
integer gold_fp, out_words, mismatches, got_last;

task drain_output;
    integer n, k, g0, g1, gap;
begin
    got_last = 0; out_words = 0; mismatches = 0;
    gold_fp = $fopen(`CTRL_GOLDEN, "rb");
    if(gold_fp == 0) $fatal(1, "*** could not open %0s ***", `CTRL_GOLDEN);
    while(!got_last) begin
        axi_read(REG_STATUS);
        if(rd_word[4]) $fatal(1, "*** out_overflow reported ***");
        n = (rd_word >> 20) & 12'hFFF;           // out_count, as decoded by the host
        if(((rd_word >> 8) & 12'hFFF) > IN_DEPTH || n > OUT_DEPTH)
            $fatal(1, "*** STATUS counts out of range: 0x%08X ***", rd_word);
        for(k=0; k<n && !got_last; k=k+1) begin
            gap = {$random} % 24;
            repeat(gap) @(posedge clk);
            axi_read(OUT_BASE);
            g0 = $fgetc(gold_fp);
            g1 = $fgetc(gold_fp);
            if(g0 !== rd_word[7:0] || g1 !== rd_word[15:8]) begin
                if(mismatches == 0)
                    $display("  [MISMATCH] word %0d: got 0x%04X expected 0x%02X%02X",
                             out_words, rd_word[15:0], g1 & 8'hFF, g0 & 8'hFF);
                mismatches = mismatches + 1;
            end
            out_words = out_words + 1;
            got_last = rd_word[16];
        end
    end
    if($fgetc(gold_fp) != -1) begin
        $display("  [MISMATCH] output ended after %0d words, golden is longer", out_words);
        mismatches = mismatches + 1;
    end
    $fclose(gold_fp);
end
endtask


// -------------------------------------------------------------------------------------------------------------------
//   main sequence
// -------------------------------------------------------------------------------------------------------------------
task expect_dim_error;
    input [31:0] width;
    input [31:0] height;
begin
    axi_write(REG_WIDTH, width);
    axi_write(REG_HEIGHT, height);
    axi_write(REG_CTRL, 32'h2);                       // sof_strobe
    axi_read(REG_STATUS);
    if(!rd_word[5] || rd_word[0] || dut.feed_state != 2'd0)
        $fatal(1, "*** %0dx%0d was not rejected (STATUS 0x%08X) ***", width, height, rd_word);
    axi_write(REG_CTRL, 32'h4);                       // done_clear also clears dim_error
    axi_read(REG_STATUS);
    if(rd_word[5]) $fatal(1, "*** dim_error not cleared ***");
    $display("  [PASS] %0dx%0d rejected", width, height);
end
endtask

initial begin
    load_img;
    $display("  ctrl test image %0s (%0dx%0d), NEAR=%0d", `CTRL_IMG, w, h, `NEAR);
    repeat(5) @(posedge clk);
    rst <= 1'b0;
    repeat(5) @(posedge clk);

    axi_read(REG_MAXW);
    if(rd_word != MAX_WIDTH) $fatal(1, "*** MAX_WIDTH reg 0x%08X ***", rd_word);
    axi_read(REG_FIFOD);
    if(rd_word != ((OUT_DEPTH << 16) | IN_DEPTH)) $fatal(1, "*** FIFO_DEPTH reg 0x%08X ***", rd_word);

    axi_write(REG_HEIGHT, 16384);
    axi_read(REG_HEIGHT);
    if(rd_word != 16384) $fatal(1, "*** HEIGHT readback %0d, expected 16384 ***", rd_word);

    expect_dim_error(4, 10);
    expect_dim_error(MAX_WIDTH + 1, 10);
    expect_dim_error(16, 0);
    expect_dim_error(16, 16385);

    axi_write(REG_WIDTH, w);
    axi_write(REG_HEIGHT, h);
    axi_write(REG_CTRL, 32'h2);
    axi_read(REG_STATUS);
    if(rd_word[5] || !rd_word[0]) $fatal(1, "*** valid frame not started (STATUS 0x%08X) ***", rd_word);

    fork
        push_pixels;
        drain_output;
    join

    $display("");
    $display("=== AXI CTRL SUMMARY (NEAR=%0d) ===", `NEAR);
    $display("  pixels accepted : %0d", acc_pix);
    $display("  pixels stored   : %0d", fifo_wr);
    $display("  peak in_count   : %0d / %0d", max_in, IN_DEPTH);
    $display("  output words    : %0d", out_words);
    if(acc_pix != w*h || fifo_wr != w*h)
        $fatal(1, "*** pixel count mismatch: %0d accepted, %0d stored, %0d expected ***", acc_pix, fifo_wr, w*h);
    if(max_in != IN_DEPTH)
        $fatal(1, "*** input FIFO never filled; back-pressure path not exercised ***");
    if(mismatches != 0)
        $fatal(1, "*** %0d output mismatches against %0s ***", mismatches, `CTRL_GOLDEN);
    $display("*** ALL CHECKS PASSED ***");
    $finish;
end

initial begin
    #(64'd2_000_000_000);
    $fatal(1, "*** TIMEOUT ***");
end

endmodule
