// SPDX-License-Identifier: GPL-3.0-only
// Copyright (c) 2026 Leonardo Capossio - bard0 design - hello@bard0.com

//--------------------------------------------------------------------------------------------------------
// Module  : tb_axi_jls4_ctrl
// Type    : simulation, top
// Function: testbench for the four-encoder demo's AXI wrapper (example/de25_nano_x4/rtl/axi_jls4_ctrl.v).
//
//   1. WIDTH/HEIGHT validation: unsupported sizes set dim_error and do not start a frame.
//   2. Two images through PIX_IN (one width not a multiple of 4, one that is): words are pushed
//      in long bursts into a small input FIFO, so wready back-pressure is exercised, while a
//      slow reader with random gaps drains the four tile FIFOs, so every reader also stalls.
//   3. Two frames of the on-chip test pattern, taken from the encoder outputs directly.
//   4. Continuous checks: no dropped input word or output word, a reader only reads complete
//      rows, the writer never overwrites a row a reader still needs, and each encoder gets
//      exactly its tile's pixels.
//
// Every frame writes <OUT_DIR>/<name>_tile<t>.jls and <name>_regs.txt (TILE_W, BYTES, CRC per
// tile and CYCLES). run_sim.py --ctrl4 checks them against CharLS and zlib.crc32.
//
// Defines (run_sim.py --ctrl4 writes them into sim_config.vh):
//   NEAR, IMG_A, IMG_B (input .pgm), PAT1_W/H, PAT2_W/H, OUT_DIR
//--------------------------------------------------------------------------------------------------------

`timescale 1ns/1ps

`ifdef HAVE_SIM_CONFIG
`include "sim_config.vh"
`endif

`ifndef NEAR
`define NEAR 0
`endif
`ifndef IMG_A
`define IMG_A "./images/test008.pgm"
`endif
`ifndef IMG_B
`define IMG_B "./images/test009.pgm"
`endif
`ifndef PAT1_W
`define PAT1_W 256
`endif
`ifndef PAT1_H
`define PAT1_H 72
`endif
`ifndef PAT2_W
`define PAT2_W 250
`endif
`ifndef PAT2_H
`define PAT2_H 40
`endif
`ifndef OUT_DIR
`define OUT_DIR "./out/ctrl4"
`endif

module tb_axi_jls4_ctrl ();

localparam integer NT         = 4;
localparam integer MAX_TILE_W = 64;
localparam integer IN_DEPTH   = 8;
localparam integer OUT_DEPTH  = 128;
localparam integer BURST      = 32;   // > IN_DEPTH so bursts stall mid-way

localparam [31:0] REG_CTRL   = 32'h0000;
localparam [31:0] REG_WIDTH  = 32'h0004;
localparam [31:0] REG_HEIGHT = 32'h0008;
localparam [31:0] REG_STATUS = 32'h000C;
localparam [31:0] REG_MAXTW  = 32'h0014;
localparam [31:0] REG_FIFOD  = 32'h0018;
localparam [31:0] REG_MODE   = 32'h001C;
localparam [31:0] REG_CYCLES = 32'h0020;
localparam [31:0] REG_NTILES = 32'h0028;
localparam [31:0] REG_TILE   = 32'h0040;
localparam [31:0] PIX_IN     = 32'h1000;
localparam [31:0] OUT_BASE   = 32'h2000;

reg clk = 1'b0;
reg rst = 1'b1;
always #10 clk = ~clk;

reg  [31:0] awaddr = 0;  reg [7:0] awlen = 0;  reg awvalid = 0;  wire awready;
reg  [31:0] wdata  = 0;  reg [3:0] wstrb = 0;  reg wlast   = 0;  reg  wvalid = 0;  wire wready;
wire [1:0]  bresp;       wire bvalid;          reg bready  = 0;
reg  [31:0] araddr = 0;  reg [7:0] arlen = 0;  reg arvalid = 0;  wire arready;
wire [31:0] rdata;       wire [1:0] rresp;     wire rlast;       wire rvalid;  reg rready = 0;
wire [3:0]  leds;

axi_jls4_ctrl #(
    .NEAR            ( `NEAR ),
    .MAX_TILE_W      ( MAX_TILE_W ),
    .IN_FIFO_DEPTH   ( IN_DEPTH ),
    .OUT_FIFO_DEPTH  ( OUT_DEPTH ),
    .SOF_HOLD_CYCLES ( 400 ),
    .CLK_KHZ         ( 50000 )
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
integer acc_words = 0;   // PIX_IN beats accepted on the AXI write channel
integer max_in    = 0;   // peak input FIFO occupancy
integer bp_cycles = 0;   // reader cycles stalled by a nearly full output FIFO

always @(posedge clk) if(!rst) begin
    if(wvalid && wready && dut.w_in_pix)
        acc_words = acc_words + 1;
    if(dut.in_count > max_in)
        max_in = dut.in_count;
    if(dut.in_wr_en && dut.in_full)
        $fatal(1, "*** accepted word dropped: input FIFO full at t=%0t ***", $time);
    if(|dut.out_overflow)
        $fatal(1, "*** encoder output dropped: output FIFO full at t=%0t ***", $time);
end

// Per tile: reader/writer row ordering, pixel count, and (pattern mode) a tap on the encoder output.
integer tap_fd [0:NT-1];
integer enc_pix [0:NT-1];
reg     tap_en = 1'b0;

genvar t;
generate for(t=0; t<NT; t=t+1) begin : g_chk
    initial begin tap_fd[t] = 0; enc_pix[t] = 0; end
    always @(posedge clk) if(!rst) begin
        if(dut.g_tile[t].go && !(dut.g_tile[t].rd_row < dut.wr_row))
            $fatal(1, "*** tile %0d read row %0d before it was written ***", t, dut.g_tile[t].rd_row);
        if(dut.wr_fire && (dut.wr_row - dut.g_tile[t].rd_row >= 2))
            $fatal(1, "*** writer overwrote a row tile %0d still needs ***", t);
        if(dut.g_tile[t].bp && dut.feeding)
            bp_cycles = bp_cycles + 1;
        if(dut.g_tile[t].enc_e)
            enc_pix[t] = enc_pix[t] + 1;
        if(tap_en && dut.g_tile[t].o_e)
            $fwrite(tap_fd[t], "%c%c", dut.g_tile[t].o_data[7:0], dut.g_tile[t].o_data[15:8]);
    end
end endgenerate


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
//   image and PIX_IN words (each row padded to ceil(w/4) words, pixel x in byte lane x%4)
// -------------------------------------------------------------------------------------------------------------------
reg [7:0]  img   [0:1048575];
reg [31:0] words [0:262143];
integer w = 0, h = 0, nwords = 0;

task load_img;
    input [256*8-1:0] path;
    reg [256*8-1:0] line;
    integer linelen, depth, n, fp, i, c, x, y, wpr;
begin
    fp = $fopen(path, "rb");
    if(fp == 0) $fatal(1, "*** could not open %0s ***", path);
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
    wpr = (w + 3) / 4;
    nwords = wpr * h;
    for(y=0; y<h; y=y+1)
        for(i=0; i<wpr; i=i+1) begin
            words[y*wpr+i] = 32'd0;
            for(x=4*i; x<4*i+4 && x<w; x=x+1)
                words[y*wpr+i][8*(x-4*i) +: 8] = img[y*w+x];
        end
end
endtask

task push_words;
    integer idx, n, k;
begin
    idx = 0;
    while(idx < nwords) begin
        n = (nwords - idx < BURST) ? (nwords - idx) : BURST;
        @(posedge clk);
        awaddr <= PIX_IN; awlen <= n - 1; awvalid <= 1'b1;
        @(posedge clk); while(!awready) @(posedge clk);
        awvalid <= 1'b0;
        for(k=0; k<n; k=k+1) begin
            wdata <= words[idx+k]; wstrb <= 4'hF; wlast <= (k == n-1); wvalid <= 1'b1;
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
//   reader: drain all four tile FIFOs round-robin, with random gaps, into per-tile files
// -------------------------------------------------------------------------------------------------------------------
task drain_tiles;
    input [256*8-1:0] name;
    integer tt, n, k, gap, fd [0:NT-1], last [0:NT-1], left;
    reg [256*8-1:0] fname;
begin
    for(tt=0; tt<NT; tt=tt+1) begin
        $sformat(fname, "%0s/%0s_tile%0d.jls", `OUT_DIR, name, tt);
        fd[tt] = $fopen(fname, "wb");
        if(fd[tt] == 0) $fatal(1, "*** could not create %0s ***", fname);
        last[tt] = 0;
    end
    left = NT;
    while(left > 0) begin
        for(tt=0; tt<NT; tt=tt+1) if(!last[tt]) begin
            axi_read(REG_TILE + 16*tt + 4);
            n = rd_word & 16'hFFFF;
            if(n > OUT_DEPTH) $fatal(1, "*** tile %0d OUT_COUNT %0d out of range ***", tt, n);
            for(k=0; k<n && !last[tt]; k=k+1) begin
                gap = {$random} % 24;
                repeat(gap) @(posedge clk);
                axi_read(OUT_BASE + 32'h400*tt);
                $fwrite(fd[tt], "%c%c", rd_word[7:0], rd_word[15:8]);
                if(rd_word[16]) begin last[tt] = 1; left = left - 1; end
            end
        end
    end
    for(tt=0; tt<NT; tt=tt+1) $fclose(fd[tt]);
end
endtask


// -------------------------------------------------------------------------------------------------------------------
//   frames
// -------------------------------------------------------------------------------------------------------------------
task start_frame;
    input [31:0] fw;
    input [31:0] fh;
    input        mode;
    integer tt;
begin
    axi_write(REG_CTRL, 32'h1);                       // soft reset: clears FIFOs and registers
    axi_write(REG_MODE, mode);
    axi_write(REG_WIDTH, fw);
    axi_write(REG_HEIGHT, fh);
    for(tt=0; tt<NT; tt=tt+1) enc_pix[tt] = 0;
    axi_write(REG_CTRL, 32'h2);                       // start
    axi_read(REG_STATUS);
    if(rd_word[5] || !rd_word[0]) $fatal(1, "*** %0dx%0d not started (STATUS 0x%08X) ***", fw, fh, rd_word);
end
endtask

task finish_frame;
    input [256*8-1:0] name;
    input [31:0] fw;
    input [31:0] fh;
    integer fd, tt, tw;
    reg [256*8-1:0] fname;
begin
    axi_read(REG_STATUS);
    while(!rd_word[1]) axi_read(REG_STATUS);
    if(rd_word[0] || rd_word[4]) $fatal(1, "*** frame end STATUS 0x%08X ***", rd_word);
    $sformat(fname, "%0s/%0s_regs.txt", `OUT_DIR, name);
    fd = $fopen(fname, "w");
    for(tt=0; tt<NT; tt=tt+1) begin
        axi_read(REG_TILE + 16*tt);
        tw = rd_word;
        if(enc_pix[tt] != tw * fh)
            $fatal(1, "*** tile %0d got %0d pixels, expected %0d ***", tt, enc_pix[tt], tw * fh);
        $fwrite(fd, "tile %0d width %0d", tt, tw);
        axi_read(REG_TILE + 16*tt + 8);
        $fwrite(fd, " bytes %0d", rd_word);
        axi_read(REG_TILE + 16*tt + 12);
        $fwrite(fd, " crc %0d\n", rd_word);
    end
    axi_read(REG_CYCLES);
    $fwrite(fd, "cycles %0d\n", rd_word);
    $fclose(fd);
    $display("  [DONE] %0s %0dx%0d: %0d cycles", name, fw, fh, rd_word);
end
endtask

task image_frame;
    input [256*8-1:0] name;
    input [256*8-1:0] path;
begin
    load_img(path);
    $display("  %0s: %0s (%0dx%0d), %0d words", name, path, w, h, nwords);
    acc_words = 0;
    start_frame(w, h, 1'b0);
    fork
        push_words;
        drain_tiles(name);
    join
    if(acc_words != nwords) $fatal(1, "*** %0d words accepted, expected %0d ***", acc_words, nwords);
    finish_frame(name, w, h);
end
endtask

task pattern_frame;
    input [256*8-1:0] name;
    input [31:0] fw;
    input [31:0] fh;
    integer tt;
    reg [256*8-1:0] fname;
begin
    for(tt=0; tt<NT; tt=tt+1) begin
        $sformat(fname, "%0s/%0s_tile%0d.jls", `OUT_DIR, name, tt);
        tap_fd[tt] = $fopen(fname, "wb");
    end
    tap_en = 1'b1;
    start_frame(fw, fh, 1'b1);
    finish_frame(name, fw, fh);
    tap_en = 1'b0;
    for(tt=0; tt<NT; tt=tt+1) $fclose(tap_fd[tt]);
end
endtask

task expect_dim_error;
    input [31:0] width;
    input [31:0] height;
begin
    axi_write(REG_WIDTH, width);
    axi_write(REG_HEIGHT, height);
    axi_write(REG_CTRL, 32'h2);
    axi_read(REG_STATUS);
    if(!rd_word[5] || rd_word[0])
        $fatal(1, "*** %0dx%0d was not rejected (STATUS 0x%08X) ***", width, height, rd_word);
    axi_write(REG_CTRL, 32'h4);
    $display("  [PASS] %0dx%0d rejected", width, height);
end
endtask


// -------------------------------------------------------------------------------------------------------------------
//   main sequence
// -------------------------------------------------------------------------------------------------------------------
initial begin
    $display("  four-tile ctrl testbench, NEAR=%0d, MAX_TILE_W=%0d", `NEAR, MAX_TILE_W);
    repeat(5) @(posedge clk);
    rst <= 1'b0;
    repeat(5) @(posedge clk);

    axi_read(REG_MAXTW);
    if(rd_word != MAX_TILE_W) $fatal(1, "*** MAX_TILE_W reg 0x%08X ***", rd_word);
    axi_read(REG_FIFOD);
    if(rd_word != ((OUT_DEPTH << 16) | IN_DEPTH)) $fatal(1, "*** FIFO_DEPTH reg 0x%08X ***", rd_word);
    axi_read(REG_NTILES);
    if(rd_word != NT) $fatal(1, "*** NTILES reg 0x%08X ***", rd_word);

    expect_dim_error(19, 10);                         // tiles narrower than 5
    expect_dim_error(4*MAX_TILE_W + 4, 10);           // tiles wider than MAX_TILE_W
    expect_dim_error(4*MAX_TILE_W - 1, 10);           // last tile 66 > 64 from the remainder
    expect_dim_error(64, 0);
    expect_dim_error(64, 16385);

    image_frame("imgA", `IMG_A);
    image_frame("imgB", `IMG_B);
    pattern_frame("pat1", `PAT1_W, `PAT1_H);
    pattern_frame("pat2", `PAT2_W, `PAT2_H);

    $display("");
    $display("=== AXI CTRL4 SUMMARY (NEAR=%0d) ===", `NEAR);
    $display("  peak in_count      : %0d / %0d", max_in, IN_DEPTH);
    $display("  reader stall cycles: %0d", bp_cycles);
    if(max_in != IN_DEPTH)
        $fatal(1, "*** input FIFO never filled; back-pressure path not exercised ***");
    if(bp_cycles == 0)
        $fatal(1, "*** no reader stalled on a full output FIFO ***");
    $display("*** ALL CHECKS PASSED ***");
    $finish;
end

initial begin
    #(64'd20_000_000_000);
    $fatal(1, "*** TIMEOUT ***");
end

endmodule
