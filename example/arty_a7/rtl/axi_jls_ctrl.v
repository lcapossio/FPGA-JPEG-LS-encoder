// SPDX-License-Identifier: GPL-3.0-only
// Copyright (c) 2026 Leonardo Capossio - bard0 design - hello@bard0.com

// AXI4 slave + jls_encoder wrapper for the Arty demo.
//
// Register map (byte-addressed, 32-bit regs, little-endian):
//   0x0000 CTRL      W/O pulse: [0]=soft_reset  [1]=sof_strobe  [2]=done_clear
//   0x0004 WIDTH     R/W [14:0] image width  (5..MAX_WIDTH)
//   0x0008 HEIGHT    R/W [14:0] image height (1..16384)
//   0x000C STATUS    R/O  [0]=busy          [1]=done_sticky
//                         [2]=in_full       [3]=out_empty
//                         [4]=out_overflow_sticky
//                         [5]=dim_error_sticky (sof_strobe with WIDTH/HEIGHT
//                             out of range; the frame was not started)
//                         [19:8]=in_count   [31:20]=out_count
//   0x0010 NEAR      R/O  [2:0] compile-time NEAR value
//   0x0014 MAX_WIDTH R/O  [15:0] compile-time encoder MAX_WIDTH
//   0x0018 FIFO_DEPTH R/O [15:0]=IN_FIFO_DEPTH  [31:16]=OUT_FIFO_DEPTH
//
//   0x1000..0x1FFC PIX_IN window (W/O): push 1 pixel per AXI beat,
//                  from wdata[7:0] (wstrb[0] must be set).
//   0x2000..0x2FFC OUT_DATA window (R/O): pop 1 encoded word per AXI beat,
//                  [15:0]=o_data, [16]=o_last.
//
// The feeder holds i_sof for SOF_HOLD_CYCLES after a sof_strobe, then
// streams pixels from the input FIFO. When the output FIFO nears full,
// it gates i_e so the encoder stalls rather than overrunning the FIFO
// (JTAG readback is much slower than the encoder).

`timescale 1ns/1ps

module axi_jls_ctrl #(
    parameter integer NEAR            = 0,
    parameter integer MAX_WIDTH       = 1920,
    parameter integer IN_FIFO_DEPTH   = 1024,  // power of 2, <= 2048 (12-bit STATUS field)
    parameter integer OUT_FIFO_DEPTH  = 256,   // power of 2, 128..2048
    parameter integer SOF_HOLD_CYCLES = 512
) (
    input  wire        clk,
    input  wire        rst,

    // AXI4 write address
    input  wire [31:0] s_awaddr,
    input  wire [7:0]  s_awlen,
    input  wire [2:0]  s_awsize,
    input  wire [1:0]  s_awburst,
    input  wire        s_awvalid,
    output reg         s_awready,

    // AXI4 write data
    input  wire [31:0] s_wdata,
    input  wire [3:0]  s_wstrb,
    input  wire        s_wlast,
    input  wire        s_wvalid,
    output reg         s_wready,

    // AXI4 write response
    output reg  [1:0]  s_bresp,
    output reg         s_bvalid,
    input  wire        s_bready,

    // AXI4 read address
    input  wire [31:0] s_araddr,
    input  wire [7:0]  s_arlen,
    input  wire [2:0]  s_arsize,
    input  wire [1:0]  s_arburst,
    input  wire        s_arvalid,
    output reg         s_arready,

    // AXI4 read data
    output reg  [31:0] s_rdata,
    output reg  [1:0]  s_rresp,
    output reg         s_rlast,
    output reg         s_rvalid,
    input  wire        s_rready,

    // Status LEDs (active-high)
    output wire [3:0]  leds
);

    wire _unused = &{1'b0, s_awsize, s_awburst, s_arsize, s_arburst, 1'b0};

    // STATUS packs in_count/out_count into 12-bit fields, and the encoder
    // drain reserve below is 64 words. Out-of-range depths fail elaboration
    // on this deliberately undefined module.
    generate
        if (IN_FIFO_DEPTH > 2048 || OUT_FIFO_DEPTH > 2048 || OUT_FIFO_DEPTH < 128 ||
            MAX_WIDTH < 5 || MAX_WIDTH > 16384) begin : g_bad_param
            axi_jls_ctrl_invalid_parameter u_invalid_parameter ();
        end
    endgenerate

    // -----------------------------------------------------------------------
    // Control registers
    // -----------------------------------------------------------------------
    reg [14:0] r_width, r_height;
    reg        soft_reset;
    reg        sof_strobe;
    reg        done_clear;
    reg        done_sticky;
    reg        busy;
    reg        out_overflow_sticky;
    reg        dim_error_sticky;

    wire rst_any = rst | soft_reset;

    // The core cannot encode widths below 5, and crops rows wider than
    // MAX_WIDTH; the demo verifies a lossless roundtrip of the whole image,
    // so both cases are rejected instead of started.
    wire dims_ok = (r_width  >= 15'd5) && (r_width  <= MAX_WIDTH) &&
                   (r_height >= 15'd1) && (r_height <= 15'd16384);

    // -----------------------------------------------------------------------
    // FIFOs
    // -----------------------------------------------------------------------
    wire                               in_full, in_empty;
    wire [$clog2(IN_FIFO_DEPTH):0]     in_count;
    wire [7:0]                         in_rd_data;
    reg  [7:0]                         in_wr_data;
    reg                                in_wr_en;
    wire                               in_rd_en;

    sync_fifo #(.WIDTH(8), .DEPTH(IN_FIFO_DEPTH)) u_in_fifo (
        .clk(clk), .rst(rst_any),
        .wr_data(in_wr_data), .wr_en(in_wr_en), .full(in_full),
        .rd_data(in_rd_data), .rd_en(in_rd_en), .empty(in_empty),
        .count(in_count)
    );

    wire                                out_full, out_empty;
    wire [$clog2(OUT_FIFO_DEPTH):0]     out_count;
    wire [16:0]                         out_rd_data;
    reg  [16:0]                         out_wr_data;
    reg                                 out_wr_en;
    reg                                 out_rd_en;

    sync_fifo #(.WIDTH(17), .DEPTH(OUT_FIFO_DEPTH)) u_out_fifo (
        .clk(clk), .rst(rst_any),
        .wr_data(out_wr_data), .wr_en(out_wr_en), .full(out_full),
        .rd_data(out_rd_data), .rd_en(out_rd_en), .empty(out_empty),
        .count(out_count)
    );

    // -----------------------------------------------------------------------
    // Feeder: i_sof hold → pixel streaming with out-FIFO back-pressure
    // -----------------------------------------------------------------------
    localparam [1:0] F_IDLE = 2'd0,
                     F_SOF  = 2'd1,
                     F_FEED = 2'd2;
    reg [1:0]  feed_state;
    reg [15:0] sof_cnt;

    reg        enc_sof;
    reg        enc_e;
    reg  [7:0] enc_x;
    reg [13:0] enc_w, enc_h;

    // The encoder has a long output pipeline and no downstream ready/stall.
    // Stop feeding well before the FIFO is actually full so in-flight words
    // can drain into the FIFO without being dropped.
    wire out_backpressure = (out_count >= (OUT_FIFO_DEPTH - 64));
    wire feed_go = (feed_state == F_FEED) && !in_empty && !out_backpressure;

    assign in_rd_en = feed_go;

    // width-1 / height-1 for the core; dims_ok bounds both to 1..16384
    wire [14:0] width_m1  = r_width  - 15'd1;
    wire [14:0] height_m1 = r_height - 15'd1;

    always @(posedge clk) begin
        if(rst_any) begin
            feed_state <= F_IDLE;
            sof_cnt    <= 0;
            enc_sof    <= 1'b0;
            enc_e      <= 1'b0;
            enc_x      <= 8'd0;
            enc_w      <= 14'd0;
            enc_h      <= 14'd0;
        end else begin
            enc_sof <= 1'b0;
            enc_e   <= feed_go;      // pixel is valid 1 cycle after pop
            enc_x   <= in_rd_data;   // latch show-ahead data

            case(feed_state)
                F_IDLE: begin
                    if(sof_strobe && dims_ok) begin
                        enc_w      <= width_m1[13:0];
                        enc_h      <= height_m1[13:0];
                        sof_cnt    <= SOF_HOLD_CYCLES[15:0];
                        feed_state <= F_SOF;
                    end
                end
                F_SOF: begin
                    enc_sof <= 1'b1;
                    if(sof_cnt == 16'd0)
                        feed_state <= F_FEED;
                    else
                        sof_cnt <= sof_cnt - 16'd1;
                end
                F_FEED: begin
                    // nothing extra — feed_go drives enc_e/enc_x
                end
                default: feed_state <= F_IDLE;
            endcase
        end
    end

    // -----------------------------------------------------------------------
    // jls_encoder
    // -----------------------------------------------------------------------
    wire        o_e, o_last;
    wire [15:0] o_data;

    jls_encoder #(.NEAR(NEAR), .MAX_WIDTH(MAX_WIDTH)) u_enc (
        .rstn  (~rst_any),
        .clk   (clk),
        .i_sof (enc_sof),
        .i_w   (enc_w),
        .i_h   (enc_h),
        .i_e   (enc_e),
        .i_x   (enc_x),
        .o_e   (o_e),
        .o_last(o_last),
        .o_data(o_data)
    );

    always @(*) begin
        out_wr_en   = o_e;
        out_wr_data = {o_last, o_data};
    end

    // -----------------------------------------------------------------------
    // busy / done
    // -----------------------------------------------------------------------
    always @(posedge clk) begin
        if(rst_any) begin
            busy        <= 1'b0;
            done_sticky <= 1'b0;
            out_overflow_sticky <= 1'b0;
            dim_error_sticky    <= 1'b0;
        end else begin
            if(sof_strobe &&  dims_ok) busy             <= 1'b1;
            if(sof_strobe && !dims_ok) dim_error_sticky <= 1'b1;
            if(o_e && o_last)  begin busy <= 1'b0; done_sticky <= 1'b1; end
            if(done_clear)     begin done_sticky <= 1'b0; dim_error_sticky <= 1'b0; end
            if(o_e && out_full)   out_overflow_sticky <= 1'b1;
        end
    end

    // -----------------------------------------------------------------------
    // AXI4 write channel
    // -----------------------------------------------------------------------
    localparam [1:0] W_IDLE = 2'd0, W_DATA = 2'd1, W_RESP = 2'd2;
    reg [1:0]  w_state;
    reg [31:0] w_addr;

    wire w_in_pix = (w_addr[15:12] == 4'h1);
    wire w_in_reg = (w_addr[15:12] == 4'h0);

    // s_wready is registered and a pixel reaches the FIFO through the
    // registered in_wr_en, so in_count trails the accepted beats by up to
    // two. Only offer wready for a pixel beat when a FIFO slot is free after
    // counting the queued write and the beat accepted this cycle; reads by
    // the feeder only free slots. Register beats are never stalled.
    wire        w_beat_pix  = s_wvalid && s_wready && w_in_pix;
    wire [12:0] in_claimed  = in_count + in_wr_en + w_beat_pix;
    wire        in_room     = (in_claimed < IN_FIFO_DEPTH);
    wire [31:0] w_next_addr = (s_wvalid && s_wready) ? (w_addr + 32'd4) : w_addr;
    wire        w_next_pix  = (w_next_addr[15:12] == 4'h1);
    wire        aw_pix      = (s_awaddr[15:12] == 4'h1);

    always @(posedge clk) begin
        if(rst_any) begin
            w_state    <= W_IDLE;
            s_awready  <= 1'b1;
            s_wready   <= 1'b0;
            s_bvalid   <= 1'b0;
            s_bresp    <= 2'b00;
            w_addr     <= 32'd0;
            in_wr_en   <= 1'b0;
            in_wr_data <= 8'd0;
            soft_reset <= 1'b0;
            sof_strobe <= 1'b0;
            done_clear <= 1'b0;
            r_width    <= 15'd0;
            r_height   <= 15'd0;
        end else begin
            // One-cycle default pulses
            in_wr_en   <= 1'b0;
            soft_reset <= 1'b0;
            sof_strobe <= 1'b0;
            done_clear <= 1'b0;

            case(w_state)
                W_IDLE: begin
                    s_bvalid <= 1'b0;
                    if(s_awvalid && s_awready) begin
                        w_addr    <= s_awaddr;
                        s_awready <= 1'b0;
                        s_wready  <= ~aw_pix | in_room;
                        w_state   <= W_DATA;
                    end
                end
                W_DATA: begin
                    s_wready <= ~w_next_pix | in_room;

                    if(s_wvalid && s_wready) begin
                        // Decode and act on this beat
                        if(w_in_pix) begin
                            in_wr_data <= s_wdata[7:0];
                            in_wr_en   <= s_wstrb[0];
                        end else if(w_in_reg) begin
                            case(w_addr[7:0])
                                8'h00: if(s_wstrb[0]) begin
                                    soft_reset <= s_wdata[0];
                                    sof_strobe <= s_wdata[1];
                                    done_clear <= s_wdata[2];
                                end
                                8'h04: if(s_wstrb[0] || s_wstrb[1])
                                           r_width  <= s_wdata[14:0];
                                8'h08: if(s_wstrb[0] || s_wstrb[1])
                                           r_height <= s_wdata[14:0];
                                default: ;
                            endcase
                        end

                        w_addr <= w_addr + 32'd4;
                        if(s_wlast) begin
                            s_wready <= 1'b0;
                            s_bvalid <= 1'b1;
                            s_bresp  <= 2'b00;
                            w_state  <= W_RESP;
                        end
                    end
                end
                W_RESP: begin
                    if(s_bvalid && s_bready) begin
                        s_bvalid  <= 1'b0;
                        s_awready <= 1'b1;
                        w_state   <= W_IDLE;
                    end
                end
                default: w_state <= W_IDLE;
            endcase
        end
    end

    // -----------------------------------------------------------------------
    // AXI4 read channel
    // -----------------------------------------------------------------------
    localparam [1:0] R_IDLE = 2'd0, R_DATA = 2'd1;
    reg [1:0]  r_state;
    reg [31:0] r_addr;
    reg [7:0]  r_len, r_beat;

    wire r_in_pix = (r_addr[15:12] == 4'h2);

    reg [31:0] status_word;
    always @(*) begin
        status_word = 32'd0;
        status_word[0]     = busy;
        status_word[1]     = done_sticky;
        status_word[2]     = in_full;
        status_word[3]     = out_empty;
        status_word[4]     = out_overflow_sticky;
        status_word[5]     = dim_error_sticky;
        // disjoint 12-bit count fields; depths are limited to 2048 above
        status_word[19:8]  = in_count;
        status_word[31:20] = out_count;
    end

    always @(posedge clk) begin
        if(rst_any) begin
            r_state   <= R_IDLE;
            s_arready <= 1'b1;
            s_rvalid  <= 1'b0;
            s_rlast   <= 1'b0;
            s_rdata   <= 32'd0;
            s_rresp   <= 2'b00;
            r_addr    <= 32'd0;
            r_len     <= 8'd0;
            r_beat    <= 8'd0;
            out_rd_en <= 1'b0;
        end else begin
            out_rd_en <= 1'b0;
            case(r_state)
                R_IDLE: begin
                    s_rvalid <= 1'b0;
                    s_rlast  <= 1'b0;
                    if(s_arvalid && s_arready) begin
                        r_addr    <= s_araddr;
                        r_len     <= s_arlen;
                        r_beat    <= 8'd0;
                        s_arready <= 1'b0;
                        r_state   <= R_DATA;
                    end
                end
                R_DATA: begin
                    if(!s_rvalid) begin
                        // Present next beat
                        if(r_in_pix) begin
                            if(!out_empty) begin
                                s_rdata   <= {15'd0, out_rd_data};
                                out_rd_en <= 1'b1;
                                s_rvalid  <= 1'b1;
                                s_rresp   <= 2'b00;
                                s_rlast   <= (r_beat == r_len);
                            end
                        end else begin
                            case(r_addr[7:0])
                                8'h00: s_rdata <= 32'd0;
                                8'h04: s_rdata <= {17'd0, r_width};
                                8'h08: s_rdata <= {17'd0, r_height};
                                8'h0C: s_rdata <= status_word;
                                8'h10: s_rdata <= NEAR;
                                8'h14: s_rdata <= MAX_WIDTH;
                                8'h18: s_rdata <= (OUT_FIFO_DEPTH << 16) | IN_FIFO_DEPTH;
                                default: s_rdata <= 32'hDEAD_BEEF;
                            endcase
                            s_rvalid <= 1'b1;
                            s_rresp  <= 2'b00;
                            s_rlast  <= (r_beat == r_len);
                        end
                    end else if(s_rready) begin
                        // Beat accepted
                        if(r_beat == r_len) begin
                            s_rvalid  <= 1'b0;
                            s_rlast   <= 1'b0;
                            s_arready <= 1'b1;
                            r_state   <= R_IDLE;
                        end else begin
                            r_beat   <= r_beat + 8'd1;
                            r_addr   <= r_addr + 32'd4;
                            s_rvalid <= 1'b0;
                            s_rlast  <= 1'b0;
                        end
                    end
                end
                default: r_state <= R_IDLE;
            endcase
        end
    end

    // -----------------------------------------------------------------------
    // LEDs: [0]=busy  [1]=done  [2]=in_empty  [3]=out_nonempty
    // -----------------------------------------------------------------------
    assign leds = {~out_empty, in_empty, done_sticky, busy};

endmodule
