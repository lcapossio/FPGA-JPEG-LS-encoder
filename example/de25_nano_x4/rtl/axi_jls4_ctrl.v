// SPDX-License-Identifier: GPL-3.0-only
// Copyright (c) 2026 Leonardo Capossio - bard0 design - hello@bard0.com

// AXI4 slave + four jls_encoder instances: the four-tile demo.
//
// The image is split into four vertical tiles, each encoded by its own
// jls_encoder as an independent JPEG-LS image. With Wt = floor(W/4), tile t
// starts at column t*Wt; tiles 0..2 are Wt wide and tile 3 takes the
// remainder. The register map, the tiling and the input packing are
// documented in example/common/jls4_demo.py, which the host and the
// simulation share.
//
// Data path, four pixels per clock in, one pixel per clock per encoder:
//
//   PIX_IN FIFO --+                  +-> tile 0 row buffer -> reader -> encoder 0 -> CRC/count, FIFO
//   (32-bit words)|-> row writer ----+-> tile 1 ...
//   test pattern -+   (1 word/clk)   +-> tile 2 ...
//                                    +-> tile 3 ...
//
// The source delivers rows in raster order, four pixels per 32-bit word and
// each row padded to whole words. The writer stores every word into the
// row buffers of the tiles its pixels belong to; each row buffer holds two
// rows and is built from four byte-lane RAMs, so a word can straddle a tile
// boundary. A tile's reader streams a row into its encoder once the whole
// row is written, and the writer only reuses a row half after every reader
// has finished with it.
//
// In pattern mode (MODE[0]) the source is an on-chip test pattern and the
// encoder outputs only update the per-tile byte counts and CRC-32s, so the
// frame runs at the full rate and CYCLES measures it. Otherwise the outputs
// go to per-tile FIFOs that the host drains over AXI, and a tile's reader
// stalls while its FIFO is nearly full.

`timescale 1ns/1ps

module axi_jls4_ctrl #(
    parameter integer NEAR            = 0,
    parameter integer MAX_TILE_W      = 1024,  // encoder MAX_WIDTH: widest tile
    parameter integer IN_FIFO_DEPTH   = 512,   // 32-bit words, power of 2, <= 2048
    parameter integer OUT_FIFO_DEPTH  = 256,   // per tile, power of 2, 128..2048
    parameter integer SOF_HOLD_CYCLES = 512,
    parameter integer CLK_KHZ         = 0      // reported in CLK_KHZ
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

    localparam integer NT = 4;

    // Words a tile row can span: up to MAX_TILE_W pixels starting at any lane.
    localparam integer RB_WORDS = (MAX_TILE_W + 2) / 4 + 1;
    localparam integer RB_AW    = $clog2(RB_WORDS);

    wire _unused = &{1'b0, s_awsize, s_awburst, s_arsize, s_arburst, 1'b0};

    generate
        if (IN_FIFO_DEPTH > 2048 || OUT_FIFO_DEPTH > 2048 || OUT_FIFO_DEPTH < 128 ||
            MAX_TILE_W < 5 || MAX_TILE_W > 4096) begin : g_bad_param
            axi_jls4_ctrl_invalid_parameter u_invalid_parameter ();
        end
    endgenerate

    // -----------------------------------------------------------------------
    // Control registers
    // -----------------------------------------------------------------------
    reg [14:0] r_width, r_height;
    reg        r_mode;
    reg        soft_reset;
    reg        start_strobe;
    reg        done_clear;
    reg        done_sticky;
    reg        out_overflow_sticky;
    reg        dim_error_sticky;
    reg [31:0] cycles;

    wire rst_any = rst | soft_reset;

    // Tiles 0..2 are floor(W/4) wide and tile 3 takes the remainder; every
    // tile needs at least the encoder's 5 columns and at most MAX_TILE_W.
    wire [12:0] wt_c   = r_width[14:2];
    wire [14:0] last_c = r_width - 15'd3 * {2'd0, wt_c};
    wire dims_ok = (r_width  >= 15'd20) && (last_c <= MAX_TILE_W) &&
                   (r_height >= 15'd1)  && (r_height <= 15'd16384);

    // Frame geometry, latched at start
    reg        g_mode;
    reg [14:0] g_h;
    reg [12:0] g_wpr;               // words per row
    reg [14:0] g_ts [0:NT-1];       // first column of each tile
    reg [14:0] g_te [0:NT-1];       // one past the last column
    reg [14:0] g_tw [0:NT-1];       // tile width

    // -----------------------------------------------------------------------
    // Sequencer: SOF hold for all encoders, then feed
    // -----------------------------------------------------------------------
    localparam [1:0] F_IDLE = 2'd0,
                     F_SOF  = 2'd1,
                     F_FEED = 2'd2;
    reg [1:0]  feed_state;
    reg [15:0] sof_cnt;
    reg        enc_sof;

    wire [NT-1:0] tile_done;
    wire          all_done = &tile_done;
    wire          start_ok = start_strobe && dims_ok && (feed_state == F_IDLE);

    integer gi;
    always @(posedge clk) begin
        if(rst_any) begin
            feed_state <= F_IDLE;
            sof_cnt    <= 16'd0;
            enc_sof    <= 1'b0;
            g_mode     <= 1'b0;
            g_h        <= 15'd0;
            g_wpr      <= 13'd0;
            for(gi=0; gi<NT; gi=gi+1) begin
                g_ts[gi] <= 15'd0; g_te[gi] <= 15'd0; g_tw[gi] <= 15'd0;
            end
        end else begin
            enc_sof <= 1'b0;
            case(feed_state)
                F_IDLE: begin
                    if(start_ok) begin
                        g_mode <= r_mode;
                        g_h    <= r_height;
                        g_wpr  <= (r_width + 15'd3) >> 2;
                        for(gi=0; gi<NT; gi=gi+1) begin
                            g_ts[gi] <= gi * wt_c;
                            g_te[gi] <= (gi == NT-1) ? r_width : (gi + 1) * wt_c;
                            g_tw[gi] <= (gi == NT-1) ? last_c  : {2'd0, wt_c};
                        end
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
                    if(all_done) feed_state <= F_IDLE;
                end
                default: feed_state <= F_IDLE;
            endcase
        end
    end

    wire running = (feed_state != F_IDLE);
    wire feeding = (feed_state == F_FEED);

    // -----------------------------------------------------------------------
    // Input FIFO (32-bit words from PIX_IN)
    // -----------------------------------------------------------------------
    wire                           in_full, in_empty;
    wire [$clog2(IN_FIFO_DEPTH):0] in_count;
    wire [31:0]                    in_rd_data;
    reg  [31:0]                    in_wr_data;
    reg                            in_wr_en;
    wire                           in_rd_en;

    sync_fifo #(.WIDTH(32), .DEPTH(IN_FIFO_DEPTH)) u_in_fifo (
        .clk(clk), .rst(rst_any),
        .wr_data(in_wr_data), .wr_en(in_wr_en), .full(in_full),
        .rd_data(in_rd_data), .rd_en(in_rd_en), .empty(in_empty),
        .count(in_count)
    );

    // -----------------------------------------------------------------------
    // Test pattern (must match pattern() in example/common/jls4_demo.py)
    // -----------------------------------------------------------------------
    function [7:0] pat_px;
        input [14:0] x;
        input [14:0] y;
        reg   [7:0]  p;
        begin
            p = x[9:2] + y[8:1] + {5'd0, x[2:0] ^ y[2:0]};
            if(x[6] ^ y[6]) p = p ^ 8'h40;
            if(x[7:5] == 3'd1 && y[6:5] == 2'd1) p = 8'h80;
            pat_px = p;
        end
    endfunction

    // -----------------------------------------------------------------------
    // Row writer: one word per clock into the tile row buffers
    // -----------------------------------------------------------------------
    reg  [14:0] wr_row;     // row being written; rows below it are complete
    reg  [12:0] wr_word;    // word within the row

    wire [NT*15-1:0] rd_row_v;   // each reader's current row

    // A row half can be rewritten once every reader is past the row it held.
    reg wr_room;
    integer ri;
    always @(*) begin
        wr_room = 1'b1;
        for(ri=0; ri<NT; ri=ri+1)
            if(wr_row - rd_row_v[ri*15 +: 15] >= 15'd2) wr_room = 1'b0;
    end

    wire [14:0] col0 = {wr_word, 2'b00};
    wire [31:0] pat_word = {pat_px(col0 + 15'd3, wr_row), pat_px(col0 + 15'd2, wr_row),
                            pat_px(col0 + 15'd1, wr_row), pat_px(col0,          wr_row)};
    wire        src_valid = g_mode ? 1'b1 : !in_empty;
    wire [31:0] src_data  = g_mode ? pat_word : in_rd_data;
    wire        wr_fire   = running && (wr_row < g_h) && src_valid && wr_room;

    assign in_rd_en = wr_fire && !g_mode;

    always @(posedge clk) begin
        if(rst_any || !running) begin
            wr_row  <= 15'd0;
            wr_word <= 13'd0;
        end else if(wr_fire) begin
            if(wr_word == g_wpr - 13'd1) begin
                wr_word <= 13'd0;
                wr_row  <= wr_row + 15'd1;
            end else begin
                wr_word <= wr_word + 13'd1;
            end
        end
    end

    // -----------------------------------------------------------------------
    // Tiles: row buffer, reader, encoder, output FIFO, byte count and CRC
    // -----------------------------------------------------------------------
    function [31:0] crc32_byte;   // zlib CRC-32, reflected, one byte
        input [31:0] c;
        input [7:0]  d;
        integer      k;
        reg   [31:0] r;
        begin
            r = c ^ {24'd0, d};
            for(k=0; k<8; k=k+1)
                r = r[0] ? ((r >> 1) ^ 32'hEDB88320) : (r >> 1);
            crc32_byte = r;
        end
    endfunction

    wire [NT-1:0]    out_empty, out_full, out_overflow;
    wire [NT*12-1:0] out_count_v;
    wire [NT*17-1:0] out_rd_data_v;
    reg  [NT-1:0]    out_rd_en;
    wire [NT*32-1:0] bytes_v, crc_v;

    genvar t, ln;
    generate
        for(t=0; t<NT; t=t+1) begin : g_tile
            // ---- row buffer: two rows, four byte lanes --------------------
            wire [14:0]       ts = g_ts[t];
            wire [14:0]       te = g_te[t];
            wire [12:0]       wrel_w = wr_word - ts[14:2];
            wire [RB_AW:0]    waddr  = {wr_row[0], wrel_w[RB_AW-1:0]};

            reg  [14:0]       rd_row, rd_col;
            reg  [RB_AW-1:0]  rd_wrel;
            reg  [1:0]        rd_lane;
            wire [RB_AW:0]    raddr = {rd_row[0], rd_wrel};
            wire [31:0]       rb_q;

            for(ln=0; ln<4; ln=ln+1) begin : g_lane
                wire [14:0] col = col0 + ln;
                wire        we  = wr_fire && (col >= ts) && (col < te);
                // The writer never touches the half a reader is using, so
                // there are no read-during-write collisions.
                (* ramstyle = "M20K, no_rw_check" *) reg [7:0] mem [0:(1 << (RB_AW+1))-1];
                reg [7:0] q;
                always @(posedge clk) begin
                    if(we) mem[waddr] <= src_data[8*ln +: 8];
                    q <= mem[raddr];
                end
                assign rb_q[8*ln +: 8] = q;
            end

            // ---- reader ---------------------------------------------------
            wire [11:0] out_count = out_count_v[12*t +: 12];
            wire        bp = !g_mode && (out_count >= OUT_FIFO_DEPTH - 64);
            wire        go = feeding && (rd_row < wr_row) && !bp;

            always @(posedge clk) begin
                if(rst_any || !feeding) begin
                    rd_row  <= 15'd0;
                    rd_col  <= 15'd0;
                    rd_wrel <= {RB_AW{1'b0}};
                    rd_lane <= ts[1:0];
                end else if(go) begin
                    if(rd_col == g_tw[t] - 15'd1) begin
                        rd_col  <= 15'd0;
                        rd_row  <= rd_row + 15'd1;
                        rd_wrel <= {RB_AW{1'b0}};
                        rd_lane <= ts[1:0];
                    end else begin
                        rd_col  <= rd_col + 15'd1;
                        rd_lane <= rd_lane + 2'd1;
                        if(rd_lane == 2'd3) rd_wrel <= rd_wrel + 1'b1;
                    end
                end
            end
            assign rd_row_v[15*t +: 15] = rd_row;

            // RAM read in the cycle after go, encoder input one cycle later
            reg       go_d;
            reg [1:0] lane_d;
            reg       enc_e;
            reg [7:0] enc_x;
            always @(posedge clk) begin
                if(rst_any) begin
                    go_d  <= 1'b0;
                    enc_e <= 1'b0;
                end else begin
                    go_d  <= go;
                    enc_e <= go_d;
                end
                lane_d <= rd_lane;
                enc_x  <= rb_q[8*lane_d +: 8];
            end

            // ---- encoder --------------------------------------------------
            reg  [13:0] enc_w, enc_h;
            always @(posedge clk) begin
                enc_w <= g_tw[t][13:0] - 14'd1;
                enc_h <= g_h[13:0] - 14'd1;
            end

            wire        o_e, o_last;
            wire [15:0] o_data;

            jls_encoder #(.NEAR(NEAR), .MAX_WIDTH(MAX_TILE_W)) u_enc (
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

            // ---- output FIFO (AXI mode) -----------------------------------
            wire [$clog2(OUT_FIFO_DEPTH):0] cnt;
            sync_fifo #(.WIDTH(17), .DEPTH(OUT_FIFO_DEPTH)) u_out_fifo (
                .clk(clk), .rst(rst_any),
                .wr_data({o_last, o_data}), .wr_en(o_e && !g_mode), .full(out_full[t]),
                .rd_data(out_rd_data_v[17*t +: 17]), .rd_en(out_rd_en[t]), .empty(out_empty[t]),
                .count(cnt)
            );
            assign out_count_v[12*t +: 12] = cnt;
            assign out_overflow[t] = o_e && out_full[t] && !g_mode;

            // ---- byte count, CRC, done ------------------------------------
            reg [31:0] nbytes, crc;
            reg        done_r;
            always @(posedge clk) begin
                if(rst_any || start_ok) begin
                    nbytes <= 32'd0;
                    crc    <= 32'hFFFF_FFFF;
                    done_r <= 1'b0;
                end else if(o_e) begin
                    nbytes <= nbytes + 32'd2;
                    crc    <= crc32_byte(crc32_byte(crc, o_data[7:0]), o_data[15:8]);
                    if(o_last) done_r <= 1'b1;
                end
            end
            assign bytes_v[32*t +: 32] = nbytes;
            assign crc_v[32*t +: 32]   = ~crc;
            assign tile_done[t]        = done_r;
        end
    endgenerate

    // -----------------------------------------------------------------------
    // done / overflow / cycle counter
    // -----------------------------------------------------------------------
    always @(posedge clk) begin
        if(rst_any) begin
            done_sticky         <= 1'b0;
            out_overflow_sticky <= 1'b0;
            dim_error_sticky    <= 1'b0;
            cycles              <= 32'd0;
        end else begin
            if(start_strobe && !dims_ok) dim_error_sticky <= 1'b1;
            if(feeding && all_done)      done_sticky      <= 1'b1;
            if(|out_overflow)            out_overflow_sticky <= 1'b1;
            if(done_clear) begin
                done_sticky      <= 1'b0;
                dim_error_sticky <= 1'b0;
            end
            if(start_ok)     cycles <= 32'd0;
            else if(running) cycles <= cycles + 32'd1;
        end
    end

    // -----------------------------------------------------------------------
    // AXI4 write channel (as axi_jls_ctrl, with 32-bit PIX_IN words)
    // -----------------------------------------------------------------------
    localparam [1:0] W_IDLE = 2'd0, W_DATA = 2'd1, W_RESP = 2'd2;
    reg [1:0]  w_state;
    reg [31:0] w_addr;

    wire w_in_pix = (w_addr[15:12] == 4'h1);
    wire w_in_reg = (w_addr[15:12] == 4'h0);

    // s_wready is registered and a word reaches the FIFO through the
    // registered in_wr_en, so in_count trails the accepted beats by up to
    // two. Only offer wready for a PIX_IN beat when a FIFO slot is free after
    // counting the queued write and the beat accepted this cycle.
    wire        w_beat_pix  = s_wvalid && s_wready && w_in_pix;
    wire [12:0] in_claimed  = in_count + in_wr_en + w_beat_pix;
    wire        in_room     = (in_claimed < IN_FIFO_DEPTH);
    wire [31:0] w_next_addr = (s_wvalid && s_wready) ? (w_addr + 32'd4) : w_addr;
    wire        w_next_pix  = (w_next_addr[15:12] == 4'h1);
    wire        aw_pix      = (s_awaddr[15:12] == 4'h1);

    always @(posedge clk) begin
        if(rst_any) begin
            w_state      <= W_IDLE;
            s_awready    <= 1'b1;
            s_wready     <= 1'b0;
            s_bvalid     <= 1'b0;
            s_bresp      <= 2'b00;
            w_addr       <= 32'd0;
            in_wr_en     <= 1'b0;
            in_wr_data   <= 32'd0;
            soft_reset   <= 1'b0;
            start_strobe <= 1'b0;
            done_clear   <= 1'b0;
            r_width      <= 15'd0;
            r_height     <= 15'd0;
            r_mode       <= 1'b0;
        end else begin
            in_wr_en     <= 1'b0;
            soft_reset   <= 1'b0;
            start_strobe <= 1'b0;
            done_clear   <= 1'b0;

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
                        if(w_in_pix) begin
                            in_wr_data <= s_wdata;
                            in_wr_en   <= |s_wstrb;
                        end else if(w_in_reg) begin
                            case(w_addr[7:0])
                                8'h00: if(s_wstrb[0]) begin
                                    soft_reset   <= s_wdata[0];
                                    start_strobe <= s_wdata[1];
                                    done_clear   <= s_wdata[2];
                                end
                                8'h04: if(s_wstrb[0] || s_wstrb[1])
                                           r_width  <= s_wdata[14:0];
                                8'h08: if(s_wstrb[0] || s_wstrb[1])
                                           r_height <= s_wdata[14:0];
                                8'h1C: if(s_wstrb[0])
                                           r_mode   <= s_wdata[0];
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

    wire       r_in_out = (r_addr[15:12] == 4'h2);
    wire [1:0] r_tile   = r_addr[11:10];
    wire [1:0] r_rtile  = r_addr[5:4];      // per-tile register block

    reg [31:0] status_word;
    always @(*) begin
        status_word        = 32'd0;
        status_word[0]     = running;
        status_word[1]     = done_sticky;
        status_word[2]     = in_full;
        status_word[4]     = out_overflow_sticky;
        status_word[5]     = dim_error_sticky;
        status_word[19:8]  = in_count;
    end

    reg [31:0] tile_reg;
    always @(*) begin
        case(r_addr[3:2])
            2'd0: tile_reg = {17'd0, g_tw[r_rtile]};
            2'd1: tile_reg = {15'd0, tile_done[r_rtile], 4'd0, out_count_v[12*r_rtile +: 12]};
            2'd2: tile_reg = bytes_v[32*r_rtile +: 32];
            default: tile_reg = crc_v[32*r_rtile +: 32];
        endcase
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
            out_rd_en <= {NT{1'b0}};
        end else begin
            out_rd_en <= {NT{1'b0}};
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
                        if(r_in_out) begin
                            if(!out_empty[r_tile]) begin
                                s_rdata            <= {15'd0, out_rd_data_v[17*r_tile +: 17]};
                                out_rd_en[r_tile]  <= 1'b1;
                                s_rvalid           <= 1'b1;
                                s_rresp            <= 2'b00;
                                s_rlast            <= (r_beat == r_len);
                            end
                        end else begin
                            if(r_addr[7:6] == 2'b01)
                                s_rdata <= tile_reg;
                            else case(r_addr[7:0])
                                8'h00: s_rdata <= 32'd0;
                                8'h04: s_rdata <= {17'd0, r_width};
                                8'h08: s_rdata <= {17'd0, r_height};
                                8'h0C: s_rdata <= status_word;
                                8'h10: s_rdata <= NEAR;
                                8'h14: s_rdata <= MAX_TILE_W;
                                8'h18: s_rdata <= (OUT_FIFO_DEPTH << 16) | IN_FIFO_DEPTH;
                                8'h1C: s_rdata <= {31'd0, r_mode};
                                8'h20: s_rdata <= cycles;
                                8'h24: s_rdata <= CLK_KHZ;
                                8'h28: s_rdata <= NT;
                                default: s_rdata <= 32'hDEAD_BEEF;
                            endcase
                            s_rvalid <= 1'b1;
                            s_rresp  <= 2'b00;
                            s_rlast  <= (r_beat == r_len);
                        end
                    end else if(s_rready) begin
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
    // LEDs: [0]=busy  [1]=done  [2]=in_empty  [3]=any output waiting
    // -----------------------------------------------------------------------
    assign leds = {~&out_empty, in_empty, done_sticky, running};

endmodule
