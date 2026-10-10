// SPDX-License-Identifier: GPL-3.0-only
// Copyright (c) 2026 Leonardo Capossio - bard0 design - hello@bard0.com

// Terasic DE25-Nano (Agilex 5) top for the JPEG-LS encoder demo.
// CLOCK1_50 drives the whole fabric directly (no PLL).
// EJTAG-AXI bridge from fpgacapZero provides AXI4 over the onboard
// USB-Blaster III (sld_virtual_jtag instance 4).
// KEY[0] = reset (active low); power-on reset is held for 255 cycles.
// LEDR[3:0] reflect encoder status (see axi_jls_ctrl); LEDR is active low.

`timescale 1ns/1ps

module de25_jls_top (
    input  wire       CLOCK1_50,
    input  wire [1:0] KEY,
    output wire [7:0] LEDR
);

    wire clk = CLOCK1_50;

    // -----------------------------------------------------------------------
    // Reset: power-on counter, KEY[0] synchronised to clk
    // -----------------------------------------------------------------------
    reg [7:0] por_count = 8'hFF;
    reg [2:0] key_sync  = 3'b111;
    always @(posedge clk) begin
        key_sync <= {key_sync[1:0], KEY[0]};
        if(!key_sync[2])            por_count <= 8'hFF;
        else if(por_count != 8'd0)  por_count <= por_count - 8'd1;
    end
    wire rst = |por_count;

    // -----------------------------------------------------------------------
    // EJTAG-AXI bridge (fpgacapZero, Intel sld_virtual_jtag wrapper)
    // Same parameters as fpgacapZero's own DE25-Nano example.
    // -----------------------------------------------------------------------
    wire [31:0] awaddr, wdata, araddr, rdata;
    wire [7:0]  awlen, arlen;
    wire [2:0]  awsize, arsize, awprot, arprot;
    wire [1:0]  awburst, arburst, bresp, rresp;
    wire [3:0]  wstrb;
    wire        awvalid, awready, wvalid, wready, wlast, bvalid, bready;
    wire        arvalid, arready, rvalid, rready, rlast;
    wire [255:0] dbg_tck_unused, dbg_tck_edge_unused;
    wire [255:0] dbg_axi_unused, dbg_axi_edge_unused;

    fcapz_ejtagaxi_intel #(
        .ADDR_W              (32),
        .DATA_W              (32),
        .FIFO_DEPTH          (16),
        .CMD_FIFO_DEPTH      (16),
        .RESP_FIFO_DEPTH     (16),
        .CMD_FIFO_MEMORY_TYPE("distributed"),
        .TIMEOUT             (4096),
        .DEBUG_EN            (0),
        .ASYNC_FIFO_IMPL     (0),
        .CHAIN               (4)
    ) u_ejtagaxi (
        .axi_clk(clk),
        .axi_rst(rst),
        .m_axi_awaddr (awaddr), .m_axi_awlen  (awlen),
        .m_axi_awsize (awsize), .m_axi_awburst(awburst),
        .m_axi_awvalid(awvalid), .m_axi_awready(awready),
        .m_axi_awprot (awprot),
        .m_axi_wdata  (wdata),  .m_axi_wstrb  (wstrb),
        .m_axi_wvalid (wvalid), .m_axi_wready (wready),
        .m_axi_wlast  (wlast),
        .m_axi_bresp  (bresp),  .m_axi_bvalid (bvalid),
        .m_axi_bready (bready),
        .m_axi_araddr (araddr), .m_axi_arlen  (arlen),
        .m_axi_arsize (arsize), .m_axi_arburst(arburst),
        .m_axi_arvalid(arvalid), .m_axi_arready(arready),
        .m_axi_arprot (arprot),
        .m_axi_rdata  (rdata),  .m_axi_rresp  (rresp),
        .m_axi_rvalid (rvalid), .m_axi_rlast  (rlast),
        .m_axi_rready (rready),
        .debug_tck     (dbg_tck_unused),
        .debug_tck_edge(dbg_tck_edge_unused),
        .debug_axi     (dbg_axi_unused),
        .debug_axi_edge(dbg_axi_edge_unused)
    );

    // -----------------------------------------------------------------------
    // JPEG-LS encoder control/glue (shared with the Arty A7 demo)
    // -----------------------------------------------------------------------
    wire [3:0] led;

    axi_jls_ctrl #(
        .NEAR            (0),
        .MAX_WIDTH       (1920),
        .IN_FIFO_DEPTH   (1024),
        .OUT_FIFO_DEPTH  (256),
        .SOF_HOLD_CYCLES (512)
    ) u_ctrl (
        .clk       (clk),
        .rst       (rst),

        .s_awaddr  (awaddr), .s_awlen  (awlen),
        .s_awsize  (awsize), .s_awburst(awburst),
        .s_awvalid (awvalid), .s_awready(awready),

        .s_wdata   (wdata),  .s_wstrb  (wstrb),
        .s_wlast   (wlast),  .s_wvalid (wvalid),
        .s_wready  (wready),

        .s_bresp   (bresp),  .s_bvalid (bvalid),
        .s_bready  (bready),

        .s_araddr  (araddr), .s_arlen  (arlen),
        .s_arsize  (arsize), .s_arburst(arburst),
        .s_arvalid (arvalid), .s_arready(arready),

        .s_rdata   (rdata),  .s_rresp  (rresp),
        .s_rlast   (rlast),  .s_rvalid (rvalid),
        .s_rready  (rready),

        .leds      (led)
    );

    assign LEDR = {4'b1111, ~led};

endmodule
