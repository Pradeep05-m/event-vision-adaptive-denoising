// axi_lite_regfile.v
// AXI4-Lite slave register file, implementing event_denoising_architecture.md sec 6.
// Sits between the PS (AXI4-Lite master) and the plain config/status ports of the
// six verified ASIC-scope modules. Does NOT modify any of those modules -- it only
// drives/reads their existing plain wires. 32-bit data bus, word-aligned addressing
// (araddr/awaddr[7:2] select one of the registers below; byte offsets per the spec
// table, e.g. 0x08 = word index 2).
//
// Register map (offset : name : R/W):
//   0x00 CTRL              RW  bit0 pipeline_enable, bit1 manual_override_en
//   0x04 STATUS            RO  bit0 pipeline_running (=pipeline_enable, no separate HW yet)
//                               bit1 dma_error (tied 0, DMA is outside this wrapper's scope)
//   0x08 EPS               RW  [7:0]  -> frame_diff_engine.eps_wr_data (pulsed via eps_wr_en)
//   0x0C THRESH_N          RW* [3:0]  -> task_feedback_proxy_controller.manual_thresh_N
//                               writable only when CTRL.bit1 (override_en) is set;
//                               readback always reflects live HW thresh_N input
//   0x10 N_MIN             RW  [3:0]  -> task_feedback_proxy_controller.N_MIN
//   0x14 N_MAX             RW  [3:0]  -> task_feedback_proxy_controller.N_MAX
//   0x18 STEP              RW  [3:0]  -> task_feedback_proxy_controller.STEP
//   0x1C K_HI              RW  [3:0]  -> task_feedback_proxy_controller.K_HI
//   0x20 K_LO              RW  [3:0]  -> task_feedback_proxy_controller.K_LO
//   0x24 EVENT_COUNT       RO  [19:0] <- denoise_filter_3x3.event_count_frame (latched)
//   0x28 EDGE_METRIC_SUM   RO  [19:0] <- sobel_edge_metric.edge_sum_frame
//   0x2C COMPRESSED_BYTES  RO  [19:0] <- external byte counter (rle output), passed in
//   0x30 FRAME_COUNT       RO  [31:0] free-running, incremented on frame_tick
//   0x34 ROW_BURST_STATUS  RO  [1:0]  <- row_buffer_fsm.row_burst_status
//   0x38 ROW_BURST_ERROR   RO, W1C [0] <- row_buffer_fsm.row_burst_error (sticky, cleared by writing 1)
//   0x3C LOG_BUF_WPTR      RO  [31:0] passed in from log-buffer write-pointer tracker
//   0x40 LOG_BUF_BASE      RW  [31:0]
//   0x44 LOG_BUF_SIZE      RW  [31:0]
//
// AXI4-Lite protocol notes:
//  - Single outstanding transaction at a time (no pipelining) -- sufficient for a
//    control/status register file accessed at most once per video frame.
//  - wstrb is honored on writes (byte-lane granularity).
//  - Unmapped addresses: writes are silently accepted (bresp=OKAY) and dropped;
//    reads return 32'h0 with rresp=OKAY. (DECERR intentionally not used, since the
//    PS driver treats any non-EXOKAY/SLVERR as success and a small closed register
//    map does not need decode-error diagnostics.)

module axi_lite_regfile #(
    parameter integer ADDR_WIDTH = 8
) (
    input  wire                     clk,
    input  wire                     rst_n,

    // ------------------------------------------------------------------
    // AXI4-Lite slave interface
    // ------------------------------------------------------------------
    input  wire [ADDR_WIDTH-1:0]    s_axi_awaddr,
    input  wire                     s_axi_awvalid,
    output reg                      s_axi_awready,

    input  wire [31:0]              s_axi_wdata,
    input  wire [3:0]               s_axi_wstrb,
    input  wire                     s_axi_wvalid,
    output reg                      s_axi_wready,

    output reg  [1:0]               s_axi_bresp,
    output reg                      s_axi_bvalid,
    input  wire                     s_axi_bready,

    input  wire [ADDR_WIDTH-1:0]    s_axi_araddr,
    input  wire                     s_axi_arvalid,
    output reg                      s_axi_arready,

    output reg  [31:0]              s_axi_rdata,
    output reg  [1:0]               s_axi_rresp,
    output reg                      s_axi_rvalid,
    input  wire                     s_axi_rready,

    // ------------------------------------------------------------------
    // Plain-port side: frame_diff_engine
    // ------------------------------------------------------------------
    output reg                      eps_wr_en,
    output reg  [7:0]               eps_wr_data,
    input  wire [7:0]               eps_q,

    // ------------------------------------------------------------------
    // Plain-port side: task_feedback_proxy_controller
    // ------------------------------------------------------------------
    output reg  [3:0]               K_HI,
    output reg  [3:0]               K_LO,
    output reg  [3:0]               STEP,
    output reg  [3:0]               N_MIN,
    output reg  [3:0]               N_MAX,
    output reg                      override_en,
    output reg  [3:0]               manual_thresh_N,
    input  wire [3:0]               thresh_N_hw,      // live thresh_N from controller, for readback

    // ------------------------------------------------------------------
    // Plain-port side: row_buffer_fsm
    // ------------------------------------------------------------------
    input  wire [1:0]               row_burst_status,
    input  wire                     row_burst_error_hw,   // sticky flag as driven by row_buffer_fsm
    output reg                      row_burst_error_clr,  // pulsed W1C acknowledge back to source, if HW latch is external

    // ------------------------------------------------------------------
    // Plain-port side: denoise_filter_3x3 / sobel_edge_metric / RLE / logging
    // ------------------------------------------------------------------
    input  wire [19:0]              event_count_frame,
    input  wire [19:0]              edge_sum_frame,
    input  wire [19:0]              compressed_bytes,
    input  wire                     frame_tick,
    input  wire [31:0]              log_buf_wptr,

    output reg                      pipeline_enable,
    output reg  [31:0]              log_buf_base,
    output reg  [31:0]              log_buf_size
);

    // word offsets (addr[7:2])
    localparam [5:0]
        WOFF_CTRL             = 6'h00, // 0x00
        WOFF_STATUS           = 6'h01, // 0x04
        WOFF_EPS              = 6'h02, // 0x08
        WOFF_THRESH_N         = 6'h03, // 0x0C
        WOFF_N_MIN            = 6'h04, // 0x10
        WOFF_N_MAX            = 6'h05, // 0x14
        WOFF_STEP             = 6'h06, // 0x18
        WOFF_K_HI             = 6'h07, // 0x1C
        WOFF_K_LO             = 6'h08, // 0x20
        WOFF_EVENT_COUNT      = 6'h09, // 0x24
        WOFF_EDGE_METRIC_SUM  = 6'h0A, // 0x28
        WOFF_COMPRESSED_BYTES = 6'h0B, // 0x2C
        WOFF_FRAME_COUNT      = 6'h0C, // 0x30
        WOFF_ROW_BURST_STATUS = 6'h0D, // 0x34
        WOFF_ROW_BURST_ERROR  = 6'h0E, // 0x38
        WOFF_LOG_BUF_WPTR     = 6'h0F, // 0x3C
        WOFF_LOG_BUF_BASE     = 6'h10, // 0x40
        WOFF_LOG_BUF_SIZE     = 6'h11; // 0x44

    // ------------------------------------------------------------------
    // Internal register state
    // ------------------------------------------------------------------
    reg        manual_override_en_r;
    reg [31:0] frame_count_r;
    reg        row_burst_error_latched; // sticky copy inside the wrapper, W1C by PS
    reg        row_burst_error_prev;

    initial begin
        pipeline_enable       = 1'b0;
        manual_override_en_r  = 1'b0;
        eps_wr_en             = 1'b0;
        eps_wr_data           = 8'd8;   // matches frame_diff_engine EPS_DEFAULT
        K_HI                  = 4'd2;
        K_LO                  = 4'd1;
        STEP                  = 4'd1;
        N_MIN                 = 4'd1;
        N_MAX                 = 4'd8;
        override_en           = 1'b0;
        manual_thresh_N       = 4'd4;
        log_buf_base          = 32'd0;
        log_buf_size          = 32'd0;
        frame_count_r         = 32'd0;
        row_burst_error_latched = 1'b0;
        row_burst_error_prev    = 1'b0;
        row_burst_error_clr     = 1'b0;
    end

    always @(*) override_en = manual_override_en_r;

    // ------------------------------------------------------------------
    // Frame counter + sticky row_burst_error capture (independent of AXI side)
    // ------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            frame_count_r           <= 32'd0;
            row_burst_error_latched <= 1'b0;
            row_burst_error_prev    <= 1'b0;
            row_burst_error_clr     <= 1'b0;
        end else begin
            row_burst_error_clr <= 1'b0; // default: 1-cycle pulse only when a W1C write occurs

            if (frame_tick)
                frame_count_r <= frame_count_r + 32'd1;

            row_burst_error_prev <= row_burst_error_hw;
            // sticky: set on any rising edge of the HW error flag, held until
            // explicitly cleared by a W1C write to WOFF_ROW_BURST_ERROR
            if (row_burst_error_hw && !row_burst_error_prev)
                row_burst_error_latched <= 1'b1;
            else if (w1c_row_burst_error_pulse)
                row_burst_error_latched <= 1'b0;
        end
    end

    // ------------------------------------------------------------------
    // AXI4-Lite write channel FSM (AW/W accepted together, single outstanding)
    // ------------------------------------------------------------------
    localparam [1:0] WR_IDLE = 2'd0, WR_DATA = 2'd1, WR_RESP = 2'd2;
    reg [1:0] wr_state;
    reg [ADDR_WIDTH-1:0] awaddr_latched;
    reg w1c_row_burst_error_pulse;

    wire [5:0] wr_word_off = awaddr_latched[7:2];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_state                  <= WR_IDLE;
            s_axi_awready              <= 1'b0;
            s_axi_wready                <= 1'b0;
            s_axi_bvalid                <= 1'b0;
            s_axi_bresp                  <= 2'b00;
            awaddr_latched              <= {ADDR_WIDTH{1'b0}};
            eps_wr_en                    <= 1'b0;
            w1c_row_burst_error_pulse    <= 1'b0;
        end else begin
            eps_wr_en                 <= 1'b0; // default: 1-cycle pulse
            w1c_row_burst_error_pulse <= 1'b0;

            case (wr_state)
                WR_IDLE: begin
                    s_axi_bvalid <= 1'b0;
                    if (s_axi_awvalid && s_axi_wvalid) begin
                        // both address and data already present: accept together
                        s_axi_awready  <= 1'b1;
                        s_axi_wready   <= 1'b1;
                        awaddr_latched <= s_axi_awaddr;
                        wr_state       <= WR_RESP;
                        do_write(s_axi_awaddr[7:2], s_axi_wdata, s_axi_wstrb);
                    end else if (s_axi_awvalid) begin
                        s_axi_awready  <= 1'b1;
                        awaddr_latched <= s_axi_awaddr;
                        wr_state       <= WR_DATA;
                    end
                end

                WR_DATA: begin
                    s_axi_awready <= 1'b0;
                    if (s_axi_wvalid) begin
                        s_axi_wready <= 1'b1;
                        wr_state     <= WR_RESP;
                        do_write(awaddr_latched[7:2], s_axi_wdata, s_axi_wstrb);
                    end
                end

                WR_RESP: begin
                    s_axi_awready <= 1'b0;
                    s_axi_wready  <= 1'b0;
                    s_axi_bvalid  <= 1'b1;
                    s_axi_bresp   <= 2'b00; // OKAY
                    if (s_axi_bvalid && s_axi_bready) begin
                        s_axi_bvalid <= 1'b0;
                        wr_state     <= WR_IDLE;
                    end
                end

                default: wr_state <= WR_IDLE;
            endcase
        end
    end

    // do_write: applies byte-lane-qualified write data to the addressed register.
    // Implemented as a task so both same-cycle (AW+W together) and split-phase
    // (AW then W) paths share one decode.
    task automatic do_write(input [5:0] woff, input [31:0] wdata, input [3:0] wstrb);
        begin
            case (woff)
                WOFF_CTRL: begin
                    if (wstrb[0]) begin
                        pipeline_enable      <= wdata[0];
                        manual_override_en_r <= wdata[1];
                    end
                end
                WOFF_EPS: begin
                    if (wstrb[0]) begin
                        eps_wr_data <= wdata[7:0];
                        eps_wr_en   <= 1'b1;
                    end
                end
                WOFF_THRESH_N: begin
                    // RW* : only takes effect while manual override is enabled
                    if (wstrb[0] && manual_override_en_r)
                        manual_thresh_N <= wdata[3:0];
                end
                WOFF_N_MIN:  if (wstrb[0]) N_MIN <= wdata[3:0];
                WOFF_N_MAX:  if (wstrb[0]) N_MAX <= wdata[3:0];
                WOFF_STEP:   if (wstrb[0]) STEP  <= wdata[3:0];
                WOFF_K_HI:   if (wstrb[0]) K_HI  <= wdata[3:0];
                WOFF_K_LO:   if (wstrb[0]) K_LO  <= wdata[3:0];
                WOFF_ROW_BURST_ERROR: begin
                    // W1C: writing a 1 to bit0 clears the sticky latch
                    if (wstrb[0] && wdata[0])
                        w1c_row_burst_error_pulse <= 1'b1;
                end
                WOFF_LOG_BUF_BASE: begin
                    if (wstrb[0]) log_buf_base[7:0]   <= wdata[7:0];
                    if (wstrb[1]) log_buf_base[15:8]  <= wdata[15:8];
                    if (wstrb[2]) log_buf_base[23:16] <= wdata[23:16];
                    if (wstrb[3]) log_buf_base[31:24] <= wdata[31:24];
                end
                WOFF_LOG_BUF_SIZE: begin
                    if (wstrb[0]) log_buf_size[7:0]   <= wdata[7:0];
                    if (wstrb[1]) log_buf_size[15:8]  <= wdata[15:8];
                    if (wstrb[2]) log_buf_size[23:16] <= wdata[23:16];
                    if (wstrb[3]) log_buf_size[31:24] <= wdata[31:24];
                end
                default: ; // unmapped or read-only: silently dropped
            endcase
        end
    endtask

    // ------------------------------------------------------------------
    // AXI4-Lite read channel FSM
    // ------------------------------------------------------------------
    localparam [1:0] RD_IDLE = 2'd0, RD_DATA = 2'd1;
    reg [1:0] rd_state;
    reg [ADDR_WIDTH-1:0] araddr_latched;
    wire [5:0] rd_word_off = araddr_latched[7:2];

    function automatic [31:0] read_reg(input [5:0] woff);
        begin
            case (woff)
                WOFF_CTRL:             read_reg = {30'd0, manual_override_en_r, pipeline_enable};
                WOFF_STATUS:           read_reg = {30'd0, 1'b0, pipeline_enable}; // bit0=running, bit1=dma_error(tied 0)
                WOFF_EPS:              read_reg = {24'd0, eps_q};
                WOFF_THRESH_N:         read_reg = {28'd0, thresh_N_hw};
                WOFF_N_MIN:            read_reg = {28'd0, N_MIN};
                WOFF_N_MAX:            read_reg = {28'd0, N_MAX};
                WOFF_STEP:             read_reg = {28'd0, STEP};
                WOFF_K_HI:             read_reg = {28'd0, K_HI};
                WOFF_K_LO:             read_reg = {28'd0, K_LO};
                WOFF_EVENT_COUNT:      read_reg = {12'd0, event_count_frame};
                WOFF_EDGE_METRIC_SUM:  read_reg = {12'd0, edge_sum_frame};
                WOFF_COMPRESSED_BYTES: read_reg = {12'd0, compressed_bytes};
                WOFF_FRAME_COUNT:      read_reg = frame_count_r;
                WOFF_ROW_BURST_STATUS: read_reg = {30'd0, row_burst_status};
                WOFF_ROW_BURST_ERROR:  read_reg = {31'd0, row_burst_error_latched};
                WOFF_LOG_BUF_WPTR:     read_reg = log_buf_wptr;
                WOFF_LOG_BUF_BASE:     read_reg = log_buf_base;
                WOFF_LOG_BUF_SIZE:     read_reg = log_buf_size;
                default:               read_reg = 32'd0;
            endcase
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_state       <= RD_IDLE;
            s_axi_arready  <= 1'b0;
            s_axi_rvalid   <= 1'b0;
            s_axi_rresp    <= 2'b00;
            s_axi_rdata    <= 32'd0;
            araddr_latched <= {ADDR_WIDTH{1'b0}};
        end else begin
            case (rd_state)
                RD_IDLE: begin
                    s_axi_rvalid <= 1'b0;
                    if (s_axi_arvalid) begin
                        s_axi_arready  <= 1'b1;
                        araddr_latched <= s_axi_araddr;
                        rd_state       <= RD_DATA;
                    end
                end

                RD_DATA: begin
                    s_axi_arready <= 1'b0;
                    s_axi_rvalid  <= 1'b1;
                    s_axi_rresp   <= 2'b00; // OKAY
                    s_axi_rdata   <= read_reg(araddr_latched[7:2]);
                    if (s_axi_rvalid && s_axi_rready) begin
                        s_axi_rvalid <= 1'b0;
                        rd_state     <= RD_IDLE;
                    end
                end

                default: rd_state <= RD_IDLE;
            endcase
        end
    end

endmodule
