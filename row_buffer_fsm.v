// =============================================================================
// row_buffer_fsm.v
//
// ASIC-scope Module 1: Row Buffer sequencing FSM.
//
// Owns the 1-row (640B) staging BRAM and the row-granularity read-then-write
// sequencing against an abstracted off-chip row store (DDR3 on FPGA, generic
// SRAM controller on ASIC). Enforces: BURST_WRITE for a row never issues
// before that row's BURST_READ data has been fully consumed by PROCESS.
//
// Per spec architecture doc, section 2.4 / section 8:
//   - The AXI4-Full master transaction logic + DDR3 itself are OUT of ASIC
//     scope (system-integration / vendor-IP glue).
//   - The sequencing FSM (IDLE -> BURST_READ -> PROCESS -> BURST_WRITE -> IDLE)
//     and its ordering guarantee ARE in ASIC scope, hand-written, technology
//     independent.
//   - This module therefore exposes a generic read_row / write_row handshake
//     at its boundary instead of raw AXI4-Full signals, per the hard
//     constraint on clean protocol-abstracted module boundaries. A real
//     AXI4-Full master shim (out of ASIC scope) sits between this module's
//     read_row_*/write_row_* ports and the actual m_axi_* bus on the FPGA
//     build; that shim is NOT part of this file.
//
// Zero DSP / multiplier usage. No vendor primitives (row_mem is a plain
// behavioral register array -- infers BRAM on Xilinx synthesis, and is
// straightforwardly re-targetable to a flip-flop array or SRAM macro on
// ASIC per spec doc section 8).
// =============================================================================

module row_buffer_fsm #(
    parameter ROW_BYTES   = 640,   // pixels per row (640x480 frame)
    parameter ADDR_W      = 20,    // row/byte address width into the frame store
    parameter CNT_W       = 10     // ceil(log2(ROW_BYTES)) with headroom; 640 needs 10 bits
) (
    input  wire              clk,
    input  wire              rst_n,          // synchronous, active-low

    // ---------------- Incoming current-frame pixel stream -------------------
    input  wire [7:0]        cur_pixel,
    input  wire              cur_valid,
    input  wire              sof,            // start-of-frame (qualifies row_start)
    input  wire              eol,            // end-of-line (informational passthrough)
    input  wire              row_start,      // pulse: a new incoming row is beginning

    // ---------------- Generic memory-side response (read_row) ---------------
    // Abstraction of the burst-read completion path of the (out-of-scope)
    // AXI4-Full master shim / off-chip row store.
    input  wire [7:0]        read_row_data,
    input  wire              read_row_data_valid,   // one pulse per returned byte

    // ---------------- Generic memory-side response (write_row) --------------
    input  wire              write_row_ready,       // memory side can accept a byte this cycle
    input  wire              write_row_done,        // burst write of the full row has committed

    // ---------------- To Frame-Diff Engine -----------------------------------
    output wire [7:0]        prev_pixel,
    output wire              prev_valid,

    // ---------------- Debug / AXI4-Lite status (out-of-scope register logic
    // consumes these; this module only produces the raw status) -------------
    output wire [1:0]        row_burst_status,      // 00 idle / 01 reading / 10 processing / 11 writing
    output reg               row_burst_error,       // sticky overrun flag, cleared only on reset

    // ---------------- Generic memory-side request (read_row) -----------------
    output reg                read_row_req,          // 1-cycle pulse: start burst read of read_row_addr
    output reg  [ADDR_W-1:0]  read_row_addr,

    // ---------------- Generic memory-side request (write_row) ----------------
    output reg                write_row_req,         // 1-cycle pulse: start burst write of write_row_addr
    output reg  [ADDR_W-1:0]  write_row_addr,
    output wire [7:0]         write_row_data,
    output wire               write_row_valid
);

    // -------------------------------------------------------------------
    // FSM state encoding -- matches row_burst_status bit meaning exactly
    // -------------------------------------------------------------------
    localparam [1:0] S_IDLE   = 2'b00,
                      S_BREAD  = 2'b01,
                      S_PROC   = 2'b10,
                      S_BWRITE = 2'b11;

    reg [1:0] state;
    assign row_burst_status = state;

    // -------------------------------------------------------------------
    // Row staging BRAM: 1 row x 8b x ROW_BYTES.
    // During BURST_READ it is filled with the previous frame's row.
    // During PROCESS, row_mem[i] is read (drives prev_pixel) and, in the
    // SAME cycle, overwritten with the current frame's cur_pixel at that
    // same index -- this is the "read-then-write in place" behavior a
    // true-dual-port BRAM (or an ASIC FF-array) supports natively, and is
    // why only 1 BRAM18 is needed (spec doc section 5), not two.
    // During BURST_WRITE, the now-overwritten row_mem is streamed out as
    // the write-back data for this row.
    // -------------------------------------------------------------------
    reg [7:0] row_mem [0:ROW_BYTES-1];

    // -------------------------------------------------------------------
    // BURST_READ fill counter
    // -------------------------------------------------------------------
    reg [CNT_W-1:0] fill_cnt;

    // -------------------------------------------------------------------
    // PROCESS consume counter (also indexes row_mem for read+overwrite)
    // -------------------------------------------------------------------
    reg [CNT_W-1:0] proc_cnt;

    // -------------------------------------------------------------------
    // BURST_WRITE stream-out counter
    // -------------------------------------------------------------------
    reg [CNT_W-1:0] wr_cnt;

    // -------------------------------------------------------------------
    // Row base address bookkeeping. sof resets the address to row 0;
    // otherwise each accepted row_start advances by one row (ROW_BYTES).
    // -------------------------------------------------------------------
    reg [ADDR_W-1:0] row_addr_reg;

    // -------------------------------------------------------------------
    // Overrun detection (combinational, sampled into sticky flop below):
    //   (a) row_start while FSM is not IDLE  -> previous row not yet
    //       fully drained (BURST_WRITE not complete) when the next row
    //       is already arriving.
    //   (b) cur_valid while FSM is not PROCESS -> incoming pixel data
    //       arrived while the row buffer was not ready to consume it
    //       (BURST_READ still filling, or BURST_WRITE still draining, or
    //       idle with no row accepted yet).
    // -------------------------------------------------------------------
    wire overrun_a = row_start  && (state != S_IDLE);
    wire overrun_b = cur_valid  && (state != S_PROC);
    wire overrun   = overrun_a || overrun_b;

    // A row_start is only actually accepted (drives the FSM) while IDLE
    // and not simultaneously overrun.
    wire accept_row_start = (state == S_IDLE) && row_start;

    integer i;

    // -------------------------------------------------------------------
    // Main sequential block
    // -------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            state           <= S_IDLE;
            fill_cnt        <= {CNT_W{1'b0}};
            proc_cnt        <= {CNT_W{1'b0}};
            wr_cnt          <= {CNT_W{1'b0}};
            row_addr_reg    <= {ADDR_W{1'b0}};
            read_row_req    <= 1'b0;
            read_row_addr   <= {ADDR_W{1'b0}};
            write_row_req   <= 1'b0;
            write_row_addr  <= {ADDR_W{1'b0}};
            row_burst_error <= 1'b0;
            for (i = 0; i < ROW_BYTES; i = i + 1) begin
                row_mem[i] <= 8'h00;
            end
        end else begin

            // Default: pulses de-assert unless re-driven below
            read_row_req  <= 1'b0;
            write_row_req <= 1'b0;

            // Sticky overrun -- never cleared except by reset. (Software
            // clears the mirrored W1C bit at the AXI4-Lite register,
            // which is out-of-scope glue that latches this signal.)
            if (overrun) begin
                row_burst_error <= 1'b1;
            end

            case (state)
                // -------------------------------------------------------
                S_IDLE: begin
                    if (accept_row_start) begin
                        // Latch this row's base address and advance the
                        // running address for the row after it.
                        if (sof) begin
                            row_addr_reg <= {ADDR_W{1'b0}};
                        end else begin
                            row_addr_reg <= row_addr_reg + ROW_BYTES[ADDR_W-1:0];
                        end
                        read_row_req  <= 1'b1;
                        read_row_addr <= sof ? {ADDR_W{1'b0}}
                                              : (row_addr_reg + ROW_BYTES[ADDR_W-1:0]);
                        fill_cnt <= {CNT_W{1'b0}};
                        state    <= S_BREAD;
                    end
                end

                // -------------------------------------------------------
                S_BREAD: begin
                    if (read_row_data_valid) begin
                        row_mem[fill_cnt] <= read_row_data;
                        if (fill_cnt == ROW_BYTES[CNT_W-1:0] - 1'b1) begin
                            fill_cnt <= {CNT_W{1'b0}};
                            proc_cnt <= {CNT_W{1'b0}};
                            state    <= S_PROC;
                        end else begin
                            fill_cnt <= fill_cnt + 1'b1;
                        end
                    end
                end

                // -------------------------------------------------------
                // PROCESS: prev_pixel (combinational, see below) already
                // presents row_mem[proc_cnt] this cycle; when cur_valid
                // arrives we consume it (advance proc_cnt) and overwrite
                // row_mem[proc_cnt] with the incoming current-row pixel.
                // -------------------------------------------------------
                S_PROC: begin
                    if (cur_valid) begin
                        row_mem[proc_cnt] <= cur_pixel;
                        if (proc_cnt == ROW_BYTES[CNT_W-1:0] - 1'b1) begin
                            proc_cnt      <= {CNT_W{1'b0}};
                            wr_cnt        <= {CNT_W{1'b0}};
                            write_row_req <= 1'b1;
                            write_row_addr<= row_addr_reg;
                            state         <= S_BWRITE;
                        end else begin
                            proc_cnt <= proc_cnt + 1'b1;
                        end
                    end
                end

                // -------------------------------------------------------
                S_BWRITE: begin
                    if (write_row_valid && write_row_ready) begin
                        wr_cnt <= wr_cnt + 1'b1;
                    end
                    if (write_row_done) begin
                        state <= S_IDLE;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    // -------------------------------------------------------------------
    // Combinational outputs
    // -------------------------------------------------------------------

    // prev_pixel/prev_valid: PROCESS-state passthrough of the staged
    // previous-row byte at the current consume index, in lockstep with
    // the incoming current-row pixel that triggered the read.
    assign prev_pixel = row_mem[proc_cnt];
    assign prev_valid = (state == S_PROC) && cur_valid;

    // write_row_valid: asserted while streaming out the captured row,
    // until all ROW_BYTES have been accepted by the memory side.
    assign write_row_valid = (state == S_BWRITE) && (wr_cnt < ROW_BYTES[CNT_W-1:0]);
    // Guard the read index: wr_cnt can reach ROW_BYTES for one cycle after
    // the last byte is accepted (write_row_valid already low by then), so
    // clamp to avoid an out-of-range read on row_mem.
    wire [CNT_W-1:0] wr_rd_idx = (wr_cnt < ROW_BYTES[CNT_W-1:0]) ? wr_cnt : (ROW_BYTES[CNT_W-1:0] - 1'b1);
    assign write_row_data  = row_mem[wr_rd_idx];

endmodule