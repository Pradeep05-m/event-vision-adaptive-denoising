// =============================================================================
// denoise_filter_3x3.v
//
// 3x3 Adaptive Denoise Filter (spec section 2.6)
//   - Background-activity / k-of-N filter on the ternary event map
//   - A center pixel's event survives only if it is itself active AND at
//     least thresh_N of its 8 spatial neighbors are also active
//   - Domain: 100 MHz processing clock
//
// -----------------------------------------------------------------------
// DESIGN DECISION (flag this -- override if it doesn't match your intent):
//
//   Border handling: rows/columns at the very edge of the frame (row 0,
//   last row, col 0, last col) don't have a full 8-neighbor context
//   without either (a) zero-padding + an end-of-frame "flush" FSM that
//   injects a fake extra row/column of zeros between real frames, or
//   (b) simply not producing output for border pixels ("valid"
//   convolution, like a same-size line-buffer FIR with no edge padding).
//
//   This module implements (b): it silently drops the outer 1-pixel
//   border of every frame (no output for row 0, row H-1, col 0, col W-1).
//   This falls out naturally from the pipeline's own 1-row + 1-column
//   latency (see below) -- no extra edge-case logic is needed, which
//   also makes it straightforward to verify bit-exactly against a
//   software reference model. If your downstream event count /
//   compression stats need literal 640x480 coverage, swap this note out
//   for a zero-pad + flush-FSM version; the core popcount/window logic
//   below is unaffected either way.
// -----------------------------------------------------------------------
//
// Windowing architecture (2 line buffers, cascaded row-shift):
//   buf_top[col] : holds row (r-2)'s pixel at that column, read BEFORE
//                  being overwritten this cycle with the OLD buf_mid value
//   buf_mid[col] : holds row (r-1)'s pixel at that column, read BEFORE
//                  being overwritten this cycle with the incoming pixel
//   incoming     : row r's pixel, live off ternary_in (no buffering needed
//                  -- it's already the "current" pixel)
//
//   Each row-source then feeds a 3-tap column shift register (w_top /
//   w_mid / w_bot), giving the full 3x3 window:
//
//        w_top[0] w_top[1] w_top[2]      <- row r-2, cols (c-2,c-1,c)
//        w_mid[0] w_mid[1] w_mid[2]      <- row r-1, cols (c-2,c-1,c)   CENTER = w_mid[1]
//        w_bot[0] w_bot[1] w_bot[2]      <- row r,   cols (c-2,c-1,c)
//
//   Center = (row r-1, col c-1), i.e. delayed 1 row + 1 column behind the
//   pixel currently arriving on ternary_in -- matches spec section 3's
//   "~2 lines + 3 px" window-fill latency figure.
//
// event_count_frame taps the SAME post-filter ternary_out stream that
// feeds RLE compression (spec section 2.6's "Tap point" note), not any
// later/derived statistic.
// =============================================================================

module denoise_filter_3x3 #(
    parameter integer FRAME_WIDTH = 640
) (
    input  wire        clk,
    input  wire        rst_n,        // async active-low reset

    // ---- pixel stream in (100 MHz domain) --------------------------------
    input  wire  [1:0]  ternary_in,
    input  wire         valid,
    input  wire         sof,          // asserted with row0/col0 of a frame
    input  wire         eol,          // asserted with the last column of a row
    input  wire  [3:0]  thresh_N,     // adaptive threshold, live input (0-8)

    // ---- ternary stream out (delayed 1 row + 1 col, border dropped) ------
    output reg   [1:0]  ternary_out,
    output reg          valid_out,
    output reg          sof_out,      // pulses on this frame's first output pixel
    output reg          eol_out,      // pulses on this frame's last output col per row

    // ---- per-frame event-count status (AXI4-Lite EVENT_COUNT readback) ---
    output reg  [19:0]  event_count_frame,   // running count, current frame
    output reg  [19:0]  event_count_latched, // frozen total of last completed frame
    output reg          event_count_valid    // 1-cycle pulse: event_count_latched updated
);

    localparam integer COL_W = 10; // covers 0..1023 >= FRAME_WIDTH-1

    // -------------------------------------------------------------------
    // Position tracking (0-indexed row/col of the INCOMING pixel)
    // -------------------------------------------------------------------
    reg [COL_W-1:0] col_cnt;
    reg [COL_W-1:0] row_cnt;

    wire in_fire = valid; // no backpressure on this streaming interface

    // -------------------------------------------------------------------
    // Line buffers (row-shift cascade)
    // -------------------------------------------------------------------
    reg [1:0] buf_top [0:FRAME_WIDTH-1];
    reg [1:0] buf_mid [0:FRAME_WIDTH-1];

    wire [1:0] val_top_old = buf_top[col_cnt]; // row r-2 @ col_cnt, pre-write
    wire [1:0] val_mid_old = buf_mid[col_cnt]; // row r-1 @ col_cnt, pre-write

    // -------------------------------------------------------------------
    // 3x3 window shift registers
    // -------------------------------------------------------------------
    reg [1:0] w_top [0:2];
    reg [1:0] w_mid [0:2];
    reg [1:0] w_bot [0:2];

    // -------------------------------------------------------------------
    // Popcount of 8-neighborhood (everything except center w_mid[1])
    // -------------------------------------------------------------------
    wire [3:0] neighbor_active_count =
        (w_top[0] != 2'b00) + (w_top[1] != 2'b00) + (w_top[2] != 2'b00) +
        (w_mid[0] != 2'b00)                        + (w_mid[2] != 2'b00) +
        (w_bot[0] != 2'b00) + (w_bot[1] != 2'b00) + (w_bot[2] != 2'b00);

    wire [1:0] center_pixel   = w_mid[1];
    wire       center_active  = (center_pixel != 2'b00);
    wire       filter_pass    = center_active && (neighbor_active_count >= thresh_N);
    wire [1:0] ternary_result = filter_pass ? center_pixel : 2'b00;

    // Position of the CENTER pixel this cycle (relative to incoming row_cnt/col_cnt)
    // valid only once both are >=2, which also automatically excludes the
    // last row/col as a center (incoming maxes at FRAME_WIDTH-1 / last row
    // index, giving center max = that-1) -- see border note above.
    //
    // The explicit "&& !sof" guard matters: on the sof cycle, row_cnt/col_cnt
    // still hold their STALE pre-reset values (leftover from the end of the
    // previous frame) until this same edge commits the reset below, and a
    // sof pixel's true position is always (0,0) -- never a valid center --
    // so it must never be allowed to fire off those stale counters.
    wire out_fire = in_fire && !sof && (row_cnt >= 10'd2) && (col_cnt >= 10'd2);

    // First output pixel of a frame: earliest point both position gates open
    wire first_out_pixel = out_fire && (row_cnt == 10'd2) && (col_cnt == 10'd2);
    // Last output column of the current output row: mirrors input eol timing
    wire last_out_col     = out_fire && eol;

    // -------------------------------------------------------------------
    // Sequential logic
    // -------------------------------------------------------------------
    integer i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            col_cnt             <= {COL_W{1'b0}};
            row_cnt             <= {COL_W{1'b0}};
            w_top[0] <= 2'b00; w_top[1] <= 2'b00; w_top[2] <= 2'b00;
            w_mid[0] <= 2'b00; w_mid[1] <= 2'b00; w_mid[2] <= 2'b00;
            w_bot[0] <= 2'b00; w_bot[1] <= 2'b00; w_bot[2] <= 2'b00;

            ternary_out          <= 2'b00;
            valid_out            <= 1'b0;
            sof_out              <= 1'b0;
            eol_out              <= 1'b0;

            event_count_frame    <= 20'd0;
            event_count_latched  <= 20'd0;
            event_count_valid    <= 1'b0;
        end else begin
            // defaults (overridden below where applicable)
            event_count_valid <= 1'b0;

            if (in_fire) begin
                // ---- shift the 3x3 window -------------------------------
                w_top[0] <= w_top[1]; w_top[1] <= w_top[2]; w_top[2] <= val_top_old;
                w_mid[0] <= w_mid[1]; w_mid[1] <= w_mid[2]; w_mid[2] <= val_mid_old;
                w_bot[0] <= w_bot[1]; w_bot[1] <= w_bot[2]; w_bot[2] <= ternary_in;

                // ---- cascade the line buffers (row shift) ----------------
                buf_top[col_cnt] <= val_mid_old;
                buf_mid[col_cnt] <= ternary_in;

                // ---- registered output for THIS cycle's center ----------
                ternary_out <= out_fire ? ternary_result : 2'b00;
                valid_out   <= out_fire;
                sof_out     <= first_out_pixel;
                eol_out     <= last_out_col;

                // ---- per-frame event counter ------------------------------
                if (sof) begin
                    // new frame starting: latch off the just-finished
                    // frame's total, then start counting fresh (this sof
                    // pixel itself never sets out_fire since row/col reset
                    // to 0 below, so no double-count race)
                    event_count_latched <= event_count_frame;
                    event_count_valid   <= 1'b1;
                    event_count_frame   <= 20'd0;
                end else if (out_fire && (ternary_result != 2'b00)) begin
                    event_count_frame <= event_count_frame + 20'd1;
                end

                // ---- position counters -------------------------------------
                if (sof) begin
                    row_cnt <= {COL_W{1'b0}};
                    col_cnt <= {COL_W{1'b0}};
                end else if (eol) begin
                    row_cnt <= row_cnt + 1'b1;
                    col_cnt <= {COL_W{1'b0}};
                end else begin
                    col_cnt <= col_cnt + 1'b1;
                end
            end else begin
                // bubble cycle (valid deasserted): internal window/line-buffer
                // state must NOT change, but the output stage must still
                // reflect that no fresh result was produced this cycle --
                // otherwise a stale valid_out/ternary_out would linger from
                // whatever the last real pixel produced.
                ternary_out <= 2'b00;
                valid_out   <= 1'b0;
                sof_out     <= 1'b0;
                eol_out     <= 1'b0;
            end
        end
    end

endmodule
