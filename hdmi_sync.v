// hdmi_sync_to_handshake.v
// Derives the valid/sof/eol/row_start handshake signals your pipeline expects
// from a standard HDMI receiver's data-enable (DE) and vertical-sync (VSYNC)
// outputs. Cycle-aligned: outputs are valid combinationally in the same cycle
// as the corresponding pixel's DE, so they can be wired straight into
// rgb2gray/row_buffer_fsm alongside dvi2rgb's RGB data output -- no extra
// pipeline delay introduced here.
//
// sof:       pulses for exactly one cycle, on the first active (DE=1) pixel
//            following a rising edge of VSYNC (i.e. the first pixel of a new frame).
// eol:       pulses for exactly one cycle, on the last active pixel of each
//            video line (column index == FRAME_WIDTH-1).
// row_start: pulses for exactly one cycle, on the first active pixel of
//            EVERY line (per event_denoising_architecture.md / row_buffer_fsm
//            spec: "pulsed once at the start of each incoming line" -- this is
//            per-row, not per-frame). A frame's first row's first pixel
//            asserts sof and row_start simultaneously.
// valid:     passthrough of DE.
//
// Assumes DE, VSYNC, and pixel data from the HDMI RX IP are already
// cycle-aligned (true for dvi2rgb / v_hdmi_rx_ss outputs) -- this module
// does not resample or synchronize DE/VSYNC themselves; they are assumed
// already in the pixel_clk domain (as dvi2rgb's outputs are).

module hdmi_sync_to_handshake #(
    parameter integer FRAME_WIDTH = 640
) (
    input  wire        pixel_clk,
    input  wire        rst_n,

    input  wire        de,      // data enable / active-video, from HDMI RX
    input  wire        vsync,   // vertical sync, from HDMI RX

    output reg         valid,
    output reg         sof,
    output reg         eol,
    output reg         row_start
);

    localparam integer COL_W = $clog2(FRAME_WIDTH);

    reg                  vsync_d;
    reg                  frame_pending; // armed on vsync rising edge, cleared on first DE pixel
    reg [COL_W-1:0]      col_cnt;       // index of the CURRENT active pixel within its line

    always @(posedge pixel_clk or negedge rst_n) begin
        if (!rst_n) begin
            vsync_d       <= 1'b0;
            frame_pending <= 1'b0;
            col_cnt       <= {COL_W{1'b0}};
            valid         <= 1'b0;
            sof           <= 1'b0;
            eol           <= 1'b0;
            row_start     <= 1'b0;
        end else begin
            vsync_d <= vsync;
            valid   <= de;

            // arm on vsync rising edge (start-of-frame pending)
            if (vsync && !vsync_d)
                frame_pending <= 1'b1;

            if (de) begin
                // sof: this is the first active pixel since the last vsync edge
                sof <= frame_pending;
                if (frame_pending)
                    frame_pending <= 1'b0;

                // row_start: this is the first active pixel of ANY line --
                // true whenever col_cnt (registered from the previous active
                // pixel, or 0 at the start of a fresh line) is still at 0.
                // This fires every row, including the frame's first row
                // (where it coincides with sof).
                row_start <= (col_cnt == {COL_W{1'b0}});

                // eol: col_cnt already holds the correct 0-based index for
                // THIS pixel (registered from the previous active pixel)
                eol <= (col_cnt == FRAME_WIDTH-1);

                // advance counter for the next active pixel; wraps at end of line
                if (col_cnt == FRAME_WIDTH-1)
                    col_cnt <= {COL_W{1'b0}};
                else
                    col_cnt <= col_cnt + 1'b1;
            end else begin
                // horizontal blanking: hold counter at 0, ready for next line
                col_cnt   <= {COL_W{1'b0}};
                sof       <= 1'b0;
                eol       <= 1'b0;
                row_start <= 1'b0;
            end
        end
    end

endmodule