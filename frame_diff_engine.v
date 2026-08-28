// =============================================================================
// frame_diff_engine.v
//
// Frame-Diff Engine (spec section 2.5)
//   - Pixel-wise diff between current pixel and previous-frame pixel
//   - Ternary encode against a fixed epsilon register (removes ADC-level noise)
//   - Domain: 100 MHz processing clock
//
// Interface widths / behaviour taken directly from architecture spec:
//   diff = {1'b0,cur} - {1'b0,prev}      (signed 9-bit)
//   diff >  EPS -> ternary = 2'b01       (+1, brighter)
//   diff < -EPS -> ternary = 2'b10       (-1, darker)
//   else        -> ternary = 2'b00       (no event)
//
// EPS is a runtime-writable register (in the real system this is backed by
// the AXI4-Lite slave, offset 0x08). Here it's exposed as a simple
// synchronous write port (eps_wr_en / eps_wr_data) so the module can be
// unit-tested standalone without dragging in the AXI4-Lite bus.
//
// Latency: 1 cycle (registered output), matches spec section 3 timing table.
// Complexity target: 1 subtractor (9-bit) + 2 comparators, no DSP.
// =============================================================================

module frame_diff_engine #(
    parameter [7:0] EPS_DEFAULT = 8'd8
) (
    input  wire        clk,
    input  wire        rst_n,        // async active-low reset

    // ---- config write port (stand-in for AXI4-Lite EPS register) --------
    input  wire         eps_wr_en,
    input  wire  [7:0]  eps_wr_data,

    // ---- pixel stream in (100 MHz domain) --------------------------------
    input  wire  [7:0]  cur_pixel,   // from Async FIFO
    input  wire  [7:0]  prev_pixel,  // from Row Buffer
    input  wire         valid,
    input  wire         sof,
    input  wire         eol,

    // ---- ternary stream out ----------------------------------------------
    output reg   [1:0]  ternary,
    output reg          valid_out,
    output reg          sof_out,
    output reg          eol_out,

    // ---- status (debug / AXI4-Lite readback) ------------------------------
    output wire  [7:0]  eps_q        // current EPS value, for readback
);

    // -------------------------------------------------------------------
    // EPS register
    // -------------------------------------------------------------------
    reg [7:0] eps_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            eps_reg <= EPS_DEFAULT;
        else if (eps_wr_en)
            eps_reg <= eps_wr_data;
    end

    assign eps_q = eps_reg;

    // -------------------------------------------------------------------
    // Combinational diff + ternary encode
    // -------------------------------------------------------------------
    wire signed [8:0] cur_ext  = {1'b0, cur_pixel};
    wire signed [8:0] prev_ext = {1'b0, prev_pixel};
    wire signed [8:0] diff     = cur_ext - prev_ext;      // 9-bit signed subtractor
    wire signed [8:0] eps_ext  = {1'b0, eps_reg};

    wire [1:0] ternary_comb = (diff >  eps_ext) ? 2'b01 :
                               (diff < -eps_ext) ? 2'b10 :
                                                    2'b00;

    // -------------------------------------------------------------------
    // Output register (1-cycle latency)
    // -------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ternary   <= 2'b00;
            valid_out <= 1'b0;
            sof_out   <= 1'b0;
            eol_out   <= 1'b0;
        end else begin
            ternary   <= valid ? ternary_comb : 2'b00;
            valid_out <= valid;
            sof_out   <= sof & valid;
            eol_out   <= eol & valid;
        end
    end

endmodule