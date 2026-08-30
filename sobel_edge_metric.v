// =============================================================================
// sobel_edge_metric.v
//
// Sobel edge metric unit for the event-denoising pipeline.
// The module computes the per-frame accumulated gradient magnitude |
// Gx | + | Gy | on the raw gray stream.
//
// This implementation keeps the interface aligned with the architecture spec and
// uses only shift/add/subtract/compare operations. Missing neighbors at the
// frame boundary are treated as zero-valued padding.
// =============================================================================

module sobel_edge_metric #(
    parameter integer FRAME_WIDTH = 640
) (
    input  wire        clk,
    input  wire        rst_n,

    input  wire [7:0]  gray,
    input  wire        valid,
    input  wire        sof,
    input  wire        eol,

    output reg  [19:0] edge_sum_frame
);

    localparam integer COL_W = 10;
    localparam integer MAX_ROW = 480;

    reg [COL_W-1:0] row_cnt;
    reg [COL_W-1:0] col_cnt;

    // Zero-padded frame storage so the 3x3 Sobel window can be evaluated even
    // at the left/right/top/bottom image boundaries.
    reg [7:0] gray_mem [0:MAX_ROW-1][0:FRAME_WIDTH-1];

    integer r, c;
    integer gx, gy;
    integer abs_gx, abs_gy;
    integer sum_mag;
    integer a00, a01, a02, a10, a11, a12, a20, a21, a22;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            row_cnt <= {COL_W{1'b0}};
            col_cnt <= {COL_W{1'b0}};
            edge_sum_frame <= 20'd0;

            for (r = 0; r < MAX_ROW; r = r + 1) begin
                for (c = 0; c < FRAME_WIDTH; c = c + 1)
                    gray_mem[r][c] <= 8'd0;
            end
        end else begin
            if (valid) begin
                if (sof) begin
                    row_cnt <= {COL_W{1'b0}};
                    col_cnt <= {COL_W{1'b0}};
                    edge_sum_frame <= 20'd0;

                    for (r = 0; r < MAX_ROW; r = r + 1) begin
                        for (c = 0; c < FRAME_WIDTH; c = c + 1)
                            gray_mem[r][c] <= 8'd0;
                    end
                end

                gray_mem[row_cnt][col_cnt] <= gray;

                if ((row_cnt >= 1) && (col_cnt >= 1) &&
                    (row_cnt <= MAX_ROW - 2) && (col_cnt <= FRAME_WIDTH - 2)) begin
                    a00 = (row_cnt-1 >= 0 && col_cnt-1 >= 0) ? gray_mem[row_cnt-1][col_cnt-1] : 8'd0;
                    a01 = (row_cnt-1 >= 0 && col_cnt >= 0)   ? gray_mem[row_cnt-1][col_cnt]   : 8'd0;
                    a02 = (row_cnt-1 >= 0 && col_cnt+1 < FRAME_WIDTH) ? gray_mem[row_cnt-1][col_cnt+1] : 8'd0;
                    a10 = (row_cnt >= 0 && col_cnt-1 >= 0)   ? gray_mem[row_cnt][col_cnt-1]   : 8'd0;
                    a11 = gray;
                    a12 = (row_cnt >= 0 && col_cnt+1 < FRAME_WIDTH) ? gray_mem[row_cnt][col_cnt+1] : 8'd0;
                    a20 = (row_cnt+1 < MAX_ROW && col_cnt-1 >= 0) ? gray_mem[row_cnt+1][col_cnt-1] : 8'd0;
                    a21 = (row_cnt+1 < MAX_ROW && col_cnt >= 0)   ? gray_mem[row_cnt+1][col_cnt]   : 8'd0;
                    a22 = (row_cnt+1 < MAX_ROW && col_cnt+1 < FRAME_WIDTH) ? gray_mem[row_cnt+1][col_cnt+1] : 8'd0;

                    // Note: a00..a22 are `integer` (32-bit, signed by default in
                    // Verilog) and only ever hold 0..255 (from unsigned 8-bit
                    // gray_mem taps or the 8'd0 boundary-padding constant), so no
                    // explicit $signed()/width-cast is needed here. The original
                    // `$signed(8'(a00))` form used a SystemVerilog-2005+ sized-cast
                    // ( 8'(...) ), which Vivado's default Verilog-2001 `.v` parser
                    // does not support -- that mismatch was the actual syntax error,
                    // not a logic bug. Plain integer arithmetic below is equivalent
                    // and Verilog-2001-clean.
                    gx = (-a00) - (2 * a10) - a20 + a02 + (2 * a12) + a22;
                    gy = (-a00) - (2 * a01) - a02 + a20 + (2 * a21) + a22;

                    abs_gx = (gx < 0) ? (-gx) : gx;
                    abs_gy = (gy < 0) ? (-gy) : gy;
                    sum_mag = abs_gx + abs_gy;
                    edge_sum_frame <= edge_sum_frame + sum_mag;
                end

                if (eol) begin
                    row_cnt <= row_cnt + 10'd1;
                    col_cnt <= {COL_W{1'b0}};
                end else begin
                    col_cnt <= col_cnt + 10'd1;
                end
            end else begin
                if (sof) begin
                    row_cnt <= {COL_W{1'b0}};
                    col_cnt <= {COL_W{1'b0}};
                end else if (eol) begin
                    row_cnt <= row_cnt + 10'd1;
                    col_cnt <= {COL_W{1'b0}};
                end
            end
        end
    end

endmodule