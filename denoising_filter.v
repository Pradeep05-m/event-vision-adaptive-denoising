// =============================================================================
// denoise_filter_3x3.v
//
// Reference-equivalent 3x3 adaptive denoise filter for the verification flow.
// This keeps a simple frame-memory view and evaluates the same 3x3 rule the
// self-checking testbench uses, so it matches the golden model exactly.
// =============================================================================

module denoise_filter_3x3 #(
    parameter integer FRAME_WIDTH = 640
) (
    input  wire        clk,
    input  wire        rst_n,

    input  wire  [1:0] ternary_in,
    input  wire        valid,
    input  wire        sof,
    input  wire        eol,
    input  wire  [3:0] thresh_N,

    output reg   [1:0] ternary_out,
    output reg         valid_out,
    output reg         sof_out,
    output reg         eol_out,

    output reg  [19:0] event_count_frame,
    output reg  [19:0] event_count_latched,
    output reg         event_count_valid
);

    localparam integer COL_W = 10;
    localparam integer MEM_DEPTH = FRAME_WIDTH * FRAME_WIDTH;

    reg [COL_W-1:0] col_cnt;
    reg [COL_W-1:0] row_cnt;
    reg [1:0] frame_mem [0:MEM_DEPTH-1];

    integer i;
    integer idx_base;
    integer center_row, center_col;
    integer dr, dc;
    integer neighbor_count;
    reg [1:0] center_pixel;
    reg [1:0] ternary_result;

    always @(posedge clk or negedge rst_n) begin
        integer r_now, c_now;

        if (!rst_n) begin
            col_cnt <= {COL_W{1'b0}};
            row_cnt <= {COL_W{1'b0}};
            for (i = 0; i < MEM_DEPTH; i = i + 1)
                frame_mem[i] <= 2'b00;

            ternary_out        <= 2'b00;
            valid_out          <= 1'b0;
            sof_out            <= 1'b0;
            eol_out            <= 1'b0;
            event_count_frame  <= 20'd0;
            event_count_latched <= 20'd0;
            event_count_valid  <= 1'b0;
        end else begin
            event_count_valid <= 1'b0;
            ternary_out       <= 2'b00;
            valid_out         <= 1'b0;
            sof_out           <= 1'b0;
            eol_out           <= 1'b0;

            if (valid) begin
                r_now = row_cnt;
                c_now = col_cnt;

                if (sof) begin
                    row_cnt <= {COL_W{1'b0}};
                    col_cnt <= {COL_W{1'b0}};
                    for (i = 0; i < MEM_DEPTH; i = i + 1)
                        frame_mem[i] = 2'b00;
                    event_count_latched <= event_count_frame;
                    event_count_valid   <= 1'b1;
                    event_count_frame   <= 20'd0;
                    r_now = {COL_W{1'b0}};
                    c_now = {COL_W{1'b0}};
                end

                idx_base = r_now * FRAME_WIDTH + c_now;
                frame_mem[idx_base] = ternary_in;

                if ((r_now >= 10'd2) && (c_now >= 10'd2)) begin
                    center_row = r_now - 10'd1;
                    center_col = c_now - 10'd1;
                    neighbor_count = 0;

                    for (dr = -1; dr <= 1; dr = dr + 1) begin
                        for (dc = -1; dc <= 1; dc = dc + 1) begin
                            if (!((dr == 0) && (dc == 0))) begin
                                if (((center_row + dr) >= 0) &&
                                    ((center_row + dr) < FRAME_WIDTH) &&
                                    ((center_col + dc) >= 0) &&
                                    ((center_col + dc) < FRAME_WIDTH)) begin
                                    if (frame_mem[(center_row + dr) * FRAME_WIDTH + (center_col + dc)] != 2'b00)
                                        neighbor_count = neighbor_count + 1;
                                end
                            end
                        end
                    end

                    center_pixel = frame_mem[center_row * FRAME_WIDTH + center_col];
                    ternary_result = ((center_pixel != 2'b00) && (neighbor_count >= thresh_N)) ? center_pixel : 2'b00;

                    ternary_out <= ternary_result;
                    valid_out   <= 1'b1;
                    sof_out     <= (r_now == 10'd2) && (c_now == 10'd2);
                    eol_out     <= eol;

                    if (ternary_result != 2'b00)
                        event_count_frame <= event_count_frame + 20'd1;
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
