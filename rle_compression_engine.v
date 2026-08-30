// =============================================================================
// rle_compression_engine.v
//
// Run-length encoder for the filtered ternary stream.
// AXI4-Stream-style output: tdata = {run_len[5:0], symbol[1:0]}.
// The RTL accepts one input symbol per valid cycle and buffers completed runs
// so short downstream tready stalls do not drop or duplicate output bytes.
// =============================================================================

module rle_compression_engine (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [1:0]  ternary_in,
    input  wire        valid,
    input  wire        sof,
    input  wire        eol,
    input  wire        tready,

    output reg  [7:0]  tdata,
    output reg         tvalid,
    output reg         tlast
);

    localparam integer FIFO_DEPTH = 4096;
    localparam integer FIFO_AW    = 12;
    localparam [FIFO_AW:0] FIFO_DEPTH_COUNT = FIFO_DEPTH;

    reg [5:0] run_len;
    reg [1:0] cur_symbol;

    reg [7:0] fifo_data [0:FIFO_DEPTH-1];
    reg       fifo_last [0:FIFO_DEPTH-1];
    reg [FIFO_AW-1:0] fifo_wr_ptr;
    reg [FIFO_AW-1:0] fifo_rd_ptr;
    reg [FIFO_AW:0]   fifo_count;

    reg [1:0] push_count;
    reg [7:0] push_data0;
    reg [7:0] push_data1;
    reg       push_last0;
    reg       push_last1;
    reg [5:0] next_run_len;
    reg [1:0] next_cur_symbol;
    reg       load_output;
    reg [FIFO_AW:0] popped_count;
    reg [FIFO_AW-1:0] wr_ptr_after_push0;

    integer i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            run_len       <= 6'd0;
            cur_symbol    <= 2'b00;
            fifo_wr_ptr   <= {FIFO_AW{1'b0}};
            fifo_rd_ptr   <= {FIFO_AW{1'b0}};
            fifo_count    <= {(FIFO_AW+1){1'b0}};
            tdata         <= 8'd0;
            tvalid        <= 1'b0;
            tlast         <= 1'b0;
            for (i = 0; i < FIFO_DEPTH; i = i + 1) begin
                fifo_data[i] <= 8'd0;
                fifo_last[i] <= 1'b0;
            end
        end else begin
            push_count      = 2'd0;
            push_data0      = 8'd0;
            push_data1      = 8'd0;
            push_last0      = 1'b0;
            push_last1      = 1'b0;
            next_run_len    = run_len;
            next_cur_symbol = cur_symbol;

            // Generate completed-run FIFO pushes from the incoming symbol.
            if (valid) begin
                if (sof || run_len == 6'd0) begin
                    if (eol) begin
                        push_count = 2'd1;
                        push_data0 = {6'd1, ternary_in};
                        push_last0 = 1'b1;
                        next_run_len = 6'd0;
                        next_cur_symbol = 2'b00;
                    end else begin
                        next_run_len = 6'd1;
                        next_cur_symbol = ternary_in;
                    end
                end else if ((ternary_in == cur_symbol) && (run_len < 6'd63)) begin
                    if (eol) begin
                        push_count = 2'd1;
                        push_data0 = {run_len + 6'd1, cur_symbol};
                        push_last0 = 1'b1;
                        next_run_len = 6'd0;
                        next_cur_symbol = 2'b00;
                    end else begin
                        next_run_len = run_len + 6'd1;
                    end
                end else begin
                    // Symbol changed, or a 64th matching symbol arrived after
                    // a saturated 63-symbol run. Emit the old run first.
                    push_count = 2'd1;
                    push_data0 = {run_len, cur_symbol};
                    push_last0 = 1'b0;
                    if (eol) begin
                        push_count = 2'd2;
                        push_data1 = {6'd1, ternary_in};
                        push_last1 = 1'b1;
                        next_run_len = 6'd0;
                        next_cur_symbol = 2'b00;
                    end else begin
                        next_run_len = 6'd1;
                        next_cur_symbol = ternary_in;
                    end
                end
            end else if (eol && run_len != 6'd0) begin
                push_count = 2'd1;
                push_data0 = {run_len, cur_symbol};
                push_last0 = 1'b1;
                next_run_len = 6'd0;
                next_cur_symbol = 2'b00;
            end

            run_len    <= next_run_len;
            cur_symbol <= next_cur_symbol;

            // AXI output hold rule: while tvalid is high and tready is low,
            // tdata/tlast remain stable and no FIFO entry is consumed.
            load_output = (!tvalid) || (tvalid && tready);
            popped_count = fifo_count;

            if (load_output) begin
                if (fifo_count != 0) begin
                    tdata       <= fifo_data[fifo_rd_ptr];
                    tlast       <= fifo_last[fifo_rd_ptr];
                    tvalid      <= 1'b1;
                    fifo_rd_ptr <= fifo_rd_ptr + {{(FIFO_AW-1){1'b0}}, 1'b1};
                    popped_count = fifo_count - {{FIFO_AW{1'b0}}, 1'b1};
                end else begin
                    tvalid <= 1'b0;
                    tlast  <= 1'b0;
                end
            end

            // Queue newly completed runs. A finite skid FIFO cannot absorb
            // unbounded backpressure because this module has no input-ready
            // port; FIFO_DEPTH is sized for the required short tready stalls.
            if (push_count != 0) begin
                if ((popped_count + push_count) <= FIFO_DEPTH_COUNT) begin
                    fifo_data[fifo_wr_ptr] <= push_data0;
                    fifo_last[fifo_wr_ptr] <= push_last0;
                    wr_ptr_after_push0 = fifo_wr_ptr + {{(FIFO_AW-1){1'b0}}, 1'b1};
                    if (push_count == 2'd2) begin
                        fifo_data[wr_ptr_after_push0] <= push_data1;
                        fifo_last[wr_ptr_after_push0] <= push_last1;
                        fifo_wr_ptr <= wr_ptr_after_push0 + {{(FIFO_AW-1){1'b0}}, 1'b1};
                    end else begin
                        fifo_wr_ptr <= wr_ptr_after_push0;
                    end
                    fifo_count <= popped_count + push_count;
                end else begin
                    fifo_count <= popped_count;
                end
            end else begin
                fifo_count <= popped_count;
            end
        end
    end

endmodule
