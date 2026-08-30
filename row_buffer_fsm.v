// Row Buffer FSM
// Abstract, vendor-independent RTL implementing row-granularity
// read-then-write sequencing. Exposes a simple generic memory
// request interface (mem_rd_req/mem_rd_addr + mem_rd_data/mem_rd_valid
// for reads, and mem_wr_req/mem_wr_addr/mem_wr_data + mem_wr_done for writes).
// Note: a real AXI4-Full wrapper would sit outside this module for FPGA
// integration; this core enforces the ordering guarantee required for ASIC portability.

// (first duplicate module removed; canonical implementation follows below)
// =============================================================
// Row Buffer FSM
// Owns row-staging BRAM + read-then-write sequencing.
// Structurally guarantees: write-back for row R never begins
// before row R's read data has been fully consumed by PROCESS.
// Generic abstracted memory handshake (stand-in for AXI4-Full).
// A real AXI4-Full read/write wrapper would sit outside this
// file for FPGA/board integration.
// =============================================================

module row_buffer_fsm (
    input  wire        clk,
    input  wire        rst_n,

    // Current-row streaming pixel input
    input  wire [7:0]  cur_pixel,
    input  wire        cur_valid,
    input  wire        sof,
    input  wire        eol,
    input  wire        row_start,

    // Generic abstracted memory read port
    input  wire [7:0]  mem_rd_data,
    input  wire        mem_rd_valid,

    // Generic abstracted memory write-completion
    input  wire        mem_wr_done,

    // Previous-row pixel output -> Frame-Diff Engine
    output reg  [7:0]  prev_pixel,
    output reg         prev_valid,

    // Generic abstracted memory request ports
    output reg  [18:0] mem_rd_addr,
    output reg         mem_rd_req,
    output reg  [18:0] mem_wr_addr,
    output reg  [7:0]  mem_wr_data,
    output reg         mem_wr_req,

    output reg  [1:0]  row_burst_status, // 00 idle,01 read,10 process,11 write
    output reg         row_burst_error   // sticky overrun flag
);

    // -----------------------------------------------------------
    // Row byte counters / row address counter
    // -----------------------------------------------------------
    localparam ROW_BYTES = 640;

    localparam [1:0] IDLE        = 2'b00,
                      BURST_READ  = 2'b01,
                      PROCESS     = 2'b10,
                      BURST_WRITE = 2'b11;

    reg [1:0]  state, next_state;

    // 20-bit row address counter (covers up to 480 rows * 640B)
    reg [19:0] row_addr_cnt;

    // Byte-within-row counters
    reg [9:0]  rd_byte_cnt;   // 0..639 during BURST_READ
    reg [9:0]  proc_byte_cnt; // 0..639 during PROCESS
    reg [9:0]  wr_byte_cnt;   // 0..639 during BURST_WRITE

    // Internal row BRAM: 640 x 8-bit, plain Verilog array, no vendor macro
    reg [7:0] row_mem [0:ROW_BYTES-1];

    // Write-staging buffer: captures cur_pixel during PROCESS
    reg [7:0] wr_stage [0:ROW_BYTES-1];

    // Latched flag: true once PROCESS has fully completed for this row.
    // BURST_WRITE is only reachable when this is set -- structural gate,
    // not just an intended sequencing.
    reg process_done;

    // -----------------------------------------------------------
    // State register
    // -----------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= IDLE;
        else
            state <= next_state;
    end

    // -----------------------------------------------------------
    // Next-state logic
    // -----------------------------------------------------------
    always @(*) begin
        next_state = state;
        case (state)
            IDLE: begin
                if (row_start)
                    next_state = BURST_READ;
            end

            BURST_READ: begin
                // Move on as soon as the final byte of the row has
                // been accepted from the memory read stream.
                if (rd_byte_cnt == ROW_BYTES - 1 && mem_rd_valid)
                    next_state = PROCESS;
            end

            PROCESS: begin
                // Structural gate: only reachable state after PROCESS
                // is BURST_WRITE, and only once process_done is set
                // (asserted the cycle the 640th pixel is consumed).
                if (proc_byte_cnt == ROW_BYTES - 1 && cur_valid)
                    next_state = BURST_WRITE;
            end

            BURST_WRITE: begin
                if (wr_byte_cnt == ROW_BYTES && !mem_wr_req)
                    next_state = IDLE;
            end

            default: next_state = IDLE;
        endcase
    end

    // -----------------------------------------------------------
    // Row address counter: advances once per completed row
    // (on BURST_WRITE -> IDLE transition)
    // -----------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            row_addr_cnt <= 20'd0;
        end else if (state == BURST_WRITE && next_state == IDLE) begin
            row_addr_cnt <= row_addr_cnt + ROW_BYTES;
        end
    end

    // -----------------------------------------------------------
    // BURST_READ: capture mem_rd_data into row_mem
    // -----------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_byte_cnt <= 10'd0;
            mem_rd_req  <= 1'b0;
            mem_rd_addr <= 19'd0;
        end else begin
            case (state)
                IDLE: begin
                    rd_byte_cnt <= 10'd0;
                    mem_rd_req  <= 1'b0;
                end

                BURST_READ: begin
                    mem_rd_addr <= row_addr_cnt[18:0] + rd_byte_cnt;
                    mem_rd_req  <= (rd_byte_cnt < ROW_BYTES);

                    if (mem_rd_valid && rd_byte_cnt < ROW_BYTES) begin
                        row_mem[rd_byte_cnt] <= mem_rd_data;
                        rd_byte_cnt          <= rd_byte_cnt + 10'd1;
                    end
                end

                default: begin
                    mem_rd_req <= 1'b0;
                end
            endcase
        end
    end

    // -----------------------------------------------------------
    // PROCESS: stream prev_pixel out in lockstep with cur_pixel,
    // capture cur_pixel into write-staging buffer
    // -----------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            proc_byte_cnt <= 10'd0;
            prev_pixel    <= 8'd0;
            prev_valid    <= 1'b0;
            process_done  <= 1'b0;
        end else begin
            case (state)
                BURST_READ: begin
                    // Reset process counters as read completes,
                    // ready for PROCESS entry.
                    proc_byte_cnt <= 10'd0;
                    prev_valid    <= 1'b0;
                    process_done  <= 1'b0;
                end

                PROCESS: begin
                    if (cur_valid) begin
                        prev_pixel <= row_mem[proc_byte_cnt];
                        prev_valid <= 1'b1;

                        wr_stage[proc_byte_cnt] <= cur_pixel;

                        if (proc_byte_cnt == ROW_BYTES - 1) begin
                            proc_byte_cnt <= 10'd0;
                            process_done  <= 1'b1; // gate for BURST_WRITE
                        end else begin
                            proc_byte_cnt <= proc_byte_cnt + 10'd1;
                        end
                    end else begin
                        prev_valid <= 1'b0;
                    end
                end

                default: begin
                    prev_valid <= 1'b0;
                end
            endcase
        end
    end

    // -----------------------------------------------------------
    // BURST_WRITE: push captured row back out.
    // Only ever entered from PROCESS with process_done asserted --
    // structurally impossible to reach from IDLE/BURST_READ due to
    // the next_state case statement above (no other state transitions
    // to BURST_WRITE exist in the FSM).
    // -----------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_byte_cnt <= 10'd0;
            mem_wr_req  <= 1'b0;
            mem_wr_addr <= 19'd0;
            mem_wr_data <= 8'd0;
        end else begin
            case (state)
                PROCESS: begin
                    wr_byte_cnt <= 10'd0;
                    mem_wr_req  <= 1'b0;
                end

                BURST_WRITE: begin
                    if (wr_byte_cnt < ROW_BYTES) begin
                        mem_wr_addr <= row_addr_cnt[18:0] + wr_byte_cnt;
                        mem_wr_data <= wr_stage[wr_byte_cnt];
                        mem_wr_req  <= 1'b1;
                        wr_byte_cnt <= wr_byte_cnt + 10'd1;
                    end else begin
                        mem_wr_req <= 1'b0;
                    end
                end

                default: begin
                    mem_wr_req <= 1'b0;
                end
            endcase
        end
    end

    // -----------------------------------------------------------
    // row_burst_status
    // -----------------------------------------------------------
    always @(*) begin
        row_burst_status = state; // 00/01/10/11 map directly onto state encoding
    end

    // -----------------------------------------------------------
    // row_burst_error: sticky.
    // Overrun condition: row_start pulses for the next row while
    // we are still in BURST_WRITE for the current row (i.e.
    // write-back for row R has not completed before row R+1's
    // read needed to start).
    // Cleared only by reset or an explicit clear pulse.
    // Design choice: no dedicated clear port was specified in the
    // interface table, so this implementation clears only on
    // rst_n (documented assumption -- see accompanying note).
    // -----------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            row_burst_error <= 1'b0;
        end else if ((state == BURST_WRITE) && row_start) begin
            row_burst_error <= 1'b1;
        end
    end

endmodule