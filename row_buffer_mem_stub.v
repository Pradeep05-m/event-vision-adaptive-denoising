// row_buffer_mem_stub.v
// Temporary bring-up stand-in for the real AXI4-Full-master-wrapped DDR3 path
// behind row_buffer_fsm's generic mem_* handshake. This is explicitly a
// throwaway/placeholder module -- swap it out for the real AXI4-Full wrapper
// once that is hand-written (per the project's Step 4 bring-up order), without
// touching row_buffer_fsm.v itself. Kept as its own standalone file so that
// swap is a drop-in module replacement, not a block-design rewire.
//
// Timing contract matched exactly against the verified row_buffer_fsm.v:
//
// READ_LATENCY convention: this parameter is the number of WAIT cycles
// observed AFTER the cycle a new read request is first asserted, before
// mem_rd_valid appears. Registering "this is a new request" combinationally
// this cycle only becomes visible starting the next edge (an unavoidable
// one-edge synchronous-design cost), so the TOTAL elapsed clock edges from
// mem_rd_req's assertion cycle to the cycle mem_rd_valid appears is
// READ_LATENCY + 1, not READ_LATENCY. Documented here after an earlier
// off-by-one in this module's own testbench, which compared the observed
// total directly against READ_LATENCY instead of READ_LATENCY+1 -- this
// DUT's data output was correct throughout; only the testbench's expected
// constant was wrong (fixed in row_buffer_mem_stub_tb.sv).
//
//   READ side:  row_buffer_fsm holds mem_rd_req high and mem_rd_addr steady
//               for a given byte until it sees mem_rd_valid for that byte,
//               then advances to the next address the following cycle. This
//               stub tolerates arbitrary (including multi-cycle) read latency
//               via the READ_LATENCY parameter -- it does NOT require
//               same-cycle response.
//   WRITE side: row_buffer_fsm fires a NEW mem_wr_req/mem_wr_addr/mem_wr_data
//               every single cycle for all 640 bytes of BURST_WRITE, with no
//               backpressure and WITHOUT ever checking mem_wr_done (confirmed
//               by inspection -- mem_wr_done is a declared input on
//               row_buffer_fsm but is not read anywhere in its logic). This
//               stub therefore MUST accept one write per cycle, every cycle,
//               with zero latency and zero backpressure -- it cannot stall
//               mem_wr_req the way it can legitimately delay mem_rd_valid.
//   mem_wr_done: since row_buffer_fsm does not consume it, this stub still
//               produces a meaningful pulse (one cycle after mem_wr_req falls,
//               marking "burst flushed") purely for other potential consumers
//               (e.g. a future status register) -- it is a correct, honest
//               implementation even though the current verified FSM ignores it.
//
// Memory sized 480 rows x 640 bytes = 307200 bytes (matches ROW_BYTES=640,
// full-frame byte range covered by the 19-bit mem_rd_addr/mem_wr_addr per
// row_buffer_fsm's spec). Plain Verilog array, no vendor BRAM macro --
// consistent with the project's portability-for-ASIC constraint, though this
// entire module is temporary and will not itself go through the ASIC flow.

module row_buffer_mem_stub #(
    parameter integer MEM_DEPTH    = 307200, // 480 * 640
    parameter integer READ_LATENCY = 2       // cycles from mem_rd_req assertion (new address) to mem_rd_valid
) (
    input  wire        clk,
    input  wire        rst_n,

    // read side
    input  wire [18:0] mem_rd_addr,
    input  wire        mem_rd_req,
    output reg  [7:0]  mem_rd_data,
    output reg         mem_rd_valid,

    // write side
    input  wire [18:0] mem_wr_addr,
    input  wire [7:0]  mem_wr_data,
    input  wire        mem_wr_req,
    output reg         mem_wr_done
);

    reg [7:0] mem [0:MEM_DEPTH-1];

    integer i;
    initial begin
        for (i = 0; i < MEM_DEPTH; i = i + 1)
            mem[i] = 8'd0;
    end

    // ------------------------------------------------------------------
    // WRITE side: single-cycle accept, every cycle, no latency, no stall.
    // ------------------------------------------------------------------
    reg mem_wr_req_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mem_wr_req_d <= 1'b0;
            mem_wr_done  <= 1'b0;
        end else begin
            if (mem_wr_req)
                mem[mem_wr_addr] <= mem_wr_data;

            mem_wr_req_d <= mem_wr_req;
            // pulse mem_wr_done one cycle after the write burst ends
            // (falling edge of mem_wr_req) -- "burst flushed" marker
            mem_wr_done <= (mem_wr_req_d && !mem_wr_req);
        end
    end

    // ------------------------------------------------------------------
    // READ side: tolerates arbitrary read latency. Detects a "new" request
    // as either the first request after idle, or a changed address while
    // mem_rd_req remains held high (row_buffer_fsm holds the same address
    // steady until its valid arrives, then moves to the next address).
    // ------------------------------------------------------------------
    reg [18:0] latched_addr;
    reg        latched_addr_valid;
    reg [7:0]  latency_cnt;
    reg        pending;

    wire is_new_request = mem_rd_req && (!latched_addr_valid || (mem_rd_addr !== latched_addr));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            latched_addr        <= 19'd0;
            latched_addr_valid  <= 1'b0;
            latency_cnt         <= 8'd0;
            pending             <= 1'b0;
            mem_rd_data         <= 8'd0;
            mem_rd_valid        <= 1'b0;
        end else begin
            mem_rd_valid <= 1'b0; // default: 1-cycle pulse only

            if (is_new_request) begin
                latched_addr       <= mem_rd_addr;
                latched_addr_valid <= 1'b1;
                latency_cnt        <= 8'd0;
                pending            <= 1'b1;
            end else if (pending) begin
                if (latency_cnt >= READ_LATENCY - 1) begin
                    mem_rd_data  <= mem[latched_addr];
                    mem_rd_valid <= 1'b1;
                    pending      <= 1'b0;
                end else begin
                    latency_cnt <= latency_cnt + 8'd1;
                end
            end

            if (!mem_rd_req) begin
                // request withdrawn (e.g. FSM returned to IDLE): clear
                // tracking so the next mem_rd_req is always treated as new
                latched_addr_valid <= 1'b0;
                pending            <= 1'b0;
            end
        end
    end

endmodule
