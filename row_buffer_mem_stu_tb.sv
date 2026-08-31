`timescale 1ns/1ps
module row_buffer_mem_stub_tb;

    localparam integer SEED         = 32'hB00B5000;
    localparam integer MEM_DEPTH    = 307200;
    localparam integer READ_LATENCY = 3;
    localparam integer NUM_RANDOM_WRITES = 20000;
    localparam integer NUM_RANDOM_READS  = 5000;

    reg clk, rst_n;
    reg  [18:0] mem_rd_addr;
    reg         mem_rd_req;
    wire [7:0]  mem_rd_data;
    wire        mem_rd_valid;

    reg  [18:0] mem_wr_addr;
    reg  [7:0]  mem_wr_data;
    reg         mem_wr_req;
    wire        mem_wr_done;

    integer errors, checks;

    row_buffer_mem_stub #(.MEM_DEPTH(MEM_DEPTH), .READ_LATENCY(READ_LATENCY)) dut (
        .clk(clk), .rst_n(rst_n),
        .mem_rd_addr(mem_rd_addr), .mem_rd_req(mem_rd_req),
        .mem_rd_data(mem_rd_data), .mem_rd_valid(mem_rd_valid),
        .mem_wr_addr(mem_wr_addr), .mem_wr_data(mem_wr_data), .mem_wr_req(mem_wr_req),
        .mem_wr_done(mem_wr_done)
    );

    initial clk = 0;
    always #5 clk = ~clk;

    // golden shadow memory, independent of the DUT's internal array
    reg [7:0] golden_mem [0:MEM_DEPTH-1];

    integer i;

    // ------------------------------------------------------------------
    // Directed test 1: back-to-back single-cycle writes, no gaps, covering
    // address-boundary bytes (first/last of the memory range) plus a
    // 640-byte consecutive run -- mirrors row_buffer_fsm's real BURST_WRITE
    // pattern (a new req/addr/data every cycle, no gaps, for many cycles).
    // ------------------------------------------------------------------
    task automatic directed_write_burst;
        integer k;
        reg [18:0] addr;
        reg [7:0]  data;
        begin
            @(negedge clk);
            mem_wr_addr = 19'd0; mem_wr_data = 8'hAA; mem_wr_req = 1'b1;
            golden_mem[0] = 8'hAA;
            @(posedge clk);

            @(negedge clk);
            mem_wr_addr = MEM_DEPTH-1; mem_wr_data = 8'h55; mem_wr_req = 1'b1;
            golden_mem[MEM_DEPTH-1] = 8'h55;
            @(posedge clk);

            for (k = 0; k < 640; k = k + 1) begin
                @(negedge clk);
                addr = 19'd1000 + k;
                data = k[7:0] ^ 8'hF0;
                mem_wr_addr = addr; mem_wr_data = data; mem_wr_req = 1'b1;
                golden_mem[addr] = data;
                @(posedge clk);
            end

            @(negedge clk);
            mem_wr_req = 1'b0;
            @(posedge clk); #1;
            // mem_wr_done pulses exactly on the first posedge after
            // mem_wr_req deasserts (confirmed empirically); the earlier
            // version of this check raced the same-edge NBA update without
            // a settling delay -- the #1 here (not an extra clock edge)
            // is the actual fix.

            checks = checks + 1;
            if (mem_wr_done !== 1'b1) begin
                errors = errors + 1;
                $display("ERROR: mem_wr_done did not pulse the cycle after burst end");
            end
            @(posedge clk); #1;
            checks = checks + 1;
            if (mem_wr_done !== 1'b0) begin
                errors = errors + 1;
                $display("ERROR: mem_wr_done stayed asserted beyond one cycle");
            end
        end
    endtask

    // ------------------------------------------------------------------
    // Directed test 2: streaming read of the same 640-byte region, holding
    // address steady until mem_rd_valid arrives (matches row_buffer_fsm's
    // actual BURST_READ pattern), confirming READ_LATENCY is honored
    // exactly and no data is lost/misread.
    // ------------------------------------------------------------------
    task automatic directed_read_burst;
        integer k, wait_cycles;
        reg [18:0] addr;
        reg [7:0]  got;
        begin
            for (k = 0; k < 640; k = k + 1) begin
                addr = 19'd1000 + k;
                @(negedge clk);
                mem_rd_addr = addr;
                mem_rd_req  = 1'b1;
                wait_cycles = 0;

                while (!mem_rd_valid) begin
                    @(posedge clk); #1;
                    wait_cycles = wait_cycles + 1;
                end

                got = mem_rd_data;
                checks = checks + 1;
                if (got !== golden_mem[addr]) begin
                    errors = errors + 1;
                    $display("ERROR: read burst mismatch at addr=%0d exp=0x%02h got=0x%02h", addr, golden_mem[addr], got);
                end
                checks = checks + 1;
                if (wait_cycles !== READ_LATENCY+1) begin
                    errors = errors + 1;
                    $display("ERROR: read latency mismatch at addr=%0d exp=%0d got=%0d", addr, READ_LATENCY+1, wait_cycles);
                end

                @(negedge clk);
                mem_rd_req = 1'b0;
                @(posedge clk);
            end
        end
    endtask

    initial begin
        errors = 0; checks = 0;
        rst_n = 0;
        mem_rd_addr = 0; mem_rd_req = 0;
        mem_wr_addr = 0; mem_wr_data = 0; mem_wr_req = 0;

        for (i = 0; i < MEM_DEPTH; i = i + 1)
            golden_mem[i] = 8'd0;

        repeat (4) @(posedge clk);
        rst_n = 1;
        @(negedge clk);

        // ================= directed tests =================
        directed_write_burst();
        directed_read_burst();

        $display("--------------------------------------------------");
        $display("DIRECTED PHASE: checks=%0d errors=%0d", checks, errors);
        $display("--------------------------------------------------");

        // ================= randomized regression, fixed seed =================
        begin
            integer r;
            integer addr_r;
            integer data_r;
            integer lat_r;

            r = SEED;
            void'($urandom(r));

            // random writes across the full address range
            for (r = 0; r < NUM_RANDOM_WRITES; r = r + 1) begin
                addr_r = $urandom_range(0, MEM_DEPTH-1);
                data_r = $urandom_range(0, 255);
                @(negedge clk);
                mem_wr_addr = addr_r;
                mem_wr_data = data_r[7:0];
                mem_wr_req  = 1'b1;
                golden_mem[addr_r] = data_r[7:0];
                @(posedge clk);
            end
            @(negedge clk);
            mem_wr_req = 1'b0;
            @(posedge clk);

            // random reads, re-checking against golden shadow memory
            for (r = 0; r < NUM_RANDOM_READS; r = r + 1) begin
                addr_r = $urandom_range(0, MEM_DEPTH-1);
                @(negedge clk);
                mem_rd_addr = addr_r;
                mem_rd_req  = 1'b1;

                lat_r = 0;
                while (!mem_rd_valid && lat_r <= 20) begin
                    @(posedge clk); #1;
                    lat_r = lat_r + 1;
                end

                if (lat_r > 20) begin
                    errors = errors + 1;
                    $display("ERROR(rand): read never returned valid for addr=%0d", addr_r);
                end else begin
                    checks = checks + 1;
                    if (mem_rd_data !== golden_mem[addr_r]) begin
                        errors = errors + 1;
                        $display("ERROR(rand): mismatch addr=%0d exp=0x%02h got=0x%02h", addr_r, golden_mem[addr_r], mem_rd_data);
                    end
                    checks = checks + 1;
                    if (lat_r !== READ_LATENCY+1) begin
                        errors = errors + 1;
                        $display("ERROR(rand): latency mismatch addr=%0d exp=%0d got=%0d", addr_r, READ_LATENCY+1, lat_r);
                    end
                end

                @(negedge clk);
                mem_rd_req = 1'b0;
                @(posedge clk);
            end
        end

        $display("--------------------------------------------------");
        $display("ROW_BUFFER_MEM_STUB TB: seed=%0d, checks=%0d, errors=%0d", SEED, checks, errors);
        if (errors == 0)
            $display("RESULT: PASS");
        else
            $display("RESULT: FAIL");
        $display("--------------------------------------------------");
        $finish;
    end

endmodule
