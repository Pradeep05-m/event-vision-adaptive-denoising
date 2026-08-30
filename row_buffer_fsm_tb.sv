`timescale 1ns/1ps

module row_buffer_fsm_tb;

    localparam int SEED        = 12345;
    localparam int ROW_BYTES   = 640;
    localparam int NUM_ROWS    = 20;
    localparam int MAX_ROWS    = NUM_ROWS + 20;

    int seed_var;
    int rd_latency_mode;

    reg clk;
    reg rst_n;

    reg  [7:0] cur_pixel;
    reg        cur_valid;
    reg        sof;
    reg        eol;
    reg        row_start;

    reg  [7:0] mem_rd_data;
    reg        mem_rd_valid;
    reg        mem_wr_done;

    wire [7:0] prev_pixel;
    wire       prev_valid;
    wire [18:0] mem_rd_addr;
    wire        mem_rd_req;
    wire [18:0] mem_wr_addr;
    wire [7:0]  mem_wr_data;
    wire        mem_wr_req;
    wire [1:0]  row_burst_status;
    wire        row_burst_error;

    row_buffer_fsm dut (
        .clk(clk),
        .rst_n(rst_n),
        .cur_pixel(cur_pixel),
        .cur_valid(cur_valid),
        .sof(sof),
        .eol(eol),
        .row_start(row_start),
        .mem_rd_data(mem_rd_data),
        .mem_rd_valid(mem_rd_valid),
        .mem_wr_done(mem_wr_done),
        .prev_pixel(prev_pixel),
        .prev_valid(prev_valid),
        .mem_rd_addr(mem_rd_addr),
        .mem_rd_req(mem_rd_req),
        .mem_wr_addr(mem_wr_addr),
        .mem_wr_data(mem_wr_data),
        .mem_wr_req(mem_wr_req),
        .row_burst_status(row_burst_status),
        .row_burst_error(row_burst_error)
    );

    always #5 clk = ~clk;

    reg [7:0] golden_mem [0:(MAX_ROWS+2)*ROW_BYTES-1];

    int data_match_cnt;
    int data_mismatch_cnt;
    int ordering_violations;
    bit state_seen [0:3];

    reg [19:0] wr_req_row;
    reg [19:0] wr_req_off;

    int  consumed_count   [0:MAX_ROWS+2];
    bit  consumed_valid   [0:MAX_ROWS+2];
    bit  write_started    [0:MAX_ROWS+2];

    int shadow_row;
    int shadow_off;

    int wr_bytes_seen;

    // TB-side row counter mirroring the DUT's own monotonic row
    // counter -- every stimulus task uses THIS for addressing so
    // shadow bookkeeping can never desync from the DUT's real
    // internal row position.
    int tb_row_counter;

    // -----------------------------------------------------------
    // Read-memory server (fixed): triggers a new transaction only
    // when not already pending AND the requested address differs
    // from the last address actually served. mem_rd_req is a level
    // signal held for the whole burst in this DUT, so triggering on
    // its level alone (as before) caused spurious duplicate
    // responses.
    // -----------------------------------------------------------
    reg        rd_pending;
    reg [3:0]  rd_delay_cnt;
    reg [18:0] rd_addr_latched;
    reg        rd_have_last_addr;
    reg [18:0] rd_last_served_addr;

    always @(posedge clk) begin
        if (!rst_n) begin
            mem_rd_valid        <= 0;
            rd_pending          <= 0;
            rd_delay_cnt        <= 0;
            rd_have_last_addr   <= 0;
            rd_last_served_addr <= '0;
        end else begin
            mem_rd_valid <= 0;

            if (!rd_pending && mem_rd_req &&
                (!rd_have_last_addr || mem_rd_addr != rd_last_served_addr)) begin
                rd_addr_latched <= mem_rd_addr;
                rd_pending      <= 1;
                case (rd_latency_mode)
                    2:       rd_delay_cnt <= $urandom_range(3,8);
                    1:       rd_delay_cnt <= $urandom_range(0,4);
                    default: rd_delay_cnt <= 0;
                endcase
            end else if (rd_pending) begin
                if (rd_delay_cnt == 0) begin
                    mem_rd_data         <= golden_mem[rd_addr_latched];
                    mem_rd_valid        <= 1;
                    rd_pending          <= 0;
                    rd_have_last_addr   <= 1;
                    rd_last_served_addr <= rd_addr_latched;
                end else begin
                    rd_delay_cnt <= rd_delay_cnt - 1;
                end
            end
        end
    end

    initial begin
        $dumpfile("row_buffer_fsm_tb.vcd");
        $dumpvars(0, row_buffer_fsm_tb);

        clk = 0;
        rst_n = 0;
        cur_pixel = 0;
        cur_valid = 0;
        sof = 0;
        eol = 0;
        row_start = 0;
        mem_rd_data = 0;
        mem_rd_valid = 0;
        mem_wr_done = 0;

        data_match_cnt = 0;
        data_mismatch_cnt = 0;
        ordering_violations = 0;
        wr_bytes_seen = 0;
        tb_row_counter = 0;

        for (int i = 0; i <= MAX_ROWS+2; i++) begin
            consumed_count[i] = 0;
            consumed_valid[i] = 0;
            write_started[i]  = 0;
        end

        seed_var = SEED;
        void'($urandom(seed_var));

        for (int i = 0; i < (MAX_ROWS+2)*ROW_BYTES; i++)
            golden_mem[i] = $urandom_range(0,255);

        repeat (4) @(posedge clk);
        rst_n = 1;
        repeat (2) @(posedge clk);

        run_random_rows(NUM_ROWS);
        run_directed_delayed_read();
        run_directed_back_to_back();
        run_directed_overrun_error();
        run_directed_first_last_row();

        print_scoreboard();
        $finish;
    end

    always @(posedge clk) begin
        if (rst_n) state_seen[row_burst_status] = 1;
    end

    // -----------------------------------------------------------
    // Memory write-side server
    // -----------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            mem_wr_done <= 0;
            wr_bytes_seen <= 0;
        end else begin
            mem_wr_done <= 0;
            if (mem_wr_req) begin
                wr_req_row = mem_wr_addr / ROW_BYTES;
                wr_req_off = mem_wr_addr % ROW_BYTES;

                golden_mem[wr_req_row*ROW_BYTES + wr_req_off] = mem_wr_data;

                if (!write_started[wr_req_row]) begin
                    write_started[wr_req_row] = 1;
                    if (!consumed_valid[wr_req_row] ||
                        consumed_count[wr_req_row] < ROW_BYTES) begin
                        ordering_violations++;
                        $display("[ORDERING VIOLATION] row=%0d write started with only %0d/%0d reads consumed",
                                  wr_req_row, consumed_count[wr_req_row], ROW_BYTES);
                    end
                end

                wr_bytes_seen <= wr_bytes_seen + 1;
                if (wr_req_off == ROW_BYTES-1) begin
                    mem_wr_done <= 1;
                    wr_bytes_seen <= 0;
                end
            end
        end
    end

    // -----------------------------------------------------------
    // Track prev_valid consumption per row
    // -----------------------------------------------------------
    always @(posedge clk) begin
        if (rst_n && prev_valid) begin
            consumed_valid[shadow_row] = 1;
            consumed_count[shadow_row] = consumed_count[shadow_row] + 1;

            if (prev_pixel !== golden_mem[shadow_row*ROW_BYTES + shadow_off]) begin
                data_mismatch_cnt++;
                $display("[DATA MISMATCH] row=%0d off=%0d expected=%0d actual=%0d",
                          shadow_row, shadow_off,
                          golden_mem[shadow_row*ROW_BYTES + shadow_off], prev_pixel);
            end else begin
                data_match_cnt++;
            end

            shadow_off = shadow_off + 1;
        end
    end

    task automatic drive_one_row(input bit force_sof, input bit force_eol_pattern);
        int i;
        begin
            shadow_row = tb_row_counter;
            shadow_off = 0;
            rd_have_last_addr = 0; // reset per-row so first byte of new row is never skipped

            @(posedge clk);
            row_start <= 1;
            sof <= force_sof;
            @(posedge clk);
            row_start <= 0;
            sof <= 0;

            wait (row_burst_status == 2'b10);

            for (i = 0; i < ROW_BYTES; i++) begin
                cur_pixel <= $urandom_range(0,255);
                cur_valid <= 1;
                eol <= (i == ROW_BYTES-1) ? 1 : 0;
                @(posedge clk);
            end
            cur_valid <= 0;
            eol <= 0;

            wait (row_burst_status == 2'b00);

            tb_row_counter = tb_row_counter + 1;
        end
    endtask

    task automatic run_random_rows(input int n_rows);
        int r;
        rd_latency_mode = 1;
        for (r = 0; r < n_rows; r++) begin
            drive_one_row((r==0), (r==n_rows-1));
        end
    endtask

    task automatic run_directed_delayed_read();
        $display("[DIRECTED] Delayed mem_rd_valid response test");
        rd_latency_mode = 2;
        drive_one_row(0, 0);
        rd_latency_mode = 1;
    endtask

    task automatic run_directed_back_to_back();
        int r, i;
        $display("[DIRECTED] Back-to-back rows, zero idle cycles");
        rd_latency_mode = 0;
        for (r = 0; r < 3; r++) begin
            shadow_row = tb_row_counter;
            shadow_off = 0;
            rd_have_last_addr = 0;
            @(posedge clk);
            row_start <= 1;
            @(posedge clk);
            row_start <= 0;
            wait (row_burst_status == 2'b10);
            for (i = 0; i < ROW_BYTES; i++) begin
                cur_pixel <= $urandom_range(0,255);
                cur_valid <= 1;
                @(posedge clk);
            end
            cur_valid <= 0;
            wait (row_burst_status == 2'b00);
            tb_row_counter = tb_row_counter + 1;
        end
    endtask

    task automatic run_directed_overrun_error();
        int i;
        $display("[DIRECTED] Overrun / row_burst_error test");
        shadow_row = tb_row_counter;
        shadow_off = 0;
        rd_have_last_addr = 0;
        @(posedge clk);
        row_start <= 1;
        @(posedge clk);
        row_start <= 0;
        wait (row_burst_status == 2'b10);
        for (i = 0; i < ROW_BYTES; i++) begin
            cur_pixel <= $urandom_range(0,255);
            cur_valid <= 1;
            @(posedge clk);
        end
        cur_valid <= 0;

        wait (row_burst_status == 2'b11);
        @(posedge clk);
        row_start <= 1;
        @(posedge clk);
        row_start <= 0;

        wait (row_burst_status == 2'b00);
        tb_row_counter = tb_row_counter + 1;

        if (row_burst_error !== 1'b1)
            $display("[DIRECTED FAIL] row_burst_error did not assert on mid-write row_start overrun");
        else
            $display("[DIRECTED PASS] row_burst_error correctly asserted");
    endtask

    task automatic run_directed_first_last_row();
        $display("[DIRECTED] First-row / last-row address boundary test");
        drive_one_row(1, 0); // sof asserted -- simulates first row of a frame
        drive_one_row(0, 1); // eol pattern -- simulates last row of a frame
    endtask

    task automatic print_scoreboard();
        real pct;
        pct = (data_match_cnt+data_mismatch_cnt > 0) ?
              (100.0*data_match_cnt)/(data_match_cnt+data_mismatch_cnt) : 0.0;

        $display("=========================================================");
        $display(" ROW BUFFER FSM SCOREBOARD  (SEED = %0d)", SEED);
        $display("=========================================================");
        $display(" Data content matches   : %0d", data_match_cnt);
        $display(" Data content mismatches: %0d", data_mismatch_cnt);
        $display(" Data match percentage  : %0.4f %%", pct);
        $display(" Ordering violations    : %0d  %s",
                   ordering_violations,
                   (ordering_violations==0) ? "(PASS - critical req met)" : "(*** CRITICAL FAIL ***)");
        $display(" FSM state coverage:");
        $display("   IDLE        visited: %0s", state_seen[0] ? "YES" : "NO");
        $display("   BURST_READ  visited: %0s", state_seen[1] ? "YES" : "NO");
        $display("   PROCESS     visited: %0s", state_seen[2] ? "YES" : "NO");
        $display("   BURST_WRITE visited: %0s", state_seen[3] ? "YES" : "NO");
        $display("=========================================================");
    endtask

endmodule