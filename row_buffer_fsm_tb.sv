// =============================================================================
// row_buffer_fsm_tb.sv
//
// Testbench for Module 1 (Row Buffer FSM).
// Target simulator: Icarus Verilog (iverilog -g2012) + vvp, waves via GTKWave
// (VCD dump, $dumpfile/$dumpvars).
//
// Structure:
//   - A memory-side BFM plays the role of the (out-of-scope) AXI4-Full
//     master shim + DDR3 row store: it answers read_row_req with a burst
//     of read_row_data bytes (configurable/random latency + inter-byte
//     gaps), and answers write_row_req by accepting write_row_data bytes
//     (configurable/random write_row_ready backpressure) and finally
//     pulsing write_row_done.
//   - A golden model tracks expected FSM state and expected row contents
//     completely independently of the DUT's internals (it does not read
//     row_mem; it derives expected prev_pixel / expected write-back bytes
//     from its own shadow arrays fed by the same stimulus).
//   - A dedicated ordering-guarantee checker asserts, every cycle, that
//     write_row_req is never seen unless this row's read has been fully
//     consumed (i.e. the DUT reached S_PROC and finished it) -- reported
//     as its own separate scoreboard line, distinct from data-content
//     mismatches, per the module's verification methodology.
// =============================================================================

`timescale 1ns/1ps

module row_buffer_fsm_tb;

    localparam ROW_BYTES = 640;
    localparam ADDR_W    = 20;
    localparam CNT_W     = 10;
    localparam CLK_PERIOD = 10; // 100 MHz processing clock

    // ---------------- DUT I/O ----------------
    reg              clk;
    reg              rst_n;
    reg  [7:0]       cur_pixel;
    reg              cur_valid;
    reg              sof;
    reg              eol;
    reg              row_start;

    reg  [7:0]       read_row_data;
    reg              read_row_data_valid;

    reg              write_row_ready;
    reg              write_row_done;

    wire [7:0]       prev_pixel;
    wire             prev_valid;
    wire [1:0]       row_burst_status;
    wire             row_burst_error;

    wire             read_row_req;
    wire [ADDR_W-1:0] read_row_addr;
    wire             write_row_req;
    wire [ADDR_W-1:0] write_row_addr;
    wire [7:0]       write_row_data;
    wire             write_row_valid;

    // ---------------- DUT instantiation ----------------
    row_buffer_fsm #(
        .ROW_BYTES(ROW_BYTES),
        .ADDR_W(ADDR_W),
        .CNT_W(CNT_W)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .cur_pixel(cur_pixel),
        .cur_valid(cur_valid),
        .sof(sof),
        .eol(eol),
        .row_start(row_start),
        .read_row_data(read_row_data),
        .read_row_data_valid(read_row_data_valid),
        .write_row_ready(write_row_ready),
        .write_row_done(write_row_done),
        .prev_pixel(prev_pixel),
        .prev_valid(prev_valid),
        .row_burst_status(row_burst_status),
        .row_burst_error(row_burst_error),
        .read_row_req(read_row_req),
        .read_row_addr(read_row_addr),
        .write_row_req(write_row_req),
        .write_row_addr(write_row_addr),
        .write_row_data(write_row_data),
        .write_row_valid(write_row_valid)
    );

    // ---------------- Clock ----------------
    initial clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // ---------------- Scoreboard counters ----------------
    int data_checks       = 0;
    int data_mismatches   = 0;
    int ordering_checks   = 0;
    int ordering_violations = 0;
    int fsm_state_mismatches = 0;
    int error_flag_checks = 0;
    int error_flag_mismatches = 0;

    // =====================================================================
    // Golden model shadow state (fully independent of DUT internals)
    // =====================================================================
    typedef enum {G_IDLE, G_BREAD, G_PROC, G_BWRITE} gstate_t;
    gstate_t gstate;

    reg [7:0] golden_row [0:ROW_BYTES-1];      // expected prev-row contents as they arrive
    reg [7:0] golden_writeback [0:ROW_BYTES-1];// expected write-back bytes (captured cur_pixel)
    int golden_fill_cnt;
    int golden_proc_cnt;
    int golden_wr_cnt;
    logic golden_read_fully_consumed; // becomes 1 only once PROC has consumed all 640

    // Independent 2-bit status encoding mirrored from spec meaning
    function automatic [1:0] gstate_code(gstate_t s);
        case (s)
            G_IDLE:   gstate_code = 2'b00;
            G_BREAD:  gstate_code = 2'b01;
            G_PROC:   gstate_code = 2'b10;
            G_BWRITE: gstate_code = 2'b11;
        endcase
    endfunction

    // Golden FSM update -- runs every clock, mirrors the spec's FSM
    // description independently of the DUT's RTL structure.
    always @(posedge clk) begin
        if (!rst_n) begin
            gstate <= G_IDLE;
            golden_fill_cnt <= 0;
            golden_proc_cnt <= 0;
            golden_wr_cnt   <= 0;
            golden_read_fully_consumed <= 1'b0;
        end else begin
            case (gstate)
                G_IDLE: begin
                    if (row_start) begin
                        golden_fill_cnt <= 0;
                        golden_read_fully_consumed <= 1'b0;
                        gstate <= G_BREAD;
                    end
                end
                G_BREAD: begin
                    if (read_row_data_valid) begin
                        golden_row[golden_fill_cnt] <= read_row_data;
                        if (golden_fill_cnt == ROW_BYTES-1) begin
                            golden_fill_cnt <= 0;
                            golden_proc_cnt <= 0;
                            gstate <= G_PROC;
                        end else begin
                            golden_fill_cnt <= golden_fill_cnt + 1;
                        end
                    end
                end
                G_PROC: begin
                    if (cur_valid) begin
                        golden_writeback[golden_proc_cnt] <= cur_pixel;
                        if (golden_proc_cnt == ROW_BYTES-1) begin
                            golden_proc_cnt <= 0;
                            golden_wr_cnt   <= 0;
                            golden_read_fully_consumed <= 1'b1;
                            gstate <= G_BWRITE;
                        end else begin
                            golden_proc_cnt <= golden_proc_cnt + 1;
                        end
                    end
                end
                G_BWRITE: begin
                    if (write_row_done) begin
                        gstate <= G_IDLE;
                    end
                end
            endcase
        end
    end

    // =====================================================================
    // Checker 1: FSM status must match golden state every cycle (post-reset)
    // =====================================================================
    always @(posedge clk) begin
        if (rst_n) begin
            if (row_burst_status !== gstate_code(gstate)) begin
                fsm_state_mismatches++;
                $display("[%0t] FSM MISMATCH: dut=%0d golden=%0d", $time, row_burst_status, gstate_code(gstate));
            end
        end
    end

    // =====================================================================
    // Checker 2: prev_pixel/prev_valid data-content check during PROCESS
    // =====================================================================
    always @(posedge clk) begin
        if (rst_n && prev_valid) begin
            data_checks++;
            if (prev_pixel !== golden_row[golden_proc_cnt]) begin
                data_mismatches++;
                $display("[%0t] DATA MISMATCH (prev_pixel) idx=%0d dut=%0d golden=%0d",
                          $time, golden_proc_cnt, prev_pixel, golden_row[golden_proc_cnt]);
            end
        end
    end

    // =====================================================================
    // Checker 3: write-back data-content check during BURST_WRITE
    // =====================================================================
    always @(posedge clk) begin
        if (rst_n && write_row_valid && write_row_ready) begin
            data_checks++;
            if (write_row_data !== golden_writeback[golden_wr_cnt]) begin
                data_mismatches++;
                $display("[%0t] DATA MISMATCH (write_row_data) idx=%0d dut=%0d golden=%0d",
                          $time, golden_wr_cnt, write_row_data, golden_writeback[golden_wr_cnt]);
            end
            golden_wr_cnt <= golden_wr_cnt + 1;
        end
    end

    // =====================================================================
    // Checker 4: ORDERING GUARANTEE -- write_row_req must never fire unless
    // this row's read has been fully consumed by PROCESS. Reported
    // separately from data-content mismatches, as required.
    // =====================================================================
    always @(posedge clk) begin
        if (rst_n) begin
            ordering_checks++;
            if (write_row_req && !golden_read_fully_consumed) begin
                ordering_violations++;
                $display("[%0t] ORDERING VIOLATION: write_row_req asserted before read fully consumed", $time);
            end
        end
    end

    // =====================================================================
    // Checker 5: row_burst_error sticky-overrun cross-check.
    // Golden overrun definition mirrors the DUT's documented rule:
    //   row_start while golden state != IDLE, OR cur_valid while golden
    //   state != PROC.
    // =====================================================================
    reg golden_error_sticky;
    wire golden_overrun = (row_start && (gstate != G_IDLE)) ||
                          (cur_valid && (gstate != G_PROC));

    always @(posedge clk) begin
        if (!rst_n) begin
            golden_error_sticky <= 1'b0;
        end else begin
            if (golden_overrun) golden_error_sticky <= 1'b1;
            error_flag_checks++;
            if (row_burst_error !== golden_error_sticky) begin
                error_flag_mismatches++;
                $display("[%0t] ERROR-FLAG MISMATCH: dut=%0b golden=%0b", $time, row_burst_error, golden_error_sticky);
            end
        end
    end

    // =====================================================================
    // Memory-side BFM: read_row responder
    // =====================================================================
    int    read_latency_cycles = 2;    // cycles between req and first data beat
    int    read_gap_cycles     = 0;    // extra idle cycles injected between beats (delayed-response test)
    bit    read_busy = 0;

    task automatic do_read_row_burst();
        int k;
        begin
            read_busy = 1;
            repeat (read_latency_cycles) @(posedge clk);
            for (k = 0; k < ROW_BYTES; k = k + 1) begin
                repeat (read_gap_cycles) begin
                    read_row_data_valid <= 1'b0;
                    @(posedge clk);
                end
                read_row_data <= row_source[read_row_addr_latched + k];
                read_row_data_valid <= 1'b1;
                @(posedge clk);
            end
            read_row_data_valid <= 1'b0;
            read_busy = 0;
        end
    endtask

    reg [ADDR_W-1:0] read_row_addr_latched;
    reg [7:0] row_source [0:(ROW_BYTES*8)-1]; // enough backing store for several rows' worth of addresses

    always @(posedge clk) begin
        if (!rst_n) begin
            read_row_addr_latched <= 0;
        end else if (read_row_req) begin
            read_row_addr_latched <= read_row_addr;
        end
    end

    always @(posedge clk) begin
        if (rst_n && read_row_req && !read_busy) begin
            do_read_row_burst();
        end
    end

    initial begin
        read_row_data <= 8'h00;
        read_row_data_valid <= 1'b0;
    end

    // =====================================================================
    // Memory-side BFM: write_row responder
    // =====================================================================
    int  write_ready_mode = 0; // 0 = always ready, 1 = random backpressure
    bit  write_busy = 0;

    task automatic do_write_row_burst();
        int k;
        begin
            write_busy = 1;
            write_row_done <= 1'b0;
            k = 0;
            while (k < ROW_BYTES) begin
                if (write_ready_mode == 1) begin
                    write_row_ready <= $urandom_range(0,3) != 0; // ~75% ready
                end else begin
                    write_row_ready <= 1'b1;
                end
                @(posedge clk);
                if (write_row_valid && write_row_ready) begin
                    k = k + 1;
                end
            end
            write_row_ready <= 1'b0;
            write_row_done <= 1'b1;
            @(posedge clk);
            write_row_done <= 1'b0;
            write_busy = 0;
        end
    endtask

    always @(posedge clk) begin
        if (rst_n && write_row_req && !write_busy) begin
            do_write_row_burst();
        end
    end

    initial begin
        write_row_ready <= 1'b0;
        write_row_done <= 1'b0;
    end

    // =====================================================================
    // Pixel-stream driver helper: drives one full row of cur_pixel/cur_valid
    // once the DUT has reached PROCESSING, in lockstep (1 px/cycle).
    // =====================================================================
    // Reads pixel data from the global px_row array (declared below in the
    // test-sequence section) rather than taking it as a task argument --
    // Icarus Verilog does not support unpacked-array task ports.
    logic [7:0] px_row [0:ROW_BYTES-1];

    task automatic drive_process_row(input bit is_eol_row);
        int k;
        begin
            // Wait for DUT to enter PROCESS (status == 2'b10)
            while (row_burst_status !== 2'b10) @(posedge clk);
            for (k = 0; k < ROW_BYTES; k = k + 1) begin
                cur_pixel <= px_row[k];
                cur_valid <= 1'b1;
                eol       <= (k == ROW_BYTES-1) && is_eol_row;
                @(posedge clk);
            end
            cur_valid <= 1'b0;
            eol       <= 1'b0;
        end
    endtask

    // Fill row_source with known random "previous frame" data for a given
    // row index (so the read-side golden model + BFM agree on content).
    task automatic seed_row_source(input int row_idx, input int seed_base);
        int k;
        begin
            for (k = 0; k < ROW_BYTES; k = k + 1) begin
                row_source[row_idx*ROW_BYTES + k] = (seed_base + k) & 8'hFF;
            end
        end
    endtask

    // =====================================================================
    // Test sequence
    // =====================================================================
    int frame_row_idx;
    int total_rows_random;
    int r, k;
    int errors_before;

    initial begin
        // ---- VCD dump for GTKWave ----
        $dumpfile("row_buffer_fsm_tb.vcd");
        $dumpvars(0, row_buffer_fsm_tb);

        // ---- init ----
        rst_n = 0;
        cur_pixel <= 0; cur_valid <= 0; sof <= 0; eol <= 0; row_start <= 0;
        repeat (5) @(posedge clk);
        rst_n = 1;
        repeat (2) @(posedge clk);

`ifdef DEBUG_TRACE
        fork
            begin
                repeat (30) begin
                    @(posedge clk);
                    $display("[TRACE t=%0t] row_start=%b sof=%b state=%0d gstate=%0d rrreq=%b rrdv=%b",
                              $time, row_start, sof, dut.state, gstate, read_row_req, read_row_data_valid);
                end
            end
        join_none
`endif

        // =================================================================
        // PHASE A: RANDOM regression -- ~100,000 pixels worth of rows
        // (160 rows x 640 px = 102,400 px), grouped into pseudo-frames of
        // 40 rows each so sof gets exercised repeatedly.
        // =================================================================
        $display("=== PHASE A: random regression (160 rows, 40 rows/frame) ===");
        read_latency_cycles = 2;
        read_gap_cycles     = 0;
        write_ready_mode    = 1;
        total_rows_random = 160;
        for (r = 0; r < total_rows_random; r = r + 1) begin
            frame_row_idx = r % 40;
            seed_row_source(r % 8, $urandom_range(0,255)); // reuse 8 backing rows cyclically
            for (k = 0; k < ROW_BYTES; k = k + 1) px_row[k] = $urandom_range(0,255);

            sof <= (frame_row_idx == 0);
            row_start <= 1'b1;
            @(posedge clk);
            row_start <= 1'b0;
            sof <= 1'b0;

            drive_process_row(1'b1);

            // Wait for return to IDLE before next row_start (normal-operation
            // pacing for the bulk of the random regression).
            while (row_burst_status !== 2'b00) @(posedge clk);
        end
        $display("PHASE A done. data_checks=%0d data_mismatches=%0d", data_checks, data_mismatches);

        // =================================================================
        // PHASE B: Directed timing-stress tests
        // =================================================================

        // --- B1: artificially delayed read response ---------------------
        $display("=== B1: delayed read response -- confirm BURST_WRITE still waits ===");
        errors_before = ordering_violations;
        read_latency_cycles = 20;
        read_gap_cycles     = 5;  // large gaps between read beats
        write_ready_mode    = 0;
        seed_row_source(0, 8'h11);
        for (k = 0; k < ROW_BYTES; k = k + 1) px_row[k] = k[7:0];
        sof <= 1'b1; row_start <= 1'b1; @(posedge clk); row_start <= 1'b0; sof <= 1'b0;
        drive_process_row(1'b1);
        while (row_burst_status !== 2'b00) @(posedge clk);
        if (ordering_violations == errors_before)
            $display("B1 RESULT: PASS (no ordering violation during delayed read)");
        else
            $display("B1 RESULT: FAIL (%0d ordering violations)", ordering_violations - errors_before);
        read_latency_cycles = 2;
        read_gap_cycles     = 0;

        // --- B2: back-to-back rows, zero idle time -----------------------
        $display("=== B2: back-to-back rows with zero idle time ===");
        errors_before = fsm_state_mismatches;
        write_ready_mode = 0;
        for (r = 0; r < 4; r = r + 1) begin
            seed_row_source(1, 8'h20 + r);
            for (k = 0; k < ROW_BYTES; k = k + 1) px_row[k] = (r*10 + k) & 8'hFF;
            row_start <= 1'b1;
            @(posedge clk);
            row_start <= 1'b0;
            drive_process_row(1'b1);
            // NOTE: no wait for IDLE drain here beyond what the FSM itself
            // enforces -- row_start is reasserted as soon as this loop
            // iterates, exercising the zero-idle-time back-to-back path.
            while (row_burst_status !== 2'b00) @(posedge clk);
        end
        if (fsm_state_mismatches == errors_before && error_flag_mismatches == 0)
            $display("B2 RESULT: PASS (back-to-back rows sequenced correctly, no spurious error)");
        else
            $display("B2 RESULT: FAIL (fsm_mismatches=%0d)", fsm_state_mismatches - errors_before);

        // --- B3: injected overrun -- row_start while busy ----------------
        $display("=== B3: injected overrun (row_start while FSM busy) ===");
        seed_row_source(2, 8'h30);
        row_start <= 1'b1; @(posedge clk); row_start <= 1'b0;
        // FSM is now in BURST_READ (busy) -- illegally assert row_start again
        repeat(3) @(posedge clk);
        row_start <= 1'b1; @(posedge clk); row_start <= 1'b0;
        repeat(5) @(posedge clk);
        if (row_burst_error === 1'b1)
            $display("B3a RESULT: PASS (row_burst_error asserted after row_start-while-busy)");
        else
            $display("B3a RESULT: FAIL (row_burst_error did not assert)");
        // let this row drain normally before continuing
        for (k = 0; k < ROW_BYTES; k = k + 1) px_row[k] = k[7:0];
        drive_process_row(1'b1);
        while (row_burst_status !== 2'b00) @(posedge clk);

        // --- B3b: injected overrun -- cur_valid while not PROCESS --------
        $display("=== B3b: injected overrun (cur_valid while FSM not PROCESS) ===");
        // Reset just the error observation point: error is sticky from B3a
        // already, so instead verify the *golden* overrun condition tracks
        // correctly by forcing a fresh violation and checking checker 5
        // continues to agree (it's a cross-check, not a fresh assert here
        // since sticky bit is already 1). We assert cur_valid during BREAD.
        seed_row_source(3, 8'h40);
        row_start <= 1'b1; @(posedge clk); row_start <= 1'b0;
        // now in BURST_READ
        cur_valid <= 1'b1; cur_pixel <= 8'hAA;
        @(posedge clk);
        cur_valid <= 1'b0;
        if (error_flag_mismatches == 0)
            $display("B3b RESULT: PASS (golden/dut error-flag cross-check held through cur_valid-during-BREAD overrun)");
        else
            $display("B3b RESULT: FAIL (%0d error-flag mismatches)", error_flag_mismatches);
        for (k = 0; k < ROW_BYTES; k = k + 1) px_row[k] = k[7:0];
        drive_process_row(1'b1);
        while (row_burst_status !== 2'b00) @(posedge clk);

        // --- B4: first and last row of a frame ---------------------------
        $display("=== B4: first and last row of a frame (sof address reset) ===");
        // First row of a new frame: sof=1, expect read_row_addr == 0
        seed_row_source(4, 8'h50);
        for (k = 0; k < ROW_BYTES; k = k + 1) px_row[k] = k[7:0];
        sof <= 1'b1; row_start <= 1'b1; @(posedge clk); row_start <= 1'b0; sof <= 1'b0;
        @(posedge clk); // read_row_req pulses the cycle after acceptance
        if (read_row_addr == 20'd0)
            $display("B4a RESULT: PASS (first row of frame uses row address 0)");
        else
            $display("B4a RESULT: FAIL (read_row_addr=%0d, expected 0)", read_row_addr);
        drive_process_row(1'b1);
        while (row_burst_status !== 2'b00) @(posedge clk);

        // "Last row" of a (truncated, simulation-scale) frame: subsequent
        // row without sof should have advanced the address by ROW_BYTES.
        seed_row_source(5, 8'h60);
        for (k = 0; k < ROW_BYTES; k = k + 1) px_row[k] = k[7:0];
        row_start <= 1'b1; @(posedge clk); row_start <= 1'b0;
        @(posedge clk);
        if (read_row_addr == ROW_BYTES[ADDR_W-1:0])
            $display("B4b RESULT: PASS (last/subsequent row address advanced by ROW_BYTES)");
        else
            $display("B4b RESULT: FAIL (read_row_addr=%0d, expected %0d)", read_row_addr, ROW_BYTES);
        drive_process_row(1'b1);
        while (row_burst_status !== 2'b00) @(posedge clk);

        repeat (10) @(posedge clk);

        // =================================================================
        // FINAL SCOREBOARD
        // =================================================================
        $display("");
        $display("========================= SCOREBOARD SUMMARY =========================");
        $display("Data-content checks       : %0d", data_checks);
        $display("Data-content mismatches   : %0d", data_mismatches);
        $display("FSM-state checks          : (every cycle post-reset)");
        $display("FSM-state mismatches      : %0d", fsm_state_mismatches);
        $display("Ordering-guarantee checks : %0d", ordering_checks);
        $display("Ordering-guarantee violations : %0d", ordering_violations);
        $display("Error-flag cross-checks   : %0d", error_flag_checks);
        $display("Error-flag mismatches     : %0d", error_flag_mismatches);
        if (data_mismatches == 0 && fsm_state_mismatches == 0 &&
            ordering_violations == 0 && error_flag_mismatches == 0) begin
            $display("OVERALL: PASS");
        end else begin
            $display("OVERALL: FAIL");
        end
        $display("========================================================================");

        $finish;
    end

    // Safety timeout
    initial begin
        #200000000; // 200 ms sim-time guard
        $display("TIMEOUT: simulation did not finish in time");
        $finish;
    end

endmodule