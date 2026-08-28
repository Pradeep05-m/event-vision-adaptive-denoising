// =============================================================================
// denoise_filter_3x3_tb.sv
//
// Self-checking testbench for denoise_filter_3x3 (spec section 2.6).
//
//   - Drives NUM_TESTS = 100_000 (1 lakh / "1L") pixel-cycles total (mix of
//     real raster pixels and randomly-interspersed bubble/idle cycles).
//   - Every cycle draws a *fresh* random seed from the global stream
//     (seed = $urandom()) and reseeds the local RNG stream with it before
//     generating that cycle's stimulus (ternary_in, thresh_N, bubble
//     decision) -- same reseed-per-iteration discipline as
//     frame_diff_engine_tb.sv, and for the same reason: any mismatch is
//     replayable from the printed seed.
//   - No SystemVerilog class/constraint/randomize() -- Icarus Verilog's
//     constraint solver support is partial, so plain $urandom_range()
//     calls are used throughout for portability across simulators.
//   - The golden reference model is a full per-frame 2D image array
//     (image_model[row][col]) rather than a line-buffer-cascade replica of
//     the DUT's internals: it's simpler to get right and independently
//     verifies the DUT's actual windowing hardware instead of just
//     re-deriving the same shift-register logic in software.
//   - The DUT streams TB_ROWS x TB_COLS synthetic frames back-to-back
//     (smaller than a real 640x480 frame, so 100,000 cycles exercises many
//     full frames' worth of sof/eol boundary transitions instead of just
//     one long frame).
//   - Border pixels (row 0, last row, col 0, last col) never produce
//     output, by design -- see the DESIGN DECISION note in
//     denoise_filter_3x3.v. The golden model mirrors that: out_fire is
//     only asserted once row_model>=2 && col_model>=2.
//   - Scoreboard tallies MATCH vs MISMATCH on {ternary_out, valid_out,
//     sof_out, eol_out} and prints a final pass/fail summary.
//
// Run (Icarus Verilog):
//   iverilog -g2012 -o sim denoise_filter_3x3.v denoise_filter_3x3_tb.sv
//   vvp sim
//   (override test count: vvp sim +NUM_TESTS=5000)
// =============================================================================

`timescale 1ns/1ps

module denoise_filter_3x3_tb;

    // -------------------------------------------------------------------
    // Config
    // -------------------------------------------------------------------
    int unsigned NUM_TESTS = 100_000;  // 1L random-seeded pixel-cycles
    localparam int TB_ROWS       = 12; // synthetic frame height (< 480, for sim speed)
    localparam int TB_COLS       = 16; // synthetic frame width  (< 640, for sim speed)
    localparam int CLK_PERIOD_NS = 10; // 100 MHz processing clock
    localparam int BUBBLE_PCT    = 10; // ~10% of cycles are idle/invalid

    initial begin
        if (!$value$plusargs("NUM_TESTS=%d", NUM_TESTS))
            NUM_TESTS = 100_000;
    end

    // -------------------------------------------------------------------
    // DUT connections
    // -------------------------------------------------------------------
    logic       clk;
    logic       rst_n;

    logic [1:0] ternary_in;
    logic       valid;
    logic       sof;
    logic       eol;
    logic [3:0] thresh_N;

    logic [1:0] ternary_out;
    logic       valid_out;
    logic       sof_out;
    logic       eol_out;
    logic [19:0] event_count_frame;
    logic [19:0] event_count_latched;
    logic        event_count_valid;

    denoise_filter_3x3 #(
        .FRAME_WIDTH(TB_COLS)
    ) dut (
        .clk                 (clk),
        .rst_n               (rst_n),
        .ternary_in          (ternary_in),
        .valid               (valid),
        .sof                 (sof),
        .eol                 (eol),
        .thresh_N            (thresh_N),
        .ternary_out         (ternary_out),
        .valid_out           (valid_out),
        .sof_out             (sof_out),
        .eol_out             (eol_out),
        .event_count_frame   (event_count_frame),
        .event_count_latched (event_count_latched),
        .event_count_valid   (event_count_valid)
    );

    // -------------------------------------------------------------------
    // Clock
    // -------------------------------------------------------------------
    initial clk = 1'b0;
    always #(CLK_PERIOD_NS/2) clk = ~clk;

    // -------------------------------------------------------------------
    // Golden reference model: full per-frame 2D image buffer
    // -------------------------------------------------------------------
    bit [1:0] image_model [0:TB_ROWS-1][0:TB_COLS-1];

    function automatic int unsigned popcount8(
        input bit [1:0] n0, input bit [1:0] n1, input bit [1:0] n2,
        input bit [1:0] n3,                     input bit [1:0] n4,
        input bit [1:0] n5, input bit [1:0] n6, input bit [1:0] n7
    );
        popcount8 = (n0 != 2'b00) + (n1 != 2'b00) + (n2 != 2'b00) +
                    (n3 != 2'b00)                  + (n4 != 2'b00) +
                    (n5 != 2'b00) + (n6 != 2'b00) + (n7 != 2'b00);
    endfunction

    // -------------------------------------------------------------------
    // Stimulus generator (plain $urandom_range, no classes)
    // -------------------------------------------------------------------
    task automatic gen_cycle(
        input  int unsigned seed,
        output bit          o_is_bubble,
        output bit [1:0]    o_ternary_in,
        output bit [3:0]    o_thresh_N
    );
        int unsigned pick;
        begin
            void'($urandom(seed));               // reseed local RNG stream

            pick        = $urandom_range(0, 99);
            o_is_bubble = (pick < BUBBLE_PCT);

            o_ternary_in = $urandom_range(0, 3);  // full 2-bit range (incl. spare code 2'b11)
            o_thresh_N   = $urandom_range(0, 15); // full 4-bit range (incl. beyond-spec 9-15)
        end
    endtask

    // -------------------------------------------------------------------
    // Scoreboard
    // -------------------------------------------------------------------
    int unsigned total_count;
    int unsigned match_count;
    int unsigned mismatch_count;
    int unsigned mismatch_report_limit = 25;

    // -------------------------------------------------------------------
    // Raster position tracking (drives the model AND the DUT stimulus)
    // -------------------------------------------------------------------
    int frame_row, frame_col;

    // -------------------------------------------------------------------
    // Main stimulus + checking loop
    // -------------------------------------------------------------------
    int unsigned seed;
    bit          is_bubble;
    bit  [1:0]   tx_ternary;
    bit  [3:0]   tx_thresh;
    bit          sof_flag, eol_flag;

    bit  [1:0]   exp_ternary;
    bit          exp_valid, exp_sof, exp_eol;
    bit          out_fire_exp;

    int center_row, center_col;
    int unsigned popc;

    real pass_rate;

    initial begin
        rst_n       = 1'b0;
        ternary_in  = 2'b00;
        valid       = 1'b0;
        sof         = 1'b0;
        eol         = 1'b0;
        thresh_N    = 4'd0;

        total_count    = 0;
        match_count    = 0;
        mismatch_count = 0;
        frame_row      = 0;
        frame_col      = 0;

        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        @(posedge clk);

        $display("=========================================================");
        $display(" denoise_filter_3x3_tb : starting %0d cycles (%0dx%0d synthetic frames)",
                  NUM_TESTS, TB_ROWS, TB_COLS);
        $display("=========================================================");

        for (int unsigned i = 0; i < NUM_TESTS; i++) begin
            seed = $urandom();
            gen_cycle(seed, is_bubble, tx_ternary, tx_thresh);

            sof_flag = (frame_row == 0) && (frame_col == 0);
            eol_flag = (frame_col == TB_COLS - 1);

            // ---- drive stimulus off the clock edge -------------------------
            @(negedge clk);
            if (is_bubble) begin
                valid      = 1'b0;
                ternary_in = tx_ternary;   // don't-care data while invalid
                sof        = 1'b0;
                eol        = 1'b0;
                thresh_N   = tx_thresh;
            end else begin
                valid      = 1'b1;
                ternary_in = tx_ternary;
                sof        = sof_flag;
                eol        = eol_flag;
                thresh_N   = tx_thresh;
            end

            // ---- compute expected result BEFORE mutating model state --------
            if (is_bubble) begin
                exp_ternary = 2'b00;
                exp_valid   = 1'b0;
                exp_sof     = 1'b0;
                exp_eol     = 1'b0;
            end else begin
                // this pixel becomes the bottom-row / current-row sample
                image_model[frame_row][frame_col] = tx_ternary;

                out_fire_exp = (frame_row >= 2) && (frame_col >= 2);

                if (out_fire_exp) begin
                    center_row = frame_row - 1;
                    center_col = frame_col - 1;
                    popc = popcount8(
                        image_model[center_row-1][center_col-1], image_model[center_row-1][center_col], image_model[center_row-1][center_col+1],
                        image_model[center_row  ][center_col-1],                                          image_model[center_row  ][center_col+1],
                        image_model[center_row+1][center_col-1], image_model[center_row+1][center_col], image_model[center_row+1][center_col+1]
                    );
                    if ((image_model[center_row][center_col] != 2'b00) && (popc >= tx_thresh))
                        exp_ternary = image_model[center_row][center_col];
                    else
                        exp_ternary = 2'b00;

                    exp_valid = 1'b1;
                    exp_sof   = (frame_row == 2) && (frame_col == 2);
                    exp_eol   = eol_flag;
                end else begin
                    exp_ternary = 2'b00;
                    exp_valid   = 1'b0;
                    exp_sof     = 1'b0;
                    exp_eol     = 1'b0;
                end
            end

            // ---- let the clock edge happen, then sample ---------------------
            @(posedge clk);
            #1;

            total_count++;

            if ((ternary_out === exp_ternary) &&
                (valid_out   === exp_valid)   &&
                (sof_out     === exp_sof)     &&
                (eol_out     === exp_eol)) begin
                match_count++;
            end else begin
                mismatch_count++;
                if (mismatch_count <= mismatch_report_limit) begin
                    $display("[MISMATCH #%0d] t=%0t seed=%0d bubble=%0b pos=(r=%0d,c=%0d) tern_in=%02b thresh=%0d sof=%0b eol=%0b",
                              mismatch_count, $time, seed, is_bubble, frame_row, frame_col, tx_ternary, tx_thresh, sof, eol);
                    $display("               expected: ternary=%02b valid=%0b sof=%0b eol=%0b",
                              exp_ternary, exp_valid, exp_sof, exp_eol);
                    $display("               got     : ternary=%02b valid=%0b sof=%0b eol=%0b",
                              ternary_out, valid_out, sof_out, eol_out);
                end
            end

            // ---- advance raster position (only on real, non-bubble pixels) --
            if (!is_bubble) begin
                if (eol_flag) begin
                    frame_col = 0;
                    frame_row = (frame_row == TB_ROWS - 1) ? 0 : (frame_row + 1);
                end else begin
                    frame_col = frame_col + 1;
                end
            end
        end

        // ---- final scoreboard report --------------------------------------
        pass_rate = (total_count == 0) ? 0.0 : (match_count * 1.0) / total_count * 100.0;

        $display("=========================================================");
        $display(" SCOREBOARD REPORT");
        $display("---------------------------------------------------------");
        $display(" Total testcases   : %0d", total_count);
        $display(" Matched           : %0d", match_count);
        $display(" Unmatched         : %0d", mismatch_count);
        $display(" Pass rate         : %0.4f %%", pass_rate);
        if (mismatch_count == 0)
            $display(" RESULT            : PASS - all testcases matched");
        else
            $display(" RESULT            : FAIL - %0d testcase(s) unmatched", mismatch_count);
        $display("=========================================================");

        $finish;
    end

    // -------------------------------------------------------------------
    // Safety timeout
    // -------------------------------------------------------------------
    initial begin
        int unsigned timeout_tests;
        if (!$value$plusargs("NUM_TESTS=%d", timeout_tests))
            timeout_tests = 100_000;
        #(CLK_PERIOD_NS * 2 * (timeout_tests + 1000));
        $display("ERROR: testbench timeout - forcing finish");
        $finish;
    end

endmodule
