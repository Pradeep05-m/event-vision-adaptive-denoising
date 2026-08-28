// =============================================================================
// frame_diff_engine_tb.sv
//
// Self-checking testbench for frame_diff_engine.
//
//   - Runs NUM_TESTS = 100_000 (1 lakh / "1L") randomized cycles.
//   - Every iteration draws a *fresh* random seed from the global stream
//     (seed = $urandom()) and reseeds the local RNG stream with it
//     (void'($urandom(seed))) before drawing that cycle's stimulus, so any
//     failing case is replayable: re-seed with the printed `seed` value and
//     you'll regenerate the exact same cur/prev/eps/valid/sof/eol.
//   - Deliberately avoids SystemVerilog class-based rand/constraint/
//     randomize() -- Icarus Verilog's constraint solver support is
//     partial/experimental and tends to produce confusing cascading parse
//     errors. Plain $urandom_range() calls are portable across Icarus,
//     Questa, VCS, Xcelium.
//   - A golden reference model mirrors the DUT's EPS-register timing and
//     computes the expected ternary/valid/sof/eol every cycle.
//   - Scoreboard tallies MATCH vs MISMATCH and prints a final pass/fail
//     summary with counts and percentage.
//
// Run (Icarus Verilog):
//   iverilog -g2012 -o sim frame_diff_engine.v frame_diff_engine_tb.sv
//   vvp sim
//   (override test count: vvp sim +NUM_TESTS=1000)
// =============================================================================

`timescale 1ns/1ps

module frame_diff_engine_tb;

    // -------------------------------------------------------------------
    // Config
    // -------------------------------------------------------------------
    int unsigned NUM_TESTS          = 100_000; // 1L random-seeded iterations
    int unsigned EPS_UPDATE_PERIOD  = 500;      // rewrite EPS every N cycles
    localparam bit [7:0] EPS_DEFAULT = 8'd8;    // must match DUT parameter
    localparam int CLK_PERIOD_NS     = 10;      // 100 MHz processing clock

    initial begin
        if (!$value$plusargs("NUM_TESTS=%d", NUM_TESTS))
            NUM_TESTS = 100_000;
    end

    // -------------------------------------------------------------------
    // DUT connections
    // -------------------------------------------------------------------
    logic        clk;
    logic        rst_n;

    logic        eps_wr_en;
    logic [7:0]  eps_wr_data;

    logic [7:0]  cur_pixel;
    logic [7:0]  prev_pixel;
    logic        valid;
    logic        sof;
    logic        eol;

    logic [1:0]  ternary;
    logic        valid_out;
    logic        sof_out;
    logic        eol_out;
    logic [7:0]  eps_q;

    frame_diff_engine #(
        .EPS_DEFAULT(EPS_DEFAULT)
    ) dut (
        .clk         (clk),
        .rst_n       (rst_n),
        .eps_wr_en   (eps_wr_en),
        .eps_wr_data (eps_wr_data),
        .cur_pixel   (cur_pixel),
        .prev_pixel  (prev_pixel),
        .valid       (valid),
        .sof         (sof),
        .eol         (eol),
        .ternary     (ternary),
        .valid_out   (valid_out),
        .sof_out     (sof_out),
        .eol_out     (eol_out),
        .eps_q       (eps_q)
    );

    // -------------------------------------------------------------------
    // Clock
    // -------------------------------------------------------------------
    initial clk = 1'b0;
    always #(CLK_PERIOD_NS/2) clk = ~clk;

    // -------------------------------------------------------------------
    // Golden reference model
    // -------------------------------------------------------------------
    function automatic bit [1:0] ref_ternary(
        input bit [7:0] cur,
        input bit [7:0] prev,
        input bit [7:0] eps
    );
        int signed diff;
        begin
            diff = int'(cur) - int'(prev);
            if (diff > int'(eps))
                ref_ternary = 2'b01;
            else if (diff < -int'(eps))
                ref_ternary = 2'b10;
            else
                ref_ternary = 2'b00;
        end
    endfunction

    // -------------------------------------------------------------------
    // Plain procedural stimulus generator (no class/constraint/randomize --
    // kept simple and portable). Reseeds the RNG stream with `seed` first,
    // then draws every field for this cycle from that seeded stream.
    // -------------------------------------------------------------------
    task automatic gen_txn(
        input  int unsigned seed,
        output bit [7:0]    o_cur_pixel,
        output bit [7:0]    o_prev_pixel,
        output bit [7:0]    o_eps,
        output bit          o_valid,
        output bit          o_sof,
        output bit          o_eol
    );
        int unsigned pick;
        begin
            void'($urandom(seed));               // reseed local RNG stream

            o_cur_pixel  = $urandom_range(0, 255);
            o_prev_pixel = $urandom_range(0, 255);
            o_eps        = $urandom_range(0, 255);

            pick    = $urandom_range(0, 99);
            o_valid = (pick < 90);                // ~90% valid cycles

            pick  = $urandom_range(0, 99);
            o_sof = (pick < 5);                   // ~5% start-of-frame

            pick  = $urandom_range(0, 99);
            o_eol = (pick < 5);                   // ~5% end-of-line
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
    // Stimulus + checking
    // -------------------------------------------------------------------
    int unsigned  seed;
    bit  [7:0]    tx_cur, tx_prev, tx_eps;
    bit           tx_valid, tx_sof, tx_eol;
    bit  [7:0]    eps_shadow;   // mirrors dut.eps_reg's *pre-edge* value
    bit  [7:0]    eps_before;   // value in effect for this cycle's compute
    bit  [1:0]    exp_ternary;
    bit           exp_valid, exp_sof, exp_eol;
    real          pass_rate;

    initial begin
        rst_n       = 1'b0;
        eps_wr_en   = 1'b0;
        eps_wr_data = '0;
        cur_pixel   = '0;
        prev_pixel  = '0;
        valid       = 1'b0;
        sof         = 1'b0;
        eol         = 1'b0;

        total_count    = 0;
        match_count    = 0;
        mismatch_count = 0;
        eps_shadow     = EPS_DEFAULT;

        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        @(posedge clk);

        $display("=========================================================");
        $display(" frame_diff_engine_tb : starting %0d randomized cycles", NUM_TESTS);
        $display("=========================================================");

        for (int unsigned i = 0; i < NUM_TESTS; i++) begin
            // ---- fresh random seed every iteration, reseed local stream ----
            seed = $urandom();
            gen_txn(seed, tx_cur, tx_prev, tx_eps, tx_valid, tx_sof, tx_eol);

            // ---- drive stimulus off the clock edge -------------------------
            @(negedge clk);
            cur_pixel  = tx_cur;
            prev_pixel = tx_prev;
            valid      = tx_valid;
            sof        = tx_sof;
            eol        = tx_eol;

            if ((i % EPS_UPDATE_PERIOD) == (EPS_UPDATE_PERIOD - 1)) begin
                eps_wr_en   = 1'b1;
                eps_wr_data = tx_eps;
            end else begin
                eps_wr_en = 1'b0;
            end

            // value the DUT's comparator will actually use *this* cycle is
            // whatever eps_reg already held going into this edge
            eps_before = eps_shadow;

            // ---- let the clock edge happen, then sample ---------------------
            @(posedge clk);
            #1; // allow nonblocking assigns to settle before sampling

            exp_ternary = valid ? ref_ternary(cur_pixel, prev_pixel, eps_before) : 2'b00;
            exp_valid   = valid;
            exp_sof     = sof & valid;
            exp_eol     = eol & valid;

            total_count++;

            if ((ternary   === exp_ternary) &&
                (valid_out === exp_valid)   &&
                (sof_out   === exp_sof)     &&
                (eol_out   === exp_eol)) begin
                match_count++;
            end else begin
                mismatch_count++;
                if (mismatch_count <= mismatch_report_limit) begin
                    $display("[MISMATCH #%0d] t=%0t seed=%0d cur=%0d prev=%0d eps=%0d valid=%0b sof=%0b eol=%0b",
                              mismatch_count, $time, seed, cur_pixel, prev_pixel, eps_before, valid, sof, eol);
                    $display("               expected: ternary=%02b valid=%0b sof=%0b eol=%0b",
                              exp_ternary, exp_valid, exp_sof, exp_eol);
                    $display("               got     : ternary=%02b valid=%0b sof=%0b eol=%0b",
                              ternary, valid_out, sof_out, eol_out);
                end
            end

            // commit the EPS update for use starting *next* cycle, mirroring
            // the DUT's own registered update
            if (eps_wr_en)
                eps_shadow = eps_wr_data;
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
    // Safety timeout (in case something hangs) - re-reads the same plusarg
    // independently so it stays correct even if NUM_TESTS is overriddenrm
       initial begin
        int unsigned timeout_tests;
        if (!$value$plusargs("NUM_TESTS=%d", timeout_tests))
            timeout_tests = 100_000;
        #(CLK_PERIOD_NS * 2 * (timeout_tests + 1000));
        $display("ERROR: testbench timeout - forcing finish");
        $finish;
    end
 
endmodule
 