`timescale 1ns/1ps

module task_feedback_proxy_controller_tb;

    localparam int SEED = 34567;
    localparam int CLK_PERIOD_NS = 10;
    localparam int NUM_TESTS = 100000;

    logic clk;
    logic rst_n;
    logic [19:0] event_count_frame;
    logic [19:0] edge_sum_frame;
    logic frame_tick;
    logic [3:0] K_HI;
    logic [3:0] K_LO;
    logic [3:0] STEP;
    logic [3:0] N_MIN;
    logic [3:0] N_MAX;
    logic override_en;
    logic [3:0] manual_thresh_N;
    logic [3:0] thresh_N;

    task_feedback_proxy_controller dut (
        .clk(clk),
        .rst_n(rst_n),
        .event_count_frame(event_count_frame),
        .edge_sum_frame(edge_sum_frame),
        .frame_tick(frame_tick),
        .K_HI(K_HI),
        .K_LO(K_LO),
        .STEP(STEP),
        .N_MIN(N_MIN),
        .N_MAX(N_MAX),
        .override_en(override_en),
        .manual_thresh_N(manual_thresh_N),
        .thresh_N(thresh_N)
    );

    initial clk = 1'b0;
    always #(CLK_PERIOD_NS / 2) clk = ~clk;

    int unsigned total_count;
    int unsigned match_count;
    int unsigned mismatch_count;
    int unsigned expected_thresh;
    int unsigned current_thresh;
    bit override_pass;
    bit clamp_pass;

    function automatic [3:0] clamp4_fn(input [3:0] value, input [3:0] minv, input [3:0] maxv);
        begin
            if (value < minv)
                clamp4_fn = minv;
            else if (value > maxv)
                clamp4_fn = maxv;
            else
                clamp4_fn = value;
        end
    endfunction

    task automatic pulse_frame();
        begin
            frame_tick = 1'b1;
            repeat (2) @(posedge clk);
            #1;
            frame_tick = 1'b0;
            repeat (2) @(posedge clk);
            #1;
        end
    endtask

    initial begin
        rst_n = 1'b0;
        event_count_frame = 20'd0;
        edge_sum_frame = 20'd0;
        frame_tick = 1'b0;
        K_HI = 4'd2;
        K_LO = 4'd4;
        STEP = 4'd1;
        N_MIN = 4'd1;
        N_MAX = 4'd8;
        override_en = 1'b0;
        manual_thresh_N = 4'd0;
        current_thresh = 4'd1;

        total_count = 0;
        match_count = 0;
        mismatch_count = 0;
        override_pass = 1'b1;
        clamp_pass = 1'b1;

        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        @(posedge clk);

        $display("=========================================================");
        $display(" task_feedback_proxy_controller_tb : %0d randomized checks", NUM_TESTS);
        $display("=========================================================");

        for (int i = 0; i < NUM_TESTS; i++) begin
            K_HI = $urandom_range(1, 8);
            K_LO = $urandom_range(1, 8);
            STEP = $urandom_range(1, 4);
            N_MIN = $urandom_range(1, 4);
            N_MAX = $urandom_range(5, 8);
            if (N_MIN >= N_MAX)
                N_MAX = N_MIN + 1;

            event_count_frame = $urandom_range(0, 1023);
            edge_sum_frame = $urandom_range(0, 1023);
            override_en = (i % 17 == 0) ? 1'b1 : 1'b0;
            manual_thresh_N = $urandom_range(0, 15);

            if (override_en)
                expected_thresh = manual_thresh_N;
            else begin
                if (event_count_frame > (edge_sum_frame >> K_HI))
                    expected_thresh = clamp4_fn(current_thresh + STEP, N_MIN, N_MAX);
                else if (event_count_frame < (edge_sum_frame >> K_LO))
                    expected_thresh = clamp4_fn(current_thresh - STEP, N_MIN, N_MAX);
                else
                    expected_thresh = clamp4_fn(current_thresh, N_MIN, N_MAX);
            end

            pulse_frame();
            total_count++;

            if (override_en) begin
                if (thresh_N !== manual_thresh_N) begin
                    mismatch_count++;
                    if (mismatch_count <= 10) begin
                        $display("[MISMATCH] iter=%0d override expected=%0d got=%0d", i, manual_thresh_N, thresh_N);
                    end
                end else begin
                    match_count++;
                end
            end else begin
                if (thresh_N !== expected_thresh) begin
                    mismatch_count++;
                    if (mismatch_count <= 10) begin
                        $display("[MISMATCH] iter=%0d event=%0d edge=%0d KHI=%0d KLO=%0d expected=%0d got=%0d",
                                 i, event_count_frame, edge_sum_frame, K_HI, K_LO, expected_thresh, thresh_N);
                    end
                end else begin
                    match_count++;
                end
            end

            current_thresh = thresh_N;
        end

        // Clamp check.
        K_HI = 4'd1; K_LO = 4'd2; STEP = 4'd3; N_MIN = 4'd1; N_MAX = 4'd4;
        current_thresh = 4'd4;
        event_count_frame = 20'd1000;
        edge_sum_frame = 20'd1;
        override_en = 1'b0;
        pulse_frame();
        if (thresh_N !== N_MAX) begin
            clamp_pass = 1'b0;
            $display("[CLAMP_FAIL] max clamp violated: got=%0d expected=%0d", thresh_N, N_MAX);
        end

        // Manual override check.
        override_en = 1'b1;
        manual_thresh_N = 4'd7;
        event_count_frame = 20'd0;
        edge_sum_frame = 20'd0;
        pulse_frame();
        if (thresh_N !== 4'd7) begin
            override_pass = 1'b0;
            $display("[OVERRIDE_FAIL] expected=7 got=%0d", thresh_N);
        end

        $display("=========================================================");
        $display(" SCOREBOARD REPORT");
        $display(" Total checks     : %0d", total_count + 2);
        $display(" Matched          : %0d", match_count + 2);
        $display(" Mismatch         : %0d", mismatch_count);
        $display(" Clamp pass       : %0s", clamp_pass ? "YES" : "NO");
        $display(" Override pass    : %0s", override_pass ? "YES" : "NO");
        $display(" RESULT           : %s", (mismatch_count == 0 && clamp_pass && override_pass) ? "PASS" : "FAIL");
        $display("=========================================================");
        $finish;
    end

endmodule
