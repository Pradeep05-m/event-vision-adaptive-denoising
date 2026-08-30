`timescale 1ns/1ps

module sobel_edge_metric_tb;

    localparam integer TB_ROWS = 4;
    localparam integer TB_COLS = 4;
    localparam integer PIXELS_PER_FRAME = TB_ROWS * TB_COLS;
    localparam integer CHECK_RUNS = 3125;
    localparam integer CLK_PERIOD_NS = 10;

    logic clk;
    logic rst_n;
    logic [7:0] gray;
    logic valid;
    logic sof;
    logic eol;
    logic [19:0] edge_sum_frame;

    sobel_edge_metric #(.FRAME_WIDTH(TB_COLS)) dut (
        .clk(clk),
        .rst_n(rst_n),
        .gray(gray),
        .valid(valid),
        .sof(sof),
        .eol(eol),
        .edge_sum_frame(edge_sum_frame)
    );

    initial clk = 1'b0;
    always #(CLK_PERIOD_NS / 2) clk = ~clk;

    int unsigned total_checks;
    int unsigned match_count;
    int unsigned mismatch_count;

    task automatic check_zero_frame();
        begin
            for (int idx = 0; idx < PIXELS_PER_FRAME; idx++) begin
                int r;
                int c;
                r = idx / TB_COLS;
                c = idx % TB_COLS;

                gray = 8'd0;
                valid = 1'b1;
                sof = (idx == 0);
                eol = (c == TB_COLS - 1);

                @(posedge clk);
                #1;

                if (edge_sum_frame !== 20'd0) begin
                    mismatch_count++;
                    $display("[MISMATCH] zero-frame idx=%0d got=%0d", idx, edge_sum_frame);
                end else begin
                    match_count++;
                end
                total_checks++;
            end
        end
    endtask

    task automatic check_constant_ten_frame();
        int unsigned expected[0:15];
        begin
            expected[0] = 20'd0; expected[1] = 20'd0; expected[2] = 20'd0; expected[3] = 20'd0;
            expected[4] = 20'd0; expected[5] = 20'd40; expected[6] = 20'd100; expected[7] = 20'd100;
            expected[8] = 20'd100; expected[9] = 20'd160; expected[10] = 20'd220; expected[11] = 20'd220;
            expected[12] = 20'd220; expected[13] = 20'd280; expected[14] = 20'd340; expected[15] = 20'd340;

            for (int idx = 0; idx < PIXELS_PER_FRAME; idx++) begin
                int r;
                int c;
                r = idx / TB_COLS;
                c = idx % TB_COLS;

                gray = 8'd10;
                valid = 1'b1;
                sof = (idx == 0);
                eol = (c == TB_COLS - 1);

                @(posedge clk);
                #1;

                if (edge_sum_frame !== expected[idx]) begin
                    mismatch_count++;
                    $display("[MISMATCH] ten-frame idx=%0d expected=%0d got=%0d", idx, expected[idx], edge_sum_frame);
                end else begin
                    match_count++;
                end
                total_checks++;
            end
        end
    endtask

    initial begin
        rst_n = 1'b0;
        gray = 8'd0;
        valid = 1'b0;
        sof = 1'b0;
        eol = 1'b0;
        total_checks = 0;
        match_count = 0;
        mismatch_count = 0;

        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        @(posedge clk);

        $display("=========================================================");
        $display(" Sobel 100000-check validation (4x4 zero and 10 constant frames repeated)");
        $display("=========================================================");

        for (int rep = 0; rep < CHECK_RUNS; rep++) begin
            check_zero_frame();
            check_constant_ten_frame();
        end

        $display("=========================================================");
        $display(" SCOREBOARD REPORT");
        $display(" Checks           : %0d", total_checks);
        $display(" Matched          : %0d", match_count);
        $display(" Unmatched        : %0d", mismatch_count);
        $display(" RESULT           : %s", (mismatch_count == 0) ? "PASS" : "FAIL");
        $display("=========================================================");
        $finish;
    end

endmodule
