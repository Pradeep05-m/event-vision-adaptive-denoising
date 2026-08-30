// rgb2gray_tb.sv
// Verification: golden-model scoreboard, 1 fixed random seed, 100,000 random vectors
// plus directed edge cases (0,0,0 / 255,255,255 / per-channel max / sof+eol combos).
`timescale 1ns/1ps

module rgb2gray_tb;

    localparam integer SEED       = 32'hC0FFEE01; // fixed seed, deterministic run
    localparam integer NUM_RANDOM = 100000;

    reg         clk;
    reg         rst_n;
    reg  [23:0] rgb_data;
    reg         rgb_valid;
    reg         rgb_sof;
    reg         rgb_eol;

    wire [7:0]  gray;
    wire        gray_valid;
    wire        sof;
    wire        eol;

    integer errors;
    integer checks;

    // golden reference queue (matches 1-cycle latency of DUT)
    reg [7:0] exp_gray_q   [$];
    reg       exp_valid_q  [$];
    reg       exp_sof_q    [$];
    reg       exp_eol_q    [$];

    rgb2gray dut (
        .clk        (clk),
        .rst_n      (rst_n),
        .rgb_data   (rgb_data),
        .rgb_valid  (rgb_valid),
        .rgb_sof    (rgb_sof),
        .rgb_eol    (rgb_eol),
        .gray       (gray),
        .gray_valid (gray_valid),
        .sof        (sof),
        .eol        (eol)
    );

    // 100 MHz-ish test clock (period value irrelevant to functional check)
    initial clk = 0;
    always #5 clk = ~clk;

    // golden model function (independent of DUT's internal bit-slicing choice)
    function automatic [7:0] golden_gray(input [7:0] r, input [7:0] g, input [7:0] b);
        integer sum;
        begin
            sum = (2*r) + (5*g) + b;
            golden_gray = sum >> 3;
        end
    endfunction

    task automatic drive_pixel(input [7:0] r, input [7:0] g, input [7:0] b,
                                input v, input s, input e);
        begin
            @(negedge clk);
            rgb_data  = {r, g, b};
            rgb_valid = v;
            rgb_sof   = s;
            rgb_eol   = e;

            exp_gray_q.push_back(v ? golden_gray(r, g, b) : 8'hxx);
            exp_valid_q.push_back(v);
            exp_sof_q.push_back(s);
            exp_eol_q.push_back(e);
        end
    endtask

    task automatic check_output;
        reg [7:0] eg;
        reg       ev, es, ee;
        begin
            if (exp_valid_q.size() > 0) begin
                eg = exp_gray_q.pop_front();
                ev = exp_valid_q.pop_front();
                es = exp_sof_q.pop_front();
                ee = exp_eol_q.pop_front();

                checks = checks + 1;

                if (gray_valid !== ev) begin
                    errors = errors + 1;
                    $display("ERROR @%0t: gray_valid mismatch. exp=%0b got=%0b", $time, ev, gray_valid);
                end
                if (sof !== es) begin
                    errors = errors + 1;
                    $display("ERROR @%0t: sof mismatch. exp=%0b got=%0b", $time, es, sof);
                end
                if (eol !== ee) begin
                    errors = errors + 1;
                    $display("ERROR @%0t: eol mismatch. exp=%0b got=%0b", $time, ee, eol);
                end
                if (ev && (gray !== eg)) begin
                    errors = errors + 1;
                    $display("ERROR @%0t: gray mismatch. exp=%0d got=%0d", $time, eg, gray);
                end
            end
        end
    endtask

    integer i;
    reg [7:0] r_r, r_g, r_b;
    reg r_v, r_s, r_e;

    initial begin
        errors = 0;
        checks = 0;
        rst_n     = 0;
        rgb_data  = 24'd0;
        rgb_valid = 0;
        rgb_sof   = 0;
        rgb_eol   = 0;

        repeat (4) @(posedge clk);
        rst_n = 1;
        @(negedge clk);

        // ---------------- directed edge cases ----------------
        // Each drive is immediately followed by its check (1 cycle later),
        // matching the DUT's actual pipeline timing instead of driving
        // several vectors back-to-back and checking afterward (which would
        // just resample the final, already-stuck input state repeatedly).
        drive_pixel(8'd0,   8'd0,   8'd0,   1, 1, 0); // black, sof
        @(posedge clk); #1; check_output();
        drive_pixel(8'd255, 8'd255, 8'd255, 1, 0, 0); // white
        @(posedge clk); #1; check_output();
        drive_pixel(8'd255, 8'd0,   8'd0,   1, 0, 0); // pure R max
        @(posedge clk); #1; check_output();
        drive_pixel(8'd0,   8'd255, 8'd0,   1, 0, 0); // pure G max
        @(posedge clk); #1; check_output();
        drive_pixel(8'd0,   8'd0,   8'd255, 1, 0, 1); // pure B max, eol
        @(posedge clk); #1; check_output();
        drive_pixel(8'd128, 8'd64,  8'd32,  1, 0, 0); // mid-range mix
        @(posedge clk); #1; check_output();
        drive_pixel(8'd0,   8'd0,   8'd0,   0, 0, 0); // valid=0, must not corrupt pipeline
        @(posedge clk); #1; check_output();
        drive_pixel(8'd1,   8'd1,   8'd1,   1, 1, 1); // sof+eol same cycle (single-pixel row)
        @(posedge clk); #1; check_output();

        // ---------------- randomized regression, fixed seed ----------------
        i = SEED;
        void'($urandom(i));

        for (i = 0; i < NUM_RANDOM; i = i + 1) begin
            r_r = $urandom_range(0, 255);
            r_g = $urandom_range(0, 255);
            r_b = $urandom_range(0, 255);
            r_v = ($urandom_range(0, 9) != 0); // ~90% valid, 10% bubbles
            r_s = ($urandom_range(0, 639) == 0); // occasional sof
            r_e = ($urandom_range(0, 639) == 639); // occasional eol

            drive_pixel(r_r, r_g, r_b, r_v, r_s, r_e);
            @(posedge clk); #1;
            check_output();
        end

        // drain remaining latency
        repeat (4) begin
            @(negedge clk);
            rgb_valid = 0;
            exp_gray_q.push_back(8'hxx);
            exp_valid_q.push_back(0);
            exp_sof_q.push_back(0);
            exp_eol_q.push_back(0);
            @(posedge clk); #1;
            check_output();
        end

        $display("--------------------------------------------------");
        $display("RGB2GRAY TB: seed=%0d, checks=%0d, errors=%0d", SEED, checks, errors);
        if (errors == 0)
            $display("RESULT: PASS");
        else
            $display("RESULT: FAIL");
        $display("--------------------------------------------------");
        $finish;
    end

endmodule
