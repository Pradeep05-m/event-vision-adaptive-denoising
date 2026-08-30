`timescale 1ns/1ps

module rle_compression_engine_tb;

    localparam int SEED = 45678;
    localparam int RANDOM_SYMBOL_TARGET = 100000;
    localparam int CLK_PERIOD_NS = 10;
    localparam bit PRINT_DIRECTED_OUTPUT = 1'b1;
    localparam bit PRINT_RANDOM_OUTPUT = 1'b0;

    logic clk;
    logic rst_n;
    logic [1:0] ternary_in;
    logic valid;
    logic sof;
    logic eol;
    logic tready;
    logic [7:0] tdata;
    logic tvalid;
    logic tlast;

    rle_compression_engine dut (
        .clk(clk),
        .rst_n(rst_n),
        .ternary_in(ternary_in),
        .valid(valid),
        .sof(sof),
        .eol(eol),
        .tready(tready),
        .tdata(tdata),
        .tvalid(tvalid),
        .tlast(tlast)
    );

    initial clk = 1'b0;
    always #(CLK_PERIOD_NS / 2) clk = ~clk;

    logic [1:0] in_seq [$];
    logic [7:0] exp_out [$];
    logic [7:0] got_out [$];
    bit got_last [$];

    int unsigned mismatch_count;
    int unsigned protocol_violation_count;
    int unsigned random_symbols_generated;
    int unsigned frames_checked;
    int unsigned total_output_bytes;
    int unsigned hold_check_count;
    int unsigned backpressure_cycle_count;
    int stall_cycles_left;
    int unsigned directed_pass_count;
    int unsigned directed_fail_count;
    bit print_case_output;

    logic prev_tvalid;
    logic prev_tready;
    logic [7:0] prev_tdata;
    logic prev_tlast;

    task automatic print_actual_output(input string case_name, input int max_bytes);
        int i;
        int limit;
        reg [7:0] actual_byte;
        begin
            limit = got_out.size();
            if ((max_bytes > 0) && (limit > max_bytes))
                limit = max_bytes;

            $display("[ACTUAL_OUT] case=%s accepted_bytes=%0d showing=%0d", case_name, got_out.size(), limit);
            for (i = 0; i < limit; i++) begin
                actual_byte = got_out[i];
                $display("  byte[%0d] tdata=0x%02h run_len=%0d symbol=%0d tlast=%0b",
                         i, actual_byte, actual_byte[7:2], actual_byte[1:0], got_last[i]);
            end
            if (got_out.size() > limit)
                $display("  ... %0d more output bytes not printed", got_out.size() - limit);
        end
    endtask

    task automatic build_expected;
        int i;
        int run_len;
        bit [1:0] sym;
        reg [5:0] run_len6;
        begin
            exp_out.delete();
            i = 0;
            while (i < in_seq.size()) begin
                sym = in_seq[i];
                run_len = 1;
                i++;
                while ((i < in_seq.size()) && (in_seq[i] == sym) && (run_len < 63)) begin
                    run_len++;
                    i++;
                end
                run_len6 = run_len[5:0];
                exp_out.push_back({run_len6, sym});
            end
        end
    endtask

    task automatic set_ready(input bit enable_backpressure);
        int r;
        begin
            if (!enable_backpressure) begin
                tready = 1'b1;
            end else if (stall_cycles_left != 0) begin
                tready = 1'b0;
                stall_cycles_left--;
                backpressure_cycle_count++;
            end else begin
                r = $urandom_range(0, 99);
                if (r < 3) begin
                    stall_cycles_left = $urandom_range(1, 3);
                    tready = 1'b0;
                    stall_cycles_left--;
                    backpressure_cycle_count++;
                end else begin
                    tready = 1'b1;
                end
            end
        end
    endtask

    task automatic monitor_cycle(input string case_name);
        begin
            @(negedge clk);

            if (prev_tvalid && !prev_tready) begin
                hold_check_count++;
                if ((tvalid !== prev_tvalid) || (tdata !== prev_tdata) || (tlast !== prev_tlast)) begin
                    protocol_violation_count++;
                    $display("[PROTOCOL_HOLD_FAIL] case=%s prev_data=%02h prev_last=%0b now_valid=%0b now_data=%02h now_last=%0b",
                             case_name, prev_tdata, prev_tlast, tvalid, tdata, tlast);
                end
            end

            if (tvalid && tready) begin
                got_out.push_back(tdata);
                got_last.push_back(tlast);
                total_output_bytes++;
            end

            prev_tvalid = tvalid;
            prev_tready = tready;
            prev_tdata  = tdata;
            prev_tlast  = tlast;

            @(posedge clk);
            #1;
        end
    endtask

    task automatic compare_case(input string case_name, output bit pass);
        int i;
        int last_count;
        int last_index;
        begin
            pass = 1'b1;
            last_count = 0;
            last_index = -1;

            foreach (got_last[i]) begin
                if (got_last[i]) begin
                    last_count++;
                    last_index = i;
                end
            end

            if (got_out.size() != exp_out.size()) begin
                pass = 1'b0;
                mismatch_count++;
                $display("[LENGTH_MISMATCH] case=%s expected_bytes=%0d actual_bytes=%0d input_symbols=%0d",
                         case_name, exp_out.size(), got_out.size(), in_seq.size());
            end

            if (last_count != 1 || last_index != (exp_out.size() - 1)) begin
                pass = 1'b0;
                protocol_violation_count++;
                $display("[TLAST_FAIL] case=%s last_count=%0d last_index=%0d expected_last_index=%0d",
                         case_name, last_count, last_index, exp_out.size() - 1);
            end

            if (got_out.size() == exp_out.size()) begin
                for (i = 0; i < exp_out.size(); i++) begin
                    if (got_out[i] !== exp_out[i]) begin
                        pass = 1'b0;
                        mismatch_count++;
                        $display("[BYTE_MISMATCH] case=%s byte_idx=%0d expected=%02h actual=%02h input_symbols=%0d",
                                 case_name, i, exp_out[i], got_out[i], in_seq.size());
                        i = exp_out.size();
                    end
                end
            end
        end
    endtask

    task automatic run_case(input string case_name, input bit enable_backpressure, output bit pass);
        int i;
        int drain_guard;
        bit saw_last;
        begin
            build_expected();
            got_out.delete();
            got_last.delete();
            stall_cycles_left = 0;
            saw_last = 1'b0;

            prev_tvalid = 1'b0;
            prev_tready = 1'b1;
            prev_tdata  = 8'd0;
            prev_tlast  = 1'b0;

            for (i = 0; i < in_seq.size(); i++) begin
                set_ready(enable_backpressure);
                ternary_in = in_seq[i];
                valid = 1'b1;
                sof = (i == 0);
                eol = (i == in_seq.size() - 1);
                monitor_cycle(case_name);
            end

            valid = 1'b0;
            sof = 1'b0;
            eol = 1'b0;
            ternary_in = 2'b00;

            drain_guard = 0;
            while (!saw_last && drain_guard < 20000) begin
                set_ready(enable_backpressure);
                monitor_cycle(case_name);
                if ((got_last.size() != 0) && got_last[got_last.size() - 1])
                    saw_last = 1'b1;
                drain_guard++;
            end

            tready = 1'b1;
            repeat (4) monitor_cycle(case_name);

            compare_case(case_name, pass);
            if (!pass) begin
                $write("  input=");
                foreach (in_seq[k]) $write(" %0d", in_seq[k]);
                $write("\n  expected=");
                foreach (exp_out[k]) $write(" %02h", exp_out[k]);
                $write("\n  actual=");
                foreach (got_out[k]) $write(" %02h", got_out[k]);
                $write("\n");
            end

            if (print_case_output && ((!enable_backpressure && PRINT_DIRECTED_OUTPUT) ||
                                      (enable_backpressure && PRINT_RANDOM_OUTPUT))) begin
                if (enable_backpressure)
                    print_actual_output(case_name, 32);
                else
                    print_actual_output(case_name, 0);
            end

            frames_checked++;
        end
    endtask

    task automatic make_repeated(input bit [1:0] sym, input int count);
        int i;
        begin
            in_seq.delete();
            for (i = 0; i < count; i++)
                in_seq.push_back(sym);
        end
    endtask

    task automatic make_alternating(input int count);
        int i;
        begin
            in_seq.delete();
            for (i = 0; i < count; i++)
                in_seq.push_back(i[1:0]);
        end
    endtask

    task automatic make_random_frame(input int max_symbols);
        int frame_len;
        int run_len;
        int bucket;
        int i;
        bit [1:0] sym;
        begin
            in_seq.delete();
            frame_len = $urandom_range(1, max_symbols);
            while (in_seq.size() < frame_len) begin
                sym = $urandom_range(0, 3);
                bucket = $urandom_range(0, 9);
                if (bucket < 6)
                    run_len = $urandom_range(1, 8);
                else if (bucket < 8)
                    run_len = $urandom_range(9, 32);
                else
                    run_len = $urandom_range(50, 80);

                if ((in_seq.size() + run_len) > frame_len)
                    run_len = frame_len - in_seq.size();

                for (i = 0; i < run_len; i++)
                    in_seq.push_back(sym);
            end
        end
    endtask

    task automatic record_directed(input string name, input bit pass);
        begin
            if (pass) begin
                directed_pass_count++;
                $display("[DIRECTED_PASS] %s", name);
            end else begin
                directed_fail_count++;
                $display("[DIRECTED_FAIL] %s", name);
            end
        end
    endtask

    initial begin
        int seed_state;
        bit pass;
        int remaining;
        int frame_max;

        seed_state = SEED;
        seed_state = $urandom(seed_state);

        rst_n = 1'b0;
        ternary_in = 2'b00;
        valid = 1'b0;
        sof = 1'b0;
        eol = 1'b0;
        tready = 1'b1;
        mismatch_count = 0;
        protocol_violation_count = 0;
        random_symbols_generated = 0;
        frames_checked = 0;
        total_output_bytes = 0;
        hold_check_count = 0;
        backpressure_cycle_count = 0;
        stall_cycles_left = 0;
        directed_pass_count = 0;
        directed_fail_count = 0;
        print_case_output = 1'b1;
        prev_tvalid = 1'b0;
        prev_tready = 1'b1;
        prev_tdata = 8'd0;
        prev_tlast = 1'b0;

        repeat (8) @(posedge clk);
        rst_n = 1'b1;
        repeat (2) @(posedge clk);

        $display("=========================================================");
        $display(" rle_compression_engine_tb");
        $display(" Seed                 : %0d", SEED);
        $display(" Random symbol target : %0d", RANDOM_SYMBOL_TARGET);
        $display("=========================================================");

        make_repeated(2'b01, 63);
        run_case("directed_exact_63", 1'b0, pass);
        record_directed("exact run of 63", pass);

        make_repeated(2'b10, 64);
        run_case("directed_exact_64", 1'b0, pass);
        record_directed("exact run of 64", pass);

        make_repeated(2'b11, 1);
        run_case("directed_single_symbol", 1'b0, pass);
        record_directed("single-symbol frame", pass);

        make_alternating(128);
        run_case("directed_alternating", 1'b0, pass);
        record_directed("fully alternating stream", pass);

        while (random_symbols_generated < RANDOM_SYMBOL_TARGET) begin
            remaining = RANDOM_SYMBOL_TARGET - random_symbols_generated;
            frame_max = (remaining > 2048) ? 2048 : remaining;
            make_random_frame(frame_max);
            random_symbols_generated += in_seq.size();
            run_case("random_backpressure", 1'b1, pass);
        end

        $display("=========================================================");
        $display(" SCOREBOARD REPORT");
        $display(" Seed                         : %0d", SEED);
        $display(" Frames checked               : %0d", frames_checked);
        $display(" Random symbols checked       : %0d", random_symbols_generated);
        $display(" Output bytes accepted        : %0d", total_output_bytes);
        $display(" Data mismatch count          : %0d", mismatch_count);
        $display(" Protocol violation count     : %0d", protocol_violation_count);
        $display(" Backpressure cycles          : %0d", backpressure_cycle_count);
        $display(" Output hold checks           : %0d", hold_check_count);
        $display(" Directed tests passed/failed : %0d/%0d", directed_pass_count, directed_fail_count);
        if (mismatch_count == 0 && protocol_violation_count == 0 && directed_fail_count == 0)
            $display(" RESULT                       : PASS");
        else
            $display(" RESULT                       : FAIL");
        $display("=========================================================");
        $finish;
    end

endmodule
