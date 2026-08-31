`timescale 1ns/1ps
module hdmi_sync_tb;

    localparam integer SEED        = 32'h5EED1234;
    localparam integer NUM_FRAMES  = 300; // randomized regression, in addition to the directed check below

    localparam integer FRAME_WIDTH = 640;
    localparam integer H_TOTAL     = 800; // active + blanking, arbitrary but > FRAME_WIDTH
    localparam integer V_ACTIVE    = 3;   // small frame height for a fast sim (logic doesn't care about height)
    localparam integer V_TOTAL     = 5;

    reg clk, rst_n;
    reg de, vsync;
    wire valid, sof, eol, row_start;

    hdmi_sync_to_handshake #(.FRAME_WIDTH(FRAME_WIDTH)) dut (
        .pixel_clk(clk), .rst_n(rst_n), .de(de), .vsync(vsync),
        .valid(valid), .sof(sof), .eol(eol), .row_start(row_start)
    );

    initial clk = 0;
    always #5 clk = ~clk;

    integer h, v;
    integer errors, checks;
    integer sof_count, eol_count, row_start_count;
    integer col_within_line;
    reg     expect_sof_next_de;

    initial begin
        errors = 0; checks = 0; sof_count = 0; eol_count = 0; row_start_count = 0;
        rst_n = 0; de = 0; vsync = 0;
        repeat (4) @(posedge clk);
        rst_n = 1;
        @(negedge clk);

        expect_sof_next_de = 1'b0;

        for (v = 0; v < V_TOTAL; v = v + 1) begin
            // one frame's worth of vsync pulse at the top of the frame
            @(negedge clk);
            vsync = (v == 0) ? 1'b1 : 1'b0; // pulse vsync once per test, at very first frame boundary for simplicity
            if (v == 0) expect_sof_next_de = 1'b1;
            @(posedge clk);
            @(negedge clk);
            vsync = 1'b0;

            for (h = 0; h < H_TOTAL; h = h + 1) begin
                @(negedge clk);
                de = (h < FRAME_WIDTH); // active video region vs horizontal blanking
                @(posedge clk); #1;

                if (de) begin
                    checks = checks + 1;
                    if (valid !== 1'b1) begin
                        errors = errors + 1;
                        $display("ERROR: valid not asserted during DE at v=%0d h=%0d", v, h);
                    end

                    col_within_line = h; // 0-based column index within active region

                    if (expect_sof_next_de) begin
                        checks = checks + 1;
                        if (sof !== 1'b1) begin
                            errors = errors + 1;
                            $display("ERROR: sof missing on expected first pixel, v=%0d h=%0d", v, h);
                        end else begin
                            sof_count = sof_count + 1;
                        end
                        expect_sof_next_de = 1'b0;
                    end else begin
                        checks = checks + 1;
                        if (sof !== 1'b0) begin
                            errors = errors + 1;
                            $display("ERROR: unexpected sof at v=%0d h=%0d", v, h);
                        end
                    end

                    if (col_within_line == FRAME_WIDTH-1) begin
                        checks = checks + 1;
                        if (eol !== 1'b1) begin
                            errors = errors + 1;
                            $display("ERROR: eol missing at last column, v=%0d h=%0d", v, h);
                        end else begin
                            eol_count = eol_count + 1;
                        end
                    end else begin
                        checks = checks + 1;
                        if (eol !== 1'b0) begin
                            errors = errors + 1;
                            $display("ERROR: unexpected eol at v=%0d h=%0d", v, h);
                        end
                    end

                    // row_start must fire on the first active pixel of EVERY
                    // line (col_within_line==0), including the frame's first
                    // row where it coincides with sof -- distinct from sof,
                    // which only fires once per frame.
                    if (col_within_line == 0) begin
                        checks = checks + 1;
                        if (row_start !== 1'b1) begin
                            errors = errors + 1;
                            $display("ERROR: row_start missing at line start, v=%0d h=%0d", v, h);
                        end else begin
                            row_start_count = row_start_count + 1;
                        end
                    end else begin
                        checks = checks + 1;
                        if (row_start !== 1'b0) begin
                            errors = errors + 1;
                            $display("ERROR: unexpected row_start at v=%0d h=%0d", v, h);
                        end
                    end
                end else begin
                    checks = checks + 1;
                    if (valid !== 1'b0) begin
                        errors = errors + 1;
                        $display("ERROR: valid asserted during blanking at v=%0d h=%0d", v, h);
                    end
                end
            end
        end

        @(negedge clk); de = 0; vsync = 0;
        @(posedge clk);

        $display("--------------------------------------------------");
        $display("HDMI_SYNC_TB DIRECTED: checks=%0d errors=%0d sof_count=%0d eol_count=%0d row_start_count=%0d (expect sof_count=1, eol_count=%0d, row_start_count=%0d)",
                   checks, errors, sof_count, eol_count, row_start_count, V_TOTAL, V_TOTAL);
        $display("--------------------------------------------------");

        // ================= randomized regression, fixed seed =================
        // Random horizontal blanking width and random frame height per frame,
        // across NUM_FRAMES randomized frames. Same per-pixel invariants checked
        // throughout: valid==de, exactly one sof per frame (on the correct first
        // active pixel following a vsync rising edge), and eol asserted exactly
        // on the last (FRAME_WIDTH-1) active column of every line.
        begin
            integer f, line, hblank, frame_height, hh;
            integer rand_sof_count, rand_eol_count_this_frame;
            integer expect_eol_count_per_frame;

            f = SEED;
            void'($urandom(f));

            for (f = 0; f < NUM_FRAMES; f = f + 1) begin
                frame_height = $urandom_range(1, 8);

                // vsync pulse marks the start of this frame
                @(negedge clk);
                vsync = 1'b1;
                expect_sof_next_de = 1'b1;
                @(posedge clk);
                @(negedge clk);
                vsync = 1'b0;

                for (line = 0; line < frame_height; line = line + 1) begin
                    hblank = $urandom_range(5, 120); // random horizontal blanking length

                    // active region: FRAME_WIDTH cycles of de=1
                    for (hh = 0; hh < FRAME_WIDTH; hh = hh + 1) begin
                        @(negedge clk);
                        de = 1'b1;
                        @(posedge clk); #1;

                        checks = checks + 1;
                        if (valid !== 1'b1) begin
                            errors = errors + 1;
                            $display("ERROR(rand): valid not asserted during DE, frame=%0d line=%0d col=%0d", f, line, hh);
                        end

                        if (expect_sof_next_de) begin
                            checks = checks + 1;
                            if (sof !== 1'b1) begin
                                errors = errors + 1;
                                $display("ERROR(rand): sof missing on expected first pixel, frame=%0d line=%0d col=%0d", f, line, hh);
                            end
                            expect_sof_next_de = 1'b0;
                        end else begin
                            checks = checks + 1;
                            if (sof !== 1'b0) begin
                                errors = errors + 1;
                                $display("ERROR(rand): unexpected sof, frame=%0d line=%0d col=%0d", f, line, hh);
                            end
                        end

                        checks = checks + 1;
                        if (hh == FRAME_WIDTH-1) begin
                            if (eol !== 1'b1) begin
                                errors = errors + 1;
                                $display("ERROR(rand): eol missing at last column, frame=%0d line=%0d col=%0d", f, line, hh);
                            end
                        end else begin
                            if (eol !== 1'b0) begin
                                errors = errors + 1;
                                $display("ERROR(rand): unexpected eol, frame=%0d line=%0d col=%0d", f, line, hh);
                            end
                        end

                        checks = checks + 1;
                        if (hh == 0) begin
                            if (row_start !== 1'b1) begin
                                errors = errors + 1;
                                $display("ERROR(rand): row_start missing at line start, frame=%0d line=%0d col=%0d", f, line, hh);
                            end else begin
                                row_start_count = row_start_count + 1;
                            end
                        end else begin
                            if (row_start !== 1'b0) begin
                                errors = errors + 1;
                                $display("ERROR(rand): unexpected row_start, frame=%0d line=%0d col=%0d", f, line, hh);
                            end
                        end
                    end

                    // horizontal blanking: random length, de=0
                    for (hh = 0; hh < hblank; hh = hh + 1) begin
                        @(negedge clk);
                        de = 1'b0;
                        @(posedge clk); #1;
                        checks = checks + 1;
                        if (valid !== 1'b0) begin
                            errors = errors + 1;
                            $display("ERROR(rand): valid asserted during blanking, frame=%0d line=%0d blank_idx=%0d", f, line, hh);
                        end
                    end
                end
            end

            @(negedge clk); de = 0; vsync = 0;
            @(posedge clk);
        end

        $display("--------------------------------------------------");
        $display("HDMI_SYNC_TB TOTAL: seed=%0d, checks=%0d, errors=%0d, row_start_count=%0d", SEED, checks, errors, row_start_count);
        if (errors == 0)
            $display("RESULT: PASS");
        else
            $display("RESULT: FAIL");
        $display("--------------------------------------------------");
        $finish;
    end

endmodule