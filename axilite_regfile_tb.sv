// axi_lite_regfile_tb.sv
// Verification for axi_lite_regfile: an internal AXI4-Lite BFM drives writes/reads,
// a golden scoreboard (independent of the DUT's RTL) tracks expected register
// state, and every read is checked against it. Fixed random seed, ~5,000+
// frame/register-level random transactions (this is a control-plane block,
// accessed at most once per video frame -- same testing philosophy as the
// task_feedback_proxy_controller per the project's verification methodology),
// plus heavy directed testing of: CTRL bits, EPS pulse-write semantics, THRESH_N
// RW* override gating, W1C clear of ROW_BURST_ERROR, split AW/W-phase writes,
// and back-to-back same-cycle AW+W writes.
`timescale 1ns/1ps

module axi_lite_regfile_tb;

    localparam integer SEED           = 32'hFACEFEED;
    localparam integer NUM_RANDOM_TXN = 5000;

    reg clk, rst_n;

    // AXI4-Lite driver signals
    reg  [7:0]  s_axi_awaddr;
    reg         s_axi_awvalid;
    wire        s_axi_awready;
    reg  [31:0] s_axi_wdata;
    reg  [3:0]  s_axi_wstrb;
    reg         s_axi_wvalid;
    wire        s_axi_wready;
    wire [1:0]  s_axi_bresp;
    wire        s_axi_bvalid;
    reg         s_axi_bready;
    reg  [7:0]  s_axi_araddr;
    reg         s_axi_arvalid;
    wire        s_axi_arready;
    wire [31:0] s_axi_rdata;
    wire [1:0]  s_axi_rresp;
    wire        s_axi_rvalid;
    reg         s_axi_rready;

    // plain-port side, driven by testbench to emulate the surrounding modules
    wire        eps_wr_en;
    wire [7:0]  eps_wr_data;
    reg  [7:0]  eps_q;

    wire [3:0]  K_HI, K_LO, STEP, N_MIN, N_MAX;
    wire        override_en;
    wire [3:0]  manual_thresh_N;
    reg  [3:0]  thresh_N_hw;

    reg  [1:0]  row_burst_status;
    reg         row_burst_error_hw;
    wire        row_burst_error_clr;

    reg  [19:0] event_count_frame;
    reg  [19:0] edge_sum_frame;
    reg  [19:0] compressed_bytes;
    reg         frame_tick;
    reg  [31:0] log_buf_wptr;

    wire        pipeline_enable;
    wire [31:0] log_buf_base;
    wire [31:0] log_buf_size;

    integer errors;
    integer checks;

    // Captures the (pulsed, single-cycle) eps_wr_en/eps_wr_data pair whenever it
    // fires, so directed/random checks can inspect it after the fact instead of
    // racing a `wait` against a pulse that may already have passed.
    reg       eps_pulse_seen;
    reg [7:0] eps_pulse_data;
    always @(posedge clk) begin
        if (eps_wr_en) begin
            eps_pulse_seen <= 1'b1;
            eps_pulse_data <= eps_wr_data;
        end
    end

    axi_lite_regfile #(.ADDR_WIDTH(8)) dut (
        .clk(clk), .rst_n(rst_n),
        .s_axi_awaddr(s_axi_awaddr), .s_axi_awvalid(s_axi_awvalid), .s_axi_awready(s_axi_awready),
        .s_axi_wdata(s_axi_wdata),   .s_axi_wstrb(s_axi_wstrb),     .s_axi_wvalid(s_axi_wvalid), .s_axi_wready(s_axi_wready),
        .s_axi_bresp(s_axi_bresp),   .s_axi_bvalid(s_axi_bvalid),   .s_axi_bready(s_axi_bready),
        .s_axi_araddr(s_axi_araddr), .s_axi_arvalid(s_axi_arvalid), .s_axi_arready(s_axi_arready),
        .s_axi_rdata(s_axi_rdata),   .s_axi_rresp(s_axi_rresp),     .s_axi_rvalid(s_axi_rvalid),  .s_axi_rready(s_axi_rready),

        .eps_wr_en(eps_wr_en), .eps_wr_data(eps_wr_data), .eps_q(eps_q),

        .K_HI(K_HI), .K_LO(K_LO), .STEP(STEP), .N_MIN(N_MIN), .N_MAX(N_MAX),
        .override_en(override_en), .manual_thresh_N(manual_thresh_N), .thresh_N_hw(thresh_N_hw),

        .row_burst_status(row_burst_status), .row_burst_error_hw(row_burst_error_hw),
        .row_burst_error_clr(row_burst_error_clr),

        .event_count_frame(event_count_frame), .edge_sum_frame(edge_sum_frame),
        .compressed_bytes(compressed_bytes), .frame_tick(frame_tick), .log_buf_wptr(log_buf_wptr),

        .pipeline_enable(pipeline_enable), .log_buf_base(log_buf_base), .log_buf_size(log_buf_size)
    );

    initial clk = 0;
    always #5 clk = ~clk;

    // ------------------------------------------------------------------
    // Golden scoreboard mirroring the DUT's writable state
    // ------------------------------------------------------------------
    reg        g_pipeline_enable;
    reg        g_override_en;
    reg [7:0]  g_eps_wr_data_last;   // last value pulsed (frame_diff_engine would latch this into eps_q externally)
    reg [3:0]  g_manual_thresh_N;
    reg [3:0]  g_N_MIN, g_N_MAX, g_STEP, g_K_HI, g_K_LO;
    reg [31:0] g_log_buf_base, g_log_buf_size;
    reg [31:0] g_frame_count;
    reg        g_row_burst_error_latched;
    reg        g_row_burst_error_hw_prev;

    localparam [7:0]
        A_CTRL   = 8'h00, A_STATUS = 8'h04, A_EPS   = 8'h08, A_THRESH_N = 8'h0C,
        A_NMIN   = 8'h10, A_NMAX   = 8'h14, A_STEP  = 8'h18, A_KHI      = 8'h1C,
        A_KLO    = 8'h20, A_EVCNT  = 8'h24, A_EDGE  = 8'h28, A_CBYTES   = 8'h2C,
        A_FRAMEC = 8'h30, A_RBSTAT = 8'h34, A_RBERR = 8'h38, A_LOGWPTR  = 8'h3C,
        A_LOGBASE= 8'h40, A_LOGSIZE= 8'h44;

    // ------------------------------------------------------------------
    // AXI4-Lite BFM tasks
    // ------------------------------------------------------------------
    task automatic axi_write(input [7:0] addr, input [31:0] data, input [3:0] strb);
        begin
            @(negedge clk);
            s_axi_awaddr  = addr;
            s_axi_awvalid = 1'b1;
            s_axi_wdata   = data;
            s_axi_wstrb   = strb;
            s_axi_wvalid  = 1'b1;
            s_axi_bready  = 1'b1;

            fork : wait_aw_w
                begin
                    while (!(s_axi_awvalid && s_axi_awready)) @(posedge clk);
                end
                begin
                    while (!(s_axi_wvalid && s_axi_wready)) @(posedge clk);
                end
            join

            @(negedge clk);
            s_axi_awvalid = 1'b0;
            s_axi_wvalid  = 1'b0;

            while (!(s_axi_bvalid && s_axi_bready)) @(posedge clk);
            @(negedge clk);
            s_axi_bready = 1'b0;
        end
    endtask

    task automatic axi_read(input [7:0] addr, output [31:0] data);
        begin
            @(negedge clk);
            s_axi_araddr  = addr;
            s_axi_arvalid = 1'b1;
            s_axi_rready  = 1'b1;

            while (!(s_axi_arvalid && s_axi_arready)) @(posedge clk);
            @(negedge clk);
            s_axi_arvalid = 1'b0;

            while (!(s_axi_rvalid && s_axi_rready)) @(posedge clk);
            data = s_axi_rdata;
            @(negedge clk);
            s_axi_rready = 1'b0;
        end
    endtask

    task automatic golden_write(input [7:0] addr, input [31:0] data, input [3:0] strb);
        begin
            case (addr)
                A_CTRL: if (strb[0]) begin
                    g_pipeline_enable = data[0];
                    g_override_en     = data[1];
                end
                A_EPS: if (strb[0]) begin
                    g_eps_wr_data_last = data[7:0];
                end
                A_THRESH_N: if (strb[0] && g_override_en) begin
                    g_manual_thresh_N = data[3:0];
                end
                A_NMIN:  if (strb[0]) g_N_MIN = data[3:0];
                A_NMAX:  if (strb[0]) g_N_MAX = data[3:0];
                A_STEP:  if (strb[0]) g_STEP  = data[3:0];
                A_KHI:   if (strb[0]) g_K_HI  = data[3:0];
                A_KLO:   if (strb[0]) g_K_LO  = data[3:0];
                A_RBERR: if (strb[0] && data[0]) g_row_burst_error_latched = 1'b0;
                A_LOGBASE: begin
                    if (strb[0]) g_log_buf_base[7:0]   = data[7:0];
                    if (strb[1]) g_log_buf_base[15:8]  = data[15:8];
                    if (strb[2]) g_log_buf_base[23:16] = data[23:16];
                    if (strb[3]) g_log_buf_base[31:24] = data[31:24];
                end
                A_LOGSIZE: begin
                    if (strb[0]) g_log_buf_size[7:0]   = data[7:0];
                    if (strb[1]) g_log_buf_size[15:8]  = data[15:8];
                    if (strb[2]) g_log_buf_size[23:16] = data[23:16];
                    if (strb[3]) g_log_buf_size[31:24] = data[31:24];
                end
                default: ;
            endcase
        end
    endtask

    function automatic [31:0] golden_read(input [7:0] addr);
        begin
            case (addr)
                A_CTRL:    golden_read = {30'd0, g_override_en, g_pipeline_enable};
                A_STATUS:  golden_read = {30'd0, 1'b0, g_pipeline_enable};
                A_EPS:     golden_read = {24'd0, eps_q};
                A_THRESH_N:golden_read = {28'd0, thresh_N_hw};
                A_NMIN:    golden_read = {28'd0, g_N_MIN};
                A_NMAX:    golden_read = {28'd0, g_N_MAX};
                A_STEP:    golden_read = {28'd0, g_STEP};
                A_KHI:     golden_read = {28'd0, g_K_HI};
                A_KLO:     golden_read = {28'd0, g_K_LO};
                A_EVCNT:   golden_read = {12'd0, event_count_frame};
                A_EDGE:    golden_read = {12'd0, edge_sum_frame};
                A_CBYTES:  golden_read = {12'd0, compressed_bytes};
                A_FRAMEC:  golden_read = g_frame_count;
                A_RBSTAT:  golden_read = {30'd0, row_burst_status};
                A_RBERR:   golden_read = {31'd0, g_row_burst_error_latched};
                A_LOGWPTR: golden_read = log_buf_wptr;
                A_LOGBASE: golden_read = g_log_buf_base;
                A_LOGSIZE: golden_read = g_log_buf_size;
                default:   golden_read = 32'd0;
            endcase
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            g_frame_count             <= 32'd0;
            g_row_burst_error_latched <= 1'b0;
            g_row_burst_error_hw_prev <= 1'b0;
        end else begin
            if (frame_tick)
                g_frame_count <= g_frame_count + 32'd1;
            g_row_burst_error_hw_prev <= row_burst_error_hw;
            if (row_burst_error_hw && !g_row_burst_error_hw_prev)
                g_row_burst_error_latched <= 1'b1;
        end
    end

    task automatic do_write_and_check(input [7:0] addr, input [31:0] data, input [3:0] strb);
        begin
            axi_write(addr, data, strb);
            golden_write(addr, data, strb);
        end
    endtask

    task automatic do_read_and_check(input [7:0] addr, input string label);
        reg [31:0] got, exp;
        begin
            axi_read(addr, got);
            exp = golden_read(addr);
            checks = checks + 1;
            if (got !== exp) begin
                errors = errors + 1;
                $display("ERROR [%s] @%0t addr=0x%0h exp=0x%08h got=0x%08h", label, $time, addr, exp, got);
            end
        end
    endtask

    integer i;
    reg [7:0]  raddr_pool [0:16];
    reg [7:0]  waddr_pool [0:10];
    reg [31:0] rnd_data;
    reg [3:0]  rnd_strb;
    reg [7:0]  rnd_addr;
    integer sel;

    initial begin
        errors = 0; checks = 0;
        rst_n = 0;
        s_axi_awaddr=0; s_axi_awvalid=0; s_axi_wdata=0; s_axi_wstrb=0; s_axi_wvalid=0; s_axi_bready=0;
        s_axi_araddr=0; s_axi_arvalid=0; s_axi_rready=0;
        eps_q = 8'd8;
        thresh_N_hw = 4'd4;
        row_burst_status = 2'b00;
        row_burst_error_hw = 1'b0;
        event_count_frame = 20'd0;
        edge_sum_frame = 20'd0;
        compressed_bytes = 20'd0;
        frame_tick = 1'b0;
        log_buf_wptr = 32'd0;

        g_pipeline_enable = 1'b0; g_override_en = 1'b0; g_eps_wr_data_last = 8'd8;
        g_manual_thresh_N = 4'd4; g_N_MIN=4'd1; g_N_MAX=4'd8; g_STEP=4'd1; g_K_HI=4'd2; g_K_LO=4'd1;
        g_log_buf_base=32'd0; g_log_buf_size=32'd0;

        repeat (4) @(posedge clk);
        rst_n = 1;
        repeat (2) @(posedge clk);

        // ================= directed tests =================

        do_read_and_check(A_CTRL,   "reset CTRL");
        do_read_and_check(A_NMIN,   "reset N_MIN");
        do_read_and_check(A_NMAX,   "reset N_MAX");
        do_read_and_check(A_STEP,   "reset STEP");
        do_read_and_check(A_KHI,    "reset K_HI");
        do_read_and_check(A_KLO,    "reset K_LO");

        do_write_and_check(A_CTRL, 32'h0000_0001, 4'hF);
        do_read_and_check(A_CTRL, "ctrl after enable");
        do_read_and_check(A_STATUS, "status after enable");

        do_write_and_check(A_THRESH_N, 32'h0000_0007, 4'hF);
        do_read_and_check(A_THRESH_N, "thresh_N unaffected without override");

        do_write_and_check(A_CTRL, 32'h0000_0003, 4'hF);
        do_write_and_check(A_THRESH_N, 32'h0000_0006, 4'hF);
        thresh_N_hw = 4'd6;
        @(posedge clk);
        do_read_and_check(A_THRESH_N, "thresh_N after override write");

        eps_pulse_seen = 1'b0;
        do_write_and_check(A_EPS, 32'h0000_002A, 4'h1);
        @(posedge clk); // allow the capture always-block one extra cycle to register
        checks = checks + 1;
        if (!eps_pulse_seen) begin
            errors = errors + 1;
            $display("ERROR: eps_wr_en pulse never observed for EPS write");
        end else if (eps_pulse_data !== 8'h2A) begin
            errors = errors + 1;
            $display("ERROR: eps_wr_data mismatch during pulse, got=0x%0h", eps_pulse_data);
        end
        eps_q = 8'h2A;
        @(posedge clk);
        do_read_and_check(A_EPS, "eps readback after latch");

        do_write_and_check(A_NMIN, 32'h0000_0002, 4'hF);
        do_write_and_check(A_NMAX, 32'h0000_0007, 4'hF);
        do_write_and_check(A_STEP, 32'h0000_0003, 4'hF);
        do_write_and_check(A_KHI,  32'h0000_0005, 4'hF);
        do_write_and_check(A_KLO,  32'h0000_0002, 4'hF);
        do_read_and_check(A_NMIN, "n_min after write");
        do_read_and_check(A_NMAX, "n_max after write");
        do_read_and_check(A_STEP, "step after write");
        do_read_and_check(A_KHI,  "k_hi after write");
        do_read_and_check(A_KLO,  "k_lo after write");

        event_count_frame = 20'd12345;
        edge_sum_frame    = 20'd54321;
        compressed_bytes  = 20'd999;
        row_burst_status  = 2'b10;
        log_buf_wptr      = 32'hDEAD_BEEF;
        @(posedge clk);
        do_read_and_check(A_EVCNT, "event_count readback");
        do_read_and_check(A_EDGE,  "edge_sum readback");
        do_read_and_check(A_CBYTES,"compressed_bytes readback");
        do_read_and_check(A_RBSTAT,"row_burst_status readback");
        do_read_and_check(A_LOGWPTR,"log_buf_wptr readback");

        repeat (3) begin
            @(negedge clk);
            frame_tick = 1'b1;
            @(posedge clk);
            @(negedge clk);
            frame_tick = 1'b0;
        end
        @(posedge clk);
        do_read_and_check(A_FRAMEC, "frame_count after 3 ticks");

        @(negedge clk);
        row_burst_error_hw = 1'b1;
        @(posedge clk); @(posedge clk);
        @(negedge clk);
        row_burst_error_hw = 1'b0;
        @(posedge clk);
        do_read_and_check(A_RBERR, "row_burst_error set (sticky)");
        do_write_and_check(A_RBERR, 32'h0000_0001, 4'h1);
        @(posedge clk);
        do_read_and_check(A_RBERR, "row_burst_error cleared after W1C");

        do_write_and_check(A_LOGBASE, 32'h1234_5678, 4'hF);
        do_read_and_check(A_LOGBASE, "log_buf_base full write");
        do_write_and_check(A_LOGBASE, 32'hAAAA_0000, 4'hC);
        do_read_and_check(A_LOGBASE, "log_buf_base partial-strobe write");
        do_write_and_check(A_LOGSIZE, 32'h0010_0000, 4'hF);
        do_read_and_check(A_LOGSIZE, "log_buf_size full write");

        do_write_and_check(8'hF0, 32'hFFFF_FFFF, 4'hF);
        do_read_and_check(8'hF0, "unmapped address returns 0");

        @(negedge clk);
        s_axi_awaddr = A_NMIN; s_axi_awvalid = 1'b1;
        @(posedge clk);
        while (!s_axi_awready) @(posedge clk);
        @(negedge clk);
        s_axi_awvalid = 1'b0;
        s_axi_wdata = 32'h0000_0003; s_axi_wstrb = 4'hF; s_axi_wvalid = 1'b1; s_axi_bready = 1'b1;
        @(posedge clk);
        while (!s_axi_wready) @(posedge clk);
        golden_write(A_NMIN, 32'h0000_0003, 4'hF);
        @(negedge clk);
        s_axi_wvalid = 1'b0;
        while (!(s_axi_bvalid && s_axi_bready)) @(posedge clk);
        @(negedge clk);
        s_axi_bready = 1'b0;
        do_read_and_check(A_NMIN, "n_min after split-phase write");

        // ================= randomized register regression =================
        i = SEED;
        void'($urandom(i));

        raddr_pool[0]=A_CTRL;  raddr_pool[1]=A_STATUS; raddr_pool[2]=A_EPS;    raddr_pool[3]=A_THRESH_N;
        raddr_pool[4]=A_NMIN;  raddr_pool[5]=A_NMAX;   raddr_pool[6]=A_STEP;   raddr_pool[7]=A_KHI;
        raddr_pool[8]=A_KLO;   raddr_pool[9]=A_EVCNT;  raddr_pool[10]=A_EDGE;  raddr_pool[11]=A_CBYTES;
        raddr_pool[12]=A_FRAMEC; raddr_pool[13]=A_RBSTAT; raddr_pool[14]=A_RBERR;
        raddr_pool[15]=A_LOGWPTR; raddr_pool[16]=A_LOGBASE;

        waddr_pool[0]=A_CTRL;  waddr_pool[1]=A_EPS;    waddr_pool[2]=A_THRESH_N; waddr_pool[3]=A_NMIN;
        waddr_pool[4]=A_NMAX;  waddr_pool[5]=A_STEP;   waddr_pool[6]=A_KHI;      waddr_pool[7]=A_KLO;
        waddr_pool[8]=A_RBERR; waddr_pool[9]=A_LOGBASE; waddr_pool[10]=A_LOGSIZE;

        for (i = 0; i < NUM_RANDOM_TXN; i = i + 1) begin
            if ($urandom_range(0,4) == 0) begin
                event_count_frame = $urandom_range(0, 20'hFFFFF);
                edge_sum_frame    = $urandom_range(0, 20'hFFFFF);
                compressed_bytes  = $urandom_range(0, 20'hFFFFF);
                row_burst_status  = $urandom_range(0,3);
                log_buf_wptr      = $urandom;
                @(posedge clk);
            end
            if ($urandom_range(0,9) == 0) begin
                @(negedge clk);
                frame_tick = 1'b1;
                @(posedge clk);
                @(negedge clk);
                frame_tick = 1'b0;
            end
            if (g_override_en && $urandom_range(0,3)==0) begin
                thresh_N_hw = g_manual_thresh_N;
                @(posedge clk);
            end
            if ($urandom_range(0,3)==0) begin
                eps_q = g_eps_wr_data_last;
                @(posedge clk);
            end

            sel = $urandom_range(0,1);
            if (sel == 0) begin
                rnd_addr = waddr_pool[$urandom_range(0,10)];
                rnd_data = $urandom;
                rnd_strb = $urandom_range(1,15);
                do_write_and_check(rnd_addr, rnd_data, rnd_strb);
            end else begin
                rnd_addr = raddr_pool[$urandom_range(0,16)];
                do_read_and_check(rnd_addr, "random");
            end
        end

        $display("--------------------------------------------------");
        $display("AXI_LITE_REGFILE TB: seed=%0d, checks=%0d, errors=%0d", SEED, checks, errors);
        if (errors == 0) $display("RESULT: PASS");
        else $display("RESULT: FAIL");
        $display("--------------------------------------------------");
        $finish;
    end

    initial begin
        #2_000_000;
        $display("TIMEOUT: simulation did not finish in time");
        $finish;
    end

endmodule
