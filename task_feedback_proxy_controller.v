// =============================================================================
// task_feedback_proxy_controller.v
//
// Frame-level task feedback controller. It compares the denoised event count
// against the Sobel edge metric using shift-based hysteresis and adjusts the
// adaptive threshold once per frame.
//
// Manual override mode is implemented as a muxed output path: when override_en
// is high, the automatic state machine is frozen and the output is driven by
// manual_thresh_N.
// =============================================================================

module task_feedback_proxy_controller (
    input  wire        clk,
    input  wire        rst_n,

    input  wire [19:0] event_count_frame,
    input  wire [19:0] edge_sum_frame,
    input  wire        frame_tick,

    input  wire [3:0]  K_HI,
    input  wire [3:0]  K_LO,
    input  wire [3:0]  STEP,
    input  wire [3:0]  N_MIN,
    input  wire [3:0]  N_MAX,
    input  wire        override_en,
    input  wire [3:0]  manual_thresh_N,

    output reg  [3:0]  thresh_N
);

    localparam [1:0] IDLE = 2'b00,
                     ACCUM = 2'b01,
                     EVAL  = 2'b10,
                     UPDATE = 2'b11;

    reg [1:0] state, next_state;
    reg [3:0] thresh_next;
    reg [19:0] edge_hi;
    reg [19:0] edge_lo;

    function automatic [19:0] shift_right_u;
        input [19:0] value;
        input [3:0] sh;
        begin
            shift_right_u = value >> sh;
        end
    endfunction

    function automatic [3:0] clamp4;
        input [3:0] value;
        input [3:0] minv;
        input [3:0] maxv;
        begin
            if (value < minv)
                clamp4 = minv;
            else if (value > maxv)
                clamp4 = maxv;
            else
                clamp4 = value;
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            thresh_N <= 4'd1;
            thresh_next <= 4'd1;
            edge_hi <= 20'd0;
            edge_lo <= 20'd0;
        end else begin
            state <= next_state;

            if (state == EVAL) begin
                thresh_N <= thresh_next;
            end

            if (override_en)
                thresh_N <= manual_thresh_N;
        end
    end

    always @(*) begin
        next_state = state;
        thresh_next = thresh_N;
        edge_hi = shift_right_u(edge_sum_frame, K_HI);
        edge_lo = shift_right_u(edge_sum_frame, K_LO);

        case (state)
            IDLE: begin
                if (frame_tick)
                    next_state = ACCUM;
            end

            ACCUM: begin
                if (frame_tick)
                    next_state = EVAL;
            end

            EVAL: begin
                if (event_count_frame > edge_hi) begin
                    thresh_next = clamp4(thresh_N + STEP, N_MIN, N_MAX);
                    next_state = UPDATE;
                end else if (event_count_frame < edge_lo) begin
                    thresh_next = clamp4(thresh_N - STEP, N_MIN, N_MAX);
                    next_state = UPDATE;
                end else begin
                    thresh_next = clamp4(thresh_N, N_MIN, N_MAX);
                    next_state = UPDATE;
                end
            end

            UPDATE: begin
                next_state = ACCUM;
            end

            default: next_state = IDLE;
        endcase

        if (override_en) begin
            thresh_next = manual_thresh_N;
            next_state = ACCUM;
        end
    end

endmodule
