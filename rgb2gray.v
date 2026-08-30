// rgb2gray.v
// RGB2Gray: pixel-clock-domain 24-bit RGB -> 8-bit luma conversion.
// gray = (2*R + 5*G + 1*B) >> 3, implemented purely as shift/add (no multiplier),
// per event_denoising_architecture.md sec 2.2:
//   gray = (R<<1) + (G<<2) + G + B, then >> 3
// 1-cycle pipeline register; sof/eol/valid pass through with matching 1-cycle delay.

module rgb2gray (
    input  wire        clk,
    input  wire        rst_n,

    input  wire [23:0] rgb_data,   // {R[23:16], G[15:8], B[7:0]}
    input  wire        rgb_valid,
    input  wire        rgb_sof,
    input  wire        rgb_eol,

    output reg  [7:0]  gray,
    output reg         gray_valid,
    output reg         sof,
    output reg         eol
);

    wire [7:0] r_in = rgb_data[23:16];
    wire [7:0] g_in = rgb_data[15:8];
    wire [7:0] b_in = rgb_data[7:0];

    // Widen enough to avoid overflow: max sum = 2*255+5*255+255 = 2040, fits in 11 bits.
    wire [10:0] r_x2  = {r_in, 1'b0};        // R << 1  (9 bits used)
    wire [10:0] g_x4  = {g_in, 2'b00};       // G << 2  (10 bits used)
    wire [10:0] sum   = r_x2 + g_x4 + g_in + b_in; // 2R + 5G + B
    wire [7:0]  gray_comb = sum[10:3];       // >> 3, truncate to 8 bits (sum fits in <=11 bits, /8 fits in 8 bits since max 2040/8=255)

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gray       <= 8'd0;
            gray_valid <= 1'b0;
            sof        <= 1'b0;
            eol        <= 1'b0;
        end else begin
            gray       <= gray_comb;
            gray_valid <= rgb_valid;
            sof        <= rgb_sof;
            eol        <= rgb_eol;
        end
    end

endmodule
