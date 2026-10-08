// Four-edge quadrature decoder. No generated clocks.
// Positive: 00 -> 01 -> 11 -> 10 -> 00 (A is the high bit).
// Confirm A/B and invert on the actual wheel before enabling control.
module encoder_quadrature #(
    parameter integer FILTER_CYCLES = 100 // 2 us at 50 MHz; verify maximum edge rate.
) (
    input wire clk,
    input wire reset_n,
    input wire encoder_a,
    input wire encoder_b,
    input wire invert,
    input wire zero_position, // Caller must gate this with !run_enable.
    output reg signed [31:0] position,
    output reg [15:0] invalid_count,
    output reg overflow
);
    (* syn_keep = 1, syn_preserve = 1 *) reg [1:0] ab_meta;
    (* syn_keep = 1, syn_preserve = 1 *) reg [1:0] ab_sync;
    reg [1:0] candidate;
    reg [1:0] accepted;
    reg initialized;
    reg [31:0] stable_cycles;
    reg signed [2:0] step;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            ab_meta <= 0;
            ab_sync <= 0;
        end else begin
            ab_meta <= {encoder_a, encoder_b};
            ab_sync <= ab_meta;
        end
    end

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            candidate <= 0;
            accepted <= 0;
            initialized <= 0;
            stable_cycles <= 0;
            position <= 0;
            invalid_count <= 0;
            overflow <= 0;
            step = 0;
        end else if (zero_position) begin
            // Align the decoder state too: zeroing must not manufacture an edge.
            candidate <= ab_sync;
            accepted <= ab_sync;
            initialized <= 1;
            stable_cycles <= 0;
            position <= 0;
            invalid_count <= 0;
            overflow <= 0;
        end else if (ab_sync != candidate) begin
            candidate <= ab_sync;
            stable_cycles <= 1;
        end else if (stable_cycles < FILTER_CYCLES) begin
            stable_cycles <= stable_cycles + 1'b1;
        end else if (!initialized) begin
            // First stable power-up level defines the initial decoder state;
            // no artificial +/- count when the wheel starts at 01 or 10.
            accepted <= candidate;
            initialized <= 1;
        end else if (accepted != candidate) begin
            step = 0;
            case ({accepted, candidate})
                4'b0001, 4'b0111, 4'b1110, 4'b1000: step = 1;
                4'b0010, 4'b1011, 4'b1101, 4'b0100: step = -1;
                default: begin
                    if (invalid_count != 16'hffff)
                        invalid_count <= invalid_count + 1'b1;
                end
            endcase
            accepted <= candidate;
            if (invert) step = -step;
            if (step > 0) begin
                if (position == 32'sh7fffffff) overflow <= 1;
                else position <= position + 32'sd1;
            end else if (step < 0) begin
                if (position == 32'sh80000000) overflow <= 1;
                else position <= position - 32'sd1;
            end
        end
    end
endmodule
