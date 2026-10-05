// CS100A / new wide-supply HC-SR04 demo. All logic runs on clk, no generated clocks.
// 50 MHz -> 1 us enable, 20 us TRIG, >=100 ms between trigger starts.
// ECHO MUST be a verified 3.3 V input (or externally level shifted).
module cs100a_demo #(
    parameter integer CLK_HZ = 50000000
) (
    input wire clk,
    input wire reset_n,
    input wire echo,
    output reg trig,
    output reg valid,
    output reg [7:0] error_code,
    output reg [31:0] sample_count,
    output reg [15:0] distance_mm
);
    localparam integer US_DIV = CLK_HZ / 1000000;
    localparam [2:0] IDLE=0, TRIGGER=1, WAIT_RISE=2, MEASURE=3;
    reg [2:0] state;
    reg [15:0] us_divider;
    reg [16:0] interval_us;
    reg [16:0] wait_us;
    reg [15:0] echo_us;
    reg [4:0] trigger_us;
    reg echo_meta, echo_sync;
    // 340 m/s, round to nearest mm: distance = echo_us * 17 / 100.
    wire [31:0] scaled_echo = {16'd0, echo_us} * 32'd17 + 32'd50;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            echo_meta <= 1'b0;
            echo_sync <= 1'b0;
        end else begin
            echo_meta <= echo;
            echo_sync <= echo_meta;
        end
    end

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state <= IDLE;
            us_divider <= 0;
            interval_us <= 0; // Allow 100 ms after reset for module startup.
            wait_us <= 0;
            echo_us <= 0;
            trigger_us <= 0;
            trig <= 0;
            valid <= 0;
            error_code <= 8'd5; // Not measured yet.
            sample_count <= 0;
            distance_mm <= 0;
        end else if (us_divider == US_DIV - 1) begin
            us_divider <= 0;
            if (interval_us < 17'd100000)
                interval_us <= interval_us + 1'b1;
            case (state)
                IDLE: begin
                    trig <= 0;
                    if (interval_us == 17'd100000) begin
                        interval_us <= 0;
                        if (echo_sync) begin
                            valid <= 0;
                            error_code <= 8'd1; // ECHO already high; do not trigger.
                        end else begin
                            trig <= 1;
                            trigger_us <= 0;
                            state <= TRIGGER;
                        end
                    end
                end
                TRIGGER: begin
                    if (trigger_us == 5'd19) begin
                        trig <= 0;
                        wait_us <= 0;
                        state <= WAIT_RISE;
                    end else trigger_us <= trigger_us + 1'b1;
                end
                WAIT_RISE: begin
                    if (echo_sync) begin
                        // Include this first high sample; a pulse spanning N
                        // sampled microseconds must not be reported as N-1.
                        echo_us <= 16'd1;
                        state <= MEASURE;
                    end else if (wait_us == 17'd69999) begin
                        valid <= 0;
                        error_code <= 8'd2; // No rising edge within 70 ms.
                        state <= IDLE;
                    end else wait_us <= wait_us + 1'b1;
                end
                MEASURE: begin
                    if (!echo_sync) begin
                        state <= IDLE;
                        // Specified range 2..450 cm, not a guarantee for every target.
                        if (echo_us >= 16'd118 && echo_us <= 16'd26470) begin
                            distance_mm <= scaled_echo / 32'd100;
                            sample_count <= sample_count + 1'b1;
                            valid <= 1;
                            error_code <= 0;
                        end else begin
                            valid <= 0;
                            error_code <= 8'd4; // Out of the configured valid range.
                        end
                    end else if (echo_us == 16'd29999) begin
                        valid <= 0;
                        error_code <= 8'd3; // Too long; includes ~66 ms no-target pulse.
                        state <= IDLE;
                    end else echo_us <= echo_us + 1'b1;
                end
                default: begin
                    trig <= 0;
                    valid <= 0;
                    error_code <= 8'd5;
                    state <= IDLE;
                end
            endcase
        end else us_divider <= us_divider + 1'b1;
    end
endmodule
