// CS100A / new wide-supply HC-SR04 demo. All logic runs on clk, no generated clocks.
// 50 MHz -> 1 us timestamps, 50 us TRIG (seller example), >=100 ms starts.
// Capture ECHO edges on EVERY clock, including while TRIG is still high.
// Retain the complete ~66 ms no-target pulse, then reject it as invalid.
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
    reg [5:0] trigger_us;
    reg echo_meta, echo_sync, echo_previous;
    reg echo_armed, echo_started, echo_finished;
    reg [31:0] timestamp_us, rise_timestamp_us, echo_width_us;
    wire echo_rise = echo_sync && !echo_previous;
    wire echo_fall = !echo_sync && echo_previous;
    // Unsigned subtraction also works when the 32-bit timestamp wraps.
    wire [31:0] active_echo_us = timestamp_us - rise_timestamp_us;
    // 340 m/s, round to nearest mm: distance = echo_us * 17 / 100.
    wire [31:0] scaled_echo = echo_width_us * 32'd17 + 32'd50;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            echo_meta <= 1'b0;
            echo_sync <= 1'b0;
            echo_previous <= 1'b0;
        end else begin
            echo_meta <= echo;
            echo_sync <= echo_meta;
            echo_previous <= echo_sync;
        end
    end

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state <= IDLE;
            us_divider <= 0;
            interval_us <= 0; // Allow 100 ms after reset for module startup.
            wait_us <= 0;
            trigger_us <= 0;
            echo_armed <= 0;
            echo_started <= 0;
            echo_finished <= 0;
            timestamp_us <= 0;
            rise_timestamp_us <= 0;
            echo_width_us <= 0;
            trig <= 0;
            valid <= 0;
            error_code <= 8'd5; // Not measured yet.
            sample_count <= 0;
            distance_mm <= 0;
        end else begin
            // Sticky edge captures cannot be lost between 1 us enables or
            // while the trigger FSM moves from TRIGGER to WAIT_RISE.
            if (echo_armed && echo_rise && !echo_started) begin
                echo_started <= 1;
                rise_timestamp_us <= timestamp_us;
            end
            if (echo_armed && echo_started && echo_fall && !echo_finished) begin
                echo_finished <= 1;
                echo_width_us <= active_echo_us;
            end
            if (us_divider == US_DIV - 1) begin
                us_divider <= 0;
                timestamp_us <= timestamp_us + 1'b1;
                if (interval_us < 17'd100000)
                    interval_us <= interval_us + 1'b1;
                case (state)
                    IDLE: begin
                        trig <= 0;
                        if (interval_us == 17'd100000) begin
                            interval_us <= 0;
                            echo_started <= 0;
                            echo_finished <= 0;
                            echo_width_us <= 0;
                            if (echo_sync) begin
                                echo_armed <= 0;
                                valid <= 0;
                                error_code <= 8'd1; // ECHO already high; do not trigger.
                            end else begin
                                trig <= 1;
                                echo_armed <= 1;
                                trigger_us <= 0;
                                state <= TRIGGER;
                            end
                        end
                    end
                    TRIGGER: begin
                        if (trigger_us == 6'd49) begin
                            trig <= 0;
                            wait_us <= 0;
                            state <= WAIT_RISE;
                        end else trigger_us <= trigger_us + 1'b1;
                    end
                    WAIT_RISE: begin
                        if (echo_started) begin
                            state <= MEASURE;
                        end else if (wait_us == 17'd69999) begin
                            valid <= 0;
                            error_code <= 8'd2; // No rising edge within 70 ms.
                            echo_armed <= 0;
                            state <= IDLE;
                        end else wait_us <= wait_us + 1'b1;
                    end
                    MEASURE: begin
                        if (echo_finished) begin
                            state <= IDLE;
                            echo_armed <= 0;
                            // Specified range 2..450 cm, not a guarantee for every target.
                            if (echo_width_us >= 32'd60000) begin
                                valid <= 0;
                                error_code <= 8'd3; // Seller's ~66 ms no-target pulse.
                            end else if (echo_width_us >= 32'd118 && echo_width_us <= 32'd26470) begin
                                distance_mm <= scaled_echo / 32'd100;
                                sample_count <= sample_count + 1'b1;
                                valid <= 1;
                                error_code <= 0;
                            end else begin
                                valid <= 0;
                                error_code <= 8'd4; // Out of the configured valid range.
                            end
                        end else if (active_echo_us >= 32'd70000) begin
                            valid <= 0;
                            error_code <= 8'd3; // ECHO remains high for >=70 ms.
                            echo_armed <= 0;
                            state <= IDLE;
                        end
                    end
                    default: begin
                        trig <= 0;
                        valid <= 0;
                        error_code <= 8'd5;
                        echo_armed <= 0;
                        state <= IDLE;
                    end
                endcase
            end else us_divider <= us_divider + 1'b1;
        end
    end
endmodule
