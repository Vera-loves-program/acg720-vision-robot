// Signed TB6612 drive: zero/disarmed/fault = IN1=IN2=0 and PWM=1.
// TB6612 STBY=1 truth table calls this STOP (high impedance / coast).
// PWM=0 would short-brake. Logical duty=0 therefore has physical PWM HIGH.
// This command cannot replace hardware STBY/estop during FPGA configuration.
module tb6612_axis_pwm #(
    parameter integer CLK_HZ = 50000000,
    parameter integer PWM_HZ = 20000,
    parameter integer REVERSE_BLANK_US = 1000
) (
    input wire clk,
    input wire reset_n,
    input wire enable,
    input wire signed [15:0] duty_permille,
    output wire pwm,
    output wire in1,
    output wire in2
);
    localparam integer PERIOD_CYCLES = CLK_HZ / PWM_HZ;
    localparam integer BLANK_CYCLES = (CLK_HZ / 1000000) * REVERSE_BLANK_US;
    reg [31:0] pwm_counter;
    reg [31:0] blank_remaining;
    reg active_direction;
    reg direction_valid;
    reg drive_active;
    wire requested_direction = !duty_permille[15];
    wire requested_nonzero = (duty_permille != 0);
    wire signed [31:0] duty_wide = {{16{duty_permille[15]}}, duty_permille};
    wire [31:0] magnitude = duty_wide < 0 ? -duty_wide : duty_wide;
    wire [31:0] bounded_magnitude = magnitude > 1000 ? 1000 : magnitude;
    wire [31:0] on_cycles = (bounded_magnitude * PERIOD_CYCLES) / 1000;
    // Include live requests in the output gate: STOP or a direction change
    // cannot leave the former drive state asserted until the next PID sample.
    wire drive_allowed = reset_n && enable && drive_active &&
                         blank_remaining == 0 && requested_nonzero &&
                         requested_direction == active_direction;
    assign pwm = !drive_allowed || pwm_counter < on_cycles;
    assign in1 = drive_allowed && active_direction;
    assign in2 = drive_allowed && !active_direction;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            pwm_counter <= 0;
            blank_remaining <= 0;
            active_direction <= 0;
            direction_valid <= 0;
            drive_active <= 0;
        end else if (!enable) begin
            pwm_counter <= 0;
            blank_remaining <= 0;
            active_direction <= 0;
            direction_valid <= 0;
            drive_active <= 0;
        end else begin
            if (pwm_counter == PERIOD_CYCLES - 1) pwm_counter <= 0;
            else pwm_counter <= pwm_counter + 1'b1;
            if (blank_remaining != 0) begin
                blank_remaining <= blank_remaining - 1'b1;
                drive_active <= 0;
                if (blank_remaining == 1) begin
                    active_direction <= requested_direction;
                    direction_valid <= requested_nonzero;
                    drive_active <= requested_nonzero;
                    pwm_counter <= 0;
                end
            end else if (!requested_nonzero) begin
                drive_active <= 0;
            end else if (direction_valid && active_direction != requested_direction) begin
                blank_remaining <= BLANK_CYCLES;
                drive_active <= 0;
            end else begin
                active_direction <= requested_direction;
                direction_valid <= 1;
                drive_active <= 1;
            end
        end
    end
endmodule
