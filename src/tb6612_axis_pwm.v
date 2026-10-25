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
    input wire drive_invert,
    input wire signed [15:0] duty_permille,
    output wire pwm,
    output wire in1,
    output wire in2
);
    localparam integer PERIOD_CYCLES = CLK_HZ / PWM_HZ;
    localparam integer BLANK_CYCLES = (CLK_HZ / 1000000) * REVERSE_BLANK_US;
    localparam integer PWM_COUNTER_BITS=PERIOD_CYCLES>1 ? $clog2(PERIOD_CYCLES) : 1;
    localparam integer BLANK_COUNTER_BITS=BLANK_CYCLES>0 ? $clog2(BLANK_CYCLES+1) : 1;
    reg [PWM_COUNTER_BITS-1:0] pwm_counter;
    reg [BLANK_COUNTER_BITS-1:0] blank_remaining;
    reg active_direction;
    reg direction_valid;
    reg drive_active;
    // Polarity swaps the bridge direction only; the signed16 magnitude,
    // including -32768, is unchanged. The owner changes this only stopped.
    wire requested_direction = (!duty_permille[15]) ^ drive_invert;
    wire requested_nonzero = (duty_permille != 0);
    // Unsigned magnitude deliberately represents abs(-32768)=32768.
    wire [15:0] magnitude=duty_permille[15] ? -duty_permille : duty_permille;
    wire [9:0] bounded_magnitude=magnitude>1000 ? 10'd1000 : magnitude[9:0];
    wire [PWM_COUNTER_BITS:0] on_cycles;
    generate
        if(PERIOD_CYCLES==2500)begin:gen_default_period
            // Exact floor(mag*2500/1000)=floor(5*mag/2), including odd mag.
            // No variable divide or general multiplier in the 20 kHz bridge.
            wire [12:0] five_magnitude=({3'd0,bounded_magnitude}<<2)+
                                       {3'd0,bounded_magnitude};
            assign on_cycles=five_magnitude[12:1];
        end else begin:gen_other_period
            // Preserve arbitrary legal CLK_HZ/PWM_HZ parameter combinations.
            // The product has exactly the width required by a 10-bit magnitude.
            wire [PWM_COUNTER_BITS+9:0] product=bounded_magnitude*PERIOD_CYCLES;
            assign on_cycles=product/1000;
        end
    endgenerate
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
