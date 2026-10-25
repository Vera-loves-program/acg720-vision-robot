// Two independent wheel controllers, one shared synchronous sample timebase.
// Any latched wheel fault immediately gates BOTH bridges to coast STOP.
// Fabricated board has STBY tied high: this is an FPGA command gate, not
// a power-disconnecting hardware estop, and does not protect during configuration.
module motor_drive_control #(
    parameter integer CLK_HZ = 50000000,
    parameter integer ENCODER_FILTER_CYCLES = 100
) (
    input wire clk,
    input wire reset_n,
    input wire safety_stop,
    input wire run_enable,
    input wire clear_fault,
    input wire zero_position,
    input wire encoder_invert_left,
    input wire encoder_invert_right,
    input wire drive_invert_left,
    input wire drive_invert_right,
    input wire calibration_enable,
    input wire signed [15:0] calibration_duty_left,
    input wire signed [15:0] calibration_duty_right,
    input wire mode_position,
    input wire encoder1_a,
    input wire encoder1_b,
    input wire encoder2_a,
    input wire encoder2_b,
    input wire signed [15:0] target_speed_left,
    input wire signed [15:0] target_speed_right,
    input wire signed [31:0] target_position_left,
    input wire signed [31:0] target_position_right,
    input wire [15:0] speed_kp_left,
    input wire [15:0] speed_ki_left,
    input wire [15:0] speed_kd_left,
    input wire [15:0] speed_kp_right,
    input wire [15:0] speed_ki_right,
    input wire [15:0] speed_kd_right,
    input wire [15:0] position_kp_left,
    input wire [15:0] position_ki_left,
    input wire [15:0] position_kd_left,
    input wire [15:0] position_kp_right,
    input wire [15:0] position_ki_right,
    input wire [15:0] position_kd_right,
    input wire [15:0] speed_deadband,
    input wire [15:0] position_deadband,
    input wire [9:0] max_duty_permille,
    input wire [9:0] min_duty_permille,
    output wire pwma,
    output wire ain1,
    output wire ain2,
    output wire pwmb,
    output wire bin1,
    output wire bin2,
    output wire signed [31:0] position_left,
    output wire signed [31:0] position_right,
    output wire signed [31:0] speed_left,
    output wire signed [31:0] speed_right,
    output wire signed [31:0] speed_raw_left,
    output wire signed [31:0] speed_raw_right,
    output wire signed [15:0] target_effective_left,
    output wire signed [15:0] target_effective_right,
    output wire signed [15:0] duty_left,
    output wire signed [15:0] duty_right,
    output wire [7:0] fault_left,
    output wire [7:0] fault_right,
    output wire [15:0] invalid_left,
    output wire [15:0] invalid_right,
    output reg tick_10ms
);
    localparam integer SAMPLE_CYCLES = CLK_HZ / 100;
    localparam integer SAMPLE_COUNTER_BITS=SAMPLE_CYCLES>0 ?
        (SAMPLE_CYCLES>1 ? $clog2(SAMPLE_CYCLES) : 1) : 32;
    reg [SAMPLE_COUNTER_BITS-1:0] sample_counter;
    reg [2:0] outer_counter;
    reg tick_50ms;
    wire combined_stop = safety_stop || fault_left != 0 || fault_right != 0;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            sample_counter <= 0;
            outer_counter <= 0;
            tick_10ms <= 0;
            tick_50ms <= 0;
        end else begin
            tick_10ms <= 0;
            tick_50ms <= 0;
            if (sample_counter == SAMPLE_CYCLES - 1) begin
                sample_counter <= 0;
                tick_10ms <= 1;
                if (outer_counter == 4) begin
                    outer_counter <= 0;
                    tick_50ms <= 1;
                end else outer_counter <= outer_counter + 1'b1;
            end else sample_counter <= sample_counter + 1'b1;
        end
    end

    motor_axis_control #(.CLK_HZ(CLK_HZ),
        .ENCODER_FILTER_CYCLES(ENCODER_FILTER_CYCLES)) left_axis (
        .clk(clk), .reset_n(reset_n), .tick_10ms(tick_10ms), .tick_50ms(tick_50ms),
        .safety_stop(combined_stop), .run_enable(run_enable), .clear_fault(clear_fault),
        .zero_position(zero_position), .encoder_invert(encoder_invert_left),
        .drive_invert(drive_invert_left),.calibration_enable(calibration_enable),
        .calibration_duty(calibration_duty_left),
        .mode_position(mode_position), .encoder_a(encoder1_a), .encoder_b(encoder1_b),
        .target_speed(target_speed_left), .target_position(target_position_left),
        .speed_kp(speed_kp_left), .speed_ki(speed_ki_left), .speed_kd(speed_kd_left),
        .position_kp(position_kp_left), .position_ki(position_ki_left), .position_kd(position_kd_left),
        .speed_deadband(speed_deadband), .position_deadband(position_deadband),
        .max_duty_permille(max_duty_permille), .min_duty_permille(min_duty_permille),
        .pwm(pwma), .in1(ain1), .in2(ain2), .position(position_left),
        .speed(speed_left), .speed_raw(speed_raw_left), .target_effective(target_effective_left),
        .duty(duty_left), .fault(fault_left), .invalid(invalid_left)
    );
    motor_axis_control #(.CLK_HZ(CLK_HZ),
        .ENCODER_FILTER_CYCLES(ENCODER_FILTER_CYCLES)) right_axis (
        .clk(clk), .reset_n(reset_n), .tick_10ms(tick_10ms), .tick_50ms(tick_50ms),
        .safety_stop(combined_stop), .run_enable(run_enable), .clear_fault(clear_fault),
        .zero_position(zero_position), .encoder_invert(encoder_invert_right),
        .drive_invert(drive_invert_right),.calibration_enable(calibration_enable),
        .calibration_duty(calibration_duty_right),
        .mode_position(mode_position), .encoder_a(encoder2_a), .encoder_b(encoder2_b),
        .target_speed(target_speed_right), .target_position(target_position_right),
        .speed_kp(speed_kp_right), .speed_ki(speed_ki_right), .speed_kd(speed_kd_right),
        .position_kp(position_kp_right), .position_ki(position_ki_right), .position_kd(position_kd_right),
        .speed_deadband(speed_deadband), .position_deadband(position_deadband),
        .max_duty_permille(max_duty_permille), .min_duty_permille(min_duty_permille),
        .pwm(pwmb), .in1(bin1), .in2(bin2), .position(position_right),
        .speed(speed_right), .speed_raw(speed_raw_right), .target_effective(target_effective_right),
        .duty(duty_right), .fault(fault_right), .invalid(invalid_right)
    );
endmodule
