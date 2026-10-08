// One wheel, all logic in clk domain. Encoder counts are FOUR-EDGE counts.
// Inner loop: 10 ms, speed unit counts/10 ms, output signed PWM permille.
// Optional outer loop: 50 ms, absolute position in encoder counts, speed ceiling
// abs(target_speed). Gains are unsigned Q8 (integer 256 represents 1.0).
// Ki/Kd include the fixed sample period; do NOT copy M0 gains without rescaling.
// Speed IIR and both measurement derivative IIRs use alpha=1/4, with Q8 state.
// Conditional integration, bounded outputs, target/PWM slew, stop reset.
// Arithmetic is staged across clk cycles, never across control sample periods.
module motor_axis_control #(
    parameter integer CLK_HZ = 50000000,
    parameter integer ENCODER_FILTER_CYCLES = 100,
    parameter integer MAX_SPEED_COUNTS = 1000,
    parameter integer TARGET_SLEW_COUNTS = 1,
    parameter integer PWM_SLEW_PERMILLE = 15,
    parameter integer STARTUP_GRACE_SAMPLES = 100,
    parameter integer STALL_SAMPLES = 100,
    parameter integer REVERSE_SAMPLES = 20,
    parameter integer MOTION_DUTY_THRESHOLD = 100,
    parameter integer MOTION_SPEED_THRESHOLD = 10,
    parameter integer INVALID_PER_SAMPLE = 8
) (
    input wire clk,
    input wire reset_n,
    input wire tick_10ms,
    input wire tick_50ms,
    input wire safety_stop,
    input wire run_enable,
    input wire clear_fault,
    input wire zero_position,
    input wire encoder_invert,
    input wire mode_position,
    input wire encoder_a,
    input wire encoder_b,
    input wire signed [15:0] target_speed,
    input wire signed [31:0] target_position,
    input wire [15:0] speed_kp,
    input wire [15:0] speed_ki,
    input wire [15:0] speed_kd,
    input wire [15:0] position_kp,
    input wire [15:0] position_ki,
    input wire [15:0] position_kd,
    input wire [15:0] speed_deadband,
    input wire [15:0] position_deadband,
    input wire [9:0] max_duty_permille,
    input wire [9:0] min_duty_permille,
    output wire pwm,
    output wire in1,
    output wire in2,
    output wire signed [31:0] position,
    output reg signed [31:0] speed,
    output reg signed [31:0] speed_raw,
    output reg signed [15:0] target_effective,
    output reg signed [15:0] duty,
    output reg [7:0] fault,
    output wire [15:0] invalid
);
    // Fault bits: 0=stall heuristic, 1=opposite feedback heuristic,
    // 2=invalid quadrature transitions, 3=position overflow,
    // 4=raw speed beyond design limit. Not collision/current/battery protection.
    wire encoder_overflow;
    reg applied_invert;
    // Direction edits are accepted only while disarmed. They also establish a
    // new zero and filter baseline, so prior coordinate history is not reused.
    wire safe_zero = !run_enable && (zero_position || encoder_invert != applied_invert);
    wire enabled = run_enable && !safety_stop && fault == 0;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) applied_invert <= 0;
        else if (!run_enable) applied_invert <= encoder_invert;
    end
    encoder_quadrature #(.FILTER_CYCLES(ENCODER_FILTER_CYCLES)) encoder (
        .clk(clk), .reset_n(reset_n), .encoder_a(encoder_a),
        .encoder_b(encoder_b), .invert(applied_invert),
        .zero_position(safe_zero), .position(position),
        .invalid_count(invalid), .overflow(encoder_overflow)
    );
    tb6612_axis_pwm #(.CLK_HZ(CLK_HZ)) bridge (
        .clk(clk), .reset_n(reset_n), .enable(enabled),
        .duty_permille(duty), .pwm(pwm), .in1(in1), .in2(in2)
    );

    function signed [63:0] extend32;
        input signed [31:0] value;
        begin extend32 = {{32{value[31]}}, value}; end
    endfunction
    function signed [63:0] extend16;
        input signed [15:0] value;
        begin extend16 = {{48{value[15]}}, value}; end
    endfunction
    function signed [31:0] saturate32;
        input signed [63:0] value;
        begin
            if (value > 64'sd2147483647) saturate32 = 32'sh7fffffff;
            else if (value < -64'sd2147483648) saturate32 = 32'sh80000000;
            else saturate32 = value[31:0];
        end
    endfunction
    function signed [63:0] abs64;
        input signed [63:0] value;
        begin abs64 = value < 0 ? -value : value; end
    endfunction
    function signed [63:0] clamp64;
        input signed [63:0] value;
        input signed [63:0] limit;
        begin
            if (value > limit) clamp64 = limit;
            else if (value < -limit) clamp64 = -limit;
            else clamp64 = value;
        end
    endfunction
    function signed [63:0] multiply_gain;
        input signed [31:0] value;
        input [15:0] gain;
        begin multiply_gain = $signed(value) * $signed({1'b0, gain}); end
    endfunction
    function signed [31:0] round_q8;
        input signed [63:0] value;
        reg signed [63:0] rounded;
        begin
            // Symmetric round-to-nearest; negative values do not acquire a
            // permanent -1 bias merely from an arithmetic right shift.
            if (value < 0) rounded = -((-value + 64'sd128) >>> 8);
            else rounded = (value + 64'sd128) >>> 8;
            round_q8 = saturate32(rounded);
        end
    endfunction

    // Sampling pipeline stays active while disarmed, so hand-turning the wheel
    // is visible. Zero clears count/filter history together while disarmed.
    reg [2:0] sample_state;
    reg sample_ready;
    reg sampled_outer_tick;
    reg signed [31:0] previous_position;
    reg signed [31:0] sampled_position;
    reg signed [31:0] previous_speed;
    reg [15:0] sampled_invalid;
    reg signed [63:0] speed_filter_q8;
    reg signed [63:0] derivative_filter_q8;
    wire signed [63:0] delta_position = extend32(position) - extend32(previous_position);
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            sample_state <= 0;
            sample_ready <= 0;
            sampled_outer_tick <= 0;
            previous_position <= 0;
            sampled_position <= 0;
            previous_speed <= 0;
            sampled_invalid <= 0;
            speed <= 0;
            speed_raw <= 0;
            speed_filter_q8 <= 0;
            derivative_filter_q8 <= 0;
        end else if (safe_zero) begin
            sample_state <= 0;
            sample_ready <= 0;
            sampled_outer_tick <= 0;
            previous_position <= 0;
            sampled_position <= 0;
            previous_speed <= 0;
            sampled_invalid <= 0;
            speed <= 0;
            speed_raw <= 0;
            speed_filter_q8 <= 0;
            derivative_filter_q8 <= 0;
        end else begin
            sample_ready <= 0;
            case (sample_state)
                0: if (tick_10ms) begin
                    speed_raw <= saturate32(delta_position);
                    previous_position <= position;
                    sampled_position <= position;
                    sampled_invalid <= invalid;
                    sampled_outer_tick <= tick_50ms;
                    sample_state <= 1;
                end
                1: begin
                    speed_filter_q8 <= speed_filter_q8 +
                        (((extend32(speed_raw) <<< 8) - speed_filter_q8) >>> 2);
                    sample_state <= 2;
                end
                2: begin
                    speed <= round_q8(speed_filter_q8);
                    sample_state <= 3;
                end
                3: begin
                    derivative_filter_q8 <= derivative_filter_q8 +
                        ((((extend32(speed) - extend32(previous_speed)) <<< 8) -
                           derivative_filter_q8) >>> 2);
                    previous_speed <= speed;
                    sample_ready <= 1;
                    sample_state <= 0;
                end
                default: sample_state <= 0;
            endcase
        end
    end

    localparam [3:0] IDLE = 0, POS_PREP = 1, POS_MUL = 2,
        POS_INTEGRATE = 3, POS_APPLY = 4, SPEED_PREP = 5,
        SPEED_ERROR = 6, SPEED_MUL = 7, SPEED_INTEGRATE = 8,
        SPEED_APPLY = 9, CHECK_FAULTS = 10, POS_FILTER = 11, POS_DERIVATIVE = 12;
    reg [3:0] control_state;
    reg signed [31:0] outer_previous_position;
    reg signed [15:0] outer_speed;
    reg signed [15:0] desired_speed;
    reg signed [31:0] sampled_target_position;
    reg [15:0] speed_kp_sample, speed_ki_sample, speed_kd_sample;
    reg [15:0] position_kp_sample, position_ki_sample, position_kd_sample;
    reg [15:0] speed_deadband_sample, position_deadband_sample;
    reg signed [63:0] speed_limit;
    reg signed [63:0] duty_limit;
    reg signed [63:0] minimum_duty;
    reg signed [63:0] position_integral_q8;
    reg signed [63:0] position_derivative_q8;
    reg signed [63:0] speed_integral_q8;
    reg signed [31:0] error;
    reg signed [31:0] measurement_delta;
    reg signed [63:0] p_term;
    reg signed [63:0] i_step;
    reg signed [63:0] d_term;
    reg signed [63:0] integral_candidate;
    reg signed [63:0] raw_command;
    reg [15:0] previous_invalid;
    reg [15:0] startup_samples;
    reg [15:0] stall_samples;
    reg [15:0] reverse_samples;
    reg last_target_sign;
    reg target_sign_valid;
    // Blocking temporaries are local arithmetic within a stage, not state.
    reg signed [63:0] temporary;
    reg signed [63:0] selected_integral;
    reg signed [63:0] selected_command;
    reg signed [63:0] rounded_command;
    reg signed [63:0] slew_command;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            control_state <= IDLE;
            outer_previous_position <= 0;
            outer_speed <= 0;
            desired_speed <= 0;
            sampled_target_position <= 0;
            speed_kp_sample <= 0;
            speed_ki_sample <= 0;
            speed_kd_sample <= 0;
            position_kp_sample <= 0;
            position_ki_sample <= 0;
            position_kd_sample <= 0;
            speed_deadband_sample <= 0;
            position_deadband_sample <= 0;
            speed_limit <= 0;
            duty_limit <= 0;
            minimum_duty <= 0;
            position_integral_q8 <= 0;
            position_derivative_q8 <= 0;
            speed_integral_q8 <= 0;
            error <= 0;
            measurement_delta <= 0;
            p_term <= 0;
            i_step <= 0;
            d_term <= 0;
            integral_candidate <= 0;
            raw_command <= 0;
            previous_invalid <= 0;
            startup_samples <= 0;
            stall_samples <= 0;
            reverse_samples <= 0;
            last_target_sign <= 0;
            target_sign_valid <= 0;
            target_effective <= 0;
            duty <= 0;
            fault <= 0;
            temporary = 0;
            selected_integral = 0;
            selected_command = 0;
            rounded_command = 0;
            slew_command = 0;
        end else begin
            if (clear_fault && !run_enable) fault <= 0;
            if (!enabled) begin
                control_state <= IDLE;
                outer_previous_position <= safe_zero ? 0 : position;
                outer_speed <= 0;
                desired_speed <= 0;
                position_integral_q8 <= 0;
                position_derivative_q8 <= 0;
                speed_integral_q8 <= 0;
                target_effective <= 0;
                duty <= 0;
                previous_invalid <= safe_zero ? 0 : invalid;
                startup_samples <= 0;
                stall_samples <= 0;
                reverse_samples <= 0;
                target_sign_valid <= 0;
            end else begin
                case (control_state)
                    IDLE: if (sample_ready) begin
                        // Snapshot one complete parameter set at sample start.
                        sampled_target_position <= target_position;
                        speed_kp_sample <= speed_kp;
                        speed_ki_sample <= speed_ki;
                        speed_kd_sample <= speed_kd;
                        position_kp_sample <= position_kp;
                        position_ki_sample <= position_ki;
                        position_kd_sample <= position_kd;
                        speed_deadband_sample <= speed_deadband;
                        position_deadband_sample <= position_deadband;
                        temporary = abs64(extend16(target_speed));
                        if (temporary > MAX_SPEED_COUNTS) temporary = MAX_SPEED_COUNTS;
                        speed_limit <= temporary;
                        temporary = max_duty_permille;
                        if (temporary > 900) temporary = 900;
                        duty_limit <= temporary;
                        minimum_duty <= min_duty_permille;
                        if (mode_position && sampled_outer_tick)
                            control_state <= POS_PREP;
                        else begin
                            if (mode_position) begin
                                // Honor a lowered outer speed ceiling immediately,
                                // even between the 50 ms outer-loop updates.
                                temporary = abs64(extend16(target_speed));
                                if (temporary > MAX_SPEED_COUNTS) temporary = MAX_SPEED_COUNTS;
                                temporary = clamp64(extend16(outer_speed), temporary);
                                desired_speed <= temporary[15:0];
                            end
                            else begin
                                temporary = clamp64(extend16(target_speed), MAX_SPEED_COUNTS);
                                desired_speed <= temporary[15:0];
                                position_integral_q8 <= 0;
                                position_derivative_q8 <= 0;
                                outer_previous_position <= sampled_position;
                            end
                            control_state <= SPEED_PREP;
                        end
                    end
                    POS_PREP: begin
                        temporary = extend32(sampled_target_position) - extend32(sampled_position);
                        if (abs64(temporary) <= position_deadband_sample) begin
                            desired_speed <= 0;
                            outer_speed <= 0;
                            position_integral_q8 <= 0;
                            position_derivative_q8 <= 0;
                            outer_previous_position <= sampled_position;
                            control_state <= SPEED_PREP;
                        end else begin
                            error <= saturate32(temporary);
                            measurement_delta <= saturate32(extend32(sampled_position) -
                                                             extend32(outer_previous_position));
                            outer_previous_position <= sampled_position;
                            control_state <= POS_FILTER;
                        end
                    end
                    POS_FILTER: begin
                        position_derivative_q8 <= position_derivative_q8 +
                            (((extend32(measurement_delta) <<< 8) - position_derivative_q8) >>> 2);
                        control_state <= POS_DERIVATIVE;
                    end
                    POS_DERIVATIVE: begin
                        measurement_delta <= round_q8(position_derivative_q8);
                        control_state <= POS_MUL;
                    end
                    POS_MUL: begin
                        p_term <= multiply_gain(error, position_kp_sample);
                        i_step <= multiply_gain(error, position_ki_sample);
                        d_term <= -multiply_gain(measurement_delta, position_kd_sample);
                        control_state <= POS_INTEGRATE;
                    end
                    POS_INTEGRATE: begin
                        temporary = clamp64(position_integral_q8 + i_step, speed_limit <<< 8);
                        integral_candidate <= temporary;
                        raw_command <= p_term + temporary + d_term;
                        control_state <= POS_APPLY;
                    end
                    POS_APPLY: begin
                        selected_integral = integral_candidate;
                        selected_command = raw_command;
                        if ((raw_command > (speed_limit <<< 8) && error > 0) ||
                            (raw_command < -(speed_limit <<< 8) && error < 0)) begin
                            selected_integral = position_integral_q8;
                            selected_command = p_term + position_integral_q8 + d_term;
                        end
                        position_integral_q8 <= selected_integral;
                        temporary = round_q8(clamp64(selected_command, speed_limit <<< 8));
                        outer_speed <= temporary[15:0];
                        desired_speed <= temporary[15:0];
                        control_state <= SPEED_PREP;
                    end
                    SPEED_PREP: begin
                        // Zero target is an explicit stop, never a stiction command.
                        if (desired_speed == 0) begin
                            target_effective <= 0;
                            speed_integral_q8 <= 0;
                            duty <= 0;
                            control_state <= CHECK_FAULTS;
                        end else begin
                            temporary = extend16(desired_speed) - extend16(target_effective);
                            if (temporary > TARGET_SLEW_COUNTS)
                                temporary = extend16(target_effective) + TARGET_SLEW_COUNTS;
                            else if (temporary < -TARGET_SLEW_COUNTS)
                                temporary = extend16(target_effective) - TARGET_SLEW_COUNTS;
                            else temporary = extend16(desired_speed);
                            target_effective <= temporary[15:0];
                            control_state <= SPEED_ERROR;
                        end
                    end
                    SPEED_ERROR: begin
                        if (target_effective == 0) begin
                            speed_integral_q8 <= 0;
                            duty <= 0;
                            control_state <= CHECK_FAULTS;
                        end else begin
                            temporary = extend16(target_effective) - extend32(speed);
                            if (abs64(temporary) <= speed_deadband_sample) temporary = 0;
                            error <= saturate32(temporary);
                            measurement_delta <= round_q8(derivative_filter_q8);
                            control_state <= SPEED_MUL;
                        end
                    end
                    SPEED_MUL: begin
                        p_term <= multiply_gain(error, speed_kp_sample);
                        i_step <= multiply_gain(error, speed_ki_sample);
                        d_term <= -multiply_gain(measurement_delta, speed_kd_sample);
                        control_state <= SPEED_INTEGRATE;
                    end
                    SPEED_INTEGRATE: begin
                        temporary = clamp64(speed_integral_q8 + i_step, duty_limit <<< 8);
                        integral_candidate <= temporary;
                        raw_command <= p_term + temporary + d_term;
                        control_state <= SPEED_APPLY;
                    end
                    SPEED_APPLY: begin
                        selected_integral = integral_candidate;
                        selected_command = raw_command;
                        if ((raw_command > (duty_limit <<< 8) && error > 0) ||
                            (raw_command < -(duty_limit <<< 8) && error < 0)) begin
                            selected_integral = speed_integral_q8;
                            selected_command = p_term + speed_integral_q8 + d_term;
                        end
                        speed_integral_q8 <= selected_integral;
                        rounded_command = round_q8(clamp64(selected_command, duty_limit <<< 8));
                        // Optional minimum MOVING duty floor. It is not applied
                        // at zero command and is always bounded by max duty.
                        temporary = minimum_duty > duty_limit ? duty_limit : minimum_duty;
                        if (rounded_command > 0 && rounded_command < temporary)
                            rounded_command = temporary;
                        else if (rounded_command < 0 && rounded_command > -temporary)
                            rounded_command = -temporary;
                        // Never jump directly from positive PWM to negative PWM.
                        if ((duty > 0 && rounded_command < 0) ||
                            (duty < 0 && rounded_command > 0)) begin
                            duty <= 0;
                        end else if (rounded_command == 0) begin
                            duty <= 0;
                        end else begin
                            slew_command = clamp64(rounded_command - extend16(duty), PWM_SLEW_PERMILLE);
                            temporary = clamp64(extend16(duty) + slew_command, duty_limit);
                            duty <= temporary[15:0];
                        end
                        control_state <= CHECK_FAULTS;
                    end
                    CHECK_FAULTS: begin
                        previous_invalid <= sampled_invalid;
                        if (encoder_overflow) fault[3] <= 1;
                        if (abs64(extend32(speed_raw)) > MAX_SPEED_COUNTS) fault[4] <= 1;
                        if (sampled_invalid == 16'hffff ||
                            (sampled_invalid - previous_invalid) >= INVALID_PER_SAMPLE)
                            fault[2] <= 1;
                        // Reset startup grace on every nonzero target sign change.
                        if (target_effective == 0) begin
                            target_sign_valid <= 0;
                            startup_samples <= 0;
                            stall_samples <= 0;
                            reverse_samples <= 0;
                        end else if (!target_sign_valid || last_target_sign != target_effective[15]) begin
                            target_sign_valid <= 1;
                            last_target_sign <= target_effective[15];
                            startup_samples <= 0;
                            stall_samples <= 0;
                            reverse_samples <= 0;
                        end else if (startup_samples < STARTUP_GRACE_SAMPLES) begin
                            startup_samples <= startup_samples + 1'b1;
                            stall_samples <= 0;
                            reverse_samples <= 0;
                        end else begin
                            if (abs64(extend16(duty)) >= MOTION_DUTY_THRESHOLD &&
                                abs64(extend16(target_effective)) >= MOTION_SPEED_THRESHOLD &&
                                abs64(extend32(speed_raw)) <= 1) begin
                                if (stall_samples >= STALL_SAMPLES - 1) fault[0] <= 1;
                                else stall_samples <= stall_samples + 1'b1;
                            end else stall_samples <= 0;
                            if (abs64(extend16(duty)) >= MOTION_DUTY_THRESHOLD &&
                                abs64(extend16(target_effective)) >= MOTION_SPEED_THRESHOLD &&
                                abs64(extend32(speed_raw)) >= MOTION_SPEED_THRESHOLD &&
                                speed_raw[31] != target_effective[15]) begin
                                if (reverse_samples >= REVERSE_SAMPLES - 1) fault[1] <= 1;
                                else reverse_samples <= reverse_samples + 1'b1;
                            end else reverse_samples <= 0;
                        end
                        control_state <= IDLE;
                    end
                    default: control_state <= IDLE;
                endcase
            end
        end
    end
endmodule
