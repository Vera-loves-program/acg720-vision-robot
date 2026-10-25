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
    input wire drive_invert,
    input wire calibration_enable,
    input wire signed [15:0] calibration_duty,
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
    output wire signed [15:0] duty,
    output reg [7:0] fault,
    output wire [15:0] invalid
);
    // Fault bits: 0=stall heuristic, 1=opposite feedback heuristic,
    // 2=invalid quadrature transitions, 3=position overflow,
    // 4=raw speed beyond design limit. Not collision/current/battery protection.
    wire encoder_overflow;
    reg applied_invert,applied_drive_invert;
    reg signed [15:0] pid_duty;
    // Direction edits are accepted only while disarmed. They also establish a
    // new zero and filter baseline, so prior coordinate history is not reused.
    wire motion_requested = run_enable || calibration_enable;
    wire safe_zero = !motion_requested && (zero_position || encoder_invert != applied_invert ||
                                           drive_invert != applied_drive_invert);
    wire bridge_enabled = reset_n && motion_requested && !safety_stop && fault == 0;
    // Open-loop calibration has explicit priority and cancels the PID job;
    // neither uncalibrated feedback direction nor zero gains can start PID.
    wire enabled = run_enable && !calibration_enable && !safety_stop && fault == 0;
    wire [9:0] calibration_limit=max_duty_permille>200 ? 10'd200 : max_duty_permille;
    wire signed [15:0] calibration_limit_signed=$signed({6'd0,calibration_limit});
    wire signed [15:0] calibration_command=calibration_duty>calibration_limit_signed ? calibration_limit_signed :
        calibration_duty< -calibration_limit_signed ? -calibration_limit_signed : calibration_duty;
    assign duty=bridge_enabled ? (calibration_enable ? calibration_command : pid_duty) : 16'sd0;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin applied_invert<=0;applied_drive_invert<=0;end
        else if (!motion_requested) begin
            applied_invert<=encoder_invert;applied_drive_invert<=drive_invert;
        end
    end
    encoder_quadrature #(.FILTER_CYCLES(ENCODER_FILTER_CYCLES)) encoder (
        .clk(clk), .reset_n(reset_n), .encoder_a(encoder_a),
        .encoder_b(encoder_b), .invert(applied_invert),
        .zero_position(safe_zero), .position(position),
        .invalid_count(invalid), .overflow(encoder_overflow)
    );
    tb6612_axis_pwm #(.CLK_HZ(CLK_HZ)) bridge (
        .clk(clk), .reset_n(reset_n), .enable(bridge_enabled),.drive_invert(applied_drive_invert),
        .duty_permille(duty), .pwm(pwm), .in1(in1), .in2(in2)
    );

    // Width proof (all legal 32-bit positions and all unsigned 16-bit gains):
    // raw/filtered speed: signed32; speed Q8: signed40; its subtraction: 41.
    // speed derivative: signed33 -> Q8 signed41; its subtraction: signed42.
    // position derivative: saturated signed32 -> Q8 signed40.
    // error32 * gain16 fits signed48 (gain has no negative values).
    // P + D + bounded I fits signed49. Position I <=32768*256 fits signed25;
    // speed I <=900*256 fits signed19. None of these bounds assume UI gains.
    function signed [31:0] saturate33;
        input signed [32:0] value;
        begin
            if (value > 33'sd2147483647) saturate33=32'sh7fffffff;
            else if (value < -33'sd2147483648) saturate33=32'sh80000000;
            else saturate33=value[31:0];
        end
    endfunction
    function [32:0] abs33;
        input signed [32:0] value;
        begin abs33=value[32] ? -value : value; end
    endfunction
    function [31:0] abs32;
        input signed [31:0] value;
        begin abs32=value[31] ? -value : value; end
    endfunction
    function [15:0] abs16;
        input signed [15:0] value;
        begin abs16=value[15] ? -value : value; end
    endfunction
    function signed [31:0] round_filter_q8;
        input signed [41:0] value;
        reg signed [41:0] adjusted,rounded;
        begin
            // Identical to -((-value+128)>>8) for negative values, without
            // two wide negations. Ties still round away from zero.
            adjusted=value+(value[41] ? 42'sd127 : 42'sd128);
            rounded=adjusted>>>8;
            if(rounded>42'sd2147483647) round_filter_q8=32'sh7fffffff;
            else if(rounded< -42'sd2147483648) round_filter_q8=32'sh80000000;
            else round_filter_q8=rounded[31:0];
        end
    endfunction
    function signed [16:0] round_command_q8;
        input signed [24:0] value;
        reg signed [24:0] adjusted;
        begin
            adjusted=value+(value[24] ? 25'sd127 : 25'sd128);
            round_command_q8=adjusted>>>8;
        end
    endfunction
    function signed [15:0] clamp_speed;
        input signed [15:0] value;
        input [15:0] limit;
        reg signed [16:0] v,l;
        begin
            v={value[15],value}; l={1'b0,limit};
            if(v>l) clamp_speed=l[15:0];
            else if(v< -l) clamp_speed=-l;
            else clamp_speed=value;
        end
    endfunction

    // Sampling remains active while disarmed. Extra 50 MHz stages are not
    // extra 10 ms sample periods: one sample is published after eight clocks.
    reg [3:0] sample_state;
    reg sample_ready,sampled_outer_tick;
    reg signed [31:0] previous_position,sampled_position,previous_speed;
    reg [15:0] sampled_invalid;
    reg signed [39:0] speed_filter_q8,position_derivative_q8;
    reg signed [40:0] speed_filter_difference;
    reg signed [40:0] derivative_filter_q8;
    reg signed [41:0] derivative_filter_difference;
    reg signed [32:0] speed_measurement_delta;
    wire signed [32:0] delta_position=
        {position[31],position}-{previous_position[31],previous_position};
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            sample_state<=0;sample_ready<=0;sampled_outer_tick<=0;
            previous_position<=0;sampled_position<=0;previous_speed<=0;
            sampled_invalid<=0;speed<=0;speed_raw<=0;speed_filter_q8<=0;
            speed_filter_difference<=0;derivative_filter_q8<=0;
            derivative_filter_difference<=0;speed_measurement_delta<=0;
        end else if(safe_zero) begin
            sample_state<=0;sample_ready<=0;sampled_outer_tick<=0;
            previous_position<=0;sampled_position<=0;previous_speed<=0;
            sampled_invalid<=0;speed<=0;speed_raw<=0;speed_filter_q8<=0;
            speed_filter_difference<=0;derivative_filter_q8<=0;
            derivative_filter_difference<=0;speed_measurement_delta<=0;
        end else begin
            sample_ready<=0;
            case(sample_state)
                0:if(tick_10ms)begin
                    speed_raw<=saturate33(delta_position);
                    previous_position<=position;sampled_position<=position;
                    sampled_invalid<=invalid;sampled_outer_tick<=tick_50ms;
                    sample_state<=1;
                end
                1:begin
                    speed_filter_difference<=
                        $signed({speed_raw[31],speed_raw,8'd0})-
                        $signed({speed_filter_q8[39],speed_filter_q8});
                    sample_state<=2;
                end
                2:begin
                    speed_filter_q8<=$signed({speed_filter_q8[39],speed_filter_q8})+
                        (speed_filter_difference>>>2);
                    sample_state<=3;
                end
                3:begin speed<=round_filter_q8(speed_filter_q8);sample_state<=4;end
                4:begin
                    speed_measurement_delta<=
                        $signed({speed[31],speed})-$signed({previous_speed[31],previous_speed});
                    previous_speed<=speed;sample_state<=5;
                end
                5:begin
                    derivative_filter_difference<=
                        $signed({speed_measurement_delta[32],speed_measurement_delta,8'd0})-
                        $signed({derivative_filter_q8[40],derivative_filter_q8});
                    sample_state<=6;
                end
                6:begin
                    derivative_filter_q8<=$signed({derivative_filter_q8[40],derivative_filter_q8})+
                        (derivative_filter_difference>>>2);
                    sample_state<=7;
                end
                7:begin sample_ready<=1;sample_state<=0;end
                default:sample_state<=0;
            endcase
        end
    end

    // One exact signed32 x unsigned16 right-shift partial-product unit for
    // every P/I/D product in BOTH loops. The signed upper sum needs 33 bits,
    // not a 48-bit shifting addend plus accumulator and duplicate product.
    // mul_value is held by the job sequencer until mul_done. A product takes
    // 16 iterations; stop/fault cancels it immediately, including partial bits.
    reg mul_start,mul_busy,mul_done;
    reg signed [31:0] mul_value;
    reg [15:0] mul_gain,mul_gain_bits;
    reg signed [32:0] mul_partial;
    reg [3:0] mul_index;
    wire signed [32:0] mul_sum=mul_partial+
        (mul_gain_bits[0] ? $signed({mul_value[31],mul_value}) : 33'sd0);
    wire signed [47:0] mul_product={mul_partial[31:0],mul_gain_bits};
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n)begin
            mul_busy<=0;mul_done<=0;mul_gain_bits<=0;
            mul_partial<=0;mul_index<=0;
        end else if(!enabled)begin
            mul_busy<=0;mul_done<=0;mul_gain_bits<=0;
            mul_partial<=0;mul_index<=0;
        end else begin
            mul_done<=0;
            if(!mul_busy)begin
                if(mul_start)begin
                    mul_gain_bits<=mul_gain;
                    mul_partial<=0;mul_index<=0;mul_busy<=1;
                end
            end else begin
                // Together these two assignments perform an arithmetic right
                // shift of {signed33 upper sum,unsigned16 remaining bits}.
                mul_partial<={mul_sum[32],mul_sum[32:1]};
                mul_gain_bits<={mul_sum[0],mul_gain_bits[15:1]};
                if(mul_index==15)begin
                    mul_busy<=0;mul_done<=1;
                end else mul_index<=mul_index+1'b1;
            end
        end
    end

    localparam [4:0] IDLE=0,POS_PREP=1,POS_FILTER_DIFF=2,POS_FILTER_APPLY=3,
        POS_DERIVATIVE=4,SPEED_PREP=5,SPEED_ERROR=6,
        PID_P_START=7,PID_P_WAIT=8,PID_I_WAIT=9,PID_D_WAIT=10,
        PID_INTEGRATE=12,PID_COMMAND=13,PID_SELECT=14,
        PID_CLAMP=15,PID_ROUND=16,PID_APPLY=17,
        SPEED_FLOOR=18,SPEED_POLARITY=19,SPEED_SLEW=20,
        SPEED_LIMIT=21,CHECK_FAULTS=22,PID_REJECT_COMMAND=23;
    reg [4:0] control_state;
    reg pid_outer;
    reg signed [31:0] outer_previous_position,sampled_target_position;
    reg signed [15:0] outer_speed,desired_speed;
    reg [15:0] speed_kp_sample,speed_ki_sample,speed_kd_sample;
    reg [15:0] position_kp_sample,position_ki_sample,position_kd_sample;
    reg [15:0] speed_deadband_sample,position_deadband_sample,speed_limit;
    reg [9:0] duty_limit,minimum_duty;
    reg signed [24:0] position_integral_q8,integral_old,integral_candidate;
    reg signed [18:0] speed_integral_q8;
    reg signed [24:0] pid_limit_q8,command_clamped_q8;
    reg signed [40:0] position_filter_difference;
    reg signed [31:0] error,measurement_delta;
    reg signed [48:0] pd_sum,pid_value;
    reg signed [16:0] rounded_command;
    reg signed [10:0] floor_command;
    reg signed [11:0] duty_delta,slew_delta;
    reg signed [12:0] duty_after_slew;
    localparam integer STARTUP_COUNTER_BITS=STARTUP_GRACE_SAMPLES>=0 && STARTUP_GRACE_SAMPLES<65535 ?
        (STARTUP_GRACE_SAMPLES>0 ? $clog2(STARTUP_GRACE_SAMPLES+1) : 1) : 16;
    localparam integer STALL_COUNTER_BITS=STALL_SAMPLES>0 && STALL_SAMPLES<=65535 ?
        (STALL_SAMPLES>1 ? $clog2(STALL_SAMPLES) : 1) : 16;
    localparam integer REVERSE_COUNTER_BITS=REVERSE_SAMPLES>0 && REVERSE_SAMPLES<=65535 ?
        (REVERSE_SAMPLES>1 ? $clog2(REVERSE_SAMPLES) : 1) : 16;
    reg [15:0] previous_invalid;
    reg [STARTUP_COUNTER_BITS-1:0] startup_samples;
    reg [STALL_COUNTER_BITS-1:0] stall_samples;
    reg [REVERSE_COUNTER_BITS-1:0] reverse_samples;
    reg last_target_sign,target_sign_valid;
    wire [15:0] target_magnitude=abs16(target_speed);
    localparam [15:0] MAX_TARGET_MAGNITUDE=
        MAX_SPEED_COUNTS>32768 ? 16'd32768 : MAX_SPEED_COUNTS;
    wire [15:0] bounded_target_magnitude=
        target_magnitude>MAX_SPEED_COUNTS ? MAX_SPEED_COUNTS : target_magnitude;
    wire signed [32:0] position_error_full=
        $signed({sampled_target_position[31],sampled_target_position})-
        $signed({sampled_position[31],sampled_position});
    wire signed [32:0] position_delta_full=
        $signed({sampled_position[31],sampled_position})-
        $signed({outer_previous_position[31],outer_previous_position});
    wire signed [16:0] target_difference=
        $signed({desired_speed[15],desired_speed})-
        $signed({target_effective[15],target_effective});
    wire signed [32:0] speed_error_full=
        $signed({{17{target_effective[15]}},target_effective})-
        $signed({speed[31],speed});
    wire [31:0] invalid_delta={16'd0,sampled_invalid}-{16'd0,previous_invalid};
    // P/I/D and conditional integration use ONE 49-bit add/subtract datapath.
    // The appended low bit supplies carry-in to that same addition for a-b:
    // floor(((a<<1)|1)+((~b<<1)|1))/2 == a+~b+1 (modulo 2^49).
    reg signed [48:0] pid_operand_a,pid_operand_b;
    reg pid_subtract;
    wire [49:0] pid_alu_full={pid_operand_a,1'b1}+
        {pid_operand_b^{49{pid_subtract}},pid_subtract};
    wire signed [48:0] pid_alu_result=pid_alu_full[49:1];
    always @(*) begin
        pid_operand_a=0;pid_operand_b=0;pid_subtract=0;
        case(control_state)
            PID_I_WAIT:begin
                pid_operand_a={{24{integral_old[24]}},integral_old};
                pid_operand_b={mul_product[47],mul_product};
            end
            PID_D_WAIT:begin
                pid_operand_a=pd_sum;pid_operand_b={mul_product[47],mul_product};pid_subtract=1;
            end
            PID_COMMAND:begin
                pid_operand_a=pd_sum;
                pid_operand_b={{24{integral_candidate[24]}},integral_candidate};
            end
            default:begin end
        endcase
    end
    // Both loops have nonnegative limits <=32768*256, which fit signed25.
    // Values outside signed25 are classified by their full signed49 sign;
    // only values that are an exact sign extension enter the 25-bit compares.
    // This retains every wide input bit without duplicating 49-bit comparators.
    wire pid_fits25=pid_value[48:24]=={25{pid_value[24]}};
    wire signed [24:0] pid_negative_limit=-pid_limit_q8;
    wire pid_above_limit=(!pid_fits25 && !pid_value[48]) ||
        (pid_fits25 && $signed(pid_value[24:0])>pid_limit_q8);
    wire pid_below_limit=(!pid_fits25 && pid_value[48]) ||
        (pid_fits25 && $signed(pid_value[24:0])<pid_negative_limit);
    wire signed [24:0] pid_clamped_q8=pid_above_limit ? pid_limit_q8 :
        pid_below_limit ? pid_negative_limit : $signed(pid_value[24:0]);
    // Blocking values are narrow, within one arithmetic stage, never state.
    reg signed [16:0] target_temporary;
    reg signed [10:0] floor_temporary,minimum_effective;

    // Worst case: eight sampling clocks + <=150 control clocks, including six
    // 16-iteration products (outer+inner). At 50 MHz this is <3.2 us, compared
    // with a 10000 us inner period. No arithmetic waits for another sample.
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n)begin
            control_state<=IDLE;pid_outer<=0;outer_previous_position<=0;
            outer_speed<=0;desired_speed<=0;sampled_target_position<=0;
            speed_kp_sample<=0;speed_ki_sample<=0;speed_kd_sample<=0;
            position_kp_sample<=0;position_ki_sample<=0;position_kd_sample<=0;
            speed_deadband_sample<=0;position_deadband_sample<=0;speed_limit<=0;
            duty_limit<=0;minimum_duty<=0;position_integral_q8<=0;
            position_derivative_q8<=0;speed_integral_q8<=0;position_filter_difference<=0;
            error<=0;measurement_delta<=0;
            integral_old<=0;integral_candidate<=0;
            pid_limit_q8<=0;command_clamped_q8<=0;pd_sum<=0;pid_value<=0;
            rounded_command<=0;floor_command<=0;duty_delta<=0;slew_delta<=0;duty_after_slew=0;
            previous_invalid<=0;startup_samples<=0;stall_samples<=0;reverse_samples<=0;
            last_target_sign<=0;target_sign_valid<=0;target_effective<=0;pid_duty<=0;fault<=0;
            mul_start<=0;mul_value<=0;mul_gain<=0;
            target_temporary=0;floor_temporary=0;minimum_effective=0;
        end else begin
            mul_start<=0;
            if(clear_fault && !motion_requested)fault<=0;
            if(!enabled)begin
                control_state<=IDLE;outer_previous_position<=safe_zero ? 0 : position;
                outer_speed<=0;desired_speed<=0;position_integral_q8<=0;
                position_derivative_q8<=0;speed_integral_q8<=0;
                target_effective<=0;pid_duty<=0;
                // Match the sampled counter, not a newer live value: enabling
                // between sample capture and sample_ready must not subtract
                // an older sample from a newer baseline and look like wrap.
                if(!bridge_enabled)previous_invalid<=safe_zero ? 0 : sampled_invalid;
                startup_samples<=0;stall_samples<=0;reverse_samples<=0;target_sign_valid<=0;
                mul_start<=0;
            end else case(control_state)
                IDLE:if(sample_ready)begin
                    sampled_target_position<=target_position;
                    speed_kp_sample<=speed_kp;speed_ki_sample<=speed_ki;speed_kd_sample<=speed_kd;
                    position_kp_sample<=position_kp;position_ki_sample<=position_ki;position_kd_sample<=position_kd;
                    speed_deadband_sample<=speed_deadband;position_deadband_sample<=position_deadband;
                    speed_limit<=bounded_target_magnitude;
                    duty_limit<=max_duty_permille>900 ? 10'd900 : max_duty_permille;
                    minimum_duty<=min_duty_permille;
                    if(mode_position && sampled_outer_tick)control_state<=POS_PREP;
                    else begin
                        if(mode_position)desired_speed<=clamp_speed(outer_speed,bounded_target_magnitude);
                        else begin
                            desired_speed<=clamp_speed(target_speed,MAX_TARGET_MAGNITUDE);
                            position_integral_q8<=0;position_derivative_q8<=0;
                            outer_previous_position<=sampled_position;
                        end
                        control_state<=SPEED_PREP;
                    end
                end
                POS_PREP:begin
                    if(abs33(position_error_full)<=position_deadband_sample)begin
                        desired_speed<=0;outer_speed<=0;position_integral_q8<=0;
                        position_derivative_q8<=0;outer_previous_position<=sampled_position;
                        control_state<=SPEED_PREP;
                    end else begin
                        error<=saturate33(position_error_full);
                        measurement_delta<=saturate33(position_delta_full);
                        outer_previous_position<=sampled_position;control_state<=POS_FILTER_DIFF;
                    end
                end
                POS_FILTER_DIFF:begin
                    position_filter_difference<=
                        $signed({measurement_delta[31],measurement_delta,8'd0})-
                        $signed({position_derivative_q8[39],position_derivative_q8});
                    control_state<=POS_FILTER_APPLY;
                end
                POS_FILTER_APPLY:begin
                    position_derivative_q8<=$signed({position_derivative_q8[39],position_derivative_q8})+
                        (position_filter_difference>>>2);
                    control_state<=POS_DERIVATIVE;
                end
                POS_DERIVATIVE:begin
                    measurement_delta<=round_filter_q8(position_derivative_q8);
                    pid_outer<=1;control_state<=PID_P_START;
                end
                SPEED_PREP:begin
                    if(desired_speed==0)begin
                        target_effective<=0;speed_integral_q8<=0;pid_duty<=0;control_state<=CHECK_FAULTS;
                    end else begin
                        if(target_difference>TARGET_SLEW_COUNTS)
                            target_temporary=$signed({target_effective[15],target_effective})+TARGET_SLEW_COUNTS;
                        else if(target_difference< -TARGET_SLEW_COUNTS)
                            target_temporary=$signed({target_effective[15],target_effective})-TARGET_SLEW_COUNTS;
                        else target_temporary={desired_speed[15],desired_speed};
                        target_effective<=target_temporary[15:0];control_state<=SPEED_ERROR;
                    end
                end
                SPEED_ERROR:begin
                    if(target_effective==0)begin
                        speed_integral_q8<=0;pid_duty<=0;control_state<=CHECK_FAULTS;
                    end else begin
                        error<=abs33(speed_error_full)<=speed_deadband_sample ? 32'sd0 : saturate33(speed_error_full);
                        measurement_delta<=round_filter_q8(derivative_filter_q8);
                        pid_outer<=0;control_state<=PID_P_START;
                    end
                end
                PID_P_START:begin
                    // Ki=0 explicitly disables and clears the weighted I term.
                    // Live tuning must not keep an old integral bias when the
                    // operator switches back to a P-only experiment.
                    integral_old<=pid_outer ? (position_ki_sample==0 ? 25'sd0 : position_integral_q8) :
                        (speed_ki_sample==0 ? 25'sd0 : {{6{speed_integral_q8[18]}},speed_integral_q8});
                    pid_limit_q8<=pid_outer ? $signed({1'b0,speed_limit,8'd0}) :
                        $signed({7'd0,duty_limit,8'd0});
                    mul_value<=error;mul_gain<=pid_outer ? position_kp_sample : speed_kp_sample;
                    mul_start<=1;control_state<=PID_P_WAIT;
                end
                PID_P_WAIT:if(mul_done)begin
                    pd_sum<={mul_product[47],mul_product};mul_value<=error;
                    mul_gain<=pid_outer ? position_ki_sample : speed_ki_sample;
                    mul_start<=1;control_state<=PID_I_WAIT;
                end
                PID_I_WAIT:if(mul_done)begin
                    pid_value<=pid_alu_result;mul_value<=measurement_delta;
                    mul_gain<=pid_outer ? position_kd_sample : speed_kd_sample;
                    mul_start<=1;control_state<=PID_INTEGRATE;
                end
                PID_D_WAIT:if(mul_done)begin
                    pd_sum<=pid_alu_result;control_state<=PID_COMMAND;
                end
                PID_INTEGRATE:begin
                    integral_candidate<=pid_clamped_q8;control_state<=PID_D_WAIT;
                end
                PID_COMMAND:begin
                    pid_value<=pid_alu_result;control_state<=PID_SELECT;
                end
                PID_SELECT:begin
                    if((pid_above_limit && error>0) || (pid_below_limit && error<0))begin
                        // Reject further windup, but still use the available
                        // saturated output this sample. Recomputing P+old I
                        // here can strand the command below the duty limit.
                        // Reuse the same clamp for old I after a live limit
                        // reduction; no second wide comparator/ALU is needed.
                        command_clamped_q8<=pid_clamped_q8;
                        pid_value<={{24{integral_old[24]}},integral_old};
                        control_state<=PID_REJECT_COMMAND;
                    end else control_state<=PID_CLAMP;
                end
                PID_REJECT_COMMAND:begin
                    integral_candidate<=pid_clamped_q8;
                    pid_value<={{24{command_clamped_q8[24]}},command_clamped_q8};
                    control_state<=PID_CLAMP;
                end
                PID_CLAMP:begin
                    if(pid_outer)position_integral_q8<=integral_candidate;
                    else speed_integral_q8<=integral_candidate[18:0];
                    command_clamped_q8<=pid_clamped_q8;control_state<=PID_ROUND;
                end
                PID_ROUND:begin rounded_command<=round_command_q8(command_clamped_q8);control_state<=PID_APPLY;end
                PID_APPLY:begin
                    if(pid_outer)begin
                        outer_speed<=rounded_command[15:0];desired_speed<=rounded_command[15:0];
                        control_state<=SPEED_PREP;
                    end else control_state<=SPEED_FLOOR;
                end
                SPEED_FLOOR:begin
                    floor_temporary=rounded_command[10:0];
                    minimum_effective=minimum_duty>duty_limit ? $signed({1'b0,duty_limit}) :
                        $signed({1'b0,minimum_duty});
                    if(floor_temporary>0 && floor_temporary<minimum_effective)floor_temporary=minimum_effective;
                    else if(floor_temporary<0 && floor_temporary> -minimum_effective)floor_temporary=-minimum_effective;
                    floor_command<=floor_temporary;control_state<=SPEED_POLARITY;
                end
                SPEED_POLARITY:begin
                    if((pid_duty>0 && floor_command<0) || (pid_duty<0 && floor_command>0) || floor_command==0)begin
                        pid_duty<=0;control_state<=CHECK_FAULTS;
                    end else begin
                        duty_delta<=$signed({floor_command[10],floor_command})-
                            $signed({pid_duty[10],pid_duty[10:0]});
                        control_state<=SPEED_SLEW;
                    end
                end
                SPEED_SLEW:begin
                    if(duty_delta>PWM_SLEW_PERMILLE)slew_delta<=PWM_SLEW_PERMILLE;
                    else if(duty_delta< -PWM_SLEW_PERMILLE)slew_delta<=-PWM_SLEW_PERMILLE;
                    else slew_delta<=duty_delta;
                    control_state<=SPEED_LIMIT;
                end
                SPEED_LIMIT:begin
                    // Two sequential stages separate slew limiting from the
                    // final absolute duty clamp. The duty output is unchanged
                    // until both calculations are complete.
                    duty_after_slew=$signed({{2{pid_duty[10]}},pid_duty[10:0]})+
                        $signed({slew_delta[11],slew_delta});
                    if(duty_after_slew>$signed({3'd0,duty_limit}))pid_duty<=$signed({6'd0,duty_limit});
                    else if(duty_after_slew< -$signed({3'd0,duty_limit}))pid_duty<=-$signed({6'd0,duty_limit});
                    else pid_duty<={{3{duty_after_slew[12]}},duty_after_slew};
                    control_state<=CHECK_FAULTS;
                end
                CHECK_FAULTS:begin
                    if(target_effective==0)begin
                        target_sign_valid<=0;startup_samples<=0;stall_samples<=0;reverse_samples<=0;
                    end else if(!target_sign_valid || last_target_sign!=target_effective[15])begin
                        target_sign_valid<=1;last_target_sign<=target_effective[15];
                        startup_samples<=0;stall_samples<=0;reverse_samples<=0;
                    end else if(startup_samples<STARTUP_GRACE_SAMPLES)begin
                        startup_samples<=startup_samples+1'b1;stall_samples<=0;reverse_samples<=0;
                    end else begin
                        if(abs16(pid_duty)>=MOTION_DUTY_THRESHOLD &&
                           abs16(target_effective)>=MOTION_SPEED_THRESHOLD && abs32(speed_raw)<=1)begin
                            if(stall_samples>=STALL_SAMPLES-1)fault[0]<=1;
                            else stall_samples<=stall_samples+1'b1;
                        end else stall_samples<=0;
                        if(abs16(pid_duty)>=MOTION_DUTY_THRESHOLD &&
                           abs16(target_effective)>=MOTION_SPEED_THRESHOLD &&
                           abs32(speed_raw)>=MOTION_SPEED_THRESHOLD &&
                           speed_raw[31]!=target_effective[15])begin
                            if(reverse_samples>=REVERSE_SAMPLES-1)fault[1]<=1;
                            else reverse_samples<=reverse_samples+1'b1;
                        end else reverse_samples<=0;
                    end
                    control_state<=IDLE;
                end
                default:control_state<=IDLE;
            endcase
            // These encoder guards apply to BOTH closed-loop and calibration
            // drive, once per existing 10 ms sample. Only the PID-specific
            // stall/opposite-target heuristics remain in CHECK_FAULTS above.
            if(bridge_enabled && sample_ready)begin
                previous_invalid<=sampled_invalid;
                if(encoder_overflow)fault[3]<=1;
                if(abs32(speed_raw)>MAX_SPEED_COUNTS)fault[4]<=1;
                if(sampled_invalid==16'hffff || invalid_delta>=INVALID_PER_SAMPLE)fault[2]<=1;
            end
        end
    end
endmodule
