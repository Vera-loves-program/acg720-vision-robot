// Differential-drive odometry and first-stage point navigation, 2026-10-23.
// This is NOT SLAM. Pose drifts without an external map/localization correction.
// x/y refer to the drive axle midpoint, +x is heading zero, +y and yaw are CCW.
// LSM6DSV16X must retain its existing +/-2000 dps setting: 0.07 deg/s/count.
// Encoder positions must already use the same positive-forward convention.
// tick_10ms MUST be 100 Hz; all other logic runs on the existing clk domain.
// Translation/GOTO require explicit wheel geometry. An intentional TURN with
// avoidance disabled can use calibrated fresh gyro alone for empty-floor tests.
// command: 0 STOP/clear diagnostic, 1 MANUAL, 2 TURN absolute, 3 GOTO, 4 GYRO CAL.
// Coordinates are millimetres, angles millidegrees, wheel targets counts/10 ms.
// Calibrate only with a stationary chassis and a vertical gyro Z axis.
// Lidar ranges are from its optical origin. User thresholds MUST include sensor
// offset, the measured body swept envelope, stopping distance and uncertainty.
// A blocked point mission stops and reports a suggested side. It deliberately
// does not claim that a single clear side proves an entire detour is traversable.
// status: 0 stop, 1 calibrating, 2 geometry/gyro uncalibrated, 3 manual, 4 turn,
//         5 drive, 6 arrived, 7 blocked, 8 stale sensor, 9 odometry/config fault.
// fault: bit0 implausible encoder jump; bit1 overrun; bit2 bad pose/goal;
//        bit3 invalid command speed/angle. STOP or accepted POSE_SET clears it.
module robot_navigation #(parameter BODY_GUARD=0)(
    input wire clk, reset_n, tick_10ms,
    input wire safety_stop, motion_enable,
    input wire signed [31:0] encoder_position_left, encoder_position_right,
    input wire encoder_zeroed,
    input wire signed [15:0] gyro_z,
    input wire gyro_sample, gyro_valid, gyro_invert,
    input wire geometry_valid,
    input wire [15:0] wheel_mm_per_rev, encoder_counts_per_rev, track_mm,
    input wire pose_set,
    input wire signed [31:0] pose_x_mm, pose_y_mm, pose_heading_mdeg,
    input wire command_valid,
    input wire [2:0] command,
    input wire signed [15:0] manual_left, manual_right,
    input wire manual_heading_hold,
    input wire signed [31:0] turn_heading_mdeg, goal_x_mm, goal_y_mm,
    input wire [1:0] turn_half_direction, // 0 shortest, 1 positive half-turn, 2 negative
    input wire [15:0] cruise_counts, turn_counts, arrival_tolerance_mm,
    input wire avoid_enable, lidar_valid,
    input wire [15:0] clear_front_mm, clear_left_mm, clear_right_mm, clear_rear_mm,
    input wire [15:0] stop_clearance_mm, slow_clearance_mm, rotate_clearance_mm,
    input wire body_guard_ready,body_front_stop,body_rear_stop,body_front_slow,body_turn_stop,
    input wire body_nav_ahead,
    input wire [3:0] body_coverage,
    input wire lidar_correction_valid,
    input wire signed [15:0] lidar_correction_x_mm, lidar_correction_y_mm,
    output reg run_request,
    output reg signed [15:0] target_speed_left, target_speed_right,
    output wire signed [31:0] x_mm, y_mm,
    output reg signed [31:0] heading_mdeg,
    output reg [15:0] heading_q6,
    output wire pose_valid,
    output reg gyro_calibrated, calibrating,
    output reg [3:0] status,
    output reg [7:0] fault,
    output reg pose_update,
    output reg turn_complete_pulse,
    output reg [1:0] recommended_side,
    output wire correction_allowed, correction_ready,
    output reg correction_accepted,
    output reg [31:0] correction_count
);
    localparam [2:0] MODE_STOP=0, MODE_MANUAL=1, MODE_TURN=2,
        MODE_GOTO=3, MODE_DONE=4, MODE_ALIGN=5;
    reg [2:0] motion_mode;
    // Retain the completed gyro-only TURN indication without restarting motion
    // if heading later drifts. This bit grants no MANUAL/GOTO permission.
    reg gyro_turn_done;
    reg [1:0] half_turn_direction;
    reg point_driving, hold_heading;
    reg signed [15:0] manual_latched_left, manual_latched_right;
    reg signed [31:0] goal_latched_x, goal_latched_y;
    reg [15:0] goto_arrival_tolerance_mm;
    reg goal_path_heading_valid;
    reg signed [31:0] goal_path_heading;
    reg signed [17:0] goal_path_cos,goal_path_sin;
    reg path_trig_pending,goal_reacquiring;
    reg signed [5:0] path_cross_mm;
    reg signed [31:0] heading_target; // 0.1 millidegree, +/-1800000
    reg signed [31:0] heading_accumulator;
    reg signed [31:0] position_x_q8, position_y_q8;
    reg [15:0] command_generation, frame_generation;
    reg correction_wait_drop;
    assign x_mm=position_x_q8>>>8;
    assign y_mm=position_y_q8>>>8;

    function signed [31:0] abs32;
        input signed [31:0] value;
        begin abs32=value[31] ? -value : value; end
    endfunction
    function signed [31:0] wrap_heading;
        input signed [31:0] angle;
        begin
            if (angle>=32'sd1800000) wrap_heading=angle-32'sd3600000;
            else if (angle< -32'sd1800000) wrap_heading=angle+32'sd3600000;
            else wrap_heading=angle;
        end
    endfunction
    function signed [31:0] atan_constant;
        input [3:0] index;
        begin
            case (index)
                0:atan_constant=450000; 1:atan_constant=265651;
                2:atan_constant=140362; 3:atan_constant=71250;
                4:atan_constant=35763; 5:atan_constant=17899;
                6:atan_constant=8952; 7:atan_constant=4476;
                8:atan_constant=2238; 9:atan_constant=1119;
                10:atan_constant=560; 11:atan_constant=280;
                12:atan_constant=140; 13:atan_constant=70;
                14:atan_constant=35; default:atan_constant=17;
            endcase
        end
    endfunction

    wire geometry_ok=geometry_valid && wheel_mm_per_rev>=50 &&
        wheel_mm_per_rev<=2000 && encoder_counts_per_rev>=4 &&
        track_mm>=50 && track_mm<=2000;
    reg signed [15:0] gyro_latest, gyro_bias;
    reg [5:0] gyro_age;
    wire gyro_fresh=gyro_valid && gyro_age<20;
    assign pose_valid=geometry_ok && gyro_calibrated && gyro_fresh && fault==0;
    wire valid_turn_command=command_valid && command==3'd2 &&
        turn_heading_mdeg>=-180000 && turn_heading_mdeg<=180000;
    // Use the incoming command on its acceptance edge: the old STOP mode must
    // not cancel a new TURN, and an old TURN must not authorize a new GOTO.
    wire gyro_only_turn=
        (command_valid ? valid_turn_command : motion_mode==MODE_TURN);
    wire gyro_only_turn_result=!command_valid &&
        motion_mode==MODE_DONE && gyro_turn_done;
    wire motion_pose_ok=pose_valid || ((gyro_only_turn || gyro_only_turn_result) &&
        gyro_calibrated && gyro_fresh && fault==0);
    // Explicit90/180 commands are gyro turns, not map missions. GOTO still
    // checks live lidar while rotating toward a goal and while translating.
    wire avoid_required=avoid_enable && !gyro_only_turn && !gyro_only_turn_result;
    wire signed [16:0] gyro_corrected={gyro_latest[15],gyro_latest}-
                                      {gyro_bias[15],gyro_bias};
    wire signed [31:0] gyro_extended={{15{gyro_corrected[16]}},gyro_corrected};
    wire signed [31:0] gyro_step_raw=(gyro_extended<<<3)-gyro_extended;
    wire signed [31:0] gyro_step=gyro_invert ? -gyro_step_raw : gyro_step_raw;
    wire signed [31:0] next_heading=wrap_heading(heading_accumulator+
        ((gyro_fresh && gyro_calibrated) ? gyro_step : 32'sd0));
    wire signed [31:0] midpoint_heading=wrap_heading(heading_accumulator+
        ((gyro_fresh && gyro_calibrated) ? (gyro_step>>>1) : 32'sd0));

    reg [6:0] calibration_count;
    reg signed [23:0] calibration_sum;
    reg signed [15:0] calibration_min, calibration_max;
    reg signed [31:0] calibration_left, calibration_right;
    wire signed [31:0] calibration_delta_left=encoder_position_left-calibration_left;
    wire signed [31:0] calibration_delta_right=encoder_position_right-calibration_right;
    wire calibration_stationary=abs32(calibration_delta_left)<=2 &&
                                  abs32(calibration_delta_right)<=2;
    wire signed [23:0] calibration_sum_next=calibration_sum+
        {{8{gyro_z[15]}},gyro_z};
    wire signed [15:0] calibration_min_next=gyro_z<calibration_min ? gyro_z : calibration_min;
    wire signed [15:0] calibration_max_next=gyro_z>calibration_max ? gyro_z : calibration_max;
    wire signed [16:0] calibration_span=
        {calibration_max_next[15],calibration_max_next}-
        {calibration_min_next[15],calibration_min_next};
    // 128 fresh gyro readings (~2.13 s at 60 Hz), restarted on wheel motion.
    // Quiet encoder readings are only a guard: operator must hold the chassis.
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            gyro_latest<=0; gyro_bias<=0; gyro_age<=63;
            gyro_calibrated<=0; calibrating<=1; calibration_count<=0;
            calibration_sum<=0; calibration_min<=32767; calibration_max<=-32768;
            calibration_left<=0; calibration_right<=0;
        end else begin
            if (tick_10ms && gyro_age!=63) gyro_age<=gyro_age+1'b1;
            if (gyro_sample && gyro_valid) begin gyro_latest<=gyro_z; gyro_age<=0; end
            if (command_valid && command==4) begin
                gyro_calibrated<=0; calibrating<=1; calibration_count<=0;
                calibration_sum<=0; calibration_min<=32767; calibration_max<=-32768;
                calibration_left<=encoder_position_left;
                calibration_right<=encoder_position_right;
            end else if (calibrating && gyro_sample && gyro_valid) begin
                if (run_request || !calibration_stationary) begin
                    calibration_count<=0; calibration_sum<=0;
                    calibration_min<=32767; calibration_max<=-32768;
                    calibration_left<=encoder_position_left;
                    calibration_right<=encoder_position_right;
                end else if (calibration_count==127) begin
                    if (calibration_span<=64) begin
                        gyro_bias<=calibration_sum_next>>>7;
                        gyro_calibrated<=1; calibrating<=0;
                    end
                    calibration_count<=0; calibration_sum<=0;
                    calibration_min<=32767; calibration_max<=-32768;
                end else begin
                    calibration_count<=calibration_count+1'b1;
                    calibration_sum<=calibration_sum_next;
                    calibration_min<=calibration_min_next;
                    calibration_max<=calibration_max_next;
                end
            end
        end
    end

    reg [4:0] state;
    reg signed [31:0] last_encoder_left, last_encoder_right;
    wire signed [31:0] delta_left=encoder_position_left-last_encoder_left;
    wire signed [31:0] delta_right=encoder_position_right-last_encoder_right;
    wire encoder_step_ok=abs32(delta_left)<=2048 && abs32(delta_right)<=2048;
    reg signed [31:0] count_sum, distance_product, distance_q8;
    reg [15:0] circumference_sample, counts_sample;
    reg sample_geometry_ok, distance_negative;
    reg signed [31:0] trig_heading;
    reg signed [31:0] cordic_x, cordic_y, cordic_angle;
    reg [3:0] cordic_index;
    // Rotation/vectoring retain exact signed32 arithmetic and share one ALU.
    // Copy old Y/X, then perform index single-bit arithmetic shifts. This
    // replaces the wide variable barrel selector without reducing precision.
    // Old X remains unchanged until Y has consumed it; direction is latched
    // before either update. Index0 skips both shift loops.
    reg [2:0] cordic_phase;
    reg [3:0] cordic_shift_remaining;
    reg signed [31:0] cordic_shift_copy;
    reg cordic_subtract_latched;
    reg signed [31:0] cordic_pending_x;
    wire cordic_subtract_x=state==4 ? !cordic_angle[31] : cordic_y[31];
    wire cordic_subtract=cordic_phase==4 ? !cordic_subtract_latched : cordic_subtract_latched;
    wire signed [31:0] cordic_operand_a=cordic_phase==2 ? cordic_x :
        cordic_phase==4 ? cordic_y : cordic_angle;
    wire signed [31:0] cordic_operand_b=cordic_phase==5 ? atan_constant(cordic_index) :
        cordic_shift_copy;
    wire signed [31:0] cordic_alu_result=cordic_operand_a+
        (cordic_operand_b^{32{cordic_subtract}})+{31'd0,cordic_subtract};
    reg signed [36:0] displacement_x, displacement_y;
    reg signed [31:0] goal_dx, goal_dy;
    // Reuse the SAME two19x18 products for body displacement and the path's
    // cross/along projections. No extra multiplier or floating-point core.
    wire path_projection=state==25 || state==28;
    wire signed [18:0] multiply_x_input=state==25 ? $signed(goal_dy[26:8]) :
        state==28 ? $signed(goal_dx[26:8]) : $signed(distance_q8[18:0]);
    wire signed [18:0] multiply_y_input=state==25 ? $signed(goal_dx[26:8]) :
        state==28 ? $signed(goal_dy[26:8]) : $signed(distance_q8[18:0]);
    wire signed [17:0] multiply_x_factor=path_projection ? goal_path_cos : $signed(cordic_x[17:0]);
    wire signed [17:0] multiply_y_factor=path_projection ? goal_path_sin : $signed(cordic_y[17:0]);
    wire signed [37:0] path_cross_product=$signed({displacement_x[36],displacement_x})-
        $signed({displacement_y[36],displacement_y});
    wire signed [37:0] path_cross_rounded=(path_cross_product+
        (path_cross_product[37] ? 38'sd16383 : 38'sd16384))>>>15;
    wire signed [37:0] path_along_product=$signed({displacement_x[36],displacement_x})+
        $signed({displacement_y[36],displacement_y});
    wire signed [31:0] path_cross_extended={{26{path_cross_mm[5]}},path_cross_mm};
    // 0.25deg per lateral mm, limited to20mm -> +/-5deg correction.
    wire signed [31:0] path_cross_angle=(path_cross_extended<<<11)+(path_cross_extended<<<9)-
        (path_cross_extended<<<6)+(path_cross_extended<<<2);
    reg signed [31:0] gyro_step_sample,turn_error_sample;
    reg turn_braking;
    reg [31:0] remaining_mm;
    reg divider_start;
    reg [31:0] divider_numerator, divider_denominator;
    wire divider_busy, divider_done;
    wire [31:0] divider_quotient, divider_remainder;
    unsigned_divmod u_geometry_divider(
        .clk(clk),.reset_n(reset_n),.start(divider_start),
        .numerator(divider_numerator),.denominator(divider_denominator),
        .busy(divider_busy),.done(divider_done),
        .quotient(divider_quotient),.remainder(divider_remainder));

    wire signed [31:0] wrapped_heading_error=wrap_heading(heading_target-heading_accumulator);
    // Exactly180 degrees has two equivalent absolute targets. Keep the
    // requested relative direction around the initial wrap boundary, including
    // a little gyro drift between request acceptance and the next10ms frame.
    wire signed [31:0] heading_error=motion_mode==MODE_TURN && half_turn_direction==1 && wrapped_heading_error< -1780000 ?
        wrapped_heading_error+3600000 : motion_mode==MODE_TURN && half_turn_direction==2 && wrapped_heading_error>1780000 ?
        wrapped_heading_error-3600000 : wrapped_heading_error;
    reg signed [31:0] heading_error_sample;
    reg [31:0] heading_error_abs_sample;
    // Integer arithmetic shifts round negative errors down: the former
    // error>>>15 commanded-1 for a tiny negative error but0 for its positive
    // mirror. Use sign/magnitude so straight tracking has no directional bias.
    // GOTO needs finer yaw feedback than the former3.2768deg/count dead zone;
    // manual heading hold keeps its existing gain. Explicit TURN is separate.
    wire [9:0] steering_magnitude=motion_mode==MODE_GOTO ?
        heading_error_abs_sample[22:13] : heading_error_abs_sample[24:15];
    wire [15:0] cruise_limit=cruise_counts==0 ? 16'd1 :
        (cruise_counts>200 ? 16'd200 : cruise_counts);
    wire [15:0] turn_limit=turn_counts==0 ? 16'd1 :
        (turn_counts>100 ? 16'd100 : turn_counts);
    wire [15:0] stop_limit=stop_clearance_mm<400 ? 16'd400 : stop_clearance_mm;
    wire [16:0] slow_floor={1'b0,stop_limit}+17'd100;
    wire [16:0] slow_limit={1'b0,slow_clearance_mm}<slow_floor ? slow_floor :
        {1'b0,slow_clearance_mm};
    wire [15:0] rotation_limit=rotate_clearance_mm<450 ? 16'd450 : rotate_clearance_mm;
    // FFFF is the C1 driver's unknown/no-return sentinel, never clear space.
    wire front_known=clear_front_mm>=50 && clear_front_mm<=16000;
    wire rear_known=clear_rear_mm>=50 && clear_rear_mm<=16000;
    wire left_known=clear_left_mm>=50 && clear_left_mm<=16000;
    wire right_known=clear_right_mm>=50 && clear_right_mm<=16000;
    wire turn_space_known=front_known && rear_known && left_known && right_known;
    wire turn_space_ok=turn_space_known && clear_front_mm>=rotation_limit &&
        clear_rear_mm>=rotation_limit && clear_left_mm>=rotation_limit && clear_right_mm>=rotation_limit;
    // Freeze the tolerance with each accepted goal. Intermediate map
    // waypoints may use60mm, while the actual requested XY uses5mm.
    wire [15:0] arrival_limit=goto_arrival_tolerance_mm;
    wire [31:0] arrival_q8={16'd0,arrival_limit}<<8;
    reg signed [10:0] steering_work, speed_work, left_work, right_work;
    reg [15:0] turn_work;
    reg blocked_work, turning_work, reverse_work, unknown_work;
    // Registered decision stages break the measured cruise_counts -> wheel
    // target/status path. All stages run at50MHz inside ONE10ms frame.
    localparam [4:0] D_SPEED=17,D_SLOW=18,D_STEER=19,D_MIX=20,
        D_GUARD=21,D_CLAMP=22,D_COMMIT=23;
    localparam [1:0] D_HOLD=0,D_MANUAL=1,D_TURN=2,D_GOTO=3;
    reg [1:0] decision_kind;
    reg [2:0] decision_mode;
    reg [3:0] decision_status;
    reg decision_valid,decision_hold_heading,decision_turn_done,decision_start_drive;
    reg [7:0] cruise_sample,approach_sample;
    reg [6:0] turn_sample;
    wire [10:0] manual_magnitude=left_work[10] ? -left_work : left_work;
    // Recheck the LIVE scan on the publication edge. A scan arriving between
    // D_GUARD and D_COMMIT must not publish an already-obstructed request.
    wire decision_unknown=BODY_GUARD ? (turning_work ? body_coverage!=4'b1111 :
        reverse_work ? !body_coverage[3] : (left_work>0 || right_work>0) ? !body_coverage[0] : 1'b0) :
        (turning_work ? !turn_space_known : reverse_work ? !rear_known :
        (left_work>0 || right_work>0) ? !front_known : 1'b0);
    wire decision_obstacle=BODY_GUARD ? (turning_work ? body_turn_stop :
        reverse_work ? body_rear_stop : (left_work>0 || right_work>0) ? body_front_stop : 1'b0) :
        (turning_work ? !turn_space_ok : reverse_work ? clear_rear_mm<stop_limit :
        (left_work>0 || right_work>0) ? clear_front_mm<stop_limit : 1'b0);
    wire decision_blocked_live=avoid_required && (decision_unknown || decision_obstacle ||
        (BODY_GUARD && !body_guard_ready));
    // Permission is a stable low-speed condition, suitable for mapper's
    // correction_enable. ready is the actual one-cycle odometry commit slot.
    // Mapper must hold valid and dx/dy until correction_accepted (ACK), then
    // integrate its frame with the SAME full displacement. Clear both modules
    // together on a manual pose calibration or encoder reset.
    assign correction_allowed=pose_valid && !calibrating && !pose_set &&
        !encoder_zeroed && abs32(distance_q8)<=512 && abs32(gyro_step)<=2000;
    assign correction_ready=correction_allowed && state==6 && !correction_wait_drop &&
        abs32({{16{lidar_correction_x_mm[15]}},lidar_correction_x_mm})<=128 &&
        abs32({{16{lidar_correction_y_mm[15]}},lidar_correction_y_mm})<=128;
    // Each pass uses sum(4+2*index,index=0..15)=304 clocks. Both passes add
    // 512 clocks (10.24 us at50MHz) versus the former shared-barrel version.
    // Products/division occur in other states, with no extra sample period.
    // Worst-case service time is far below the 500000-cycle 10 ms interval.
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            motion_mode<=MODE_STOP; gyro_turn_done<=0; point_driving<=0; hold_heading<=0;
            manual_latched_left<=0; manual_latched_right<=0;
            goal_latched_x<=0; goal_latched_y<=0; goto_arrival_tolerance_mm<=5; heading_target<=0;
            goal_path_heading_valid<=0;goal_path_heading<=0;
            goal_path_cos<=32768;goal_path_sin<=0;path_trig_pending<=0;goal_reacquiring<=0;path_cross_mm<=0;
            gyro_step_sample<=0;turn_error_sample<=0;turn_braking<=0;
            heading_accumulator<=0; position_x_q8<=0; position_y_q8<=0;
            heading_mdeg<=0; heading_q6<=0; state<=0; last_encoder_left<=0; last_encoder_right<=0;
            command_generation<=0; frame_generation<=0; correction_wait_drop<=0;
            correction_accepted<=0; correction_count<=0;
            count_sum<=0; distance_product<=0; distance_q8<=0;
            circumference_sample<=0; counts_sample<=1; sample_geometry_ok<=0;
            distance_negative<=0; trig_heading<=0;
            cordic_x<=0; cordic_y<=0; cordic_angle<=0; cordic_index<=0;
            cordic_phase<=0;cordic_shift_remaining<=0;cordic_shift_copy<=0;
            cordic_subtract_latched<=0;cordic_pending_x<=0;
            displacement_x<=0; displacement_y<=0; goal_dx<=0; goal_dy<=0;
            remaining_mm<=0; divider_start<=0; divider_numerator<=0;
            heading_error_sample<=0; heading_error_abs_sample<=0;
            divider_denominator<=1; run_request<=0; target_speed_left<=0;
            target_speed_right<=0; status<=0; fault<=0; pose_update<=0;
            recommended_side<=0;turn_complete_pulse<=0;half_turn_direction<=0;
            steering_work<=0;speed_work<=0;left_work<=0;right_work<=0;turn_work<=0;
            blocked_work<=0;turning_work<=0;reverse_work<=0;unknown_work<=0;
            decision_kind<=D_HOLD;decision_mode<=MODE_STOP;decision_status<=0;
            decision_valid<=0;decision_hold_heading<=0;decision_turn_done<=0;decision_start_drive<=0;
            cruise_sample<=1;approach_sample<=1;turn_sample<=1;
        end else begin
            turn_complete_pulse<=0;
            divider_start<=0; pose_update<=0; correction_accepted<=0;
            if (!lidar_correction_valid) correction_wait_drop<=0;
            case (state)
                0:if (tick_10ms && !divider_busy) begin
                    frame_generation<=command_generation;
                    last_encoder_left<=encoder_position_left;
                    last_encoder_right<=encoder_position_right;
                    circumference_sample<=wheel_mm_per_rev;
                    counts_sample<=encoder_counts_per_rev;
                    sample_geometry_ok<=geometry_ok && encoder_step_ok;
                    count_sum<=encoder_step_ok ? delta_left+delta_right : 32'sd0;
                    if (!encoder_step_ok) fault[0]<=1;
                    heading_accumulator<=next_heading;
                    trig_heading<=midpoint_heading;
                    gyro_step_sample<=(gyro_fresh && gyro_calibrated) ? gyro_step : 32'sd0;
                    state<=1;
                end
                1:begin
                    // Both inputs have explicit bounds before multiplication.
                    // Missing geometry suppresses translation only. Keep this
                    // frame running so gyro heading reaches the TURN controller.
                    distance_product<=sample_geometry_ok ?
                        $signed(count_sum[13:0])*$signed({1'b0,circumference_sample[10:0]}) : 0;
                    state<=2;
                end
                2:begin
                    // ds_Q8=(dl+dr)*circumference*128/counts_per_revolution.
                    // Max legal numerator: 4096*2000*128=1048576000 (<2^30).
                    distance_negative<=distance_product[31];
                    divider_numerator<=abs32(distance_product)<<7;
                    divider_denominator<=counts_sample==0 ? 32'd1 : {16'd0,counts_sample};
                    divider_start<=1; state<=3;
                end
                3:if (divider_done) begin
                    // Reject >500 mm/10 ms rather than corrupting pose silently.
                    if (divider_quotient>128000) begin distance_q8<=0; fault[0]<=1; end
                    else distance_q8<=distance_negative ? -$signed(divider_quotient) : $signed(divider_quotient);
                    cordic_x<=(trig_heading>900000 || trig_heading< -900000) ? -32'sd19898 : 32'sd19898;
                    cordic_y<=0;
                    if (trig_heading>900000) cordic_angle<=trig_heading-1800000;
                    else if (trig_heading< -900000) cordic_angle<=trig_heading+1800000;
                    else cordic_angle<=trig_heading;
                    cordic_index<=0;cordic_phase<=0;cordic_shift_remaining<=0; state<=4;
                end
                4,9:begin
                    case(cordic_phase)
                        0:begin
                            cordic_shift_copy<=cordic_y;
                            cordic_shift_remaining<=cordic_index;
                            cordic_subtract_latched<=cordic_subtract_x;
                            cordic_phase<=cordic_index==0 ? 2 : 1;
                        end
                        1,3:begin
                            cordic_shift_copy<=cordic_shift_copy>>>1;
                            if(cordic_shift_remaining==1)begin
                                cordic_shift_remaining<=0;
                                cordic_phase<=cordic_phase==1 ? 2 : 4;
                            end else cordic_shift_remaining<=cordic_shift_remaining-1'b1;
                        end
                        2:begin
                            cordic_pending_x<=cordic_alu_result;
                            cordic_shift_copy<=cordic_x;
                            cordic_shift_remaining<=cordic_index;
                            cordic_phase<=cordic_index==0 ? 4 : 3;
                        end
                        4:begin cordic_y<=cordic_alu_result;cordic_phase<=5;end
                        5:begin
                            cordic_x<=cordic_pending_x;cordic_angle<=cordic_alu_result;
                            cordic_phase<=0;
                            if(cordic_index==15)state<=state==4 ? (path_trig_pending ? 24 : 5) : 10;
                            else cordic_index<=cordic_index+1'b1;
                        end
                        default:cordic_phase<=0;
                    endcase
                end
                5,25,28:begin
                    displacement_x<=multiply_x_input*multiply_x_factor;
                    displacement_y<=multiply_y_input*multiply_y_factor;
                    state<=state==25 ? 26 : state==28 ? 29 : 6;
                end
                6:begin
                    position_x_q8<=position_x_q8+(displacement_x>>>15);
                    position_y_q8<=position_y_q8+(displacement_y>>>15);
                    if (lidar_correction_valid && correction_ready) begin
                        position_x_q8<=position_x_q8+(displacement_x>>>15)+
                            ($signed({{16{lidar_correction_x_mm[15]}},lidar_correction_x_mm})<<<8);
                        position_y_q8<=position_y_q8+(displacement_y>>>15)+
                            ($signed({{16{lidar_correction_y_mm[15]}},lidar_correction_y_mm})<<<8);
                        correction_wait_drop<=1; correction_accepted<=1;
                        correction_count<=correction_count+1'b1;
                    end
                    divider_numerator<=abs32(heading_accumulator);
                    divider_denominator<=10; divider_start<=1; state<=7;
                end
                7:if (divider_done) begin
                    heading_mdeg<=heading_accumulator[31] ? -$signed(divider_quotient) : $signed(divider_quotient);
                    // q6 degrees = normalized_0.1mdeg*4/625, no wide divider.
                    divider_numerator<=(heading_accumulator[31] ?
                        heading_accumulator+32'sd3600000 : heading_accumulator)<<2;
                    divider_denominator<=625; divider_start<=1; state<=13;
                end
                13:if (divider_done) begin
                    heading_q6<=divider_quotient[15:0]; pose_update<=1;
                    goal_dx<=(goal_latched_x<<<8)-position_x_q8;
                    goal_dy<=(goal_latched_y<<<8)-position_y_q8;
                    state<=8;
                end
                8:begin
                    remaining_mm<=(abs32(goal_dx)>abs32(goal_dy) ? abs32(goal_dx) : abs32(goal_dy))>>8;
                    if ((motion_mode==MODE_GOTO || motion_mode==MODE_ALIGN) && pose_valid && frame_generation==command_generation) begin
                        if (abs32(goal_dx)<=arrival_q8 && abs32(goal_dy)<=arrival_q8) begin
                            if(goto_arrival_tolerance_mm<=5 && goal_path_heading_valid)begin
                                // Final XY is not the final chassis orientation.
                                // Restore the initial path tangent without taking
                                // atan2 of the last few noisy millimetres again.
                                heading_target<=goal_path_heading;motion_mode<=MODE_ALIGN;state<=11;
                            end else begin motion_mode<=MODE_DONE;status<=6;state<=11;end
                        end else begin
                            if(goal_path_heading_valid && !goal_reacquiring && motion_mode!=MODE_ALIGN)state<=25;
                            else begin
                                if(motion_mode==MODE_ALIGN)begin
                                    motion_mode<=MODE_GOTO;point_driving<=0;goal_reacquiring<=1;
                                end
                                // Initial path bearing, or an exceptional
                                // recovery after overshoot/alignment XY drift.
                                cordic_x<=goal_dx[31] ? -goal_dx : goal_dx;
                                cordic_y<=goal_dx[31] ? -goal_dy : goal_dy;
                                cordic_angle<=goal_dx[31] ? (goal_dy[31] ? -1800000 : 1800000) : 0;
                                cordic_index<=0;cordic_phase<=0;cordic_shift_remaining<=0; state<=9;
                            end
                        end
                    end else state<=11;
                end
                10:begin
                    if (motion_mode==MODE_GOTO && frame_generation==command_generation)begin
                        heading_target<=wrap_heading(cordic_angle);
                        if(!goal_path_heading_valid)begin
                            goal_path_heading<=wrap_heading(cordic_angle);goal_path_heading_valid<=1;
                            // One rotation pass per segment freezes its unit
                            // direction. Ordinary tracking does not atan2 the
                            // shrinking endpoint residual on every frame.
                            cordic_x<=(cordic_angle>900000 || cordic_angle< -900000) ? -32'sd19898 : 32'sd19898;
                            cordic_y<=0;
                            if(cordic_angle>900000)cordic_angle<=cordic_angle-1800000;
                            else if(cordic_angle< -900000)cordic_angle<=cordic_angle+1800000;
                            path_trig_pending<=1;
                            cordic_index<=0;cordic_phase<=0;cordic_shift_remaining<=0;state<=4;
                        end else state<=11;
                    end else state<=11;
                end
                24:begin
                    goal_path_cos<=cordic_x[17:0];goal_path_sin<=cordic_y[17:0];
                    path_trig_pending<=0;state<=11;
                end
                26:begin
                    if(path_cross_rounded>20)path_cross_mm<=20;
                    else if(path_cross_rounded< -20)path_cross_mm<=-20;
                    else path_cross_mm<=path_cross_rounded[5:0];
                    state<=27;
                end
                27:begin heading_target<=wrap_heading(goal_path_heading+path_cross_angle);state<=28;end
                29:begin
                    if(frame_generation==command_generation && path_along_product< -38'sd163840)begin
                        // The goal fell more than5mm behind the segment after
                        // an overshoot/correction. Stop translating first and
                        // reacquire the real point, preserving final heading.
                        goal_reacquiring<=1;point_driving<=0;
                        cordic_x<=goal_dx[31] ? -goal_dx : goal_dx;
                        cordic_y<=goal_dx[31] ? -goal_dy : goal_dy;
                        cordic_angle<=goal_dx[31] ? (goal_dy[31] ? -1800000 : 1800000) : 0;
                        cordic_index<=0;cordic_phase<=0;cordic_shift_remaining<=0;state<=9;
                    end else state<=11;
                end
                11:begin heading_error_sample<=heading_error; state<=16; end
                16:begin
                    heading_error_abs_sample<=abs32(heading_error_sample);
                    // Predict120ms of current yaw motion before choosing the
                    // turn effort. This brakes a fast turn before its final
                    // stop band, while zero-rate residuals retain minimum1.
                    turn_error_sample<=heading_error_sample-(gyro_step_sample<<<3)-(gyro_step_sample<<<2);
                    state<=12;
                end
                14:if (!divider_busy) begin
                    divider_numerator<=(heading_accumulator[31] ?
                        heading_accumulator+32'sd3600000 : heading_accumulator)<<2;
                    divider_denominator<=625; divider_start<=1; state<=15;
                end
                15:if (divider_done) begin
                    heading_q6<=divider_quotient[15:0]; pose_update<=1; state<=0;
                end
                12:begin
                    // Keep the preceding output active until D_COMMIT. A low
                    // run_request pulse here would reset the wheel PID every
                    // frame. Runtime STOP/sensor interlocks below still win on
                    // EVERY clock, including every pipeline stage.
                    decision_kind<=D_HOLD;decision_mode<=motion_mode;
                    decision_valid<=0;decision_status<=0;
                    decision_hold_heading<=0;decision_turn_done<=0;decision_start_drive<=0;
                    left_work<=0;right_work<=0;speed_work<=0;
                    turning_work<=0;reverse_work<=0;blocked_work<=0;unknown_work<=0;
                    cruise_sample<=cruise_limit[7:0];turn_sample<=turn_limit[6:0];
                    // Bound distance independently, BEFORE comparing it with
                    // the registered cruise setting in the next stage.
                    // Counts/10ms: cruise30 is reached at240mm remaining.
                    // Keep one encoder count/10ms at the precise5mm finish;
                    // integer rounding must never make an unfinished leg zero.
                    if(remaining_mm>=1600)approach_sample<=200;
                    else if(remaining_mm<8)approach_sample<=1;
                    else approach_sample<=remaining_mm[10:3];
                    steering_work<=heading_error_sample[31] ? -$signed({1'b0,steering_magnitude}) :
                        $signed({1'b0,steering_magnitude});
                    turn_work<=abs32(turn_error_sample)>>14;
                    turn_braking<=turn_error_sample[31]!=heading_error_sample[31] && gyro_step_sample!=0;
                    if (motion_mode==MODE_STOP) decision_status<=calibrating ? 1 : 0;
                    else if (motion_mode==MODE_DONE) decision_status<=6;
                    else if (fault!=0) decision_status<=9;
                    else if ((!geometry_ok && motion_mode!=MODE_TURN) ||
                             !gyro_calibrated) decision_status<=calibrating ? 1 : 2;
                    else if (!gyro_fresh || (avoid_required && !lidar_valid)) decision_status<=8;
                    else if (motion_enable && !safety_stop && frame_generation==command_generation) begin
                        decision_valid<=1;
                        if (motion_mode==MODE_MANUAL) begin
                            decision_kind<=D_MANUAL;decision_status<=3;
                            left_work<=manual_latched_left[10:0];right_work<=manual_latched_right[10:0];
                            decision_hold_heading<=hold_heading && manual_latched_left==manual_latched_right && manual_latched_left!=0;
                        end else if (motion_mode==MODE_TURN || motion_mode==MODE_ALIGN || !point_driving ||
                                     heading_error_abs_sample>150000) begin
                            // Keep the verified explicit TURN2deg stop band.
                            // GOTO entry aligns to0.5deg; final-only ALIGN1deg.
                            if (heading_error_abs_sample<=(motion_mode==MODE_TURN ? 32'd20000 :
                                    motion_mode==MODE_ALIGN ? 32'd10000 : 32'd5000)) begin
                                if (motion_mode==MODE_TURN || motion_mode==MODE_ALIGN) begin
                                    decision_turn_done<=1;decision_status<=6;
                                end
                                else begin decision_start_drive<=1;decision_status<=5;end
                            end else begin
                                decision_kind<=D_TURN;decision_status<=4;
                            end
                        end else begin
                            decision_kind<=D_GOTO;decision_status<=5;
                        end
                    end
                    state<=D_SPEED;
                end
                D_SPEED:begin
                    if(decision_kind==D_GOTO)
                        speed_work<=approach_sample<cruise_sample ? $signed({3'd0,approach_sample}) : $signed({3'd0,cruise_sample});
                    else if(decision_kind==D_MANUAL && decision_hold_heading)
                        speed_work<=manual_magnitude>20 ? 11'sd20 : $signed(manual_magnitude);
                    if(decision_kind==D_TURN && turn_braking)turn_work<=0;
                    else if(turn_work==0)turn_work<=1;
                    else if(turn_work>{9'd0,turn_sample})turn_work<={9'd0,turn_sample};
                    state<=D_SLOW;
                end
                D_SLOW:begin
                    if(decision_kind==D_MANUAL && avoid_required && body_front_slow && left_work>0 && right_work>0)begin
                        left_work<=left_work>1 ? left_work>>>1 : left_work;
                        right_work<=right_work>1 ? right_work>>>1 : right_work;
                        speed_work<=speed_work>1 ? speed_work>>>1 : speed_work;
                    end
                    if(decision_kind==D_GOTO && avoid_enable &&
                       (BODY_GUARD ? body_front_slow : {1'b0,clear_front_mm}<slow_limit))
                        speed_work<=speed_work<=1 ? 11'sd1 : speed_work>>>1;
                    state<=D_STEER;
                end
                D_STEER:begin
                    if(steering_work>speed_work)steering_work<=speed_work;
                    else if(steering_work< -speed_work)steering_work<=-speed_work;
                    state<=D_MIX;
                end
                D_MIX:begin
                    if(decision_kind==D_GOTO)begin
                        left_work<=speed_work-steering_work;right_work<=speed_work+steering_work;
                    end else if(decision_kind==D_MANUAL && decision_hold_heading)begin
                        left_work<=left_work-steering_work;right_work<=right_work+steering_work;
                    end else if(decision_kind==D_TURN)begin
                        if(heading_error_sample>0)begin left_work<=-$signed(turn_work[10:0]);right_work<=$signed(turn_work[10:0]);end
                        else begin left_work<=$signed(turn_work[10:0]);right_work<=-$signed(turn_work[10:0]);end
                    end
                    state<=D_GUARD;
                end
                D_GUARD:begin
                    reverse_work<=left_work<0 && right_work<0;
                    turning_work<=(left_work<0 && right_work>=0) || (right_work<0 && left_work>=0);
                    state<=D_CLAMP;
                end
                D_CLAMP:begin
                    blocked_work<=decision_blocked_live;
                    unknown_work<=avoid_required && decision_unknown;
                    if(left_work>200)left_work<=200;
                    else if(left_work< -200)left_work<=-200;
                    if(right_work>200)right_work<=200;
                    else if(right_work< -200)right_work<=-200;
                    state<=D_COMMIT;
                end
                D_COMMIT:begin
                    run_request<=0;target_speed_left<=0;target_speed_right<=0;
                    recommended_side<=0;
                    if(decision_mode==motion_mode && frame_generation==command_generation)
                        status<=decision_status;
                    if(decision_valid && decision_mode==motion_mode &&
                       frame_generation==command_generation && motion_enable && !safety_stop &&
                       motion_pose_ok && !calibrating && (!avoid_required || lidar_valid))begin
                        if(blocked_work || decision_blocked_live)begin
                            status<=unknown_work || decision_unknown ? 8 : 7;
                            if(left_known && clear_left_mm>=rotation_limit &&
                               (!right_known || clear_left_mm>clear_right_mm))recommended_side<=1;
                            else if(right_known && clear_right_mm>=rotation_limit)recommended_side<=2;
                        end else begin
                            if(decision_turn_done)begin
                                if(motion_mode==MODE_TURN)turn_complete_pulse<=1;
                                motion_mode<=MODE_DONE;
                                gyro_turn_done<=motion_mode==MODE_TURN;
                            end
                            if(decision_start_drive)point_driving<=1;
                            if(left_work!=0 || right_work!=0)begin
                                target_speed_left<={{5{left_work[10]}},left_work};
                                target_speed_right<={{5{right_work[10]}},right_work};
                                run_request<=1;
                            end
                        end
                    end
                    state<=0;
                end
                default:state<=0;
            endcase
            if (tick_10ms && state!=0 && state!=14 && state!=15) begin
                fault[1]<=1;turn_complete_pulse<=0;
            end

            // An accepted pose calibration also rebases encoder history.
            // Never teleport the odometry while a commanded movement is running.
            if (pose_set && !run_request) begin
                turn_complete_pulse<=0;
                gyro_turn_done<=0;
                run_request<=0; target_speed_left<=0; target_speed_right<=0;
                motion_mode<=MODE_STOP; point_driving<=0; hold_heading<=0;
                if (pose_x_mm>=-32000 && pose_x_mm<=32000 &&
                    pose_y_mm>=-32000 && pose_y_mm<=32000 &&
                    pose_heading_mdeg>=-180000 && pose_heading_mdeg<=180000) begin
                    position_x_q8<=pose_x_mm<<<8; position_y_q8<=pose_y_mm<<<8;
                    heading_accumulator<=wrap_heading((pose_heading_mdeg<<<3)+(pose_heading_mdeg<<<1));
                    heading_mdeg<=pose_heading_mdeg==180000 ? -180000 : pose_heading_mdeg;
                    last_encoder_left<=encoder_position_left; last_encoder_right<=encoder_position_right;
                    state<=14; divider_start<=0; fault<=0; pose_update<=0;
                    correction_wait_drop<=0; correction_count<=0;
                    path_trig_pending<=0;goal_path_heading_valid<=0;goal_reacquiring<=0;
                    command_generation<=command_generation+1'b1;
                end else fault[2]<=1;
            end
            // encoder_zeroed is the same-cycle zero_position event delivered to
            // both encoder counters. Their registered positions become zero on
            // this edge; reading their old outputs here would rebase incorrectly.
            if (encoder_zeroed && !run_request) begin
                turn_complete_pulse<=0;
                gyro_turn_done<=0;
                last_encoder_left<=0; last_encoder_right<=0;
                state<=14; divider_start<=0; correction_wait_drop<=0;
                path_trig_pending<=0;goal_path_heading_valid<=0;goal_reacquiring<=0;
                motion_mode<=MODE_STOP; target_speed_left<=0; target_speed_right<=0;
                command_generation<=command_generation+1'b1;
            end
            if (command_valid) begin
                // Streamed speed updates preserve the captured heading and
                // in-flight10ms result. A fresh MANUAL command starts the run.
                if(command==5 && motion_mode==MODE_MANUAL)begin
                    if(manual_left>=-200 && manual_left<=200 && manual_right>=-200 && manual_right<=200)begin
                        manual_latched_left<=manual_left;manual_latched_right<=manual_right;
                    end
                end else begin
                turn_complete_pulse<=0;
                gyro_turn_done<=0;
                command_generation<=command_generation+1'b1;
                case (command)
                    0:begin
                        motion_mode<=MODE_STOP; run_request<=0; target_speed_left<=0;
                        target_speed_right<=0; fault<=0; status<=0; point_driving<=0;
                    end
                    1:begin
                        run_request<=0; target_speed_left<=0; target_speed_right<=0;
                        if (manual_left>=-200 && manual_left<=200 && manual_right>=-200 && manual_right<=200) begin
                            motion_mode<=MODE_MANUAL; manual_latched_left<=manual_left;
                            manual_latched_right<=manual_right;
                            hold_heading<=manual_heading_hold;
                            heading_target<=heading_accumulator;
                        end else begin motion_mode<=MODE_STOP; fault[3]<=1; end
                    end
                    2:begin
                        half_turn_direction<=turn_half_direction;
                        run_request<=0; target_speed_left<=0; target_speed_right<=0;
                        if (turn_heading_mdeg>=-180000 && turn_heading_mdeg<=180000) begin
                            heading_target<=wrap_heading((turn_heading_mdeg<<<3)+(turn_heading_mdeg<<<1));
                            motion_mode<=MODE_TURN; point_driving<=0;
                        end else begin motion_mode<=MODE_STOP; fault[3]<=1; end
                    end
                    3:begin
                        run_request<=0; target_speed_left<=0; target_speed_right<=0;
                        if (goal_x_mm>=-32000 && goal_x_mm<=32000 && goal_y_mm>=-32000 && goal_y_mm<=32000) begin
                            goal_latched_x<=goal_x_mm; goal_latched_y<=goal_y_mm;
                            goto_arrival_tolerance_mm<=arrival_tolerance_mm<5 ? 16'd5 : arrival_tolerance_mm;
                            goal_path_heading_valid<=0;
                            // An old path-trig pass may still be finishing.
                            // Preserve its return selector until state24 so
                            // it cannot be mistaken for a second odometry pass.
                            goal_reacquiring<=0;
                            motion_mode<=MODE_GOTO; point_driving<=0;
                        end else begin motion_mode<=MODE_STOP; fault[2]<=1; end
                    end
                    4:begin
                        motion_mode<=MODE_STOP; run_request<=0; target_speed_left<=0;
                        target_speed_right<=0; status<=1;
                    end
                    default:begin motion_mode<=MODE_STOP; run_request<=0; fault[3]<=1; end
                endcase
                end
            end
            // High-priority runtime interlocks; a lost enable requires a fresh
            // movement command. Sensor freshness is evaluated every clk, too.
            if (safety_stop || !motion_enable) begin
                turn_complete_pulse<=0;
                motion_mode<=MODE_STOP; gyro_turn_done<=0; run_request<=0;
                target_speed_left<=0; target_speed_right<=0;
            end else if ((pose_set || encoder_zeroed) && !run_request) begin
                turn_complete_pulse<=0;
                // An accepted rebase cancels a concurrent command as well as
                // the old mode; never launch a TURN from a partial old frame.
                motion_mode<=MODE_STOP; gyro_turn_done<=0; run_request<=0;
                target_speed_left<=0; target_speed_right<=0;
                status<=calibrating ? 1 : 0;
            end else if (!motion_pose_ok || calibrating || (avoid_required && !lidar_valid)) begin
                turn_complete_pulse<=0;
                // Do not queue a movement that could start unexpectedly after
                // calibration or sensor recovery. A new command is required.
                motion_mode<=MODE_STOP; gyro_turn_done<=0;
                run_request<=0; target_speed_left<=0; target_speed_right<=0;
                status<=fault!=0 ? 9 : calibrating ? 1 :
                        (!geometry_ok || !gyro_calibrated) ? 2 : 8;
            end else if(BODY_GUARD && avoid_enable && motion_mode==MODE_GOTO &&
                        point_driving && run_request && body_nav_ahead &&
                        target_speed_left>=0 && target_speed_right>=0 &&
                        (target_speed_left>0 || target_speed_right>0))begin
                // A NAV-only forward lookahead parks BEFORE the20mm hard
                // guard. Stop on this clock, invalidate an in-flight frame,
                // then let the mission owner refresh the map and plan a detour.
                // Explicit gyro turns and MANUAL do not use this lookahead.
                motion_mode<=MODE_STOP;run_request<=0;
                target_speed_left<=0;target_speed_right<=0;status<=7;
                command_generation<=command_generation+1'b1;
            end
        end
    end
endmodule
