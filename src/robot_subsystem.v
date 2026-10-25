// Local FPGA-only commissioning: C1, odometry, mapping, conservative waypoint
// avoidance and LCD controls. The Orange Pi is not needed by this subsystem.
module robot_subsystem (
    input wire clk,reset_n,pixel_clk,safety_stop,remote_takeover,remote_active,
    input wire remote_turn_request,
    input wire task_valid,
    input wire [3:0] task_action,
    input wire signed [31:0] task_value,
    output wire task_ready,task_rsp_valid,
    output wire [7:0] task_rsp_status,
    output wire signed [31:0] task_rsp_value,
    input wire signed [15:0] remote_turn_degrees,
    output wire remote_turn_ready,
    output reg remote_turn_active,
    output reg turn_complete_valid,
    input wire turn_complete_ready,turn_report_busy,
    input wire [7:0] motor_fault_left,motor_fault_right,
    input wire motor_armed,motor_running,motor_selected_wheel,motor_edit_position,motor_mode_position,motor_link_fault,
    input wire encoder_invert_left,encoder_invert_right,drive_invert_left,drive_invert_right,
    input wire signed [15:0] duty_left,duty_right,
    input wire signed [15:0] motor_target_left,motor_target_right,
    input wire [15:0] speed_kp_left,speed_kp_right,speed_ki_left,speed_ki_right,speed_kd_left,speed_kd_right,
    input wire signed [31:0] motor_position_target_left,motor_position_target_right,
    input wire [15:0] position_kp_left,position_kp_right,position_ki_left,position_ki_right,position_kd_left,position_kd_right,
    input wire gains_ready,tick_10ms,encoder_zeroed,
    input wire signed [31:0] position_left,position_right,speed_left,speed_right,
    input wire signed [15:0] gyro_z,
    input wire gyro_valid,
    input wire [31:0] gyro_samples,
    input wire touch_ready,snapshot_valid,snapshot_cancel,
    input wire [1:0] contact_count,
    input wire [10:0] x0,y0,x1,y1,
    input wire [3:0] id0,
    input wire lidar_rx,
    output wire lidar_tx,
    output wire [2:0] page,subpage,
    output wire pid_action_valid,
    output wire [7:0] pid_action,
    output wire signed [31:0] pid_argument,
    output wire calibration_enable,
    output wire signed [15:0] calibration_duty_left,calibration_duty_right,
    output wire emergency_stop_pulse,clear_stop_pulse,
    output wire drive_enable,armed,
    output wire signed [15:0] target_left,target_right,
    output wire [2047:0] status_text,
    output wire [31:0] status_flags,
    output wire status_valid,
    output wire status_byte_valid,
    output wire [7:0] status_byte_data,status_byte_index,
    input wire status_byte_ready,
    output wire lidar_front_valid,
    output wire [15:0] lidar_front_mm,
    output wire [7:0] lidar_error,
    output wire [31:0] lidar_scans,
    input wire map_rd_en,
    input wire [6:0] map_rd_x,map_rd_y,
    output wire [1:0] map_rd_cell,
    output wire map_rd_valid,
    output wire [39:0] map_car_model,
    input wire follow_request,
    input wire [255:0] follow_data,
    output wire follow_ack,
    output wire [255:0] follow_telemetry,
    output wire [255:0] follow_laser_telemetry,
    output wire follow_pan_pwm,follow_tilt_pwm,
    input wire battery_a_low,
    output wire shutdown_requested,software_stopped
);
    wire ui_command_valid,pose_set,geometry_valid,gyro_invert,scan_enable,lidar_restart,clear_map;
    wire [2:0] ui_command;
    wire signed [15:0] manual_left,manual_right,offset_x,offset_y;
    wire manual_heading_hold;
    wire [15:0] manual_speed_left,manual_speed_right;
    wire manual_linked;
    wire [3:0] ui_notice;
    // One-clock notification for the UI/follow owners. The motor axis/bridge
    // and VOFA transaction guards still consume the RAW faults immediately.
    // This cuts the high-fanout fault -> owner -> wheel-setpoint data chain.
    reg ui_motor_fault;
    reg gains_ready_q;
    always @(posedge clk or negedge reset_n)begin
        if(!reset_n)begin ui_motor_fault<=0;gains_ready_q<=0;end
        else begin
            ui_motor_fault<=(motor_fault_left|motor_fault_right)!=0 || motor_link_fault;
            gains_ready_q<=gains_ready;
        end
    end
    wire explicit_local_stop=pid_action_valid && (pid_action==10 || pid_action==11);
    wire gyro_ready,lidar_ready,avoid_lidar_ready;
    wire signed [31:0] set_x,set_y,set_heading,goal_x,goal_y,turn_heading;
    wire [15:0] circumference,counts_per_rev,track,cruise,turn_speed;
    wire [15:0] body_front,body_rear,body_left,body_right,drive_margin,turn_margin,contact_margin;
    wire [14:0] forward_angle;
    wire [4:0] selected_field;
    wire [1:0] step_select;
    wire signed [31:0] selected_value;
    wire [31:0] selected_step;
    wire signed [31:0] robot_x,robot_y,robot_heading;
    wire [15:0] heading_q6;
    wire pose_valid,gyro_calibrated,calibrating;
    // Capability chooses the coordinate frame; transient stale/fast poses
    // HOLD an existing world map rather than turn it into a local map.
    wire map_local=!geometry_valid || !gyro_calibrated || calibrating;
    reg map_local_previous;
    wire map_mode_change=map_local_previous!=map_local;
    reg [1:0] map_pose_age;
    reg map_pose_seen,map_pose_slow;
    reg signed [31:0] map_pose_x,map_pose_y;
    reg [15:0] map_pose_heading;
    wire [3:0] nav_status;
    wire [3:0] mission_status;
    wire mission_command_valid,nav_permit;
    wire [2:0] mission_command;
    wire signed [31:0] mission_goal_x,mission_goal_y;
    wire [7:0] nav_fault;
    wire [1:0] recommended_side;
    wire nav_run,pose_update,correction_allowed,correction_ready,correction_ack,nav_turn_complete;
    wire signed [15:0] nav_target_left,nav_target_right;
    wire [31:0] correction_count;
    wire ui_armed,follow_armed,follow_permit,follow_command_valid;
    wire [2:0] follow_command;
    wire signed [15:0] follow_speed;
    wire follow_mission;
    wire signed [31:0] follow_goal_x,follow_goal_y,follow_heading;
    wire visual_query_valid,visual_laser_valid;
    wire [255:0] visual_query,visual_report,follow_report;
    wire [15:0] visual_target;
    wire signed [31:0] visual_bearing;
    wire [7:0] visual_status;
    wire signed [15:0] visual_x,visual_y;
    wire [7:0] visual_candidates,visual_points;
    // The TX mailbox owner alternates at CAPTURE time. A free-running mux at
    // 100Hz sampled at10Hz would alias and could suppress one packet forever.
    assign follow_telemetry=follow_report;
    assign follow_laser_telemetry=visual_report;
    wire ui_tap_valid;
    wire [7:0] ui_tap;
    wire [7:0] follow_gain;
    wire [2:0] task_visual_ready;
    wire [15:0] follow_cap,follow_selected,follow_candidate;
    wire [7:0] follow_kind,follow_confidence,follow_state,follow_diagnostic,follow_dog_score;
    wire body_guard_ready,body_front_stop,body_rear_stop,body_front_slow,body_turn_stop,virtual_contact;
    wire map_reset=clear_map || pose_set || encoder_zeroed || map_mode_change;
    wire mission_explicit_cancel=safety_stop || clear_map || pose_set || encoder_zeroed;
    assign armed=ui_armed || follow_armed;
    robot_ui_control u_controls(
        .clk(clk),.reset_n(reset_n),.safety_stop(safety_stop),.motor_fault(ui_motor_fault),.tick_10ms(tick_10ms),
        .remote_takeover(remote_takeover),.remote_active(remote_active),
        .touch_ready(touch_ready),.snapshot_valid(snapshot_valid),.snapshot_cancel(snapshot_cancel),
        .contact_count(contact_count),.x0(x0),.y0(y0),.x1(x1),.y1(y1),.id0(id0),
        .live_x_mm(robot_x),.live_y_mm(robot_y),.live_heading_mdeg(robot_heading),
        .motor_selected_wheel(motor_selected_wheel),.motor_edit_position(motor_edit_position),
        .gains_ready(gains_ready_q),.gyro_ready(gyro_ready),.lidar_ready(avoid_lidar_ready),
        .task_valid(task_valid),.task_action(task_action),.task_value(task_value),.task_visual_ready(task_visual_ready),
        .task_ready(task_ready),.task_rsp_valid(task_rsp_valid),.task_rsp_status(task_rsp_status),.task_rsp_value(task_rsp_value),
        .page(page),.subpage(subpage),.pid_action_valid(pid_action_valid),.pid_action(pid_action),.pid_argument(pid_argument),
        .emergency_stop_pulse(emergency_stop_pulse),.clear_stop_pulse(clear_stop_pulse),.armed(ui_armed),
        .tap_valid(ui_tap_valid),.tap_button(ui_tap),.follow_gain(follow_gain),.follow_cap(follow_cap),
        .command_valid(ui_command_valid),.command(ui_command),.manual_left(manual_left),.manual_right(manual_right),
        .manual_heading_hold(manual_heading_hold),.pose_set(pose_set),.pose_x_mm(set_x),.pose_y_mm(set_y),
        .manual_speed_left(manual_speed_left),.manual_speed_right(manual_speed_right),.manual_linked(manual_linked),
        .calibration_enable(calibration_enable),.calibration_duty_left(calibration_duty_left),
        .calibration_duty_right(calibration_duty_right),.ui_notice(ui_notice),
        .pose_heading_mdeg(set_heading),.goal_x_mm(goal_x),.goal_y_mm(goal_y),.turn_heading_mdeg(turn_heading),
        .geometry_valid(geometry_valid),.wheel_mm_per_rev(circumference),.encoder_counts_per_rev(counts_per_rev),
        .track_mm(track),.cruise_counts(cruise),.turn_counts(turn_speed),.gyro_invert(gyro_invert),
        .lidar_forward_q6(forward_angle),.lidar_offset_x_mm(offset_x),.lidar_offset_y_mm(offset_y),
        .body_front_mm(body_front),.body_rear_mm(body_rear),.body_left_mm(body_left),.body_right_mm(body_right),
        .drive_margin_mm(drive_margin),.turn_margin_mm(turn_margin),.contact_margin_mm(contact_margin),
        .scan_enable(scan_enable),.lidar_restart(lidar_restart),.map_clear(clear_map),
        .selected_field(selected_field),.step_select(step_select),.selected_value(selected_value),.selected_step(selected_step)
    );
    robot_follow_control #(.ENABLE_CHOOSER(0)) u_follow(
        .clk(clk),.reset_n(reset_n),.tick_10ms(tick_10ms),
        .request(follow_request),.data(follow_data),.ack(follow_ack),
        .page(page),.subpage(subpage),.tap_valid(ui_tap_valid),.tap_button(ui_tap),
        .stop_all(safety_stop || emergency_stop_pulse),
        .battery_a_low(battery_a_low),.shutdown_requested(shutdown_requested),.software_stopped(software_stopped),
        .cancel(remote_takeover || explicit_local_stop || ui_motor_fault || !touch_ready || (snapshot_valid && snapshot_cancel)),
        .motion_ready(gains_ready_q && geometry_valid && gyro_ready && !calibrating && avoid_lidar_ready),
        .gain(follow_gain),.speed_cap(follow_cap),.heading(robot_heading),
        .actual_left(speed_left[15:0]),.actual_right(speed_right[15:0]),.nav_status(mission_status),
        .turn_done(nav_turn_complete),.pose_x(robot_x),.pose_y(robot_y),
        .armed(follow_armed),.permit(follow_permit),.command_valid(follow_command_valid),
        .command(follow_command),.speed(follow_speed),.telemetry(follow_report),
        .mission_mode(follow_mission),.goal_x(follow_goal_x),.goal_y(follow_goal_y),.turn_heading(follow_heading),
        .query_valid(visual_query_valid),.query_data(visual_query),
        .laser_valid(visual_laser_valid),.laser_target(visual_target),.laser_bearing(visual_bearing),
        .turn_clear(body_guard_ready && avoid_lidar_ready && !body_turn_stop),
        .task_ready_flags(task_visual_ready),
        .pan_pwm(follow_pan_pwm),.tilt_pwm(follow_tilt_pwm),
        .selected_id(follow_selected),.candidate_id(follow_candidate),
        .candidate_kind(follow_kind),.candidate_confidence(follow_confidence),.state_code(follow_state),
        .diagnostic_code(follow_diagnostic),.dog_score(follow_dog_score)
    );
    wire point_valid,scan_start,lidar_running,scan_complete;
    wire [14:0] point_angle;
    wire [15:0] point_distance,raw_front,raw_left,raw_right,raw_rear;
    wire [5:0] point_quality;
    wire [3:0] sector_valid;
    wire [7:0] health;
    wire [15:0] health_error;
    wire [31:0] rx_bytes,node_count,point_count,uart_errors,packet_errors,restart_count;
    wire [31:0] lidar_debug_status,lidar_last_rx_word;
    wire [127:0] lidar_reply_capture;
    wire [31:0] lidar_last_tx_word;
    rplidar_c1 u_c1(
        .clk(clk),.reset_n(reset_n),.scan_enable(scan_enable),.restart(lidar_restart),
        .forward_angle_q6(forward_angle),.lidar_rx(lidar_rx),.lidar_tx(lidar_tx),
        .point_valid(point_valid),.angle_q6(point_angle),.distance_mm(point_distance),.quality(point_quality),
        .scan_start(scan_start),.running(lidar_running),.ready(lidar_ready),.error_code(lidar_error),
        .health_status(health),.health_error(health_error),.rx_bytes(rx_bytes),.node_count(node_count),
        .point_count(point_count),.scan_count(lidar_scans),.uart_errors(uart_errors),.packet_errors(packet_errors),
        .restart_count(restart_count),.front_mm(raw_front),.right_mm(raw_right),.back_mm(raw_rear),.left_mm(raw_left),
        .sector_valid(sector_valid),.scan_complete(scan_complete),
        .debug_status(lidar_debug_status),.last_rx_word(lidar_last_rx_word),
        .reply_capture(lidar_reply_capture),.last_tx_word(lidar_last_tx_word)
    );
    wire [15:0] front,right,left,rear;
    wire [15:0] filtered_front,filtered_right,filtered_rear,filtered_left;
    wire [15:0] body_self_points,rotation_radius;
    wire signed [15:0] blocking_x,blocking_y;
    wire [3:0] body_coverage;
    // Same SELF rejection as mapping; values remain optical-origin ranges,
    // NOT body edge clearances. Unknown/stale sectors display FFFF.
    assign front=body_guard_ready && body_coverage[0] ? filtered_front : 16'hffff;
    assign right=body_guard_ready && body_coverage[1] ? filtered_right : 16'hffff;
    assign left=body_guard_ready && body_coverage[2] ? filtered_left : 16'hffff;
    assign rear=body_guard_ready && body_coverage[3] ? filtered_rear : 16'hffff;
    wire map_point_valid;
    wire [14:0] map_point_angle;
    wire [15:0] map_point_distance;
    wire signed [15:0] body_point_x,body_point_y;
    lidar_body_guard u_body_guard(
        .clk(clk),.reset_n(reset_n),.tick_10ms(tick_10ms),.active(scan_enable && lidar_running),
        .point_valid(point_valid),.scan_start(scan_start),.angle_q6(point_angle),.distance_mm(point_distance),
        .forward_angle_q6(forward_angle),.offset_x_mm(offset_x),.offset_y_mm(offset_y),
        .body_front_mm(body_front),.body_rear_mm(body_rear),.body_left_mm(body_left),.body_right_mm(body_right),
        .drive_margin_mm(drive_margin),.turn_margin_mm(turn_margin),.contact_margin_mm(contact_margin),
        .ready(body_guard_ready),.front_stop(body_front_stop),.rear_stop(body_rear_stop),
        .front_slow(body_front_slow),.turn_stop(body_turn_stop),.virtual_contact(virtual_contact),
        .self_points(body_self_points),.rotation_radius_mm(rotation_radius),
        .blocking_x_mm(blocking_x),.blocking_y_mm(blocking_y),
        .coverage(body_coverage),
        .filtered_front_mm(filtered_front),.filtered_right_mm(filtered_right),
        .filtered_rear_mm(filtered_rear),.filtered_left_mm(filtered_left),
        .body_point_x(body_point_x),.body_point_y(body_point_y),
        .map_point_valid(map_point_valid),.map_point_angle(map_point_angle),.map_point_distance(map_point_distance)
    );
    // A valid byte/node stream alone cannot refresh an old obstacle scan.
    // C1 nominal10Hz revolutions get a300ms complete-scan allowance here;
    // the driver's independent sector/node watchdogs remain in force too.
    reg [5:0] complete_scan_age;
    always @(posedge clk or negedge reset_n)begin
        if(!reset_n)complete_scan_age<=63;
        else if(!scan_enable || !lidar_running)complete_scan_age<=63;
        else if(scan_complete)complete_scan_age<=0;
        else if(tick_10ms && complete_scan_age!=63)complete_scan_age<=complete_scan_age+1'b1;
    end
    assign avoid_lidar_ready=lidar_ready && complete_scan_age<30;
    // NAV replanning lookahead, not a larger manual collision clearance.
    // Evaluate current-heading nonSELF points in the actual drive corridor.
    // A400mm forward body gap leaves room for scan latency and the coarse
    // planner's inflated start cell. A point beyond the user's final stopping
    // envelope does not interrupt a shorter trip (e.g. wall beyond the goal).
    // A planned intermediate leg is already known-free. Clip that leg at its
    // own stopping envelope so the same distant person cannot cause a park /
    // replan loop before a valid short step then lateral detour. Direct/final
    // legs retain the complete user's goal envelope.
    wire signed [31:0] mission_dx=mission_goal_x-robot_x;
    wire signed [31:0] mission_dy=mission_goal_y-robot_y;
    wire [31:0] mission_abs_dx=mission_dx[31] ? -mission_dx : mission_dx;
    wire [31:0] mission_abs_dy=mission_dy[31] ? -mission_dy : mission_dy;
    wire [32:0] mission_forward_envelope={1'b0,mission_abs_dx}+{1'b0,mission_abs_dy}+
        {17'd0,body_front}+{17'd0,drive_margin};
    wire signed [17:0] nav_point_x={{2{body_point_x[15]}},body_point_x};
    wire signed [17:0] nav_point_y={{2{body_point_y[15]}},body_point_y};
    wire signed [17:0] nav_front=$signed({2'b00,body_front});
    wire signed [17:0] nav_left=$signed({2'b00,body_left})+$signed({2'b00,drive_margin});
    wire signed [17:0] nav_right=$signed({2'b00,body_right})+$signed({2'b00,drive_margin});
    wire nav_ahead_hit=map_point_valid && nav_point_x>=nav_front &&
        nav_point_x<=nav_front+18'sd400 && nav_point_y<=nav_left && nav_point_y>=-nav_right &&
        {17'd0,body_point_x}<=mission_forward_envelope;
    reg nav_ahead_work,nav_ahead_pending,nav_ahead_published;
    always @(posedge clk or negedge reset_n)begin
        if(!reset_n)begin nav_ahead_work<=0;nav_ahead_pending<=0;nav_ahead_published<=0;end
        else if(map_reset || !avoid_lidar_ready || !body_guard_ready || nav_status==4 ||
                (ui_command_valid && ui_command==3) || (follow_command_valid && follow_command==3) ||
                (mission_command_valid && mission_command==3))begin
            nav_ahead_work<=0;nav_ahead_pending<=0;nav_ahead_published<=0;
        end else begin
            if(scan_start)begin nav_ahead_pending<=nav_ahead_work;nav_ahead_work<=0;end
            else if(nav_ahead_hit)nav_ahead_work<=1;
            if(scan_complete)nav_ahead_published<=nav_ahead_pending;
        end
    end
    wire nav_obstacle_ahead=body_guard_ready && avoid_lidar_ready &&
        (nav_ahead_work || nav_ahead_published || nav_ahead_hit);
    robot_visual_laser u_visual_laser(
        .clk(clk),.reset_n(reset_n),.tick_10ms(tick_10ms),.clear(map_reset || page!=5),
        .scan_start(scan_start),.point_valid(map_point_valid),.point_x(body_point_x),.point_y(body_point_y),
        .pose_valid(pose_valid && gyro_ready),.laser_ready(avoid_lidar_ready),
        .speed_left(speed_left[15:0]),.speed_right(speed_right[15:0]),
        .pose_x(robot_x),.pose_y(robot_y),.heading(robot_heading),
        .query_valid(visual_query_valid),.query(visual_query),.telemetry(visual_report),
        .valid(visual_laser_valid),.target_id(visual_target),.bearing_world(visual_bearing),.result_status(visual_status),
        .body_x(visual_x),.body_y(visual_y),.cluster_count(visual_candidates),.beam_points(visual_points)
    );
    assign lidar_front_mm=front;
    assign lidar_front_valid=avoid_lidar_ready && body_guard_ready && body_coverage[0];
    reg [31:0] last_gyro_samples;
    reg [5:0] static_gyro_age;
    assign gyro_ready=gyro_calibrated && gyro_valid && static_gyro_age<20;
    always @(posedge clk or negedge reset_n)begin
        if(!reset_n)begin last_gyro_samples<=0;static_gyro_age<=63;end
        else begin
            last_gyro_samples<=gyro_samples;
            if(tick_10ms && static_gyro_age!=63)static_gyro_age<=static_gyro_age+1'b1;
            if(gyro_samples!=last_gyro_samples && gyro_valid)static_gyro_age<=0;
        end
    end
    wire gyro_sample=gyro_samples!=last_gyro_samples;
    wire map_ready,map_busy,correction_valid;
    wire signed [15:0] correction_x,correction_y;
    wire [7:0] match_score,match_baseline,match_margin;
    wire [31:0] map_points,map_dropped,map_oob,map_scans;
    wire [15:0] map_integrated_id;
    wire map_integrated_valid;
    wire map_integrated_pulse;
    wire planner_start,planner_cancel,planner_busy,planner_done,planner_reachable,freeze_map,map_frozen;
    wire [6:0] plan_current_x,plan_current_y,plan_goal_x,plan_goal_y,next_x,next_y;
    wire [7:0] planner_error;
    wire [12:0] path_length,expanded_nodes;
    wire plan_read_en,plan_read_valid;
    wire [6:0] plan_read_x,plan_read_y;
    wire [1:0] plan_read_cell;
    wire planning,no_path,mission_active;
    wire mission_waiting_map;
    wire mission_waiting_path;
    wire [7:0] mission_error;
    wire [7:0] effective_planner_error=mission_error!=0 ? mission_error : planner_error;
    wire [2:0] recovery_state;
    wire signed [15:0] recovery_speed;
    // correction_allowed already bounds translation to2mm/10ms (0.2m/s)
    // and gyro change to0.2deg/10ms (20deg/s). These limits pause WORLD
    // acquisition only; body-frame LOCAL LIVE remains available while moving.
    // Publishing predicted poses at100Hz does not make an old IMU sample
    // fresh. The60Hz gyro normally refreshes in16.7ms; require its separate
    // age below3 ticks for world writes, while NAV retains its200ms policy.
    wire world_map_pose_valid=pose_valid && correction_allowed && map_pose_seen && map_pose_slow && map_pose_age<2 && static_gyro_age<3;
    always @(posedge clk or negedge reset_n)begin
        if(!reset_n)begin
            map_local_previous<=1;map_pose_age<=3;map_pose_seen<=0;map_pose_slow<=0;
            map_pose_x<=0;map_pose_y<=0;map_pose_heading<=0;
        end else begin
            map_local_previous<=map_local;
            if(tick_10ms && map_pose_age!=3)map_pose_age<=map_pose_age+1'b1;
            if(map_reset || !pose_valid)begin map_pose_seen<=0;map_pose_age<=3;map_pose_slow<=0;end
            else if(pose_update)begin
                // The navigation publisher updates heading and position
                // before this pulse. Latch all three at ONE commit, then each
                // incoming lidar point captures that coherent 10ms sample.
                map_pose_x<=robot_x;map_pose_y<=robot_y;map_pose_heading<=heading_q6;
                map_pose_seen<=1;map_pose_slow<=correction_allowed;map_pose_age<=0;
            end
        end
    end
    wire [15:0] mission_arrival_tolerance;
    // A waypoint's local DONE is not the complete user mission. Keep LCD/Pi
    // in MOVING until the mission owner confirms its precise final XY.
    assign mission_status=mission_active && nav_status==6 ? 4'd5 : nav_status;
    wire manual_view=page==4 && subpage==0;
    wire manual_permit=armed && manual_view && touch_ready && !safety_stop && !ui_motor_fault;
    wire avoid_view=page==2 && subpage==3;
    wire local_permit=armed && ((page==4 && subpage==1) || avoid_view) && touch_ready && gains_ready_q && !safety_stop && !ui_motor_fault;
    reg manual_active,manual_turn_active;
    reg signed [15:0] manual_target_left,manual_target_right;
    reg remote_turn_start,remote_turn_abort;
    reg signed [31:0] remote_turn_heading;
    reg [1:0] remote_half_direction;
    wire remote_turn_permit=remote_turn_active && gyro_ready && !calibrating &&
        nav_fault==0 && !safety_stop && !ui_motor_fault && !explicit_local_stop && !map_reset && !remote_takeover;
    assign remote_turn_ready=gyro_ready && !calibrating && nav_fault==0 &&
        !safety_stop && !ui_motor_fault && !explicit_local_stop && !map_reset && !remote_takeover &&
        !nav_turn_complete && !turn_complete_valid && !turn_report_busy;
    // Relative integer degrees become one bounded heading add. Multiplication
    // by1000 is fixed shifts/adds; reuse the existing gyro TURN datapath.
    wire signed [31:0] remote_degrees_extended={{16{remote_turn_degrees[15]}},remote_turn_degrees};
    wire signed [31:0] remote_heading_sum=robot_heading+
        (remote_degrees_extended<<<10)-(remote_degrees_extended<<<4)-(remote_degrees_extended<<<3);
    wire signed [31:0] remote_heading_wrapped=remote_heading_sum>=180000 ? remote_heading_sum-360000 :
        remote_heading_sum< -180000 ? remote_heading_sum+360000 : remote_heading_sum;
    always @(posedge clk or negedge reset_n)begin
        if(!reset_n)begin
            remote_turn_active<=0;remote_turn_start<=0;remote_turn_abort<=0;
            remote_turn_heading<=0;turn_complete_valid<=0;remote_half_direction<=0;
        end else begin
            remote_turn_start<=0;remote_turn_abort<=0;
            if(turn_complete_valid && turn_complete_ready)turn_complete_valid<=0;
            if(nav_turn_complete && !safety_stop && !ui_motor_fault && !explicit_local_stop && !remote_takeover && !map_reset)begin
                turn_complete_valid<=1;remote_turn_active<=0;
            end
            if(remote_turn_request && remote_turn_ready && remote_turn_degrees!=0 &&
               remote_turn_degrees>=-180 && remote_turn_degrees<=180)begin
                remote_turn_heading<=remote_heading_wrapped;
                remote_half_direction<=remote_turn_degrees==180 ? 2'd1 : remote_turn_degrees== -180 ? 2'd2 : 2'd0;
                remote_turn_active<=1;remote_turn_start<=1;
            end
            // Every new RUN/STOP/direction takeover replaces an old TURN.
            // No stale turn may restart after a fault or subsequent LCD browse.
            if(remote_takeover || explicit_local_stop || safety_stop || ui_motor_fault || map_reset ||
               (remote_turn_active && (!gyro_ready || calibrating || nav_fault!=0)))begin
                remote_turn_active<=0;remote_turn_start<=0;remote_turn_abort<=1;
            end
        end
    end
    always @(posedge clk or negedge reset_n)begin
        if(!reset_n)begin manual_active<=0;manual_turn_active<=0;manual_target_left<=0;manual_target_right<=0;end
        else begin
            if(ui_command_valid && manual_view)begin
                manual_active<=ui_command==1 && manual_permit;
                manual_turn_active<=ui_command==2 && manual_permit;
                manual_target_left<=manual_left;manual_target_right<=manual_right;
            end
            if(!manual_permit || map_reset)begin
                manual_active<=0;manual_turn_active<=0;manual_target_left<=0;manual_target_right<=0;
            end
        end
    end
    robot_mission_control u_mission(
        .clk(clk),.reset_n(reset_n),.tick_10ms(tick_10ms),
        .permit((local_permit || (follow_permit && follow_mission)) && !map_mode_change),.safety_stop(mission_explicit_cancel),
        .user_command_valid(page==5 ? follow_command_valid : ui_command_valid),.user_command(page==5 ? follow_command : ui_command),
        .user_goal_x(page==5 ? follow_goal_x : goal_x),.user_goal_y(page==5 ? follow_goal_y : goal_y),
        .pose_x(robot_x),.pose_y(robot_y),.pose_valid(pose_valid),.nav_status(nav_status),
        .virtual_contact(virtual_contact),.rear_clear(body_guard_ready && body_coverage[3] && !body_rear_stop),.recovery_counts(cruise),
        .map_ready(map_ready),.map_busy(map_busy),.map_capture_ready(world_map_pose_valid && avoid_lidar_ready &&
            body_guard_ready && speed_left>=-1 && speed_left<=1 && speed_right>=-1 && speed_right<=1),
        .scan_complete(scan_complete),.map_scan_count(map_scans),
        .capture_boundary(scan_start),.scan_id(lidar_scans[15:0]),
        .map_integrated_id(map_integrated_id),.map_integrated_valid(map_integrated_valid),
        .map_integrated_pulse(map_integrated_pulse),
        .planner_busy(planner_busy),.planner_done(planner_done),.planner_reachable(planner_reachable),
        .planner_next_x(next_x),.planner_next_y(next_y),.planner_start(planner_start),.planner_cancel(planner_cancel),
        .planner_current_x(plan_current_x),.planner_current_y(plan_current_y),.planner_goal_x(plan_goal_x),.planner_goal_y(plan_goal_y),
        .nav_command_valid(mission_command_valid),.nav_command(mission_command),.nav_goal_x(mission_goal_x),.nav_goal_y(mission_goal_y),
        .nav_arrival_tolerance_mm(mission_arrival_tolerance),
        .motion_permit(nav_permit),.planning(planning),.no_path(no_path),.mission_active(mission_active),
        .waiting_map(mission_waiting_map),.waiting_path(mission_waiting_path),.mission_error(mission_error),
        .recovery_state(recovery_state),.recovery_speed(recovery_speed)
    );
    lidar_path_planner u_planner(
        .clk(clk),.reset_n(reset_n),.start(planner_start && !planner_cancel && !map_reset),
        .cancel(planner_cancel || map_reset),
        .current_x(plan_current_x),.current_y(plan_current_y),.goal_x(plan_goal_x),.goal_y(plan_goal_y),
        .heading_q6(heading_q6),
        .clearance_radius_mm(rotation_radius),
        .map_ready(map_ready),.map_frozen(map_frozen),.freeze_map(freeze_map),.busy(planner_busy),.done(planner_done),
        .reachable(planner_reachable),.error_code(planner_error),.next_x(next_x),.next_y(next_y),
        .path_length(path_length),.expanded_nodes(expanded_nodes),.plan_rd_en(plan_read_en),
        .plan_rd_x(plan_read_x),.plan_rd_y(plan_read_y),.plan_rd_cell(plan_read_cell),.plan_rd_valid(plan_read_valid)
    );
    robot_navigation #(.BODY_GUARD(1)) u_navigation(
        .clk(clk),.reset_n(reset_n),.tick_10ms(tick_10ms),.safety_stop(safety_stop),
        .motion_enable(remote_turn_permit || (page==5 ? (follow_mission ? nav_permit : follow_permit) : manual_view ? manual_permit : nav_permit)),
        .encoder_position_left(position_left),.encoder_position_right(position_right),.encoder_zeroed(encoder_zeroed),
        .gyro_z(gyro_z),.gyro_sample(gyro_sample),.gyro_valid(gyro_valid),.gyro_invert(gyro_invert),
        .geometry_valid(geometry_valid),.wheel_mm_per_rev(circumference),.encoder_counts_per_rev(counts_per_rev),.track_mm(track),
        .pose_set(pose_set),.pose_x_mm(set_x),.pose_y_mm(set_y),.pose_heading_mdeg(set_heading),
        .command_valid(remote_turn_abort || remote_turn_start || (!remote_turn_active && (page==5 && !follow_mission ? follow_command_valid : manual_view ? ui_command_valid : mission_command_valid))),
        .command(remote_turn_abort ? 3'd0 : remote_turn_start ? 3'd2 : page==5 && !follow_mission ? follow_command : manual_view ? (ui_command==2 ? 3'd2 : ui_command==4 ? 3'd4 : 3'd0) : mission_command),
        .manual_left(page==5 ? follow_speed : recovery_state==2 ? recovery_speed : manual_left),
        .manual_right(page==5 ? follow_speed : recovery_state==2 ? recovery_speed : manual_right),
        .manual_heading_hold(page==5 ? 1'b1 : recovery_state==2 ? 1'b0 : manual_heading_hold),.turn_heading_mdeg(remote_turn_active ? remote_turn_heading : page==5 ? follow_heading : turn_heading),.goal_x_mm(mission_goal_x),.goal_y_mm(mission_goal_y),
        .turn_half_direction(remote_turn_active ? remote_half_direction : 2'd0),
        .cruise_counts(cruise),.turn_counts(turn_speed),.arrival_tolerance_mm(mission_arrival_tolerance),
        .avoid_enable(!manual_view && !remote_turn_active),.lidar_valid(avoid_lidar_ready),.clear_front_mm(front),.clear_left_mm(left),.clear_right_mm(right),.clear_rear_mm(rear),
        // Legacy radial thresholds are unused with BODY_GUARD=1. Actual gaps
        // come from the runtime body-frame envelope, not a550mm sensor radius.
        .stop_clearance_mm(16'd0),.slow_clearance_mm(16'd0),.rotate_clearance_mm(16'd0),
        .body_guard_ready(body_guard_ready),.body_front_stop(body_front_stop),.body_rear_stop(body_rear_stop),
        .body_front_slow(body_front_slow),.body_turn_stop(body_turn_stop),
        .body_nav_ahead(nav_obstacle_ahead),
        .body_coverage(body_coverage),
        .lidar_correction_valid(correction_valid && !manual_active && !manual_turn_active && !remote_turn_active && !clear_map && !pose_set && !encoder_zeroed),
        .lidar_correction_x_mm(correction_x),.lidar_correction_y_mm(correction_y),
        .run_request(nav_run),.target_speed_left(nav_target_left),.target_speed_right(nav_target_right),
        .x_mm(robot_x),.y_mm(robot_y),.heading_mdeg(robot_heading),.heading_q6(heading_q6),.pose_valid(pose_valid),
        .gyro_calibrated(gyro_calibrated),.calibrating(calibrating),.status(nav_status),.fault(nav_fault),.pose_update(pose_update),.turn_complete_pulse(nav_turn_complete),
        .recommended_side(recommended_side),.correction_allowed(correction_allowed),.correction_ready(correction_ready),
        .correction_accepted(correction_ack),.correction_count(correction_count)
    );
    assign target_left=!remote_turn_active && manual_view && manual_active ? manual_target_left : nav_target_left;
    assign target_right=!remote_turn_active && manual_view && manual_active ? manual_target_right : nav_target_right;
    assign drive_enable=(remote_turn_permit && nav_run) || (manual_permit && (manual_active || (manual_turn_active && nav_run))) ||
        (local_permit && nav_permit && nav_run) || (follow_permit && nav_run);
    // Local scans replace the previous revolution even while the vehicle is
    // moving. World scans accumulate only after geometry/gyro calibration;
    // stale or excessively fast poses discard acquisition, not the old map.
    lidar_grid_map u_map(
        .clk(clk),.reset_n(reset_n),.enable(scan_enable),.clear_map(map_reset),
        .point_valid(map_point_valid),.scan_start(scan_start),.scan_id(lidar_scans[15:0]),
        .integrated_scan_id(map_integrated_id),.integrated_scan_valid(map_integrated_valid),
        .integrated_scan_pulse(map_integrated_pulse),
        .angle_q6({1'b0,map_point_angle}),.distance_mm(map_point_distance),.quality(6'd1),
        .pose_valid(world_map_pose_valid),.local_scan(map_local),.pose_x_mm(map_pose_x),
        .pose_y_mm(map_pose_y),.pose_heading_q6(map_pose_heading),
        .forward_angle_q6(forward_angle),.lidar_offset_x_mm(offset_x),.lidar_offset_y_mm(offset_y),
        .body_front_mm(body_front),.body_rear_mm(body_rear),.body_left_mm(body_left),.body_right_mm(body_right),
        .correction_enable(!map_local && world_map_pose_valid && !manual_active && !manual_turn_active && !remote_turn_active && !map_reset),.correction_ack(correction_ack),
        .freeze_map(freeze_map),.map_frozen(map_frozen),.plan_rd_en(plan_read_en),.plan_rd_x(plan_read_x),.plan_rd_y(plan_read_y),
        .plan_rd_cell(plan_read_cell),.plan_rd_valid(plan_read_valid),
        .map_ready(map_ready),.busy(map_busy),.point_count(map_points),.dropped_points(map_dropped),.out_of_bounds(map_oob),.scan_count(map_scans),
        .correction_valid(correction_valid),.correction_x_mm(correction_x),.correction_y_mm(correction_y),
        .match_score(match_score),.match_baseline(match_baseline),.match_margin(match_margin),
        .rd_clk(pixel_clk),.rd_en(map_rd_en),.rd_x(map_rd_x),.rd_y(map_rd_y),.rd_cell(map_rd_cell),.rd_valid(map_rd_valid)
    );
    robot_map_car_pose u_car_pose(
        .clk(clk),.reset_n(reset_n),.pixel_clk(pixel_clk),.pixel_idle(!map_rd_en),
        .tick_10ms(tick_10ms),.pose_update(pose_update),.local_scan(map_local),.pose_valid(pose_valid),
        .x_mm(robot_x),.y_mm(robot_y),.heading_q6(heading_q6),
        .body_front_mm(body_front),.body_rear_mm(body_rear),.body_left_mm(body_left),.body_right_mm(body_right),
        .pixel_model(map_car_model)
    );
    // LCD and UART consume the indexed byte stream in this subsystem.
    // Explicitly omit the unused parallel compatibility frame registers.
    robot_status_text #(.PARALLEL_COMPAT(0)) u_status(
        .follow_selected(follow_selected),.follow_candidate(follow_candidate),.follow_kind(follow_kind),
        .follow_confidence(follow_confidence),.follow_state(follow_state),.follow_speed(follow_speed),
        .follow_gain(follow_gain),.follow_cap(follow_cap),
        .follow_diagnostic(follow_diagnostic),.follow_dog_score(follow_dog_score),
        .follow_pan_us(follow_report[111:96]),.follow_tilt_us(follow_report[95:80]),.visual_x(visual_x),.visual_y(visual_y),
        .planner_error(effective_planner_error),.waiting_map(mission_waiting_map),.waiting_path(mission_waiting_path),
        .clk(clk),.reset_n(reset_n),.page(page),.subpage(subpage),.armed(armed),.geometry_valid(geometry_valid),
        .motor_armed(motor_armed),.motor_running(motor_running),.motor_selected_wheel(motor_selected_wheel),
        .motor_mode_position(motor_mode_position),
        .motor_target_left(motor_target_left),.motor_target_right(motor_target_right),
        .speed_kp_left(speed_kp_left),.speed_kp_right(speed_kp_right),
        .speed_ki_left(speed_ki_left),.speed_ki_right(speed_ki_right),
        .speed_kd_left(speed_kd_left),.speed_kd_right(speed_kd_right),
        .motor_position_target_left(motor_position_target_left),.motor_position_target_right(motor_position_target_right),
        .position_kp_left(position_kp_left),.position_kp_right(position_kp_right),
        .position_ki_left(position_ki_left),.position_ki_right(position_ki_right),
        .position_kd_left(position_kd_left),.position_kd_right(position_kd_right),
        .encoder_invert_left(encoder_invert_left),.encoder_invert_right(encoder_invert_right),
        .drive_invert_left(drive_invert_left),.drive_invert_right(drive_invert_right),
        .calibration_enable(calibration_enable),.manual_active(manual_active),.manual_turn_active(manual_turn_active),
        .manual_linked(manual_linked),.ui_notice(ui_notice),.virtual_contact(virtual_contact),.recovery_state(recovery_state),
        .blocking_x_mm(blocking_x),.blocking_y_mm(blocking_y),
        .body_self_points(body_self_points),.motor_fault(motor_fault_left|motor_fault_right|{7'd0,motor_link_fault}),
        .speed_left(speed_left),.speed_right(speed_right),.position_left(position_left),.position_right(position_right),
        .duty_left(duty_left),.duty_right(duty_right),.gyro_raw(gyro_z),
        .manual_speed_left(manual_speed_left),.manual_speed_right(manual_speed_right),
        .wheel_mm_per_rev(circumference),.encoder_counts_per_rev(counts_per_rev),.track_mm(track),
        .cruise_counts(cruise),.turn_counts(turn_speed),
        .set_x_mm(set_x),.set_y_mm(set_y),.set_heading_mdeg(set_heading),
        .rx_bytes(rx_bytes),.node_count(node_count),.uart_errors(uart_errors),.packet_errors(packet_errors),
        .restart_count(restart_count),.map_points(map_points),.map_scans(map_scans),.map_dropped(map_dropped),
        .lidar_debug_status(lidar_debug_status),.lidar_last_rx_word(lidar_last_rx_word),
        .lidar_reply_capture(lidar_reply_capture),.lidar_last_tx_word(lidar_last_tx_word),
        .lidar_zero_deg(forward_angle[14:6]),.lidar_offset_x_mm(offset_x),.lidar_offset_y_mm(offset_y),.gyro_invert(gyro_invert),
        .gyro_calibrated(gyro_calibrated),.calibrating(calibrating),.lidar_ready(lidar_ready),.scan_enable(scan_enable),
        .safety_stop(safety_stop),.map_ready(map_ready),.gains_ready(gains_ready_q),
        .map_local(map_local),.map_world_ready(world_map_pose_valid),
        .nav_status(planning ? 4'd10 : no_path ? 4'd11 : mission_status),
        .selected_field(selected_field),.nav_fault(nav_fault),.lidar_error(lidar_error),
        .selected_value(selected_value),.selected_step(selected_step),.x_mm(robot_x),.y_mm(robot_y),.heading_mdeg(robot_heading),
        .goal_x_mm(goal_x),.goal_y_mm(goal_y),.lidar_points(point_count),.front_mm(front),.left_mm(left),.right_mm(right),.rear_mm(rear),
        .match_score(match_score),.match_baseline(match_baseline),.match_margin(match_margin),
        .text_rows(status_text),.flags(status_flags),.text_valid(status_valid),
        .text_byte_valid(status_byte_valid),.text_byte_data(status_byte_data),.text_byte_index(status_byte_index),
        .text_byte_ready(status_byte_ready)
    );
endmodule

// Camera direction gates compact, contiguous returns in AXLE coordinates.
// No image/PWM-derived depth. A result contains the pose frozen at scan start;
// Linux transforms it to a fixed world target and proposes a standoff goal.
// Proposal packets alone NEVER start motion. All measurements are millimetres.
module robot_visual_laser(
    input wire clk,reset_n,tick_10ms,clear,scan_start,point_valid,pose_valid,laser_ready,
    input wire signed [15:0] point_x,point_y,speed_left,speed_right,
    input wire signed [31:0] pose_x,pose_y,heading,
    input wire query_valid,
    input wire [255:0] query,
    output wire [255:0] telemetry,
    output wire valid,
    output reg [15:0] target_id,
    output reg signed [31:0] bearing_world,
    output reg [7:0] result_status,
    output wire signed [15:0] body_x,body_y,
    output wire [7:0] cluster_count,beam_points
);
    reg [255:0] live_query,scan_query;
    reg [7:0] query_age,result_age;
    reg seen,scan_ok;
    reg signed [31:0] sx,sy,sh,result_x,result_y,result_heading;
    reg signed [15:0] dx,dy,cx,cy,previous_x,previous_y;
    reg [15:0] uncertainty,scan_sequence;
    reg signed [15:0] last_x,last_y,min_x,max_x,min_y,max_y;
    reg [7:0] count,candidates,chosen_points;
    reg cluster_in_cone;
    reg signed [15:0] chosen_min_x,chosen_max_x,chosen_min_y,chosen_max_y;
    reg signed [15:0] result_bx,result_by,prior_bx,prior_by;
    reg [15:0] prior_id;
    reg prior_good;
    // Scan finalization has two registered service stages after the boundary:
    // freeze the completed cluster first, then form its midpoint/extents,
    // then commit stability/status. No old-to-new scan combinational chain.
    reg [1:0] publish_state;
    reg [7:0] pending_candidates,pending_points;
    reg signed [15:0] pending_min_x,pending_max_x,pending_min_y,pending_max_y;
    reg signed [15:0] publish_bx,publish_by,pending_prior_bx,pending_prior_by;
    reg [15:0] pending_id,pending_prior_id;
    reg pending_seen,pending_query_good,pending_scan_good,pending_prior_good,publish_compact;
    reg signed [31:0] pending_pose_x,pending_pose_y,pending_heading,pending_bearing;
    reg [1:0] atan_state;
    reg [3:0] atan_index,atan_shifts;
    // Full signed16-mm X/Y, scaled by256, remain below2^25 after
    // sqrt(2)*CORDICgain<2.33 expansion. 26 signed bits preserve every old
    // vectoring bit while shortening four shift/register/add networks.
    reg signed [25:0] atan_x,atan_y,atan_sx,atan_sy;
    reg signed [31:0] atan_angle,atan_heading;
    function signed [31:0] atan_step;
        input [3:0] i;
        begin case(i)
            0:atan_step=45000;1:atan_step=26565;2:atan_step=14036;3:atan_step=7125;
            4:atan_step=3576;5:atan_step=1790;6:atan_step=895;7:atan_step=448;
            8:atan_step=224;9:atan_step=112;10:atan_step=56;11:atan_step=28;
            12:atan_step=14;13:atan_step=7;14:atan_step=3;default:atan_step=2;
        endcase end
    endfunction
    wire signed [31:0] atan_world=atan_heading+atan_angle;
    reg [7:0] result_candidates,result_points;
    assign body_x=result_bx;assign body_y=result_by;
    assign cluster_count=result_candidates;assign beam_points=result_points;
    // C1 points are thousands of50MHz clocks apart. Four dot/cross products
    // share one explicit17x16 multiplier over a six-clock point service.
    reg [2:0] point_phase;
    reg signed [16:0] relative_x,relative_y;
    reg signed [32:0] point_partial;
    wire pipeline=point_phase==5;
    wire signed [16:0] point_operand_a=(point_phase==2 || point_phase==4) ? relative_y : relative_x;
    wire signed [15:0] point_operand_b=(point_phase==2 || point_phase==3) ? dy : dx;
    wire signed [32:0] point_product=point_operand_a*point_operand_b;
    reg signed [32:0] dot_product,cross_product;
    reg signed [15:0] held_x,held_y;
    wire stationary=speed_left>=-2 && speed_left<=2 && speed_right>=-2 && speed_right<=2;
    wire query_good=query_age<35 && live_query[208] && live_query[191:176]!=0 && live_query[175:160]<=500;
    wire signed [31:0] yaw_change=heading-sh;
    wire same_pose=stationary && pose_valid && pose_x-sx<20 && sx-pose_x<20 && pose_y-sy<20 && sy-pose_y<20 &&
        yaw_change<2000 && yaw_change> -2000;
    wire signed [31:0] bearing_change=$signed(live_query[79:64])-$signed(scan_query[79:64]);
    wire same_query=query_good && live_query[191:176]==scan_query[191:176] && bearing_change<200 && bearing_change> -200;
    wire [32:0] cross_abs=cross_product[32] ? -cross_product : cross_product;
    // tan(7.125deg)=1/8: fixed shift, no wide variable multiplier.
    wire in_cone=dot_product>33'sd819200 && cross_abs<($unsigned(dot_product) >> 3)+({17'd0,uncertainty}<<14);
    wire consecutive=count!=0 && held_x-last_x<=100 && last_x-held_x<=100 && held_y-last_y<=100 && last_y-held_y<=100;
    wire compact=count>=3 && cluster_in_cone && max_x-min_x<=400 && max_y-min_y<=400;
    wire touches=candidates!=0 && min_x<=chosen_max_x+100 && max_x>=chosen_min_x-100 &&
        min_y<=chosen_max_y+100 && max_y>=chosen_min_y-100;
    // Finalize a cluster at a gap or end of scan. First/last clusters at the
    // angular wrap are merged only when their Cartesian boxes touch.
    reg [7:0] final_candidates,final_points;
    reg signed [15:0] final_min_x,final_max_x,final_min_y,final_max_y;
    always @* begin
        final_candidates=candidates;final_points=chosen_points;
        final_min_x=chosen_min_x;final_max_x=chosen_max_x;final_min_y=chosen_min_y;final_max_y=chosen_max_y;
        if(compact)begin
            if(candidates==0)begin
                final_candidates=1;final_points=count;
                final_min_x=min_x;final_max_x=max_x;final_min_y=min_y;final_max_y=max_y;
            end else if(touches)begin
                final_points=chosen_points>255-count ? 8'd255 : chosen_points+count;
                if(min_x<chosen_min_x)final_min_x=min_x;if(max_x>chosen_max_x)final_max_x=max_x;
                if(min_y<chosen_min_y)final_min_y=min_y;if(max_y>chosen_max_y)final_max_y=max_y;
            end else if(candidates!=255)final_candidates=candidates+1'b1;
        end
        // A wall crossing the cone is still a wall: cluster the COMPLETE
        // physical run before checking direction. Never cut a wall at the
        // cone boundary and then call its central fragment a compact object.
        if(count>=3 && touches && !compact)final_candidates=2;
    end
    wire signed [16:0] midpoint_x=($signed({pending_min_x[15],pending_min_x})+$signed({pending_max_x[15],pending_max_x}))>>>1;
    wire signed [16:0] midpoint_y=($signed({pending_min_y[15],pending_min_y})+$signed({pending_max_y[15],pending_max_y}))>>>1;
    wire signed [16:0] pending_width=$signed({pending_max_x[15],pending_max_x})-$signed({pending_min_x[15],pending_min_x});
    wire signed [16:0] pending_height=$signed({pending_max_y[15],pending_max_y})-$signed({pending_min_y[15],pending_min_y});
    wire unique_result=pending_candidates==1 && publish_compact && pending_scan_good;
    wire signed [16:0] stable_dx=$signed({publish_bx[15],publish_bx})-$signed({pending_prior_bx[15],pending_prior_bx});
    wire signed [16:0] stable_dy=$signed({publish_by[15],publish_by})-$signed({pending_prior_by[15],pending_prior_by});
    wire stable=pending_prior_good && pending_prior_id==pending_id &&
        stable_dx<17'sd128 && stable_dx> -17'sd128 && stable_dy<17'sd128 && stable_dy> -17'sd128;
    wire signed [25:0] publish_scaled_x=$signed({{10{publish_bx[15]}},publish_bx})<<<8;
    wire signed [25:0] publish_scaled_y=$signed({{10{publish_by[15]}},publish_by})<<<8;
    wire signed [31:0] bearing_sum=sh+$signed(scan_query[79:64])*32'sd10;
    assign valid=result_status==4 && publish_state==0 && atan_state==0 && result_age<50 && query_good && target_id==live_query[191:176];
    wire [15:0] age_ms={8'd0,result_age}*16'd10;
    wire [247:0] report={32'h4c445231,8'd1,{7'd0,valid},scan_sequence,
        result_x,result_y,result_heading,result_bx,result_by,target_id,result_candidates,result_points,age_ms,result_status};
    function [7:0] checksum;
        input [247:0] bytes;integer i;
        begin checksum=0;for(i=0;i<31;i=i+1)checksum=checksum^bytes[i*8+:8];end
    endfunction
    assign telemetry={report,checksum(report)};
    always @(posedge clk or negedge reset_n)begin
        if(!reset_n)begin
            live_query<=0;scan_query<=0;query_age<=255;result_age<=255;seen<=0;scan_ok<=0;
            sx<=0;sy<=0;sh<=0;result_x<=0;result_y<=0;result_heading<=0;dx<=0;dy<=0;cx<=0;cy<=0;
            uncertainty<=0;scan_sequence<=0;last_x<=0;last_y<=0;min_x<=0;max_x<=0;min_y<=0;max_y<=0;
            count<=0;cluster_in_cone<=0;candidates<=0;chosen_points<=0;chosen_min_x<=0;chosen_max_x<=0;chosen_min_y<=0;chosen_max_y<=0;
            result_bx<=0;result_by<=0;prior_bx<=0;prior_by<=0;prior_id<=0;prior_good<=0;
            result_candidates<=0;result_points<=0;point_phase<=0;relative_x<=0;relative_y<=0;point_partial<=0;dot_product<=0;cross_product<=0;
            held_x<=0;held_y<=0;previous_x<=0;previous_y<=0;target_id<=0;bearing_world<=0;result_status<=0;
            atan_state<=0;atan_index<=0;atan_shifts<=0;atan_x<=0;atan_y<=0;atan_sx<=0;atan_sy<=0;atan_angle<=0;atan_heading<=0;
            publish_state<=0;pending_candidates<=0;pending_points<=0;
            pending_min_x<=0;pending_max_x<=0;pending_min_y<=0;pending_max_y<=0;
            publish_bx<=0;publish_by<=0;pending_prior_bx<=0;pending_prior_by<=0;
            pending_id<=0;pending_prior_id<=0;pending_seen<=0;pending_query_good<=0;pending_scan_good<=0;pending_prior_good<=0;publish_compact<=0;
            pending_pose_x<=0;pending_pose_y<=0;pending_heading<=0;pending_bearing<=0;
        end else begin
            // Serial vectoring CORDIC of the actual LASER cluster (not camera
            // yaw). Shift one bit per clock to avoid a32-bit barrel shifter.
            case(atan_state)
                1:begin
                    atan_sx<=atan_x;atan_sy<=atan_y;atan_shifts<=atan_index;atan_state<=2;
                end
                2:if(atan_shifts!=0)begin
                    atan_sx<=atan_sx>>>1;atan_sy<=atan_sy>>>1;atan_shifts<=atan_shifts-1'b1;
                end else begin
                    atan_x<=atan_y[25] ? atan_x-atan_sy : atan_x+atan_sy;
                    atan_y<=atan_y[25] ? atan_y+atan_sx : atan_y-atan_sx;
                    atan_angle<=atan_y[25] ? atan_angle-atan_step(atan_index) : atan_angle+atan_step(atan_index);
                    if(atan_index==15)atan_state<=3;
                    else begin atan_index<=atan_index+1'b1;atan_state<=1;end
                end
                3:begin
                    bearing_world<=atan_world>=180000 ? atan_world-360000 : atan_world< -180000 ? atan_world+360000 : atan_world;
                    atan_state<=0;
                end
                default:begin end
            endcase
            if(tick_10ms)begin
                if(query_age!=255)query_age<=query_age+1'b1;
                if(result_age!=255)result_age<=result_age+1'b1;
            end
            if(query_valid)begin live_query<=query;query_age<=0;end
            if(publish_state==1)begin
                publish_bx<=midpoint_x[15:0];publish_by<=midpoint_y[15:0];
                publish_compact<=(pending_width<=17'sd400 && pending_height<=17'sd400);
                publish_state<=2;
            end else if(publish_state==2)begin
                // Every field below belongs to the SAME frozen past scan,
                // even if a query/pose for the next scan has already changed.
                scan_sequence<=scan_sequence+1'b1;result_age<=0;
                result_x<=pending_pose_x;result_y<=pending_pose_y;result_heading<=pending_heading;target_id<=pending_id;
                result_candidates<=pending_candidates;result_points<=pending_points;
                result_bx<=publish_bx;result_by<=publish_by;
                result_status<=!pending_seen || !pending_query_good ? 0 : !pending_scan_good ? 5 :
                    pending_candidates==0 ? 1 : pending_candidates>1 || !publish_compact ? 2 : stable ? 4 : 3;
                if(!unique_result)prior_good<=0;
                else begin prior_good<=1;prior_id<=pending_id;prior_bx<=publish_bx;prior_by<=publish_by;end
                bearing_world<=pending_bearing>=180000 ? pending_bearing-360000 : pending_bearing< -180000 ? pending_bearing+360000 : pending_bearing;
                if(unique_result)begin
                    atan_x<=publish_bx[15] ? -publish_scaled_x : publish_scaled_x;
                    atan_y<=publish_bx[15] ? -publish_scaled_y : publish_scaled_y;
                    atan_angle<=publish_bx[15] ? (publish_by[15] ? -180000 : 180000) : 0;
                    atan_heading<=pending_heading;atan_index<=0;atan_state<=1;
                end
                publish_state<=0;
            end
            case(point_phase)
                1:begin point_partial<=point_product;point_phase<=2;end
                2:begin dot_product<=point_partial+point_product;point_phase<=3;end
                3:begin point_partial<=point_product;point_phase<=4;end
                4:begin cross_product<=point_partial-point_product;point_phase<=5;end
                5:point_phase<=0;
                default:begin end
            endcase
            if(point_valid && point_phase==0)begin
                held_x<=point_x;held_y<=point_y;
                relative_x<=$signed({point_x[15],point_x})-$signed({cx[15],cx});
                relative_y<=$signed({point_y[15],point_y})-$signed({cy[15],cy});
                point_phase<=1;
            end
            // Never silently omit an overloaded point and publish a partial
            // cluster as unique. Normal460800-baud C1 cannot hit this path.
            if(point_valid && point_phase!=0)scan_ok<=0;
            if(!same_pose || !same_query)scan_ok<=0;
            if(pipeline)begin
                if(!consecutive)begin
                    candidates<=final_candidates;chosen_points<=final_points;
                    chosen_min_x<=final_min_x;chosen_max_x<=final_max_x;chosen_min_y<=final_min_y;chosen_max_y<=final_max_y;
                    count<=1;cluster_in_cone<=in_cone;min_x<=held_x;max_x<=held_x;min_y<=held_y;max_y<=held_y;
                end else begin
                    if(in_cone)cluster_in_cone<=1;
                    if(count!=255)count<=count+1'b1;
                    if(held_x<min_x)min_x<=held_x;if(held_x>max_x)max_x<=held_x;
                    if(held_y<min_y)min_y<=held_y;if(held_y>max_y)max_y<=held_y;
                end
                last_x<=held_x;last_y<=held_y;
            end
            if(scan_start)begin
                pending_candidates<=final_candidates;pending_points<=final_points;
                pending_min_x<=final_min_x;pending_max_x<=final_max_x;pending_min_y<=final_min_y;pending_max_y<=final_max_y;
                pending_seen<=seen;pending_query_good<=query_good;
                pending_scan_good<=scan_ok && same_pose && same_query && laser_ready && point_phase==0;
                pending_pose_x<=sx;pending_pose_y<=sy;pending_heading<=sh;pending_bearing<=bearing_sum;
                pending_id<=scan_query[191:176];pending_prior_id<=prior_id;
                pending_prior_good<=prior_good;pending_prior_bx<=prior_bx;pending_prior_by<=prior_by;
                publish_state<=1;
                seen<=1;scan_ok<=query_good && pose_valid && stationary && laser_ready;
                scan_query<=live_query;sx<=pose_x;sy<=pose_y;sh<=heading;
                dx<=live_query[159:144];dy<=live_query[143:128];cx<=live_query[127:112];cy<=live_query[111:96];
                uncertainty<=live_query[63:48];count<=0;cluster_in_cone<=0;candidates<=0;chosen_points<=0;point_phase<=0;
            end
            if(clear || !pose_valid || !laser_ready)begin
                result_status<=0;prior_good<=0;seen<=0;scan_ok<=0;count<=0;cluster_in_cone<=0;candidates<=0;point_phase<=0;atan_state<=0;publish_state<=0;
            end
        end
    end
endmodule

// Small display-only pose mailbox. The source word remains unchanged until
// a pixel-domain ACK has crossed back; no asynchronous live pose bus is drawn.
// Eight heading sectors give a simple oriented rectangle without pixel-rate
// multipliers, another framebuffer or changes to the occupancy map.
module robot_map_car_pose(
    input wire clk,reset_n,pixel_clk,pixel_idle,tick_10ms,pose_update,local_scan,pose_valid,
    input wire signed [31:0] x_mm,y_mm,
    input wire [15:0] heading_q6,
    input wire [15:0] body_front_mm,body_rear_mm,body_left_mm,body_right_mm,
    output reg [39:0] pixel_model
);
    reg [39:0] held_model;
    reg request,ack;
    (* async_reg="true" *) reg [2:0] request_sync,ack_sync;
    reg [2:0] direction;
    always @* begin
        if(heading_q6<1440 || heading_q6>=21600)direction=0;
        else if(heading_q6<4320)direction=1;
        else if(heading_q6<7200)direction=2;
        else if(heading_q6<10080)direction=3;
        else if(heading_q6<12960)direction=4;
        else if(heading_q6<15840)direction=5;
        else if(heading_q6<18720)direction=6;
        else direction=7;
    end
    wire signed [31:0] grid_x=(x_mm+32'sd4096)>>>6,grid_y=(y_mm+32'sd4096)>>>6;
    wire coordinates_inside=x_mm>=-4096 && x_mm<4096 && y_mm>=-4096 && y_mm<4096;
    wire [16:0] front_rounded={1'b0,body_front_mm}+17'd63;
    wire [16:0] rear_rounded={1'b0,body_rear_mm}+17'd63;
    wire [16:0] left_rounded={1'b0,body_left_mm}+17'd63;
    wire [16:0] right_rounded={1'b0,body_right_mm}+17'd63;
    always @(posedge clk or negedge reset_n)begin
        if(!reset_n)begin held_model<=0;request<=0;ack_sync<=0;end
        else begin
            ack_sync<={ack_sync[1:0],ack};
            if(request==ack_sync[2] && (local_scan ? tick_10ms : pose_update && pose_valid))begin
                held_model<={local_scan || coordinates_inside,
                    local_scan ? 3'd0 : direction,
                    front_rounded[10:6],rear_rounded[10:6],left_rounded[10:6],right_rounded[10:6],
                    local_scan ? 7'd64 : grid_x[6:0],local_scan ? 7'd64 : grid_y[6:0],2'b00};
                request<=!request;
            end
        end
    end
    always @(posedge pixel_clk or negedge reset_n)begin
        if(!reset_n)begin request_sync<=0;ack<=0;pixel_model<=0;end
        else begin
            request_sync<={request_sync[1:0],request};
            if(request_sync[2]!=ack && pixel_idle)begin
                pixel_model<=held_model;ack<=request_sync[2];
            end
        end
    end
endmodule

// USB vision metadata owner. No image RAM, detector, floating point or extra
// motion PID is implemented here. Pi computes a signed common wheel target;
// the existing navigation datapath holds yaw and the wheel PIDs close speed.
module robot_follow_control #(parameter integer ENABLE_CHOOSER=1)(
    input wire clk,reset_n,tick_10ms,request,
    input wire [255:0] data,
    output reg ack,
    input wire [2:0] page,subpage,
    input wire tap_valid,
    input wire [7:0] tap_button,
    input wire stop_all,cancel,motion_ready,battery_a_low,
    output reg shutdown_requested,software_stopped,
    input wire [7:0] gain,
    input wire [15:0] speed_cap,
    input wire signed [31:0] heading,
    input wire signed [15:0] actual_left,actual_right,
    input wire [3:0] nav_status,
    input wire turn_done,
    input wire signed [31:0] pose_x,pose_y,
    output reg armed,
    output wire permit,
    output reg command_valid,
    output reg [2:0] command,
    output wire signed [15:0] speed,
    output wire [255:0] telemetry,
    output wire pan_pwm,tilt_pwm,
    output reg [15:0] selected_id,
    output wire [15:0] candidate_id,
    output wire [7:0] candidate_kind,candidate_confidence,
    output wire [7:0] state_code,
    output reg [7:0] diagnostic_code,dog_score
    ,output wire mission_mode
    ,output reg signed [31:0] goal_x,goal_y,turn_heading
    ,output reg query_valid
    ,output reg [255:0] query_data
    ,input wire laser_valid,turn_clear
    ,input wire [15:0] laser_target
    ,input wire signed [31:0] laser_bearing
    ,output wire [2:0] task_ready_flags
);
    reg [255:0] packet_hold;
    reg decode_pending;
    reg [1:0] task_motion; //0 side follow,2 gyro turn,3 planned approach
    reg search_active,turn_consumed,search_turn_done,turn_was_running;
    reg [15:0] vec_sequence,gol_sequence;
    reg have_vec,have_gol;
    reg [7:0] vec_age,gol_age;
    reg signed [31:0] cached_goal_x,cached_goal_y;
    reg [15:0] cached_target;
    assign mission_mode=task_motion!=0;
    reg [2:0] req_sync;
    reg [255:0] catalog;
    // Browse by stable track ID, never by a slot that inference may reorder.
    reg [15:0] browse_id;
    reg [2:0] catalog_count;
    reg [15:0] catalog_seq,rx_seq;
    reg have_rx,have_catalog;
    reg [7:0] rx_age,catalog_age,rx_flags;
    reg [15:0] target_age,rx_target,event_ack;
    reg [8:0] low_ticks;
    reg [11:0] shutdown_ticks;
    reg shutdown_seen;
    reg signed [15:0] requested_speed;
    reg [11:0] pan_us,tilt_us,pan_low,pan_high,tilt_low,tilt_high;
    wire [11:0] applied_pan,applied_tilt;
    reg [8:0] delay_ticks;
    reg nav_started,auto_pending;
    reg [15:0] event_seq;
    reg [7:0] event_op;
    reg [2:0] previous_page;
    reg prepare_pose_pending;
    wire [15:0] packet_seq=packet_hold[207:192];
    wire [15:0] vec_delta=packet_seq-vec_sequence,gol_delta=packet_seq-gol_sequence;
    wire [15:0] rx_delta=packet_seq-rx_seq,cat_delta=packet_seq-catalog_seq;
    wire fol_new=!have_rx || rx_age>=200 || (rx_delta!=0 && !rx_delta[15]);
    wire cat_new=!have_catalog || catalog_age>=200 || (cat_delta!=0 && !cat_delta[15]);
    // Byte offsets10..21 are unsigned16 on wire; upper nibbles must be zero.
    wire pulse_ok=packet_hold[175:160]>0 && packet_hold[175:160]<=4095 &&
        packet_hold[159:144]>0 && packet_hold[159:144]<=4095 &&
        packet_hold[143:128]>0 && packet_hold[143:128]<=packet_hold[175:160] &&
        packet_hold[127:112]>=packet_hold[175:160] && packet_hold[127:112]<=4095 &&
        packet_hold[111:96]>0 && packet_hold[111:96]<=packet_hold[159:144] &&
        packet_hold[95:80]>=packet_hold[159:144] && packet_hold[95:80]<=4095;
    wire link_live=have_rx && rx_age<35;
    wire target_live=link_live && rx_flags[0] && target_age<=500 &&
        selected_id!=0 && rx_target==selected_id && event_ack==event_seq;
    wire reference_ready=target_live && !rx_flags[6] && rx_flags[2:1]==2'b11;
    wire ui_active=page==5 && (subpage==3 || subpage==4);
    wire blocked=stop_all || cancel || !ui_active || shutdown_requested;
    // Remote commands report the same availability as the existing LCD
    // actions. Readiness grants no movement until a real task/tap arrives.
    assign task_ready_flags[0]=link_live && !rx_flags[6] && !shutdown_requested && !stop_all;
    assign task_ready_flags[1]=laser_valid && laser_target==selected_id && target_live && motion_ready && turn_clear && !blocked;
    assign task_ready_flags[2]=have_gol && gol_age<50 && cached_target==selected_id && target_live && laser_valid && motion_ready && !blocked;
    assign permit=armed && delay_ticks==0 && !blocked && motion_ready && link_live && !rx_flags[6] && (task_motion!=2 || turn_clear);
    wire signed [15:0] cap=$signed(speed_cap>20 ? 16'd20 : speed_cap);
    assign speed=!target_live ? 16'sd0 : requested_speed>cap ? cap :
        requested_speed< -cap ? -cap : requested_speed;
    function [31:0] entry;
        input [1:0] index;
        begin case(index)
            0:entry=catalog[191:160];1:entry=catalog[159:128];
            2:entry=catalog[127:96];default:entry=catalog[95:64];
        endcase end
    endfunction
    wire [1:0] cursor=ENABLE_CHOOSER==0 ? 2'd0 : catalog_count>0 && catalog[191:176]==browse_id ? 2'd0 :
        catalog_count>1 && catalog[159:144]==browse_id ? 2'd1 :
        catalog_count>2 && catalog[127:112]==browse_id ? 2'd2 : 2'd3;
    wire [31:0] candidate=entry(cursor);
    wire catalog_live=have_catalog && catalog_age<50;
    wire candidate_live=ENABLE_CHOOSER!=0 && catalog_live && browse_id!=0 && cursor<catalog_count && candidate[31:16]==browse_id;
    wire [1:0] previous_cursor=cursor==0 ? catalog_count[1:0]-1'b1 : cursor-1'b1;
    wire [1:0] next_cursor={1'b0,cursor}+3'd1>=catalog_count ? 2'd0 : cursor+1'b1;
    wire [31:0] previous_entry=entry(previous_cursor),next_entry=entry(next_cursor);
    assign candidate_id=candidate_live ? candidate[31:16] : 16'd0;
    assign candidate_kind=candidate_live ? candidate[15:8] : 8'd0;
    assign candidate_confidence=candidate_live ? candidate[7:0] : 8'd0;
    //0 no link,1 choose,2 center/reference,3 teach,4 ready,5..7 countdown,
    //8 following,9 wait target,10 prerequisite unavailable,11 lidar blocked.
    assign state_code=shutdown_requested ? (rx_flags[6] ? 21 : shutdown_seen ? 13 : 12) :
        !link_live ? 0 : rx_flags[6] ? 15 :
        ENABLE_CHOOSER!=0 && page==5 && subpage==1 ? (!catalog_live ? 16 : catalog_count==0 ? 17 : !candidate_live ? 18 : 19) :
        page==5 && subpage==0 && selected_id!=0 && target_live ? 22 :
        selected_id==0 ? 1 : event_ack!=event_seq ? 20 : !target_live ? 9 : !rx_flags[1] ? 2 :
        !rx_flags[2] ? 3 : !motion_ready ? 10 : delay_ticks>200 ? 7 :
        delay_ticks>100 ? 6 : delay_ticks!=0 ? 5 : armed ?
        (!target_live ? 9 : nav_status==7 ? 11 : (nav_status==8 || nav_status==9) ? 10 : 8) : 4;
    wire servo_live=link_live && !stop_all && !cancel && page==5 && !shutdown_requested;
    robot_follow_pwm u_pan(.clk(clk),.reset_n(reset_n),.enable(servo_live && rx_flags[3]),
        .pulse_us(pan_us),.low_us(pan_low),.high_us(pan_high),.pwm(pan_pwm),.applied_us(applied_pan));
    robot_follow_pwm u_tilt(.clk(clk),.reset_n(reset_n),.enable(servo_live && rx_flags[4]),
        .pulse_us(tilt_us),.low_us(tilt_low),.high_us(tilt_high),.pwm(tilt_pwm),.applied_us(applied_tilt));
    wire [247:0] report={32'h52425431,8'd1,
        {1'b1,search_turn_done,laser_valid,shutdown_requested,selected_id!=0,link_live,delay_ticks!=0,armed},event_seq,event_op,selected_id,
        gain,speed_cap,heading,{4'd0,applied_pan},{4'd0,applied_tilt},rx_seq,
        actual_left,actual_right,catalog_seq,{4'd0,nav_status}};
    function [7:0] report_xor;
        input [247:0] bytes;
        integer k;
        begin report_xor=0;for(k=0;k<31;k=k+1)report_xor=report_xor ^ bytes[k*8+:8];end
    endfunction
    assign telemetry={report,report_xor(report)};
    always @(posedge clk or negedge reset_n)begin
        if(!reset_n)begin
            req_sync<=0;ack<=0;packet_hold<=0;decode_pending<=0;catalog<=0;browse_id<=0;catalog_count<=0;catalog_seq<=0;rx_seq<=0;
            have_rx<=0;have_catalog<=0;rx_age<=255;catalog_age<=255;rx_flags<=0;target_age<=65535;rx_target<=0;
            requested_speed<=0;pan_us<=0;tilt_us<=0;pan_low<=0;pan_high<=0;tilt_low<=0;tilt_high<=0;
            armed<=0;delay_ticks<=0;nav_started<=0;auto_pending<=0;diagnostic_code<=0;dog_score<=0;command_valid<=0;command<=0;
            event_seq<=0;event_op<=0;selected_id<=0;event_ack<=0;
            previous_page<=0;prepare_pose_pending<=0;
            low_ticks<=0;shutdown_ticks<=0;shutdown_requested<=0;software_stopped<=0;shutdown_seen<=0;
            task_motion<=0;search_active<=0;turn_consumed<=0;search_turn_done<=0;turn_was_running<=0;
            vec_sequence<=0;gol_sequence<=0;have_vec<=0;have_gol<=0;vec_age<=255;gol_age<=255;
            cached_goal_x<=0;cached_goal_y<=0;cached_target<=0;goal_x<=0;goal_y<=0;turn_heading<=0;
            query_valid<=0;query_data<=0;
        end else begin
            req_sync<={req_sync[1:0],request};command_valid<=0;query_valid<=0;decode_pending<=0;
            previous_page<=page;
            if(page!=5)prepare_pose_pending<=0;
            else if(previous_page!=5)prepare_pose_pending<=1;
            // Explicit entry into a vision task prepares the requested head
            // pose, without selecting a target or granting chassis motion.
            // TRACK also prepares on Pi, so a Pi boot baseline may safely
            // discard this event without replaying an old movement command.
            if(prepare_pose_pending && page==5 && link_live && event_seq==event_ack &&
               !stop_all && !cancel && !shutdown_requested && !tap_valid)begin
                prepare_pose_pending<=0;event_seq<=event_seq+1'b1;event_op<=11;
            end
            if(tick_10ms)begin
                if(battery_a_low)begin if(low_ticks<500)low_ticks<=low_ticks+1'b1;end
                else low_ticks<=0;
                // Pi poweroff never locks or switches off the FPGA.
                software_stopped<=0;
                if(rx_age!=255)rx_age<=rx_age+1'b1;
                if(catalog_age!=255)catalog_age<=catalog_age+1'b1;
                if(vec_age!=255)vec_age<=vec_age+1'b1;
                if(gol_age!=255)gol_age<=gol_age+1'b1;
                if(delay_ticks!=0)delay_ticks<=delay_ticks-1'b1;
                if(permit && task_motion==0)begin command_valid<=1;command<=nav_started ? 3'd5 : 3'd1;nav_started<=1;end
                if(armed && task_motion!=0)begin
                    if(nav_status==4 || nav_status==5 || nav_status==10)turn_was_running<=1;
                    if(task_motion==3 && turn_was_running && nav_status==6 &&
                       pose_x-goal_x<=5 && goal_x-pose_x<=5 && pose_y-goal_y<=5 && goal_y-pose_y<=5)begin
                        armed<=0;nav_started<=0;turn_was_running<=0;
                    end
                end
            end
            if(armed && task_motion==2 && turn_done)begin
                armed<=0;nav_started<=0;turn_was_running<=0;
                if(search_active)search_turn_done<=1;
            end
            // Held-data mailbox: source does not mutate until this ACK returns.
            if(req_sync[2]!=ack)begin
                // Capture first, decode on the following50MHz edge. The
                // source may release only after ACK crosses back; validation
                // never sits in a125MHz->50MHz combinational data path.
                packet_hold<=data;decode_pending<=1;ack<=req_sync[2];
            end
            if(decode_pending)begin
                if(packet_hold[255:224]==32'h464f4c31 && fol_new && pulse_ok &&
                   packet_hold[63:48]<=1000 &&
                   (!packet_hold[215] || (packet_hold[214:208]==0 && packet_hold[191:176]==0 && packet_hold[31:16]==0)) &&
                   $signed(packet_hold[191:176])>=-200 && $signed(packet_hold[191:176])<=200)begin
                    rx_seq<=packet_seq;have_rx<=1;rx_age<=0;rx_flags<=packet_hold[215:208];
                    requested_speed<=packet_hold[191:176];target_age<=packet_hold[79:64];rx_target<=packet_hold[31:16];event_ack<=packet_hold[47:32];
                    if(shutdown_requested && packet_hold[213])shutdown_seen<=1;
                    if(auto_pending && packet_hold[47:32]==event_seq && packet_hold[208] && packet_hold[31:16]!=0)begin
                        selected_id<=packet_hold[31:16];auto_pending<=0;event_op<=0;
                    end
                    pan_us<=packet_hold[171:160];tilt_us<=packet_hold[155:144];
                    pan_low<=packet_hold[139:128];pan_high<=packet_hold[123:112];
                    tilt_low<=packet_hold[107:96];tilt_high<=packet_hold[91:80];
                    if(packet_hold[215])begin
                        shutdown_requested<=0;shutdown_seen<=0;armed<=0;selected_id<=0;auto_pending<=0;
                        search_active<=0;turn_consumed<=0;search_turn_done<=0;event_op<=0;
                        task_motion<=0;cached_target<=0;gol_age<=255;command_valid<=1;command<=0;
                    end
                end
                if(packet_hold[255:224]==32'h56454331 && packet_hold[215:208]<=3 &&
                   (!have_vec || vec_age>=200 || (vec_delta!=0 && !vec_delta[15])) &&
                   $signed(packet_hold[159:144])>=-16384 && $signed(packet_hold[159:144])<=16384 &&
                   $signed(packet_hold[143:128])>=-16384 && $signed(packet_hold[143:128])<=16384 &&
                   $signed(packet_hold[79:64])>=-18000 && $signed(packet_hold[79:64])<=18000 && packet_hold[63:48]<=500)begin
                    have_vec<=1;vec_sequence<=packet_seq;vec_age<=0;query_data<=packet_hold;query_valid<=1;
                    if(!packet_hold[209])begin
                        turn_consumed<=0;search_turn_done<=0;
                        if(search_active && task_motion==2 && armed)begin armed<=0;command_valid<=1;command<=0;end
                    end
                    else if(search_active && !turn_consumed && !armed && motion_ready && turn_clear && link_live && !blocked)begin
                        turn_consumed<=1;task_motion<=2;armed<=1;turn_was_running<=0;
                        turn_heading<=heading>=90000 ? heading-270000 : heading+90000;
                        command_valid<=1;command<=2;
                    end
                end
                if(packet_hold[255:224]==32'h474f4c31 && packet_hold[215:208]==0 &&
                   (!have_gol || gol_age>=200 || (gol_delta!=0 && !gol_delta[15])) &&
                   packet_hold[191:176]==selected_id && selected_id!=0 && laser_valid && laser_target==selected_id &&
                   packet_hold[175:160]<=500 && packet_hold[31:16]>=350 && packet_hold[31:16]<=1500 &&
                   $signed(packet_hold[159:128])> -4000 && $signed(packet_hold[159:128])<4000 &&
                   $signed(packet_hold[127:96])> -4000 && $signed(packet_hold[127:96])<4000)begin
                    have_gol<=1;gol_sequence<=packet_seq;gol_age<=0;cached_target<=selected_id;
                    cached_goal_x<=packet_hold[159:128];cached_goal_y<=packet_hold[127:96];
                end
                if(packet_hold[255:224]==32'h54475431 && cat_new && packet_hold[215:208]<=4 && (packet_hold[63:16]==0 || (packet_hold[63:56]==8'ha1 && packet_hold[55:48]<=63 && packet_hold[47:40]<=100 && packet_hold[39:32]<=100)))begin
                    // Always accept a fresh list, even when the page was
                    // opened empty. Preserve browse_id across reorder/loss;
                    // a vanished ID cannot silently become another target.
                    if(ENABLE_CHOOSER!=0)begin
                        catalog<=packet_hold;catalog_count<=packet_hold[210:208];
                        if(browse_id==0 && packet_hold[210:208]!=0)browse_id<=packet_hold[191:176];
                    end
                    if(packet_hold[63:56]==8'ha1)begin diagnostic_code<=packet_hold[55:48];dog_score<=packet_hold[47:40];end
                    else begin diagnostic_code<=15;dog_score<=0;end
                    catalog_seq<=packet_seq;
                    have_catalog<=1;catalog_age<=0;
                end
            end
            if(!permit)nav_started<=0;
            if(tap_valid && page==5 && !shutdown_requested)begin
                prepare_pose_pending<=0;
                // An explicit UI action cancels any pending motor start.
                armed<=0;delay_ticks<=0;nav_started<=0;command_valid<=1;command<=0;
                search_active<=0;turn_was_running<=0;
                if((subpage==0 || subpage==4) && tap_button==1)begin
                    task_motion<=0;
                    selected_id<=0;auto_pending<=1;event_seq<=event_seq+1'b1;event_op<=5;rx_flags<=0;
                end
                if(subpage==0 && tap_button==2)begin
                    task_motion<=0;
                    selected_id<=0;auto_pending<=1;event_seq<=event_seq+1'b1;event_op<=9;rx_flags<=0;
                end
                if(subpage==0 && tap_button==3)begin
                    selected_id<=0;auto_pending<=0;rx_flags<=0;event_seq<=event_seq+1'b1;event_op<=4;
                end
                if(subpage==4)begin
                    task_motion<=0;
                    if(tap_button==2)begin event_seq<=event_seq+1'b1;event_op<=3;end
                    if(tap_button==3 && reference_ready && motion_ready && !stop_all && !cancel)begin
                        armed<=1;delay_ticks<=0;
                    end
                end
                if(subpage==3)case(tap_button)
                    1:begin
                        selected_id<=0;auto_pending<=1;event_seq<=event_seq+1'b1;event_op<=10;rx_flags<=0;
                        search_active<=1;task_motion<=0;turn_consumed<=0;search_turn_done<=0;cached_target<=0;gol_age<=255;
                    end
                    2:if(laser_valid && laser_target==selected_id && target_live && motion_ready && turn_clear)begin
                        task_motion<=2;armed<=1;turn_heading<=laser_bearing;command<=2;
                    end
                    3:if(have_gol && gol_age<50 && cached_target==selected_id && target_live && laser_valid && motion_ready)begin
                        task_motion<=3;armed<=1;goal_x<=cached_goal_x;goal_y<=cached_goal_y;command<=3;
                    end
                    5:begin event_seq<=event_seq+1'b1;event_op<=8;end
                    6:begin selected_id<=0;auto_pending<=0;rx_flags<=0;event_seq<=event_seq+1'b1;event_op<=4;end
                    default:begin end
                endcase
                if(ENABLE_CHOOSER!=0 && subpage==1)case(tap_button)
                    1:if(catalog_live && catalog_count!=0)browse_id<=candidate_live ? previous_entry[31:16] : catalog[191:176];
                    2:if(catalog_live && catalog_count!=0)browse_id<=candidate_live ? next_entry[31:16] : catalog[191:176];
                    3:if(candidate_id!=0 && (candidate_kind==1 || candidate_kind==2) && candidate_confidence<=100)begin
                        selected_id<=candidate_id;event_seq<=event_seq+1'b1;event_op<=1;
                        rx_flags[2:0]<=0;
                    end
                    4:begin selected_id<=0;rx_flags<=0;event_seq<=event_seq+1'b1;event_op<=4;end
                    6:begin browse_id<=catalog_live && catalog_count!=0 ? catalog[191:176] : 16'd0;end
                    default:begin end
                endcase
            end
            // Loss of a picture zeros speed but preserves the user's follow
            // request. A lost link/STOP/page exit requires another START.
            if(blocked || (armed && (!motion_ready || !link_live || rx_flags[6])))begin
                armed<=0;delay_ticks<=0;nav_started<=0;search_active<=0;
                if(nav_started || armed)begin command_valid<=1;command<=0;end
            end
            if(cancel && page==5 && !shutdown_requested && (selected_id!=0 || auto_pending || search_active))begin
                selected_id<=0;auto_pending<=0;search_active<=0;rx_flags<=0;event_seq<=event_seq+1'b1;event_op<=4;
            end
            if(stop_all && !shutdown_requested)begin
                selected_id<=0;rx_flags<=0;auto_pending<=0;search_active<=0;
                if(selected_id!=0 || auto_pending || search_active)begin event_seq<=event_seq+1'b1;event_op<=4;end
            end
            // A83 full mapping rounds raw17 to20%. Top gates this input on
            // a fresh valid gauge reading; require five continuous seconds.
            // This is a latched software stop, not an electrical power switch.
            if((!shutdown_requested && low_ticks==500) ||
               (tap_valid && page==5 && (subpage==0 || subpage==3 || subpage==4) && tap_button==7))begin
                shutdown_requested<=1;shutdown_ticks<=0;shutdown_seen<=0;armed<=0;delay_ticks<=0;nav_started<=0;auto_pending<=0;
                command_valid<=1;command<=0;event_seq<=event_seq+1'b1;event_op<=6;
            end
            if(page!=5 && !shutdown_requested && (selected_id!=0 || auto_pending))begin
                selected_id<=0;auto_pending<=0;rx_flags<=0;event_seq<=event_seq+1'b1;event_op<=4;
            end
        end
    end
endmodule

//50Hz PWM: apply the complete pulse only at the next frame boundary. Pi's
// calibrated controller already ramps requests; do not add1us/frame lag here.
module robot_follow_pwm(
    input wire clk,reset_n,enable,
    input wire [11:0] pulse_us,low_us,high_us,
    output wire pwm,
    output reg [11:0] applied_us
);
    reg [19:0] phase;
    wire valid_pulse=enable && low_us!=0 && low_us<=pulse_us && pulse_us<=high_us;
    wire [17:0] pulse_cycles=({6'd0,applied_us}<<5)+({6'd0,applied_us}<<4)+({6'd0,applied_us}<<1);
    assign pwm=valid_pulse && applied_us!=0 && phase<{2'd0,pulse_cycles};
    always @(posedge clk or negedge reset_n)begin
        if(!reset_n)begin phase<=0;applied_us<=0;end
        else if(!valid_pulse)begin phase<=0;applied_us<=0;end
        else if(phase==0)begin applied_us<=pulse_us;phase<=1;end
        else if(phase==999999)phase<=0;
        else phase<=phase+1'b1;
    end
endmodule
