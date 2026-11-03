// Compact six-task controller. No LCD PID/configuration/debug editor.
module robot_ui_control #(parameter integer CLK_HZ=50000000)(
    input wire clk,reset_n,safety_stop,motor_fault,tick_10ms,remote_takeover,remote_active,
    input wire touch_ready,snapshot_valid,snapshot_cancel,
    input wire [1:0] contact_count,
    input wire [10:0] x0,y0,x1,y1,
    input wire [3:0] id0,
    input wire signed [31:0] live_x_mm,live_y_mm,live_heading_mdeg,
    input wire motor_selected_wheel,motor_edit_position,gains_ready,gyro_ready,lidar_ready,
    input wire task_valid,
    input wire [3:0] task_action,
    input wire signed [31:0] task_value,
    input wire [2:0] task_visual_ready,
    output wire task_ready,
    output reg task_rsp_valid,
    output reg [7:0] task_rsp_status,
    output reg signed [31:0] task_rsp_value,
    output reg [2:0] page,subpage,
    output reg pid_action_valid,
    output reg [7:0] pid_action,
    output reg signed [31:0] pid_argument,
    output reg emergency_stop_pulse,clear_stop_pulse,armed,
    output wire tap_valid,
    output wire [7:0] tap_button,
    output wire [7:0] follow_gain,
    output wire [15:0] follow_cap,
    output reg command_valid,
    output reg [2:0] command,
    output reg signed [15:0] manual_left,manual_right,
    output reg manual_heading_hold,
    output wire [15:0] manual_speed_left,manual_speed_right,
    output wire manual_linked,
    output wire calibration_enable,
    output wire signed [15:0] calibration_duty_left,calibration_duty_right,
    output reg [3:0] ui_notice,
    output reg pose_set,
    output wire signed [31:0] pose_x_mm,pose_y_mm,pose_heading_mdeg,
    output reg signed [31:0] goal_x_mm,goal_y_mm,turn_heading_mdeg,
    output wire geometry_valid,
    output wire [15:0] wheel_mm_per_rev,encoder_counts_per_rev,track_mm,
    output wire [15:0] cruise_counts,turn_counts,
    output wire gyro_invert,
    output wire [14:0] lidar_forward_q6,
    output wire signed [15:0] lidar_offset_x_mm,lidar_offset_y_mm,
    output wire [15:0] body_front_mm,body_rear_mm,body_left_mm,body_right_mm,
    output wire [15:0] drive_margin_mm,turn_margin_mm,contact_margin_mm,
    output reg scan_enable,lidar_restart,map_clear,
    output wire [4:0] selected_field,
    output reg [1:0] step_select,
    output wire signed [31:0] selected_value,
    output wire [31:0] selected_step
);
    // Measured commissioning values are constants, not a bank of live LCD
    // registers. VOFA retains motor tuning; this UI contains only demos.
    assign follow_gain=8'd20;assign follow_cap=16'd5;
    assign manual_speed_left=16'd30;assign manual_speed_right=16'd30;
    assign manual_linked=1'b1;assign calibration_enable=1'b0;
    assign calibration_duty_left=16'sd0;assign calibration_duty_right=16'sd0;
    assign pose_x_mm=32'sd0;assign pose_y_mm=32'sd0;assign pose_heading_mdeg=32'sd0;
    assign geometry_valid=1'b1;assign wheel_mm_per_rev=16'd204;
    assign encoder_counts_per_rev=16'd1469;assign track_mm=16'd213;
    assign cruise_counts=16'd30;assign turn_counts=16'd12;assign gyro_invert=1'b0;
    assign lidar_offset_x_mm=16'sd33;assign lidar_offset_y_mm=-16'sd77;
    assign body_front_mm=16'd232;assign body_rear_mm=16'd118;
    assign body_left_mm=16'd119;assign body_right_mm=16'd119;
    assign drive_margin_mm=16'd20;assign turn_margin_mm=16'd30;assign contact_margin_mm=16'd10;
    assign selected_field=5'd0;assign selected_value=32'sd0;
    assign selected_step=step_select==0 ? 32'd10 : step_select==1 ? 32'd100 : 32'd500;
    reg [8:0] forward_degrees;
    assign lidar_forward_q6={forward_degrees,6'd0};
    reg [1:0] rebase_pending;
    reg [2:0] task_phase;
    reg [3:0] task_held_action;
    reg signed [31:0] task_held_value;
    reg [7:0] task_error;
    // VOFA owns no second motion controller: a task stages the same page and
    // action consumed below by the LCD path, then replies after application.
    reg [$clog2(CLK_HZ/1000)-1:0] ms_count;
    reg [9:0] touch_age;
    reg [7:0] view_cooldown_ms;
    reg wait_release,invalid_tap;
    reg [1:0] previous_count;
    reg [10:0] down_x,down_y;
    reg [3:0] down_id;
    reg [7:0] down_button,action;
    reg action_valid;
    assign task_ready=task_phase==0 && !task_rsp_valid && !action_valid && rebase_pending==0;
    assign tap_button=action;
    wire [10:0] dx=x0>=down_x ? x0-down_x : down_x-x0;
    wire [10:0] dy=y0>=down_y ? y0-down_y : down_y-y0;
    wire [7:0] current_button,second_button;
    robot_ui_hit_test u_hit0(.x(x0),.y(y0[9:0]),.page(page),.subpage(subpage),.button(current_button));
    robot_ui_hit_test u_hit1(.x(x1),.y(y1[9:0]),.page(page),.subpage(subpage),.button(second_button));
    function stop_button;
        input [7:0] b;input [2:0] p,s;
        begin stop_button=b==106 || (p==4 && b==6) ||
            (p==2 && s==3 && b==2) ||
            (p==5 && ((s==0 && b==3)||(s==3 && b==6)||(s==4 && b==4)));end
    endfunction
    wire stop_contact=snapshot_valid && touch_ready &&
        ((contact_count!=0 && stop_button(current_button,page,subpage)) ||
         (contact_count==2 && stop_button(second_button,page,subpage)));
    wire local_ready=!safety_stop && !motor_fault && touch_ready && !remote_active;
    // A queued remote FIND must not escape on the edge a local STOP arrives.
    // Preserve stop/release taps themselves while masking movement actions.
    wire task_cancel_live=safety_stop || motor_fault || !touch_ready || stop_contact ||
        (snapshot_valid && snapshot_cancel) || remote_takeover || remote_active ||
        (task_phase!=0 && task_error!=0);
    assign tap_valid=action_valid && (!task_cancel_live || stop_button(action,page,subpage) || action==105);
    reg [2:0] next_page,next_subpage;
    always @* begin
        next_page=page;next_subpage=subpage;
        if(action==100)begin next_page=0;next_subpage=0;end
        else if(action==101)begin
            if(page==3 && subpage==3)begin next_page=4;next_subpage=1;end
            else if(page==2 && subpage==1)next_subpage=3;
            else if(page==5 && subpage==4)next_subpage=0;
            else begin next_page=0;next_subpage=0;end
        end else if(page==0 && subpage==0)case(action)
            1:begin next_page=0;next_subpage=1;end
            2:begin next_page=4;next_subpage=0;end
            3:begin next_page=2;next_subpage=3;end
            4:begin next_page=4;next_subpage=1;end
            5:begin next_page=5;next_subpage=0;end
            6:begin next_page=5;next_subpage=3;end
            default:begin end
        endcase
        else if(page==4 && subpage==1 && action==5)begin next_page=3;next_subpage=3;end
        else if(page==3 && subpage==3 && action==6)begin next_page=4;next_subpage=1;end
        else if(page==2 && subpage==3 && action==3)next_subpage=1;
        else if(page==2 && subpage==1 && action==1)next_subpage=3;
        else if(page==5 && subpage==0 && action==5)next_subpage=4;
    end
    wire view_change=next_page!=page || next_subpage!=subpage;
    // A bounded forward destination gives task3 the SAME known-free planner
    // and stop/replan lifecycle as task4, rather than an endless MANUAL run.
    wire avoid_goal_cancel=view_change || safety_stop || motor_fault || !touch_ready ||
        stop_contact || (snapshot_valid && snapshot_cancel) || remote_takeover || remote_active ||
        !gains_ready || !gyro_ready || !lidar_ready ||
        (action_valid && (action==105 || action==106 ||
            (page==2 && subpage==3 && (action==2 || action==4))));
    wire avoid_goal_start=action_valid && page==2 && subpage==3 && action==1 &&
        local_ready && gains_ready && gyro_ready && lidar_ready;
    wire avoid_goal_done,avoid_goal_error;
    wire signed [31:0] avoid_goal_x,avoid_goal_y;
    robot_avoid_forward_goal u_avoid_goal(
        .clk(clk),.reset_n(reset_n),.start(avoid_goal_start),.cancel(avoid_goal_cancel),
        .pose_x(live_x_mm),.pose_y(live_y_mm),.heading_mdeg(live_heading_mdeg),
        .done(avoid_goal_done),.error(avoid_goal_error),.goal_x(avoid_goal_x),.goal_y(avoid_goal_y)
    );
    function signed [31:0] relative_heading;
        input signed [31:0] h,d;
        reg signed [31:0] sum;
        begin sum=h+d;relative_heading=sum>=180000 ? sum-360000 : sum< -180000 ? sum+360000 : sum;end
    endfunction
    function signed [31:0] step_goal;
        input signed [31:0] value;
        input [31:0] step;
        input subtract;
        reg signed [32:0] sum;
        begin
            sum={value[31],value}+(subtract ? -$signed({1'b0,step}) : $signed({1'b0,step}));
            step_goal=sum>3900 ? 32'sd3900 : sum< -3900 ? -32'sd3900 : sum[31:0];
        end
    endfunction
    always @(posedge clk or negedge reset_n)begin
        if(!reset_n)begin
            page<=0;subpage<=0;pid_action_valid<=0;pid_action<=10;pid_argument<=0;
            emergency_stop_pulse<=0;clear_stop_pulse<=0;armed<=0;
            command_valid<=0;command<=0;manual_left<=0;manual_right<=0;manual_heading_hold<=0;
            ui_notice<=0;pose_set<=0;goal_x_mm<=300;goal_y_mm<=0;turn_heading_mdeg<=0;
            rebase_pending<=0;forward_degrees<=90;scan_enable<=0;lidar_restart<=0;map_clear<=0;
            step_select<=1;ms_count<=0;touch_age<=1023;view_cooldown_ms<=150;wait_release<=1;
            previous_count<=0;down_x<=0;down_y<=0;down_id<=0;down_button<=0;
            invalid_tap<=1;action<=0;action_valid<=0;
            task_phase<=0;task_held_action<=0;task_held_value<=0;task_error<=0;
            task_rsp_valid<=0;task_rsp_status<=0;task_rsp_value<=0;
        end else begin
            pid_action_valid<=0;pid_argument<=0;emergency_stop_pulse<=0;clear_stop_pulse<=0;
            command_valid<=0;pose_set<=0;map_clear<=0;lidar_restart<=0;action_valid<=0;
            task_rsp_valid<=0;
            if(avoid_goal_done && !avoid_goal_cancel && page==2 && subpage==3)begin
                if(!avoid_goal_error && local_ready && gains_ready && gyro_ready && lidar_ready)begin
                    goal_x_mm<=avoid_goal_x;goal_y_mm<=avoid_goal_y;
                    armed<=1;command<=3;command_valid<=1;ui_notice<=0;
                end else begin
                    armed<=0;command<=0;command_valid<=1;
                    ui_notice<=avoid_goal_error ? 4'd10 : !gains_ready ? 4'd2 : !gyro_ready ? 4'd3 : 4'd5;
                end
            end
            if(ms_count==CLK_HZ/1000-1)begin
                ms_count<=0;if(touch_age!=1023)touch_age<=touch_age+1'b1;
                if(view_cooldown_ms!=0)view_cooldown_ms<=view_cooldown_ms-1'b1;
            end else ms_count<=ms_count+1'b1;
            if(rebase_pending!=0)begin
                rebase_pending<=rebase_pending-1'b1;
                if(rebase_pending==1)begin
                    goal_x_mm<=300;goal_y_mm<=0;pose_set<=1;map_clear<=1;
                end
            end
            if(task_phase!=0 || !touch_ready || (snapshot_valid && snapshot_cancel) || view_cooldown_ms!=0 || wait_release)begin
                previous_count<=0;invalid_tap<=1;touch_age<=1023;down_button<=0;
                if(snapshot_valid && contact_count==0 && view_cooldown_ms==0)wait_release<=0;
            end else if(snapshot_valid)begin
                previous_count<=contact_count;
                if(contact_count==1 && previous_count==0)begin
                    down_x<=x0;down_y<=y0;down_id<=id0;down_button<=current_button;touch_age<=0;invalid_tap<=0;
                    if(page==4 && subpage==0 && current_button>=1 && current_button<=4)begin
                        action<=current_button;action_valid<=1;invalid_tap<=1;
                    end
                end else if(contact_count==2)invalid_tap<=1;
                else if(contact_count==1)begin
                    if(previous_count==2 || id0!=down_id || dx>16 || dy>16 || current_button!=down_button)invalid_tap<=1;
                end else if(contact_count==0 && previous_count==1 && !invalid_tap && touch_age<=600 && down_button!=0)begin
                    invalid_tap<=1;action<=down_button;action_valid<=1;
                end
            end
            if(action_valid)begin
                ui_notice<=0;
                if(action==105)begin
                    armed<=0;clear_stop_pulse<=1;command<=0;command_valid<=1;
                    pid_action<=11;pid_action_valid<=1;
                end
                if(page==4 && subpage==0)begin
                    if(action>=1 && action<=4 && local_ready)begin
                        armed<=1;manual_left<=(action==2 || action==3) ? -$signed(manual_speed_left) : $signed(manual_speed_left);
                        manual_right<=(action==2 || action==4) ? -$signed(manual_speed_right) : $signed(manual_speed_right);
                        manual_heading_hold<=0;command<=1;command_valid<=1;
                    end else if(action==5 && local_ready && gyro_ready)begin
                        armed<=1;turn_heading_mdeg<=relative_heading(live_heading_mdeg,180000);
                        command<=2;command_valid<=1;
                    end
                end
                if(page==4 && subpage==1)begin
                    if(action==3)begin
                        armed<=0;command<=0;command_valid<=1;rebase_pending<=3;ui_notice<=9;
                    end else if(action==1 || action==2 || action==4)begin
                        scan_enable<=1;
                        if(local_ready && gains_ready && gyro_ready && lidar_ready)begin
                            armed<=1;command<=action==4 ? 3'd3 : 3'd2;command_valid<=1;
                            if(action!=4)turn_heading_mdeg<=relative_heading(live_heading_mdeg,action==1 ? 90000 : -90000);
                        end else ui_notice<=!gains_ready ? 2 : !gyro_ready ? 3 : 5;
                    end
                end
                if(page==3 && subpage==3)begin
                    if(action==1 || action==2)goal_x_mm<=step_goal(goal_x_mm,selected_step,action==1);
                    if(action==3 || action==4)goal_y_mm<=step_goal(goal_y_mm,selected_step,action==3);
                    if(action==5)step_select<=step_select==2 ? 0 : step_select+1'b1;
                end
                if(page==2 && subpage==1 && action==2)map_clear<=1;
                if(page==2 && subpage==3)begin
                    if(action==4)begin
                        forward_degrees<=forward_degrees>=270 ? forward_degrees-270 : forward_degrees+90;
                        map_clear<=1;armed<=0;command<=0;command_valid<=1;
                    end
                    if(action==1)begin
                        scan_enable<=1;
                        // Cancel the preceding run while the forward goal is
                        // captured. No delayed result may escape a local STOP.
                        armed<=0;command<=0;command_valid<=1;
                        if(!local_ready || !gains_ready || !gyro_ready || !lidar_ready)
                            ui_notice<=!gains_ready ? 2 : !gyro_ready ? 3 : 5;
                    end
                end
                if(view_change)begin
                    armed<=0;command<=0;command_valid<=1;
                    if(next_page==2 || (next_page==4 && next_subpage==1) || next_page==5)scan_enable<=1;
                    page<=next_page;subpage<=next_subpage;view_cooldown_ms<=150;wait_release<=1;invalid_tap<=1;previous_count<=0;
                    if(!remote_active)begin pid_action<=10;pid_action_valid<=1;end
                end
            end
            case(task_phase)
                0:if(task_valid && task_ready)begin
                    task_held_action<=task_action;task_held_value<=task_value;task_phase<=1;
                    task_error<=task_action==7 ? 8'd0 :
                        (safety_stop || motor_fault || !touch_ready || stop_contact ||
                        (snapshot_valid && snapshot_cancel)) ? 8'd4 :
                        (remote_takeover || remote_active) ? 8'd5 : 8'd0;
                    // FACE/GO NEAR reuse a live selected dog. A raw motor STOP
                    // here would erase that Pi selection before the action.
                    if(task_action!=5 && task_action!=6)begin
                        armed<=0;command<=0;command_valid<=1;pid_action<=10;pid_action_valid<=1;
                    end
                end
                1:begin
                    scan_enable<=1;task_phase<=2;wait_release<=1;invalid_tap<=1;
                    if(task_held_action<=3)begin page<=4;subpage<=1;end
                    else if(task_held_action<=7)begin page<=5;subpage<=3;end
                    else if(task_held_action==8)begin page<=5;subpage<=0;end
                end
                2:begin
                    task_phase<=3;
                    if(task_error!=0)begin end
                    else if(task_held_action>8 || ((task_held_action==1 || task_held_action==2) &&
                       (task_held_value< -3900 || task_held_value>3900)) ||
                       (task_held_action!=1 && task_held_action!=2 && task_held_value!=0))task_error<=2;
                    else if(task_held_action!=7 && (safety_stop || motor_fault || !touch_ready || remote_active ||
                            (snapshot_valid && snapshot_cancel) || stop_contact))task_error<=4;
                    else case(task_held_action)
                        0:begin action<=3;action_valid<=1;end
                        1:goal_x_mm<=task_held_value;
                        2:goal_y_mm<=task_held_value;
                        3:if(gains_ready && gyro_ready && lidar_ready)begin action<=4;action_valid<=1;end else task_error<=4;
                        4:if(task_visual_ready[0])begin action<=1;action_valid<=1;end else task_error<=4;
                        5:if(task_visual_ready[1])begin action<=2;action_valid<=1;end else task_error<=4;
                        6:if(task_visual_ready[2])begin action<=3;action_valid<=1;end else task_error<=4;
                        7:begin action<=6;action_valid<=1;armed<=0;command<=0;command_valid<=1;pid_action<=10;pid_action_valid<=1;end
                        8:if(task_visual_ready[0])begin action<=2;action_valid<=1;end else task_error<=4;
                        default:task_error<=2;
                    endcase
                end
                3:task_phase<=4;
                4:if(task_held_action!=0 || task_error!=0 || (rebase_pending==0 && !pose_set))begin
                    task_rsp_valid<=1;task_rsp_status<=task_error!=0 ? task_error :
                        task_held_action!=7 && (safety_stop || motor_fault || !touch_ready || stop_contact ||
                        (snapshot_valid && snapshot_cancel)) ? 8'd4 :
                        task_held_action!=7 && remote_takeover ? 8'd5 : 8'd0;
                    task_rsp_value<=0;task_phase<=0;
                    if(task_held_action==1)task_rsp_value<=goal_x_mm;
                    if(task_held_action==2)task_rsp_value<=goal_y_mm;
                end
                default:task_phase<=0;
            endcase
            // STOP is immediate on contact. Radar keeps scanning so that a
            // cleared stop does not silently leave NAV without live feedback.
            if(safety_stop || motor_fault || stop_contact || !touch_ready || (snapshot_valid && snapshot_cancel))begin
                armed<=0;command<=0;command_valid<=1;
                if(!remote_active || safety_stop || motor_fault || stop_contact)begin
                    pid_action_valid<=1;pid_action<=action_valid && action==105 ? 11 : 10;
                end
                if(stop_contact || !touch_ready || (snapshot_valid && snapshot_cancel))begin action_valid<=0;invalid_tap<=1;end
                if(stop_contact && (current_button==106 || (contact_count==2 && second_button==106)))emergency_stop_pulse<=1;
                if(task_phase!=0 && task_held_action!=7)begin task_error<=4;action_valid<=0;end
            end
            if(remote_takeover)begin
                armed<=0;command<=0;command_valid<=1;
                if(task_phase!=0 && task_held_action!=7)begin task_error<=5;action_valid<=0;end
            end
        end
    end
endmodule

// Project a one-metre demonstration goal from a frozen axle pose/heading.
// Twelve sequential CORDIC iterations, fixed one-bit shifts, no DSP or ROM.
// This module only proposes coordinates; it never grants motor permission.
module robot_avoid_forward_goal(
    input wire clk,reset_n,start,cancel,
    input wire signed [31:0] pose_x,pose_y,heading_mdeg,
    output reg done,error,
    output reg signed [31:0] goal_x,goal_y
);
    localparam IDLE=0,SHIFT_LOAD=1,SHIFT_STEP=2,APPLY=3,SUM=4,COMMIT=5;
    reg [2:0] state;
    reg [3:0] iteration,remaining;
    reg signed [16:0] cx,cy,shift_x,shift_y;
    reg signed [18:0] cz;
    reg signed [31:0] origin_x,origin_y;
    reg signed [32:0] sum_x,sum_y;
    wire signed [17:0] rounded_x=$signed({cx[16],cx})+18'sd8;
    wire signed [17:0] rounded_y=$signed({cy[16],cy})+18'sd8;
    wire signed [32:0] delta_x=$signed({{15{rounded_x[17]}},rounded_x})>>>4;
    wire signed [32:0] delta_y=$signed({{15{rounded_y[17]}},rounded_y})>>>4;
    function signed [18:0] atan_mdeg;
        input [3:0] index;
        begin case(index)
            0:atan_mdeg=45000;1:atan_mdeg=26565;2:atan_mdeg=14036;3:atan_mdeg=7125;
            4:atan_mdeg=3576;5:atan_mdeg=1790;6:atan_mdeg=895;7:atan_mdeg=448;
            8:atan_mdeg=224;9:atan_mdeg=112;10:atan_mdeg=56;default:atan_mdeg=28;
        endcase end
    endfunction
    always @(posedge clk or negedge reset_n)begin
        if(!reset_n)begin
            state<=IDLE;iteration<=0;remaining<=0;cx<=0;cy<=0;shift_x<=0;shift_y<=0;cz<=0;
            origin_x<=0;origin_y<=0;sum_x<=0;sum_y<=0;goal_x<=0;goal_y<=0;done<=0;error<=0;
        end else begin
            done<=0;
            if(cancel)begin state<=IDLE;error<=0;end
            else if(start)begin
                error<=0;origin_x<=pose_x;origin_y<=pose_y;cy<=0;iteration<=0;
                if(heading_mdeg< -180000 || heading_mdeg>180000 ||
                   pose_x< -3900 || pose_x>3900 || pose_y< -3900 || pose_y>3900)begin
                    error<=1;done<=1;state<=IDLE;
                end else begin
                    // 607.25mm in Q4 compensates the CORDIC rotation gain.
                    if(heading_mdeg>90000)begin cx<=-17'sd9716;cz<=heading_mdeg-180000;end
                    else if(heading_mdeg< -90000)begin cx<=-17'sd9716;cz<=heading_mdeg+180000;end
                    else begin cx<=17'sd9716;cz<=heading_mdeg;end
                    state<=SHIFT_LOAD;
                end
            end else case(state)
                SHIFT_LOAD:begin shift_x<=cx;shift_y<=cy;remaining<=iteration;state<=SHIFT_STEP;end
                SHIFT_STEP:begin
                    if(remaining==0)state<=APPLY;
                    else begin shift_x<=shift_x>>>1;shift_y<=shift_y>>>1;remaining<=remaining-1'b1;end
                end
                APPLY:begin
                    if(cz>=0)begin cx<=cx-shift_y;cy<=cy+shift_x;cz<=cz-atan_mdeg(iteration);end
                    else begin cx<=cx+shift_y;cy<=cy-shift_x;cz<=cz+atan_mdeg(iteration);end
                    if(iteration==11)state<=SUM;
                    else begin iteration<=iteration+1'b1;state<=SHIFT_LOAD;end
                end
                SUM:begin
                    sum_x<=$signed({origin_x[31],origin_x})+delta_x;
                    sum_y<=$signed({origin_y[31],origin_y})+delta_y;state<=COMMIT;
                end
                COMMIT:begin
                    // Reject a destination outside the supported field;
                    // clipping would silently change the promised distance.
                    error<=sum_x< -3900 || sum_x>3900 || sum_y< -3900 || sum_y>3900;
                    goal_x<=sum_x[31:0];goal_y<=sum_y[31:0];done<=1;state<=IDLE;
                end
                default:state<=IDLE;
            endcase
        end
    end
endmodule
