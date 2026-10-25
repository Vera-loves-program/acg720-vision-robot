// Preserve one user goal while the local navigator follows safe map waypoints.
// No automatic reverse is issued by this module. A blockage parks the drive,
// waits for fresh stationary scan integrations, then freezes a known-free map
// for body-inflated BFS. Unknown space is blocked.
// Recoverable map/obstacle/sensor failures park the SAME mission and retry
// only after a new eligible completed map. Explicit STOP/manual takeover and
// safety_stop cancel; ARRIVED finishes. No automatic physical fault clearing.
module robot_mission_control (
    input wire clk,reset_n,tick_10ms,permit,safety_stop,
    input wire user_command_valid,
    input wire [2:0] user_command,
    input wire signed [31:0] user_goal_x,user_goal_y,
    input wire signed [31:0] pose_x,pose_y,
    input wire pose_valid,
    input wire [3:0] nav_status,
    input wire virtual_contact,rear_clear,
    input wire [15:0] recovery_counts,
    input wire map_ready,map_busy,map_capture_ready,scan_complete,
    input wire capture_boundary,
    input wire [15:0] scan_id,map_integrated_id,
    input wire map_integrated_valid,map_integrated_pulse,
    input wire [31:0] map_scan_count,
    input wire planner_busy,planner_done,planner_reachable,
    input wire [6:0] planner_next_x,planner_next_y,
    output reg planner_start,
    output wire planner_cancel,
    output wire [6:0] planner_current_x,planner_current_y,planner_goal_x,planner_goal_y,
    output reg nav_command_valid,
    output reg [2:0] nav_command,
    output reg signed [31:0] nav_goal_x,nav_goal_y,
    output reg [15:0] nav_arrival_tolerance_mm,
    output wire motion_permit,
    output wire waiting_map,
    output reg waiting_path,
    output reg [7:0] mission_error,
    output reg planning,no_path,mission_active,
    output reg [2:0] recovery_state,
    output wire signed [15:0] recovery_speed
);
    reg detouring,wait_depart,pending;
    reg [1:0] settle_ticks;
    reg [2:0] pending_command;
    reg signed [31:0] pending_x,pending_y;
    reg signed [31:0] final_x,final_y;
    reg refreshing_map;
    reg refresh_phase;
    reg [15:0] required_capture_id;
    reg [8:0] refresh_ticks;
    reg [5:0] map_age_ticks;
    reg invalid_goal,fault_paused;
    reg turn_mission;
    reg [1:0] recovery_attempts;
    reg [8:0] recovery_ticks;
    reg signed [31:0] retreat_x,retreat_y;
    wire signed [31:0] retreat_dx=pose_x-retreat_x,retreat_dy=pose_y-retreat_y;
    wire retreat_done=retreat_dx>=100 || retreat_dx<=-100 || retreat_dy>=100 || retreat_dy<=-100;
    wire [15:0] retreat_counts=recovery_counts==0 ? 16'd1 : recovery_counts>3 ? 16'd3 : recovery_counts;
    assign recovery_speed=16'sd0;
    wire signed [31:0] grid_x=(pose_x+32'sd4096)>>>6;
    wire signed [31:0] grid_y=(pose_y+32'sd4096)>>>6;
    wire signed [31:0] goal_grid_x=(final_x+32'sd4096)>>>6;
    wire signed [31:0] goal_grid_y=(final_y+32'sd4096)>>>6;
    wire coordinates_inside=pose_x>=-4096 && pose_x<4096 && pose_y>=-4096 && pose_y<4096 &&
        final_x>=-4096 && final_x<4096 && final_y>=-4096 && final_y<4096;
    // The centred64x64 planning lattice cannot represent fine index127.
    // Invalid goals stay visibly parked until a replacement GO or STOP.
    wire user_goal_inside=user_goal_x>=-4096 && user_goal_x<4032 &&
        user_goal_y>=-4096 && user_goal_y<4032;
    wire mission_requested=(mission_active && !user_command_valid) ||
        (user_command_valid && user_command==3);
    assign planner_current_x=grid_x[6:0];assign planner_current_y=grid_y[6:0];
    assign planner_goal_x=goal_grid_x[6:0];assign planner_goal_y=goal_grid_y[6:0];
    // Every new request supersedes the old search, including another GOTO.
    // A new movement is held until cancellation has released planner_busy.
    assign planner_cancel=!permit || safety_stop || user_command_valid || recovery_state!=0;
    assign motion_permit=permit && !planning && !planner_busy && !no_path && !pending;
    assign waiting_map=refreshing_map;
    wire [15:0] current_map_scan_age=scan_id-map_integrated_id;
    wire reusable_map=map_integrated_valid && (map_age_ticks<30 || map_integrated_pulse) &&
        current_map_scan_age<=16'd2 && map_capture_ready;
    wire [15:0] integrated_after_required=map_integrated_id-required_capture_id;
    // The one-clock integration event also excludes a sticky old16bit tag after a
    // long pause/half-wrap. Equality OR modulo-newer accepts a later complete
    // fresh scan if the first required scan was dropped by a correction.
    wire required_scan_integrated=map_integrated_valid &&
        map_integrated_pulse && !integrated_after_required[15];
    // Error0 delegates to planner_error. These codes distinguish failures
    // which never launched BFS, rather than displaying an old NO ROUTE.
    // 13 map-refresh timeout,14 pose unavailable,15 map unavailable,
    // 16 virtual contact,17 exact final leg blocked,18 stale NAV sensors,
    // 19 NAV fault,20 coordinates outside map,21 motion permit unavailable.
    wire signed [31:0] final_dx=pose_x-final_x,final_dy=pose_y-final_y;
    // A map cell/waypoint is NOT the user's exact destination. Keep a
    // relaxed waypoint acceptance, but finish only within5mm on BOTH axes.
    wire final_arrived=final_dx>=-5 && final_dx<=5 && final_dy>=-5 && final_dy<=5;
    wire mission_completed_now=mission_active && !waiting_path && !planning && !planner_busy && !wait_depart &&
        nav_status==6 && final_arrived && !user_command_valid;
    wire [7:0] rounded_grid_x={1'b0,grid_x[6:0]}+8'd1;
    wire [7:0] rounded_grid_y={1'b0,grid_y[6:0]}+8'd1;
    wire [7:0] rounded_goal_x={1'b0,goal_grid_x[6:0]}+8'd1;
    wire [7:0] rounded_goal_y={1'b0,goal_grid_y[6:0]}+8'd1;
    wire [7:0] rounded_next_x={1'b0,planner_next_x}+8'd1;
    wire [7:0] rounded_next_y={1'b0,planner_next_y}+8'd1;
    wire same_goal_cell=coordinates_inside && rounded_grid_x[7:1]==rounded_goal_x[7:1] && rounded_grid_y[7:1]==rounded_goal_y[7:1];
    wire next_is_goal_cell=rounded_next_x[7:1]==rounded_goal_x[7:1] && rounded_next_y[7:1]==rounded_goal_y[7:1];
    always @(posedge clk or negedge reset_n)begin
        if(!reset_n)begin
            planner_start<=0;nav_command_valid<=0;nav_command<=0;nav_goal_x<=0;nav_goal_y<=0;nav_arrival_tolerance_mm<=5;
            planning<=0;no_path<=0;mission_error<=0;mission_active<=0;detouring<=0;wait_depart<=0;final_x<=0;final_y<=0;
            pending<=0;pending_command<=0;pending_x<=0;pending_y<=0;settle_ticks<=0;
            turn_mission<=0;recovery_attempts<=0;recovery_ticks<=0;retreat_x<=0;retreat_y<=0;recovery_state<=0;
            refreshing_map<=0;refresh_phase<=0;required_capture_id<=0;
            refresh_ticks<=0;map_age_ticks<=63;
            waiting_path<=0;invalid_goal<=0;fault_paused<=0;
        end else begin
            planner_start<=0;nav_command_valid<=0;
            if(!map_integrated_valid)map_age_ticks<=63;
            else if(map_integrated_pulse)map_age_ticks<=0;
            else if(tick_10ms && map_age_ticks!=63)map_age_ticks<=map_age_ticks+1'b1;
            // Status may legitimately stay BLOCKED across two GOTO commands.
            // Wait three 10 ms ticks. The first can coincide with command
            // consumption and invalidate that control frame's generation;
            // the second then completes a fresh frame before the third tick.
            // This does not depend on a visible status value transition.
            if(wait_depart && tick_10ms)begin
                if(settle_ticks>1)settle_ticks<=settle_ticks-1'b1;
                else begin settle_ticks<=0;wait_depart<=0;end
            end
            if(user_command_valid)begin
                recovery_state<=0;recovery_attempts<=0;recovery_ticks<=0;turn_mission<=user_command==2;
                nav_command<=0;nav_command_valid<=1;
                pending<=permit && user_command>=1 && user_command<=3;
                pending_command<=user_command;pending_x<=user_goal_x;pending_y<=user_goal_y;
                if(user_command==4)nav_command<=4;
                planning<=0;no_path<=0;mission_error<=0;detouring<=0;wait_depart<=0;settle_ticks<=0;
                refreshing_map<=0;refresh_phase<=0;refresh_ticks<=0;
                waiting_path<=0;
                fault_paused<=0;
                invalid_goal<=user_command==3 && !user_goal_inside;
                mission_active<=user_command==3;
                if(user_command==3)begin final_x<=user_goal_x;final_y<=user_goal_y;end
            end else if(pending && !planner_busy && !planner_start)begin
                pending<=0;
                if(pending_command==3)begin
                    // Plan the FIRST GOTO before any wheel movement, reusing a
                    // fresh completed map without a fixed scan/ARM delay.
                    if(invalid_goal || !coordinates_inside || !pose_valid || !map_ready)begin
                        no_path<=1;waiting_path<=1;
                        mission_error<=invalid_goal ? 8'd20 : !pose_valid ? 8'd14 : !map_ready ? 8'd15 : 8'd20;
                    end else begin
                        planning<=1;
                        if(reusable_map)planner_start<=1;
                        else begin refreshing_map<=1;refresh_phase<=0;refresh_ticks<=0;end
                    end
                end else begin
                    nav_command<=pending_command;nav_command_valid<=1;
                    nav_goal_x<=pending_x;nav_goal_y<=pending_y;
                    nav_arrival_tolerance_mm<=5;
                    wait_depart<=1;settle_ticks<=3;
                end
            end else if(refreshing_map)begin
                // The old map can omit a newly arrived person. Keep motion
                // parked without freezing acquisition. Capture one WHOLE scan
                // beginning after stationary eligibility, then require its
                // committed identity, not arbitrary old queued integrations.
                if(tick_10ms && refresh_ticks<300)refresh_ticks<=refresh_ticks+1'b1;
                if(!pose_valid || !map_ready || refresh_ticks==300)begin
                    refreshing_map<=0;planning<=0;no_path<=1;waiting_path<=1;
                    mission_error<=!pose_valid ? 8'd14 : !map_ready ? 8'd15 : 8'd13;
                end else if(!map_capture_ready)begin
                    refresh_phase<=0;
                end else case(refresh_phase)
                    0:if(capture_boundary)begin
                        required_capture_id<=scan_id;refresh_phase<=1;
                    end
                    1:if(required_scan_integrated)begin
                        refreshing_map<=0;planner_start<=1;
                    end
                endcase
            end else if(planning && planner_done)begin
                planning<=0;
                if(planner_reachable && permit && pose_valid)begin
                    if(next_is_goal_cell)begin
                        // A128mm coarse cell cannot substitute for exact XY.
                        nav_goal_x<=final_x;nav_goal_y<=final_y;
                        nav_arrival_tolerance_mm<=5;detouring<=0;
                    end else begin
                        // Planner tiles are centred on multiples of128mm.
                        // +/-60mm remains inside the validated +/-64mm box.
                        nav_goal_x<=-32'sd4096+$signed({1'b0,planner_next_x[6:1],7'd0});
                        nav_goal_y<=-32'sd4096+$signed({1'b0,planner_next_y[6:1],7'd0});
                        nav_arrival_tolerance_mm<=60;detouring<=1;
                    end
                    nav_command<=3;nav_command_valid<=1;wait_depart<=1;settle_ticks<=3;
                end else begin
                    nav_command<=0;nav_command_valid<=1;
                    no_path<=1;waiting_path<=1;
                end
            end else if(mission_active && waiting_path && !planning && !planner_busy && !pending)begin
                // No busy-loop replanning, and no timer that silently ends a
                // mission. A new coherent observation is the retry trigger.
                if(!invalid_goal && !fault_paused && permit && coordinates_inside && pose_valid && map_ready &&
                   reusable_map && map_integrated_pulse)begin
                    waiting_path<=0;no_path<=0;mission_error<=0;planning<=1;planner_start<=1;
                    
                end
            end else if(mission_active && !planning && !planner_busy && !wait_depart)begin
                if(nav_status==6 && (turn_mission || final_arrived))begin mission_active<=0;detouring<=0;waiting_path<=0;no_path<=0;end
                else if(virtual_contact || (turn_mission && nav_status==7))begin
                    nav_command<=0;nav_command_valid<=1;
                    no_path<=1;waiting_path<=1;mission_error<=16;
                end
                else if(nav_status==6 && (!detouring || same_goal_cell))begin
                    // The old segment ended outside the final tolerance.
                    // Issue the actual final XY, never replan the same cell
                    // or accept a coarse waypoint as a completed mission.
                    nav_goal_x<=final_x;nav_goal_y<=final_y;
                    nav_arrival_tolerance_mm<=5;nav_command<=3;nav_command_valid<=1;
                    detouring<=0;wait_depart<=1;settle_ticks<=3;
                end
                else if(nav_status==7 || (detouring && nav_status==6))begin
                    nav_command<=0;nav_command_valid<=1;
                    if(nav_status==7 && same_goal_cell && !detouring)begin
                        // A blocked precise final leg cannot be solved by
                        // repeatedly asking BFS for that identical coarse cell.
                        no_path<=1;waiting_path<=1;mission_error<=17;
                    end else if(coordinates_inside && pose_valid && map_ready)begin
                        planning<=1;
                        if(nav_status==7)begin
                            refreshing_map<=1;refresh_phase<=0;refresh_ticks<=0;
                        end else planner_start<=1;
                    end else begin
                        no_path<=1;waiting_path<=1;
                        mission_error<=!pose_valid ? 8'd14 : !map_ready ? 8'd15 : 8'd20;
                    end
                end else if(nav_status==8 || nav_status==9)begin
                    nav_command<=0;nav_command_valid<=1;
                    no_path<=1;waiting_path<=1;
                    mission_error<=nav_status==8 ? 8'd18 : 8'd19;
                    if(nav_status==9)fault_paused<=1;
                end
            end
            if(!permit)begin
                // Permit may disappear because of a page change, stale input
                // or a physical motor fault. Park; never clear that fault.
                // Explicit manual commands still cancel the old mission.
                planning<=0;detouring<=0;wait_depart<=0;pending<=0;settle_ticks<=0;
                refreshing_map<=0;refresh_phase<=0;refresh_ticks<=0;
                nav_command<=0;nav_command_valid<=1;
                if(mission_requested && !mission_completed_now)begin
                    mission_active<=1;waiting_path<=1;no_path<=1;
                    mission_error<=user_command_valid && user_command==3 && !user_goal_inside ? 8'd20 : 8'd21;
                end
                if(user_command_valid && user_command==4)begin nav_command<=4;nav_command_valid<=1;end
            end
            if(safety_stop)begin
                recovery_state<=0;recovery_ticks<=0;recovery_attempts<=0;
                planning<=0;mission_active<=0;detouring<=0;wait_depart<=0;pending<=0;settle_ticks<=0;
                waiting_path<=0;invalid_goal<=0;fault_paused<=0;
                refreshing_map<=0;refresh_phase<=0;refresh_ticks<=0;
                nav_command<=0;nav_command_valid<=1;
                if(user_command_valid && user_command==4 && !safety_stop)begin nav_command<=4;nav_command_valid<=1;end
            end
        end
    end
endmodule
