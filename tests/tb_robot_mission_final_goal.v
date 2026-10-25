`timescale 1ns/1ps
// Standalone HDL regression, supplied for simulator use; not a synthesis file.
module tb_robot_mission_final_goal;
    reg clk=0;always #10 clk=~clk;
    reg reset_n=0,permit=1,user_valid=0,planner_busy=0,planner_done=0;
    reg [2:0] user_command=0;
    reg signed [31:0] pose_x=0,pose_y=0;
    reg [3:0] nav_status=0;
    reg [6:0] next_x=67,next_y=64;
    reg scan_complete=0,map_capture_ready=1,map_busy=0;
    reg [31:0] map_scans=0;
    integer planner_events=0;
    wire command_valid,planner_start,active,no_path;
    wire [2:0] command;
    wire signed [31:0] goal_x,goal_y;
    wire [15:0] tolerance;
    always @(posedge clk)if(reset_n && planner_start)planner_events<=planner_events+1;
    robot_mission_control dut(
        .clk(clk),.reset_n(reset_n),.tick_10ms(1'b1),.permit(permit),.safety_stop(1'b0),
        .user_command_valid(user_valid),.user_command(user_command),.user_goal_x(32'sd300),.user_goal_y(32'sd0),
        .pose_x(pose_x),.pose_y(pose_y),.pose_valid(1'b1),.nav_status(nav_status),
        .virtual_contact(1'b0),.rear_clear(1'b1),.recovery_counts(16'd10),
        .map_ready(1'b1),.map_busy(map_busy),.map_capture_ready(map_capture_ready),
        .scan_complete(scan_complete),.map_scan_count(map_scans),
        .planner_busy(planner_busy),.planner_done(planner_done),.planner_reachable(1'b1),
        .planner_next_x(next_x),.planner_next_y(next_y),.planner_start(planner_start),
        .planner_cancel(),.planner_current_x(),.planner_current_y(),.planner_goal_x(),.planner_goal_y(),
        .nav_command_valid(command_valid),.nav_command(command),.nav_goal_x(goal_x),.nav_goal_y(goal_y),
        .nav_arrival_tolerance_mm(tolerance),.motion_permit(),.planning(),.no_path(no_path),
        .mission_active(active),.recovery_state(),.recovery_speed()
    );
    task clock_step;
        begin @(negedge clk);end
    endtask
    task request;
        begin user_command=3;user_valid=1;clock_step;user_valid=0;repeat(8)clock_step;end
    endtask
    task respond;
        begin planner_busy=0;planner_done=1;clock_step;planner_done=0;repeat(7)clock_step;end
    endtask
    task fresh_map_after_block;
        integer before_count;
        begin
            before_count=planner_events;
            repeat(5)clock_step;
            if(planner_events!=before_count)$fatal(1,"BFS froze stale map before fresh scan");
            scan_complete=1;clock_step;scan_complete=0;clock_step;
            scan_complete=1;clock_step;scan_complete=0;clock_step;
            if(planner_events!=before_count)$fatal(1,"BFS did not wait for integrations");
            map_scans=map_scans+1;clock_step;clock_step;
            if(planner_events!=before_count)$fatal(1,"one queued integration accepted as fresh");
            map_busy=1;map_scans=map_scans+1;repeat(3)clock_step;
            if(planner_events!=before_count)$fatal(1,"BFS froze busy map before integration commit");
            map_busy=0;repeat(3)clock_step;
            if(planner_events!=before_count+1)$fatal(1,"fresh map did not start BFS exactly once");
            planner_busy=1;
        end
    endtask
    initial begin
        repeat(3)clock_step;reset_n=1;clock_step;request;
        if(goal_x!=300 || goal_y!=0 || tolerance!=5 || !active)$fatal(1,"direct goal must be exact300/0 tolerance5");
        // Old local DONE at240 must neither complete the mission nor authorize
        // a relaxed final stop. Reissue the actual coordinate if it occurs.
        pose_x=240;nav_status=6;repeat(5)clock_step;
        if(!active || goal_x!=300 || tolerance!=5)$fatal(1,"premature arrival240 accepted");
        nav_status=7;repeat(6)clock_step;nav_status=0;fresh_map_after_block;
        next_x=67;next_y=64;respond;
        if(tolerance!=60 || goal_x!=192 || goal_y!=64)$fatal(1,"intermediate coarse centre/tolerance failed");
        pose_x=180;pose_y=32;nav_status=6;repeat(7)clock_step;planner_busy=1;nav_status=0;
        next_x=68;next_y=64;respond;
        if(tolerance!=5 || goal_x!=300 || goal_y!=0)$fatal(1,"BFS goal cell usedcentre instead of300");
        pose_x=288;pose_y=0;nav_status=6;repeat(7)clock_step;
        if(!active || goal_x!=300 || tolerance!=5)$fatal(1,"coarse-cell centre accepted as final goal");
        pose_x=294;nav_status=6;repeat(7)clock_step;
        if(!active)$fatal(1,"6mm short accepted as final goal");
        pose_x=295;pose_y=6;nav_status=6;repeat(7)clock_step;
        if(!active)$fatal(1,"Y axis outside5mm accepted as final goal");
        pose_y=5;nav_status=6;repeat(7)clock_step;
        if(active)$fatal(1,"both-axis5mm boundary not completed");
        nav_status=0;pose_x=288;request;nav_status=7;repeat(7)clock_step;
        if(!no_path || active || planner_start)$fatal(1,"blocked final leg replans identical cell forever");
        nav_status=0;pose_x=0;request;permit=0;repeat(7)clock_step;
        if(active || (command_valid && command==3))$fatal(1,"STOP did not cancel final goal");
        permit=1;repeat(7)clock_step;
        if(active || (command_valid && command==3))$fatal(1,"old goal resumed without new request");
        nav_status=0;pose_x=0;pose_y=0;request;nav_status=7;repeat(8)clock_step;nav_status=0;
        repeat(310)clock_step;
        if(active || !no_path)$fatal(1,"stale map refresh did not time out parked");
        $display("PASS final5mm, coarse centre60mm, exactXY, fresh-map barrier and terminalSTOP");$finish;
    end
    initial begin #100000;$fatal(1,"timeout");end
endmodule
