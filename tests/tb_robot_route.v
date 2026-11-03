`timescale 1ns/1ps
// Actual mission owner + inflated BFS planner. A synchronous test map and
// completed-segment pose events model the surroundings/chassis, not the RTL.
module tb_robot_route;
    reg clk=0;always #10 clk=~clk;
    reg reset_n=0,tick=0,permit=1,stop=0,user_valid=0;
    reg [2:0] user_command=0;
    reg signed [31:0] pose_x=0,pose_y=0,user_x=1000,user_y=0;
    reg [3:0] nav_status=0;
    reg [15:0] scan_id=10,integrated_id=9;
    reg integrated=0,boundary=0;
    reg obstacle=1,unknown_start=0;
    wire plan_start,plan_cancel,busy,done,reachable,freeze;
    wire [6:0] current_x,current_y,goal_x,goal_y,next_x,next_y,rd_x,rd_y;
    wire rd_en;
    reg rd_valid=0;
    reg [1:0] rd_cell=0;
    wire command_valid,active,no_path,planning,waiting_map,waiting_path,motion_permit;
    wire [2:0] command;
    wire signed [31:0] nav_goal_x,nav_goal_y;
    wire [7:0] error,plan_error;
    integer commands=0,plans=0,cycles,prior_commands,prior_plans,legs,lateral;
    reg signed [31:0] issued_x,issued_y;
    always @(posedge clk)begin
        rd_valid<=rd_en;
        if(rd_en)rd_cell<=unknown_start && rd_x>=62 && rd_x<=65 && rd_y>=62 && rd_y<=65 ? 2'b00 :
            obstacle && rd_x>=71 && rd_x<=72 && rd_y>=61 && rd_y<=66 ? 2'b11 : 2'b01;
        if(reset_n && command_valid && command==3)begin
            commands=commands+1;issued_x=nav_goal_x;issued_y=nav_goal_y;
        end
        if(reset_n && plan_start && !plan_cancel)plans=plans+1;
    end
    robot_mission_control mission(
        .clk(clk),.reset_n(reset_n),.tick_10ms(tick),.permit(permit),.safety_stop(stop),
        .user_command_valid(user_valid),.user_command(user_command),.user_goal_x(user_x),.user_goal_y(user_y),
        .pose_x(pose_x),.pose_y(pose_y),.pose_valid(1'b1),.nav_status(nav_status),
        .virtual_contact(1'b0),.rear_clear(1'b1),.recovery_counts(16'd30),
        .map_ready(1'b1),.map_busy(1'b0),.map_capture_ready(1'b1),.scan_complete(1'b0),
        .capture_boundary(boundary),.scan_id(scan_id),.map_integrated_id(integrated_id),
        .map_integrated_valid(1'b1),.map_integrated_pulse(integrated),.map_scan_count(32'd10),
        .planner_busy(busy),.planner_done(done),.planner_reachable(reachable),
        .planner_next_x(next_x),.planner_next_y(next_y),.planner_start(plan_start),.planner_cancel(plan_cancel),
        .planner_current_x(current_x),.planner_current_y(current_y),.planner_goal_x(goal_x),.planner_goal_y(goal_y),
        .nav_command_valid(command_valid),.nav_command(command),.nav_goal_x(nav_goal_x),.nav_goal_y(nav_goal_y),
        .motion_permit(motion_permit),.waiting_map(waiting_map),.waiting_path(waiting_path),
        .mission_error(error),.planning(planning),.no_path(no_path),.mission_active(active)
    );
    lidar_path_planner planner(
        .clk(clk),.reset_n(reset_n),.start(plan_start && !plan_cancel),.cancel(plan_cancel),
        .current_x(current_x),.current_y(current_y),.goal_x(goal_x),.goal_y(goal_y),
        .heading_q6(16'd0),.clearance_radius_mm(16'd291),.map_ready(1'b1),.map_frozen(freeze),
        .freeze_map(freeze),.busy(busy),.done(done),.reachable(reachable),.error_code(plan_error),
        .next_x(next_x),.next_y(next_y),.plan_rd_en(rd_en),.plan_rd_x(rd_x),.plan_rd_y(rd_y),
        .plan_rd_cell(rd_cell),.plan_rd_valid(rd_valid)
    );
    task clocks;
        input integer n;
        begin repeat(n)@(negedge clk);end
    endtask
    task pulse_tick;
        begin @(negedge clk);tick=1;@(negedge clk);tick=0;clocks(3);end
    endtask
    task request;
        input [2:0] value;
        begin @(negedge clk);user_command=value;user_valid=1;@(negedge clk);user_valid=0;clocks(3);end
    endtask
    task refresh;
        begin
            @(negedge clk);scan_id=scan_id+1;boundary=1;
            @(negedge clk);boundary=0;clocks(3);
            // An older queued integration must not release the capture barrier.
            prior_plans=plans;integrated=1;
            @(negedge clk);integrated=0;clocks(4);
            if(plans!=prior_plans)$fatal(1,"old integration released stationary scan barrier");
            integrated_id=scan_id;integrated=1;
            @(negedge clk);integrated=0;clocks(4);
        end
    endtask
    task await_goal;
        input integer previous;
        begin
            cycles=0;
            while(commands==previous && !no_path && cycles<5000000)begin @(negedge clk);cycles=cycles+1;end
            if(commands!=previous+1 || no_path)$fatal(1,"expected next route leg: error=%0d planner=%0d",error,plan_error);
        end
    endtask
    task finish_route;
        begin
            legs=0;lateral=0;
            while(active && legs<16)begin
                if(issued_y!=0)lateral=lateral+1;
                if(issued_x< -4096 || issued_x>=4032 || issued_y< -4096 || issued_y>=4032)
                    $fatal(1,"wrapped route waypoint");
                nav_status=5;pulse_tick;pulse_tick;pulse_tick;
                pose_x=issued_x;pose_y=issued_y;nav_status=6;prior_commands=commands;
                clocks(8);legs=legs+1;
                if(active)await_goal(prior_commands);
            end
            if(active || pose_x!=user_x || pose_y!=user_y)$fatal(1,"mission did not finish at exact original goal");
        end
    endtask
    initial begin
        clocks(4);reset_n=1;clocks(4);request(3);refresh;await_goal(0);
        if(issued_x==1000 && issued_y==0)$fatal(1,"planner crossed inflated obstacle directly");
        finish_route;
        if(lateral==0 || legs<3)$fatal(1,"no actual BFS detour was followed");
        // A new obstacle appears after a direct route was issued. The old
        // destination must survive BLOCKED and the fresh-map capture barrier.
        pose_x=0;pose_y=0;nav_status=0;obstacle=0;
        integrated_id=scan_id;integrated=1;clocks(1);integrated=0;
        prior_commands=commands;request(3);await_goal(prior_commands);
        if(issued_x!=1000 || issued_y!=0)$fatal(1,"open map did not return exact destination");
        nav_status=5;pulse_tick;pulse_tick;pulse_tick;obstacle=1;nav_status=7;clocks(8);
        if(!waiting_map || motion_permit)$fatal(1,"dynamic blockage did not park for fresh map");
        prior_commands=commands;refresh;nav_status=0;await_goal(prior_commands);finish_route;
        if(lateral==0)$fatal(1,"dynamic obstacle was not bypassed");
        // Unknown space parks, a NEW completed map enables retry, and STOP
        // while BFS is busy cannot publish a stale movement afterwards.
        pose_x=0;pose_y=0;nav_status=0;obstacle=0;unknown_start=1;
        integrated_id=scan_id;integrated=1;clocks(1);integrated=0;
        request(3);cycles=0;
        while(!no_path && cycles<5000000)begin clocks(1);cycles=cycles+1;end
        if(!active || !waiting_path || plan_error!=8)$fatal(1,"unknown start was not held");
        unknown_start=0;prior_plans=plans;clocks(20);
        if(plans!=prior_plans)$fatal(1,"retry without a new map");
        integrated_id=scan_id;integrated=1;clocks(1);integrated=0;clocks(12);
        if(!busy)$fatal(1,"fresh map did not restart parked mission");
        prior_commands=commands;request(0);clocks(20);
        if(active || busy || freeze || commands!=prior_commands)$fatal(1,"STOP failed to cancel BFS ownership");
        // Manual takeover also cancels rather than resuming the old goal.
        request(3);clocks(15);request(1);clocks(20);
        if(active || busy || freeze)$fatal(1,"manual takeover retained navigation task");
        request(0);clocks(10);
        $display("PASS: actual inflated BFS + mission: static/dynamic detour to exact goal, fresh-map barrier, unknown retry, STOP and manual cancellation");
        $finish;
    end
    initial begin
        #1000000000;#1000000000;#1000000000;
        $fatal(1,"route regression timeout");
    end
endmodule
