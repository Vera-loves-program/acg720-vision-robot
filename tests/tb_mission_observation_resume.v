`timescale 1ns/1ps
// Real mission FSM regression. Map observations and planner replies are
// explicit handshakes; scan_count changes cannot substitute for scan IDs.
module tb_mission_observation_resume;
    reg clk=0;always #5 clk=~clk;
    reg reset_n=0,permit=1,safety_stop=0,user_valid=0;
    reg [2:0] user_command=0;
    reg signed [31:0] user_x=1000,user_y=0,pose_x=0,pose_y=0;
    reg [3:0] nav_status=0;
    reg map_capture_ready=1,capture_boundary=0,map_pulse=0,map_valid=0;
    reg [15:0] scan_id=0,integrated_id=0;
    reg planner_busy=0,planner_done=0,planner_reachable=0;
    reg [6:0] planner_next_x=79,planner_next_y=64;
    reg planner_next_is_connector=0;
    wire planner_start,planner_cancel,nav_valid,motion_permit,waiting_map,waiting_path;
    wire planning,no_path,active;
    wire [2:0] nav_command;
    wire signed [31:0] nav_x,nav_y;
    wire [15:0] tolerance;
    wire [7:0] mission_error;
    integer starts=0,checks=0;
    robot_mission_control dut(
        .clk(clk),.reset_n(reset_n),.tick_10ms(1'b1),.permit(permit),.safety_stop(safety_stop),
        .user_command_valid(user_valid),.user_command(user_command),.user_goal_x(user_x),.user_goal_y(user_y),
        .pose_x(pose_x),.pose_y(pose_y),.pose_valid(1'b1),.nav_status(nav_status),
        .virtual_contact(1'b0),.rear_clear(1'b1),.recovery_counts(16'd1),
        .map_ready(1'b1),.map_busy(1'b0),.map_capture_ready(map_capture_ready),.scan_complete(1'b0),
        .capture_boundary(capture_boundary),.scan_id(scan_id),.map_integrated_id(integrated_id),
        .map_integrated_valid(map_valid),.map_integrated_pulse(map_pulse),.map_scan_count(32'd999),
        .planner_busy(planner_busy),.planner_done(planner_done),.planner_reachable(planner_reachable),
        .planner_next_x(planner_next_x),.planner_next_y(planner_next_y),.planner_next_is_connector(planner_next_is_connector),
        .planner_start(planner_start),.planner_cancel(planner_cancel),.planner_current_x(),.planner_current_y(),
        .planner_goal_x(),.planner_goal_y(),.nav_command_valid(nav_valid),.nav_command(nav_command),
        .nav_goal_x(nav_x),.nav_goal_y(nav_y),.nav_arrival_tolerance_mm(tolerance),.motion_permit(motion_permit),
        .waiting_map(waiting_map),.waiting_path(waiting_path),.mission_error(mission_error),
        .planning(planning),.no_path(no_path),.mission_active(active),.recovery_state(),.recovery_speed());
    always @(posedge clk)begin
        if(!reset_n)starts<=0;
        else if(planner_start)starts<=starts+1;
    end
    task step;begin @(negedge clk);end endtask
    task observe;
        input [15:0] id,current_scan;
        begin integrated_id=id;scan_id=current_scan;map_valid=1;map_pulse=1;step;map_pulse=0;step;end
    endtask
    task go;
        input integer x,y;
        begin user_x=x;user_y=y;user_command=3;user_valid=1;step;user_valid=0;step;end
    endtask
    task expect_start;
        input integer count;
        integer clocks;
        begin
            clocks=0;while(starts<count && clocks<20)begin step;clocks=clocks+1;end
            if(starts!=count || !planning)$fatal(1,"expected planning event%0d got%0d planning%0d",count,starts,planning);
            planner_busy=1;checks=checks+1;
        end
    endtask
    task reply;
        input integer reachable;
        begin planner_busy=0;planner_reachable=reachable;planner_done=1;step;planner_done=0;repeat(6)step;end
    endtask
    task parked;
        begin
            if(!active || !waiting_path || !no_path || motion_permit || planning)
                $fatal(1,"failed route did not preserve SAME parked mission");
            if(dut.final_x!=1000 || dut.final_y!=0)$fatal(1,"parked mission lost original goal");
            checks=checks+1;
        end
    endtask
    task stop;
        begin
            user_command=0;user_valid=1;#1;
            if(!planner_cancel)$fatal(1,"STOP did not cancel planner immediately");
            step;user_valid=0;repeat(4)step;
            if(active || planning || waiting_path)$fatal(1,"STOP did not cancel mission");
            checks=checks+1;
        end
    endtask
    initial begin
        repeat(3)step;reset_n=1;step;
        observe(100,101);go(1000,0);expect_start(1);reply(1);
        if(nav_x!=1000 || nav_y!=0 || tolerance!=5 || !active)$fatal(1,"initial final coordinate not exact");
        // Person appears on the moving route: preserve goal, park, and wait
        // for a whole new eligible captured scan and its committed identity.
        pose_x=200;nav_status=7;step;nav_status=0;repeat(3)step;
        if(!waiting_map || !planning || motion_permit || !active)$fatal(1,"block did not park before fresh-map capture");
        if(starts!=1)$fatal(1,"block replanned stale pre-person map");
        scan_id=102;capture_boundary=1;step;capture_boundary=0;step;
        observe(101,103);repeat(3)step;
        if(starts!=1)$fatal(1,"integration older than required post-stop capture resumed route");
        observe(102,103);expect_start(2);reply(0);parked;
        // The person is still there / old callback repeats: pulse alone is
        // insufficient. Equal AND older-but-age-eligible IDs cannot retry.
        observe(102,103);repeat(3)step;
        if(starts!=2)$fatal(1,"duplicate integrated ID retried failed route");
        parked;observe(101,103);repeat(3)step;
        if(starts!=2)$fatal(1,"older integrated ID retried failed route");
        // Person clears and a newer coherent map commits: same goal resumes.
        observe(103,104);expect_start(3);reply(1);
        if(nav_x!=1000 || nav_y!=0 || tolerance!=5 || waiting_path || no_path || !active)
            $fatal(1,"fresh observation did not resume original1000/0 goal");
        checks=checks+1;
        stop;
        observe(104,105);planner_done=1;planner_reachable=1;step;planner_done=0;repeat(8)step;
        if(active || starts!=3 || (nav_valid && nav_command==3))$fatal(1,"stale map/planner reply resurrected STOPped goal");
        checks=checks+1;
        // The special exact-start connector shares final goal's coarse tile.
        // It MUST remain centre128/0 tolerance60 before exact100/0 tolerance5.
        pose_x=0;observe(105,106);go(100,0);expect_start(4);
        planner_next_x=66;planner_next_y=64;planner_next_is_connector=1;reply(1);
        if(nav_x!=128 || nav_y!=0 || tolerance!=60 || !active)$fatal(1,"goal-tile connector incorrectly replaced by final XY");
        pose_x=128;nav_status=6;step;nav_status=0;repeat(5)step;
        if(nav_x!=100 || nav_y!=0 || tolerance!=5 || !active)$fatal(1,"connector did not hand off to exact final XY");
        pose_x=94;nav_status=6;repeat(6)step;
        if(!active)$fatal(1,"6mm short accepted as ARRIVED");
        pose_x=95;pose_y=6;repeat(6)step;
        if(!active)$fatal(1,"Y outside5mm accepted as ARRIVED");
        pose_y=5;repeat(6)step;
        if(active)$fatal(1,"both-axis5mm inclusive boundary did not finish");
        checks=checks+1;
        // Modulo16bit identity:65535->0 is genuinely newer; an old65535
        // callback after failing ID0 is stale even though its age is only2.
        reset_n=0;repeat(3)step;reset_n=1;nav_status=0;pose_x=0;pose_y=0;planner_next_is_connector=0;step;
        observe(16'hffff,0);go(1000,0);expect_start(1);reply(0);parked;
        observe(0,1);expect_start(2);reply(0);parked;
        observe(16'hffff,1);repeat(4)step;
        if(starts!=2)$fatal(1,"wrapped stale observation retried route");
        observe(1,2);expect_start(3);reply(1);
        stop;
        $display("PASS mission actualFSM: %0d checks, post-stop captureID barrier, stale/equal/wrapped ID rejection, same-goal fresh-map resume, STOP, connector and final5mm",checks);
        $finish;
    end
    initial begin #200000;$fatal(1,"mission test watchdog");end
endmodule
