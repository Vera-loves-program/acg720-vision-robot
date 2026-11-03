`timescale 1ns/1ps
// Execute the actual sequential projector, not a separate CORDIC model.
// Offline only; excluded from the synthesis project.
module tb_lidar_body_guard;
    reg clk=0; always #10 clk=~clk;
    reg reset_n=0,active=1,tick_10ms=0,point_valid=0,scan_start=0;
    reg [14:0] angle=0,forward=5760;
    reg [15:0] distance=0;
    wire ready,front_stop,rear_stop,front_slow,turn_stop,contact,map_valid;
    wire [3:0] coverage;
    wire [15:0] self_points,radius;
    wire signed [15:0] map_x,map_y;
    reg command_valid=0,gyro_sample=0;
    reg [2:0] command=0;
    reg signed [31:0] goal_x=300,goal_y=0;
    reg signed [31:0] turn_heading=0;
    wire nav_run,gyro_calibrated;
    wire signed [15:0] target_left,target_right;
    wire [3:0] nav_status;
    wire [7:0] nav_fault;
    integer projected=0,emitted=0,cycles;
    integer a,d,h,expected_x,expected_y,tolerance;
    real radians,reference_x,reference_y;
    lidar_body_guard dut(
        .clk(clk),.reset_n(reset_n),.active(active),.tick_10ms(tick_10ms),
        .point_valid(point_valid),.scan_start(scan_start),.angle_q6(angle),
        .forward_angle_q6(forward),.distance_mm(distance),
        .offset_x_mm(16'sd33),.offset_y_mm(-16'sd77),
        .body_front_mm(16'd232),.body_rear_mm(16'd118),
        .body_left_mm(16'd119),.body_right_mm(16'd119),
        .drive_margin_mm(16'd20),.turn_margin_mm(16'd30),.contact_margin_mm(16'd10),
        .ready(ready),.front_stop(front_stop),.rear_stop(rear_stop),
        .front_slow(front_slow),.turn_stop(turn_stop),.virtual_contact(contact),
        .coverage(coverage),.self_points(self_points),.rotation_radius_mm(radius),
        .map_point_valid(map_valid),.body_point_x(map_x),.body_point_y(map_y)
    );
    always @(negedge clk) if(map_valid) emitted=emitted+1;

    robot_navigation #(.BODY_GUARD(1),.GUARD_EXPLICIT_TURN(1)) nav(
        .clk(clk),.reset_n(reset_n),.tick_10ms(tick_10ms),
        .safety_stop(1'b0),.motion_enable(1'b1),
        .encoder_position_left(32'sd0),.encoder_position_right(32'sd0),.encoder_zeroed(1'b0),
        .gyro_z(16'sd0),.gyro_sample(gyro_sample),.gyro_valid(1'b1),.gyro_invert(1'b0),
        .geometry_valid(1'b1),.wheel_mm_per_rev(16'd204),.encoder_counts_per_rev(16'd1469),.track_mm(16'd213),
        .pose_set(1'b0),.pose_x_mm(32'sd0),.pose_y_mm(32'sd0),.pose_heading_mdeg(32'sd0),
        .command_valid(command_valid),.command(command),.manual_left(16'sd5),.manual_right(16'sd5),
        .manual_heading_hold(1'b1),.turn_heading_mdeg(turn_heading),.turn_half_direction(2'd0),
        .goal_x_mm(goal_x),.goal_y_mm(goal_y),.cruise_counts(16'd30),.turn_counts(16'd12),
        .arrival_tolerance_mm(16'd5),.avoid_enable(1'b1),.lidar_valid(1'b1),
        .clear_front_mm(16'd1000),.clear_left_mm(16'd1000),.clear_right_mm(16'd1000),.clear_rear_mm(16'd1000),
        .stop_clearance_mm(16'd0),.slow_clearance_mm(16'd0),.rotate_clearance_mm(16'd0),
        .body_guard_ready(ready),.body_front_stop(front_stop),.body_rear_stop(rear_stop),
        .body_front_slow(front_slow),.body_turn_stop(turn_stop),.body_coverage(coverage),.body_nav_ahead(1'b0),
        .lidar_correction_valid(1'b0),.lidar_correction_x_mm(16'sd0),.lidar_correction_y_mm(16'sd0),
        .run_request(nav_run),.target_speed_left(target_left),.target_speed_right(target_right),
        .gyro_calibrated(gyro_calibrated),.status(nav_status),.fault(nav_fault)
    );

    task send_command;
        input [2:0] value;
        begin
            @(negedge clk);command=value;command_valid=1;
            @(negedge clk);command_valid=0;
        end
    endtask
    task control_frame;
        begin
            // Advance one logical 10ms sample; allow the full sequential
            // navigation datapath to complete before another sample.
            @(negedge clk);gyro_sample=1;tick_10ms=1;
            @(negedge clk);gyro_sample=0;tick_10ms=0;
            repeat(1600)@(negedge clk);
            if(nav.state!=0 || nav_fault!=0)$fatal(1,"navigation frame failed: fault=%0d",nav_fault);
        end
    endtask
    task open_scan;
        begin
            ray(5760,1000);ray(0,1000);ray(11520,1000);ray(17280,1000);
            boundary;
        end
    endtask

    task idle;
        begin
            cycles=0;
            while(dut.state!=dut.IDLE && cycles<500)begin
                @(negedge clk);cycles=cycles+1;
            end
            if(dut.state!=dut.IDLE)$fatal(1,"body guard FSM timeout");
        end
    endtask
    task boundary;
        begin
            idle;
            @(negedge clk);scan_start=1;
            @(negedge clk);scan_start=0;
        end
    endtask
    task ray;
        input integer raw_angle,range_mm;
        begin
            idle;
            @(negedge clk);angle=raw_angle;distance=range_mm;point_valid=1;
            @(negedge clk);point_valid=0;
            idle;
            radians=($signed({1'b0,forward})-raw_angle)*3.141592653589793/11520.0;
            reference_x=33+range_mm*$cos(radians);
            reference_y=-77+range_mm*$sin(radians);
            expected_x=$rtoi(reference_x+(reference_x<0 ? -0.5 : 0.5));
            expected_y=$rtoi(reference_y+(reference_y<0 ? -0.5 : 0.5));
            tolerance=range_mm/500+3;
            if($signed(dut.px)<expected_x-tolerance || $signed(dut.px)>expected_x+tolerance ||
               $signed(dut.py)<expected_y-tolerance || $signed(dut.py)>expected_y+tolerance)
                $fatal(1,"projection: forward=%0d angle=%0d range=%0d expected=(%0d,%0d) actual=(%0d,%0d) front_stop=%b turn_stop=%b",
                    forward,raw_angle,range_mm,expected_x,expected_y,$signed(dut.px),$signed(dut.py),front_stop,turn_stop);
            projected=projected+1;
        end
    endtask
    task clear_scan;
        begin boundary;boundary;end
    endtask
    task clear_flags;
        begin
            if(front_stop || rear_stop || front_slow || turn_stop || contact)
                $fatal(1,"empty scan contains a phantom obstacle");
        end
    endtask

    initial begin
        repeat(4)@(negedge clk);reset_n=1;
        repeat(128)begin
            @(negedge clk);gyro_sample=1;@(negedge clk);gyro_sample=0;
        end
        if(!gyro_calibrated)$fatal(1,"gyro calibration failed");
        idle;boundary;
        // At the measured default mount this is a chassis return, not a wall.
        ray(52,50);boundary;
        if(!ready || self_points!=1 || emitted!=0)$fatal(1,"SELF rejection failed");
        clear_flags;
        if(radius!=291)$fatal(1,"rotation radius incorrect");
        // Empty surroundings plus chassis echoes must stay clear on every scan.
        for(h=0;h<3;h=h+1)begin
            for(a=0;a<23040;a=a+64)ray(a,1000);
            ray(0,100);ray(52,50);boundary;
            if(!ready || coverage!=15 || self_points!=2)$fatal(1,"free scan coverage/SELF failed");
            clear_flags;
            if(h==0)send_command(3);
            control_frame;control_frame;
            if(!nav_run || nav_status!=5 || target_left!=30 || target_right!=30)
                $fatal(1,"free-space GOTO blocked: status=%0d run=%b targets=(%0d,%0d)",nav_status,nav_run,target_left,target_right);
        end
        clear_scan;
        // Real obstacle: slow, stop and contact remain distinct.
        open_scan;ray(5760,230);boundary;
        if(!front_slow || front_stop || contact)$fatal(1,"slow-only obstacle failed");
        control_frame;
        if(!nav_run || target_left!=15 || target_right!=15)$fatal(1,"GOTO did not slow for real obstacle");
        clear_scan;open_scan;ray(5760,210);boundary;
        if(!front_stop || contact)$fatal(1,"front stop failed");
        control_frame;
        if(nav_run || nav_status!=7 || target_left!=0 || target_right!=0)$fatal(1,"GOTO did not stop for real obstacle");
        open_scan;control_frame;
        if(!nav_run || nav_status!=5)$fatal(1,"GOTO failed to resume after clear scan");
        clear_scan;ray(5760,205);boundary;
        if(!front_stop || !contact)$fatal(1,"virtual contact failed");
        clear_scan;ray(17280,165);boundary;
        if(!rear_stop || front_stop)$fatal(1,"rear stop failed");
        clear_scan;ray(0,300);boundary;
        if(!turn_stop || front_stop)$fatal(1,"side rotation guard failed");
        clear_scan;clear_flags;
        open_scan;goal_x=0;goal_y=300;send_command(3);control_frame;
        if(!nav_run || nav_status!=4 || target_left>=0 || target_right<=0)$fatal(1,"GOTO heading alignment failed");
        ray(5760,1000);ray(0,1000);ray(11520,1000);ray(17280,1000);
        ray(0,300);boundary;control_frame;
        if(nav_run || nav_status!=7)$fatal(1,"GOTO rotated into side obstacle");
        open_scan;control_frame;
        if(!nav_run || nav_status!=4)$fatal(1,"GOTO alignment failed to recover");
        send_command(0);control_frame;
        if(nav_run || nav_status!=0)$fatal(1,"STOP failed");
        turn_heading=90000;send_command(2);control_frame;
        if(!nav_run || nav_status!=4)$fatal(1,"guarded explicit turn did not start in clear space");
        ray(5760,1000);ray(0,1000);ray(11520,1000);ray(17280,1000);
        ray(0,300);boundary;control_frame;
        if(nav_run || nav_status!=7)$fatal(1,"NAV explicit turn bypassed collision envelope");
        open_scan;control_frame;
        if(!nav_run || nav_status!=4)$fatal(1,"guarded explicit turn failed to recover");
        send_command(0);control_frame;
        // Sweep all quadrants at several ranges and installation zero angles.
        for(h=0;h<4;h=h+1)begin
            @(negedge clk);forward=h*5760;
            @(negedge clk);idle;boundary;
            for(d=0;d<5;d=d+1)begin
                for(a=0;a<23040;a=a+64)
                    ray(a,d==0 ? 50 : d==1 ? 100 : d==2 ? 400 : d==3 ? 4000 : 12000);
                boundary;
                if(!ready)$fatal(1,"valid point spacing caused scan invalidation");
            end
        end
        // Late/missing scans and overruns must still revoke permission.
        repeat(30)begin
            @(negedge clk);tick_10ms=1;@(negedge clk);tick_10ms=0;
        end
        if(ready)$fatal(1,"stale scan remained ready");
        boundary;
        @(negedge clk);point_valid=1;distance=1000;
        repeat(2)@(negedge clk);point_valid=0;idle;boundary;
        if(ready)$fatal(1,"overrun scan remained ready");
        boundary;
        if(!ready)$fatal(1,"clean scan did not recover");
        @(negedge clk);active=0;@(negedge clk);
        if(ready)$fatal(1,"disabled sensor remained ready");
        $display("PASS: %0d actual RTL projections; SELF/free scans, slow/stop/contact/turn, stale/overrun/recovery",projected);
        $display("PASS: actual guard -> navigation GOTO: free drive, slow, BLOCKED stop, clear recovery, guarded alignment, explicit TURN, STOP");
        $finish;
    end
    initial begin #100000000;$fatal(1,"test timeout");end
endmodule
