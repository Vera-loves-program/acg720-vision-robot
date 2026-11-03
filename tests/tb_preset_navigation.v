`timescale 1ns/1ps
// Actual sequencer + navigator. An ideal wheel/gyro plant drives raw sensor
// inputs; this does not model servo mechanics, slip, inertia or stopping gaps.
module tb_preset_navigation;
    reg clk=0;always #5 clk=~clk;
    reg reset_n=0,tick=0,gyro_sample=0,start=0,cancel=0,search=0;
    reg lidar=1,guard_ready=1,front_stop=0,turn_stop=0;
    reg [3:0] coverage=15;
    reg signed [15:0] gyro=0;
    reg signed [31:0] enc_left=0,enc_right=0;
    wire [3:0] phase,status;
    wire [7:0] fault;
    wire engaged,moving,cv,run,turn_done,calibrated,calibrating,servo;
    wire [2:0] cmd,seconds_left;
    wire signed [15:0] speed,left,right;
    wire signed [31:0] yaw,target,px,py;
    wire ready=lidar && guard_ready && calibrated && !calibrating;
    integer k,checks=0,frames=0,sum_yaw=0,last_yaw=0,delta=0;
    reg [15:0] held_elapsed;
    wire staged_enable,staged_valid,staged_hold,staged_avoid;
    wire [2:0] staged_cmd;
    wire [1:0] staged_half;
    wire signed [15:0] staged_left,staged_right;
    wire signed [31:0] staged_heading,staged_gx,staged_gy;
    wire [15:0] staged_cruise,staged_turn,staged_tolerance;
    robot_preset_demo #(.DRIVE1_TICKS(8),.DRIVE2_TICKS(8),.DRIVE3_TICKS(8),
        .SEARCH_DRIVE_TICKS(8),.PAUSE_TICKS(5)) demo(
        .clk(clk),.reset_n(reset_n),.tick_10ms(tick),.start(start),.search_mode(search),
        .cancel(cancel),.ready(ready),.heading_mdeg(yaw),.nav_run(run),.turn_done(turn_done),
        .nav_status(status),.nav_fault(fault),.engaged(engaged),.moving_phase(moving),
        .phase(phase),.seconds_left(seconds_left),.command_valid(cv),.command(cmd),
        .speed(speed),.turn_heading(target),.servo_enable(servo),.pan_us(),.tilt_us()
    );
    robot_navigation_command_stage stage(
        .clk(clk),.reset_n(reset_n),.cancel(cancel),
        .in_enable(engaged && moving && !cancel),.in_valid(cv),.in_command(cmd),
        .in_hold(1'b1),.in_avoid(1'b1),.in_half_direction(2'd0),
        .in_left(speed),.in_right(speed),.in_heading(target),.in_goal_x(32'sd0),.in_goal_y(32'sd0),
        .in_cruise(16'd10),.in_turn(16'd12),.in_tolerance(16'd5),
        .enable(staged_enable),.valid(staged_valid),.command(staged_cmd),
        .hold_heading(staged_hold),.avoid(staged_avoid),.half_direction(staged_half),
        .left_speed(staged_left),.right_speed(staged_right),.heading(staged_heading),
        .goal_x(staged_gx),.goal_y(staged_gy),.cruise(staged_cruise),
        .turn_speed(staged_turn),.tolerance(staged_tolerance)
    );
    robot_navigation #(.BODY_GUARD(1),.GUARD_EXPLICIT_TURN(1)) nav(
        .clk(clk),.reset_n(reset_n),.tick_10ms(tick),.safety_stop(cancel),
        .motion_enable(staged_enable),
        .encoder_position_left(enc_left),.encoder_position_right(enc_right),.encoder_zeroed(1'b0),
        .gyro_z(gyro),.gyro_sample(gyro_sample),.gyro_valid(1'b1),.gyro_invert(1'b0),
        .geometry_valid(1'b1),.wheel_mm_per_rev(16'd204),.encoder_counts_per_rev(16'd1469),.track_mm(16'd213),
        .pose_set(1'b0),.pose_x_mm(32'sd0),.pose_y_mm(32'sd0),.pose_heading_mdeg(32'sd0),
        .command_valid(staged_valid),.command(staged_cmd),.manual_left(staged_left),.manual_right(staged_right),.manual_heading_hold(staged_hold),
        .turn_heading_mdeg(staged_heading),.turn_half_direction(staged_half),.goal_x_mm(staged_gx),.goal_y_mm(staged_gy),
        .cruise_counts(staged_cruise),.turn_counts(staged_turn),.arrival_tolerance_mm(staged_tolerance),
        .avoid_enable(staged_avoid),.lidar_valid(lidar),.clear_front_mm(16'd1000),.clear_left_mm(16'd1000),
        .clear_right_mm(16'd1000),.clear_rear_mm(16'd1000),.stop_clearance_mm(16'd0),
        .slow_clearance_mm(16'd0),.rotate_clearance_mm(16'd0),.body_guard_ready(guard_ready),
        .body_front_stop(front_stop),.body_rear_stop(1'b0),.body_front_slow(1'b0),
        .body_turn_stop(turn_stop),.body_nav_ahead(1'b0),.body_coverage(coverage),
        .lidar_correction_valid(1'b0),.lidar_correction_x_mm(16'sd0),.lidar_correction_y_mm(16'sd0),
        .run_request(run),.target_speed_left(left),.target_speed_right(right),
        .x_mm(px),.y_mm(py),.heading_mdeg(yaw),.gyro_calibrated(calibrated),.calibrating(calibrating),
        .status(status),.fault(fault),.turn_complete_pulse(turn_done)
    );
    task check;
        input condition;input [511:0] message;
        begin checks=checks+1;if(!condition)$fatal(1,"%s phase%0d nav%0d fault%0d",message,phase,status,fault);end
    endtask
    task frame;
        begin
            @(negedge clk);
            if(run)begin enc_left=enc_left+left;enc_right=enc_right+right;gyro=(right-left)*60;end
            else gyro=0;
            gyro_sample=1;@(negedge clk);gyro_sample=0;
            tick=1;@(negedge clk);tick=0;
            repeat(1600)@(negedge clk);
            frames=frames+1;
            if(fault!=0)$fatal(1,"navigator arithmetic/overrun fault%0d",fault);
            delta=yaw-last_yaw;
            if(delta>180000)delta=delta-360000;
            if(delta< -180000)delta=delta+360000;
            sum_yaw=sum_yaw+delta;last_yaw=yaw;
        end
    endtask
    task start_demo;
        begin @(negedge clk);start=1;@(negedge clk);start=0;repeat(3)@(negedge clk);end
    endtask
    task stop_demo;
        begin @(negedge clk);cancel=1;@(negedge clk);cancel=0;repeat(5)@(negedge clk);end
    endtask
    initial begin
        repeat(4)@(negedge clk);reset_n=1;
        repeat(128)frame;
        check(calibrated,"actual gyro bias calibration");
        start_demo;
        repeat(500)begin frame;check(!run && left==0 && right==0 && !servo,"actual navigator quiet during5s");end
        frame;frame;check(phase==3 && run && left==10 && right==10,"known-free guarded forward accepted");
        front_stop=1;frame;check(!run && status==7,"actual close obstacle blocks forward");
        held_elapsed=demo.elapsed;repeat(10)frame;
        check(demo.elapsed==held_elapsed,"script timer paused by real navigator guard");
        front_stop=0;frame;frame;check(run && phase==3,"same script resumes when real obstacle clears");
        // Finish route using raw gyro from the ideal differential wheel plant.
        k=0;while(phase!=12 && phase!=13 && k<3000)begin frame;k=k+1;end
        check(phase==12 && !run,"task5 completes through actual gyro turns");
        check(sum_yaw>=-4000 && sum_yaw<=4000,"task5 returns to original heading");
        stop_demo;search=1;start_demo;sum_yaw=0;last_yaw=yaw;
        repeat(501)frame;
        turn_stop=1;frame;frame;
        check(!run && status==7,"real turn guard retained");
        turn_stop=0;
        k=0;while(phase!=12 && phase!=13 && k<4000)begin frame;k=k+1;end
        check(phase==12 && !run,"search sequence completes through actual gyro turns");
        check(sum_yaw>285000 && sum_yaw<300000,"unwrapped total is about+292deg, never-68deg");
        stop_demo;search=0;coverage=14;start_demo;repeat(505)frame;
        check(!run && status==8 && phase==3,"missing front evidence stays UNKNOWN and stationary");
        coverage=15;frame;frame;check(run,"known valid coverage restores same pending straight leg");
        lidar=0;frame;check(phase==13 && !run && !servo,"lost lidar aborts script without restart");
        lidar=1;repeat(5)frame;check(!run && phase==13,"lidar recovery alone does not relaunch");
        stop_demo;start_demo;repeat(501)frame;stop_demo;repeat(10)frame;
        check(!run && !engaged && left==0 && right==0,"STOP leaves actual navigator quiet");
        $display("PASS: preset + actual navigator %0d checks, %0d frames; raw gyro300deg, guards, resume, STOP",checks,frames);
        $finish;
    end
    initial begin #500000000;$fatal(1,"integration timeout");end
endmodule
