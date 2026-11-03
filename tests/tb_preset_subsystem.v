`timescale 1ns/1ps
// Ownership integration with actual UI -> subsystem -> navigation ports.
// Sensor readiness/nav outputs are controlled fixtures in this bench only;
// tb_preset_navigation separately exercises the actual guarded navigator.
module tb_preset_subsystem;
    GSR GSR(.GSRI(1'b1));
    reg clk=0;always #10 clk=~clk;
    reg reset_n=0,tick=0,snapshot=0,safety=0;
    reg [1:0] contacts=0;
    reg [10:0] x=0,y=0;
    wire [2:0] page,subpage;
    wire drive,pan,tilt;
    integer checks=0,k;
    reg saw_start_command=0,saw_stop_command=0;
    robot_subsystem dut(
        .clk(clk),.reset_n(reset_n),.pixel_clk(clk),.safety_stop(safety),
        .remote_takeover(1'b0),.remote_active(1'b0),.remote_turn_request(1'b0),
        .task_valid(1'b0),.task_action(4'd0),.task_value(32'sd0),.remote_turn_degrees(16'sd0),
        .turn_complete_ready(1'b0),.turn_report_busy(1'b0),
        .motor_fault_left(8'd0),.motor_fault_right(8'd0),.motor_armed(1'b0),.motor_running(1'b0),
        .motor_selected_wheel(1'b0),.motor_edit_position(1'b0),.motor_mode_position(1'b0),.motor_link_fault(1'b0),
        .gains_ready(1'b1),.tick_10ms(tick),.encoder_zeroed(1'b0),
        .position_left(32'sd0),.position_right(32'sd0),.speed_left(32'sd0),.speed_right(32'sd0),
        .gyro_z(16'sd0),.gyro_valid(1'b1),.gyro_samples(32'd0),.touch_ready(1'b1),
        .snapshot_valid(snapshot),.snapshot_cancel(1'b0),.contact_count(contacts),
        .x0(x),.y0(y),.x1(11'd0),.y1(11'd0),.id0(4'd1),.lidar_rx(1'b1),
        .page(page),.subpage(subpage),.drive_enable(drive),.map_rd_en(1'b0),
        .map_rd_x(7'd0),.map_rd_y(7'd0),.follow_request(1'b0),.follow_data(256'd0),
        .follow_pan_pwm(pan),.follow_tilt_pwm(tilt),.battery_a_low(1'b0),.status_byte_ready(1'b1)
    );
    defparam dut.u_controls.CLK_HZ=1000;
    defparam dut.u_status.CLK_HZ=1000;
    always @(negedge clk)if(reset_n && dut.u_navigation.command_valid)begin
        if(dut.u_navigation.command==1)saw_start_command=1;
        if(dut.u_navigation.command==0)saw_stop_command=1;
    end
    task check;
        input condition;input [511:0] message;
        begin checks=checks+1;if(!condition)$fatal(1,"%s page%0d/%0d phase%0d",message,page,subpage,dut.demo_phase);end
    endtask
    task wait_ui;
        begin
            repeat(500)@(negedge clk);
            // Touch controller continues publishing an all-up frame after
            // the page-change cooldown, as it does on the physical LCD.
            snapshot=1;contacts=0;@(negedge clk);snapshot=0;
            repeat(3)@(negedge clk);
        end
    endtask
    task touch;
        input integer tx,ty,n;
        begin
            @(negedge clk);x=tx;y=ty;contacts=n;snapshot=1;
            @(negedge clk);snapshot=0;repeat(3)@(negedge clk);
        end
    endtask
    task tap;
        input integer tx,ty;
        begin touch(tx,ty,1);touch(tx,ty,0);wait_ui;end
    endtask
    task one_tick;
        begin @(negedge clk);tick=1;@(negedge clk);tick=0;repeat(10)@(negedge clk);end
    endtask
    initial begin
        repeat(4)@(negedge clk);reset_n=1;touch(0,0,0);wait_ui;
        force dut.gyro_ready=1;force dut.calibrating=0;force dut.avoid_lidar_ready=1;
        force dut.body_guard_ready=1;force dut.nav_fault=0;
        force dut.nav_run=1;force dut.nav_status=3;force dut.robot_heading=0;
        force dut.normal_pan_pwm=1;force dut.normal_tilt_pwm=1;
        tap(200,392);check(page==5 && subpage==0,"task5 entrance retained");
        tap(680,240);check(dut.demo_phase==1 && dut.demo_seconds==5,"real PRESET button starts countdown");
        check(saw_stop_command && !saw_start_command,"start stops old navigation, no early MANUAL");
        for(k=0;k<500;k=k+1)begin
            one_tick;check(!drive && !pan && !tilt,"countdown masks stale nav/Pi requests");
        end
        one_tick;check(drive && saw_start_command,"new demo command reaches actual navigation port");
        check(dut.u_navigation.avoid_enable && dut.u_navigation.manual_heading_hold,"safety and heading hold remain enabled");
        check(dut.u_navigation.manual_left==10 && dut.u_navigation.manual_right==10,"script speed uses navigator");
        check(dut.u_status.live_title==110,"active LCD explicitly says PRESET DEMO");
        // STOP is caught on contact, before release/debounce.
        touch(480,240,1);check(!drive,"task5 STOP immediately cancels motor permission");
        touch(480,240,0);wait_ui;
        check(!dut.demo_engaged && !pan && !tilt,"old Pi PWM does not resume after cancellation");
        repeat(510)one_tick;check(!dut.demo_engaged,"no delayed resurrection after STOP");
        tap(480,150);check(!dut.demo_head_hold,"explicit real tracking action releases servo hold");
        tap(680,240);one_tick;tap(900,90);check(page==0 && !drive && !dut.demo_engaged,"HOME cancels countdown");
        tap(600,392);check(page==5 && subpage==3,"task6 entrance retained");
        tap(680,240);check(dut.demo_phase==1 && dut.u_demo.mode,"task6 PRESET starts search script, no navigation to submenu");
        safety=1;repeat(3)@(negedge clk);check(!drive && !pan && !tilt && !dut.demo_engaged,"safety stop wins");
        safety=0;wait_ui;
        tap(900,90);tap(200,392);
        tap(480,330);check(page==5 && subpage==4,"side follow remains available via task5 slot5");
        tap(900,160);check(page==5 && subpage==0,"side follow BACK returns to task5");
        $display("PASS: UI + preset subsystem %0d checks; real buttons, ownership, labelled status, cancellation",checks);
        $finish;
    end
    initial begin #100000000;$fatal(1,"subsystem test timeout");end
endmodule
