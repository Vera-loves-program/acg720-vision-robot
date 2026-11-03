`timescale 1ns/1ps
// Public-touch UI plus actual bounded-goal RTL. No motor/device access.
module tb_robot_avoid_ui;
    reg clk=0;always #10 clk=~clk;
    reg reset_n=0,start=0,cancel=0;
    reg signed [31:0] px=0,py=0,yaw=0;
    wire done,error;
    wire signed [31:0] gx,gy;
    robot_avoid_forward_goal goal_dut(
        .clk(clk),.reset_n(reset_n),.start(start),.cancel(cancel),
        .pose_x(px),.pose_y(py),.heading_mdeg(yaw),
        .done(done),.error(error),.goal_x(gx),.goal_y(gy)
    );
    reg snapshot=0,snapshot_cancel=0,remote=0,ready=1,safety=0,motor_fault=0;
    reg gains=1,gyro=1,lidar=1;
    reg [1:0] contacts=0;
    reg [10:0] x=0,y=0;
    wire [2:0] page,subpage,command;
    wire command_valid,armed;
    wire signed [31:0] ui_gx,ui_gy;
    wire [3:0] notice;
    integer go_events=0,turn_events=0,k,expected_x,expected_y,count,delay,expected_events;
    real rad;
    robot_ui_control #(.CLK_HZ(1000)) ui(
        .clk(clk),.reset_n(reset_n),.safety_stop(safety),.motor_fault(motor_fault),.tick_10ms(1'b0),
        .remote_takeover(remote),.remote_active(1'b0),.touch_ready(ready),
        .snapshot_valid(snapshot),.snapshot_cancel(snapshot_cancel),.contact_count(contacts),
        .x0(x),.y0(y),.x1(11'd0),.y1(11'd0),.id0(4'd1),
        .live_x_mm(px),.live_y_mm(py),.live_heading_mdeg(yaw),
        .motor_selected_wheel(1'b0),.motor_edit_position(1'b0),
        .gains_ready(gains),.gyro_ready(gyro),.lidar_ready(lidar),
        .task_valid(1'b0),.task_action(4'd0),.task_value(32'sd0),.task_visual_ready(3'd0),
        .page(page),.subpage(subpage),.command_valid(command_valid),.command(command),
        .armed(armed),.goal_x_mm(ui_gx),.goal_y_mm(ui_gy),.ui_notice(notice)
    );
    always @(posedge clk)if(reset_n && command_valid && command==3)go_events=go_events+1;
    always @(posedge clk)if(reset_n && command_valid && command==2)turn_events=turn_events+1;
    task pulse_start;
        begin @(negedge clk);start=1;@(negedge clk);start=0;end
    endtask
    task wait_goal;
        begin
            count=0;
            while(!done && count<160)begin @(negedge clk);count=count+1;end
            if(!done)$fatal(1,"goal projection timeout");
        end
    endtask
    task touch;
        input integer tx,ty,n;
        begin
            @(negedge clk);x=tx;y=ty;contacts=n;snapshot=1;
            @(negedge clk);snapshot=0;
            repeat(2)@(negedge clk);
        end
    endtask
    task tap;
        input integer tx,ty;
        begin touch(tx,ty,1);touch(tx,ty,0);end
    endtask
    task release_view;
        begin repeat(160)@(negedge clk);touch(0,0,0);end
    endtask
    task reset_ui;
        begin
            @(negedge clk);reset_n=0;remote=0;ready=1;safety=0;motor_fault=0;
            gains=1;gyro=1;lidar=1;snapshot_cancel=0;contacts=0;snapshot=0;
            repeat(3)@(negedge clk);reset_n=1;release_view;
            tap(100,260);release_view;
            if(page!=2 || subpage!=3)$fatal(1,"task3 menu route failed");
        end
    endtask
    initial begin
        repeat(3)@(negedge clk);reset_n=1;
        for(k=-180000;k<=180000;k=k+100)begin
            px=100;py=-200;yaw=k;pulse_start;
            // A live pose update must not move the captured target.
            px=1000;py=1200;yaw=0;wait_goal;
            rad=k*3.141592653589793/180000.0;
            expected_x=$rtoi(100+1000*$cos(rad));expected_y=$rtoi(-200+1000*$sin(rad));
            if(error || gx<expected_x-2 || gx>expected_x+2 || gy<expected_y-2 || gy>expected_y+2)
                $fatal(1,"forward goal yaw=%0d expected=(%0d,%0d) actual=(%0d,%0d)",k,expected_x,expected_y,gx,gy);
            @(negedge clk);
        end
        px=3500;py=0;yaw=0;pulse_start;wait_goal;
        if(!error)$fatal(1,"out-of-field goal accepted");
        for(delay=0;delay<105;delay=delay+1)begin
            px=0;py=0;yaw=45000;pulse_start;
            repeat(delay)@(negedge clk);cancel=1;@(negedge clk);cancel=0;
            repeat(160)begin @(negedge clk);if(done)$fatal(1,"cancelled goal escaped");end
        end
        px=0;py=0;yaw=90000;reset_ui;expected_events=go_events;
        tap(450,150);repeat(160)@(negedge clk);
        if(!armed || go_events!=expected_events+1 || ui_gx< -2 || ui_gx>2 || ui_gy<998 || ui_gy>1002)
            $fatal(1,"task3 did not send a captured one-metre GOTO");
        // STOP, navigation away and readiness/takeover loss during projection
        // must cancel it even when permissions return before completion.
        for(k=0;k<9;k=k+1)begin
            reset_ui;expected_events=go_events;
            tap(450,150);repeat(5)@(negedge clk);
            case(k)
                0:touch(650,150,1);
                1:tap(850,80);
                2:begin gyro=0;@(negedge clk);gyro=1;end
                3:begin lidar=0;@(negedge clk);lidar=1;end
                4:begin gains=0;@(negedge clk);gains=1;end
                5:begin remote=1;@(negedge clk);remote=0;end
                6:begin ready=0;@(negedge clk);ready=1;end
                7:begin safety=1;@(negedge clk);safety=0;end
                8:begin motor_fault=1;@(negedge clk);motor_fault=0;end
            endcase
            repeat(180)@(negedge clk);
            if(armed || go_events!=expected_events)$fatal(1,"UI cancellation escaped: case=%0d",k);
        end
        reset_ui;px=3500;py=0;yaw=0;expected_events=go_events;
        tap(450,150);repeat(160)@(negedge clk);
        if(armed || go_events!=expected_events || notice!=10)$fatal(1,"UI out-of-field goal failed");
        px=0;tap(850,80);release_view;tap(450,260);release_view;
        if(page!=4 || subpage!=1)$fatal(1,"NAV menu route failed");
        expected_events=turn_events;lidar=0;tap(450,150);clocks_after_turn;
        if(armed || turn_events!=expected_events || notice!=5)$fatal(1,"NAV turn accepted stale lidar");
        lidar=1;tap(450,150);clocks_after_turn;
        if(!armed || turn_events!=expected_events+1)$fatal(1,"NAV turn rejected ready lidar");
        $display("PASS: task3 3601 captured heading goals, field bounds, 105 phase cancellations, public touch GOTO and 9 STOP/takeover/readiness cancellations");
        $display("PASS: public NAV turn buttons require live lidar");
        $finish;
    end
    task clocks_after_turn;
        begin repeat(4)@(negedge clk);end
    endtask
    initial begin #50000000;$fatal(1,"timeout");end
endmodule
