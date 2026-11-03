`timescale 1ns/1ps
module tb_robot_preset_demo;
    reg clk=0;always #5 clk=~clk;
    reg reset_n=0,tick=0,start=0,search=0,cancel=0,ready=1;
    reg signed [31:0] heading=0;
    reg nav_run=0,turn_done=0;
    reg [3:0] nav_status=0;
    reg [7:0] nav_fault=0;
    wire engaged,moving,valid,servo;
    wire [3:0] phase;
    wire [2:0] seconds_left,command;
    wire signed [15:0] speed;
    wire signed [31:0] target;
    wire [11:0] pan,tilt;
    integer checks=0,starts=0,turns=0,stops=0,k,drive_remaining,previous_turns,setup_stage;
    reg [3:0] before_phase;
    robot_preset_demo #(.DRIVE1_TICKS(3),.DRIVE2_TICKS(3),.DRIVE3_TICKS(3),
        .SEARCH_DRIVE_TICKS(3),.PAUSE_TICKS(2),.TURN_TIMEOUT_TICKS(30)) dut(
        .clk(clk),.reset_n(reset_n),.tick_10ms(tick),.start(start),.search_mode(search),
        .cancel(cancel),.ready(ready),.heading_mdeg(heading),.nav_run(nav_run),
        .turn_done(turn_done),.nav_status(nav_status),.nav_fault(nav_fault),
        .engaged(engaged),.moving_phase(moving),.phase(phase),.seconds_left(seconds_left),
        .command_valid(valid),.command(command),.speed(speed),.turn_heading(target),
        .servo_enable(servo),.pan_us(pan),.tilt_us(tilt)
    );
    task check;
        input condition;input [511:0] message;
        begin checks=checks+1;if(!condition)$fatal(1,"%s (phase=%0d)",message,phase);end
    endtask
    task clocks;input integer n;begin repeat(n)@(negedge clk);end endtask
    task pulse_start;
        begin @(negedge clk);start=1;@(negedge clk);start=0;clocks(2);end
    endtask
    task tick_once;
        begin @(negedge clk);tick=1;@(negedge clk);tick=0;clocks(4);end
    endtask
    task cancel_now;
        begin @(negedge clk);cancel=1;@(negedge clk);cancel=0;clocks(2);nav_status=0;nav_run=0;end
    endtask
    // Command-level navigator model is used ONLY in this sequencer unit test.
    // The separate integration bench uses the actual robot_navigation RTL.
    always @(negedge clk)if(reset_n && valid)begin
        case(command)
            0:begin stops=stops+1;nav_run=0;nav_status=0;end
            1:begin starts=starts+1;nav_run=1;nav_status=3;end
            2:begin turns=turns+1;nav_run=1;nav_status=4;end
            default:$fatal(1,"unexpected script command");
        endcase
    end
    task complete_turn;
        begin
            check(nav_status==4,"turn must have reached navigator");
            clocks(3);heading=target;nav_run=0;nav_status=6;
            // Completion pulse lies entirely between10ms ticks.
            @(negedge clk);turn_done=1;@(negedge clk);turn_done=0;
            tick_once;check(phase==6 || phase==10 || phase==15,"completion pulse latched");
        end
    endtask
    initial begin
        clocks(3);reset_n=1;clocks(2);
        pulse_start;
        for(k=0;k<499;k=k+1)begin
            tick_once;check(phase==1 && !moving && !servo && starts==0 && turns==0,"no motion during first499 ticks");
        end
        tick_once;check(phase==2 && starts==0 && !servo,"500th tick completes countdown only");
        tick_once;check(phase==3 && starts==1 && speed==10,"explicit straight motion after5s");
        // Counted travel time freezes while actual guard blocks the vehicle.
        tick_once;before_phase=phase;drive_remaining=dut.elapsed;
        nav_run=0;nav_status=7;
        repeat(12)tick_once;
        check(phase==before_phase && dut.elapsed==drive_remaining && !servo,"obstacle freezes travel time and head");
        nav_run=1;nav_status=3;tick_once;tick_once;
        check(phase==4,"same leg resumes after obstacle clears");
        tick_once;tick_once;
        check(phase==5 && target==30000,"task5 left30");complete_turn;
        tick_once;tick_once;check(phase==7,"task5 middle straight");
        repeat(3)tick_once;check(phase==8,"task5 second pause");
        tick_once;tick_once;check(phase==9 && target==0,"task5 right30 returns initial heading");complete_turn;
        tick_once;tick_once;check(phase==11,"task5 last straight");
        repeat(3)tick_once;check(phase==12 && !moving && !servo && !nav_run,"task5 terminal stop");
        cancel_now;check(!engaged,"cancel releases ownership");
        // Cancellation and restart require a fresh full500 ticks.
        pulse_start;repeat(250)tick_once;cancel_now;
        repeat(510)tick_once;check(!engaged && !servo && !nav_run,"cancelled countdown never resurrects");
        heading=170000;search=1;pulse_start;repeat(500)tick_once;
        tick_once;check(phase==5 && target== -40000,"search first +150 wraps through180");complete_turn;
        tick_once;tick_once;check(phase==9 && target==110000,"search second +150, not shortest -60");complete_turn;
        tick_once;tick_once;check(phase==14 && target==102000,"search fine adjustment -8");complete_turn;
        tick_once;tick_once;check(phase==11,"search final straight");
        repeat(3)tick_once;check(phase==12 && !servo && !nav_run,"search terminal stop");
        cancel_now;search=0;ready=0;pulse_start;repeat(510)tick_once;
        check(phase==2 && !servo && !nav_run,"sensors missing waits without movement");
        ready=1;tick_once;check(phase==3,"readiness permits launch after countdown");
        ready=0;clocks(4);check(phase==13 && !servo && !nav_run,"sensor loss aborts, does not queue restart");
        ready=1;repeat(8)tick_once;check(phase==13,"recovery requires explicit restart after fault");
        cancel_now;pulse_start;repeat(501)tick_once;
        nav_fault=1;clocks(4);check(phase==13 && !nav_run,"navigation fault stops script");
        nav_fault=0;cancel_now;search=1;pulse_start;repeat(501)tick_once;
        repeat(31)tick_once;check(phase==13 && !nav_run,"turn completion timeout stops");
        cancel_now;
        @(negedge clk);start=1;cancel=1;@(negedge clk);start=0;cancel=0;clocks(3);
        check(!engaged,"simultaneous cancel wins over start");
        // Negative wrap exercises the narrowed arithmetic's opposite bound.
        heading=-115000;search=1;pulse_start;repeat(501)tick_once;
        check(target==35000,"negative initial heading +150");complete_turn;
        tick_once;tick_once;check(target== -175000,"second150 wraps negative");complete_turn;
        tick_once;tick_once;check(target==177000,"fine adjustment crosses negative180 boundary");complete_turn;
        cancel_now;
        // Cancel on each added register stage. No queued TURN may escape
        // after a STOP even though target addition has already completed.
        for(setup_stage=1;setup_stage<=2;setup_stage=setup_stage+1)begin
            previous_turns=turns;pulse_start;repeat(500)tick_once;
            @(negedge clk);tick=1;@(negedge clk);tick=0;
            if(setup_stage==2)clocks(1);
            cancel=1;@(negedge clk);cancel=0;clocks(5);
            check(turns==previous_turns && !engaged && !nav_run,"STOP kills pipelined pending turn");
        end
        $display("PASS: preset sequencer %0d checks; full5s, two150 turns, pause/resume, cancellation, faults",checks);
        $finish;
    end
    initial begin #2000000;$fatal(1,"preset test timeout");end
endmodule
