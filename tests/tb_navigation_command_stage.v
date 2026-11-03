`timescale 1ns/1ps
module tb_navigation_command_stage;
    reg clk=0;always #5 clk=~clk;
    reg reset_n=0,cancel=0,ie=0,iv=0,ih=0,ia=0;
    reg [2:0] ic=0;reg [1:0] ib=0;
    reg signed [15:0] il=0,ir=0;
    reg signed [31:0] iyaw=0,ix=0,iy=0;
    reg [15:0] icruise=0,iturn=0,itol=0;
    wire en,v,h,a;wire [2:0] c;wire [1:0] b;
    wire signed [15:0] l,r;wire signed [31:0] yaw,x,y;
    wire [15:0] cruise,turn_speed,tol;
    integer k,checks=0;
    robot_navigation_command_stage dut(
        .clk(clk),.reset_n(reset_n),.cancel(cancel),
        .in_enable(ie),.in_valid(iv),.in_hold(ih),.in_avoid(ia),.in_command(ic),
        .in_half_direction(ib),.in_left(il),.in_right(ir),.in_heading(iyaw),
        .in_goal_x(ix),.in_goal_y(iy),.in_cruise(icruise),.in_turn(iturn),.in_tolerance(itol),
        .enable(en),.valid(v),.hold_heading(h),.avoid(a),.command(c),.half_direction(b),
        .left_speed(l),.right_speed(r),.heading(yaw),.goal_x(x),.goal_y(y),
        .cruise(cruise),.turn_speed(turn_speed),.tolerance(tol)
    );
    task check;input condition;input [511:0] message;
        begin checks=checks+1;if(!condition)$fatal(1,"%s",message);end endtask
    initial begin
        repeat(3)@(negedge clk);reset_n=1;
        for(k=0;k<200;k=k+1)begin
            @(negedge clk);ie=1;iv=1;ic=k%5;ih=k%2;ia=(k%3)!=0;ib=k%3;
            il=k-100;ir=100-k;iyaw=k*1000-100000;ix=k*31;iy=-k*17;
            icruise=10+k;iturn=12+k;itol=5+k;
            @(posedge clk);#1;
            check({en,v,c,h,a,b,l,r,yaw,x,y,cruise,turn_speed,tol}===
                  {ie,iv,ic,ih,ia,ib,il,ir,iyaw,ix,iy,icruise,iturn,itol},
                  "command and all associated data stay atomic on adjacent cycles");
        end
        @(negedge clk);cancel=1;ic=1;iv=1;#1;
        check(!en && !v,"queued motion is suppressed before the next clock");
        @(posedge clk);#1;check(!en && !v,"cancel prevents capturing new motion");
        @(negedge clk);cancel=0;iv=0;#1;check(!v,"release cannot unmask cancelled command");
        @(posedge clk);#1;check(!v,"no delayed launch after cancellation");
        @(negedge clk);cancel=1;ic=0;iv=1;
        @(posedge clk);#1;check(v && c==0 && !en,"STOP still reaches navigator while cancelled");
        @(negedge clk);ic=2;iv=1;
        @(posedge clk);#1;check(!v && !en,"TURN cannot replace STOP while cancelled");
        @(negedge clk);cancel=0;iv=0;
        @(posedge clk);#1;check(!v,"no TURN replay after cancellation");
        @(negedge clk);reset_n=0;#1;check(!en && !v && yaw==0,"reset clears staged state");
        $display("PASS: navigation command stage %0d checks; atomic data, immediate cancellation, STOP survives, no replay",checks);
        $finish;
    end
endmodule
