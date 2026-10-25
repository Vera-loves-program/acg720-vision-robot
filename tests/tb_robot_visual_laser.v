`timescale 1ns/1ps
// Offline testbench for an HDL simulator. No vendor clock/IP or device access.
// This file is not added to the synthesis project.
module tb_robot_visual_laser;
    reg clk=0;always #10 clk=~clk;
    reg reset_n=0,scan_start=0,point_valid=0,query_valid=0,test_clear=0;
    reg signed [15:0] point_x=0,point_y=0;
    reg signed [31:0] heading=0;
    reg [255:0] query=0;
    wire valid;
    wire [255:0] telemetry;
    wire [15:0] target;
    wire signed [31:0] bearing;
    wire [7:0] status,clusters,points;
    wire signed [15:0] bx,by;
    robot_visual_laser dut(
        .clk(clk),.reset_n(reset_n),.tick_10ms(1'b0),.clear(test_clear),
        .scan_start(scan_start),.point_valid(point_valid),.point_x(point_x),.point_y(point_y),
        .speed_left(16'sd0),.speed_right(16'sd0),.pose_valid(1'b1),.laser_ready(1'b1),
        .pose_x(32'sd0),.pose_y(32'sd0),.heading(heading),.query_valid(query_valid),.query(query),
        .telemetry(telemetry),.valid(valid),.target_id(target),.bearing_world(bearing),.result_status(status),
        .body_x(bx),.body_y(by),.cluster_count(clusters),.beam_points(points)
    );
    task boundary;
        begin @(negedge clk);scan_start=1;@(negedge clk);scan_start=0;repeat(240)@(negedge clk);end
    endtask
    task point;
        input integer x,y;
        begin @(negedge clk);point_x=x;point_y=y;point_valid=1;
            @(negedge clk);point_valid=0;repeat(8)@(negedge clk);end
    endtask
    task dog;
        integer y;
        begin for(y=-60;y<=60;y=y+20)point(1000,y);end
    endtask
    integer y;
    initial begin
        repeat(3)@(negedge clk);reset_n=1;
        query[255:224]=32'h56454331;query[223:216]=1;query[208]=1;
        query[191:176]=1;query[159:144]=16384;query[63:48]=261;
        query_valid=1;@(negedge clk);query_valid=0;repeat(3)@(negedge clk);
        boundary;dog;boundary;
        if(valid || status!=3)$fatal(1,"first scan must require stability");
        dog;boundary;
        if(!valid || bx!=1000 || by!=0 || target!=1 || bearing>20 || bearing< -20)
            $fatal(1,"unique stable dog or laser bearing failed");
        for(y=-1200;y<=1200;y=y+20)point(1000,y);
        boundary;
        if(valid || status!=1)$fatal(1,"wall fragment accepted as dog");
        for(y=-40;y<=40;y=y+20)point(600,y);
        dog;boundary;
        if(valid || status!=2 || clusters!=2)$fatal(1,"foreground ambiguity accepted");
        heading=90000;boundary;dog;boundary;dog;boundary;
        if(!valid || bearing<89980 || bearing>90020)$fatal(1,"scan-time gyro fusion failed");
        // An impossible-on-C1 point overrun must invalidate the complete
        // scan, rather than omit evidence and call the remainder unique.
        @(negedge clk);point_x=1000;point_y=0;point_valid=1;
        @(negedge clk);point_y=20;
        @(negedge clk);point_valid=0;repeat(12)@(negedge clk);boundary;
        if(valid || status!=5)$fatal(1,"overloaded point stream accepted");
        // A newer query/pose during publication cannot replace the frozen
        // completed scan. It may mask valid, but fields remain coherent.
        dog;
        @(negedge clk);scan_start=1;
        @(negedge clk);scan_start=0;heading=-90000;query[191:176]=2;query_valid=1;
        @(negedge clk);query_valid=0;repeat(5)@(negedge clk);
        if(target!=1 || telemetry[127:96]!=32'sd90000 || valid)
            $fatal(1,"publication mixed next query/pose with past scan");
        // Clear between boundary capture and commit must cancel publication.
        @(negedge clk);scan_start=1;
        @(negedge clk);scan_start=0;test_clear=1;
        @(negedge clk);test_clear=0;repeat(240)@(negedge clk);
        if(valid || status!=0 || dut.publish_state!=0 || dut.prior_good)
            $fatal(1,"clear allowed pending scan resurrection");
        $display("PASS unique cluster, stability, wall rejection, ambiguity, gyro bearing, overrun, coherent publication, clear race");
        $finish;
    end
    initial begin #1000000;$fatal(1,"timeout");end
endmodule
