`timescale 1ns/1ps
module tb_map_ray_evidence;
 GSR GSR(.GSRI(1'b1));
 reg clk=0;always #10 clk=~clk;
 reg rst=0,start=0,pv=0,freeze=0,rd=0;
 reg[15:0] sid=0,distance=12000,angle=0;reg[5:0] quality=19;
 reg[6:0] x=0,y=64;wire[1:0] observed_cell;wire rv,frozen,ready,busy;wire[31:0] scans,points;
 integer cases=0,waits;
 lidar_grid_map #(.FRAME_POINTS(8)) dut(.clk(clk),.reset_n(rst),.enable(1'b1),.clear_map(1'b0),
 .scan_id(sid),.point_valid(pv),.scan_start(start),.angle_q6(angle),.distance_mm(distance),.quality(quality),
 .pose_valid(1'b1),.local_scan(1'b0),.pose_x_mm(32'sd0),.pose_y_mm(32'sd0),.pose_heading_q6(16'd0),
 .forward_angle_q6(15'd0),.lidar_offset_x_mm(16'sd0),.lidar_offset_y_mm(16'sd0),
 .body_front_mm(16'd0),.body_rear_mm(16'd0),.body_left_mm(16'd0),.body_right_mm(16'd0),
 .correction_enable(1'b0),.correction_ack(1'b0),.freeze_map(freeze),.map_frozen(frozen),
 .plan_rd_en(rd),.plan_rd_x(x),.plan_rd_y(y),.plan_rd_cell(observed_cell),.plan_rd_valid(rv),
 .map_ready(ready),.busy(busy),.scan_count(scans),.point_count(points),
 .rd_clk(clk),.rd_en(1'b0),.rd_x(7'd0),.rd_y(7'd0));
 task step;begin @(negedge clk);end endtask
 task boundary;begin sid=sid+1;start=1;step;start=0;repeat(3)step;end endtask
 task beam;input[15:0] r;input[5:0] q;begin distance=r;quality=q;pv=1;step;pv=0;repeat(500)step;end endtask
 task finished;input[31:0] expected;begin
  waits=0;while((scans<expected || busy) && waits<300000)begin step;waits=waits+1;end
  if(waits==300000)$fatal(1,"map integration timeout %0d",scans);
 end endtask
 task check_observed_cell;input[6:0] cx;input[1:0] expected;begin
  freeze=1;waits=0;while(!frozen && waits<100000)begin step;waits=waits+1;end
  if(!frozen)$fatal(1,"freeze timeout");x=cx;rd=1;step;
  if(!rv || observed_cell!==expected)$fatal(1,"observed_cell%0d expected%0d got%0d valid%b",cx,expected,observed_cell,rv);
  rd=0;step;cases=cases+1;freeze=0;repeat(10)step;
 end endtask
 initial begin
  repeat(3)step;rst=1;repeat(18000)step;
  boundary;beam(12000,19);boundary;finished(1);
  check_observed_cell(64,1);check_observed_cell(100,1);check_observed_cell(127,1);
  // True endpoint observations mark the hit; farther measured rays clear it.
  boundary;beam(512,19);boundary;finished(2);check_observed_cell(72,2);
  boundary;beam(512,19);boundary;finished(3);check_observed_cell(72,3);
  boundary;beam(12000,19);boundary;finished(4);check_observed_cell(72,2);
  boundary;beam(12000,19);boundary;finished(5);check_observed_cell(72,1);check_observed_cell(127,1);
  // Invalid distance evidence does not fabricate a free or occupied endpoint.
  angle=5760;boundary;beam(1024,0);boundary;repeat(1000)step;y=56;check_observed_cell(64,0);
  $display("PASS map ray evidence %0d raw RAM checks; clipped far returns FREE, moved hits clear",cases);$finish;
 end
 initial begin #200000000;$fatal(1,"timeout");end
endmodule
