`timescale 1ns/1ps
// Clocked integration: actual guard -> source-extracted ROOT lookahead pipeline
// -> actual robot_navigation. The extractor wraps existing RTL, not a model.
module tb_lidar_lookahead;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,tick=0,sample=0,pv=0,raw=0,start=0,complete=0,cv=0;
 reg[2:0] cmd=3;reg[14:0] angle=0;reg[15:0] distance=0,scan_id=0;
 reg[5:0] quality=20;reg[2:0] reason=0;
 reg signed[31:0] goal=1000;
 wire ready,front_stop,rear_stop,slow,turn_stop,contact,mv,confirmed,ahead,run,calibrated;
 wire[3:0] free_cov,nav_status;wire signed[15:0] bx,by,left_speed,right_speed;
 integer checks=0,mapped=0,confirmed_count=0,budget;
 lidar_body_guard guard(
  .clk(clk),.reset_n(rst),.tick_10ms(tick),.active(1'b1),.point_valid(pv),.scan_start(start),
  .angle_q6(angle),.forward_angle_q6(15'd0),.distance_mm(distance),.quality(quality),
  .raw_node_valid(raw),.raw_reject_reason(reason),.raw_parse_error(1'b0),.scan_id(scan_id),
  .offset_x_mm(16'sd33),.offset_y_mm(-16'sd77),.body_front_mm(16'd232),.body_rear_mm(16'd118),
  .body_left_mm(16'd119),.body_right_mm(16'd119),.drive_margin_mm(16'd20),.turn_margin_mm(16'd30),.contact_margin_mm(16'd10),
  .ready(ready),.front_stop(front_stop),.rear_stop(rear_stop),.front_slow(slow),.turn_stop(turn_stop),.virtual_contact(contact),
  .free_coverage(free_cov),.map_point_valid(mv),.confirmed_point_valid(confirmed),.body_point_x(bx),.body_point_y(by)
 );
 actual_root_lookahead lookahead(
  .clk(clk),.reset_n(rst),.map_reset(1'b0),.avoid_lidar_ready(1'b1),.body_guard_ready(ready),
  .scan_start(start),.scan_complete(complete),.nav_status(nav_status),
  .ui_command_valid(1'b0),.follow_command_valid(1'b0),.mission_command_valid(cv),
  .ui_command(3'd0),.follow_command(3'd0),.mission_command(cmd),
  .mission_goal_x(goal),.mission_goal_y(32'sd0),.robot_x(32'sd0),.robot_y(32'sd0),
  .body_front(16'd232),.body_left(16'd119),.body_right(16'd119),.drive_margin(16'd20),
  .body_point_x(bx),.body_point_y(by),.map_point_valid(mv),.confirmed_point_valid(confirmed),.nav_obstacle_ahead(ahead)
 );
 robot_navigation #(.BODY_GUARD(1)) nav(
  .clk(clk),.reset_n(rst),.tick_10ms(tick),.safety_stop(1'b0),.motion_enable(1'b1),
  .encoder_position_left(32'sd0),.encoder_position_right(32'sd0),.encoder_zeroed(1'b0),
  .gyro_z(16'sd0),.gyro_sample(sample),.gyro_valid(1'b1),.gyro_invert(1'b0),
  .geometry_valid(1'b1),.wheel_mm_per_rev(16'd204),.encoder_counts_per_rev(16'd1469),.track_mm(16'd213),
  .pose_set(1'b0),.pose_x_mm(32'sd0),.pose_y_mm(32'sd0),.pose_heading_mdeg(32'sd0),
  .command_valid(cv),.command(cmd),.manual_left(16'sd10),.manual_right(16'sd10),.manual_heading_hold(1'b0),
  .turn_heading_mdeg(32'sd0),.goal_x_mm(goal),.goal_y_mm(32'sd0),.turn_half_direction(2'd0),
  .cruise_counts(16'd30),.turn_counts(16'd12),.arrival_tolerance_mm(16'd5),
  .avoid_enable(1'b1),.lidar_valid(1'b1),.clear_front_mm(16'd1000),.clear_left_mm(16'd1000),
  .clear_right_mm(16'd1000),.clear_rear_mm(16'd1000),.stop_clearance_mm(16'd0),.slow_clearance_mm(16'd0),.rotate_clearance_mm(16'd0),
  .body_guard_ready(ready),.body_front_stop(front_stop),.body_rear_stop(rear_stop),.body_front_slow(slow),
  .body_turn_stop(turn_stop),.body_nav_ahead(ahead),.body_coverage(free_cov),
  .lidar_correction_valid(1'b0),.lidar_correction_x_mm(16'sd0),.lidar_correction_y_mm(16'sd0),
  .run_request(run),.target_speed_left(left_speed),.target_speed_right(right_speed),.gyro_calibrated(calibrated),.status(nav_status)
 );
 always @(posedge clk)begin if(mv)mapped=mapped+1;if(confirmed)confirmed_count=confirmed_count+1;end
 task step;begin @(negedge clk);end endtask
 task check;input condition;input[511:0] message;begin checks=checks+1;if(condition!==1'b1)$fatal(1,"%0s status=%0d ahead=%b ready=%b",message,nav_status,ahead,ready);end endtask
 task idle;begin budget=0;while(guard.state!=6 && budget<600)begin step;budget=budget+1;end check(budget<600,"guard service timeout");step;end endtask
 task boundary;begin idle;scan_id=scan_id+1;start=1;step;start=0;step;complete=1;step;complete=0;step;end endtask
 task node;input integer a,d;begin idle;angle=a;distance=d;quality=20;reason=0;raw=1;pv=1;step;raw=0;pv=0;idle;step;end endtask
 task frame;begin sample=1;step;sample=0;tick=1;step;tick=0;repeat(2200)step;end endtask
 task issue;input[2:0] value;begin cmd=value;cv=1;step;cv=0;end endtask
 task setup;
  integer bin,a;
  begin
   rst=0;pv=0;raw=0;start=0;complete=0;cv=0;goal=1000;repeat(4)step;rst=1;
   repeat(128)begin sample=1;step;sample=0;step;end
   check(calibrated,"calibrate gyro from normal inputs");idle;boundary;
   for(bin=0;bin<128;bin=bin+1)begin
    a=(bin*180+30-2880+23040)%23040;node(a,1000);
    a=(bin*180+100-2880+23040)%23040;node(a,1010);
   end
   boundary;check(ready && free_cov==15,"actual clear scan establishes observation support");
   issue(3);frame;frame;check(run && left_speed>0 && right_speed>0,"actual NAV GOTO is driving");
  end
 endtask
 integer saved_map,saved_confirmed;
 initial begin
  setup;saved_map=mapped;saved_confirmed=confirmed_count;
  node(0,417);
  check(mapped==saved_map+1,"isolated point preserved in raw map stream");
  check(confirmed_count==saved_confirmed && !ahead && run,"isolated450mm return cannot trigger NAV lookahead");
  node(64,417);
  check(ahead && !run && nav_status==7 && !front_stop,"confirmed450mm pair parks actual NAV through root lookahead");

  setup;node(0,417);node(0,500);
  check(!ahead && run,"range-inconsistent forward returns remain unconfirmed");
  node(0,200);check(contact && ahead && !run,"single233mm near contact retains immediate stop");

  setup;node(5760,600);node(5824,600);
  check(!ahead && run,"confirmed side pair outside drive corridor does not park NAV");
  goal=300;issue(3);frame;frame;check(run,"short new goal starts");
  node(0,567);node(64,567);
  check(!ahead && run,"confirmed point beyond final stopping envelope does not block shorter trip");
  issue(0);frame;check(!run,"manual STOP remains effective");
  $display("PASS tb_lidar_lookahead: %0d clocked assertions; actual guard/root lookahead/NAV, isolated-noise negative control, pair/contact/envelope/STOP",checks);$finish;
 end
 initial begin #100000000;$fatal(1,"lookahead TB watchdog");end
endmodule
