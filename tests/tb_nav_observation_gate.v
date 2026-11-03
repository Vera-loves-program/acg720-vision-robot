`timescale 1ns/1ps
module tb_nav_observation_gate;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,tick=0,sample=0,gv=1,lv=1,guard_ready=1,front_stop=0,turn_stop=0;
 reg cv=0;reg[2:0] cmd=1;reg[3:0] coverage=0;
 wire run,calibrated;wire[3:0] status;wire[7:0] fault;
 wire signed[15:0] ls,rs;wire[3:0] required;
 wire measurement_unknown;
 integer cases=0;
 robot_navigation #(.BODY_GUARD(1)) dut(
  .clk(clk),.reset_n(rst),.tick_10ms(tick),.safety_stop(1'b0),.motion_enable(1'b1),
  .encoder_position_left(32'sd0),.encoder_position_right(32'sd0),.encoder_zeroed(1'b0),
  .gyro_z(16'sd0),.gyro_sample(sample),.gyro_valid(gv),.gyro_invert(1'b0),
  .geometry_valid(1'b1),.wheel_mm_per_rev(16'd204),.encoder_counts_per_rev(16'd1469),.track_mm(16'd213),
  .pose_set(1'b0),.pose_x_mm(32'sd0),.pose_y_mm(32'sd0),.pose_heading_mdeg(32'sd0),
  .command_valid(cv),.command(cmd),.manual_left(16'sd10),.manual_right(16'sd10),.manual_heading_hold(1'b0),
  .turn_heading_mdeg(32'sd90000),.goal_x_mm(32'sd1000),.goal_y_mm(32'sd0),.turn_half_direction(2'd0),
  .cruise_counts(16'd30),.turn_counts(16'd12),.arrival_tolerance_mm(16'd5),
  .avoid_enable(1'b1),.lidar_valid(lv),.clear_front_mm(16'd1000),.clear_left_mm(16'd1000),
  .clear_right_mm(16'd1000),.clear_rear_mm(16'd1000),.stop_clearance_mm(16'd0),.slow_clearance_mm(16'd0),.rotate_clearance_mm(16'd0),
  .body_guard_ready(guard_ready),.body_front_stop(front_stop),.body_rear_stop(1'b0),.body_front_slow(1'b0),
  .body_turn_stop(turn_stop),.body_nav_ahead(1'b0),.body_coverage(coverage),
  .lidar_correction_valid(1'b0),.lidar_correction_x_mm(16'sd0),.lidar_correction_y_mm(16'sd0),
  .run_request(run),.target_speed_left(ls),.target_speed_right(rs),.gyro_calibrated(calibrated),.status(status),.fault(fault),
  .diagnostic_required_coverage(required),.diagnostic_measurement_unknown(measurement_unknown)
 );
 task step;begin @(negedge clk);end endtask
 task issue; input[2:0] value;begin cmd=value;cv=1;step;cv=0;end endtask
 task frame;begin sample=1;step;sample=0;tick=1;step;tick=0;repeat(2200)step;end endtask
 task check;input condition;input[255:0] message;begin cases=cases+1;if(!condition)$fatal(1,"%s status%0d fault%0d",message,status,fault);end endtask
 initial begin
  repeat(4)step;rst=1;
  repeat(128)begin sample=1;step;sample=0;step;end
  check(calibrated,"gyro calibration from inputs");
  issue(1);frame;frame;
  check(!run && status==8 && required==1 && measurement_unknown,"no forward evidence UNKNOWN");
  coverage=1;frame;
  check(run && ls==10 && rs==10 && !measurement_unknown,"known free accepts motion");
  front_stop=1;frame;
  check(!run && status==7,"actual front obstacle blocked");
  front_stop=0;frame;
  check(run,"same active manual task recovers");
  lv=0;frame;check(!run && status==8 && !measurement_unknown,"scan loss stops");lv=1;
  gv=0;frame;check(!run && status==8,"gyro invalid stops");gv=1;
  issue(1);guard_ready=0;frame;check(!run && status==8 && !measurement_unknown,"processing unhealthy is sensor unavailable");guard_ready=1;
  coverage=15;issue(2);frame;frame;$display("TURN run%0d L%0d R%0d req%0d kind%0d turnw%0d brake%b error%0d state%0d",run,ls,rs,required,dut.decision_kind,dut.turn_work,dut.turn_braking,dut.heading_error_sample,dut.state);check(run && required==15,"observed turn accepts");
  coverage=7;frame;check(!run && status==8 && measurement_unknown,"turn missing rear UNKNOWN");
  coverage=15;turn_stop=1;frame;check(!run && status==7,"actual turn obstacle");
  turn_stop=0;issue(1);frame;
  repeat(21)begin tick=1;step;tick=0;repeat(2200)step;end
  check(!run && status==8,"gyro age expires without samples");
  frame;check(!run,"manual stale stop is not auto-rearmed");
  issue(1);frame;check(run,"fresh gyro accepts new safe command");
  issue(0);frame;check(!run && status==0,"explicit STOP remains stopped");
  $display("PASS nav observation gate %0d runtime assertions",cases);$finish;
 end
 initial begin #100000000;$fatal(1,"timeout");end
endmodule
