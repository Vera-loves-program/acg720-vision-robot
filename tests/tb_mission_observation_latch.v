`timescale 1ns/1ps
// Real mission FSM. Map/planner handshakes are explicit fixtures; this unit
// regression isolates a lost completion event, not chassis/map simulation.
module tb_mission_observation_latch;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,permit=1,user_valid=0,pose_valid=1,capture_ready=1,boundary=0,pulse=0;
 reg [2:0] user_command=0;
 reg [3:0] nav_status=0;
 reg [15:0] scan_id=2,integrated_id=1;
 reg valid=1,planner_busy=0,planner_done=0,reachable=0;
 wire start,active,waiting,refreshing,no_path,motion,nav_valid;
 wire [2:0] command;wire [7:0] error;
 wire signed [31:0] goal_x,goal_y;
 wire [15:0] tolerance;
 integer starts=0,checks=0;
 robot_mission_control dut(.clk(clk),.reset_n(rst),.tick_10ms(1'b1),.permit(permit),.safety_stop(1'b0),
  .user_command_valid(user_valid),.user_command(user_command),.user_goal_x(32'sd1000),.user_goal_y(32'sd0),
  .pose_x(32'sd0),.pose_y(32'sd0),.pose_valid(pose_valid),.nav_status(nav_status),
  .virtual_contact(1'b0),.rear_clear(1'b1),.recovery_counts(16'd30),
  .map_ready(1'b1),.map_busy(1'b0),.map_capture_ready(capture_ready),.scan_complete(1'b0),
  .capture_boundary(boundary),.scan_id(scan_id),.map_integrated_id(integrated_id),
  .map_integrated_valid(valid),.map_integrated_pulse(pulse),.map_scan_count(32'd100),
  .planner_busy(planner_busy),.planner_done(planner_done),.planner_reachable(reachable),
  .planner_next_x(7'd79),.planner_next_y(7'd64),.planner_start(start),
  .nav_command_valid(nav_valid),.nav_command(command),.nav_goal_x(goal_x),.nav_goal_y(goal_y),
  .nav_arrival_tolerance_mm(tolerance),.motion_permit(motion),.waiting_map(refreshing),
  .waiting_path(waiting),.mission_error(error),.mission_active(active),.no_path(no_path));
 always @(posedge clk)if(!rst)starts<=0;else if(start)starts<=starts+1;
 task step;begin @(negedge clk);end endtask
 task check;input condition;input [511:0] text;
  begin if(condition!==1'b1)$fatal(1,"%0s: starts%0d error%0d phase%0d waiting%b",text,starts,error,dut.refresh_phase,waiting);checks=checks+1;end
 endtask
 task expect_start;input integer expected;integer clocks;
  begin clocks=0;while(starts<expected && clocks<20)begin step;clocks=clocks+1;end
   check(starts==expected,"fresh completed observation was lost");planner_busy=1;end
 endtask
 task reply;input integer success;
  begin planner_busy=0;reachable=success;planner_done=1;step;planner_done=0;repeat(6)step;end
 endtask
 task observe;input [15:0] id;
  begin integrated_id=id;scan_id=id+1'b1;pulse=1;step;pulse=0;end
 endtask
 task go;
  begin user_command=3;user_valid=1;step;user_valid=0;step;end
 endtask
 initial begin
  repeat(4)step;rst=1;observe(1);go;expect_start(1);reply(1);
  check(goal_x==1000 && goal_y==0 && tolerance==5,"initial exact goal changed");
  nav_status=7;step;nav_status=0;repeat(3)step;
  check(refreshing && active && !motion,"obstacle did not retain parked mission");
  scan_id=3;boundary=1;step;boundary=0;step;
  // The map completed a valid post-stop scan. One consumer readiness sample
  // coincides with its finish; the result remains valid/fresh afterwards.
  capture_ready=0;observe(3);capture_ready=1;
  expect_start(2);reply(0);
  check(waiting && active && !motion,"failed path did not remain parked");
  capture_ready=0;observe(4);capture_ready=1;
  expect_start(3);reply(0);
  // Duplicate completions cannot be used to retry an unchanged failed map.
  observe(4);repeat(8)step;
  check(starts==3 && waiting && active,"duplicate map revision retried route");
  // A transient gyro/pose gate on commit must not lose a newer observation.
  pose_valid=0;observe(5);pose_valid=1;
  expect_start(4);reply(1);
  check(goal_x==1000 && goal_y==0 && tolerance==5,"resume lost original1000/0 goal");
  user_command=0;user_valid=1;step;user_valid=0;observe(6);repeat(8)step;
  check(!active && starts==4,"STOP resurrected by held observation");
  // Held16bit revisions must still be fresh and genuinely newer across wrap.
  rst=0;repeat(4)step;rst=1;nav_status=0;pose_valid=1;capture_ready=1;observe(16'hffff);
  go;expect_start(1);reply(0);
  observe(16'hffff);repeat(6)step;
  check(starts==1&&waiting,"duplicate wrapped revision retried");
  capture_ready=0;observe(0);repeat(31)step;capture_ready=1;repeat(4)step;
  check(starts==1&&waiting,"expired held map was used");
  observe(1);expect_start(2);reply(1);
  check(goal_x==1000&&goal_y==0,"wrapped fresh map lost original goal");
  nav_status=9;step;nav_status=0;observe(2);repeat(6)step;
  check(starts==2&&waiting&&active&&!motion&&error==19,"physical NAV fault automatically resumed");
  $display("PASS mission observation latch %0d assertions: commit readiness gap, held completion, duplicate ID, pose recovery, exact goal, STOP",checks);$finish;
 end
 initial begin #1000000;$fatal(1,"observation regression timeout");end
endmodule
