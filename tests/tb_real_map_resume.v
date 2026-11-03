`timescale 1ns/1ps
// Actual mapper/live RAM + planner + mission. A pre-observed FREE room is an
// explicit fixture; subsequent obstacle hits/clearings go through real rays.
// NAV status and readiness are controlled environment inputs, not a chassis.
module tb_real_map_resume;
 GSR GSR(.GSRI(1'b1));
 reg clk=0;always #10 clk=~clk;
 reg rst=0,tick=0,point_valid=0,boundary=0,user_valid=0,contact=0;
 reg [2:0] user_command=0;reg [3:0] nav_status=0;
 reg [15:0] angle=0,distance=0,scan_id=0;
 wire map_ready,map_busy,frozen,freeze,integrated_pulse,integrated_valid;
 wire [15:0] integrated_id;wire [31:0] scans;
 wire plan_start,plan_cancel,plan_busy,plan_done,reachable,rd_en,rd_valid;
 wire [6:0] current_x,current_y,goal_x,goal_y,next_x,next_y,rd_x,rd_y;
 wire [1:0] cell_value;wire [7:0] planner_error,mission_error;
 wire nav_valid,active,waiting,refreshing,no_path,motion;
 wire [2:0] command;wire signed [31:0] nav_x,nav_y;
 integer starts=0,commands=0,checks=0,i,budget,previous_commands;
 integer route_min_y=0,route_max_y=0,back_y;
 // A one-clock consumer readiness hole on EACH actual map commit is enough
 // to reproduce the old permanent stall despite a healthy continuing map.
 wire capture_ready=!integrated_pulse;
 robot_mission_control mission(.clk(clk),.reset_n(rst),.tick_10ms(tick),.permit(1'b1),.safety_stop(1'b0),
  .user_command_valid(user_valid),.user_command(user_command),.user_goal_x(32'sd1000),.user_goal_y(32'sd0),
  .pose_x(32'sd0),.pose_y(32'sd0),.pose_valid(1'b1),.nav_status(nav_status),
  .virtual_contact(contact),.rear_clear(1'b1),.recovery_counts(16'd30),
  .map_ready(map_ready),.map_busy(map_busy),.map_capture_ready(capture_ready),.scan_complete(1'b0),
  .capture_boundary(boundary),.scan_id(scan_id),.map_integrated_id(integrated_id),
  .map_integrated_valid(integrated_valid),.map_integrated_pulse(integrated_pulse),.map_scan_count(scans),
  .planner_busy(plan_busy),.planner_done(plan_done),.planner_reachable(reachable),
  .planner_next_x(next_x),.planner_next_y(next_y),.planner_start(plan_start),.planner_cancel(plan_cancel),
  .planner_current_x(current_x),.planner_current_y(current_y),.planner_goal_x(goal_x),.planner_goal_y(goal_y),
  .nav_command_valid(nav_valid),.nav_command(command),.nav_goal_x(nav_x),.nav_goal_y(nav_y),
  .motion_permit(motion),.waiting_map(refreshing),.waiting_path(waiting),.mission_error(mission_error),
  .mission_active(active),.no_path(no_path));
 lidar_path_planner planner(.clk(clk),.reset_n(rst),.start(plan_start&&!plan_cancel),.cancel(plan_cancel),
  .current_x(current_x),.current_y(current_y),.goal_x(goal_x),.goal_y(goal_y),.heading_q6(16'd0),
  .clearance_radius_mm(16'd291),.map_ready(map_ready),.map_frozen(frozen),.freeze_map(freeze),
  .busy(plan_busy),.done(plan_done),.reachable(reachable),.error_code(planner_error),.next_x(next_x),.next_y(next_y),
  .plan_rd_en(rd_en),.plan_rd_x(rd_x),.plan_rd_y(rd_y),.plan_rd_cell(cell_value),.plan_rd_valid(rd_valid));
 lidar_grid_map map(.clk(clk),.reset_n(rst),.enable(1'b1),.clear_map(1'b0),.scan_id(scan_id),
  .point_valid(point_valid),.scan_start(boundary),.angle_q6(angle),.distance_mm(distance),.quality(6'd12),
  .pose_valid(1'b1),.local_scan(1'b0),.pose_x_mm(32'sd0),.pose_y_mm(32'sd0),.pose_heading_q6(16'd0),
  .forward_angle_q6(15'd0),.lidar_offset_x_mm(16'sd33),.lidar_offset_y_mm(-16'sd77),
  .body_front_mm(16'd232),.body_rear_mm(16'd118),.body_left_mm(16'd119),.body_right_mm(16'd119),
  .correction_enable(1'b0),.correction_ack(1'b0),.freeze_map(freeze),.map_frozen(frozen),
  .plan_rd_en(rd_en),.plan_rd_x(rd_x),.plan_rd_y(rd_y),.plan_rd_cell(cell_value),.plan_rd_valid(rd_valid),
  .map_ready(map_ready),.busy(map_busy),.scan_count(scans),.integrated_scan_id(integrated_id),
  .integrated_scan_valid(integrated_valid),.integrated_scan_pulse(integrated_pulse),
  .rd_clk(clk),.rd_en(1'b0),.rd_x(7'd0),.rd_y(7'd0));
 always @(posedge clk)if(!rst)begin starts<=0;commands<=0;end else begin
  if(plan_start&&!plan_cancel)begin starts<=starts+1;route_min_y=0;route_max_y=0;end
  if(planner.state==planner.BACK_USE)begin
   back_y=planner.back_index[11:6]*128-4096;
   if(back_y<route_min_y)route_min_y=back_y;
   if(back_y>route_max_y)route_max_y=back_y;
  end
  if(nav_valid&&command==3)commands<=commands+1;
 end
 task step;begin @(negedge clk);end endtask
 task check;input condition;input [511:0] text;
  begin if(condition!==1'b1)$fatal(1,"%0s starts%0d cmd%0d merror%0d perror%0d scan%0d integrated%0d",text,starts,commands,mission_error,planner_error,scan_id,integrated_id);checks=checks+1;end
 endtask
 task start_scan;begin scan_id=scan_id+1'b1;boundary=1;step;boundary=0;repeat(3)step;end endtask
 task ray;input [15:0] a,d;
  begin angle=a;distance=d;point_valid=1;step;point_valid=0;repeat(250)step;end
 endtask
 task emit_scan;input integer front_distance;reg [15:0] expected;
  begin
   // Planner freeze legitimately discarded capture. Resume with a real new
   // revolution boundary before sending its rays, as the C1 stream does.
   if(map.wait_scan_start)start_scan;
   expected=scan_id;ray(0,front_distance);ray(11520,2000);start_scan;
   budget=0;while(!(integrated_pulse&&integrated_id==expected)&&budget<200000)begin step;budget=budget+1;end
   check(budget<200000,"real mapper did not commit complete scan");
   step;
  end
 endtask
 task request;input [2:0] action;
  begin user_command=action;user_valid=1;step;user_valid=0;repeat(4)step;end
 endtask
 task settle;integer n;begin for(n=0;n<3;n=n+1)begin tick=1;step;tick=0;repeat(4)step;end end endtask
 task await_command;input integer previous;
  begin budget=0;while(commands==previous&&budget<4000000)begin step;budget=budget+1;end
   check(commands==previous+1&&!no_path,"complete-map observation did not resume planned motion");end
 endtask
 task initialise;
  begin
   rst=0;point_valid=0;boundary=0;user_valid=0;contact=0;nav_status=0;scan_id=0;repeat(4)step;rst=1;
   repeat(40000)step;
   check(map_ready&&!map_busy,"real map initial clear/seed did not finish");
   // Known room fixture, not a replacement of observed obstacle updates.
   for(i=0;i<16384;i=i+1)map.cells[i]=1;
   start_scan;emit_scan(2000);repeat(40000)step;
  end
 endtask
 initial begin
  initialise;request(3);await_command(0);settle;
  check(nav_x==1000&&nav_y==0,"clear room did not issue original goal");
  nav_status=7;step;nav_status=0;repeat(4)step;
  check(active&&refreshing&&!motion,"dynamic obstacle did not park same mission");
  emit_scan(600);await_command(1);
  // The first legal waypoint may advance straight before the actual turn.
  // Inspect real parent-RAM backtrace instead of mistaking that for no detour.
  check(route_max_y>0&&route_min_y==0&&(nav_x!=1000||nav_y!=0)&&mission.final_x==1000&&mission.final_y==0,
        "actual map obstacle did not produce body-left detour to retained goal");
  request(0);check(!active,"STOP did not cancel detour");
  // Near-contact episode: a real200mm ray puts a hit at233,-77. After the
  // obstacle is moved, two far rays clear its actual liveRAM cell11->10->01.
  initialise;request(3);await_command(0);settle;contact=1;step;repeat(4)step;
  emit_scan(200);emit_scan(200);
  check(active&&waiting&&!motion&&map.cells[62*128+67]==3,"near-contact hit not retained in real map/mission");
  emit_scan(2000);emit_scan(2000);
  check(map.cells[62*128+67]==1,"moved obstacle did not clear through actual FREE rays");
  previous_commands=commands;contact=0;step;await_command(previous_commands);
  check(nav_x==1000&&nav_y==0&&active,"clear contact did not resume original exact goal");
  request(0);repeat(10)step;check(!active,"held observation resurrected cancelled mission");
  $display("PASS real mapper+planner+mission %0d checks: post-stop scan barrier, commit/readiness phase gap, actual left detour, liveRAM near-hit and FREE clearing, same-goal resume, STOP",checks);$finish;
 end
 initial begin #500000000;$fatal(1,"real-map resume watchdog");end
endmodule
