`timescale 1ns/1ps
// Actual subsystem predicate/priority statements are extracted by the runner;
// the formatter and mapper/planner below are real RTL. This is not full SoC.
module tb_robot_rejection_status;
 GSR GSR(.GSRI(1'b1));
 reg clk=0;always #5 clk=~clk;
 reg rst=0,no_path=0,projection_bad=0,front_stop=0;
 reg [7:0] mission_error=0,planner_error=0;
 reg [3:0] required=0,visibility=15;
 wire [7:0] rejection;
 wire [7:0] effective_error=mission_error!=0 ? mission_error : planner_error;
 wire text_valid,byte_valid;
 wire [7:0] byte_data,byte_index;
 reg [127:0] status_row=0;
 integer cases=0,waits=0,e,p,n;
 reg display_done=0,planner_done_all=0;
 actual_rejection_pipeline causes(
  .clk(clk),.reset_n(rst),.scan_enable(1'b1),.lidar_running(1'b1),.complete_scan_seen(1'b1),.complete_scan_age(6'd0),
  .gyro_valid(1'b1),.gyro_calibrated(1'b1),.calibrating(1'b0),.gyro_fresh_diag(1'b1),
  .projection_bad(projection_bad),.last_scan_bad(1'b0),.lidar_ready(1'b1),.body_guard_ready(1'b1),
  .virtual_contact(1'b0),.nav_obstacle_ahead(1'b0),.body_front_stop(front_stop),.body_rear_stop(1'b0),.body_turn_stop(1'b0),
  .nav_required_coverage(required),.body_free_coverage(visibility),.map_local(1'b0),.mission_waiting_map(1'b0),
  .no_path(no_path),.mission_error(mission_error),.planner_error(planner_error),.reject_diagnostic(rejection));
 robot_status_text #(.CLK_HZ(1000),.PARALLEL_COMPAT(0)) text_dut(
  .clk(clk),.reset_n(rst),.page(3'd4),.subpage(3'd1),.planner_error(effective_error),
  .reject_diagnostic(rejection),.waiting_map(1'b0),.waiting_path(no_path),
  .armed(1'b1),.geometry_valid(1'b1),.gyro_calibrated(1'b1),.calibrating(1'b0),
  .lidar_ready(1'b1),.scan_enable(1'b1),.safety_stop(1'b0),.map_ready(1'b1),.gains_ready(1'b1),
  .map_local(1'b0),.map_world_ready(1'b1),.nav_status(no_path ? 4'd11 : 4'd0),
  .ui_notice(4'd0),.nav_fault(8'd0),.lidar_error(8'd0),.motor_fault(8'd0),
  .x_mm(32'sd246),.y_mm(32'sd4),.heading_mdeg(32'sd6000),.goal_x_mm(32'sd300),.goal_y_mm(32'sd0),
  .text_valid(text_valid),.text_byte_valid(byte_valid),.text_byte_data(byte_data),.text_byte_index(byte_index),.text_byte_ready(1'b1));
 always @(posedge clk)if(byte_valid && byte_index>=16 && byte_index<32)
  status_row[127-(byte_index-16)*8-:8]<=byte_data;
 task step;begin @(negedge clk);end endtask
 task expect_status;
  input [6:0] id;
  input [127:0] expected;
  integer frames;
  begin
   repeat(3)step;
   if(text_dut.live_status!==id)$fatal(1,"status override mission%0d planner%0d no_path%0d rejection%0d expectedID%0d got%0d",
       mission_error,planner_error,no_path,rejection,id,text_dut.live_status);
   for(frames=0;frames<2;frames=frames+1)begin
    while(text_valid)step;waits=0;
    while(!text_valid && waits<5000)begin step;waits=waits+1;end
    if(waits==5000)$fatal(1,"formatter frame timeout");
   end
   if(status_row!==expected)$fatal(1,"actual16-byte status row mismatch expected[%s] got[%s]",expected,status_row);
   cases=cases+1;
  end
 endtask
 task mission_status;
  input integer code;
  begin case(code)
   13:expect_status(92,"MAP WAIT TIMEOUT");
   14:expect_status(93,"POSE NOT READY  ");
   15:expect_status(94,"MAP NOT READY   ");
   16:expect_status(95,"CONTACT STOP    ");
   17:expect_status(96,"FINAL LEG BLOCK ");
   18:expect_status(97,"CHECK SENSORS   ");
   19:expect_status(98,"NAV MATH FAULT  ");
   20:expect_status(99,"OUTSIDE MAP     ");
   21:expect_status(108,"MOTION PAUSED   ");
   22:expect_status(115,"MEASURE UNKNOWN ");
  endcase end
 endtask
 initial begin
  repeat(3)step;rst=1;
  no_path=1;
  // A stale preceding plannererror4 must not replace the more specific
  // current mission rejection. Read BOTH live ID and full transmitted row.
  for(p=0;p<2;p=p+1)begin
   planner_error=p==0 ? 0 : 4;
   for(e=13;e<=22;e=e+1)begin mission_error=e;mission_status(e);end
  end
  mission_error=0;planner_error=4;expect_status(87,"NO KNOWN PATH   ");
  planner_error=2;expect_status(85,"START BLOCKED   ");
  planner_error=8;expect_status(22,"START UNKNOWN   ");
  planner_error=3;expect_status(86,"GOAL BLOCKED    ");
  planner_error=9;expect_status(23,"GOAL UNKNOWN    ");
  mission_error=17;planner_error=4;required=1;front_stop=1;
  expect_status(116,"NEAR OBSTACLE   "); // preserve physical-cause priority
  front_stop=0;visibility=0;expect_status(115,"MEASURE UNKNOWN ");
  projection_bad=1;expect_status(114,"LIDAR OVERRUN   ");
  projection_bad=0;visibility=15;required=0;mission_error=0;planner_error=0;
  expect_status(109,"WAIT CLEAR PATH ");
  $display("PASS actual rejection pipeline+formatter %0d full status rows; mission13..21/no_path/staleplanner4 and physical priority",cases);
  display_done=1;
 end

 // Real mapper control RAM and real planner, not the old one-port occupancy
 // mock. Inject known map fixtures after its serialclear; assert every valid
 // read returns the location requested on the SAME preceding clock edge.
 reg plan_start=0;
 wire freeze,frozen,map_ready,map_busy,plan_busy,plan_done,reachable,rd_en,rd_valid;
 wire [6:0] rd_x,rd_y,next_x,next_y;
 wire [1:0] rd_cell,reject_cell;
 wire [7:0] plan_error;
 wire [12:0] path_length;
 wire reject_valid;
 wire signed [9:0] reject_x,reject_y;
 wire [3:0] reject_reason;
 integer read_checks=0,k,plan_waits=0,plans=0;
 reg [1:0] read_expected;
 reg check_read;
 lidar_grid_map mapper(
  .clk(clk),.reset_n(rst),.enable(1'b1),.clear_map(1'b0),.scan_id(16'd0),
  .point_valid(1'b0),.scan_start(1'b0),.angle_q6(16'd0),.distance_mm(16'd0),.quality(6'd0),
  .pose_valid(1'b0),.local_scan(1'b0),.pose_x_mm(32'sd246),.pose_y_mm(32'sd4),.pose_heading_q6(16'd384),
  .forward_angle_q6(15'd0),.lidar_offset_x_mm(16'sd33),.lidar_offset_y_mm(-16'sd77),
  .body_front_mm(16'd0),.body_rear_mm(16'd0),.body_left_mm(16'd0),.body_right_mm(16'd0),
  .correction_enable(1'b0),.correction_ack(1'b0),.freeze_map(freeze),.map_frozen(frozen),
  .plan_rd_en(rd_en),.plan_rd_x(rd_x),.plan_rd_y(rd_y),.plan_rd_cell(rd_cell),.plan_rd_valid(rd_valid),
  .map_ready(map_ready),.busy(map_busy),.rd_clk(clk),.rd_en(1'b0),.rd_x(7'd0),.rd_y(7'd0));
 lidar_path_planner planner(
  .clk(clk),.reset_n(rst),.start(plan_start),.cancel(1'b0),
  .current_x(7'd67),.current_y(7'd64),.goal_x(7'd68),.goal_y(7'd64),
  .current_x_mm(32'sd246),.current_y_mm(32'sd4),.heading_q6(16'd384),.clearance_radius_mm(16'd291),
  .map_ready(map_ready),.map_frozen(frozen),.freeze_map(freeze),.busy(plan_busy),.done(plan_done),.reachable(reachable),
  .error_code(plan_error),.next_x(next_x),.next_y(next_y),.next_is_connector(),
  .reject_valid(reject_valid),.reject_x(reject_x),.reject_y(reject_y),.reject_cell(reject_cell),.reject_reason(reject_reason),
  .start_check_phase(),.path_length(path_length),.expanded_nodes(),
  .plan_rd_en(rd_en),.plan_rd_x(rd_x),.plan_rd_y(rd_y),.plan_rd_cell(rd_cell),.plan_rd_valid(rd_valid));
 always @(posedge clk)begin
  check_read=rst && frozen && rd_en;
  if(check_read)read_expected=mapper.cells[{rd_y,rd_x}];
  #1;if(check_read)begin
   if(!rd_valid || rd_cell!==read_expected)$fatal(1,"actual mapper/planner RAM response mismatch");
   read_checks=read_checks+1;
  end
 end
 task plan;
  input [7:0] expected_error;
  begin
   step;plan_start=1;step;plan_start=0;plan_waits=0;
   while(!plan_done && plan_waits<3000000)begin step;plan_waits=plan_waits+1;end
   if(!plan_done || plan_error!=expected_error || reachable!=(expected_error==0))
    $fatal(1,"actual map/planner expected%0d got%0d reachable%0d",expected_error,plan_error,reachable);
   if(freeze || frozen || plan_busy)$fatal(1,"plan result failed to release live map");
   plans=plans+1;step;
  end
 endtask
 initial begin
  while(!rst)step;while(!map_ready)step;step;
  for(k=0;k<16384;k=k+1)mapper.cells[k]=1;
  plan(0);
  if(planner.start_index!=planner.goal_index || path_length!=0 || next_x!=68 || next_y!=64)
   $fatal(1,"246,4 and300,0 same-coarse-cell constraint broken");
  // Isolated real occupied and UNKNOWN cells at currentO must keep distinct
  // errors and exact rejected addresses through the actual synchronous port.
  mapper.cells[64*128+67]=3;plan(2);
  if(!reject_valid || reject_x!=67 || reject_y!=64 || reject_cell!=3 || reject_reason!=2)
   $fatal(1,"actual start OCCUPIED readback lost");
  mapper.cells[64*128+67]=0;plan(8);
  if(!reject_valid || reject_x!=67 || reject_y!=64 || reject_cell!=0 || reject_reason!=1)
   $fatal(1,"actual start UNKNOWN readback lost");
  $display("PASS actual mapper+planner %0d plans %0d synchronousRAM reads; samecell246/300 zero-edge route, occupied/unknown distinguish, freeze release",plans,read_checks);
  planner_done_all=1;
 end
 initial begin wait(display_done && planner_done_all);$display("PASS rejection display and actual mapper/planner integration");$finish;end
 initial begin #100000000;$fatal(1,"rejection/planner integration timeout");end
endmodule
