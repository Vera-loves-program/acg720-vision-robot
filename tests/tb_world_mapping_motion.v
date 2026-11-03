`timescale 1ns/1ps
// Actual navigation -> extracted actual root pose gate -> actual mapper.
// The old correction-qualified gate feeds a second real mapper as a negative
// control. Body-frame processing shares the same raw rays and remains live.
module tb_world_mapping_motion;
 GSR GSR(.GSRI(1'b1));
 reg clk=0;always #10 clk=~clk;
 reg rst=0,tick=0,sample=0,gyro_valid=1;
 reg signed[15:0] gyro=0;
 reg signed[31:0] encoder=0;
 wire pose_valid,pose_update,mapping_allowed,correction_allowed;
 wire signed[31:0] px,py;wire[15:0] heading;
 wire[7:0] fault;
 robot_navigation nav(.clk(clk),.reset_n(rst),.tick_10ms(tick),.safety_stop(1'b0),.motion_enable(1'b1),
 .encoder_position_left(encoder),.encoder_position_right(encoder),.encoder_zeroed(1'b0),
 .gyro_z(gyro),.gyro_sample(sample),.gyro_valid(gyro_valid),.gyro_invert(1'b0),
 .geometry_valid(1'b1),.wheel_mm_per_rev(16'd204),.encoder_counts_per_rev(16'd1469),.track_mm(16'd213),
 .pose_set(1'b0),.pose_x_mm(32'sd0),.pose_y_mm(32'sd0),.pose_heading_mdeg(32'sd0),
 .command_valid(1'b0),.command(3'd0),.manual_left(16'sd0),.manual_right(16'sd0),.manual_heading_hold(1'b0),
 .turn_heading_mdeg(32'sd0),.goal_x_mm(32'sd1000),.goal_y_mm(32'sd0),.turn_half_direction(2'd0),
 .cruise_counts(16'd30),.turn_counts(16'd12),.arrival_tolerance_mm(16'd5),
 .avoid_enable(1'b0),.lidar_valid(1'b1),.clear_front_mm(16'd1000),.clear_left_mm(16'd1000),
 .clear_right_mm(16'd1000),.clear_rear_mm(16'd1000),.stop_clearance_mm(16'd0),.slow_clearance_mm(16'd0),.rotate_clearance_mm(16'd0),
 .body_guard_ready(1'b1),.body_front_stop(1'b0),.body_rear_stop(1'b0),.body_front_slow(1'b0),.body_turn_stop(1'b0),.body_nav_ahead(1'b0),.body_coverage(4'hf),
 .lidar_correction_valid(1'b0),.lidar_correction_x_mm(16'sd0),.lidar_correction_y_mm(16'sd0),
 .x_mm(px),.y_mm(py),.heading_q6(heading),.pose_valid(pose_valid),.pose_update(pose_update),.fault(fault),
 .mapping_allowed(mapping_allowed),.correction_allowed(correction_allowed));
 reg[5:0] static_age=0;
 wire fresh_world,legacy_world,correction_permitted;
 wire signed[31:0] mx,my,lmx,lmy;wire[15:0] mh,lmh;
 actual_root_mapping_pose_gate capture(.clk(clk),.reset_n(rst),.tick_10ms(tick),.map_reset(1'b0),.map_local(1'b0),
 .pose_valid(pose_valid),.pose_update(pose_update),.mapping_allowed(mapping_allowed),.correction_allowed(correction_allowed),
 .static_gyro_age(static_age),.robot_x(px),.robot_y(py),.heading_q6(heading),
 .world_map_pose_valid(fresh_world),.map_pose_x(mx),.map_pose_y(my),.map_pose_heading(mh));
 legacy_root_mapping_pose_gate old_capture(.clk(clk),.reset_n(rst),.tick_10ms(tick),.map_reset(1'b0),.map_local(1'b0),
 .pose_valid(pose_valid),.pose_update(pose_update),.mapping_allowed(mapping_allowed),.correction_allowed(correction_allowed),
 .static_gyro_age(static_age),.robot_x(px),.robot_y(py),.heading_q6(heading),
 .world_map_pose_valid(legacy_world),.map_pose_x(lmx),.map_pose_y(lmy),.map_pose_heading(lmh));
 actual_root_mapping_correction_gate correction_gate(.map_local(1'b0),.world_map_pose_valid(fresh_world),
 .correction_allowed(correction_allowed),.manual_active(1'b0),.manual_turn_active(1'b0),.remote_turn_active(1'b0),.map_reset(1'b0),.permitted(correction_permitted));
 reg raw_valid=0,scan_start=0,freeze=0,read_enable=0;
 reg[14:0] angle=0;reg[15:0] distance=2000,sid=0;
 wire map_point;wire[14:0] projected_angle;wire[15:0] projected_distance;wire[5:0] projected_quality;
 integer body_points=0,confirmed_points=0,checks=0,budget,before_points;
 wire confirmed,turn_stop;
 lidar_body_guard body(.clk(clk),.reset_n(rst),.tick_10ms(tick),.active(1'b1),
 .point_valid(raw_valid),.scan_start(scan_start),.angle_q6(angle),.forward_angle_q6(15'd0),
 .distance_mm(distance),.quality(6'd20),.raw_node_valid(raw_valid),.raw_reject_reason(3'd0),.raw_parse_error(1'b0),.scan_id(sid),
 .offset_x_mm(16'sd33),.offset_y_mm(-16'sd77),.body_front_mm(16'd232),.body_rear_mm(16'd118),.body_left_mm(16'd119),.body_right_mm(16'd119),
 .drive_margin_mm(16'd20),.turn_margin_mm(16'd30),.contact_margin_mm(16'd10),
 .map_point_valid(map_point),.map_point_angle(projected_angle),.map_point_distance(projected_distance),.map_point_quality(projected_quality),
 .confirmed_point_valid(confirmed),.turn_stop(turn_stop));
 always @(posedge clk)begin if(map_point)body_points=body_points+1;if(confirmed)confirmed_points=confirmed_points+1;end
 wire[1:0] new_cell,old_cell;
 wire new_valid,old_valid,new_frozen,old_frozen,new_ready,old_ready,new_busy,old_busy;
 wire[31:0] new_scans,old_scans;
 wire[15:0] new_id,old_id;
 wire new_integrated,new_pulse,old_integrated,old_pulse;
 reg[6:0] rx=80,ry=62;
 reg[1:0] sampled_new_cell,sampled_old_cell;
 lidar_grid_map #(.FRAME_POINTS(16)) mapper(.clk(clk),.reset_n(rst),.enable(1'b1),.clear_map(1'b0),
 .scan_id(sid),.point_valid(map_point),.scan_start(scan_start),.angle_q6({1'b0,projected_angle}),.distance_mm(projected_distance),.quality(projected_quality),
 .pose_valid(fresh_world),.local_scan(1'b0),.pose_x_mm(mx),.pose_y_mm(my),.pose_heading_q6(mh),
 .forward_angle_q6(15'd0),.lidar_offset_x_mm(16'sd33),.lidar_offset_y_mm(-16'sd77),
 .body_front_mm(16'd232),.body_rear_mm(16'd118),.body_left_mm(16'd119),.body_right_mm(16'd119),
 .correction_enable(correction_permitted),.correction_ack(1'b0),.freeze_map(freeze),.map_frozen(new_frozen),
 .plan_rd_en(read_enable),.plan_rd_x(rx),.plan_rd_y(ry),.plan_rd_cell(new_cell),.plan_rd_valid(new_valid),
 .map_ready(new_ready),.busy(new_busy),.scan_count(new_scans),.integrated_scan_id(new_id),.integrated_scan_valid(new_integrated),.integrated_scan_pulse(new_pulse),
 .rd_clk(clk),.rd_en(1'b0),.rd_x(7'd0),.rd_y(7'd0));
 lidar_grid_map #(.FRAME_POINTS(16)) legacy_mapper(.clk(clk),.reset_n(rst),.enable(1'b1),.clear_map(1'b0),
 .scan_id(sid),.point_valid(map_point),.scan_start(scan_start),.angle_q6({1'b0,projected_angle}),.distance_mm(projected_distance),.quality(projected_quality),
 .pose_valid(legacy_world),.local_scan(1'b0),.pose_x_mm(lmx),.pose_y_mm(lmy),.pose_heading_q6(lmh),
 .forward_angle_q6(15'd0),.lidar_offset_x_mm(16'sd33),.lidar_offset_y_mm(-16'sd77),
 .body_front_mm(16'd232),.body_rear_mm(16'd118),.body_left_mm(16'd119),.body_right_mm(16'd119),
 .correction_enable(1'b0),.correction_ack(1'b0),.freeze_map(freeze),.map_frozen(old_frozen),
 .plan_rd_en(read_enable),.plan_rd_x(rx),.plan_rd_y(ry),.plan_rd_cell(old_cell),.plan_rd_valid(old_valid),
 .map_ready(old_ready),.busy(old_busy),.scan_count(old_scans),.integrated_scan_id(old_id),.integrated_scan_valid(old_integrated),.integrated_scan_pulse(old_pulse),
 .rd_clk(clk),.rd_en(1'b0),.rd_x(7'd0),.rd_y(7'd0));
 task step;begin @(negedge clk);end endtask
 task check;input condition;input[511:0] msg;begin checks=checks+1;if(condition!==1'b1)$fatal(1,"%0s distq8=%0d mapping%b corr%b world%b legacy%b",msg,nav.distance_q8,mapping_allowed,correction_allowed,fresh_world,legacy_world);end endtask
 task frame;input integer counts,raw_gyro;begin encoder=encoder+counts;gyro=raw_gyro;sample=1;step;sample=0;tick=1;step;tick=0;repeat(2200)step;end endtask
 task boundary;begin sid=sid+1;scan_start=1;step;scan_start=0;repeat(3)step;end endtask
 task beam;input integer a,r;begin angle=a;distance=r;raw_valid=1;step;raw_valid=0;repeat(600)step;end endtask
 task finish_scan;input integer count;begin budget=0;while((new_scans<count || new_busy) && budget<150000)begin step;budget=budget+1;end check(budget<150000,"new scan integration completed");end endtask
 task raw_read;begin freeze=1;budget=0;while((!new_frozen || !old_frozen) && budget<100000)begin step;budget=budget+1;end check(budget<100000,"both live maps grant planner freeze");read_enable=1;step;check(new_valid && old_valid,"synchronous planner read valid");sampled_new_cell=new_cell;sampled_old_cell=old_cell;read_enable=0;step;freeze=0;repeat(5)step;end endtask
 initial begin
  repeat(4)step;rst=1;repeat(128)begin sample=1;step;sample=0;step;end
  repeat(17000)step;check(new_ready && old_ready,"both actual map RAMs initialized");
  frame(30,0);frame(30,0);
  check(pose_valid && mapping_allowed && fresh_world,"30counts cruise maps through coherent actual pose");
  check(!correction_allowed && !legacy_world && !correction_permitted,"30counts cannot correct or pass old capture qualifier");
  boundary;beam(0,2000);beam(5760,2000);beam(11520,2000);beam(17280,2000);boundary;finish_scan(1);
  check(new_id==1 && new_integrated && old_scans==0 && !old_integrated,"new ID1 integrates while legacy mapper remains unchanged");
  raw_read;check(sampled_new_cell==1 && sampled_old_cell==0,"planner reads current live FREE, not old LCD bank or legacy UNKNOWN");
  before_points=body_points;frame(60,0);check(!mapping_allowed && !fresh_world,"speed above8mm per10ms rejects world writes");
  boundary;beam(0,2000);boundary;repeat(500)step;
  check(new_scans==1 && body_points>before_points,"fast world hold leaves body processing live");
  frame(57,0);check(mapping_allowed && fresh_world && !correction_allowed,"57counts below8mm accepted only for mapping");
  frame(58,0);check(!mapping_allowed && !fresh_world,"58counts above8mm rejected");
  frame(-57,0);check(mapping_allowed && fresh_world,"negative57counts uses symmetric bound");
  frame(-58,0);check(!mapping_allowed && !fresh_world,"negative58counts rejected");
  frame(0,300);check(!mapping_allowed && !correction_allowed && !fresh_world,"original0.2degree angular limit unchanged");
  frame(30,0);static_age=3;step;check(!fresh_world,"original30ms root gyro freshness unchanged");
  before_points=body_points;boundary;beam(0,2000);boundary;repeat(500)step;
  check(new_scans==1 && body_points>before_points,"stale world pose does not stop body processing");
  static_age=0;frame(30,0);check(fresh_world,"fresh gyro allows new coherent moving capture");
  boundary;beam(0,2000);beam(5760,2000);boundary;finish_scan(2);
  check(new_id==sid-1 && new_id>1 && old_scans==0,"latest post-hold scan ID committed, no stale scan replay");
  gyro_valid=0;step;check(!mapping_allowed && !correction_allowed && !fresh_world,"invalid gyro fails both qualifiers closed");
  gyro_valid=1;frame(0,0);check(mapping_allowed && correction_allowed && correction_permitted,"stationary original correction qualifier remains usable");
  frame(10000,0);check(fault!=0 && !mapping_allowed && !fresh_world,"encoder fault cannot authorize moving map");
  $display("PASS moving world capture %0d actualHDL checks: cruise30 negativecontrol, liveRAM/newIDs,8mm bounds, original gyro/fault/correction guards, live body processing",checks);$finish;
 end
 initial begin #200000000;$fatal(1,"moving map watchdog");end
endmodule
