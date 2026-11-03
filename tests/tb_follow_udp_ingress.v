`timescale 1ns/1ps
// Full Ethernet framing at GMII, independent125/50MHz clocks, real mailbox/PWM.
// Excludes the physical RGMII/PHY and post-route timing.
module tb_follow_udp_ingress;
 reg rx_clk=0,clk=0;always #4 rx_clk=~rx_clk;always #10 clk=~clk;
 reg reset_n=0,dv=0;reg [7:0] rxd=0;
 wire valid,done,error,request,ack;wire [7:0] byte_out;
 wire [15:0] length;wire [255:0] data,report;wire [31:0] crc;
 wire pan_pwm,tilt_pwm;
 integer payloads=0,packets=0,errors=0,commits=0,checks=0;
 reg last_request=0;
 reg [7:0] frames[0:2559],frame_lengths[0:19];
 eth_udp_rx_gmii receiver(.reset_p(!reset_n),.local_mac(48'h02ac72000002),
  .local_ip(32'hc0a80a02),.local_port(16'd5001),.gmii_rx_clk(rx_clk),
  .gmii_rxdv(dv),.gmii_rxd(rxd),.data_overflow_i(1'b0),
  .payload_valid_o(valid),.payload_dat_o(byte_out),.rx_data_length(length),
  .one_pkt_done(done),.pkt_error(error),.debug_crc_check(crc));
 udp_detection_feedback #(.ENABLE_DISPLAY(0)) metadata(.clk(rx_clk),.reset_n(reset_n),
  .payload_valid(valid),.payload_byte(byte_out),.payload_length(length),
  .packet_done(done),.packet_error(error),.display_ack(1'b0),
  .robot_ack(ack),.robot_request(request),.robot_data(data));
 robot_follow_control #(.ENABLE_CHOOSER(0)) follow(.clk(clk),.reset_n(reset_n),
  .tick_10ms(1'b0),.request(request),.data(data),.ack(ack),
  .page(3'd5),.subpage(3'd0),.tap_valid(1'b0),.tap_button(8'd0),
  .stop_all(1'b0),.cancel(1'b0),.motion_ready(1'b1),.battery_a_low(1'b0),
  .gain(8'd20),.speed_cap(16'd5),.heading(32'sd0),
  .actual_left(16'sd0),.actual_right(16'sd0),.nav_status(4'd0),
  .turn_done(1'b0),.pose_x(32'sd0),.pose_y(32'sd0),
  .telemetry(report),.pan_pwm(pan_pwm),.tilt_pwm(tilt_pwm),
  .laser_valid(1'b0),.turn_clear(1'b1),.laser_target(16'd0),.laser_bearing(32'sd0));
 always @(posedge rx_clk)if(reset_n)begin
  if(valid)payloads=payloads+1;
  if(done)begin
   packets=packets+1;if(error)errors=errors+1;
   $display("packet=%0d bytes=%0d error=%b crc=%h metadata_count=%0d",packets,payloads,error,crc,metadata.count);
  end
  if(last_request!=request)begin commits=commits+1;last_request=request;end
 end
 task wire_frame;
  input integer index;
  integer j;
  begin
   @(negedge rx_clk);dv=1;
   for(j=0;j<frame_lengths[index];j=j+1)begin rxd=frames[index*128+j];@(negedge rx_clk);end
   dv=0;rxd=0;
  end
 endtask
 task send_frame;
  input integer index;
  begin wire_frame(index);repeat(120)@(negedge rx_clk);end
 endtask
 task check;
  input condition;input [511:0] message;
  begin if(condition!==1'b1)$fatal(1,"%0s",message);checks=checks+1;end
 endtask
 initial begin
  $readmemh("tmp/follow-udp-ingress-20261103/frames.hex",frames);
  $readmemh("tmp/follow-udp-ingress-20261103/lengths.hex",frame_lengths);
  repeat(8)@(negedge rx_clk);reset_n=1;repeat(8)@(negedge rx_clk);
  send_frame(0);
  check(packets==1 && errors==0 && payloads==32,"valid broadcast frame failed vendor UDP RX");
  check(commits==1 && follow.rx_seq==15130 && follow.link_live,"valid FOL not accepted through CDC mailbox");
  check(follow.applied_pan==1200 && follow.applied_tilt==1076,"135deg pose command not applied");
  check(report[79:64]==15130,"wire RBT ACK does not match FOL sequence");
  send_frame(1);
  check(commits==2 && follow.catalog_seq==15131,"valid TGT catalog not acknowledged");
  send_frame(2);
  check(commits==2 && follow.rx_seq==15130,"bad metadata CCITT CRC accepted");
  send_frame(3);
  check(commits==3 && follow.rx_seq==15131,"valid packet after metadata CRC error failed");
  send_frame(4);
  check(errors==1 && commits==3 && follow.rx_seq==15131,"bad Ethernet FCS accepted");
  send_frame(5);
  check(packets==5 && commits==3,"wrong IP accepted");
  send_frame(6);
  check(packets==5 && commits==3,"wrong port accepted");
  send_frame(7);
  check(commits==4 && follow.rx_seq==15132,"unicast metadata not accepted");
  send_frame(8);
  check(commits==5 && follow.rx_seq==15132,"stale sequence changed follow ACK");
  send_frame(9);
  check(commits==6 && follow.rx_seq==15133,"refresh after stale packet failed");
  send_frame(10);
  check(packets==9 && errors==1 && payloads==264 && commits==6,"8byte control/padding changed framing");
  send_frame(11);
  check(packets==10 && errors==1 && payloads==281 && commits==6,"17byte padded payload boundary failed");
  send_frame(12);
  check(packets==11 && errors==1 && payloads==299 && commits==6,"18byte unpadded payload boundary failed");
  send_frame(13);
  check(packets==12 && errors==1 && payloads==363 && commits==6,"oversized FOL must be rejected by length");
  send_frame(14);
  check(packets==13 && errors==1 && payloads==395 && commits==7 && follow.rx_seq==15134,"valid frame after oversized payload failed");
  send_frame(15);
  check(packets==13 && commits==7 && follow.rx_seq==15134,"bad IPv4 checksum with valid FCS accepted");
  send_frame(16);
  check(packets==14 && errors==1 && payloads==427 && commits==8 && follow.rx_seq==15135,"IP checksum reject broke following valid frame");
  // Actual Pi sends FOL/VEC/TGT in one loop. Exercise the minimum Ethernet
  // interframe gap, with no artificial software pause for mailbox ACK.
  wire_frame(17);repeat(12)@(negedge rx_clk);
  wire_frame(18);repeat(12)@(negedge rx_clk);
  wire_frame(19);repeat(120)@(negedge rx_clk);
  check(packets==17 && errors==1 && payloads==523 && commits==11,"minimum-gap FOL/VEC/TGT burst lost metadata");
  check(follow.rx_seq==15136 && follow.catalog_seq==15137 && follow.vec_sequence==77,"minimum-gap burst failed follow decode/ACK");
  repeat(1000100)@(negedge clk);
  check(follow.applied_pan==1800 && follow.applied_tilt==1300,"new pose not applied by next20ms frame");
  $display("PASS real GMII UDP ingress %0d checks, %0d packets, %0d mailbox commits; broadcast/unicast,CCITT,FCS,IP,port,sequence,PWM,RBT ACK",checks,packets,commits);
  $finish;
 end
 initial begin #30000000;$fatal(1,"UDP ingress test timeout");end
endmodule
