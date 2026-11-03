`timescale 1ns/1ps
// Actual normal-vision PWM regression; no device or servo connection.
module tb_follow_pwm_frame;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,enable=0;
 reg [11:0] pulse=1200,low=500,high=2500;
 wire pwm;
 wire [11:0] applied;
 integer clocks=0,checks=0;
 robot_follow_pwm dut(.clk(clk),.reset_n(rst),.enable(enable),.pulse_us(pulse),
  .low_us(low),.high_us(high),.pwm(pwm),.applied_us(applied));
 task step;begin @(negedge clk);end endtask
 initial begin
  repeat(3)step;rst=1;enable=1;step;
  if(applied!=1200 || !pwm)$fatal(1,"first valid command not applied");checks=checks+1;
  repeat(99)step;pulse=1800;step;
  if(applied!=1200)$fatal(1,"valid step changed current pulse midframe");checks=checks+1;
  clocks=0;
  while(applied!=1800 && clocks<1000002)begin step;clocks=clocks+1;end
  if(applied!=1800 || clocks>1000001)$fatal(1,"600us step slowed by FPGA slew/frame");checks=checks+1;
  while(dut.phase<90000)step;
  if(pwm || applied!=1800)$fatal(1,"1800us pulse did not stop at90000 clocks");checks=checks+1;
  enable=0;#1;if(pwm)$fatal(1,"disable delayed output");step;
  if(applied!=0)$fatal(1,"disable retained applied command");checks=checks+1;
  enable=1;pulse=3000;step;
  if(pwm || applied!=0)$fatal(1,"out-of-window request accepted");checks=checks+1;
  pulse=2100;step;
  if(applied!=2100)$fatal(1,"valid restart pulse was ramped");checks=checks+1;
  low=0;step;
  if(pwm || applied!=0)$fatal(1,"invalid bounds accepted");checks=checks+1;
  $display("PASS actual normal-vision PWM %0d checks:600us directstep next20msframe,50cycles/us,noextra1us-frame slew,bounds/disable",checks);
  $finish;
 end
 initial begin #50000000;$fatal(1,"PWM audit timeout");end
endmodule
