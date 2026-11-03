`timescale 1ns/1ps
module tb_motor_edit_equivalence;
 reg clk=0;
 reg reset_n=0;
 reg safety_stop=0;
 reg [7:0] fault_left=0;
 reg [7:0] fault_right=0;
 reg ui_action_valid=0;
 reg [7:0] ui_action=0;
 reg signed [31:0] ui_argument=0;
 reg uart_action_valid=0;
 reg [7:0] uart_action=0;
 reg signed [31:0] uart_argument=0;
 reg uart_heartbeat=0;
 reg uart_live_update=0;
 always #5 clk=~clk;
 wire old_armed;
 wire new_armed;
 wire old_running;
 wire new_running;
 wire old_mode_position;
 wire new_mode_position;
 wire old_position_unlocked;
 wire new_position_unlocked;
 wire old_link_fault;
 wire new_link_fault;
 wire old_remote_owner;
 wire new_remote_owner;
 wire old_clear_fault;
 wire new_clear_fault;
 wire old_clear_stop;
 wire new_clear_stop;
 wire old_zero_position;
 wire new_zero_position;
 wire old_selected_wheel;
 wire new_selected_wheel;
 wire old_edit_position;
 wire new_edit_position;
 wire [3:0] old_selected_field;
 wire [3:0] new_selected_field;
 wire [1:0] old_step_select;
 wire [1:0] new_step_select;
 wire old_encoder_invert_left;
 wire new_encoder_invert_left;
 wire old_encoder_invert_right;
 wire new_encoder_invert_right;
 wire old_drive_invert_left;
 wire new_drive_invert_left;
 wire old_drive_invert_right;
 wire new_drive_invert_right;
 wire signed [15:0] old_target_speed_left;
 wire signed [15:0] new_target_speed_left;
 wire signed [15:0] old_target_speed_right;
 wire signed [15:0] new_target_speed_right;
 wire signed [31:0] old_target_position_left;
 wire signed [31:0] new_target_position_left;
 wire signed [31:0] old_target_position_right;
 wire signed [31:0] new_target_position_right;
 wire [15:0] old_speed_kp_left;
 wire [15:0] new_speed_kp_left;
 wire [15:0] old_speed_ki_left;
 wire [15:0] new_speed_ki_left;
 wire [15:0] old_speed_kd_left;
 wire [15:0] new_speed_kd_left;
 wire [15:0] old_speed_kp_right;
 wire [15:0] new_speed_kp_right;
 wire [15:0] old_speed_ki_right;
 wire [15:0] new_speed_ki_right;
 wire [15:0] old_speed_kd_right;
 wire [15:0] new_speed_kd_right;
 wire [15:0] old_position_kp_left;
 wire [15:0] new_position_kp_left;
 wire [15:0] old_position_ki_left;
 wire [15:0] new_position_ki_left;
 wire [15:0] old_position_kd_left;
 wire [15:0] new_position_kd_left;
 wire [15:0] old_position_kp_right;
 wire [15:0] new_position_kp_right;
 wire [15:0] old_position_ki_right;
 wire [15:0] new_position_ki_right;
 wire [15:0] old_position_kd_right;
 wire [15:0] new_position_kd_right;
 wire [9:0] old_max_duty_permille;
 wire [9:0] new_max_duty_permille;
 wire [9:0] old_min_duty_permille;
 wire [9:0] new_min_duty_permille;
 wire [15:0] old_speed_deadband;
 wire [15:0] new_speed_deadband;
 wire [15:0] old_position_deadband;
 wire [15:0] new_position_deadband;
 wire signed [31:0] old_selected_value;
 wire signed [31:0] new_selected_value;
 wire [31:0] old_selected_step;
 wire [31:0] new_selected_step;
 motor_tuning_control_reference #(.CLK_HZ(1000)) old_dut(.clk(clk),.reset_n(reset_n),.safety_stop(safety_stop),.fault_left(fault_left),.fault_right(fault_right),.ui_action_valid(ui_action_valid),.ui_action(ui_action),.ui_argument(ui_argument),.uart_action_valid(uart_action_valid),.uart_action(uart_action),.uart_argument(uart_argument),.uart_heartbeat(uart_heartbeat),.uart_live_update(uart_live_update),.armed(old_armed),.running(old_running),.mode_position(old_mode_position),.position_unlocked(old_position_unlocked),.link_fault(old_link_fault),.remote_owner(old_remote_owner),.clear_fault(old_clear_fault),.clear_stop(old_clear_stop),.zero_position(old_zero_position),.selected_wheel(old_selected_wheel),.edit_position(old_edit_position),.selected_field(old_selected_field),.step_select(old_step_select),.encoder_invert_left(old_encoder_invert_left),.encoder_invert_right(old_encoder_invert_right),.drive_invert_left(old_drive_invert_left),.drive_invert_right(old_drive_invert_right),.target_speed_left(old_target_speed_left),.target_speed_right(old_target_speed_right),.target_position_left(old_target_position_left),.target_position_right(old_target_position_right),.speed_kp_left(old_speed_kp_left),.speed_ki_left(old_speed_ki_left),.speed_kd_left(old_speed_kd_left),.speed_kp_right(old_speed_kp_right),.speed_ki_right(old_speed_ki_right),.speed_kd_right(old_speed_kd_right),.position_kp_left(old_position_kp_left),.position_ki_left(old_position_ki_left),.position_kd_left(old_position_kd_left),.position_kp_right(old_position_kp_right),.position_ki_right(old_position_ki_right),.position_kd_right(old_position_kd_right),.max_duty_permille(old_max_duty_permille),.min_duty_permille(old_min_duty_permille),.speed_deadband(old_speed_deadband),.position_deadband(old_position_deadband),.selected_value(old_selected_value),.selected_step(old_selected_step));
 motor_tuning_control #(.CLK_HZ(1000)) new_dut(.clk(clk),.reset_n(reset_n),.safety_stop(safety_stop),.fault_left(fault_left),.fault_right(fault_right),.ui_action_valid(ui_action_valid),.ui_action(ui_action),.ui_argument(ui_argument),.uart_action_valid(uart_action_valid),.uart_action(uart_action),.uart_argument(uart_argument),.uart_heartbeat(uart_heartbeat),.uart_live_update(uart_live_update),.armed(new_armed),.running(new_running),.mode_position(new_mode_position),.position_unlocked(new_position_unlocked),.link_fault(new_link_fault),.remote_owner(new_remote_owner),.clear_fault(new_clear_fault),.clear_stop(new_clear_stop),.zero_position(new_zero_position),.selected_wheel(new_selected_wheel),.edit_position(new_edit_position),.selected_field(new_selected_field),.step_select(new_step_select),.encoder_invert_left(new_encoder_invert_left),.encoder_invert_right(new_encoder_invert_right),.drive_invert_left(new_drive_invert_left),.drive_invert_right(new_drive_invert_right),.target_speed_left(new_target_speed_left),.target_speed_right(new_target_speed_right),.target_position_left(new_target_position_left),.target_position_right(new_target_position_right),.speed_kp_left(new_speed_kp_left),.speed_ki_left(new_speed_ki_left),.speed_kd_left(new_speed_kd_left),.speed_kp_right(new_speed_kp_right),.speed_ki_right(new_speed_ki_right),.speed_kd_right(new_speed_kd_right),.position_kp_left(new_position_kp_left),.position_ki_left(new_position_ki_left),.position_kd_left(new_position_kd_left),.position_kp_right(new_position_kp_right),.position_ki_right(new_position_ki_right),.position_kd_right(new_position_kd_right),.max_duty_permille(new_max_duty_permille),.min_duty_permille(new_min_duty_permille),.speed_deadband(new_speed_deadband),.position_deadband(new_position_deadband),.selected_value(new_selected_value),.selected_step(new_selected_step));
 integer checked=0,wheel,edit_mode,field,step,k; reg signed [31:0] value;
 always @(posedge clk)begin #1;if(reset_n)begin checked=checked+1;if({old_armed,old_running,old_mode_position,old_position_unlocked,old_link_fault,old_remote_owner,old_clear_fault,old_clear_stop,old_zero_position,old_selected_wheel,old_edit_position,old_selected_field,old_step_select,old_encoder_invert_left,old_encoder_invert_right,old_drive_invert_left,old_drive_invert_right,old_target_speed_left,old_target_speed_right,old_target_position_left,old_target_position_right,old_speed_kp_left,old_speed_ki_left,old_speed_kd_left,old_speed_kp_right,old_speed_ki_right,old_speed_kd_right,old_position_kp_left,old_position_ki_left,old_position_kd_left,old_position_kp_right,old_position_ki_right,old_position_kd_right,old_max_duty_permille,old_min_duty_permille,old_speed_deadband,old_position_deadband,old_selected_value,old_selected_step} !== {new_armed,new_running,new_mode_position,new_position_unlocked,new_link_fault,new_remote_owner,new_clear_fault,new_clear_stop,new_zero_position,new_selected_wheel,new_edit_position,new_selected_field,new_step_select,new_encoder_invert_left,new_encoder_invert_right,new_drive_invert_left,new_drive_invert_right,new_target_speed_left,new_target_speed_right,new_target_position_left,new_target_position_right,new_speed_kp_left,new_speed_ki_left,new_speed_kd_left,new_speed_kp_right,new_speed_ki_right,new_speed_kd_right,new_position_kp_left,new_position_ki_left,new_position_kd_left,new_position_kp_right,new_position_ki_right,new_position_kd_right,new_max_duty_permille,new_min_duty_permille,new_speed_deadband,new_position_deadband,new_selected_value,new_selected_step})$fatal(1,"motor editor mismatch cycle%0d action%0d field%0d arg%0d",checked,ui_action_valid ? ui_action : uart_action,new_selected_field,ui_action_valid ? ui_argument : uart_argument);end end

 task action;
  input [7:0] code;input signed [31:0] arg;input remote;
  begin @(negedge clk);ui_action=code;uart_action=code;ui_argument=arg;uart_argument=arg;
   ui_action_valid=!remote;uart_action_valid=remote;
   @(negedge clk);ui_action_valid=0;uart_action_valid=0;
  end
 endtask
 initial begin
  repeat(4)@(negedge clk);reset_n=1;uart_live_update=1;
  for(wheel=0;wheel<2;wheel=wheel+1)begin
   action(16,wheel,0);
   for(edit_mode=0;edit_mode<2;edit_mode=edit_mode+1)begin
    action(17,edit_mode,0);
    for(field=0;field<10;field=field+1)begin
     action(18,field,0);
     for(step=0;step<3;step=step+1)begin
      action(20,step,0);
      for(k=0;k<20;k=k+1)begin
       case(k)
        0:value=32'sh80000000;1:value=-2000001;2:value=-2000000;
        3:value=-201;4:value=-200;5:value=-1;6:value=0;7:value=1;
        8:value=32;9:value=50;10:value=200;11:value=256;
        12:value=600;13:value=900;14:value=1000;15:value=32767;
        16:value=2000000;17:value=2000001;18:value=32'sh7fffffff;default:value=$random;
       endcase
       action(19,value,k%2);action(5,0,0);action(6,0,1);
      end
     end
    end
   end
  end
  for(k=0;k<20000;k=k+1)begin
   safety_stop=(k%117)==0;fault_left=(k%127)==0;fault_right=(k%131)==0;
   uart_live_update=(k%3)!=0;uart_heartbeat=k%2;
   action(1+($unsigned($random)%25),$random,k%2);
  end
  repeat(4)@(negedge clk);
  $display("PASS: motor editor cycle-equivalence %0d cycles; all outputs,32bit writes,both wheels/modes/fields,limits,STOP",checked);
  $finish;
 end
 initial begin #2000000;$fatal(1,"equivalence timeout");end
endmodule
