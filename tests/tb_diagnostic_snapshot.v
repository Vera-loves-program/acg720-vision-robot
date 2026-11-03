`timescale 1ns/1ps
module tb_diagnostic_snapshot;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,cmd_valid=0,event_change=0;
 reg[3:0] field=1;reg signed[31:0] value=0;
 wire ready,rsp,capture,busy,valid;wire[7:0] code;
 wire signed[31:0] result;wire[5:0] index;wire[31:0] data;wire[15:0] epoch;
 wire core_action,takeover,turn_req,task_req;reg[1535:0] words=0;
 integer n,cases=0;
 motor_vofa_settings_adapter adapter(.clk(clk),.reset_n(rst),.safety_stop(1'b1),
  .cmd_valid(cmd_valid),.cmd_ready(ready),.cmd_seq(16'd11),.cmd_op(4'd10),.cmd_wheel(2'd0),.cmd_field(field),.cmd_value(value),
  .rsp_valid(rsp),.rsp_ready(1'b1),.rsp_status(code),.rsp_value(result),
  .core_action_valid(core_action),.takeover(takeover),.turn_request(turn_req),.turn_ready(1'b0),.turn_active(1'b0),
  .task_valid(task_req),.task_ready(1'b0),.task_rsp_valid(1'b0),.task_rsp_status(8'd0),.task_rsp_value(32'sd0),
  .diagnostic_capture(capture),.diagnostic_index(index),.diagnostic_data(data),.diagnostic_busy(busy),.diagnostic_valid(valid),.diagnostic_epoch(epoch),
  .core_stop_override(1'b1),.core_armed(1'b0),.core_running(1'b0),.core_mode_position(1'b0),.core_link_fault(1'b1),
  .core_fault_left(8'd255),.core_fault_right(8'd255),.core_selected_value(32'sd0),.core_min_duty(10'd0),.core_max_duty(10'd600));
 robot_diag_snapshot snapshot(.clk(clk),.reset_n(rst),.capture(capture),.epoch_event(event_change),.live_words(words),
  .read_index(index),.read_data(data),.busy(busy),.valid(valid),.epoch(epoch));
 task step;begin @(negedge clk);end endtask
 task request;input[3:0] f;input signed[31:0] v;begin
  while(!ready)step;field=f;value=v;cmd_valid=1;step;cmd_valid=0;
  n=0;while(!rsp && n<200)begin step;n=n+1;end
  if(!rsp)$fatal(1,"diagnostic command timeout");cases=cases+1;
 end endtask
 always @(posedge clk)if(rst && (core_action || takeover || turn_req || task_req))$fatal(1,"read-only diagnostic changed motion owner");
 initial begin
  repeat(3)step;rst=1;for(n=0;n<48;n=n+1)words[n*32+:32]=32'hf0010000+n;
  request(0,0);if(code!=4)$fatal(1,"uninitialized snapshot accepted");step;
  request(1,0);if(code!=0 || result!=1 || !valid)$fatal(1,"capture ACK before completed RAM");step;
  words=0;request(0,0);if(code!=0 || result!==32'hf0010000)$fatal(1,"frozen word0 not stable");step;
  request(0,45);if(result!==32'hf001002d)$fatal(1,"indexed RAM read latency wrong");step;
  request(0,48);if(code!=2)$fatal(1,"out-of-range diagnostic index");step;
  event_change=1;request(1,0);event_change=0;step;
  request(0,47);if(result[15:0]!=2 || !result[16])$fatal(1,"mixed producer epoch not marked");
  $display("PASS diagnostic snapshot/adapter %0d transactions read-only under hardfault",cases);$finish;
 end
 initial begin #1000000;$fatal(1,"timeout");end
endmodule
