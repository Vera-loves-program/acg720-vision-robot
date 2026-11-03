`timescale 1ns/1ps
// Independent reference vectors include maximal carries and alternating header
// values. Check actual default-TX and pipelined-RX latency, not string matching.
module tb_ip_checksum_pipeline;
 reg clk=0;always #4 clk=~clk;
 reg rst=1,cal=0;
 reg [15:0] words[0:8];
 reg [15:0] vectors[0:5139];
 wire [15:0] legacy,pipelined;
 genvar instance_id;
 generate for(instance_id=0;instance_id<2;instance_id=instance_id+1)begin:engines
  wire [15:0] value;
  ip_checksum #(.PIPELINED_FOLD(instance_id)) dut(.clk(clk),.reset_p(rst),.cal_en(cal),
   .IP_ver(words[0][15:12]),.IP_hdr_len(words[0][11:8]),.IP_tos(words[0][7:0]),
   .IP_total_len(words[1]),.IP_id(words[2]),.IP_rsv(words[3][15]),.IP_df(words[3][14]),
   .IP_mf(words[3][13]),.IP_frag_offset(words[3][12:0]),
   .IP_ttl(words[4][15:8]),.IP_protocol(words[4][7:0]),
   .src_ip({words[5],words[6]}),.dst_ip({words[7],words[8]}),.checksum(value));
  if(instance_id==0)assign legacy=value;else assign pipelined=value;
 end endgenerate
 integer vector_id,word_id,checks=0;
 task step;begin @(negedge clk);end endtask
 initial begin
  $readmemh("tmp/follow-udp-ingress-20261103/checksum.hex",vectors);
  for(word_id=0;word_id<9;word_id=word_id+1)words[word_id]=0;
  repeat(3)step;rst=0;
  for(vector_id=0;vector_id<514;vector_id=vector_id+1)begin
   for(word_id=0;word_id<9;word_id=word_id+1)words[word_id]=vectors[vector_id*10+word_id];
   cal=1;step;
   if(legacy!==vectors[vector_id*10+9])$fatal(1,"TX checksum/latency vector%0d",vector_id);
   checks=checks+1;cal=0;step;step;
   if(pipelined!==vectors[vector_id*10+9])$fatal(1,"RX checksum/carry/old-header vector%0d expected%h actual%h",vector_id,vectors[vector_id*10+9],pipelined);
   checks=checks+1;step;
  end
  $display("PASS actual IPv4 checksum %0d assertions:514 headers,carry/previous-header,unchanged default TX latency,pipelined RX latency",checks);
  $finish;
 end
 initial begin #1000000;$fatal(1,"checksum timeout");end
endmodule
