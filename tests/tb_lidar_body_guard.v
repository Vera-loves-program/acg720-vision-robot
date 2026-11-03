`timescale 1ns/1ps
// Dynamic unit test: serial projection/classification is actually clocked.
// Geometry/ZERO here are explicit test fixtures, not installation calibration.
module tb_lidar_body_guard;
    reg clk=0;always #10 clk=~clk;
    reg reset_n=0,active=1,tick_10ms=0,point_valid=0,scan_start=0;
    reg raw_node_valid=0,raw_parse_error=0;
    reg [2:0] raw_reject_reason=0;
    reg [14:0] angle=0,zero=0;
    reg [15:0] distance=0,scan_id=0,front=232;
    reg [5:0] quality=0;
    wire ready,front_stop,rear_stop,front_slow,turn_stop,contact,projection_bad,last_bad;
    wire [3:0] hits,free_coverage;
    wire [5:0] age;
    wire [7:0] period_ticks,parse_errors,overruns;
    wire [9:0] raw_nodes,usable_nodes,invalid_nodes;
    wire [31:0] direction_counts;
    wire [4:0] unconfirmed,trigger_kind;
    wire trigger_valid;
    wire [14:0] trigger_angle;
    wire [15:0] trigger_distance,trigger_scan;
    wire [5:0] trigger_quality,map_quality;
    wire [14:0] map_angle;
    wire [15:0] map_distance;
    wire signed [15:0] trigger_x,trigger_y,bx,by;
    wire [15:0] self_points,radius,left_mm;
    wire map_valid;
    integer map_count=0,last_quality=0,cycles=0,tests=0,max_service=0;
    real expected_x,expected_y,error_x,error_y,tolerance;
    lidar_body_guard dut(
        .clk(clk),.reset_n(reset_n),.tick_10ms(tick_10ms),.active(active),
        .point_valid(point_valid),.scan_start(scan_start),.angle_q6(angle),.forward_angle_q6(zero),
        .distance_mm(distance),.quality(quality),.raw_node_valid(raw_node_valid),
        .raw_reject_reason(raw_reject_reason),.raw_parse_error(raw_parse_error),.scan_id(scan_id),
        .offset_x_mm(16'sd33),.offset_y_mm(-16'sd77),
        .body_front_mm(front),.body_rear_mm(16'd118),.body_left_mm(16'd119),.body_right_mm(16'd119),
        .drive_margin_mm(16'd20),.turn_margin_mm(16'd30),.contact_margin_mm(16'd10),
        .ready(ready),.front_stop(front_stop),.rear_stop(rear_stop),.front_slow(front_slow),
        .turn_stop(turn_stop),.virtual_contact(contact),.coverage(hits),.free_coverage(free_coverage),
        .scan_age_ticks(age),.scan_period_ticks(period_ticks),.projection_bad(projection_bad),.last_scan_bad(last_bad),
        .scan_raw_nodes(raw_nodes),.scan_usable_nodes(usable_nodes),.scan_invalid_nodes(invalid_nodes),
        .scan_parse_errors(parse_errors),.scan_overruns(overruns),.direction_point_counts(direction_counts),
        .unconfirmed_flags(unconfirmed),.trigger_valid(trigger_valid),.trigger_kind(trigger_kind),
        .trigger_angle_q6(trigger_angle),.trigger_distance_mm(trigger_distance),.trigger_scan_id(trigger_scan),
        .trigger_quality(trigger_quality),.trigger_x_mm(trigger_x),.trigger_y_mm(trigger_y),
        .self_points(self_points),.rotation_radius_mm(radius),.filtered_left_mm(left_mm),
        .map_point_valid(map_valid),.body_point_x(bx),.body_point_y(by),.map_point_quality(map_quality),
        .map_point_angle(map_angle),.map_point_distance(map_distance)
    );
    always @(posedge clk)begin
        cycles<=cycles+1;
        if(map_valid)begin
            map_count=map_count+1;last_quality=map_quality;
            if(map_quality==0)$fatal(1,"quality0 emitted to mapper");
            expected_x=33.0+map_distance*$cos(($itor(zero)-$itor(map_angle))*3.141592653589793/11520.0);
            expected_y=-77.0+map_distance*$sin(($itor(zero)-$itor(map_angle))*3.141592653589793/11520.0);
            error_x=$itor(bx)-expected_x;error_y=$itor(by)-expected_y;
            if(error_x<0)error_x=-error_x;if(error_y<0)error_y=-error_y;
            tolerance=map_distance<=1000 ? 3.0 : 20.0;
            if(error_x>tolerance || error_y>tolerance)
                $fatal(1,"actual serial coordinate wrong: raw=%0d/%0d ZERO=%0d XY=%0d,%0d expected=%f,%f",map_angle,map_distance,zero,bx,by,expected_x,expected_y);
        end
    end
    task check;
        input condition;input [511:0] description;
        begin tests=tests+1;if(condition!==1'b1)$fatal(1,"%0s",description);end
    endtask
    task idle;
        integer budget;
        begin
            budget=0;
            while(dut.state!=6 && budget<600)begin @(negedge clk);budget=budget+1;end
            check(budget<600,"guard must finish serial service within600 clocks");
            if(budget>max_service)max_service=budget;
            @(negedge clk);
        end
    endtask
    task boundary;
        begin idle;scan_id=scan_id+1;scan_start=1;@(negedge clk);scan_start=0;@(negedge clk);end
    endtask
    task tick;
        begin @(negedge clk);tick_10ms=1;@(negedge clk);tick_10ms=0;end
    endtask
    task reset_guard;
        begin
            @(negedge clk);reset_n=0;point_valid=0;raw_node_valid=0;raw_parse_error=0;scan_start=0;
            zero=0;front=232;scan_id=0;repeat(3)@(negedge clk);reset_n=1;idle;boundary;
            check(radius==291,"measured default swept radius remains291mm");
        end
    endtask
    task node;
        input integer a,d,q;
        integer before_count;
        begin
            idle;before_count=map_count;
            angle=a;distance=d;quality=q;raw_node_valid=1;
            raw_reject_reason={(d>12000),(d<50),(q==0)};
            point_valid=(q!=0 && d>=50 && d<=12000);
            @(negedge clk);point_valid=0;raw_node_valid=0;
            idle;
            if(map_count!=before_count)check(last_quality==q,"mapper preserves original6bit quality through projection");
        end
    endtask
    task clear_scan;
        integer bin,a;
        begin
            for(bin=0;bin<128;bin=bin+1)begin
                a=(bin*180+30-2880+zero+23040)%23040;node(a,1000,12);
                a=(bin*180+100-2880+zero+23040)%23040;node(a,1010,13);
            end
        end
    endtask
    integer before_count,start_cycle,sweep_zero,sweep_angle,sweep_range;
    initial begin
        reset_guard;
        before_count=map_count;node(0,100,20);node(0,0,0);
        repeat(10)tick;
        boundary;
        check(ready && !last_bad,"noecho does not make processing/serial health falsely bad");
        check(raw_nodes==2 && usable_nodes==1 && invalid_nodes==1,"raw usable/invalid counters remain separate");
        check(self_points==1 && hits==0 && free_coverage==0 && map_count==before_count,"SELF/noecho never become FREE or mapper points");
        check(period_ticks==10,"scan period counts10ms enables independently");

        reset_guard;clear_scan;boundary;
        check(ready && hits==15 && free_coverage==15,"all128 bins with2 far rays establish sampled visibility");
        check(direction_counts==32'h40404040,"per-direction counts report64 external hits each");
        check(!front_stop && !rear_stop && !turn_stop && !contact,"empty measured room does not invent obstacle");
        repeat(30)tick;check(!ready && age>=30,"scan age expires independently of coverage");

        reset_guard;node(0,215,23);
        check(!front_stop && !turn_stop && unconfirmed[0],"single ordinary marginal return remains unconfirmed");
        node(64,215,21);
        check(front_stop && turn_stop && !contact,"two nearby ordinary rays confirm without waiting whole scan");
        check(trigger_valid && trigger_angle==64 && trigger_distance==215 && trigger_quality==21,"trigger retains actual raw confirming ray");
        check(trigger_x>=246 && trigger_x<=250 && trigger_y>=-83 && trigger_y<=-79 && trigger_scan==scan_id,"trigger records bodyXY and scan identity");

        reset_guard;node(0,215,23);node(640,215,23);
        check(!front_stop && !turn_stop,"angularly distant rays cannot confirm one cluster");
        reset_guard;node(5760,80,23);node(5760,160,23);
        check(!turn_stop && !contact,"range-discontinuous rays cannot confirm one cluster");

        reset_guard;start_cycle=cycles;node(0,200,7);
        check(contact && front_stop && turn_stop,"credible single10mm near-contact return stops immediately");
        check(cycles-start_cycle<600 && trigger_quality==7 && trigger_x==233 && trigger_y==-77,"near-contact fast path records233,-77 actual point");
        reset_guard;node(0,200,0);boundary;
        check(!contact && !front_stop && free_coverage==0 && invalid_nodes==1,"quality0 cannot trigger contact or free evidence");

        reset_guard;clear_scan;node(0,215,23);node(30,1000,20);node(90,1000,20);boundary;
        check(hits==15 && !free_coverage[0] && free_coverage[3:1]==7,"near point veto stays for entire bin despite later far rays");

        reset_guard;
        @(negedge clk);raw_parse_error=1;@(negedge clk);raw_parse_error=0;
        angle=0;distance=1000;quality=17;raw_reject_reason=0;raw_node_valid=1;point_valid=1;
        @(negedge clk);point_valid=0;raw_node_valid=0;
        repeat(2)@(negedge clk);point_valid=1;raw_node_valid=1;
        @(negedge clk);point_valid=0;raw_node_valid=0;idle;
        check(projection_bad && !ready,"busy input overrun invalidates health without pretending obstacle");
        boundary;check(last_bad && overruns==1 && parse_errors==1 && free_coverage==0,"scan diagnostics retain exact overrun/parse evidence");
        node(0,1000,20);boundary;check(ready && !last_bad,"next complete good scan restores processing health");

        reset_guard;zero=5760;idle;boundary;node(0,100,20);node(0,800,20);boundary;
        check(self_points==1 && hits==4 && left_mm==800,"explicit ZERO90 fixture rejects100mm SELF and retains800mm left wall");
        check(!turn_stop,"offset left wall is not an invented swept obstacle");

        reset_guard;node(0,215,20);front=233;idle;
        check(!ready && !trigger_valid && free_coverage==0,"geometry change invalidates old health/evidence");
        for(sweep_zero=0;sweep_zero<4;sweep_zero=sweep_zero+1)begin
            reset_guard;zero=sweep_zero*5760;idle;boundary;
            for(sweep_angle=0;sweep_angle<23040;sweep_angle=sweep_angle+960)begin
                for(sweep_range=0;sweep_range<6;sweep_range=sweep_range+1)begin
                    case(sweep_range)
                        0:node(sweep_angle,50,31);1:node(sweep_angle,100,31);
                        2:node(sweep_angle,200,31);3:node(sweep_angle,1000,31);
                        4:node(sweep_angle,4000,31);5:node(sweep_angle,12000,31);
                    endcase
                end
            end
        end
        $display("PASS tb_lidar_body_guard: %0d checks, %0d external coordinate/quality checks, max service %0d clocks; noise/evidence/fastdanger/invalid/SELF/overrun/stale",tests,map_count,max_service);
        $finish;
    end
    initial begin #100000000;$fatal(1,"guard TB watchdog");end
endmodule

// UART-level parser test at the ACTUAL50MHz/460800 fractional ratio.
// Only supervisor START/STOP/motor settling delays are accelerated; no UART,
// decode state, node data or filtering is forced. This is not a cold-start
// mechanical/timing test of the real sensor.
module tb_rplidar_raw_diagnostics;
    reg clk=0;always #10 clk=~clk;
    reg reset_n=0,rx=1;
    wire node_valid,point_valid,parse_error;
    wire [2:0] reason;
    wire [14:0] angle;
    wire [15:0] distance;
    wire [5:0] quality;
    integer nodes=0,usable=0,invalid=0,errors=0,tests=0,wire_phase=0,wire_clocks;
    rplidar_c1 dut(
        .clk(clk),.reset_n(reset_n),.scan_enable(1'b1),.restart(1'b0),.forward_angle_q6(15'd0),
        .lidar_rx(rx),.point_valid(point_valid),.angle_q6(angle),.distance_mm(distance),.quality(quality),
        .raw_node_valid(node_valid),.raw_reject_reason(reason),.raw_parse_error(parse_error)
    );
    always @(posedge clk)begin
        if(node_valid)begin nodes=nodes+1;if(point_valid)usable=usable+1;else invalid=invalid+1;end
        if(parse_error)errors=errors+1;
    end
    always @(negedge clk)begin
        if(reset_n && (dut.state==1 || dut.state==8 || dut.state==18))force dut.ms_tick=1'b1;
        else release dut.ms_tick;
    end
    task bit_wait;
        begin
            wire_phase=wire_phase+50000000;
            wire_clocks=wire_phase/460800;wire_phase=wire_phase%460800;
            repeat(wire_clocks)@(negedge clk);
        end
    endtask
    task byte_tx;
        input [7:0] value;integer bitno;
        begin
            @(negedge clk);rx=0;bit_wait;
            for(bitno=0;bitno<8;bitno=bitno+1)begin rx=value[bitno];bit_wait;end
            rx=1;bit_wait;repeat(20)@(negedge clk);
        end
    endtask
    task node_tx;
        input integer a,q2,q,start;
        begin
            byte_tx((q<<2)|(start ? 1 : 2));byte_tx(((a&127)<<1)|1);byte_tx(a>>7);
            byte_tx(q2&255);byte_tx(q2>>8);repeat(8)@(negedge clk);
        end
    endtask
    task check;
        input condition;input [511:0] description;
        begin tests=tests+1;if(condition!==1'b1)$fatal(1,"%0s",description);end
    endtask
    initial begin
        repeat(4)@(negedge clk);reset_n=1;
        wait(dut.state==9); // HEALTH descriptor after the actual command TX
        byte_tx('hA5);byte_tx('h5A);byte_tx(3);byte_tx(0);byte_tx(0);byte_tx(0);byte_tx(6);
        byte_tx(0);byte_tx(0);byte_tx(0);
        wait(dut.state==11); // Standard SCAN descriptor after motor command
        byte_tx('hA5);byte_tx('h5A);byte_tx(5);byte_tx(0);byte_tx(0);byte_tx('h40);byte_tx('h81);
        node_tx(5760,4000,25,1);check(nodes==1 && usable==1 && angle==5760 && distance==1000 && quality==25,"official Q6/Q2 byte decoding unchanged");
        node_tx(100,0,0,0);check(nodes==2 && invalid==1 && reason==3,"quality0/noecho node remains observable but unusable");
        node_tx(100,4000,0,0);check(reason==1 && invalid==2,"quality0 finite return is diagnostic only");
        node_tx(100,100,25,0);check(reason==2 && invalid==3,"below50mm reason explicit");
        node_tx(100,48004,25,0);check(reason==4 && invalid==4,"above12m reason explicit");
        node_tx(100,201,25,0);check(reason==0 && distance==50 && usable==2,"quarter-mm wire distance still truncates exactly once");
        byte_tx(0);byte_tx(0);check(errors>=2,"bad framing emits parse diagnostic");
        node_tx(100,4000,25,0);check(nodes==7 && usable==3,"parser recovers legal node framing after garbage");
        $display("PASS tb_rplidar_raw_diagnostics: %0d checks; actual50MHz/460800 UART108/109clock bits, invalid nodes, Q2 unchanged; supervisor settle delays accelerated",tests);$finish;
    end
    initial begin #100000000;$fatal(1,"parser TB watchdog");end
endmodule
