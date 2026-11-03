`timescale 1ns/1ps
// Executed by check_lidar_planner_hdl.py with Icarus; real DUT memories/FSM.
module tb_lidar_planner_start;
    reg clk=0;always #5 clk=~clk;
    reg reset_n=0,start=0,cancel=0,map_ready=1;
    reg signed [31:0] current_x_mm=0,current_y_mm=0;
    reg [6:0] current_x=64,current_y=64,goal_x=68,goal_y=64;
    reg [15:0] clearance_radius_mm=291;
    wire freeze_map,busy,done,reachable,plan_rd_en;
    wire [7:0] error_code;
    wire [6:0] next_x,next_y,plan_rd_x,plan_rd_y;
    wire [12:0] path_length,expanded_nodes;
    reg [1:0] plan_rd_cell=0;
    reg plan_rd_valid=0;
`ifndef LEGACY_PLANNER
    wire next_is_connector,reject_valid;
    wire signed [9:0] reject_x,reject_y;
    wire [1:0] reject_cell;
    wire [3:0] reject_reason;
    wire [2:0] start_check_phase;
`endif
    lidar_path_planner #(.TIMEOUT_CYCLES(4000000)) dut(
        .clk(clk),.reset_n(reset_n),.start(start),.cancel(cancel),
        .current_x(current_x),.current_y(current_y),.goal_x(goal_x),.goal_y(goal_y),
`ifndef LEGACY_PLANNER
        .current_x_mm(current_x_mm),.current_y_mm(current_y_mm),
        .next_is_connector(next_is_connector),.reject_valid(reject_valid),
        .reject_x(reject_x),.reject_y(reject_y),.reject_cell(reject_cell),
        .reject_reason(reject_reason),.start_check_phase(start_check_phase),
`endif
        .heading_q6(16'd0),.clearance_radius_mm(clearance_radius_mm),
        .map_ready(map_ready),.map_frozen(freeze_map && map_ready),
        .freeze_map(freeze_map),.busy(busy),.done(done),.reachable(reachable),
        .error_code(error_code),.next_x(next_x),.next_y(next_y),
        .path_length(path_length),.expanded_nodes(expanded_nodes),
        .plan_rd_en(plan_rd_en),.plan_rd_x(plan_rd_x),.plan_rd_y(plan_rd_y),
        .plan_rd_cell(plan_rd_cell),.plan_rd_valid(plan_rd_valid));
    reg [1:0] occupancy[0:16383];
    integer requests=0,cycles=0,tests=0;
    integer reply_delay=1,reply_left=0;
    reg replies_enabled=1;
    reg [1:0] reply_cell;
    // A bounded variable-latency synchronous map model. The planner must
    // hold its request coordinate until each actual valid response.
    always @(posedge clk)begin
        plan_rd_valid<=0;
        if(!reset_n)begin reply_left=0;requests=0;end
        else if(replies_enabled)begin
            if(reply_left>0)begin
                reply_left=reply_left-1;
                if(reply_left==0)begin plan_rd_valid<=1;plan_rd_cell<=reply_cell;end
            end
            if(plan_rd_en)begin
                if(reply_left!=0)$fatal(1,"overlapping map requests");
                reply_cell=occupancy[plan_rd_y*128+plan_rd_x];
                reply_left=reply_delay;requests=requests+1;
            end
        end
    end
    task all_free;
        integer i;
        begin for(i=0;i<16384;i=i+1)occupancy[i]=1;end
    endtask
    task pose;
        input integer x,y;
        begin current_x_mm=x;current_y_mm=y;
            current_x=(x+4096)>>>6;current_y=(y+4096)>>>6;end
    endtask
    task begin_plan;
        begin
            @(negedge clk);start=1;
            @(negedge clk);start=0;
        end
    endtask
    task finish_plan;
        input integer expected_error;
        begin
            cycles=0;
            while(!done && cycles<4000100)begin @(negedge clk);cycles=cycles+1;end
            if(!done)$fatal(1,"planner never completed");
            if(error_code!=expected_error || reachable!=(expected_error==0))
                $fatal(1,"expected error%0d got error%0d reach%0d phase%0d",expected_error,error_code,reachable,
`ifndef LEGACY_PLANNER
                    start_check_phase
`else
                    0
`endif
                );
            if(busy || freeze_map)$fatal(1,"completion failed to release freeze/busy");
            tests=tests+1;
            $display("PASS case%0d error%0d next(%0d,%0d) clocks%0d",tests,error_code,next_x,next_y,cycles);
            @(negedge clk);
            if(done)$fatal(1,"done is not a one-cycle event");
        end
    endtask
    integer y,sample_x,sample_y,cell_x,cell_y,k,dx,dy,expected_hit;
    initial begin
        all_free();repeat(3)@(negedge clk);reset_n=1;
        // R7 negative control: cell58 has x[-384,-320], outside the actual
        //291mm disk, yet intersects the containing +/-64mm tile sweep.
        occupancy[64*128+58]=0;pose(0,0);begin_plan();
`ifdef LEGACY_PLANNER
        finish_plan(8);
        $display("PASS legacy negative control: false START UNKNOWN reproduced");
        $finish;
`else
        finish_plan(0);
        if(!next_is_connector || next_x!=66 || next_y!=64 || reject_valid)
            $fatal(1,"special root did not return its separately validated +X connector");
        // Occupied outside the true disk is equally irrelevant at start.
        occupancy[64*128+58]=3;begin_plan();finish_plan(0);
        if(!next_is_connector)$fatal(1,"outside hit unexpectedly bypassed connector");
        // A first connector in the user's goal TILE is still its centre,
        // not the unvalidated exact final XY substituted by integration.
        goal_x=65;begin_plan();finish_plan(0);
        if(!next_is_connector || next_x!=66 || next_y!=64)
            $fatal(1,"goal-tile connector was compressed into exact final leg");
        goal_x=68;
        // UNKNOWN, tentative and stable hits INSIDE the real disk all block.
        all_free();occupancy[64*128+59]=0;begin_plan();finish_plan(8);
        if(!reject_valid || reject_x!=59 || reject_y!=64 || reject_cell!=0 || reject_reason!=1 || start_check_phase!=1)
            $fatal(1,"exact unknown diagnostic is not raw rejected cell");
        occupancy[64*128+59]=2;begin_plan();finish_plan(2);
        if(reject_cell!=2 || reject_reason!=2)$fatal(1,"tentative hit lost");
        occupancy[64*128+59]=3;begin_plan();finish_plan(2);
        if(reject_cell!=3)$fatal(1,"stable hit lost");
        // Closed contact at320mm is blocking, rather than rounding it away.
        all_free();occupancy[64*128+58]=0;clearance_radius_mm=320;
        begin_plan();finish_plan(8);clearance_radius_mm=291;
        // Downstream coarse tile at centreX=512 has leftedge448. Cell65
        // ends at128, exactly320mm away: old ceil-only kernel omitted it.
        // The actual start disk is FREE, so this must be a GOAL rejection.
        all_free();occupancy[64*128+65]=0;pose(1024,0);
        goal_x=72;clearance_radius_mm=320;begin_plan();finish_plan(9);
        if(reject_x!=65 || reject_y!=64 || reject_reason!=7)
            $fatal(1,"coarse64mm-multiple contact omitted");
        clearance_radius_mm=291;goal_x=68;
        // Actual subcell pose matters; neither sign can jump the lattice.
        all_free();occupancy[64*128+58]=0;pose(1,0);begin_plan();finish_plan(0);
        pose(-1,0);begin_plan();finish_plan(0);
        pose(63,0);begin_plan();finish_plan(0);
        pose(-63,0);begin_plan();finish_plan(8);
        if(reject_x!=58)$fatal(1,"subcell pose was not used in disk test");
        // This diagonal UNKNOWN is outside the exact start circle and the
        // +X/+Y full-safe neighbour tiles, but IN the accepted endpoint
        // connector sweep. It must block both first edges, never be forcedFREE.
        all_free();pose(0,0);occupancy[59*128+60]=0;
        begin_plan();finish_plan(4);
        if(!reject_valid || reject_x!=60 || reject_y!=59 || reject_reason!=4 || start_check_phase!=2)
            $fatal(1,"unknown connector bypassed or diagnostic lost");
        // A full unknown stripe blocks future routing although exact start
        // and goal tile are FREE. This executes the actual BFS/parent RAM.
        all_free();for(y=0;y<128;y=y+1)occupancy[y*128+70]=0;
        goal_x=(1000+4096)/64;begin_plan();finish_plan(4);
        if(expanded_nodes<10)$fatal(1,"BFS did not search the reachable side");
        goal_x=68;
        // A goal-tile UNKNOWN still blocks; exact-start exception is local.
        all_free();occupancy[64*128+71]=0;begin_plan();finish_plan(9);
        if(!reject_valid || reject_reason!=7 || reject_x!=71 || reject_cell!=0)
            $fatal(1,"goal rejection lost raw fine-cell diagnostic");
        // Actual footprint beyond map boundaries is UNKNOWN, not wrapped.
        all_free();pose(-3900,0);begin_plan();finish_plan(8);
        if(!reject_valid || reject_reason!=3 || reject_x>=0)$fatal(1,"OOB start wrapped fine index");
        // Capture at freeze, then ignore later live-pose changes.
        all_free();pose(0,0);reply_delay=3;begin_plan();
        wait(start_check_phase==1);@(negedge clk);pose(-3900,0);
        finish_plan(0);reply_delay=1;
        if(next_is_connector)$fatal(1,"fully safe root should retain first-turn compression");
        // Cancel while a serial precise square is in progress.
        pose(0,0);begin_plan();wait(start_check_phase==1);repeat(5)@(negedge clk);
        cancel=1;@(negedge clk);cancel=0;finish_plan(7);
        // Invalid radius and unrepresentable upper boundary fail closed.
        clearance_radius_mm=1985;begin_plan();finish_plan(10);clearance_radius_mm=291;
        pose(4032,0);begin_plan();finish_plan(11);
        pose(0,0);current_x=65;begin_plan();finish_plan(11);
        // Missing map reply times out, releasing RAM ownership; later valid
        // request is independent of the cancelled/expired prior transaction.
        pose(0,0);replies_enabled=0;begin_plan();finish_plan(1);
        replies_enabled=1;reply_left=0;begin_plan();finish_plan(0);
        // Sweep fractional poses and diagonal/axis boundary cells using an
        // independent testbench millimetre rectangle-distance oracle. Stop
        // after phase1 succeeds; these are actual executed HDL circle masks,
        // not assertions on a translated Python planner or source text.
        for(k=0;k<32;k=k+1)begin
            all_free();
            sample_x=(k%8)*19-63;sample_y=((k*7)%9)*15-61;
            pose(sample_x,sample_y);
            cell_x=current_x+(k%2 ? 1 : -1)*(3+k%3);
            cell_y=current_y+(k%4<2 ? 1 : -1)*((k/4)%5);
            occupancy[cell_y*128+cell_x]=0;
            dx=cell_x*64-4096-sample_x;
            if(dx<0)begin dx=sample_x-(cell_x*64-4096+64);if(dx<0)dx=0;end
            dy=cell_y*64-4096-sample_y;
            if(dy<0)begin dy=sample_y-(cell_y*64-4096+64);if(dy<0)dy=0;end
            expected_hit=dx*dx+dy*dy<=291*291;
            begin_plan();cycles=0;
            while(!done && start_check_phase!=3 && cycles<10000)begin @(negedge clk);cycles=cycles+1;end
            if(cycles>=10000)$fatal(1,"precise checker exceeded bounded serial scan");
            if(expected_hit)begin
                finish_plan(8);
                if(reject_x!=cell_x || reject_y!=cell_y || reject_cell!=0 || reject_reason!=1)
                    $fatal(1,"oracle rejection mismatch sample%0d",k);
            end else begin
                if(done)$fatal(1,"false exact-pose rejection sample%0d",k);
                cancel=1;@(negedge clk);cancel=0;finish_plan(7);
            end
        end
        $display("PASS %0d dynamic planner scenarios: exact pose, connector, UNKNOWN/hits, bounds, BFS, timeout/cancel",tests);
        $finish;
`endif
    end
endmodule
