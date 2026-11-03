`timescale 1ns/1ps
// Real planner FSM and parent RAM. The synchronous map is a fixture, not a
// replacement planner. Watch the selected backtrace to distinguish body-left
// from world+Y and check that preference never overrides an unavailable side.
module tb_lidar_detour_preference;
    reg clk=0;always #5 clk=~clk;
    reg reset_n=0,start=0;
    reg [15:0] heading=0;
    reg [6:0] goal_x=86,goal_y=64;
    reg [1:0] cell_value=0;
    reg cell_valid=0;
    wire freeze,busy,done,reachable,read_en;
    wire [6:0] read_x,read_y,next_x,next_y;
    wire [7:0] error_code;
    reg [1:0] cells[0:16383];
    integer checks=0,quadrant=0,back_x,back_y,body_left;
    integer min_left=0,max_left=0,cycles=0,base_x,base_y,world_x,world_y,i;
    lidar_path_planner #(.TIMEOUT_CYCLES(4000000)) dut(
        .clk(clk),.reset_n(reset_n),.start(start),.cancel(1'b0),
        .current_x(7'd64),.current_y(7'd64),
        .goal_x(goal_x),.goal_y(goal_y),.heading_q6(heading),.clearance_radius_mm(16'd291),
        .map_ready(1'b1),.map_frozen(freeze),.freeze_map(freeze),
        .busy(busy),.done(done),.reachable(reachable),.error_code(error_code),
        .next_x(next_x),.next_y(next_y),
        .plan_rd_en(read_en),.plan_rd_x(read_x),.plan_rd_y(read_y),
        .plan_rd_cell(cell_value),.plan_rd_valid(cell_valid));
    always @(posedge clk)begin
        cell_valid<=reset_n && read_en;
        if(read_en)cell_value<=cells[read_y*128+read_x];
        if(reset_n && dut.state==dut.BACK_USE)begin
            back_x=dut.back_index[5:0]*128-4096;
            back_y=dut.back_index[11:6]*128-4096;
            case(quadrant)
                0:body_left=back_y;
                1:body_left=-back_x;
                2:body_left=-back_y;
                default:body_left=back_x;
            endcase
            if(body_left<min_left)min_left=body_left;
            if(body_left>max_left)max_left=body_left;
        end
    end
    task check;input condition;input [511:0] message;begin
        checks=checks+1;
        if(condition!==1'b1)$fatal(1,"%0s quadrant%0d error%0d left[%0d,%0d]",message,quadrant,error_code,min_left,max_left);
    end endtask
    task set_cell;input integer x,y,value;begin
        case(quadrant)
            0:begin world_x=x;world_y=y;end
            1:begin world_x=127-y;world_y=x;end
            2:begin world_x=127-x;world_y=127-y;end
            default:begin world_x=y;world_y=127-x;end
        endcase
        cells[world_y*128+world_x]=value;
    end endtask
    task fixture;input integer q;begin
        quadrant=q;heading=q*5760;
        for(i=0;i<16384;i=i+1)cells[i]=1;
        case(q)
            0:begin goal_x=86;goal_y=64;end
            1:begin goal_x=64;goal_y=86;end
            2:begin goal_x=42;goal_y=64;end
            default:begin goal_x=64;goal_y=42;end
        endcase
        // Symmetric physical obstacle x[640,768], y[-64,+64] mm.
        for(base_x=74;base_x<=75;base_x=base_x+1)
            for(base_y=63;base_y<=64;base_y=base_y+1)set_cell(base_x,base_y,3);
    end endtask
    task plan;begin
        @(negedge clk);min_left=0;max_left=0;start=1;
        @(negedge clk);start=0;cycles=0;
        while(!done && cycles<4000100)begin @(negedge clk);cycles=cycles+1;end
        check(done && reachable && error_code==0,"real planner must find a validated detour");
        check(!busy && !freeze,"completion releases map freeze");
        @(negedge clk);
    end endtask
    integer q;
    initial begin
        repeat(3)@(negedge clk);reset_n=1;
        for(q=0;q<4;q=q+1)begin
            fixture(q);plan;
            check(max_left>0 && min_left==0,"symmetric shortest routes prefer BODY LEFT at every heading");
        end
        fixture(0);
        // A left-side wall makes the right route shorter. The preference is
        // a shortest-route tie-break, never an instruction to force left.
        for(base_x=56;base_x<=93;base_x=base_x+1)set_cell(base_x,71,3);
        plan;check(min_left<0 && max_left==0,"occupied left must permit known-free right detour");
        fixture(0);
        for(base_x=56;base_x<=93;base_x=base_x+1)set_cell(base_x,71,0);
        plan;check(min_left<0 && max_left==0,"UNKNOWN left stays blocked while right remains available");
        $display("PASS tb_lidar_detour_preference: %0d assertions, actual planner backtrace; 4 headings left tie-break, occupied/UNKNOWN alternatives",checks);
        $finish;
    end
    initial begin #300000000;$fatal(1,"detour preference watchdog");end
endmodule
