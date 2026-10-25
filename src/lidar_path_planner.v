// Stopped-vehicle, known-free path planner. 128x128 @64 mm occupancy input is
// conservatively reduced to 64x64 @128 mm, with tile CENTRES on world
// multiples of128mm. In particular ZERO lies at a tile centre, not its corner;
// a1mm pose sign change must not shift the assumed footprint by128mm.
// Only exact FREE (01) is traversable;
// unknown, tentative occupied and stable occupied are all blocking.
//
// For EACH coarse cell all fine rectangles intersecting its 2x2 footprint
// swept by the body rotation disk must be free. Latch the live swept radius
// (including TURN GAP). Round UP only the bounding box to64mm; the circular
// mask retains the actual millimetre radius (291mm, five-cell bounds).
// Test rectangle-to-rectangle distance, not cell-centre distance: a cell
// whose corner intersects the disk must still be inspected. This removes
// only the old square's outside corners, never measured/unknown obstacles
// in the swept disk. Default kernel:132 fine cells rather than144.
// Consecutive 4-neighbour cells and their shared edge are therefore swept safe
// under this model, including turns. Sensors/latency still require live stops.
//
// BFS searches only known free space, with no exploration or arbitrary SLAM.
// Reverse path traversal returns the FIRST TURN from the start (or the goal
// if straight), avoiding one 128 mm stop per node. Root converts next fine
// grid index to a coarse-cell waypoint, then reuses GOTO. Intermediate
// waypoint acceptance must remain inside that validated coarse footprint.
// Keep wheels stopped while busy; cancel on manual pose/map calibration.
// Freeze handshake shares the map's control RAM port, LCD reads continue.
module lidar_path_planner #(
    parameter RADIUS_CELLS=5,
    parameter TIMEOUT_CYCLES=25000000
) (
    input wire clk,reset_n,start,cancel,
    input wire [6:0] current_x,current_y,goal_x,goal_y,
    input wire [15:0] heading_q6,
    input wire [15:0] clearance_radius_mm,
    input wire map_ready,map_frozen,
    output reg freeze_map,busy,done,reachable,
    output reg [7:0] error_code,
    output reg [6:0] next_x,next_y,
    output reg [12:0] path_length,expanded_nodes,
    output reg plan_rd_en,
    output reg [6:0] plan_rd_x,plan_rd_y,
    input wire [1:0] plan_rd_cell,
    input wire plan_rd_valid
);
    reg [4:0] radius_cells;
    wire [16:0] rounded_radius={1'b0,clearance_radius_mm}+17'd63;
    wire [6:0] kernel_side={1'b0,radius_cells,1'b0}+7'd2;
    wire [10:0] requested_radius=clearance_radius_mm==0 ? RADIUS_CELLS*64 :
        clearance_radius_mm>1984 ? 11'd1984 : clearance_radius_mm[10:0];
    reg [21:0] radius_square_acc,radius_square_shift;
    reg [10:0] radius_square_multiplier;
    reg [3:0] radius_square_step;
    reg [9:0] radius_square_cells;
    wire [21:0] radius_square_next=radius_square_acc+
        (radius_square_multiplier[0] ? radius_square_shift : 22'd0);
    localparam [4:0] IDLE=0,WAIT_FREEZE=1,BUILD_REQ=2,BUILD_WAIT=3,
        BUILD_STORE=4,START_REQ=5,START_CHECK=6,GOAL_REQ=7,GOAL_CHECK=8,
        START_MARK=9,QPOP=10,QWAIT=11,QLATCH=12,NREQ=13,NREAD=14,
        NWRITE=15,NNEXT=16,BACK_REQ=17,BACK_USE=18,FINISH=19,FAIL=20,
        BUILD_NEXT=21,RADIUS_SQUARE=22;
    // errors: 01 freeze/map timeout, 02 start footprint not known free,
    // 03 goal footprint not known free, 04 no known route,
    // 05 queue overflow, 06 corrupt parent chain, 07 cancelled,
    // 08/09 unknown start/goal,10 radius exceeds31-cell model capacity,
    // 11/12 start/goal outside the centred planning lattice.
    reg [4:0] state;
    reg [24:0] watchdog;
    reg [11:0] start_index,goal_index,build_index,current_index,back_index,waypoint_index;
    reg [6:0] exact_goal_x,exact_goal_y;
    reg [5:0] kernel_x,kernel_y;
    reg build_free,build_unknown;
    reg [1:0] neighbour_rank,heading_quadrant,previous_back_direction;
    // BFS remains shortest-path. Only equal-length ties prefer body-forward,
    // then body-right, body-left, body-back at the captured planning heading.
    // Direction codes in parent RAM remain world +X,-X,+Y,-Y.
    function [1:0] ordered_direction;
        input [1:0] quadrant,rank;
        begin
            case({quadrant,rank})
                4'h0:ordered_direction=0;4'h1:ordered_direction=3;
                4'h2:ordered_direction=2;4'h3:ordered_direction=1;
                4'h4:ordered_direction=2;4'h5:ordered_direction=0;
                4'h6:ordered_direction=1;4'h7:ordered_direction=3;
                4'h8:ordered_direction=1;4'h9:ordered_direction=2;
                4'hA:ordered_direction=3;4'hB:ordered_direction=0;
                4'hC:ordered_direction=3;4'hD:ordered_direction=1;
                4'hE:ordered_direction=0;4'hF:ordered_direction=2;
            endcase
        end
    endfunction
    wire [1:0] neighbour_direction=ordered_direction(heading_quadrant,neighbour_rank);
    reg back_direction_valid;
    reg [12:0] queue_head,queue_tail,back_steps;
    wire [7:0] rounded_current_x={1'b0,current_x}+8'd1;
    wire [7:0] rounded_current_y={1'b0,current_y}+8'd1;
    wire [7:0] rounded_goal_x={1'b0,goal_x}+8'd1;
    wire [7:0] rounded_goal_y={1'b0,goal_y}+8'd1;

    // [3]=inflated known free, [2]=visited, [1:0]=direction back to parent.
    // One sequential initialization write per cell, no reset on the memories.
    (* syn_ramstyle="block_ram" *) reg [3:0] nodes[0:4095];
    reg [11:0] node_address;
    reg [3:0] node_write_data,node_read_data;
    reg node_write_enable;
    // Write cycles hold the read register: no read-before-write requirement
    // on GW5AT-60. Each consumer has an explicit preceding read state.
    always @(posedge clk) begin
        if(node_write_enable) nodes[node_address]<=node_write_data;
        else node_read_data<=nodes[node_address];
    end
    (* syn_ramstyle="block_ram" *) reg [11:0] queue[0:4095];
    reg [11:0] queue_write_address,queue_write_data;
    reg queue_write_enable;
    reg [11:0] queue_read_data;
    always @(posedge clk) begin
        if(queue_write_enable) queue[queue_write_address]<=queue_write_data;
        else queue_read_data<=queue[queue_head[11:0]];
    end

    wire signed [9:0] query_x=$signed({3'b000,build_index[5:0],1'b0})-10'sd1+
        $signed({4'd0,kernel_x})-$signed({5'd0,radius_cells});
    wire signed [9:0] query_y=$signed({3'b000,build_index[11:6],1'b0})-10'sd1+
        $signed({4'd0,kernel_y})-$signed({5'd0,radius_cells});
    wire query_in_bounds=query_x>=0 && query_x<128 && query_y>=0 && query_y<128;
    // Kernel coordinate k represents fine rectangle j=k-radius, [j,j+1]
    // in64mm units. Coarse footprint is[0,2]. Its minimum separation from
    // that rectangle is max(-j-1, j-2, 0), independently on each axis.
    // Small square lookup keeps this geometric mask out of the DSP budget.
    function [9:0] square5;
        input [4:0] value;
        begin
            case(value)
                0:square5=0;1:square5=1;2:square5=4;3:square5=9;
                4:square5=16;5:square5=25;6:square5=36;7:square5=49;
                8:square5=64;9:square5=81;10:square5=100;11:square5=121;
                12:square5=144;13:square5=169;14:square5=196;15:square5=225;
                16:square5=256;17:square5=289;18:square5=324;19:square5=361;
                20:square5=400;21:square5=441;22:square5=484;23:square5=529;
                24:square5=576;25:square5=625;26:square5=676;27:square5=729;
                28:square5=784;29:square5=841;30:square5=900;31:square5=961;
            endcase
        end
    endfunction
    wire [5:0] gap_x=kernel_x<{1'b0,radius_cells} ?
        {1'b0,radius_cells}-kernel_x-6'd1 :
        kernel_x>{1'b0,radius_cells}+6'd2 ? kernel_x-{1'b0,radius_cells}-6'd2 : 6'd0;
    wire [5:0] gap_y=kernel_y<{1'b0,radius_cells} ?
        {1'b0,radius_cells}-kernel_y-6'd1 :
        kernel_y>{1'b0,radius_cells}+6'd2 ? kernel_y-{1'b0,radius_cells}-6'd2 : 6'd0;
    wire [10:0] gap_squared={1'b0,square5(gap_x[4:0])}+{1'b0,square5(gap_y[4:0])};
    wire query_required=gap_squared<={1'b0,radius_square_cells};
    reg [11:0] neighbour_index,parent_index;
    reg neighbour_in_bounds;
    always @* begin
        neighbour_index=current_index;neighbour_in_bounds=0;
        case(neighbour_direction)
            0:begin neighbour_index={current_index[11:6],current_index[5:0]+6'd1};
                neighbour_in_bounds=current_index[5:0]!=63;end
            1:begin neighbour_index={current_index[11:6],current_index[5:0]-6'd1};
                neighbour_in_bounds=current_index[5:0]!=0;end
            2:begin neighbour_index={current_index[11:6]+6'd1,current_index[5:0]};
                neighbour_in_bounds=current_index[11:6]!=63;end
            default:begin neighbour_index={current_index[11:6]-6'd1,current_index[5:0]};
                neighbour_in_bounds=current_index[11:6]!=0;end
        endcase
        case(node_read_data[1:0])
            0:parent_index={back_index[11:6],back_index[5:0]-6'd1};
            1:parent_index={back_index[11:6],back_index[5:0]+6'd1};
            2:parent_index={back_index[11:6]-6'd1,back_index[5:0]};
            default:parent_index={back_index[11:6]+6'd1,back_index[5:0]};
        endcase
    end
    always @* begin
        plan_rd_en=0;plan_rd_x=query_x[6:0];plan_rd_y=query_y[6:0];
        node_address=0;node_write_enable=0;node_write_data=0;
        queue_write_enable=0;queue_write_address=queue_tail[11:0];queue_write_data=neighbour_index;
        if(state==BUILD_REQ && query_required && query_in_bounds) plan_rd_en=1;
        if(state==BUILD_STORE) begin
            // Before BFS, bit2 explains a rejected footprint (unknown vs hit).
            // Free nodes start unvisited; BFS reuses bit2 as its visited bit.
            node_address=build_index;node_write_enable=1;node_write_data={build_free,build_unknown,2'b00};
        end else if(state==START_REQ || state==START_CHECK) node_address=start_index;
        else if(state==GOAL_REQ || state==GOAL_CHECK) node_address=goal_index;
        else if(state==START_MARK) begin
            node_address=start_index;node_write_enable=1;node_write_data=4'b1100;
            queue_write_enable=1;queue_write_address=0;queue_write_data=start_index;
        end else if(state==NREQ || state==NREAD || state==NWRITE) begin
            node_address=neighbour_index;
            if(state==NWRITE) begin
                node_write_enable=1;node_write_data={2'b11,neighbour_direction};
                queue_write_enable=1;
            end
        end else if(state==BACK_REQ || state==BACK_USE) node_address=back_index;
    end

    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            state<=IDLE;watchdog<=0;freeze_map<=0;busy<=0;done<=0;reachable<=0;
            error_code<=0;next_x<=0;next_y<=0;path_length<=0;expanded_nodes<=0;
            start_index<=0;goal_index<=0;build_index<=0;current_index<=0;back_index<=0;
            waypoint_index<=0;exact_goal_x<=0;exact_goal_y<=0;
            radius_cells<=RADIUS_CELLS;
            radius_square_acc<=0;radius_square_shift<=0;radius_square_multiplier<=0;
            radius_square_step<=0;radius_square_cells<=0;
            kernel_x<=0;kernel_y<=0;build_free<=0;build_unknown<=0;neighbour_rank<=0;heading_quadrant<=0;
            previous_back_direction<=0;back_direction_valid<=0;
            queue_head<=0;queue_tail<=0;back_steps<=0;
        end else begin
            done<=0;
            if(busy) watchdog<=watchdog+1'b1;
            if(cancel && busy) begin
                freeze_map<=0;busy<=0;done<=1;reachable<=0;error_code<=7;state<=IDLE;
            end else if(busy && watchdog==TIMEOUT_CYCLES-1) begin
                freeze_map<=0;busy<=0;done<=1;reachable<=0;error_code<=1;state<=IDLE;
            end else case(state)
                IDLE:begin
                    if(start) begin
                        freeze_map<=1;busy<=1;reachable<=0;error_code<=0;
                        watchdog<=0;path_length<=0;expanded_nodes<=0;
                        state<=WAIT_FREEZE;
                    end
                end
                WAIT_FREEZE:begin
                    if(map_ready && map_frozen) begin
                        // Freeze may have waited for an in-flight odometry
                        // correction. Sample the CURRENT position only now.
                        start_index<={rounded_current_y[6:1],rounded_current_x[6:1]};
                        goal_index<={rounded_goal_y[6:1],rounded_goal_x[6:1]};
                        exact_goal_x<=goal_x;exact_goal_y<=goal_y;
                        heading_quadrant<=heading_q6<16'd2880 || heading_q6>=16'd20160 ? 2'd0 :
                            heading_q6<16'd8640 ? 2'd1 : heading_q6<16'd14400 ? 2'd2 : 2'd3;
                        radius_cells<=clearance_radius_mm==0 ? RADIUS_CELLS :
                            rounded_radius[16:6]>31 ? 5'd31 : rounded_radius[10:6];
                        radius_square_acc<=0;radius_square_shift<={11'd0,requested_radius};
                        radius_square_multiplier<=requested_radius;radius_square_step<=0;
                        build_index<=0;kernel_x<=0;kernel_y<=0;build_free<=1;build_unknown<=0;
                        // Fine index127 rounds to coarse64, outside this
                        // fixed64x64 lattice. Never wrap it to index0.
                        if(rounded_current_x[7] || rounded_current_y[7])begin error_code<=11;state<=FAIL;end
                        else if(rounded_goal_x[7] || rounded_goal_y[7])begin error_code<=12;state<=FAIL;end
                        else if(clearance_radius_mm>1984)begin error_code<=10;state<=FAIL;end
                        else state<=RADIUS_SQUARE;
                    end
                end
                RADIUS_SQUARE:begin
                    // One setup-only shift/add square, no DSP allocation.
                    // gap_squared is in64mm cells squared, hence divide
                    // actual radius squared by4096. Flooring is exact for
                    // comparison with the integer squared cell separation.
                    radius_square_acc<=radius_square_next;
                    radius_square_shift<=radius_square_shift<<1;
                    radius_square_multiplier<=radius_square_multiplier>>1;
                    if(radius_square_step==10)begin
                        radius_square_cells<=radius_square_next[21:12];state<=BUILD_REQ;
                    end else radius_square_step<=radius_square_step+1'b1;
                end
                BUILD_REQ:begin
                    if(!query_required) state<=BUILD_NEXT;
                    else if(!query_in_bounds) begin build_free<=0;build_unknown<=1;state<=BUILD_STORE;end
                    else state<=BUILD_WAIT;
                end
                BUILD_WAIT:begin
                    if(plan_rd_valid) begin
                        if(plan_rd_cell!=2'b01) begin build_free<=0;build_unknown<=plan_rd_cell==0;state<=BUILD_STORE;end
                        else state<=BUILD_NEXT;
                    end
                end
                BUILD_NEXT:begin
                    if({1'b0,kernel_x}==kernel_side-1) begin
                        kernel_x<=0;
                        if({1'b0,kernel_y}==kernel_side-1) state<=BUILD_STORE;
                        else begin kernel_y<=kernel_y+1'b1;state<=BUILD_REQ;end
                    end else begin kernel_x<=kernel_x+1'b1;state<=BUILD_REQ;end
                end
                BUILD_STORE:begin
                    if(build_index==4095) state<=START_REQ;
                    else begin
                        build_index<=build_index+1'b1;kernel_x<=0;kernel_y<=0;
                        build_free<=1;build_unknown<=0;state<=BUILD_REQ;
                    end
                end
                START_REQ:state<=START_CHECK;
                START_CHECK:begin
                    if(!node_read_data[3]) begin error_code<=node_read_data[2] ? 8'd8 : 8'd2;state<=FAIL;end
                    else state<=GOAL_REQ;
                end
                GOAL_REQ:state<=GOAL_CHECK;
                GOAL_CHECK:begin
                    if(!node_read_data[3]) begin error_code<=node_read_data[2] ? 8'd9 : 8'd3;state<=FAIL;end
                    else begin queue_head<=0;queue_tail<=1;state<=START_MARK;end
                end
                START_MARK:state<=QPOP;
                QPOP:begin
                    if(queue_head==queue_tail) begin error_code<=4;state<=FAIL;end
                    else state<=QWAIT;
                end
                QWAIT:state<=QLATCH;
                QLATCH:begin
                    current_index<=queue_read_data;queue_head<=queue_head+1'b1;
                    expanded_nodes<=expanded_nodes+1'b1;
                    if(queue_read_data==goal_index) begin
                        back_index<=goal_index;waypoint_index<=goal_index;back_steps<=0;
                        back_direction_valid<=0;state<=BACK_REQ;
                    end else begin neighbour_rank<=0;state<=NREQ;end
                end
                NREQ:begin
                    if(neighbour_in_bounds) state<=NREAD;
                    else state<=NNEXT;
                end
                NREAD:begin
                    if(node_read_data[3] && !node_read_data[2]) begin
                        if(queue_tail>=4096) begin error_code<=5;state<=FAIL;end
                        else state<=NWRITE;
                    end else state<=NNEXT;
                end
                NWRITE:begin queue_tail<=queue_tail+1'b1;state<=NNEXT;end
                NNEXT:begin
                    if(neighbour_rank==3) state<=QPOP;
                    else begin neighbour_rank<=neighbour_rank+1'b1;state<=NREQ;end
                end
                BACK_REQ:state<=BACK_USE;
                BACK_USE:begin
                    if(back_index==start_index) begin
                        path_length<=back_steps;
                        if(waypoint_index==goal_index) begin next_x<=exact_goal_x;next_y<=exact_goal_y;end
                        else begin next_x<={waypoint_index[5:0],1'b0};next_y<={waypoint_index[11:6],1'b0};end
                        state<=FINISH;
                    end else if(back_steps>=4096 || !node_read_data[2]) begin
                        error_code<=6;state<=FAIL;
                    end else begin
                        if(back_direction_valid && node_read_data[1:0]!=previous_back_direction)
                            waypoint_index<=back_index;
                        previous_back_direction<=node_read_data[1:0];back_direction_valid<=1;
                        back_index<=parent_index;back_steps<=back_steps+1'b1;state<=BACK_REQ;
                    end
                end
                FINISH:begin freeze_map<=0;busy<=0;done<=1;reachable<=1;state<=IDLE;end
                FAIL:begin freeze_map<=0;busy<=0;done<=1;reachable<=0;state<=IDLE;end
                default:begin freeze_map<=0;busy<=0;reachable<=0;error_code<=6;state<=IDLE;end
            endcase
        end
    end
endmodule
