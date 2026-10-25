// Finite local occupancy map, with conservative translation-only scan matching.
// This is NOT full SLAM: heading and predicted position come from wheel/IMU
// odometry; there is no loop closure, global relocalisation or heading matching.
// Default window [-4096,4096) mm in x/y, 128x128 cells of 64 mm (4 KiB).
// World heading increases anticlockwise; C1 beam angles normally clockwise.
// lidar_offset_* are body-frame offsets from the odometry reference; their
// default integration value is zero until the physical mount is calibrated.
// Valid mount offsets are +/-1000 mm per axis; larger values reject samples.
// Clear the map whenever POS_CALIB changes the world frame discontinuously.
//
// Each scan stores <=1024 rays. A 1/8 subset is matched against the OLD map,
// then integrated. +/-2 cell translation candidates are accepted only with a
// mature reference map, spread in both axes, hits in all four body-relative
// quadrants, enough hits and a unique peak. A single wall is not a reference.
// A rejected match does not invalidate wheel odometry: integration uses its
// predicted pose. Map quality therefore still depends on odometry/gyro quality.
// Capture continues in the opposite scan bank while the previous scan is
// processed. A correction ACK discards only the unconsumed capture epoch.
// One-port ray RAM gives capture writes priority; read-issue states wait for
// a genuine RAM read before consuming its Q. The <=1024-ray compute budget
// plus LCD snapshot is <7 ms at 50 MHz, plus any wait for the odometer's ACK.
// LCD snapshots use separate banks with explicit port CE ownership; the LCD
// never reads the live map while the map engine writes. Check
// real synthesis/timing reports before deployment; these are algorithm budgets.
module lidar_grid_map #(
    parameter MAP_BITS = 7,
    parameter CELL_MM_BITS = 6,
    parameter integer ORIGIN_X_MM = -4096,
    parameter integer ORIGIN_Y_MM = -4096,
    parameter ANGLE_CLOCKWISE = 1,
    parameter integer YAW_OFFSET_Q6 = 0,
    parameter POINT_DECIMATION = 1,
    parameter FRAME_POINTS = 1024,
    parameter MATCH_STRIDE_BITS = 3,
    parameter MIN_MATCH_POINTS = 32,
    parameter MIN_MATCH_HITS = 16,
    parameter MIN_SCORE_IMPROVEMENT = 4,
    parameter MIN_SCORE_MARGIN = 3,
    parameter MIN_MATCH_SPREAD_CELLS = 8,
    parameter MIN_REFERENCE_SCANS = 3
) (
    input wire clk, reset_n, enable, clear_map,
    input wire [15:0] scan_id,
    input wire point_valid, scan_start,
    input wire [15:0] angle_q6, distance_mm,
    input wire [5:0] quality,
    // Local preview replaces its cells once per revolution. World acquisition
    // requires a fresh, coherent, sufficiently slow odometry pose per point.
    input wire pose_valid,local_scan,
    input wire signed [31:0] pose_x_mm, pose_y_mm,
    input wire [15:0] pose_heading_q6,
    input wire [14:0] forward_angle_q6,
    input wire signed [15:0] lidar_offset_x_mm,lidar_offset_y_mm,
    input wire [15:0] body_front_mm,body_rear_mm,body_left_mm,body_right_mm,
    input wire correction_enable,correction_ack,
    input wire freeze_map,
    output wire map_frozen,
    input wire plan_rd_en,
    input wire [MAP_BITS-1:0] plan_rd_x,plan_rd_y,
    output wire [1:0] plan_rd_cell,
    output reg plan_rd_valid,
    output reg map_ready, busy,
    output reg [31:0] point_count, dropped_points, out_of_bounds, scan_count,
    output reg [15:0] integrated_scan_id,
    output reg integrated_scan_valid,integrated_scan_pulse,
    output reg correction_valid,
    output reg signed [15:0] correction_x_mm, correction_y_mm,
    output reg [7:0] match_score, match_baseline, match_margin,
    input wire rd_clk, rd_en,
    input wire [MAP_BITS-1:0] rd_x, rd_y,
    output wire [1:0] rd_cell,
    output reg rd_valid
);
    localparam MAP_SIDE = (1 << MAP_BITS);
    localparam MAP_CELLS = (1 << (2*MAP_BITS));
    // Maturity is a saturating functional counter. Keep the wide diagnostic
    // scan_count out of matching decisions so unused statistics can prune.
    localparam REFERENCE_COUNT_BITS=MIN_REFERENCE_SCANS<1 ? 1 : $clog2(MIN_REFERENCE_SCANS+1);
    reg [REFERENCE_COUNT_BITS-1:0] reference_scans;
    wire reference_mature=reference_scans>=MIN_REFERENCE_SCANS;
    // FRAME_POINTS <=1024; MATCH_STRIDE_BITS>=3 bounds a score to <=128 hits.
    // One extra bit distinguishes a real return from a clipped free ray.
    localparam POINT_BITS = 4*MAP_BITS+1;
    localparam [3:0] F_IDLE=0,F_NORM=1,F_QUANTISE=2,F_TRIG_SETUP=3,
        F_TRIG_SIN_WAIT=4,F_TRIG_SIN=5,F_TRIG_COS=6,F_MUL_INIT=7,
        F_MUL=8,F_MUL_DONE=9,F_BEAM_WORLD=10,F_OFFSET_WORLD=11,
        F_GRID=12,F_STORE=13,F_CLIP_INIT=14,F_CLIP=15;
    localparam [4:0] S_CLEAR=0,S_IDLE=1,S_MPOINT=2,S_MPOINT_LATCH=3,
        S_MREQ=4,S_MUSE=5,S_CANDIDATE_DONE=6,S_DECIDE=8,
        S_IPOINT=9,S_IPOINT_LATCH=10,S_RAY_INIT=11,S_RAY_READ=12,
        S_RAY_WRITE=13,S_FINISH=14,S_CORRECTION_WAIT=15,
        S_SNAPSHOT_READ=16,S_SNAPSHOT_WRITE=17,S_LOCAL_CLEAR=18,S_BODY_SEED=19;
    reg [4:0] state;

    // No array reset: clear one location per clock, preserving block-RAM
    // inference. The live map has ONLY its 50 MHz read/write port. LCD reads
    // separate snapshots, never this array: Arora V also forbids cross-port
    // read/write collisions within a physical RAM row. GW5AT-60 does not support
    // read-before-write: on a write cycle hold the A-port read output. The
    // read/modify/write engine already samples the old cell in S_RAY_READ.
    (* syn_ramstyle = "block_ram" *) reg [1:0] cells [0:MAP_CELLS-1];
    reg [2*MAP_BITS-1:0] map_addr;
    reg [1:0] map_wdata, map_rdata;
    reg map_we;
    always @(posedge clk) begin
        if (map_we) cells[map_addr] <= map_wdata;
        else map_rdata <= cells[map_addr];
    end
    assign plan_rd_cell=map_rdata;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) plan_rd_valid<=0;
        else plan_rd_valid<=map_frozen && plan_rd_en && !clear_map;
    end
    // Snapshot ownership: initial reader owns bank 0, writer owns bank 1.
    // req toggle also identifies the COMPLETED bank. It is published only
    // after the final write; write_bank flips then, but cannot be written until
    // the pixel ACK has crossed three source clocks. No pending-ACK wait stalls
    // mapping: if req!=ACK, S_FINISH simply skips that display snapshot.
    reg snapshot_request,snapshot_write_bank;
    reg snapshot_epoch_valid,snapshot_display_valid;
    reg [2*MAP_BITS-1:0] snapshot_index;
    reg snapshot_ack,display_active_bank,display_read_bank;
    (* async_reg="true" *) reg [2:0] snapshot_ack_sync;
    (* async_reg="true" *) reg [2:0] snapshot_request_sync,display_valid_sync;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) snapshot_ack_sync<=0;
        else snapshot_ack_sync<={snapshot_ack_sync[1:0],snapshot_ack};
    end
    always @(posedge rd_clk or negedge reset_n) begin
        if(!reset_n) begin
            snapshot_request_sync<=0;display_valid_sync<=0;
            snapshot_ack<=0;display_active_bank<=0;display_read_bank<=0;rd_valid<=0;
        end else begin
            snapshot_request_sync<={snapshot_request_sync[1:0],snapshot_request};
            display_valid_sync<={display_valid_sync[1:0],snapshot_display_valid};
            // Switch only when no LCD RAM read is being issued. ACK then
            // crosses three 50 MHz clocks before the old bank can be reused.
            if(snapshot_request_sync[2]!=snapshot_ack && !rd_en) begin
                display_active_bank<=snapshot_request_sync[2];
                snapshot_ack<=snapshot_request_sync[2];
            end
            // DPB READ_MODE1=0 already supplies the synchronous one-clock read.
            // Register the bank selection, NOT the RAM output, to retain the
            // original one-clock rd_cell/rd_valid contract.
            display_read_bank<=display_active_bank;
            rd_valid<=rd_en && display_valid_sync[2];
        end
    end
    wire snapshot_copy_enable=reset_n && !clear_map && state==S_SNAPSHOT_WRITE;
    wire display_read_enable=reset_n && display_valid_sync[2] && rd_en;
    wire [1:0] snapshot_read_0,snapshot_read_1;
    lidar_map_snapshot_bank #(.ADDRESS_BITS(2*MAP_BITS)) u_snapshot_0 (
        .write_clk(clk),.read_clk(rd_clk),.reset_n(reset_n),
        .write_enable(snapshot_copy_enable && !snapshot_write_bank),
        .read_enable(display_read_enable && !display_active_bank),
        .write_address(snapshot_index),.read_address({rd_y,rd_x}),
        .write_data(map_rdata),.read_data(snapshot_read_0)
    );
    lidar_map_snapshot_bank #(.ADDRESS_BITS(2*MAP_BITS)) u_snapshot_1 (
        .write_clk(clk),.read_clk(rd_clk),.reset_n(reset_n),
        .write_enable(snapshot_copy_enable && snapshot_write_bank),
        .read_enable(display_read_enable && display_active_bank),
        .write_address(snapshot_index),.read_address({rd_y,rd_x}),
        .write_data(map_rdata),.read_data(snapshot_read_1)
    );
    assign rd_cell=display_read_bank ? snapshot_read_1 : snapshot_read_0;

    // Two scan banks, one being filled and the other consumed. A scan bank is
    // held untouched until the map engine completes; no multi-bit CDC here.
    (* syn_ramstyle = "block_ram" *) reg [POINT_BITS-1:0] rays [0:2047];
    reg [10:0] ray_waddr;
    reg [POINT_BITS-1:0] ray_wdata, ray_rdata;
    reg ray_we;
    reg [10:0] ray_raddr;
    always @(posedge clk) begin
        if (ray_we) rays[ray_waddr] <= ray_wdata;
        else ray_rdata <= rays[ray_raddr];
    end

    function signed [3:0] candidate_x;
        input [4:0] n;
        begin
            case(n)
                0,5,10,15,20:candidate_x=-4'sd2;
                1,6,11,16,21:candidate_x=-4'sd1;
                2,7,12,17,22:candidate_x=4'sd0;
                3,8,13,18,23:candidate_x=4'sd1;
                default:candidate_x=4'sd2;
            endcase
        end
    endfunction
    function signed [3:0] candidate_y;
        input [4:0] n;
        begin
            if(n<5) candidate_y=-4'sd2;
            else if(n<10) candidate_y=-4'sd1;
            else if(n<15) candidate_y=4'sd0;
            else if(n<20) candidate_y=4'sd1;
            else candidate_y=4'sd2;
        end
    endfunction

    reg [3:0] front_state;
    reg fill_bank, frame_ready, frame_bank;
    reg [15:0] fill_scan_id,frame_scan_id,work_scan_id;
    reg work_bank;
    reg [10:0] work_count,work_index;
    wire fill_bank_available=!busy || state==S_BODY_SEED || fill_bank!=work_bank;
    reg wait_scan_start;
    reg [10:0] fill_count, frame_count;
    reg [MAP_BITS-1:0] fill_min_x,fill_max_x,fill_min_y,fill_max_y;
    reg [MAP_BITS-1:0] frame_min_x,frame_max_x,frame_min_y,frame_max_y;
    reg [7:0] decimation_count;
    reg frame_take;
    // A single synchronous quarter-wave ROM replaces the variable-shift
    // CORDIC datapath. Round Q6 degrees to the NEAREST 0.25 degree, not down.
    // +/-0.125 degree contributes <=8.73 mm beam error at 4 m (34.91 at
    // 16 m), plus Q14/integer rounding. Mount-vector error is independent.
    // All products share one signed shift/add unit, 15 clocks per product;
    // the <=6 products require <140 clocks/point (+<=128 for edge clipping), versus >=10000 available
    // clocks between 5 ksample/s C1 points. No input decimation is added.
    reg [15:0] point_distance;
    reg signed [15:0] point_offset_x,point_offset_y;
    reg [15:0] point_heading;
    reg lookup_heading;
    reg signed [17:0] world_angle_q6;
    wire [14:0] rounded_angle_q6=world_angle_q6[14:0]+15'd8;
    reg [10:0] trig_index;
    reg [8:0] trig_address,trig_cos_address;
    reg trig_sin_negative,trig_cos_negative;
    wire [14:0] trig_value;
    reg signed [15:0] trig_sin,trig_cos;
    lidar_quarter_sine_rom u_coordinate_sine (
        .clk(clk),.address(trig_address),.data_q14(trig_value)
    );
    reg [14:0] mul_factor;
    reg signed [31:0] mul_value,mul_accum;
    reg [3:0] mul_bit;
    reg [2:0] mul_phase;
    reg signed [31:0] offset_term;
    reg signed [15:0] beam_delta_x,beam_delta_y,offset_delta_x,offset_delta_y;
    wire signed [15:0] selected_operand=(mul_phase<2) ? $signed(point_distance) :
        ((mul_phase==2 || mul_phase==4) ? point_offset_x : point_offset_y);
    wire signed [15:0] selected_coefficient=(mul_phase==1 || mul_phase==3 ||
        mul_phase==4) ? trig_sin : trig_cos;
    wire signed [31:0] selected_operand_wide={{16{selected_operand[15]}},selected_operand};
    wire [15:0] coefficient_magnitude=selected_coefficient[15] ?
        -selected_coefficient : selected_coefficient;
    wire signed [31:0] mul_sum=mul_accum+(mul_factor[0] ? mul_value : 32'sd0);
    wire signed [31:0] offset_combined=(mul_phase==3) ?
        offset_term-mul_accum : offset_term+mul_accum;
    reg signed [31:0] point_pose_x,point_pose_y,world_x,world_y;
    reg signed [31:0] sensor_world_x,sensor_world_y;
    reg signed [31:0] hit_cell_x,hit_cell_y,start_cell_x,start_cell_y;
    wire valid_range = distance_mm>=16'd50 && distance_mm<=16'd16000;
    wire [15:0] capture_heading_q6=local_scan ? 16'd0 : pose_heading_q6;
    wire valid_angles = angle_q6<16'd23040 && capture_heading_q6<16'd23040 &&
        forward_angle_q6<15'd23040;
    wire valid_offsets=lidar_offset_x_mm>=-16'sd1000 && lidar_offset_x_mm<=16'sd1000 &&
        lidar_offset_y_mm>=-16'sd1000 && lidar_offset_y_mm<=16'sd1000;
    wire front_bounds = hit_cell_x>=0 && hit_cell_x<MAP_SIDE &&
        hit_cell_y>=0 && hit_cell_y<MAP_SIDE &&
        start_cell_x>=0 && start_cell_x<MAP_SIDE &&
        start_cell_y>=0 && start_cell_y<MAP_SIDE;
    wire start_in_bounds=start_cell_x>=0 && start_cell_x<MAP_SIDE &&
        start_cell_y>=0 && start_cell_y<MAP_SIDE;
    reg point_hit;
    reg signed [MAP_BITS+1:0] clip_x,clip_y;
    reg [9:0] clip_dx,clip_dy;
    reg clip_positive_x,clip_positive_y;
    reg signed [10:0] clip_error;
    wire signed [11:0] clip_twice_error=$signed({clip_error[10],clip_error})<<<1;
    wire clip_step_x=clip_twice_error>-$signed({2'b00,clip_dy});
    wire clip_step_y=clip_twice_error<$signed({2'b00,clip_dx});
    wire signed [MAP_BITS+1:0] clip_next_x=clip_x+
        (clip_step_x ? (clip_positive_x ? 1 : -1) : 0);
    wire signed [MAP_BITS+1:0] clip_next_y=clip_y+
        (clip_step_y ? (clip_positive_y ? 1 : -1) : 0);
    wire clip_next_inside=clip_next_x>=0 && clip_next_x<MAP_SIDE &&
        clip_next_y>=0 && clip_next_y<MAP_SIDE;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            front_state<=F_IDLE;fill_bank<=0;frame_ready<=0;frame_bank<=0;
            fill_scan_id<=0;frame_scan_id<=0;
            wait_scan_start<=1;
            fill_count<=0;frame_count<=0;decimation_count<=0;
            fill_min_x<=0;fill_max_x<=0;fill_min_y<=0;fill_max_y<=0;
            frame_min_x<=0;frame_max_x<=0;frame_min_y<=0;frame_max_y<=0;
            point_distance<=0;point_offset_x<=0;point_offset_y<=0;point_heading<=0;
            lookup_heading<=0;world_angle_q6<=0;trig_index<=0;
            trig_address<=0;trig_cos_address<=0;
            trig_sin_negative<=0;trig_cos_negative<=0;trig_sin<=0;trig_cos<=0;
            mul_factor<=0;mul_value<=0;mul_accum<=0;mul_bit<=0;mul_phase<=0;
            offset_term<=0;beam_delta_x<=0;beam_delta_y<=0;
            offset_delta_x<=0;offset_delta_y<=0;
            point_pose_x<=0;point_pose_y<=0;world_x<=0;world_y<=0;
            sensor_world_x<=0;sensor_world_y<=0;
            hit_cell_x<=0;hit_cell_y<=0;start_cell_x<=0;start_cell_y<=0;
            point_hit<=0;clip_x<=0;clip_y<=0;clip_dx<=0;clip_dy<=0;
            clip_positive_x<=0;clip_positive_y<=0;clip_error<=0;
            ray_we<=0;ray_waddr<=0;ray_wdata<=0;
            point_count<=0;dropped_points<=0;out_of_bounds<=0;
        end else begin
            ray_we<=0;
            if(frame_take) frame_ready<=0;
            if(clear_map || !enable || !map_ready || freeze_map || (!local_scan && !pose_valid)) begin
                front_state<=F_IDLE;fill_count<=0;frame_ready<=0;
                decimation_count<=0;wait_scan_start<=1;
                if(clear_map) begin
                    fill_scan_id<=0;frame_scan_id<=0;
                    point_count<=0;dropped_points<=0;out_of_bounds<=0;
                end
            end else if(correction_valid && correction_ack) begin
                // The engine owns work_bank until integration finishes. Its
                // correction has just committed globally, so discard ONLY the
                // front-end bank/count and any unconsumed front-end mailbox.
                // Never clear/rewrite work_bank, work_count or its ray data.
                // Busy already prevents capture during processing, but this
                // explicit epoch barrier also protects future front-end edits.
                // A previous-cycle RAM write may commit at this edge; it is
                // in the discarded opposite bank, never in work_bank.
                front_state<=F_IDLE;fill_count<=0;frame_ready<=0;
                fill_bank<=!work_bank;
                fill_scan_id<=0;
                ray_we<=0;decimation_count<=0;wait_scan_start<=1;
                if(point_valid) dropped_points<=dropped_points+1'b1;
            end else begin
                // The boundary sample is deliberately omitted. All accepted
                // rays belong entirely to one completed scan and pose epoch.
                // C1 may flag a revolution boundary on a zero-distance node;
                // scan_start is intentionally independent of point_valid.
                if(scan_start) begin
                    decimation_count<=0;
                    if(wait_scan_start) begin
                        // Work in the OTHER bank does not erase the same
                        // angular wedge at the beginning of every revolution.
                        if(front_state==F_IDLE && fill_bank_available && !frame_ready) begin
                            wait_scan_start<=0;fill_count<=0;
                            fill_scan_id<=scan_id;
                        end
                    end else if(front_state==F_IDLE && fill_bank_available && !frame_ready && (fill_count!=0 || local_scan)) begin
                        frame_ready<=1;frame_bank<=fill_bank;frame_count<=fill_count;
                        // Seal the PREVIOUS accepted revolution, while this
                        // same boundary opens a new capture in the other bank.
                        frame_scan_id<=fill_scan_id;fill_scan_id<=scan_id;
                        frame_min_x<=fill_min_x;frame_max_x<=fill_max_x;
                        frame_min_y<=fill_min_y;frame_max_y<=fill_max_y;
                        fill_bank<=!fill_bank;fill_count<=0;
                    end else begin
                        // A genuinely overloaded/incomplete revolution is
                        // discarded; never concatenate two scan/pose epochs.
                        // A previously sealed frame_ready bank remains held.
                        front_state<=F_IDLE;fill_count<=0;wait_scan_start<=1;
                    end
                    if(point_valid) dropped_points<=dropped_points+1'b1;
                end else if(point_valid) begin
                    if(wait_scan_start || front_state!=F_IDLE || !fill_bank_available || frame_ready) begin
                        dropped_points<=dropped_points+1'b1;
                        if(!fill_bank_available || frame_ready)begin
                            fill_count<=0;wait_scan_start<=1;
                        end
                    end else if((local_scan || pose_valid) && quality!=0 && valid_range && valid_angles && valid_offsets) begin
                        if(decimation_count==POINT_DECIMATION-1) begin
                            decimation_count<=0;
                            if(fill_count<FRAME_POINTS) begin
                                point_pose_x<=local_scan ? 32'sd0 : pose_x_mm;
                                point_pose_y<=local_scan ? 32'sd0 : pose_y_mm;
                                point_offset_x<=lidar_offset_x_mm;point_offset_y<=lidar_offset_y_mm;
                                point_heading<=capture_heading_q6;
                                if(ANGLE_CLOCKWISE)
                                    world_angle_q6<=$signed({2'b00,capture_heading_q6})-
                                        $signed({2'b00,angle_q6})+
                                        $signed({3'b000,forward_angle_q6})+YAW_OFFSET_Q6;
                                else world_angle_q6<=$signed({2'b00,capture_heading_q6})+
                                        $signed({2'b00,angle_q6})-
                                        $signed({3'b000,forward_angle_q6})+YAW_OFFSET_Q6;
                                point_distance<=distance_mm;lookup_heading<=0;
                                front_state<=F_NORM;
                            end else dropped_points<=dropped_points+1'b1;
                        end else decimation_count<=decimation_count+1'b1;
                    end else dropped_points<=dropped_points+1'b1;
                end
                if(!scan_start)case(front_state)
                    F_IDLE:begin end
                    F_NORM:begin
                        if(world_angle_q6<0) world_angle_q6<=world_angle_q6+18'sd23040;
                        else if(world_angle_q6>=18'sd23040)
                            world_angle_q6<=world_angle_q6-18'sd23040;
                        else front_state<=F_QUANTISE;
                    end
                    F_QUANTISE:begin
                        // 359.875..359.984375 degrees round to 360, then wrap.
                        if(rounded_angle_q6[14:4]>=11'd1440) trig_index<=0;
                        else trig_index<=rounded_angle_q6[14:4];
                        front_state<=F_TRIG_SETUP;
                    end
                    F_TRIG_SETUP:begin
                        // Quarter-wave folding includes exact cardinal angles.
                        // Both ROM addresses stay in 0..360. Signed Q14 +1 and
                        // -1 are +16384/-16384, safely inside 16 signed bits.
                        if(trig_index<11'd360) begin
                            trig_address<=trig_index[8:0];
                            trig_cos_address<=11'd360-trig_index;
                            trig_sin_negative<=0;trig_cos_negative<=0;
                        end else if(trig_index<11'd720) begin
                            trig_address<=11'd720-trig_index;
                            trig_cos_address<=trig_index-11'd360;
                            trig_sin_negative<=0;trig_cos_negative<=1;
                        end else if(trig_index<11'd1080) begin
                            trig_address<=trig_index-11'd720;
                            trig_cos_address<=11'd1080-trig_index;
                            trig_sin_negative<=1;trig_cos_negative<=1;
                        end else begin
                            trig_address<=11'd1440-trig_index;
                            trig_cos_address<=trig_index-11'd1080;
                            trig_sin_negative<=1;trig_cos_negative<=0;
                        end
                        front_state<=F_TRIG_SIN_WAIT;
                    end
                    F_TRIG_SIN_WAIT:begin
                        // ROM sees the sine address at this edge. Address the
                        // cosine now, then consume sine/cosine on separate edges.
                        trig_address<=trig_cos_address;front_state<=F_TRIG_SIN;
                    end
                    F_TRIG_SIN:begin
                        if(trig_sin_negative) trig_sin<=-$signed({1'b0,trig_value});
                        else trig_sin<=$signed({1'b0,trig_value});
                        front_state<=F_TRIG_COS;
                    end
                    F_TRIG_COS:begin
                        if(trig_cos_negative) trig_cos<=-$signed({1'b0,trig_value});
                        else trig_cos<=$signed({1'b0,trig_value});
                        if(lookup_heading) mul_phase<=2;
                        else mul_phase<=0;
                        front_state<=F_MUL_INIT;
                    end
                    F_MUL_INIT:begin
                        // Keep coefficient magnitude nonnegative. Negate the
                        // sign-extended mm operand before shifting for negatives.
                        // This also handles the exact -16384 cardinal values.
                        if(selected_coefficient[15]) mul_value<=-selected_operand_wide;
                        else mul_value<=selected_operand_wide;
                        mul_factor<=coefficient_magnitude[14:0];
                        mul_accum<=0;mul_bit<=0;front_state<=F_MUL;
                    end
                    F_MUL:begin
                        mul_accum<=mul_sum;
                        mul_value<=mul_value<<<1;mul_factor<=mul_factor>>1;
                        if(mul_bit==14) front_state<=F_MUL_DONE;
                        else mul_bit<=mul_bit+1'b1;
                    end
                    F_MUL_DONE:begin
                        case(mul_phase)
                            0:begin
                                beam_delta_x<=mul_accum>>>14;
                                mul_phase<=1;front_state<=F_MUL_INIT;
                            end
                            1:begin
                                beam_delta_y<=mul_accum>>>14;
                                front_state<=F_BEAM_WORLD;
                            end
                            2:begin
                                offset_term<=mul_accum;
                                mul_phase<=3;front_state<=F_MUL_INIT;
                            end
                            3:begin
                                offset_delta_x<=offset_combined>>>14;
                                mul_phase<=4;front_state<=F_MUL_INIT;
                            end
                            4:begin
                                offset_term<=mul_accum;
                                mul_phase<=5;front_state<=F_MUL_INIT;
                            end
                            default:begin
                                offset_delta_y<=offset_combined>>>14;
                                front_state<=F_OFFSET_WORLD;
                            end
                        endcase
                    end
                    F_BEAM_WORLD:begin
                        world_x<=point_pose_x+{{16{beam_delta_x[15]}},beam_delta_x};
                        world_y<=point_pose_y+{{16{beam_delta_y[15]}},beam_delta_y};
                        sensor_world_x<=point_pose_x;sensor_world_y<=point_pose_y;
                        if(point_offset_x!=0 || point_offset_y!=0) begin
                            lookup_heading<=1;
                            world_angle_q6<=$signed({2'b00,point_heading});
                            front_state<=F_QUANTISE;
                        end else front_state<=F_GRID;
                    end
                    F_OFFSET_WORLD:begin
                        world_x<=world_x+{{16{offset_delta_x[15]}},offset_delta_x};
                        world_y<=world_y+{{16{offset_delta_y[15]}},offset_delta_y};
                        sensor_world_x<=point_pose_x+{{16{offset_delta_x[15]}},offset_delta_x};
                        sensor_world_y<=point_pose_y+{{16{offset_delta_y[15]}},offset_delta_y};
                        front_state<=F_GRID;
                    end
                    F_GRID:begin
                        hit_cell_x<=(world_x-ORIGIN_X_MM)>>>CELL_MM_BITS;
                        hit_cell_y<=(world_y-ORIGIN_Y_MM)>>>CELL_MM_BITS;
                        start_cell_x<=(sensor_world_x-ORIGIN_X_MM)>>>CELL_MM_BITS;
                        start_cell_y<=(sensor_world_y-ORIGIN_Y_MM)>>>CELL_MM_BITS;
                        front_state<=F_CLIP_INIT;
                    end
                    F_CLIP_INIT:begin
                        point_hit<=front_bounds;
                        if(front_bounds)front_state<=F_STORE;
                        else if(start_in_bounds)begin
                            // A distant wall still proves free space inside
                            // our window. Walk to the last in-window cell;
                            // never clamp x/y independently or invent a wall.
                            clip_x<=start_cell_x;clip_y<=start_cell_y;
                            clip_dx<=hit_cell_x>=start_cell_x ? hit_cell_x-start_cell_x : start_cell_x-hit_cell_x;
                            clip_dy<=hit_cell_y>=start_cell_y ? hit_cell_y-start_cell_y : start_cell_y-hit_cell_y;
                            clip_error<=(hit_cell_x>=start_cell_x ? hit_cell_x-start_cell_x : start_cell_x-hit_cell_x)-
                                (hit_cell_y>=start_cell_y ? hit_cell_y-start_cell_y : start_cell_y-hit_cell_y);
                            clip_positive_x<=hit_cell_x>=start_cell_x;clip_positive_y<=hit_cell_y>=start_cell_y;
                            out_of_bounds<=out_of_bounds+1'b1;front_state<=F_CLIP;
                        end else begin out_of_bounds<=out_of_bounds+1'b1;front_state<=F_IDLE;end
                    end
                    F_CLIP:begin
                        if(!clip_next_inside)begin
                            hit_cell_x<=clip_x;hit_cell_y<=clip_y;front_state<=F_STORE;
                        end else begin
                            clip_x<=clip_next_x;clip_y<=clip_next_y;
                            if(clip_step_x && clip_step_y)clip_error<=clip_error-$signed({1'b0,clip_dy})+$signed({1'b0,clip_dx});
                            else if(clip_step_x)clip_error<=clip_error-$signed({1'b0,clip_dy});
                            else if(clip_step_y)clip_error<=clip_error+$signed({1'b0,clip_dx});
                        end
                    end
                    F_STORE:begin
                        if(front_bounds && fill_bank_available) begin
                            ray_we<=1;ray_waddr<={fill_bank,fill_count[9:0]};
                            ray_wdata<={point_hit,start_cell_x[MAP_BITS-1:0],start_cell_y[MAP_BITS-1:0],
                                hit_cell_x[MAP_BITS-1:0],hit_cell_y[MAP_BITS-1:0]};
                            fill_count<=fill_count+1'b1;point_count<=point_count+1'b1;
                            if(fill_count==0) begin
                                fill_min_x<=hit_cell_x[MAP_BITS-1:0];fill_max_x<=hit_cell_x[MAP_BITS-1:0];
                                fill_min_y<=hit_cell_y[MAP_BITS-1:0];fill_max_y<=hit_cell_y[MAP_BITS-1:0];
                            end else begin
                                if(hit_cell_x<fill_min_x) fill_min_x<=hit_cell_x[MAP_BITS-1:0];
                                if(hit_cell_x>fill_max_x) fill_max_x<=hit_cell_x[MAP_BITS-1:0];
                                if(hit_cell_y<fill_min_y) fill_min_y<=hit_cell_y[MAP_BITS-1:0];
                                if(hit_cell_y>fill_max_y) fill_max_y<=hit_cell_y[MAP_BITS-1:0];
                            end
                        end else if(!front_bounds)out_of_bounds<=out_of_bounds+1'b1;
                        else begin
                            dropped_points<=dropped_points+1'b1;
                            fill_count<=0;wait_scan_start<=1;
                        end
                        front_state<=F_IDLE;
                    end
                    default:front_state<=F_IDLE;
                endcase
            end
        end
    end

    reg [2*MAP_BITS-1:0] clear_addr;
    reg body_seed_done,freeze_granted;
    wire body_seed_dimensions=body_front_mm>=2 && body_front_mm<=1000 &&
        body_rear_mm>=2 && body_rear_mm<=1000 && body_left_mm>=2 && body_left_mm<=1000 && body_right_mm>=2 && body_right_mm<=1000;
    // Only HERE IS ZERO's fresh world origin has an axis-aligned measured
    // footprint. Skip other headings/poses rather than invent an outward box.
    wire seed_origin_valid=!local_scan && pose_valid && pose_x_mm==0 && pose_y_mm==0 && pose_heading_q6==0 && body_seed_dimensions;
    wire signed [31:0] seed_x=ORIGIN_X_MM+$signed({1'b0,clear_addr[MAP_BITS-1:0],{CELL_MM_BITS{1'b0}}});
    wire signed [31:0] seed_y=ORIGIN_Y_MM+$signed({1'b0,clear_addr[2*MAP_BITS-1:MAP_BITS],{CELL_MM_BITS{1'b0}}});
    // Outward-rounded chassis dimensions are reduced by1mm. Seed only cells
    // whose WHOLE square is contained, never a clearance halo or occluded ray.
    wire seed_cell_inside=seed_x>=-($signed({16'd0,body_rear_mm})-32'sd1) &&
        seed_x+(1<<CELL_MM_BITS)<=$signed({16'd0,body_front_mm})-32'sd1 &&
        seed_y>=-($signed({16'd0,body_right_mm})-32'sd1) &&
        seed_y+(1<<CELL_MM_BITS)<=$signed({16'd0,body_left_mm})-32'sd1;
    reg [MAP_BITS-1:0] work_min_x,work_max_x,work_min_y,work_max_y;
    reg [MAP_BITS-1:0] sx,sy,hx,hy;
    reg [4:0] candidate,best_candidate;
    // Candidate-outer traversal keeps one running score, rather than a 25-word
    // dynamically addressed score/mask array. Extra ray-RAM reads cost little
    // at the scan rate and remove the per-point scatter mux/decoder network.
    reg [7:0] candidate_score,baseline_score;
    reg [3:0] candidate_quadrants,best_quadrants;
    reg [7:0] best_score,second_score;
    wire [10:0] match_next_index=work_index+(1<<MATCH_STRIDE_BITS);
    wire [10:0] match_point_count=(work_count+((1<<MATCH_STRIDE_BITS)-1))>>MATCH_STRIDE_BITS;
    reg signed [3:0] offset_x,offset_y;
    wire signed [MAP_BITS+1:0] candidate_cell_x=$signed({1'b0,hx})+candidate_x(candidate);
    wire signed [MAP_BITS+1:0] candidate_cell_y=$signed({1'b0,hy})+candidate_y(candidate);
    wire candidate_in_bounds=candidate_cell_x>=0 && candidate_cell_x<MAP_SIDE &&
        candidate_cell_y>=0 && candidate_cell_y<MAP_SIDE;
    wire signed [MAP_BITS+1:0] shifted_sx=$signed({1'b0,sx})+offset_x;
    wire signed [MAP_BITS+1:0] shifted_sy=$signed({1'b0,sy})+offset_y;
    wire signed [MAP_BITS+1:0] shifted_hx=$signed({1'b0,hx})+offset_x;
    wire signed [MAP_BITS+1:0] shifted_hy=$signed({1'b0,hy})+offset_y;
    wire shifted_in_bounds=shifted_sx>=0 && shifted_sx<MAP_SIDE &&
        shifted_sy>=0 && shifted_sy<MAP_SIDE && shifted_hx>=0 && shifted_hx<MAP_SIDE &&
        shifted_hy>=0 && shifted_hy<MAP_SIDE;
    reg [MAP_BITS-1:0] ray_x,ray_y,target_x,target_y;
    reg ray_hit;
    reg [MAP_BITS:0] ray_dx,ray_dy;
    reg ray_sx_positive,ray_sy_positive;
    reg signed [MAP_BITS+2:0] ray_error;
    wire signed [MAP_BITS+3:0] twice_error=$signed(ray_error)<<<1;
    wire ray_step_x=twice_error>-$signed({1'b0,ray_dy});
    wire ray_step_y=twice_error<$signed({1'b0,ray_dx});
    wire ray_endpoint=(ray_x==target_x && ray_y==target_y);
    wire [MAP_BITS:0] initial_dx=shifted_hx>=shifted_sx ? shifted_hx-shifted_sx : shifted_sx-shifted_hx;
    wire [MAP_BITS:0] initial_dy=shifted_hy>=shifted_sy ? shifted_hy-shifted_sy : shifted_sy-shifted_hy;
    // Once the planner has received ownership, a later pose-valid transition
    // must not revoke its RAM access or start a body seed under its reads.
    assign map_frozen=map_ready && freeze_map && state==S_IDLE && !clear_map &&
        (freeze_granted || body_seed_done || !seed_origin_valid);
    // Hits become stable after two observations. Free rays demote a stale hit
    // gradually (11 -> 10 -> 01), allowing a moved box to disappear later.
    always @* begin
        map_addr=0;map_we=0;map_wdata=0;
        ray_raddr={work_bank,work_index[9:0]};
        if(state==S_CLEAR || state==S_LOCAL_CLEAR) begin map_addr=clear_addr;map_we=1;end
        else if(state==S_BODY_SEED)begin
            map_addr=clear_addr;map_we=seed_origin_valid && seed_cell_inside;map_wdata=2'b01;
        end
        else if(state==S_SNAPSHOT_READ || state==S_SNAPSHOT_WRITE) map_addr=snapshot_index;
        else if(map_frozen) map_addr={plan_rd_y,plan_rd_x};
        else if(state==S_MREQ || state==S_MUSE)
            map_addr={candidate_cell_y[MAP_BITS-1:0],candidate_cell_x[MAP_BITS-1:0]};
        else if(state==S_RAY_READ || state==S_RAY_WRITE) begin
            map_addr={ray_y,ray_x};
            if(state==S_RAY_WRITE) begin
                map_we=1;
                if(ray_endpoint && ray_hit) map_wdata=map_rdata[1] ? 2'b11 : 2'b10;
                else if(map_rdata==2'b11) map_wdata=2'b10;
                else map_wdata=2'b01;
            end
        end
    end
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            state<=S_CLEAR;clear_addr<=0;map_ready<=0;busy<=1;frame_take<=0;
            body_seed_done<=0;freeze_granted<=0;
            work_scan_id<=0;integrated_scan_id<=0;integrated_scan_valid<=0;integrated_scan_pulse<=0;reference_scans<=0;
            work_bank<=0;work_count<=0;work_index<=0;
            work_min_x<=0;work_max_x<=0;work_min_y<=0;work_max_y<=0;
            sx<=0;sy<=0;hx<=0;hy<=0;candidate<=0;best_candidate<=12;
            best_score<=0;second_score<=0;offset_x<=0;offset_y<=0;
            candidate_score<=0;baseline_score<=0;candidate_quadrants<=0;best_quadrants<=0;
            ray_x<=0;ray_y<=0;target_x<=0;target_y<=0;ray_dx<=0;ray_dy<=0;ray_hit<=0;
            ray_sx_positive<=0;ray_sy_positive<=0;ray_error<=0;
            correction_valid<=0;correction_x_mm<=0;correction_y_mm<=0;
            match_score<=0;match_baseline<=0;match_margin<=0;scan_count<=0;
            snapshot_request<=0;snapshot_write_bank<=1;snapshot_index<=0;
            snapshot_epoch_valid<=0;snapshot_display_valid<=0;
        end else begin
            frame_take<=0;
            integrated_scan_pulse<=0;
            if(!enable)integrated_scan_valid<=0;
            if(!freeze_map || clear_map)freeze_granted<=0;
            else if(map_frozen)freeze_granted<=1;
            // A newly cleared epoch becomes visible only AFTER its published
            // bank has actually been selected and ACKed by the LCD. In
            // particular, never show the initial/uninitialised reader bank.
            if(snapshot_request==snapshot_ack_sync[2] && snapshot_epoch_valid)
                snapshot_display_valid<=1;
            if(clear_map) begin
                state<=S_CLEAR;clear_addr<=0;map_ready<=0;busy<=1;
                body_seed_done<=0;
                work_scan_id<=0;integrated_scan_id<=0;integrated_scan_valid<=0;integrated_scan_pulse<=0;reference_scans<=0;
                correction_valid<=0;
                scan_count<=0;match_score<=0;match_baseline<=0;match_margin<=0;
                // Abort an unfinished copy and hide the previous world frame.
                // Ownership request/ACK/bank selectors MUST survive clear_map.
                snapshot_epoch_valid<=0;snapshot_display_valid<=0;snapshot_index<=0;
            end else case(state)
                S_CLEAR:begin
                    if(clear_addr==MAP_CELLS-1) begin
                        map_ready<=1;busy<=0;state<=S_IDLE;
                    end else clear_addr<=clear_addr+1'b1;
                end
                S_IDLE:begin
                    busy<=0;
                    if(enable && !freeze_granted && !body_seed_done && seed_origin_valid)begin
                        // Before granting planner freeze, establish only the
                        // physically occupied body interior at this origin.
                        clear_addr<=0;busy<=1;state<=S_BODY_SEED;
                    end else if(enable && frame_ready && !freeze_map) begin
                        busy<=1;frame_take<=1;work_bank<=frame_bank;work_count<=frame_count;
                        work_scan_id<=frame_scan_id;
                        work_min_x<=frame_min_x;work_max_x<=frame_max_x;
                        work_min_y<=frame_min_y;work_max_y<=frame_max_y;
                        work_index<=0;offset_x<=0;offset_y<=0;
                        candidate<=0;candidate_score<=0;candidate_quadrants<=0;
                        baseline_score<=0;best_score<=0;second_score<=0;
                        best_candidate<=12;best_quadrants<=0;
                        // Reuse the same single-port map and serial clear. A
                        // live body-frame preview must never accumulate scans
                        // from different vehicle positions as if they shared
                        // one world origin. No matching against a local frame.
                        if(local_scan)begin clear_addr<=0;state<=S_LOCAL_CLEAR;end
                        else state<=S_MPOINT;
                    end
                end
                S_BODY_SEED:begin
                    if(!seed_origin_valid)begin busy<=0;state<=S_IDLE;end
                    else if(clear_addr==MAP_CELLS-1)begin body_seed_done<=1;busy<=0;state<=S_IDLE;end
                    else clear_addr<=clear_addr+1'b1;
                end
                S_LOCAL_CLEAR:begin
                    if(clear_addr==MAP_CELLS-1)begin
                        work_index<=0;state<=work_count==0 ? S_FINISH : S_IPOINT;
                    end
                    else clear_addr<=clear_addr+1'b1;
                end
                // A capture write holds ray_rdata. Do not advance the issue
                // state until this edge actually performs the required read.
                S_MPOINT:if(!ray_we)state<=S_MPOINT_LATCH;
                S_MPOINT_LATCH:begin
                    {ray_hit,sx,sy,hx,hy}<=ray_rdata;state<=S_MREQ;
                end
                S_MREQ:state<=S_MUSE;
                S_MUSE:begin
                    if(ray_hit && candidate_in_bounds && map_rdata==2'b11) begin
                        candidate_score<=candidate_score+1'b1;
                        candidate_quadrants<=candidate_quadrants |
                            (4'b0001<<{hy<sy,hx<sx});
                    end
                    if(match_next_index>=work_count) state<=S_CANDIDATE_DONE;
                    else begin work_index<=match_next_index;state<=S_MPOINT;end
                end
                S_CANDIDATE_DONE:begin
                    // The last MUSE update has committed before this state.
                    // Strict > matches the original ascending candidate rank:
                    // ties enter second_score and force a zero peak margin.
                    if(candidate==12) baseline_score<=candidate_score;
                    if(candidate_score>best_score) begin
                        second_score<=best_score;best_score<=candidate_score;
                        best_candidate<=candidate;best_quadrants<=candidate_quadrants;
                    end else if(candidate_score>second_score) second_score<=candidate_score;
                    if(candidate==24) state<=S_DECIDE;
                    else begin
                        candidate<=candidate+1'b1;work_index<=0;
                        candidate_score<=0;candidate_quadrants<=0;state<=S_MPOINT;
                    end
                end
                S_DECIDE:begin
                    match_score<=best_score;match_baseline<=baseline_score;
                    match_margin<=best_score-second_score;
                    if(pose_valid && correction_enable && reference_mature &&
                       match_point_count>=MIN_MATCH_POINTS && best_candidate!=12 &&
                       work_max_x-work_min_x>=MIN_MATCH_SPREAD_CELLS &&
                       work_max_y-work_min_y>=MIN_MATCH_SPREAD_CELLS &&
                       best_quadrants==4'b1111 &&
                       best_score>=MIN_MATCH_HITS &&
                       {1'b0,best_score}>={1'b0,baseline_score}+MIN_SCORE_IMPROVEMENT &&
                       {1'b0,best_score}>={1'b0,second_score}+MIN_SCORE_MARGIN) begin
                        correction_x_mm<=$signed(candidate_x(best_candidate))*(1<<CELL_MM_BITS);
                        correction_y_mm<=$signed(candidate_y(best_candidate))*(1<<CELL_MM_BITS);
                        correction_valid<=1;
                        state<=S_CORRECTION_WAIT;
                    end else begin work_index<=0;state<=S_IPOINT;end
                end
                S_CORRECTION_WAIT:begin
                    // The odometer must apply this translation exactly once,
                    // then ACK. Only then may the map use the same translation.
                    if(correction_valid && correction_ack) begin
                        offset_x<=candidate_x(best_candidate);offset_y<=candidate_y(best_candidate);
                        correction_valid<=0;work_index<=0;state<=S_IPOINT;
                    end else if(!correction_enable || !pose_valid) begin
                        // Both modules use the same permission condition. If
                        // it falls before an ACK, the odometer cannot accept
                        // this request, so safely continue without correction.
                        // ACK has priority: a preceding committed correction
                        // must never be discarded when permission changes.
                        correction_valid<=0;offset_x<=0;offset_y<=0;
                        work_index<=0;state<=S_IPOINT;
                    end
                end
                S_IPOINT:if(!ray_we)state<=S_IPOINT_LATCH;
                S_IPOINT_LATCH:begin {ray_hit,sx,sy,hx,hy}<=ray_rdata;state<=S_RAY_INIT;end
                S_RAY_INIT:begin
                    if(shifted_in_bounds) begin
                        ray_x<=shifted_sx[MAP_BITS-1:0];ray_y<=shifted_sy[MAP_BITS-1:0];
                        target_x<=shifted_hx[MAP_BITS-1:0];target_y<=shifted_hy[MAP_BITS-1:0];
                        ray_dx<=initial_dx;ray_dy<=initial_dy;
                        ray_sx_positive<=shifted_hx>=shifted_sx;
                        ray_sy_positive<=shifted_hy>=shifted_sy;
                        ray_error<=$signed({1'b0,initial_dx})-$signed({1'b0,initial_dy});
                        state<=S_RAY_READ;
                    end else if(work_index==work_count-1'b1) state<=S_FINISH;
                    else begin work_index<=work_index+1'b1;state<=S_IPOINT;end
                end
                S_RAY_READ:state<=S_RAY_WRITE;
                S_RAY_WRITE:begin
                    if(ray_endpoint) begin
                        if(work_index==work_count-1'b1) state<=S_FINISH;
                        else begin work_index<=work_index+1'b1;state<=S_IPOINT;end
                    end else begin
                        if(ray_step_x) ray_x<=ray_sx_positive ? ray_x+1'b1 : ray_x-1'b1;
                        if(ray_step_y) ray_y<=ray_sy_positive ? ray_y+1'b1 : ray_y-1'b1;
                        if(ray_step_x && ray_step_y)
                            ray_error<=ray_error-$signed({1'b0,ray_dy})+$signed({1'b0,ray_dx});
                        else if(ray_step_x) ray_error<=ray_error-$signed({1'b0,ray_dy});
                        else if(ray_step_y) ray_error<=ray_error+$signed({1'b0,ray_dx});
                        state<=S_RAY_READ;
                    end
                end
                S_FINISH:begin
                    scan_count<=scan_count+1'b1;
                    if(!reference_mature)reference_scans<=reference_scans+1'b1;
                    // Publish identity ONLY after the last ray write. A queued
                    // older scan or partially captured revolution cannot ACK a
                    // mission's fresh stationary scan barrier.
                    integrated_scan_id<=work_scan_id;integrated_scan_valid<=enable;integrated_scan_pulse<=enable;
                    if(snapshot_request==snapshot_ack_sync[2]) begin
                        snapshot_index<=0;state<=S_SNAPSHOT_READ;
                    end else begin busy<=0;state<=S_IDLE;end
                end
                S_SNAPSHOT_READ:state<=S_SNAPSHOT_WRITE;
                S_SNAPSHOT_WRITE:begin
                    if(snapshot_index==MAP_CELLS-1) begin
                        snapshot_request<=!snapshot_request;
                        snapshot_write_bank<=!snapshot_write_bank;
                        snapshot_epoch_valid<=1;busy<=0;state<=S_IDLE;
                    end else begin
                        snapshot_index<=snapshot_index+1'b1;state<=S_SNAPSHOT_READ;
                    end
                end
                default:begin
                    state<=S_CLEAR;clear_addr<=0;map_ready<=0;busy<=1;
                    body_seed_done<=0;
                    snapshot_epoch_valid<=0;snapshot_display_valid<=0;
                end
            endcase
        end
    end
endmodule

// Two occupancy bitplanes per snapshot bank, one explicit DPB per plane.
// 16K x1 uses all 14 address bits; MAP_BITS<=7 is supported by zero padding.
// Actual CEA/CEB are gated by exclusive ownership, not merely OCE or an output
// mux. This obeys UG300's same-physical-row read/write prohibition even when
// CLKA and CLKB are asynchronous. WRITE_MODE=00 is supported by Arora V.
module lidar_map_snapshot_bank #(
    parameter ADDRESS_BITS=14
) (
    input wire write_clk,read_clk,reset_n,write_enable,read_enable,
    input wire [ADDRESS_BITS-1:0] write_address,read_address,
    input wire [1:0] write_data,
    output wire [1:0] read_data
);
    wire [13:0] physical_write_address={{(14-ADDRESS_BITS){1'b0}},write_address};
    wire [13:0] physical_read_address={{(14-ADDRESS_BITS){1'b0}},read_address};
    genvar plane;
    generate for(plane=0;plane<2;plane=plane+1) begin: g_plane
        wire [15:0] read_word;
        DPB #(
            .BIT_WIDTH_0(1),.BIT_WIDTH_1(1),
            .WRITE_MODE0(2'b00),.WRITE_MODE1(2'b00),
            .READ_MODE0(1'b0),.READ_MODE1(1'b0),
            .BLK_SEL_0(3'b000),.BLK_SEL_1(3'b000),.RESET_MODE("SYNC")
        ) u_ram (
            .DOA(),.DOB(read_word),.DIA({15'd0,write_data[plane]}),.DIB(16'd0),
            .BLKSELA(3'b000),.BLKSELB(3'b000),
            .ADA(physical_write_address),.ADB(physical_read_address),
            .WREA(1'b1),.WREB(1'b0),.CLKA(write_clk),.CLKB(read_clk),
            .CEA(reset_n && write_enable),.CEB(reset_n && read_enable),
            .OCEA(1'b1),.OCEB(1'b1),.RESETA(1'b0),.RESETB(1'b0)
        );
        assign read_data[plane]=read_word[0];
    end endgenerate
endmodule

// Synchronous read-only coefficient table. 512 x 15 bits fits one block RAM;
// only the first 361 values are addressed (sin(0..90 deg), quarter-degree).
// The table is generated from round(sin(i*pi/720)*16384). No real-valued HDL
// arithmetic, external .mem dependency or resettable memory is used.
module lidar_quarter_sine_rom (
    input wire clk,
    input wire [8:0] address,
    output reg [14:0] data_q14
);
    (* syn_romstyle="block_rom" *) reg [14:0] sine [0:511];
    integer init_index;
    initial begin
        for(init_index=0;init_index<512;init_index=init_index+1) sine[init_index]=15'd0;
        sine[0]=15'd0;
        sine[1]=15'd71;
        sine[2]=15'd143;
        sine[3]=15'd214;
        sine[4]=15'd286;
        sine[5]=15'd357;
        sine[6]=15'd429;
        sine[7]=15'd500;
        sine[8]=15'd572;
        sine[9]=15'd643;
        sine[10]=15'd715;
        sine[11]=15'd786;
        sine[12]=15'd857;
        sine[13]=15'd929;
        sine[14]=15'd1000;
        sine[15]=15'd1072;
        sine[16]=15'd1143;
        sine[17]=15'd1214;
        sine[18]=15'd1285;
        sine[19]=15'd1357;
        sine[20]=15'd1428;
        sine[21]=15'd1499;
        sine[22]=15'd1570;
        sine[23]=15'd1641;
        sine[24]=15'd1713;
        sine[25]=15'd1784;
        sine[26]=15'd1855;
        sine[27]=15'd1926;
        sine[28]=15'd1997;
        sine[29]=15'd2068;
        sine[30]=15'd2139;
        sine[31]=15'd2209;
        sine[32]=15'd2280;
        sine[33]=15'd2351;
        sine[34]=15'd2422;
        sine[35]=15'd2492;
        sine[36]=15'd2563;
        sine[37]=15'd2634;
        sine[38]=15'd2704;
        sine[39]=15'd2775;
        sine[40]=15'd2845;
        sine[41]=15'd2915;
        sine[42]=15'd2986;
        sine[43]=15'd3056;
        sine[44]=15'd3126;
        sine[45]=15'd3196;
        sine[46]=15'd3266;
        sine[47]=15'd3336;
        sine[48]=15'd3406;
        sine[49]=15'd3476;
        sine[50]=15'd3546;
        sine[51]=15'd3616;
        sine[52]=15'd3686;
        sine[53]=15'd3755;
        sine[54]=15'd3825;
        sine[55]=15'd3894;
        sine[56]=15'd3964;
        sine[57]=15'd4033;
        sine[58]=15'd4102;
        sine[59]=15'd4171;
        sine[60]=15'd4240;
        sine[61]=15'd4310;
        sine[62]=15'd4378;
        sine[63]=15'd4447;
        sine[64]=15'd4516;
        sine[65]=15'd4585;
        sine[66]=15'd4653;
        sine[67]=15'd4722;
        sine[68]=15'd4790;
        sine[69]=15'd4859;
        sine[70]=15'd4927;
        sine[71]=15'd4995;
        sine[72]=15'd5063;
        sine[73]=15'd5131;
        sine[74]=15'd5199;
        sine[75]=15'd5266;
        sine[76]=15'd5334;
        sine[77]=15'd5402;
        sine[78]=15'd5469;
        sine[79]=15'd5536;
        sine[80]=15'd5604;
        sine[81]=15'd5671;
        sine[82]=15'd5738;
        sine[83]=15'd5805;
        sine[84]=15'd5872;
        sine[85]=15'd5938;
        sine[86]=15'd6005;
        sine[87]=15'd6071;
        sine[88]=15'd6138;
        sine[89]=15'd6204;
        sine[90]=15'd6270;
        sine[91]=15'd6336;
        sine[92]=15'd6402;
        sine[93]=15'd6467;
        sine[94]=15'd6533;
        sine[95]=15'd6599;
        sine[96]=15'd6664;
        sine[97]=15'd6729;
        sine[98]=15'd6794;
        sine[99]=15'd6859;
        sine[100]=15'd6924;
        sine[101]=15'd6989;
        sine[102]=15'd7053;
        sine[103]=15'd7118;
        sine[104]=15'd7182;
        sine[105]=15'd7246;
        sine[106]=15'd7311;
        sine[107]=15'd7374;
        sine[108]=15'd7438;
        sine[109]=15'd7502;
        sine[110]=15'd7565;
        sine[111]=15'd7629;
        sine[112]=15'd7692;
        sine[113]=15'd7755;
        sine[114]=15'd7818;
        sine[115]=15'd7881;
        sine[116]=15'd7943;
        sine[117]=15'd8006;
        sine[118]=15'd8068;
        sine[119]=15'd8130;
        sine[120]=15'd8192;
        sine[121]=15'd8254;
        sine[122]=15'd8316;
        sine[123]=15'd8377;
        sine[124]=15'd8438;
        sine[125]=15'd8500;
        sine[126]=15'd8561;
        sine[127]=15'd8621;
        sine[128]=15'd8682;
        sine[129]=15'd8743;
        sine[130]=15'd8803;
        sine[131]=15'd8863;
        sine[132]=15'd8923;
        sine[133]=15'd8983;
        sine[134]=15'd9043;
        sine[135]=15'd9102;
        sine[136]=15'd9162;
        sine[137]=15'd9221;
        sine[138]=15'd9280;
        sine[139]=15'd9339;
        sine[140]=15'd9397;
        sine[141]=15'd9456;
        sine[142]=15'd9514;
        sine[143]=15'd9572;
        sine[144]=15'd9630;
        sine[145]=15'd9688;
        sine[146]=15'd9746;
        sine[147]=15'd9803;
        sine[148]=15'd9860;
        sine[149]=15'd9917;
        sine[150]=15'd9974;
        sine[151]=15'd10031;
        sine[152]=15'd10087;
        sine[153]=15'd10143;
        sine[154]=15'd10199;
        sine[155]=15'd10255;
        sine[156]=15'd10311;
        sine[157]=15'd10366;
        sine[158]=15'd10422;
        sine[159]=15'd10477;
        sine[160]=15'd10531;
        sine[161]=15'd10586;
        sine[162]=15'd10641;
        sine[163]=15'd10695;
        sine[164]=15'd10749;
        sine[165]=15'd10803;
        sine[166]=15'd10856;
        sine[167]=15'd10910;
        sine[168]=15'd10963;
        sine[169]=15'd11016;
        sine[170]=15'd11069;
        sine[171]=15'd11121;
        sine[172]=15'd11174;
        sine[173]=15'd11226;
        sine[174]=15'd11278;
        sine[175]=15'd11330;
        sine[176]=15'd11381;
        sine[177]=15'd11433;
        sine[178]=15'd11484;
        sine[179]=15'd11535;
        sine[180]=15'd11585;
        sine[181]=15'd11636;
        sine[182]=15'd11686;
        sine[183]=15'd11736;
        sine[184]=15'd11786;
        sine[185]=15'd11835;
        sine[186]=15'd11885;
        sine[187]=15'd11934;
        sine[188]=15'd11982;
        sine[189]=15'd12031;
        sine[190]=15'd12080;
        sine[191]=15'd12128;
        sine[192]=15'd12176;
        sine[193]=15'd12223;
        sine[194]=15'd12271;
        sine[195]=15'd12318;
        sine[196]=15'd12365;
        sine[197]=15'd12412;
        sine[198]=15'd12458;
        sine[199]=15'd12505;
        sine[200]=15'd12551;
        sine[201]=15'd12597;
        sine[202]=15'd12642;
        sine[203]=15'd12688;
        sine[204]=15'd12733;
        sine[205]=15'd12778;
        sine[206]=15'd12822;
        sine[207]=15'd12867;
        sine[208]=15'd12911;
        sine[209]=15'd12955;
        sine[210]=15'd12998;
        sine[211]=15'd13042;
        sine[212]=15'd13085;
        sine[213]=15'd13128;
        sine[214]=15'd13170;
        sine[215]=15'd13213;
        sine[216]=15'd13255;
        sine[217]=15'd13297;
        sine[218]=15'd13338;
        sine[219]=15'd13380;
        sine[220]=15'd13421;
        sine[221]=15'd13462;
        sine[222]=15'd13502;
        sine[223]=15'd13543;
        sine[224]=15'd13583;
        sine[225]=15'd13623;
        sine[226]=15'd13662;
        sine[227]=15'd13702;
        sine[228]=15'd13741;
        sine[229]=15'd13780;
        sine[230]=15'd13818;
        sine[231]=15'd13856;
        sine[232]=15'd13894;
        sine[233]=15'd13932;
        sine[234]=15'd13970;
        sine[235]=15'd14007;
        sine[236]=15'd14044;
        sine[237]=15'd14081;
        sine[238]=15'd14117;
        sine[239]=15'd14153;
        sine[240]=15'd14189;
        sine[241]=15'd14225;
        sine[242]=15'd14260;
        sine[243]=15'd14295;
        sine[244]=15'd14330;
        sine[245]=15'd14364;
        sine[246]=15'd14399;
        sine[247]=15'd14433;
        sine[248]=15'd14466;
        sine[249]=15'd14500;
        sine[250]=15'd14533;
        sine[251]=15'd14566;
        sine[252]=15'd14598;
        sine[253]=15'd14631;
        sine[254]=15'd14663;
        sine[255]=15'd14694;
        sine[256]=15'd14726;
        sine[257]=15'd14757;
        sine[258]=15'd14788;
        sine[259]=15'd14819;
        sine[260]=15'd14849;
        sine[261]=15'd14879;
        sine[262]=15'd14909;
        sine[263]=15'd14938;
        sine[264]=15'd14968;
        sine[265]=15'd14996;
        sine[266]=15'd15025;
        sine[267]=15'd15053;
        sine[268]=15'd15082;
        sine[269]=15'd15109;
        sine[270]=15'd15137;
        sine[271]=15'd15164;
        sine[272]=15'd15191;
        sine[273]=15'd15218;
        sine[274]=15'd15244;
        sine[275]=15'd15270;
        sine[276]=15'd15296;
        sine[277]=15'd15321;
        sine[278]=15'd15346;
        sine[279]=15'd15371;
        sine[280]=15'd15396;
        sine[281]=15'd15420;
        sine[282]=15'd15444;
        sine[283]=15'd15468;
        sine[284]=15'd15491;
        sine[285]=15'd15515;
        sine[286]=15'd15537;
        sine[287]=15'd15560;
        sine[288]=15'd15582;
        sine[289]=15'd15604;
        sine[290]=15'd15626;
        sine[291]=15'd15647;
        sine[292]=15'd15668;
        sine[293]=15'd15689;
        sine[294]=15'd15709;
        sine[295]=15'd15729;
        sine[296]=15'd15749;
        sine[297]=15'd15769;
        sine[298]=15'd15788;
        sine[299]=15'd15807;
        sine[300]=15'd15826;
        sine[301]=15'd15844;
        sine[302]=15'd15862;
        sine[303]=15'd15880;
        sine[304]=15'd15897;
        sine[305]=15'd15914;
        sine[306]=15'd15931;
        sine[307]=15'd15948;
        sine[308]=15'd15964;
        sine[309]=15'd15980;
        sine[310]=15'd15996;
        sine[311]=15'd16011;
        sine[312]=15'd16026;
        sine[313]=15'd16041;
        sine[314]=15'd16055;
        sine[315]=15'd16069;
        sine[316]=15'd16083;
        sine[317]=15'd16096;
        sine[318]=15'd16110;
        sine[319]=15'd16123;
        sine[320]=15'd16135;
        sine[321]=15'd16147;
        sine[322]=15'd16159;
        sine[323]=15'd16171;
        sine[324]=15'd16182;
        sine[325]=15'd16193;
        sine[326]=15'd16204;
        sine[327]=15'd16214;
        sine[328]=15'd16225;
        sine[329]=15'd16234;
        sine[330]=15'd16244;
        sine[331]=15'd16253;
        sine[332]=15'd16262;
        sine[333]=15'd16270;
        sine[334]=15'd16279;
        sine[335]=15'd16287;
        sine[336]=15'd16294;
        sine[337]=15'd16302;
        sine[338]=15'd16309;
        sine[339]=15'd16315;
        sine[340]=15'd16322;
        sine[341]=15'd16328;
        sine[342]=15'd16333;
        sine[343]=15'd16339;
        sine[344]=15'd16344;
        sine[345]=15'd16349;
        sine[346]=15'd16353;
        sine[347]=15'd16358;
        sine[348]=15'd16362;
        sine[349]=15'd16365;
        sine[350]=15'd16368;
        sine[351]=15'd16371;
        sine[352]=15'd16374;
        sine[353]=15'd16376;
        sine[354]=15'd16378;
        sine[355]=15'd16380;
        sine[356]=15'd16382;
        sine[357]=15'd16383;
        sine[358]=15'd16383;
        sine[359]=15'd16384;
        sine[360]=15'd16384;
    end
    always @(posedge clk) data_q14<=sine[address];
endmodule
