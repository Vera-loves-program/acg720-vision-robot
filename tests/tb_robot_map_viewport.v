`timescale 1ns/1ps
// Dynamic test of the GENERATED panel RTL and its actual RAM address/color
// contract. The RAM model has the map snapshot port's one-clock latency.
// The bank owner model exercises !rd_en handoff; it is not a CDC proof.
module tb_robot_map_viewport;
    reg clk=0;always #5 clk=~clk;
    reg [10:0] x=0;
    reg [9:0] y=0;
    reg [31:0] flags=0;
    reg [39:0] car=0;
    reg [29:0] overlay=0;
    reg follow_car=0;
    reg [1:0] ram_cell=0;
    reg read_valid=0;
    wire read_en;
    wire [6:0] read_x,read_y;
    wire [15:0] base_color;
    integer checked=0,color_checked=0,ix,iy,mode,carx,cary;
    integer expected_x,expected_y,origin_x,origin_y,px,py;
    reg request_bank=0,active_bank=0,bank_ack=0;
    reg guard_reset_n=0,guard_point=0,guard_scan=0;
    reg [15:0] guard_distance=0,guard_scan_id=0;
    wire guard_ready,guard_front,guard_rear,guard_slow,guard_turn,guard_contact;
    wire [15:0] guard_radius,guard_self,guard_front_mm;
    reg [5:0] held_guard_flags;
    integer guard_checks=0;
    robot_control_panel dut(
        .clk(clk),.x(x),.y(y),.flags(flags),.map_cell(ram_cell),.map_car_model(car),
        .map_overlay_model(overlay),.map_follow_car(follow_car),
        .map_read_valid(read_valid),.map_read_en(read_en),
        .map_read_x(read_x),.map_read_y(read_y),.base_color(base_color),
        .label_source(),.label_row(),.label_literal(),.label_x(),.label_right(),
        .label_y(),.label_color(),.label_enable(),.override_label(),.override_base()
    );
    // REAL physical guard RTL, with measured geometry; no viewport/magnification
    // input. Keep its scene fixed while toggling the generated display inputs.
    lidar_body_guard physical_guard(
        .clk(clk),.reset_n(guard_reset_n),.tick_10ms(1'b0),.active(1'b1),
        .point_valid(guard_point),.scan_start(guard_scan),.angle_q6(15'd0),.forward_angle_q6(15'd0),
        .distance_mm(guard_distance),.quality(6'd20),.raw_node_valid(guard_point),
        .raw_parse_error(1'b0),.raw_reject_reason(3'd0),.scan_id(guard_scan_id),
        .offset_x_mm(16'sd33),.offset_y_mm(-16'sd77),
        .body_front_mm(16'd232),.body_rear_mm(16'd118),.body_left_mm(16'd119),.body_right_mm(16'd119),
        .drive_margin_mm(16'd20),.turn_margin_mm(16'd30),.contact_margin_mm(16'd10),
        .ready(guard_ready),.front_stop(guard_front),.rear_stop(guard_rear),.front_slow(guard_slow),
        .turn_stop(guard_turn),.virtual_contact(guard_contact),.coverage(),.free_coverage(),
        .scan_age_ticks(),.scan_period_ticks(),.projection_bad(),.last_scan_bad(),
        .scan_raw_nodes(),.scan_usable_nodes(),.scan_invalid_nodes(),.scan_parse_errors(),.scan_overruns(),
        .direction_point_counts(),.unconfirmed_flags(),.trigger_valid(),.trigger_kind(),.trigger_angle_q6(),
        .trigger_distance_mm(),.trigger_scan_id(),.trigger_quality(),.trigger_x_mm(),.trigger_y_mm(),
        .filtered_front_mm(guard_front_mm),.filtered_right_mm(),.filtered_rear_mm(),.filtered_left_mm(),
        .self_points(guard_self),.rotation_radius_mm(guard_radius),.blocking_x_mm(),.blocking_y_mm(),
        .map_point_valid(),.body_point_x(),.body_point_y(),.map_point_angle(),.map_point_distance(),.map_point_quality()
    );
    function [1:0] memory_cell;
        input integer gx,gy;
        input bank;
        begin memory_cell=(gx^(gy<<1)^bank)&3;end
    endfunction
    function [15:0] palette;
        input [1:0] value;
        begin case(value)
            0:palette=16'hc618;1:palette=16'hffff;
            2:palette=16'h45f2;default:palette=16'h18e3;
        endcase end
    endfunction
    // Same one-edge data/valid contract as READ_MODE1=0 snapshot RAM. Both
    // viewports use this ONE port; no copied local-map storage is present.
    always @(posedge clk)begin
        if(!read_en && request_bank!=bank_ack)begin
            active_bank<=request_bank;bank_ack<=request_bank;
        end
        if(read_en)ram_cell<=memory_cell(read_x,read_y,active_bank);
        read_valid<=read_en;
    end
    task sample_pixel;
        input integer sx,sy,gx,gy;
        reg [1:0] expected_cell;
        begin
            @(negedge clk);x=sx-1;y=sy;#1;
            if(!read_en || !dut.read_location_valid || read_x!=gx || read_y!=gy)
                $fatal(1,"prefetch pixel %0d,%0d expected %0d,%0d got %0d,%0d en%0d valid%0d",
                    sx,sy,gx,gy,read_x,read_y,read_en,dut.read_location_valid);
            expected_cell=memory_cell(gx,gy,active_bank);
            @(posedge clk);#1;x=sx;#1;
            if(!read_valid || ram_cell!==expected_cell || !dut.draw_location_valid ||
                dut.draw_x_signed!=gx || dut.draw_y_signed!=gy)
                $fatal(1,"one-clock address/data/draw mismatch at pixel%0d,%0d",sx,sy);
            if(dut.draw_grid_x!=gx || dut.draw_grid_y!=gy || !dut.draw_sample_valid)
                $fatal(1,"retimed world coordinate differs from same-edge RAM request");
            if(!dut.roi_outline && !dut.field_outline && !dut.leg_line &&
                !dut.goal_marker && !dut.leg_marker &&
                !(car[39] && (dut.car_origin || dut.car_arrow)))begin
                if(base_color!==palette(expected_cell))
                    $fatal(1,"palette pixel%0d,%0d got%h expected%h view%0d kind%0d draw%0d hit%0d",sx,sy,base_color,palette(expected_cell),dut.view,dut.view_kind,dut.draw_location_valid,dut.hit_button);
                color_checked=color_checked+1;
            end
            checked=checked+1;
        end
    endtask
    task color_at;
        input integer sx,sy,gx,gy;
        input [15:0] color;
        begin sample_pixel(sx,sy,gx,gy);
            if(base_color!==color)$fatal(1,"overlay %0d,%0d color%h expected%h",sx,sy,base_color,color);
        end
    endtask
    task no_location;
        input integer sx,sy;
        begin @(negedge clk);x=sx;y=sy;#1;
            if(dut.read_location_valid || read_x!==0 || read_y!==0)
                $fatal(1,"out-of-range address wrapped at %0d,%0d: %0d,%0d",sx,sy,read_x,read_y);
        end
    endtask
    task scan_boundary;
        begin @(negedge clk);guard_scan=1;guard_scan_id=guard_scan_id+1'b1;
            @(negedge clk);guard_scan=0;repeat(2)@(negedge clk);end
    endtask
    task guard_ray;
        input [15:0] mm;
        begin @(negedge clk);guard_distance=mm;guard_point=1;
            @(negedge clk);guard_point=0;repeat(300)@(negedge clk);end
    endtask
    task compare_guard_modes;
        begin
            held_guard_flags={guard_ready,guard_contact,guard_turn,guard_slow,guard_rear,guard_front};
            for(mode=0;mode<6;mode=mode+1)begin
                follow_car=mode[0];car={1'b1,3'd0,20'd0,7'd64,7'd64,2'd0};
                // Inspect a pixel in each viewport while the same real scene
                // remains in the physical guard. No geometry/threshold rewrite.
                sample_pixel(180,250,74,58);sample_pixel(650,300,71,64);
                if({guard_ready,guard_contact,guard_turn,guard_slow,guard_rear,guard_front}!==held_guard_flags ||
                    guard_radius!=291)
                    $fatal(1,"display ROI changed physical guard/radius");
                guard_checks=guard_checks+1;
            end
        end
    endtask
    initial begin
        repeat(2)@(negedge clk);
        guard_reset_n=1;
        flags=32'h10000002; // page2,subpage1, after reset-like startup
        // All global cells, including all four signed-map edges.
        for(iy=0;iy<128;iy=iy+1)
            for(ix=0;ix<128;ix=ix+1)sample_pixel(32+ix*2,367-iy*2,ix,iy);
        // Local uses a subset of the SAME addresses. FIELD alwaysorigin32;
        // CAR clamps the 64-cell ROI so it never wraps at either map edge.
        for(mode=0;mode<7;mode=mode+1)begin
            follow_car=(mode!=0);carx=mode==1 ? 0 : mode==2 ? 31 : mode==3 ? 32 : mode==4 ? 64 : mode==5 ? 96 :127;
            cary=127-carx;car={1'b1,3'd0,20'd0,carx[6:0],cary[6:0],2'd0};
            origin_x=mode==0 ? 32 : carx<=32 ? 0 : carx>=96 ? 64 : carx-32;
            origin_y=mode==0 ? 32 : cary<=32 ? 0 : cary>=96 ? 64 : cary-32;
            #1;if(dut.roi_x!=origin_x || dut.roi_y!=origin_y)$fatal(1,"ROI clamp failed");
            for(iy=0;iy<64;iy=iy+1)begin
                for(ix=0;ix<64;ix=ix+1)begin
                    // Pixel1 of each6x6 block; avoid relying on cell centers.
                    sample_pixel(417+ix*6,494-iy*6,origin_x+ix,origin_y+iy);
                end
            end
        end
        // An invalid/off-map car cannot drag the ROI to truncated addresses.
        car=0;follow_car=1;#1;
        if(dut.roi_x!=32 || dut.roi_y!=32)$fatal(1,"invalid car moved ROI");
        no_location(0,200);no_location(30,200);no_location(287,200);
        no_location(414,200);no_location(799,200);no_location(415,111);
        no_location(415,496);no_location(2047,200);no_location(0,1023);
        // Exact4m field occupies125 global pixels or375 localpixels. The
        // surrounding ROI is4.096m, not an inaccurately labelled4m grid.
        follow_car=0;car=0;
        color_at(97,250,32,58,16'hfd20);
        color_at(222,250,95,58,16'hfd20);
        color_at(180,177,74,95,16'hfd20);
        color_at(180,302,74,32,16'hfd20);
        color_at(420,300,32,64,16'hfd20);
        color_at(795,300,95,64,16'hfd20);
        color_at(650,116,71,95,16'hfd20);
        color_at(650,491,71,32,16'hfd20);
        color_at(96,250,32,58,16'h03df); // ROIoutside precisefield
        // Goal/current leg/car share the map transform in BOTH viewports.
        car={1'b1,3'd0,20'd0,7'd64,7'd64,2'd0};
        overlay={1'b1,7'd70,7'd67,1'b1,7'd70,7'd64};
        color_at(172,233,70,67,16'h13a8);
        color_at(647,282,70,67,16'h13a8);
        color_at(172,239,70,64,16'h07ff);
        color_at(647,300,70,64,16'h07ff);
        color_at(168,239,68,64,16'h781f);
        color_at(635,300,68,64,16'h781f);
        color_at(160,239,64,64,16'hd965);
        color_at(611,300,64,64,16'hd965);
        // Snapshot publication may be requested between the global/local
        // reads; the map reader holds ownership over the entire rasterband.
        overlay=0;car=0;
        @(negedge clk);x=0;y=80;request_bank=1;
        repeat(2)@(negedge clk);
        if(active_bank!=1)$fatal(1,"idle bank handoff failed");
        sample_pixel(180,250,74,58);
        request_bank=0;
        @(negedge clk);x=350;y=250;#1;
        if(!read_en)$fatal(1,"gap released map bank");
        repeat(3)@(negedge clk);
        if(active_bank!=1)$fatal(1,"bank swapped between views");
        sample_pixel(650,300,71,64);
        @(negedge clk);x=1023;y=400;#1;
        if(!read_en)$fatal(1,"blanking released map bank");
        repeat(2)@(negedge clk);
        if(active_bank!=1)$fatal(1,"bank swapped before map end");
        @(negedge clk);x=0;y=496;
        repeat(2)@(negedge clk);
        if(active_bank!=0)$fatal(1,"completed map band did not handoff bank");
        sample_pixel(180,250,74,58);sample_pixel(650,300,71,64);
        // Known SELF plus a clear ray: read actual guard output, then check
        // the same flags with field-centred and car-centred displays.
        repeat(100)@(negedge clk);
        if(!physical_guard.config_ready)$fatal(1,"real guard config timeout");
        scan_boundary;guard_ray(16'd100);guard_ray(16'd1000);scan_boundary;
        if(!guard_ready || guard_front || guard_contact || guard_self!=1 || guard_front_mm!=1000)
            $fatal(1,"clear/SELF real guard fixture failed");
        compare_guard_modes;
        // Real sensor-origin207mm -> body x240,y-77mm, inside10mm contact
        // zone beyond232mm front. This is NOT a viewport-scaled distance.
        guard_ray(16'd207);scan_boundary;
        if(!guard_ready || !guard_front || !guard_contact || guard_front_mm!=207)
            $fatal(1,"near-obstacle real guard fixture failed");
        compare_guard_modes;
        flags=0;#1;if(read_en)$fatal(1,"nonmap view still reads map");
        $display("PASS generated RTL viewport: %0d addresses/data, %0d palette pixels; same snapshot, signed edges, ROI clamp, exact4m, car/goal/currentleg; %0d REAL guard mode checks",checked,color_checked,guard_checks);
        $finish;
    end
    initial begin #10000000;$fatal(1,"viewport test timeout");end
endmodule
