// Generated from tools/robot_ui_spec.py; edit the shared specification.
// Main page stays 3 bits. Subpage occupies flags[30:28]; flags[31] is gains.
// Compact descriptors preserve the existing layout/character/glyph/RGB pipe.
module robot_control_panel (
    input wire clk,
    input wire [10:0] x,
    input wire [9:0] y,
    input wire [31:0] flags,
    input wire [1:0] map_cell,
    input wire [39:0] map_car_model,
    // Pixel-domain, coherent with map_car_model; valid bits precede address truncation.
    input wire [29:0] map_overlay_model,
    input wire map_follow_car,
    input wire map_read_valid,
    output wire map_read_en,
    output wire [6:0] map_read_x,map_read_y,
    output reg [2:0] label_source,
    output reg [3:0] label_row,
    output reg [6:0] label_literal,
    output reg [10:0] label_x,label_right,
    output reg [9:0] label_y,
    output reg [15:0] label_color,base_color,
    output reg label_enable,override_label,override_base
);
    localparam [15:0] BG=16'hf79e,WHITE=16'hffff,INK=16'h1926,BLUE=16'h03df,
        SOFT=16'he79f,RED=16'hd965,GREEN=16'h13a8,MUTED=16'h63b1;
    wire [2:0] page=flags[2:0],subpage=flags[30:28];
    wire [5:0] view={page,subpage};
    reg [2:0] view_kind;
    always @* begin
        case(view)
            6'd0:view_kind=3'd0;
            6'd19,6'd27,6'd32,6'd33,6'd40,6'd43,6'd44:view_kind=3'd1;
            6'd1:view_kind=3'd2;
            6'd17:view_kind=3'd3;
            default:view_kind=3'd5;
        endcase
    end
    wire [7:0] hit_button;
    robot_ui_hit_test u_pixel_hit(.x(x),.y(y),.page(page),.subpage(subpage),.button(hit_button));
    // Both views read ONE immutable LCD snapshot through its existing one-clock
    // RAM port. Global:128 cells *2 pixels. Local:64 cells *6 pixels (4.096 m).
    // The 4 m field below is a separate exact [-2000,+2000] mm world outline.
    wire [6:0] roi_x=map_follow_car && map_car_model[39] ?
        (map_car_model[15:9]<=32 ? 7'd0 : map_car_model[15:9]>=96 ? 7'd64 : map_car_model[15:9]-7'd32) : 7'd32;
    wire [6:0] roi_y=map_follow_car && map_car_model[39] ?
        (map_car_model[8:2]<=32 ? 7'd0 : map_car_model[8:2]>=96 ? 7'd64 : map_car_model[8:2]-7'd32) : 7'd32;
    wire global_map=x>=32 && x<288 && y>=112 && y<368;
    wire local_map=x>=416 && x<800 && y>=112 && y<496;
    wire in_map=global_map || local_map;
    wire global_prefetch=x>=31 && x<287 && y>=112 && y<368;
    wire local_prefetch=x>=415 && x<799 && y>=112 && y<496;
    // Widen and SIGN before subtraction, division and bounds checks. Never
    // truncate a negative/out-of-window coordinate to a 7-bit RAM address.
    wire signed [11:0] screen_x=$signed({1'b0,x});
    wire signed [11:0] screen_y=$signed({2'b00,y});
    // Screen bounds above establish nonnegative deltas BEFORE narrowing these
    // small constant-/6 operands; no signed twelve-bit divider is inferred.
    wire [8:0] local_read_column=local_prefetch ? x-11'd415 : 9'd0;
    wire [8:0] local_row=local_prefetch ? 10'd495-y : 9'd0;
    wire [6:0] local_read_dx=local_read_column/9'd6;
    wire [6:0] local_dy=local_row/9'd6;
    wire signed [11:0] read_x_signed=local_prefetch ? $signed({5'd0,roi_x})+$signed({5'd0,local_read_dx}) :
        (screen_x-12'sd31)>>>1;
    wire signed [11:0] read_y_signed=local_prefetch ? $signed({5'd0,roi_y})+$signed({5'd0,local_dy}) :
        (12'sd367-screen_y)>>>1;
    wire read_location_valid=(global_prefetch || local_prefetch) &&
        read_x_signed>=0 && read_x_signed<128 && read_y_signed>=0 && read_y_signed<128;
    // Keep the reader's bank ownership for the ENTIRE map band, including gaps
    // and horizontal blanking. !rd_en still switches/ACKs the completed bank;
    // it cannot switch between the two views or halfway through a visible map.
    assign map_read_en=page==2 && subpage==1 && y>=112 && y<496;
    assign map_read_x=read_location_valid ? read_x_signed[6:0] : 7'd0;
    assign map_read_y=read_location_valid ? read_y_signed[6:0] : 7'd0;
    // RAM and these15FF sample the SAME prefetch on the SAME pixel edge.
    // Returned map_cell and registered world-grid coordinates then describe
    // the current screen pixel. This splits /6+ROI from marker comparisons;
    // it adds no RAM latency, raster delay, display bank or CDC crossing.
    reg [6:0] draw_grid_x=0,draw_grid_y=0;
    reg draw_sample_valid=0;
    always @(posedge clk)begin
        draw_grid_x<=map_read_x;draw_grid_y<=map_read_y;
        draw_sample_valid<=read_location_valid;
    end
    wire signed [8:0] draw_x_signed=$signed({2'b00,draw_grid_x});
    wire signed [8:0] draw_y_signed=$signed({2'b00,draw_grid_y});
    wire draw_location_valid=in_map && draw_sample_valid;
    wire signed [8:0] car_dx=draw_x_signed-$signed({2'b00,map_car_model[15:9]});
    wire signed [8:0] car_dy=draw_y_signed-$signed({2'b00,map_car_model[8:2]});
    wire car_origin=(car_dx==0 && car_dy>=-1 && car_dy<=1) || (car_dy==0 && car_dx>=-1 && car_dx<=1);
    reg car_arrow;
    always @* begin
        case(map_car_model[38:36])
            0:car_arrow=car_dy==0 && car_dx>0 && car_dx<=3;
            1:car_arrow=car_dx==car_dy && car_dx>0 && car_dx<=2;
            2:car_arrow=car_dx==0 && car_dy>0 && car_dy<=3;
            3:car_arrow=car_dx== -car_dy && car_dy>0 && car_dy<=2;
            4:car_arrow=car_dy==0 && car_dx<0 && car_dx>=-3;
            5:car_arrow=car_dx==car_dy && car_dx<0 && car_dx>=-2;
            6:car_arrow=car_dx==0 && car_dy<0 && car_dy>=-3;
            default:car_arrow=car_dx== -car_dy && car_dx>0 && car_dx<=2;
        endcase
    end
    // Goal/current-leg endpoint use exactly the SAME world-grid coordinate as
    // occupancy and car. This is the current commanded leg, not the full BFS
    // route. Draw its straight axis-aligned part; other legs show endpoints.
    wire [6:0] goal_x=map_overlay_model[28:22],goal_y=map_overlay_model[21:15];
    wire [6:0] leg_x=map_overlay_model[13:7],leg_y=map_overlay_model[6:0];
    wire signed [8:0] goal_dx=draw_x_signed-$signed({2'b00,goal_x});
    wire signed [8:0] goal_dy=draw_y_signed-$signed({2'b00,goal_y});
    wire signed [8:0] leg_dx=draw_x_signed-$signed({2'b00,leg_x});
    wire signed [8:0] leg_dy=draw_y_signed-$signed({2'b00,leg_y});
    wire goal_marker=map_overlay_model[29] && ((goal_dx==0 && goal_dy>=-1 && goal_dy<=1) ||
        (goal_dy==0 && goal_dx>=-1 && goal_dx<=1));
    wire leg_marker=map_overlay_model[14] && ((leg_dx==0 && leg_dy>=-1 && leg_dy<=1) ||
        (leg_dy==0 && leg_dx>=-1 && leg_dx<=1));
    wire leg_line=map_overlay_model[14] && map_car_model[39] &&
        ((leg_x==map_car_model[15:9] && car_dx==0 &&
          ((car_dy>=0 && leg_dy<=0)||(car_dy<=0 && leg_dy>=0))) ||
         (leg_y==map_car_model[8:2] && car_dy==0 &&
          ((car_dx>=0 && leg_dx<=0)||(car_dx<=0 && leg_dx>=0))));
    // World-coordinate boundaries in twelfths of a 64 mm cell. Pixel spans
    // are 6 units globally and 2 locally. +/-2000 mm is EXACTLY393/1143 units
    // from the map's -4096 mm edge; no 62/63-cell approximation or fake scale.
    wire [11:0] global_px=x-11'd32,global_py=10'd367-y;
    wire [11:0] local_px=x-11'd416,local_py=10'd495-y;
    wire [11:0] roi_qx=({5'd0,roi_x}<<3)+({5'd0,roi_x}<<2);
    wire [11:0] roi_qy=({5'd0,roi_y}<<3)+({5'd0,roi_y}<<2);
    wire [11:0] pixel_qx=local_map ? roi_qx+(local_px<<1) : (global_px<<2)+(global_px<<1);
    wire [11:0] pixel_qy=local_map ? roi_qy+(local_py<<1) : (global_py<<2)+(global_py<<1);
    wire [11:0] pixel_span=local_map ? 12'd2 : 12'd6;
    wire field_x_edge=(pixel_qx<=393 && pixel_qx+pixel_span>393) || (pixel_qx<=1143 && pixel_qx+pixel_span>1143);
    wire field_y_edge=(pixel_qy<=393 && pixel_qy+pixel_span>393) || (pixel_qy<=1143 && pixel_qy+pixel_span>1143);
    wire field_outline=(field_x_edge && pixel_qy<=1143 && pixel_qy+pixel_span>393) ||
        (field_y_edge && pixel_qx<=1143 && pixel_qx+pixel_span>393);
    wire roi_x_edge=(pixel_qx<=roi_qx && pixel_qx+pixel_span>roi_qx) ||
        (pixel_qx<roi_qx+768 && pixel_qx+pixel_span>=roi_qx+768);
    wire roi_y_edge=(pixel_qy<=roi_qy && pixel_qy+pixel_span>roi_qy) ||
        (pixel_qy<roi_qy+768 && pixel_qy+pixel_span>=roi_qy+768);
    wire roi_outline=global_map && ((roi_x_edge && pixel_qy>=roi_qy && pixel_qy<roi_qy+768) ||
        (roi_y_edge && pixel_qx>=roi_qx && pixel_qx<roi_qx+768));
    wire [3:0] info_line=(y-10'd132)>>5;
    wire info_line_pixel=y>=132 && y<308 && (y[4:0]>=4 && y[4:0]<20);
    wire [3:0] map_info_line=(y-10'd80)>>5;
    wire map_info_pixel=y>=80 && y<256 && y[4:0]>=16;
    wire [9:0] diag_dy=y-10'd332;
    wire [2:0] diag_line=diag_dy[7:5];
    wire diag_pixel=y>=332 && y<476 && diag_dy[4:0]<16;
    reg [10:0] button_text_x,button_label_right;
    reg [9:0] button_text_y;
    reg [6:0] button_caption;
    wire stop_button=hit_button==106 || (view==6'd9 && hit_button==4) ||
        ((view==6'd10 || view==6'd11) && hit_button==2) ||
        ((view==6'd32 || view==6'd33) && hit_button==6) ||
        ((view==6'd40 && (hit_button==3 || hit_button==7)) || (view==6'd43 && (hit_button==6 || hit_button==7)) || (view==6'd44 && (hit_button==4 || hit_button==7))) ||
        (view==6'd41 && hit_button==4) || (view==6'd19 && hit_button==2);
    function rect;
        input [10:0] px,left,right;
        input [9:0] py,top,bottom;
        begin rect=px>=left && px<right && py>=top && py<bottom;end
    endfunction
    function [6:0] view_caption;
        input [5:0] view;
        input [3:0] slot;
        begin
            view_caption=0;
            case(view)
                6'd0:case(slot)
                    4'd1:view_caption=7'd5;
                    4'd2:view_caption=7'd6;
                    4'd3:view_caption=7'd7;
                    4'd4:view_caption=7'd8;
                    4'd5:view_caption=7'd9;
                    4'd6:view_caption=7'd10;
                    default:view_caption=0;
                endcase
                6'd17:case(slot)
                    4'd1:view_caption=7'd2;
                    4'd2:view_caption=7'd15;
                    4'd3:view_caption=7'd16;
                    default:view_caption=0;
                endcase
                6'd19:case(slot)
                    4'd1:view_caption=7'd19;
                    4'd2:view_caption=7'd20;
                    4'd3:view_caption=7'd21;
                    4'd4:view_caption=7'd22;
                    default:view_caption=0;
                endcase
                6'd27:case(slot)
                    4'd1:view_caption=7'd25;
                    4'd2:view_caption=7'd26;
                    4'd3:view_caption=7'd27;
                    4'd4:view_caption=7'd28;
                    4'd5:view_caption=7'd29;
                    4'd6:view_caption=7'd30;
                    default:view_caption=0;
                endcase
                6'd32:case(slot)
                    4'd1:view_caption=7'd33;
                    4'd2:view_caption=7'd34;
                    4'd3:view_caption=7'd35;
                    4'd4:view_caption=7'd36;
                    4'd5:view_caption=7'd37;
                    4'd6:view_caption=7'd20;
                    default:view_caption=0;
                endcase
                6'd33:case(slot)
                    4'd1:view_caption=7'd40;
                    4'd2:view_caption=7'd41;
                    4'd3:view_caption=7'd42;
                    4'd4:view_caption=7'd43;
                    4'd5:view_caption=7'd44;
                    4'd6:view_caption=7'd20;
                    default:view_caption=0;
                endcase
                6'd40:case(slot)
                    4'd1:view_caption=7'd45;
                    4'd2:view_caption=7'd46;
                    4'd3:view_caption=7'd47;
                    4'd7:view_caption=7'd48;
                    default:view_caption=0;
                endcase
                6'd43:case(slot)
                    4'd1:view_caption=7'd51;
                    4'd2:view_caption=7'd52;
                    4'd3:view_caption=7'd53;
                    4'd4:view_caption=7'd54;
                    4'd5:view_caption=7'd55;
                    4'd6:view_caption=7'd20;
                    4'd7:view_caption=7'd48;
                    default:view_caption=0;
                endcase
                6'd44:case(slot)
                    4'd1:view_caption=7'd45;
                    4'd2:view_caption=7'd58;
                    4'd3:view_caption=7'd59;
                    4'd4:view_caption=7'd20;
                    4'd7:view_caption=7'd48;
                    default:view_caption=0;
                endcase
                default:view_caption=0;
            endcase
        end
    endfunction
    function [6:0] view_hint;
        input [5:0] view;
        begin case(view)
            6'd0:view_hint=7'd11;
            6'd1:view_hint=7'd13;
            6'd17:view_hint=7'd17;
            6'd19:view_hint=7'd23;
            6'd27:view_hint=7'd31;
            6'd32:view_hint=7'd38;
            6'd33:view_hint=7'd31;
            6'd40:view_hint=7'd49;
            6'd43:view_hint=7'd56;
            6'd44:view_hint=7'd60;
            default:view_hint=7'd0;
        endcase end
    endfunction
    function [6:0] view_detail;
        input [5:0] view;
        begin case(view)
            6'd0:view_detail=7'd12;
            6'd1:view_detail=7'd14;
            6'd17:view_detail=7'd18;
            6'd19:view_detail=7'd24;
            6'd27:view_detail=7'd32;
            6'd32:view_detail=7'd39;
            6'd33:view_detail=7'd32;
            6'd40:view_detail=7'd50;
            6'd43:view_detail=7'd57;
            6'd44:view_detail=7'd61;
            default:view_detail=7'd0;
        endcase end
    endfunction
    // Fixed text anchors avoid variable subtract/add/centering hardware.
    // Fill and touch both use the SINGLE shared hit-test module above.
    always @* begin
        button_text_x=0;button_label_right=0;button_text_y=0;button_caption=0;
        if(hit_button>=100)begin
            button_text_x=840;button_label_right=1004;
            case(hit_button)
                100:begin button_text_y=80;button_caption=7'd1;end
                101:begin button_text_y=152;button_caption=7'd2;end
                105:begin button_text_y=414;button_caption=7'd3;end
                106:begin button_text_y=500;button_caption=7'd4;end
                default:begin end
            endcase
        end else if(hit_button!=0)begin
            button_caption=view_caption(view,hit_button[3:0]);
            if(view_kind==0)begin
                button_text_x=hit_button[0] ? 48 : 448;button_label_right=hit_button[0] ? 380 : 780;
                if(hit_button<=2)button_text_y=160;
                else if(hit_button<=4)button_text_y=272;else button_text_y=384;
            end else if(view_kind==3)begin
                button_text_x=hit_button==2 ? 168 : 40;
                button_label_right=hit_button==1 ? 156 : 284;
                button_text_y=hit_button==3 ? 426 : 484;
            end else if(view_kind==4)begin
                button_text_x=hit_button[0] ? 40 : 440;
                button_label_right=hit_button[0] ? 380 : 780;
                button_text_y=476;
            end else if(hit_button<=6)begin
                button_text_x=hit_button[0] ? 416 : 616;button_label_right=hit_button[0] ? 592 : 792;
                if(hit_button<=2)button_text_y=150;
                else if(hit_button<=4)button_text_y=238;else button_text_y=326;
            end else begin
                button_text_x=hit_button==8 ? 216 : 40;
                button_label_right=(view==6'd32 || view==6'd33) && hit_button==7 ? 188 : 380;
                button_text_y=454;
            end
        end
    end
    always @* begin
        label_source=0;label_row=0;label_literal=0;label_x=0;label_right=1024;label_y=0;
        label_color=INK;label_enable=0;override_label=1;override_base=0;base_color=BG;
        if(view_kind!=2 && rect(x,16,816,y,56,536))begin
            override_base=1;
            if(view_kind==1 && x>=28 && x<388 && y>=116 && y<(view==16 ? 500 : 416))base_color=WHITE;
            if(view_kind==4 && y>=112 && y<444 && ((x>=32 && x<384)||(x>=432 && x<784)))base_color=WHITE;
            if(view_kind==3 && draw_location_valid)base_color=!map_read_valid ? 16'h8410 :
                map_cell==0 ? 16'hc618 : map_cell==1 ? WHITE : map_cell==2 ? 16'h45f2 : 16'h18e3;
            if(view_kind==3 && draw_location_valid)begin
                if(roi_outline)base_color=BLUE;
                if(field_outline)base_color=16'hfd20;
                if(leg_line)base_color=16'h781f;
                if(goal_marker)base_color=GREEN;
                if(leg_marker)base_color=16'h07ff;
            end
            if(view_kind==3 && draw_location_valid && map_car_model[39])begin
                if(car_arrow)base_color=16'hffe0;
                if(car_origin)base_color=RED;
            end
        end
        if(rect(x,824,1016,y,52,540))begin override_base=1;base_color=WHITE;end
        //16 cells =1024 mm, exactly32 global pixels; ticks are not a rounded1m.
        if(view_kind==3 && ((x>=32 && x<=64 && y>=402 && y<404) ||
            ((x==32 || x==64) && y>=398 && y<408)))begin override_base=1;base_color=INK;end
        if(hit_button!=0)begin
            override_base=1;base_color=stop_button ? RED : SOFT;
            if(hit_button==100)base_color=BLUE;
            if(view_kind==3 && x==159 && y>=472)base_color=WHITE;
            // Touch rectangles are much larger than their centered text.
            label_enable=1;label_literal=button_caption;
            label_x=button_text_x;label_right=button_label_right;label_y=button_text_y;
            label_color=stop_button || hit_button==100 ? WHITE : BLUE;
        end
        if(view_kind==0 && y>=88 && y<104)begin
            label_enable=1;label_y=88;
            if(x>=32 && x<384)begin label_x=32;label_right=384;label_literal=view_hint(view);end
            else if(x>=432 && x<784)begin label_x=432;label_right=784;label_literal=view_detail(view);label_color=MUTED;end
            else label_enable=0;
        end
        if(view_kind==1 && x>=36 && x<388 && info_line_pixel)begin
            label_enable=1;label_x=36;label_right=388;
            label_y=10'd132+({6'd0,info_line}<<5);label_source=1;label_row=info_line+4'd2;
        end
        if(view_kind==1 && x>=36 && x<388 && ((y>=376 && y<392)||(y>=400 && y<416)))begin
            label_enable=1;label_source=0;label_x=36;label_right=388;label_color=MUTED;
            label_y=y<392 ? 376 : 400;label_literal=y<392 ? view_hint(view) : view_detail(view);
        end
        if(view_kind==3 && y>=88 && y<104)begin
            label_enable=1;label_y=88;
            if(x>=32 && x<288)begin label_x=32;label_right=288;label_literal=7'd62;end
            else if(x>=416 && x<800)begin label_x=416;label_right=800;label_literal=7'd63;end
            else label_enable=0;
        end
        if(view_kind==3 && x>=32 && x<288 && y>=376 && y<392)begin
            label_enable=1;label_literal=7'd64;label_x=32;label_right=288;label_y=376;label_color=MUTED;
        end
        if(view_kind==3 && x>=80 && x<288 && y>=396 && y<412)begin
            label_enable=1;label_literal=7'd69;label_x=80;label_right=288;label_y=396;label_color=MUTED;
        end
        if(view_kind==3 && y>=512 && y<528)begin
            label_enable=1;label_y=512;
            if(x>=416 && x<608)begin label_x=416;label_right=608;label_literal=map_follow_car ? 7'd67 : 7'd66;end
            else if(x>=620 && x<800)begin label_x=620;label_right=800;label_literal=7'd68;label_color=16'h781f;end
            else label_enable=0;
        end
        if(view_kind==3 && x>=32 && x<288 && y>=516 && y<532)begin
            label_enable=1;label_x=32;label_right=288;label_y=516;label_literal=7'd65;label_color=16'hfd20;
        end
        // Exactly four concise status lines, common across all levels.
        if(x>=824 && x<1024 && y>=224 && y<336 && y[4:0]<16)begin
            label_enable=1;label_source=1;label_x=824;label_right=1024;
            label_y={y[9:5],5'd0};
            case(y[6:5])
                3:label_row=1;0:label_row=8;1:label_row=9;default:label_row=10;
            endcase
            if(flags[9])label_color=RED;
        end
        if(y>=14 && y<30)begin
            label_enable=1;label_y=14;
            if(x>=20 && x<280)begin label_x=20;label_right=280;label_source=1;label_row=0;end
            else if(x>=300 && x<560)begin label_x=300;label_right=560;label_source=4;label_row=0;end
            else if(x>=600 && x<820)begin label_x=600;label_right=820;label_source=4;label_row=1;end
            else label_enable=0;
        end
        if((y>=558 && y<574)||(y>=582 && y<598))begin
            label_enable=1;label_source=3;label_y=y<574 ? 558 : 582;label_color=y<574 ? INK : MUTED;
            if(x>=20 && x<280)begin label_x=20;label_right=280;label_row=y<574 ? 1 : 4;end
            else if(x>=300 && x<560)begin label_x=300;label_right=560;label_row=y<574 ? 2 : 5;end
            else if(x>=600 && x<820)begin label_x=600;label_right=820;label_row=y<574 ? 3 : 6;end
            else label_enable=0;
        end
    end
endmodule

// This exact hit module is shared by the LCD and both touch contacts.
module robot_ui_hit_test (
    input wire [10:0] x,
    input wire [9:0] y,
    input wire [2:0] page,subpage,
    output reg [7:0] button
);
    wire [5:0] view={page,subpage};
    reg [2:0] view_kind;
    always @* begin
        case(view)
            6'd0:view_kind=3'd0;
            6'd19,6'd27,6'd32,6'd33,6'd40,6'd43,6'd44:view_kind=3'd1;
            6'd1:view_kind=3'd2;
            6'd17:view_kind=3'd3;
            default:view_kind=3'd5;
        endcase
    end
    function rect;
        input [10:0] px;
        input [9:0] py;
        input [10:0] left,right;
        input [9:0] top,bottom;
        begin rect=px>=left && px<right && py>=top && py<bottom;end
    endfunction
    always @* begin
        button=0;
        case(view_kind)
            3'd0:begin
                if(rect(x,y,11'd32,11'd384,10'd128,10'd208))button=8'd1;
                if(rect(x,y,11'd432,11'd784,10'd128,10'd208))button=8'd2;
                if(rect(x,y,11'd32,11'd384,10'd240,10'd320))button=8'd3;
                if(rect(x,y,11'd432,11'd784,10'd240,10'd320))button=8'd4;
                if(rect(x,y,11'd32,11'd384,10'd352,10'd432))button=8'd5;
                if(rect(x,y,11'd432,11'd784,10'd352,10'd432))button=8'd6;
            end
            3'd1:begin
                if(rect(x,y,11'd408,11'd596,10'd128,10'd188))button=8'd1;
                if(rect(x,y,11'd608,11'd796,10'd128,10'd188))button=8'd2;
                if(rect(x,y,11'd408,11'd596,10'd216,10'd276))button=8'd3;
                if(rect(x,y,11'd608,11'd796,10'd216,10'd276))button=8'd4;
                if(rect(x,y,11'd408,11'd596,10'd304,10'd364))button=8'd5;
                if(rect(x,y,11'd608,11'd796,10'd304,10'd364))button=8'd6;
            end
            3'd3:begin
                if(rect(x,y,11'd32,11'd160,10'd472,10'd512))button=8'd1;
                if(rect(x,y,11'd160,11'd288,10'd472,10'd512))button=8'd2;
                if(rect(x,y,11'd32,11'd288,10'd416,10'd452))button=8'd3;
            end
            3'd4:begin
                if(rect(x,y,11'd32,11'd384,10'd452,10'd516))button=8'd1;
                if(rect(x,y,11'd432,11'd784,10'd452,10'd516))button=8'd2;
            end
            default:begin end
        endcase
        if(view==6'd40 || view==6'd43 || view==6'd44)begin
            if(rect(x,y,11'd32,11'd384,10'd432,10'd492))button=8'd7;
        end
        // Only visible core actions receive a touch ID.
        if(view==6'd0 && button>0 && button<100 && !(button==1 || button==2 || button==3 || button==4 || button==5 || button==6))button=0;
        if(view==6'd17 && button>0 && button<100 && !(button==1 || button==2 || button==3))button=0;
        if(view==6'd19 && button>0 && button<100 && !(button==1 || button==2 || button==3 || button==4))button=0;
        if(view==6'd27 && button>0 && button<100 && !(button==1 || button==2 || button==3 || button==4 || button==5 || button==6))button=0;
        if(view==6'd32 && button>0 && button<100 && !(button==1 || button==2 || button==3 || button==4 || button==5 || button==6))button=0;
        if(view==6'd33 && button>0 && button<100 && !(button==1 || button==2 || button==3 || button==4 || button==5 || button==6))button=0;
        if(view==6'd40 && button>0 && button<100 && !(button==1 || button==2 || button==3 || button==7))button=0;
        if(view==6'd43 && button>0 && button<100 && !(button==1 || button==2 || button==3 || button==4 || button==5 || button==6 || button==7))button=0;
        if(view==6'd44 && button>0 && button<100 && !(button==1 || button==2 || button==3 || button==4 || button==7))button=0;

        if(rect(x,y,11'd832,11'd1008,10'd60,10'd116))button=8'd100;
        if(rect(x,y,11'd832,11'd1008,10'd132,10'd188))button=8'd101;
        if(rect(x,y,11'd832,11'd1008,10'd400,10'd444))button=8'd105;
        if(rect(x,y,11'd832,11'd1008,10'd480,10'd536))button=8'd106;
    end
endmodule

// 128 x16 ASCII captions, one synchronous character-stage ROM.
module robot_label_rom (
    input wire clk,
    input wire [6:0] label_id,
    input wire [3:0] character_index,
    output reg [7:0] character
);
    (* syn_romstyle="block_rom" *) reg [7:0] bytes[0:2047];
    integer i;
    initial begin
        // Gowin limits each initialization loop to 2000 iterations.
        // Fill both halves in 1024 iterations; labels override below.
        for(i=0;i<1024;i=i+1)begin
            bytes[i]=8'h20;
            bytes[i+1024]=8'h20;
        end
        // ID 1: "HOME            "
        bytes[16]=8'h48;
        bytes[17]=8'h4f;
        bytes[18]=8'h4d;
        bytes[19]=8'h45;
        // ID 2: "BACK            "
        bytes[32]=8'h42;
        bytes[33]=8'h41;
        bytes[34]=8'h43;
        bytes[35]=8'h4b;
        // ID 3: "CLEAR STOP      "
        bytes[48]=8'h43;
        bytes[49]=8'h4c;
        bytes[50]=8'h45;
        bytes[51]=8'h41;
        bytes[52]=8'h52;
        bytes[53]=8'h20;
        bytes[54]=8'h53;
        bytes[55]=8'h54;
        bytes[56]=8'h4f;
        bytes[57]=8'h50;
        // ID 4: "ALL STOP        "
        bytes[64]=8'h41;
        bytes[65]=8'h4c;
        bytes[66]=8'h4c;
        bytes[67]=8'h20;
        bytes[68]=8'h53;
        bytes[69]=8'h54;
        bytes[70]=8'h4f;
        bytes[71]=8'h50;
        // ID 5: "1 CAMERA + IMU  "
        bytes[80]=8'h31;
        bytes[81]=8'h20;
        bytes[82]=8'h43;
        bytes[83]=8'h41;
        bytes[84]=8'h4d;
        bytes[85]=8'h45;
        bytes[86]=8'h52;
        bytes[87]=8'h41;
        bytes[88]=8'h20;
        bytes[89]=8'h2b;
        bytes[90]=8'h20;
        bytes[91]=8'h49;
        bytes[92]=8'h4d;
        bytes[93]=8'h55;
        // ID 6: "2 MOTOR CONTROL "
        bytes[96]=8'h32;
        bytes[97]=8'h20;
        bytes[98]=8'h4d;
        bytes[99]=8'h4f;
        bytes[100]=8'h54;
        bytes[101]=8'h4f;
        bytes[102]=8'h52;
        bytes[103]=8'h20;
        bytes[104]=8'h43;
        bytes[105]=8'h4f;
        bytes[106]=8'h4e;
        bytes[107]=8'h54;
        bytes[108]=8'h52;
        bytes[109]=8'h4f;
        bytes[110]=8'h4c;
        // ID 7: "3 LIDAR AVOID   "
        bytes[112]=8'h33;
        bytes[113]=8'h20;
        bytes[114]=8'h4c;
        bytes[115]=8'h49;
        bytes[116]=8'h44;
        bytes[117]=8'h41;
        bytes[118]=8'h52;
        bytes[119]=8'h20;
        bytes[120]=8'h41;
        bytes[121]=8'h56;
        bytes[122]=8'h4f;
        bytes[123]=8'h49;
        bytes[124]=8'h44;
        // ID 8: "4 XY NAVIGATION "
        bytes[128]=8'h34;
        bytes[129]=8'h20;
        bytes[130]=8'h58;
        bytes[131]=8'h59;
        bytes[132]=8'h20;
        bytes[133]=8'h4e;
        bytes[134]=8'h41;
        bytes[135]=8'h56;
        bytes[136]=8'h49;
        bytes[137]=8'h47;
        bytes[138]=8'h41;
        bytes[139]=8'h54;
        bytes[140]=8'h49;
        bytes[141]=8'h4f;
        bytes[142]=8'h4e;
        // ID 9: "5 GIMBAL TRACK  "
        bytes[144]=8'h35;
        bytes[145]=8'h20;
        bytes[146]=8'h47;
        bytes[147]=8'h49;
        bytes[148]=8'h4d;
        bytes[149]=8'h42;
        bytes[150]=8'h41;
        bytes[151]=8'h4c;
        bytes[152]=8'h20;
        bytes[153]=8'h54;
        bytes[154]=8'h52;
        bytes[155]=8'h41;
        bytes[156]=8'h43;
        bytes[157]=8'h4b;
        // ID 10: "6 FIND FACE     "
        bytes[160]=8'h36;
        bytes[161]=8'h20;
        bytes[162]=8'h46;
        bytes[163]=8'h49;
        bytes[164]=8'h4e;
        bytes[165]=8'h44;
        bytes[166]=8'h20;
        bytes[167]=8'h46;
        bytes[168]=8'h41;
        bytes[169]=8'h43;
        bytes[170]=8'h45;
        // ID 11: "SELECT A TASK   "
        bytes[176]=8'h53;
        bytes[177]=8'h45;
        bytes[178]=8'h4c;
        bytes[179]=8'h45;
        bytes[180]=8'h43;
        bytes[181]=8'h54;
        bytes[182]=8'h20;
        bytes[183]=8'h41;
        bytes[184]=8'h20;
        bytes[185]=8'h54;
        bytes[186]=8'h41;
        bytes[187]=8'h53;
        bytes[188]=8'h4b;
        // ID 12: "BACK TO HOME    "
        bytes[192]=8'h42;
        bytes[193]=8'h41;
        bytes[194]=8'h43;
        bytes[195]=8'h4b;
        bytes[196]=8'h20;
        bytes[197]=8'h54;
        bytes[198]=8'h4f;
        bytes[199]=8'h20;
        bytes[200]=8'h48;
        bytes[201]=8'h4f;
        bytes[202]=8'h4d;
        bytes[203]=8'h45;
        // ID 13: "LOCAL FPGA VIDEO"
        bytes[208]=8'h4c;
        bytes[209]=8'h4f;
        bytes[210]=8'h43;
        bytes[211]=8'h41;
        bytes[212]=8'h4c;
        bytes[213]=8'h20;
        bytes[214]=8'h46;
        bytes[215]=8'h50;
        bytes[216]=8'h47;
        bytes[217]=8'h41;
        bytes[218]=8'h20;
        bytes[219]=8'h56;
        bytes[220]=8'h49;
        bytes[221]=8'h44;
        bytes[222]=8'h45;
        bytes[223]=8'h4f;
        // ID 14: "ONE TAP PER STEP"
        bytes[224]=8'h4f;
        bytes[225]=8'h4e;
        bytes[226]=8'h45;
        bytes[227]=8'h20;
        bytes[228]=8'h54;
        bytes[229]=8'h41;
        bytes[230]=8'h50;
        bytes[231]=8'h20;
        bytes[232]=8'h50;
        bytes[233]=8'h45;
        bytes[234]=8'h52;
        bytes[235]=8'h20;
        bytes[236]=8'h53;
        bytes[237]=8'h54;
        bytes[238]=8'h45;
        bytes[239]=8'h50;
        // ID 15: "CLEAR MAP       "
        bytes[240]=8'h43;
        bytes[241]=8'h4c;
        bytes[242]=8'h45;
        bytes[243]=8'h41;
        bytes[244]=8'h52;
        bytes[245]=8'h20;
        bytes[246]=8'h4d;
        bytes[247]=8'h41;
        bytes[248]=8'h50;
        // ID 16: "FIELD / CAR     "
        bytes[256]=8'h46;
        bytes[257]=8'h49;
        bytes[258]=8'h45;
        bytes[259]=8'h4c;
        bytes[260]=8'h44;
        bytes[261]=8'h20;
        bytes[262]=8'h2f;
        bytes[263]=8'h20;
        bytes[264]=8'h43;
        bytes[265]=8'h41;
        bytes[266]=8'h52;
        // ID 17: "8.192M / 64MM   "
        bytes[272]=8'h38;
        bytes[273]=8'h2e;
        bytes[274]=8'h31;
        bytes[275]=8'h39;
        bytes[276]=8'h32;
        bytes[277]=8'h4d;
        bytes[278]=8'h20;
        bytes[279]=8'h2f;
        bytes[280]=8'h20;
        bytes[281]=8'h36;
        bytes[282]=8'h34;
        bytes[283]=8'h4d;
        bytes[284]=8'h4d;
        // ID 18: "LIVE MAP        "
        bytes[288]=8'h4c;
        bytes[289]=8'h49;
        bytes[290]=8'h56;
        bytes[291]=8'h45;
        bytes[292]=8'h20;
        bytes[293]=8'h4d;
        bytes[294]=8'h41;
        bytes[295]=8'h50;
        // ID 19: "START           "
        bytes[304]=8'h53;
        bytes[305]=8'h54;
        bytes[306]=8'h41;
        bytes[307]=8'h52;
        bytes[308]=8'h54;
        // ID 20: "STOP            "
        bytes[320]=8'h53;
        bytes[321]=8'h54;
        bytes[322]=8'h4f;
        bytes[323]=8'h50;
        // ID 21: "VIEW MAP        "
        bytes[336]=8'h56;
        bytes[337]=8'h49;
        bytes[338]=8'h45;
        bytes[339]=8'h57;
        bytes[340]=8'h20;
        bytes[341]=8'h4d;
        bytes[342]=8'h41;
        bytes[343]=8'h50;
        // ID 22: "RADAR +90       "
        bytes[352]=8'h52;
        bytes[353]=8'h41;
        bytes[354]=8'h44;
        bytes[355]=8'h41;
        bytes[356]=8'h52;
        bytes[357]=8'h20;
        bytes[358]=8'h2b;
        bytes[359]=8'h39;
        bytes[360]=8'h30;
        // ID 23: "SLOW AND STOP   "
        bytes[368]=8'h53;
        bytes[369]=8'h4c;
        bytes[370]=8'h4f;
        bytes[371]=8'h57;
        bytes[372]=8'h20;
        bytes[373]=8'h41;
        bytes[374]=8'h4e;
        bytes[375]=8'h44;
        bytes[376]=8'h20;
        bytes[377]=8'h53;
        bytes[378]=8'h54;
        bytes[379]=8'h4f;
        bytes[380]=8'h50;
        // ID 24: "LIVE LIDAR      "
        bytes[384]=8'h4c;
        bytes[385]=8'h49;
        bytes[386]=8'h56;
        bytes[387]=8'h45;
        bytes[388]=8'h20;
        bytes[389]=8'h4c;
        bytes[390]=8'h49;
        bytes[391]=8'h44;
        bytes[392]=8'h41;
        bytes[393]=8'h52;
        // ID 25: "X -             "
        bytes[400]=8'h58;
        bytes[401]=8'h20;
        bytes[402]=8'h2d;
        // ID 26: "X +             "
        bytes[416]=8'h58;
        bytes[417]=8'h20;
        bytes[418]=8'h2b;
        // ID 27: "Y -             "
        bytes[432]=8'h59;
        bytes[433]=8'h20;
        bytes[434]=8'h2d;
        // ID 28: "Y +             "
        bytes[448]=8'h59;
        bytes[449]=8'h20;
        bytes[450]=8'h2b;
        // ID 29: "STEP            "
        bytes[464]=8'h53;
        bytes[465]=8'h54;
        bytes[466]=8'h45;
        bytes[467]=8'h50;
        // ID 30: "DONE            "
        bytes[480]=8'h44;
        bytes[481]=8'h4f;
        bytes[482]=8'h4e;
        bytes[483]=8'h45;
        // ID 31: "X FRONT Y LEFT  "
        bytes[496]=8'h58;
        bytes[497]=8'h20;
        bytes[498]=8'h46;
        bytes[499]=8'h52;
        bytes[500]=8'h4f;
        bytes[501]=8'h4e;
        bytes[502]=8'h54;
        bytes[503]=8'h20;
        bytes[504]=8'h59;
        bytes[505]=8'h20;
        bytes[506]=8'h4c;
        bytes[507]=8'h45;
        bytes[508]=8'h46;
        bytes[509]=8'h54;
        // ID 32: "MM / DEGREES    "
        bytes[512]=8'h4d;
        bytes[513]=8'h4d;
        bytes[514]=8'h20;
        bytes[515]=8'h2f;
        bytes[516]=8'h20;
        bytes[517]=8'h44;
        bytes[518]=8'h45;
        bytes[519]=8'h47;
        bytes[520]=8'h52;
        bytes[521]=8'h45;
        bytes[522]=8'h45;
        bytes[523]=8'h53;
        // ID 33: "FORWARD         "
        bytes[528]=8'h46;
        bytes[529]=8'h4f;
        bytes[530]=8'h52;
        bytes[531]=8'h57;
        bytes[532]=8'h41;
        bytes[533]=8'h52;
        bytes[534]=8'h44;
        // ID 34: "REVERSE         "
        bytes[544]=8'h52;
        bytes[545]=8'h45;
        bytes[546]=8'h56;
        bytes[547]=8'h45;
        bytes[548]=8'h52;
        bytes[549]=8'h53;
        bytes[550]=8'h45;
        // ID 35: "TURN LEFT       "
        bytes[560]=8'h54;
        bytes[561]=8'h55;
        bytes[562]=8'h52;
        bytes[563]=8'h4e;
        bytes[564]=8'h20;
        bytes[565]=8'h4c;
        bytes[566]=8'h45;
        bytes[567]=8'h46;
        bytes[568]=8'h54;
        // ID 36: "TURN RIGHT      "
        bytes[576]=8'h54;
        bytes[577]=8'h55;
        bytes[578]=8'h52;
        bytes[579]=8'h4e;
        bytes[580]=8'h20;
        bytes[581]=8'h52;
        bytes[582]=8'h49;
        bytes[583]=8'h47;
        bytes[584]=8'h48;
        bytes[585]=8'h54;
        // ID 37: "TURN 180        "
        bytes[592]=8'h54;
        bytes[593]=8'h55;
        bytes[594]=8'h52;
        bytes[595]=8'h4e;
        bytes[596]=8'h20;
        bytes[597]=8'h31;
        bytes[598]=8'h38;
        bytes[599]=8'h30;
        // ID 38: "CLICK TO MOVE   "
        bytes[608]=8'h43;
        bytes[609]=8'h4c;
        bytes[610]=8'h49;
        bytes[611]=8'h43;
        bytes[612]=8'h4b;
        bytes[613]=8'h20;
        bytes[614]=8'h54;
        bytes[615]=8'h4f;
        bytes[616]=8'h20;
        bytes[617]=8'h4d;
        bytes[618]=8'h4f;
        bytes[619]=8'h56;
        bytes[620]=8'h45;
        // ID 39: "STOP TO FINISH  "
        bytes[624]=8'h53;
        bytes[625]=8'h54;
        bytes[626]=8'h4f;
        bytes[627]=8'h50;
        bytes[628]=8'h20;
        bytes[629]=8'h54;
        bytes[630]=8'h4f;
        bytes[631]=8'h20;
        bytes[632]=8'h46;
        bytes[633]=8'h49;
        bytes[634]=8'h4e;
        bytes[635]=8'h49;
        bytes[636]=8'h53;
        bytes[637]=8'h48;
        // ID 40: "LEFT 90         "
        bytes[640]=8'h4c;
        bytes[641]=8'h45;
        bytes[642]=8'h46;
        bytes[643]=8'h54;
        bytes[644]=8'h20;
        bytes[645]=8'h39;
        bytes[646]=8'h30;
        // ID 41: "RIGHT 90        "
        bytes[656]=8'h52;
        bytes[657]=8'h49;
        bytes[658]=8'h47;
        bytes[659]=8'h48;
        bytes[660]=8'h54;
        bytes[661]=8'h20;
        bytes[662]=8'h39;
        bytes[663]=8'h30;
        // ID 42: "HERE IS ZERO    "
        bytes[672]=8'h48;
        bytes[673]=8'h45;
        bytes[674]=8'h52;
        bytes[675]=8'h45;
        bytes[676]=8'h20;
        bytes[677]=8'h49;
        bytes[678]=8'h53;
        bytes[679]=8'h20;
        bytes[680]=8'h5a;
        bytes[681]=8'h45;
        bytes[682]=8'h52;
        bytes[683]=8'h4f;
        // ID 43: "GO TO GOAL      "
        bytes[688]=8'h47;
        bytes[689]=8'h4f;
        bytes[690]=8'h20;
        bytes[691]=8'h54;
        bytes[692]=8'h4f;
        bytes[693]=8'h20;
        bytes[694]=8'h47;
        bytes[695]=8'h4f;
        bytes[696]=8'h41;
        bytes[697]=8'h4c;
        // ID 44: "SET GOAL        "
        bytes[704]=8'h53;
        bytes[705]=8'h45;
        bytes[706]=8'h54;
        bytes[707]=8'h20;
        bytes[708]=8'h47;
        bytes[709]=8'h4f;
        bytes[710]=8'h41;
        bytes[711]=8'h4c;
        // ID 45: "TRACK DOG       "
        bytes[720]=8'h54;
        bytes[721]=8'h52;
        bytes[722]=8'h41;
        bytes[723]=8'h43;
        bytes[724]=8'h4b;
        bytes[725]=8'h20;
        bytes[726]=8'h44;
        bytes[727]=8'h4f;
        bytes[728]=8'h47;
        // ID 46: "TRACK FACE      "
        bytes[736]=8'h54;
        bytes[737]=8'h52;
        bytes[738]=8'h41;
        bytes[739]=8'h43;
        bytes[740]=8'h4b;
        bytes[741]=8'h20;
        bytes[742]=8'h46;
        bytes[743]=8'h41;
        bytes[744]=8'h43;
        bytes[745]=8'h45;
        // ID 47: "STOP TRACK      "
        bytes[752]=8'h53;
        bytes[753]=8'h54;
        bytes[754]=8'h4f;
        bytes[755]=8'h50;
        bytes[756]=8'h20;
        bytes[757]=8'h54;
        bytes[758]=8'h52;
        bytes[759]=8'h41;
        bytes[760]=8'h43;
        bytes[761]=8'h4b;
        // ID 48: "POWER OFF PI    "
        bytes[768]=8'h50;
        bytes[769]=8'h4f;
        bytes[770]=8'h57;
        bytes[771]=8'h45;
        bytes[772]=8'h52;
        bytes[773]=8'h20;
        bytes[774]=8'h4f;
        bytes[775]=8'h46;
        bytes[776]=8'h46;
        bytes[777]=8'h20;
        bytes[778]=8'h50;
        bytes[779]=8'h49;
        // ID 49: "HEAD 135 MIN 90 "
        bytes[784]=8'h48;
        bytes[785]=8'h45;
        bytes[786]=8'h41;
        bytes[787]=8'h44;
        bytes[788]=8'h20;
        bytes[789]=8'h31;
        bytes[790]=8'h33;
        bytes[791]=8'h35;
        bytes[792]=8'h20;
        bytes[793]=8'h4d;
        bytes[794]=8'h49;
        bytes[795]=8'h4e;
        bytes[796]=8'h20;
        bytes[797]=8'h39;
        bytes[798]=8'h30;
        // ID 50: "CHOOSE A TARGET "
        bytes[800]=8'h43;
        bytes[801]=8'h48;
        bytes[802]=8'h4f;
        bytes[803]=8'h4f;
        bytes[804]=8'h53;
        bytes[805]=8'h45;
        bytes[806]=8'h20;
        bytes[807]=8'h41;
        bytes[808]=8'h20;
        bytes[809]=8'h54;
        bytes[810]=8'h41;
        bytes[811]=8'h52;
        bytes[812]=8'h47;
        bytes[813]=8'h45;
        bytes[814]=8'h54;
        // ID 51: "FIND FACE       "
        bytes[816]=8'h46;
        bytes[817]=8'h49;
        bytes[818]=8'h4e;
        bytes[819]=8'h44;
        bytes[820]=8'h20;
        bytes[821]=8'h46;
        bytes[822]=8'h41;
        bytes[823]=8'h43;
        bytes[824]=8'h45;
        // ID 52: "TURN TO FACE    "
        bytes[832]=8'h54;
        bytes[833]=8'h55;
        bytes[834]=8'h52;
        bytes[835]=8'h4e;
        bytes[836]=8'h20;
        bytes[837]=8'h54;
        bytes[838]=8'h4f;
        bytes[839]=8'h20;
        bytes[840]=8'h46;
        bytes[841]=8'h41;
        bytes[842]=8'h43;
        bytes[843]=8'h45;
        // ID 53: "GO NEAR FACE    "
        bytes[848]=8'h47;
        bytes[849]=8'h4f;
        bytes[850]=8'h20;
        bytes[851]=8'h4e;
        bytes[852]=8'h45;
        bytes[853]=8'h41;
        bytes[854]=8'h52;
        bytes[855]=8'h20;
        bytes[856]=8'h46;
        bytes[857]=8'h41;
        bytes[858]=8'h43;
        bytes[859]=8'h45;
        // ID 54: "SIDE FOLLOW     "
        bytes[864]=8'h53;
        bytes[865]=8'h49;
        bytes[866]=8'h44;
        bytes[867]=8'h45;
        bytes[868]=8'h20;
        bytes[869]=8'h46;
        bytes[870]=8'h4f;
        bytes[871]=8'h4c;
        bytes[872]=8'h4c;
        bytes[873]=8'h4f;
        bytes[874]=8'h57;
        // ID 55: "CAMERA FRONT    "
        bytes[880]=8'h43;
        bytes[881]=8'h41;
        bytes[882]=8'h4d;
        bytes[883]=8'h45;
        bytes[884]=8'h52;
        bytes[885]=8'h41;
        bytes[886]=8'h20;
        bytes[887]=8'h46;
        bytes[888]=8'h52;
        bytes[889]=8'h4f;
        bytes[890]=8'h4e;
        bytes[891]=8'h54;
        // ID 56: "LASER RANGE ONLY"
        bytes[896]=8'h4c;
        bytes[897]=8'h41;
        bytes[898]=8'h53;
        bytes[899]=8'h45;
        bytes[900]=8'h52;
        bytes[901]=8'h20;
        bytes[902]=8'h52;
        bytes[903]=8'h41;
        bytes[904]=8'h4e;
        bytes[905]=8'h47;
        bytes[906]=8'h45;
        bytes[907]=8'h20;
        bytes[908]=8'h4f;
        bytes[909]=8'h4e;
        bytes[910]=8'h4c;
        bytes[911]=8'h59;
        // ID 57: "FRONT REF FIRST "
        bytes[912]=8'h46;
        bytes[913]=8'h52;
        bytes[914]=8'h4f;
        bytes[915]=8'h4e;
        bytes[916]=8'h54;
        bytes[917]=8'h20;
        bytes[918]=8'h52;
        bytes[919]=8'h45;
        bytes[920]=8'h46;
        bytes[921]=8'h20;
        bytes[922]=8'h46;
        bytes[923]=8'h49;
        bytes[924]=8'h52;
        bytes[925]=8'h53;
        bytes[926]=8'h54;
        // ID 58: "FRONT SAMPLE    "
        bytes[928]=8'h46;
        bytes[929]=8'h52;
        bytes[930]=8'h4f;
        bytes[931]=8'h4e;
        bytes[932]=8'h54;
        bytes[933]=8'h20;
        bytes[934]=8'h53;
        bytes[935]=8'h41;
        bytes[936]=8'h4d;
        bytes[937]=8'h50;
        bytes[938]=8'h4c;
        bytes[939]=8'h45;
        // ID 59: "START FOLLOW    "
        bytes[944]=8'h53;
        bytes[945]=8'h54;
        bytes[946]=8'h41;
        bytes[947]=8'h52;
        bytes[948]=8'h54;
        bytes[949]=8'h20;
        bytes[950]=8'h46;
        bytes[951]=8'h4f;
        bytes[952]=8'h4c;
        bytes[953]=8'h4c;
        bytes[954]=8'h4f;
        bytes[955]=8'h57;
        // ID 60: "DOG AT CAR LEFT "
        bytes[960]=8'h44;
        bytes[961]=8'h4f;
        bytes[962]=8'h47;
        bytes[963]=8'h20;
        bytes[964]=8'h41;
        bytes[965]=8'h54;
        bytes[966]=8'h20;
        bytes[967]=8'h43;
        bytes[968]=8'h41;
        bytes[969]=8'h52;
        bytes[970]=8'h20;
        bytes[971]=8'h4c;
        bytes[972]=8'h45;
        bytes[973]=8'h46;
        bytes[974]=8'h54;
        // ID 61: "SAMPLE THEN GO  "
        bytes[976]=8'h53;
        bytes[977]=8'h41;
        bytes[978]=8'h4d;
        bytes[979]=8'h50;
        bytes[980]=8'h4c;
        bytes[981]=8'h45;
        bytes[982]=8'h20;
        bytes[983]=8'h54;
        bytes[984]=8'h48;
        bytes[985]=8'h45;
        bytes[986]=8'h4e;
        bytes[987]=8'h20;
        bytes[988]=8'h47;
        bytes[989]=8'h4f;
        // ID 62: "GLOBAL 8.192M   "
        bytes[992]=8'h47;
        bytes[993]=8'h4c;
        bytes[994]=8'h4f;
        bytes[995]=8'h42;
        bytes[996]=8'h41;
        bytes[997]=8'h4c;
        bytes[998]=8'h20;
        bytes[999]=8'h38;
        bytes[1000]=8'h2e;
        bytes[1001]=8'h31;
        bytes[1002]=8'h39;
        bytes[1003]=8'h32;
        bytes[1004]=8'h4d;
        // ID 63: "LOCAL 4.096M    "
        bytes[1008]=8'h4c;
        bytes[1009]=8'h4f;
        bytes[1010]=8'h43;
        bytes[1011]=8'h41;
        bytes[1012]=8'h4c;
        bytes[1013]=8'h20;
        bytes[1014]=8'h34;
        bytes[1015]=8'h2e;
        bytes[1016]=8'h30;
        bytes[1017]=8'h39;
        bytes[1018]=8'h36;
        bytes[1019]=8'h4d;
        // ID 64: "64MM / CELL     "
        bytes[1024]=8'h36;
        bytes[1025]=8'h34;
        bytes[1026]=8'h4d;
        bytes[1027]=8'h4d;
        bytes[1028]=8'h20;
        bytes[1029]=8'h2f;
        bytes[1030]=8'h20;
        bytes[1031]=8'h43;
        bytes[1032]=8'h45;
        bytes[1033]=8'h4c;
        bytes[1034]=8'h4c;
        // ID 65: "4M FIELD        "
        bytes[1040]=8'h34;
        bytes[1041]=8'h4d;
        bytes[1042]=8'h20;
        bytes[1043]=8'h46;
        bytes[1044]=8'h49;
        bytes[1045]=8'h45;
        bytes[1046]=8'h4c;
        bytes[1047]=8'h44;
        // ID 66: "FIELD CENTER    "
        bytes[1056]=8'h46;
        bytes[1057]=8'h49;
        bytes[1058]=8'h45;
        bytes[1059]=8'h4c;
        bytes[1060]=8'h44;
        bytes[1061]=8'h20;
        bytes[1062]=8'h43;
        bytes[1063]=8'h45;
        bytes[1064]=8'h4e;
        bytes[1065]=8'h54;
        bytes[1066]=8'h45;
        bytes[1067]=8'h52;
        // ID 67: "CAR CENTER      "
        bytes[1072]=8'h43;
        bytes[1073]=8'h41;
        bytes[1074]=8'h52;
        bytes[1075]=8'h20;
        bytes[1076]=8'h43;
        bytes[1077]=8'h45;
        bytes[1078]=8'h4e;
        bytes[1079]=8'h54;
        bytes[1080]=8'h45;
        bytes[1081]=8'h52;
        // ID 68: "CURRENT LEG     "
        bytes[1088]=8'h43;
        bytes[1089]=8'h55;
        bytes[1090]=8'h52;
        bytes[1091]=8'h52;
        bytes[1092]=8'h45;
        bytes[1093]=8'h4e;
        bytes[1094]=8'h54;
        bytes[1095]=8'h20;
        bytes[1096]=8'h4c;
        bytes[1097]=8'h45;
        bytes[1098]=8'h47;
        // ID 69: "1.024M          "
        bytes[1104]=8'h31;
        bytes[1105]=8'h2e;
        bytes[1106]=8'h30;
        bytes[1107]=8'h32;
        bytes[1108]=8'h34;
        bytes[1109]=8'h4d;
    end
    always @(posedge clk)character<=bytes[{label_id,character_index}];
endmodule
