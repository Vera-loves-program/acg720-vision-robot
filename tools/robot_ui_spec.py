#!/usr/bin/env python3
"""Shared robot view/button geometry and deterministic RTL/preview source.

Only explicit --write-rtl edits the panel. No HDL tools, network or hardware
are used. The offline preview contains illustrative text, never measurements.
"""
from dataclasses import dataclass
from pathlib import Path
import argparse
import html
import json
import re

ROOT = Path(__file__).resolve().parents[1]
MENU_RECTS = tuple((left, top, left + 352, top + 80)
                   for top in (128, 240, 352) for left in (32, 432))
LEAF_RECTS = tuple((left, top, left + 188, top + 60)
                   for top in (128, 216, 304) for left in (408, 608))
MAP_RECTS = ((32, 472, 160, 512), (160, 472, 288, 512),
             (32, 416, 288, 452))
MAP_LABELS = ('GLOBAL 8.192M', 'LOCAL 4.096M', '64MM / CELL', '4M FIELD',
              'FIELD CENTER', 'CAR CENTER', 'CURRENT LEG', '1.024M')
SIDEBAR = {100: ((832, 60, 1008, 116), 'HOME'),
           101: ((832, 132, 1008, 188), 'BACK'),
           105: ((832, 400, 1008, 444), 'CLEAR STOP'),
           106: ((832, 480, 1008, 536), 'ALL STOP')}
MANUAL_EXTRA = {7: (32, 432, 384, 492)}
NAV_EXTRA = dict(MANUAL_EXTRA)
FOLLOW_EXTRA = {7: (32, 432, 384, 492)}
DASHBOARD_RECTS = {
    1: (32, 452, 384, 516), 2: (432, 452, 784, 516),
}
DASHBOARD_ROWS = ((130, 2, 11), (184, 3, 12), (242, 4, 13),
                  (314, 5, 14), (386, 6, 15))


@dataclass(frozen=True)
class View:
    title: str
    kind: str
    captions: tuple
    hint: str = 'STOP BEFORE EDIT'
    detail: str = 'ONE TAP PER STEP'
    # Source 1=robot, 2=PID; the six row identities stay within 256 bytes.
    rows: tuple = tuple((1, row) for row in range(2, 8))


VIEWS = {
    (0, 0): View('SIX TASK DEMOS', 'menu', ('1 CAMERA + IMU', '2 MOTOR CONTROL', '3 LIDAR AVOID', '4 XY NAVIGATION', '5 GIMBAL TRACK', '6 FIND FACE'), 'SELECT A TASK', 'BACK TO HOME'),
    (0, 1): View('CAMERA', 'camera', (), 'LOCAL FPGA VIDEO'),
    (2, 1): View('LIDAR MAP', 'map', ('BACK', 'CLEAR MAP', 'FIELD / CAR'), '8.192M / 64MM', 'LIVE MAP'),
    (2, 3): View('3 LIDAR AVOID', 'leaf', ('START', 'STOP', 'VIEW MAP', 'RADAR +90'), 'SLOW AND STOP', 'LIVE LIDAR'),
    (3, 3): View('GOAL X Y MM', 'leaf', ('X -', 'X +', 'Y -', 'Y +', 'STEP', 'DONE'), 'X FRONT Y LEFT', 'MM / DEGREES'),
    (4, 0): View('2 MOTOR CONTROL', 'leaf', ('FORWARD', 'REVERSE', 'TURN LEFT', 'TURN RIGHT', 'TURN 180', 'STOP'), 'CLICK TO MOVE', 'STOP TO FINISH'),
    (4, 1): View('4 XY NAVIGATION', 'leaf', ('LEFT 90', 'RIGHT 90', 'HERE IS ZERO', 'GO TO GOAL', 'SET GOAL', 'STOP'), 'X FRONT Y LEFT', 'MM / DEGREES'),
    (5, 0): View('5 GIMBAL TRACK', 'leaf', ('TRACK DOG', 'TRACK FACE', 'STOP TRACK', '', '', '', 'POWER OFF PI'), 'HEAD 135 MIN 90', 'CHOOSE A TARGET'),
    (5, 3): View('6 FIND FACE', 'leaf', ('FIND FACE', 'TURN TO FACE', 'GO NEAR FACE', 'SIDE FOLLOW', 'CAMERA FRONT', 'STOP', 'POWER OFF PI'), 'LASER RANGE ONLY', 'FRONT REF FIRST'),
    (5, 4): View('DOG SIDE FOLLOW', 'leaf', ('TRACK DOG', 'FRONT SAMPLE', 'START FOLLOW', 'STOP', '', '', 'POWER OFF PI'), 'DOG AT CAR LEFT', 'SAMPLE THEN GO'),
}


def labels():
    result = []
    for _, caption in SIDEBAR.values():
        if caption not in result:
            result.append(caption)
    for view in VIEWS.values():
        for text in (*view.captions, view.hint, view.detail):
            if text and text not in result:
                result.append(text)
    result.extend(text for text in MAP_LABELS if text not in result)
    assert len(result) < 128
    assert all(len(text) <= 16 and text.isascii() for text in result)
    return result


LABELS = labels()
LABEL_ID = {text: index + 1 for index, text in enumerate(LABELS)}


def button_rects(page, subpage):
    view = VIEWS.get((page, subpage))
    result = {button: rect for button, (rect, _) in SIDEBAR.items()}
    if view is None or view.kind == 'camera':
        return result
    if view.kind == 'dashboard':
        result.update(DASHBOARD_RECTS)
    else:
        rects = MENU_RECTS if view.kind == 'menu' else MAP_RECTS if view.kind == 'map' else LEAF_RECTS
        result.update({index + 1: rect for index, rect in enumerate(rects[:len(view.captions)]) if view.captions[index]})
    if (page, subpage) == (4, 0):
        pass
    elif (page, subpage) == (4, 1):
        pass
    elif (page, subpage) in ((5, 0), (5,3), (5,4)):
        result.update(FOLLOW_EXTRA)
    return {b: rect for b,rect in result.items() if b>=100 or (b<=len(view.captions) and view.captions[b-1])}


def hit_test(x, y, page, subpage):
    for button, (left, top, right, bottom) in button_rects(page, subpage).items():
        if left <= x < right and top <= y < bottom:
            return button
    return 0


def caption_for(page, subpage, button):
    if button in SIDEBAR:
        return SIDEBAR[button][1]
    view = VIEWS.get((page, subpage))
    return view.captions[button - 1] if view and 1 <= button <= len(view.captions) else ''


def button_label(rect, kind, button):
    left, top, right, bottom = rect
    padding = 16 if kind == 'menu' and button < 100 else 8
    return left + padding, top + (bottom - top - 16) // 2, right - 4


def _view_constants():
    lines = []
    for kind, value in (('menu', 0), ('leaf', 1), ('camera', 2), ('map', 3), ('dashboard', 4)):
        cases = ','.join(f"6'd{page * 8 + sub}" for (page, sub), view in VIEWS.items() if view.kind == kind)
        if cases: lines.append(f"            {cases}:view_kind=3'd{value};")
    return '\n'.join(lines)


def _caption_function():
    lines = ['    function [6:0] view_caption;', '        input [5:0] view;',
             '        input [3:0] slot;', '        begin', '            view_caption=0;',
             '            case(view)']
    for (page, sub), view in VIEWS.items():
        if not view.captions:
            continue
        lines.append(f"                6'd{page * 8 + sub}:case(slot)")
        for slot, caption in enumerate(view.captions, 1):
            if not caption: continue
            lines.append(f"                    4'd{slot}:view_caption=7'd{LABEL_ID[caption]};")
        lines += ['                    default:view_caption=0;', '                endcase']
    lines += ['                default:view_caption=0;', '            endcase', '        end', '    endfunction']
    return '\n'.join(lines)


def _hint_function(which):
    name = 'view_hint' if which == 'hint' else 'view_detail'
    lines = [f'    function [6:0] {name};', '        input [5:0] view;',
             '        begin case(view)']
    for (page, sub), view in VIEWS.items():
        text = getattr(view, which)
        lines.append(f"            6'd{page * 8 + sub}:{name}=7'd{LABEL_ID[text]};")
    lines += [f"            default:{name}=7'd0;", '        endcase end', '    endfunction']
    return '\n'.join(lines)


def _rect_case(rects, indent='                '):
    return '\n'.join(f"{indent}if(rect(x,y,11'd{left},11'd{right},10'd{top},10'd{bottom}))button=8'd{slot};"
                     for slot, (left, top, right, bottom) in rects.items())


def generate_hit_test():
    sidebar = _rect_case({key: value[0] for key, value in SIDEBAR.items()}, '        ')
    menu = _rect_case(dict(enumerate(MENU_RECTS, 1)))
    leaf = _rect_case(dict(enumerate(LEAF_RECTS, 1)))
    map_rects = _rect_case(dict(enumerate(MAP_RECTS, 1)))
    dashboard = _rect_case(DASHBOARD_RECTS)
    return f'''// This exact hit module is shared by the LCD and both touch contacts.
module robot_ui_hit_test (
    input wire [10:0] x,
    input wire [9:0] y,
    input wire [2:0] page,subpage,
    output reg [7:0] button
);
    wire [5:0] view={{page,subpage}};
    reg [2:0] view_kind;
    always @* begin
        case(view)
{_view_constants()}
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
{menu}
            end
            3'd1:begin
{leaf}
            end
            3'd3:begin
{map_rects}
            end
            3'd4:begin
{dashboard}
            end
            default:begin end
        endcase
        if(view==6'd40 || view==6'd43 || view==6'd44)begin
{_rect_case(FOLLOW_EXTRA, '            ')}
        end
        // Only visible core actions receive a touch ID.
{''.join("        if(view==6'd"+str(p*8+q)+" && button>0 && button<100 && !("+' || '.join('button=='+str(i) for i,t in enumerate(v.captions,1) if t)+"))button=0;"+chr(10) for (p,q),v in VIEWS.items() if v.captions)}
{sidebar}
    end
endmodule
'''


def generate_rom():
    lines = ['// 128 x16 ASCII captions, one synchronous character-stage ROM.',
             'module robot_label_rom (', '    input wire clk,',
             '    input wire [6:0] label_id,', '    input wire [3:0] character_index,',
             '    output reg [7:0] character', ');',
             '    (* syn_romstyle="block_rom" *) reg [7:0] bytes[0:2047];',
             '    integer i;', '    initial begin',
             '        // Gowin limits each initialization loop to 2000 iterations.',
             '        // Fill both halves in 1024 iterations; labels override below.',
             '        for(i=0;i<1024;i=i+1)begin',
             "            bytes[i]=8'h20;",
             "            bytes[i+1024]=8'h20;",
             '        end']
    for label, text in enumerate(LABELS, 1):
        lines.append(f'        // ID {label}: "{text:<16}"')
        for column, value in enumerate(text.encode('ascii')):
            lines.append(f"        bytes[{label * 16 + column}]=8'h{value:02x};")
    lines += ['    end', '    always @(posedge clk)character<=bytes[{label_id,character_index}];',
              'endmodule', '']
    return '\n'.join(lines)


def generate_panel():
    panel = PANEL_TEMPLATE.replace('@CAPTIONS@', _caption_function()).replace(
        '@HINT@', _hint_function('hint')).replace('@DETAIL@', _hint_function('detail')).replace(
        '@VIEW_KINDS@', _view_constants()).replace('@HOME@', str(LABEL_ID['HOME'])).replace(
        '@BACK@', str(LABEL_ID['BACK'])).replace('@CLEAR@', str(LABEL_ID['CLEAR STOP'])).replace(
        '@STOP@', str(LABEL_ID['ALL STOP'])).replace('@SETTINGS@', '0').replace('@SCALE@', str(LABEL_ID['8.192M / 64MM']))
    for token, caption in zip(('GLOBAL', 'LOCAL', 'CELL', 'FIELD', 'FIELD_CENTER', 'CAR_CENTER', 'LEG', 'SCALE'), MAP_LABELS):
        panel = panel.replace('@MAP_' + token + '@', str(LABEL_ID[caption]))
    return panel + '\n' + generate_hit_test() + '\n' + generate_rom()


PANEL_TEMPLATE = '''// Generated from tools/robot_ui_spec.py; edit the shared specification.
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
@VIEW_KINDS@
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
@CAPTIONS@
@HINT@
@DETAIL@
    // Fixed text anchors avoid variable subtract/add/centering hardware.
    // Fill and touch both use the SINGLE shared hit-test module above.
    always @* begin
        button_text_x=0;button_label_right=0;button_text_y=0;button_caption=0;
        if(hit_button>=100)begin
            button_text_x=840;button_label_right=1004;
            case(hit_button)
                100:begin button_text_y=80;button_caption=7'd@HOME@;end
                101:begin button_text_y=152;button_caption=7'd@BACK@;end
                105:begin button_text_y=414;button_caption=7'd@CLEAR@;end
                106:begin button_text_y=500;button_caption=7'd@STOP@;end
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
            if(x>=32 && x<288)begin label_x=32;label_right=288;label_literal=7'd@MAP_GLOBAL@;end
            else if(x>=416 && x<800)begin label_x=416;label_right=800;label_literal=7'd@MAP_LOCAL@;end
            else label_enable=0;
        end
        if(view_kind==3 && x>=32 && x<288 && y>=376 && y<392)begin
            label_enable=1;label_literal=7'd@MAP_CELL@;label_x=32;label_right=288;label_y=376;label_color=MUTED;
        end
        if(view_kind==3 && x>=80 && x<288 && y>=396 && y<412)begin
            label_enable=1;label_literal=7'd@MAP_SCALE@;label_x=80;label_right=288;label_y=396;label_color=MUTED;
        end
        if(view_kind==3 && y>=512 && y<528)begin
            label_enable=1;label_y=512;
            if(x>=416 && x<608)begin label_x=416;label_right=608;label_literal=map_follow_car ? 7'd@MAP_CAR_CENTER@ : 7'd@MAP_FIELD_CENTER@;end
            else if(x>=620 && x<800)begin label_x=620;label_right=800;label_literal=7'd@MAP_LEG@;label_color=16'h781f;end
            else label_enable=0;
        end
        if(view_kind==3 && x>=32 && x<288 && y>=516 && y<532)begin
            label_enable=1;label_x=32;label_right=288;label_y=516;label_literal=7'd@MAP_FIELD@;label_color=16'hfd20;
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
'''


def route_for(view, slot):
    if slot==100:return (0,0)
    if slot==101:
        return (4,1) if view==(3,3) else (2,3) if view==(2,1) else (5,3) if view==(5,4) else (0,0)
    if view==(0,0):return {1:(0,1),2:(4,0),3:(2,3),4:(4,1),5:(5,0),6:(5,3)}.get(slot)
    return {((4,1),5):(3,3),((3,3),6):(4,1),((2,3),3):(2,1),((2,1),1):(2,3),((5,3),4):(5,4)}.get((view,slot))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--write-rtl', action='store_true')
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--html', type=Path)
    args = parser.parse_args()
    if args.write_rtl:
        (ROOT / 'src/robot_control_panel.v').write_text(generate_panel(), encoding='utf-8')
    if args.check:
        assert (ROOT / 'src/robot_control_panel.v').read_text(encoding='utf-8-sig') == generate_panel()
        print(f'PASS: {len(VIEWS)} hierarchical views; {len(LABELS)} captions; shared hit-test/panel/ROM source')
    if args.html:
        from preview_robot_ui import write_preview
        write_preview(args.html)


if __name__ == '__main__':
    main()
