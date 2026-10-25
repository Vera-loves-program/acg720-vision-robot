#!/usr/bin/env python3
"""Shared hierarchical UI/source and independent byte/pixel arithmetic checks.

No HDL is executed. The 18-view geometry intentionally replaces the old
five flat pages. The four-stage pixel/byte-RAM and camera/map timing contracts
remain unchanged; pure Python checks are not synthesis or hardware proof.
"""
from pathlib import Path
import random
import re
import tempfile

from check_motor_integration import uncomment, check_blocks
from robot_ui_spec import (VIEWS, LABELS, LABEL_ID, SIDEBAR, MENU_RECTS,
    LEAF_RECTS, MAP_RECTS, button_rects, button_label, caption_for,
    generate_panel, hit_test, route_for, DASHBOARD_RECTS, DASHBOARD_ROWS)
from preview_robot_ui import write_preview

ROOT=Path(__file__).resolve().parents[1]
panel=(ROOT/'src/robot_control_panel.v').read_text(encoding='utf-8-sig')
lcd=(ROOT/'src/lcd1024_interaction_ui.v').read_text(encoding='utf-8-sig')
panel_code=re.sub(r'\s+','',uncomment(panel))
lcd_code=re.sub(r'\s+','',uncomment(lcd))
assert panel==generate_panel(), 'Regenerate RTL after changing the shared specification'
assert len(VIEWS)==21 and len(LABELS)<128
EXPECTED_LABELS=[text.ljust(16) for text in LABELS]

# Freeze the approved action order independently of the generator's RTL code.
assert VIEWS[0,0].captions==('WHEELS','LIDAR','POSE SETUP','DRIVE','CAMERA','FOLLOW')
assert VIEWS[1,0].captions==('ENCODER SETUP','SPEED PID VIEW','POSITION VIEW','DRIVE TEST')
assert VIEWS[1,1].captions==('WHEEL L/R','ENC SIGN','MOTOR SIGN','ARM TEST','JOG FORWARD','JOG REVERSE')
assert route_for((1,0),4)==(4,0)
assert route_for((1,0),5) is None and route_for((1,0),6) is None
assert not any(key in VIEWS for key in ((1,4),(1,6),(1,7)))
assert set(button_rects(1,0))=={1,2,3,4,100,101,105,106}
for key in ((1,2),(1,3)):
    assert VIEWS[key].kind=='dashboard'
    assert VIEWS[key].captions==('DRIVE TEST','STOP BOTH')
    assert set(button_rects(*key))=={1,2,100,101,105,106}
    assert route_for(key,1)==(4,0) and route_for(key,2) is None
    for slot in range(3,100):
        assert route_for(key,slot) is None
assert VIEWS[4,3].captions==('PREV FIELD','NEXT FIELD','VALUE -','VALUE +','STEP','BACK NAV')
assert route_for((4,1),8)==(4,3) and route_for((4,3),6)==(4,1)
assert MENU_RECTS==((32,128,384,208),(432,128,784,208),(32,240,384,320),(432,240,784,320),(32,352,384,432),(432,352,784,432))
assert LEAF_RECTS==((408,128,596,188),(608,128,796,188),(408,216,596,276),(608,216,796,276),(408,304,596,364),(608,304,796,364))
assert MAP_RECTS==((440,352,620,396),(620,352,800,396),(440,408,620,452),(620,408,800,452))
assert set(SIDEBAR)=={100,101,105,106}
assert tuple(v[0] for v in SIDEBAR.values())==((832,60,1008,116),(832,132,1008,188),(832,400,1008,444),(832,480,1008,536))

# Every approved view is reachable through the actual preview navigation.
# Retired parameter editors have no route or hidden touch target.
reachable={(0,0)}
pending=[(0,0)]
while pending:
    key=pending.pop()
    for button in button_rects(*key):
        target=route_for(key,button)
        if target is not None:
            assert target in VIEWS
            if target not in reachable:
                reachable.add(target);pending.append(target)
assert reachable==set(VIEWS)
assert all(key not in reachable for key in ((1,4),(1,6),(1,7)))

# Extract the actual hit-test rectangle branches and decode its view cases.
helper=panel_code.split('modulerobot_ui_hit_test(',1)[1].split('endmodule',1)[0]
view_kinds={}
for keys,kind in re.findall(r"((?:6'd\d+,?)+):view_kind=3'd(\d);",helper):
    for number in re.findall(r"6'd(\d+)",keys):
        assert int(number) not in view_kinds
        view_kinds[int(number)]=int(kind)
case_body=helper.split('case(view_kind)',1)[1].split('endcase',1)[0]
def extract_rects(text):
    return [(int(button),(int(left),int(top),int(right),int(bottom)))
        for left,right,top,bottom,button in re.findall(
        r"if\(rect\(x,y,11'd(\d+),11'd(\d+),10'd(\d+),10'd(\d+)\)\)button=8'd(\d+);",text)]
actual_groups={}
for kind,body in re.findall(r"3'd(\d):begin(.*?)end",case_body,re.S):
    actual_groups[int(kind)]=extract_rects(body)
assert actual_groups[0]==list(enumerate(MENU_RECTS,1))
assert actual_groups[1]==list(enumerate(LEAF_RECTS,1))
assert actual_groups[3]==list(enumerate(MAP_RECTS,1))
assert actual_groups[4]==list(DASHBOARD_RECTS.items())
actual_sidebar=extract_rects(helper.rsplit("if(view==6'd8",1)[1])
assert actual_sidebar==[(b,r) for b,(r,_) in SIDEBAR.items()]

def actual_hit(x,y,page,sub):
    view=page*8+sub
    candidates=list(actual_groups.get(view_kinds.get(view,5),[]))
    if view==32:candidates += [(7,(32,432,192,492)),(8,(208,432,384,492))]
    if view==33:candidates += [(7,(32,432,192,492)),(8,(208,432,384,492))]
    if view==40:candidates += [(7,(32,432,384,492))]
    if view==8:candidates=[(b,r) for b,r in candidates if b not in (5,6)]
    candidates += actual_sidebar
    found=0
    for button,(left,top,right,bottom) in candidates:
        if left<=x<right and top<=y<bottom:found=button
    return found

rng=random.Random(20261024)
hit_cases=0
for page in range(8):
    for sub in range(8):
        key=page,sub
        rectangles=button_rects(*key)
        for button,rect in rectangles.items():
            left,top,right,bottom=rect
            for x in (left-1,left,left+1,right-1,right):
                for y in (top-1,top,top+1,bottom-1,bottom):
                    assert actual_hit(x,y,*key)==hit_test(x,y,*key)
                    hit_cases+=1
            for other,other_rect in rectangles.items():
                if button>=other:continue
                l,t,r,b=other_rect
                assert not(left<r and l<right and top<b and t<bottom),(key,button,other)
            if key in VIEWS:
                x,y,limit=button_label(rect,VIEWS[key].kind,button)
                text=caption_for(*key,button)
                assert x+12*len(text)<=limit,(key,button,text)
                assert top<=y and y+16<=bottom
        for _ in range(500):
            x,y=rng.randrange(2048),rng.randrange(1024)
            assert actual_hit(x,y,*key)==hit_test(x,y,*key)
            hit_cases+=1

# Caption lookup selects seven bits only; no 128-bit row mux was reintroduced.
actual_captions={}
caption_fn=panel_code.split('function[6:0]view_caption;',1)[1].split('endfunction',1)[0]
for view,body in re.findall(r"6'd(\d+):case\(slot\)(.*?)endcase",caption_fn):
    for slot,label in re.findall(r"4'd(\d+):view_caption=7'd(\d+);",body):
        actual_captions[int(view),int(slot)]=int(label)
for (page,sub),view in VIEWS.items():
    for slot,text in enumerate(view.captions,1):
        assert actual_captions[page*8+sub,slot]==LABEL_ID[text]
assert 'label_text' not in panel_code
for module in ('robot_control_panel','robot_ui_hit_test','robot_label_rom'):
    text=re.search(r'\bmodule\s+'+module+r'\b(.*?)\bendmodule\b',uncomment(panel),re.S)
    check_blocks('module '+module+text[1]+'endmodule',module)
assert 'wire[2:0]page=flags[2:0],subpage=flags[30:28];' in panel_code
assert 'assignmap_read_en=page==2&&subpage==1&&map_prefetch;' in panel_code
assert '.page(page),.subpage(subpage),.button(hit_button)' in panel_code

# Both six-line regions are checked for every 10-bit y value.
body_scanlines=map_scanlines=0
for y in range(1024):
    actual=132<=y<308 and 4<=(y&31)<20
    expected=any(132+32*n<=y<148+32*n for n in range(6))
    assert actual==expected
    if actual:
        assert 0<=((y-132)>>5)<6
        body_scanlines+=1
    actual_map=80<=y<256 and (y&31)>=16
    expected_map=any(80+32*n<=y<96+32*n for n in range(6))
    assert actual_map==expected_map
    if actual_map:
        assert 0<=((y-80)>>5)<6
        map_scanlines+=1
assert body_scanlines==map_scanlines==96
assert "view==6'd14" not in panel_code and "view==6'd15" not in panel_code

# Every dashboard value is read-only, including targets and PID gains.
# All ten values use existing source-1 rows, keeping both wheels visible in
# one committed 256-byte snapshot instead of capturing another wide bus.
assert DASHBOARD_ROWS==((130,2,11),(184,3,12),(242,4,13),(314,5,14),(386,6,15))
for y,left_row,right_row in DASHBOARD_ROWS:
    assert f'if(y>={y}&&y<{y+16})beginlabel_enable=1;label_y={y};label_row=x>=432?{right_row}:{left_row};' in panel_code
    assert left_row not in (1,8,9,10) and right_row not in (1,8,9,10)
for key in ((1,2),(1,3)):
    for x in (48,448):
        for y,_,_ in DASHBOARD_ROWS:
            assert hit_test(x,y,*key)==0,'read-only PID value must not open an editor'
    assert hit_test(48,486,*key)==1 and hit_test(448,486,*key)==2
assert "((view==6'd10||view==6'd11)&&hit_button==2)" in panel_code
assert "if(view==6'd8&&(button==5||button==6))button=0;" in panel_code
assert 'label_source=2;label_row=x>=432?12:11;label_x=x>=432?444:44;label_right=x>=432?780:380;label_y=428;' in panel_code

# The full 128-label synchronous ROM has explicit SPACE defaults. Model the
# actual initialization addresses rather than assuming every byte is filled.
# Gowin EX3934 rejected the former single 2048-iteration loop (limit2000).
def blank_rom(code):
    source=code.split('modulerobot_label_rom(',1)[1].split('endmodule',1)[0]
    count=re.search(r'for\(i=0;i<(\d+);i=i\+1\)',source)
    assert count is not None,'missing ROM initializer'
    bound=int(count.group(1))
    assert 0<bound<=2000,'ROM initialization exceeds Gowin loop count2000'
    body=re.search(r'for\(i=0;i<\d+;i=i\+1\)begin(.*?)end',source)
    assert body is not None,'ROM initializer must group both half writes'
    writes=re.findall(r"bytes\[i(?:\+(\d+))?\]=8'h([0-9a-fA-F]+);",body.group(1))
    assert len(writes)==2,'both ROM halves must be initialized'
    initialized=[None]*2048
    for i in range(bound):
        for offset,value in writes:
            address=i+int(offset or 0)
            assert 0<=address<2048,'ROM initialization out of bounds'
            assert initialized[address] is None,'ROM initialization overlaps'
            initialized[address]=int(value,16)
    assert initialized==[32]*2048,'ROM initialization leaves holes or non-SPACE bytes'
    return initialized

rom=blank_rom(panel_code)
for bad in (
    panel_code.replace('for(i=0;i<1024;i=i+1)','for(i=0;i<2048;i=i+1)',1),
    panel_code.replace('for(i=0;i<1024;i=i+1)','for(i=0;i<512;i=i+1)',1),
    panel_code.replace('bytes[i+1024]','bytes[i+1023]',1),
    panel_code.replace('bytes[i+1024]','bytes[i+1025]',1),
    panel_code.replace("bytes[i+1024]=8'h20;",'',1),
    panel_code.replace("bytes[i+1024]=8'h20;","bytes[i+1024]=8'h00;",1),
):
    assert bad!=panel_code
    try:blank_rom(bad)
    except AssertionError:pass
    else:raise AssertionError('broken ROM initializer escaped the source/coverage guard')
for address,value in re.findall(r"bytes\[(\d+)\]=8'h([0-9a-fA-F]+);",panel):
    rom[int(address)]=int(value,16)
for label in range(128):
    expected=LABELS[label-1].ljust(16).encode() if 1<=label<=len(LABELS) else b' '*16
    assert bytes(rom[label*16:label*16+16])==expected
    for col in range(16):assert ((label<<4)|col)<2048
assert "for(i=0;i<1024;i=i+1)beginbytes[i]=8'h20;bytes[i+1024]=8'h20;end" in panel_code
assert 'always@(posedgeclk)character<=bytes[{label_id,character_index}];' in panel_code
assert 'reg[7:0]bytes[0:2047];' in panel_code
assert 'wire[6:0]robot_label_literal;' in lcd_code
assert 'reg[6:0]robot_label_literal_l=0;' in lcd_code

# Existing whole-row versus byte-address and glyph arithmetic.

def old_byte(packed,row,col,nrows):
    word=(packed>>((nrows-1-row)*128))&((1<<128)-1)
    return (word>>((15-col)*8))&255

def new_byte(packed,row,col,source):
    if source in (1,2):msb=((~row&15)<<7)|((~col&15)<<3)|7
    elif source==3:msb=((~row&7)<<7)|((~col&15)<<3)|7
    else:msb=((4-row)<<7)|((~col&15)<<3)|7
    return (packed>>(msb-7))&255

rng=random.Random(20261023)
byte_cases=0
pixel_examples=[]
for frame in range(64):
    for source,nrows in ((1,16),(2,16),(3,8),(4,5)):
        data=bytes(rng.choice(b"ABCXYZ0123456789+-/% ") for _ in range(nrows*16))
        packed=int.from_bytes(data,"big")
        for row in range(nrows):
            for col in range(16):
                assert old_byte(packed,row,col,nrows)==new_byte(packed,row,col,source)
                byte_cases+=1
        if frame<4:
            row=rng.randrange(nrows)
            pixel_examples.append((data[row*16:row*16+16],lambda col,p=packed,r=row,s=source:new_byte(p,r,col,s)))
for label,text in enumerate(EXPECTED_LABELS,1):
    pixel_examples.append((text.encode(),lambda col,l=label:rom[l*16+col]))
# The former disabled/default row was NUL-filled. Its glyph is blank, exactly
# as the explicit space-filled descriptor/unused ROM defaults are now.
pixel_examples.append((bytes(16),lambda col:rom[col]))

# Compare all 192x16 local pixels for every caption and seeded dynamic rows.
# This evaluates integer glyph geometry, not HDL cycles or a display model.
glyphs={ord(c):int(bits,16) for c,bits in re.findall(r'"(.)":glyph=40\'h([0-9a-fA-F]+);',lcd)}
assert glyphs[ord("A")]==0x7e1111117e

def ink(character,x,y):
    gx=(x%12)>>1
    gy=y>>1
    return gx<5 and gy<7 and ((glyphs.get(character,0)>>(32-gx*8+gy))&1)!=0

pixel_cases=0
for reference,get_character in pixel_examples:
    for y in range(16):
        for x in range(192):
            assert ink(reference[x//12],x,y)==ink(get_character(x//12),x,y)
            pixel_cases+=1

# Existing pixel bundle, legacy and fetch contracts.

for edge in (
    "robot_control_l<=robot_control;", "robot_label_source_l<=robot_label_source;",
    "robot_label_row_l<=robot_label_row;", "robot_label_literal_l<=robot_label_literal;",
    "character_a<=robot_control_l?robot_dynamic_character:character;",
    "robot_constant_a<=robot_control_l&&robot_label_source_l==3'd0;",
    "robot_byte_source_a<=BYTE_TEXT_MODE&&robot_control_l?(robot_label_source_l==3'd1?2'd1:robot_label_source_l==3'd2?2'd2:2'd0):2'd0;",
    "font_b<=glyph(glyph_character_a);", "glyph_x_a<=glyph_x_now;", "glyph_y_a<=glyph_y_now;",
    "glyph_x_b<=glyph_x_a;", "glyph_y_b<=glyph_y_a;", "de_l<=active;", "de_a<=de_l;",
    "de_b<=de_a;", "de<=de_b;", "hs_a<=hs_l;", "hs_b<=hs_a;", "hs<=hs_b;",
    "vs_a<=vs_l;", "vs_b<=vs_a;", "vs<=vs_b;", "base_color_l<=base_color;",
    "base_color_a<=base_color_l;", "base_color_b<=base_color_a;",
    "rgb<=text_pixel?label_color_b:base_color_b;",
):
    assert edge in lcd_code,edge
assert "glyph_character_a=robot_constant_a?robot_constant_character_a:glyph_dynamic_character_a;" in lcd_code
assert "glyph_dynamic_character_a=robot_byte_source_a==2'd1?robot_byte_character:robot_byte_source_a==2'd2?pid_byte_character:character_a;" in lcd_code
assert 'robot_byte_source_a<=0;' in lcd_code
assert 'assignbyte_text_read_index={robot_label_row_l,char_ix};' in lcd_code
assert "assignrobot_byte_read_en=BYTE_TEXT_MODE&&run&&in_label_l&&robot_control_l&&robot_label_source_l==3'd1;" in lcd_code
assert "assignpid_byte_read_en=BYTE_TEXT_MODE&&run&&in_label_l&&robot_control_l&&robot_label_source_l==3'd2;" in lcd_code
assert ".label_id(robot_label_literal_l),.character_index(char_ix)" in lcd_code
assert "char_ix=label_dx_l/11'd12;" in lcd_code
assert "character=label_text_l[127-(char_ix*8)-:8];" in lcd_code
assert "if(actuator_test)beginlabel_text=test_label_text;" in lcd_code
assert "if(robot_control)beginlabel_text=128'd0;" in lcd_code
assert "wire[10:0]robot_byte_msb={~robot_label_row_l,~char_ix,3'b111};" in lcd_code
assert "wire[9:0]sensor_byte_msb={~robot_label_row_l[2:0],~char_ix,3'b111};" in lcd_code
assert "wire[9:0]battery_byte_msb={3'd4-robot_label_row_l[2:0],~char_ix,3'b111};" in lcd_code
assert "map_prefetch=x>=27&&x<411&&y>=104&&y<488;" in panel_code
assert "map_x_wide=(x-11'd27)/11'd3;" in panel_code
assert "map_y_wide=(10'd487-y)/10'd3;" in panel_code
assert "frame_restart=run&&h==H_TOTAL-1'b1&&v==V_TOTAL-1'b1;" in lcd_code
assert ".camera_pop(camera_pop),.display_pixel(display_pixel)" in lcd_code

with tempfile.TemporaryDirectory(prefix='robot-ui-check-',dir=ROOT/'tools') as temp:
    output=Path(temp)/'preview.html'
    write_preview(output)
    document=output.read_text(encoding='utf-8')
    assert document.count('<section data-view=')==21
    assert 'START BOTH' not in document and 'MAX PWM' not in document
    assert 'EDIT PID VALUE' not in document and 'PID TEST LOOP' not in document
    assert 'READ ONLY LCD' in document and 'VOFA PID EDIT' in document
    assert '绝非实测' in document and '不连接或控制小车' in document
    assert 'fetch(' not in document and 'WebSocket(' not in document
    assert '<script src=' not in document
print(f'PASS: 21 hierarchical views; {hit_cases} shared-hit boundary/random points; read-only PID views and metadata FOLLOW')
print(f'PASS: {len(LABELS)} captions/2048 ROM bytes, six relevant rows per leaf, seven-bit descriptor and same four-stage pixel pipeline')
print(f'PASS: {byte_cases} seeded dynamic character comparisons; {pixel_cases} glyph pixels; camera/map/legacy guards retained')
print('NO HDL compilation/simulation, synthesis resource measurement, timing closure or board validation')
