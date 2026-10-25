"""Display-only pose, eight-sector geometry and stable mailbox contracts.

No HDL compilation/simulation or board access. Rendering is an illustrative
software model; actual FPGA resource/timing validation remains separate.
"""
from pathlib import Path
from random import Random
from preview_robot_ui import car_cell_color
from robot_ui_spec import generate_panel

ROOT=Path(__file__).resolve().parents[1]

def packet(x,y,heading,extents=(232,118,119,119),local=False):
    direction=((heading+1440)//2880)%8
    valid=local or (-4096<=x<4096 and -4096<=y<4096)
    fields=(int(valid),0 if local else direction,*((v+63)//64 for v in extents),
            64 if local else ((x+4096)//64)&127,
            64 if local else ((y+4096)//64)&127,0)
    result=0
    for value,width in zip(fields,(1,3,5,5,5,5,7,7,2)):
        assert 0<=value<(1<<width)
        result=(result<<width)|value
    assert result.bit_length()<=40
    return result

positions=0
for x in (-4097,-4096,-2048,-1,0,1,2000,4000,4095,4096):
    for y in (-4097,-4096,-1,0,4000,4095,4096):
        for heading in range(0,23040,64):
            p=packet(x,y,heading)
            assert bool(p>>39)==(-4096<=x<4096 and -4096<=y<4096)
            if p>>39:
                assert ((p>>9)&127)==(x+4096)//64
                assert ((p>>2)&127)==(y+4096)//64
            assert (packet(x,y,heading,local=True)>>2)&16383==(64<<7)|64
            positions+=1

# Cardinal heading goldens: +X right, +Y up; origin and front direction remain
# meaningful even with an asymmetric body relative to the axle midpoint.
for direction,(dx,dy) in enumerate(((6,0),(0,6),(-6,0),(0,-6))):
    assert car_cell_color(dx,dy,direction*2)=='#ffff00'
    assert car_cell_color(0,0,direction*2)=='RED'
    assert car_cell_color(-dx,-dy,direction*2) is None
pixels=0
for direction in range(8):
    visible={(x,y):car_cell_color(x,y,direction) for x in range(-30,31) for y in range(-30,31)}
    assert visible[(0,0)]=='RED'
    assert any(c=='BLUE' for c in visible.values()) and any(c=='#ffff00' for c in visible.values())
    assert all(c is None for (x,y),c in visible.items() if abs(x)>26 or abs(y)>26)
    pixels+=len(visible)

# Bundled-data producer never overwrites a word awaiting ACK, even while live
# pose updates keep arriving. Delayed pixel consumption sees an entire word.
rng=Random(71);held=0;request=ack=0;pixel=0;sent=[];received=[]
for cycle in range(10000):
    live=packet(rng.randrange(-4000,4000),rng.randrange(-4000,4000),rng.randrange(23040))
    before=held
    if request==ack and cycle%7==0:
        held=live;request^=1;sent.append(held)
    elif request!=ack:assert held==before
    if cycle%19==0 and request!=ack:
        pixel=held;ack=request;received.append(pixel)
assert received==sent[:len(received)]

sub=(ROOT/'src/robot_subsystem.v').read_text(encoding='utf-8-sig')
pose=sub.split('module robot_map_car_pose(',1)[1]
assert 'request==ack_sync[2]' in pose and 'request_sync[2]!=ack && pixel_idle' in pose
assert 'pixel_model<=held_model;ack<=request_sync[2];' in pose
assert 'map_we' not in pose and 'cells[' not in pose
panel=(ROOT/'src/robot_control_panel.v').read_text(encoding='utf-8-sig')
assert panel==generate_panel()
assert 'if(view_kind==3 && in_map && map_car_model[39])' in panel
assert 'map_car_model[38:36]' in panel and 'if(car_origin)base_color=RED;' in panel
for name in ('vision_robot_ui_top','lcd1024_interaction_ui'):
    assert '.map_car_model(map_car_model)' in (ROOT/f'src/{name}.v').read_text(encoding='utf-8-sig')
print(f'PASS: {positions} map position/heading packets; {pixels} overlay cells; cardinal/diagonal arrows; 10000 stalled mailbox events.')
print('Display-only additions/comparisons, separate atomic pose mailbox; occupancy RAM unchanged. No HDL simulation or timing claim.')
