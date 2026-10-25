#!/usr/bin/env python3
"""Independent formatter row/byte transaction checks; never runs HDL tools.

Motor row expressions retain their pre-stream fingerprints, including spaces
INSIDE string literals. Robot text deliberately changes to21 semantic views;
independent ASCII/32-bit decimal references cover those meanings and boundaries.
A transaction model checks byte order, backpressure, frame publication and reset.
This is not a clock-level HDL simulation.
"""
from copy import deepcopy
from pathlib import Path
import random
import re

ROOT = Path(__file__).resolve().parents[1]
MASK32 = (1 << 32) - 1
MOTOR_FIELD_LABELS = ('FIELD KP        ', 'FIELD KI        ', 'FIELD KD        ', 'FIELD TARGET    ',
                     'FIELD START PWM ', 'FIELD MAX PWM   ', 'FIELD SPEED DB  ', 'FIELD POS DB    ',
                     'FIELD ENC DIR   ', 'FIELD MOTOR DIR ')


def canonical(text):
    return ''.join(re.findall(r'"(?:\\.|[^"\\])*"|[^\s]', text))


def fnv1a(text):
    value = 2166136261
    for char in text:
        value = ((value ^ ord(char)) * 16777619) & MASK32
    return f'{value:08x}'


def source_checks():
    sources = {}
    for filename, rowfield in (('motor_status_text.v', 'field'), ('robot_status_text.v', 'rowfield')):
        source = (ROOT / 'src' / filename).read_text(encoding='utf-8')
        bare = re.sub(r'//[^\n]*|/\*.*?\*/', '', source, flags=re.S)
        if filename == 'motor_status_text.v':
            # These are the complete legacy-format contracts. The active
            # three-row robot branch has its own exhaustive numeric/stream
            # model in check_motor_compact_text.py; do not mistake the two
            # mutually exclusive COMMIT sites for a duplicated publication.
            assert 'parameter integer COMPACT_ROBOT=0' in bare
            header = bare.split('    generate if(COMPACT_ROBOT!=0)', 1)[0]
            legacy = bare.split('end else begin:full_formatter', 1)[1].rsplit('end endgenerate', 1)[0]
            bare = header + legacy + '\nendmodule\n'
        sources[filename] = bare
        for token in ('output wire text_byte_valid', 'output wire [7:0] text_byte_data',
                      'output wire [7:0] text_byte_index', 'input wire text_byte_ready',
                      'assign text_byte_valid=reset_n && state==EMIT',
                      f'assign text_byte_index={{{rowfield},byte_in_row}}',
                      'EMIT:if(text_byte_ready)begin', 'if(byte_in_row==15)begin',
                      f'if({rowfield}==15)state<=COMMIT',
                      'COMMIT:begin'):
            assert token in bare, (filename, token)
        # No backpressure-dependent valid and no complete-frame byte selector.
        valid = re.search(r'assign text_byte_valid=([^;]+);', bare)[1]
        assert 'ready' not in valid and not re.search(r'text_rows\s*\[[^]]+\]\s*;', bare)
        # rows[i] in the existing constant-bound COMMIT loop is a static
        # elaboration-time read, not a runtime row read for serialization.
        reads = re.findall(r'(?:=|<=)\s*rows\[([^]]+)\]', bare)
        allowed = {'i'} if rowfield == 'field' else {'i', '0', '1', '8', '9', '10', '11', '14', '15'}
        assert set(reads) <= allowed, (filename, reads)
        emit = bare.split('EMIT:if(text_byte_ready)begin', 1)[1].split('COMMIT:begin', 1)[0]
        assert 'text_valid<=' not in emit and 'text_rows[' not in emit
        assert not re.search(r'\b(?:decimal|bcd|values|negative|signs)\s*(?:\[[^]]+\])?\s*<=', emit)
        assert len(re.findall(r'\btext_valid<=1\b', bare)) == 1
        assert re.search(r'COMMIT:begin[\s\S]*?text_valid<=1', bare)
        # Keep the old fixed per-row LCD assignments: a dynamic128-bit
        # formatted_row write would destroy each row's constant-bit folding.
        writes = re.findall(r'\brows\[([^]]+)\]\s*<=', bare)
        assert all(index.isdigit() or index == 'i' for index in writes), (filename, writes)
        assert not re.search(r'\brows\[[^]]+\]\s*<=\s*formatted_row', bare)
    motor = sources['motor_status_text.v']
    for token in ('assign text_byte_data=emit_shift[127:120]',
                  'emit_shift<=formatted_row;byte_in_row<=0;state<=EMIT',
                  "emit_shift<={emit_shift[119:0],8'd0};byte_in_row<=byte_in_row+1'b1"):
        assert token in motor
    field_function = re.search(r'function \[127:0\] field_label;(.*?)endfunction',motor,re.S)[1]
    field_labels = {int(index): label for index,label in re.findall(r'(\d+):field_label="([^"]*)"',field_function)}
    assert field_labels == dict(enumerate(MOTOR_FIELD_LABELS)), 'motor field0..9 label/reference mismatch'
    assert 'default:field_label="FIELD INVALID   ";' in field_function
    assert all(len(label)==16 for label in MOTOR_FIELD_LABELS)
    body = re.search(r'formatted_row=0;\s+case\(field\)([\s\S]*?)endcase', motor)[1]
    # Fingerprints were computed from the unchanged pre-stream STORE case,
    # replacing only rows[constant]<= with formatted_row=.
    assert fnv1a(canonical(body)) == '98e00416', 'motor row expression/spacing changed'
    motor_lcd = motor.split('EMIT:if(text_byte_ready)begin', 1)[1]
    motor_lcd = re.search(r'case\(field\)([\s\S]*?)endcase', motor_lcd)[1]
    motor_lcd = re.sub(r'rows\[\d+\]\s*<=', 'formatted_row=', motor_lcd)
    assert canonical(motor_lcd) == canonical(body), 'fixed LCD writes differ from emitted motor rows'
    robot = sources['robot_status_text.v']
    robot_full = (ROOT/'src/robot_status_text.v').read_text(encoding='utf-8')
    labels={name:label for name,_,label in re.findall(
        r'localparam \[7:0\] (TEXT\d+)=8\'d(\d+); // "([^"\n]*)"',robot_full)}
    # Expand only for semantic checks. Physical RTL uses eight-bit IDs and a
    # byte-wide synchronousROM, never these128-bit constant words as signals.
    semantic=re.sub(r'\bTEXT\d+\b',lambda match:'"'+labels[match[0]]+'"',robot)
    # Legacy function fingerprints predate the ROM conversion. Dashboard and
    # diagnostic captures were intentionally revised for ten slots/two hex
    # overlays and are independently checked by robot_rows/event references.
    # Current IDs include the FOLLOW views, body fields and countdown notices.
    semantic_functions = dict(field_name='8839049e',view_title='7311348a',
        navigation_label='5f30c53e',notice_label='e43b448a',unit_hint='5980e06f',
        menu_line='bf7c2702',decimal_text='6429da06')
    for name, expected in semantic_functions.items():
        function=re.search(r'function \[[^\]]+\] '+name+r';(.*?)endfunction',semantic,re.S)[1]
        assert fnv1a(canonical(function)) == expected, (name,'pre-ROM semantics changed')
    fixed_capture='rows[0]<='+semantic.split('row_ids[0]<=',1)[1].split('held_score<=match_score;',1)[0]
    fixed_capture=fixed_capture.replace('row_ids[','rows[')
    # Partial wheel gains must not be described as both outputs being zero.
    # Dashboard/editor hints are informational, never run gates; the PID
    # editor does not display unrelated geometry/gyro/C1 requirements.
    assert fnv1a(canonical(fixed_capture)) == '53704e13', 'map/FOLLOW/sidebar snapshot changed'
    numeric_capture=robot.split('for(i=0;i<6;i=i+1)values[i]<=0;',1)[1].split('field<=0;rowfield<=0;state<=LOAD;',1)[0]
    assert fnv1a(canonical(numeric_capture)) == '403621ca', 'shared ten-slot numeric capture changed'
    for capture in ('values[6]<=map_points;values[7]<=map_dropped;signs[6]<=0;',
                    'values[3]<={16\'d0,follow_cap};values[4]<={16\'d0,follow_pan_us};values[5]<={16\'d0,follow_tilt_us};'):
        assert capture in robot, capture
    assert 'input wire [2:0] page,subpage' in robot
    assert 'input wire motor_armed,motor_running,motor_selected_wheel,motor_mode_position' in robot
    assert 'row_ids[0]<=view==17 ? (map_local ? "LOCAL LIVE SCAN " : map_world_ready ? "WORLD MAP       " : "WORLD MAP HOLD  ") :' in semantic
    assert 'view==15 ? (motor_mode_position ? "POSITION TEST   " : "SPEED LOOP TEST ") : view_title(view);' in semantic
    assert 'reg [31:0] values[0:9];' in robot and 'reg [9:0] signs,fractional;' in robot
    assert 'wire [87:0] number=fractional[field] ? fixed4_text(bcd) : decimal_text(bcd,signs[field]);' in robot
    assert 'reg [87:0] emit_number;' in robot
    assert 'assign text_byte_data=emit_numeric && byte_in_row>=5 ? emit_number[87:80] : decorated_byte;' in robot
    assert 'STORE:begin emit_number<=number;byte_in_row<=0;state<=EMIT;end' in robot
    assert "if(emit_numeric && byte_in_row>=5)emit_number<={emit_number[79:0],8'd0};" in robot
    assert 'formatted_id="VALUE           ";formatted_numeric=1;' in semantic
    assert 'formatted_id="STEP            ";formatted_numeric=1;' in semantic
    assert 'wire held_pid_view=held_view==10 || held_view==11;' in robot
    assert 'if(held_pid_view ? ((rowfield>=2 && rowfield<=6) || rowfield>=11) :' in robot
    assert 'if(held_pid_view ? ((rowfield>=2 && rowfield<=6) || (rowfield>=11 && rowfield<=14)) :' in robot
    assert 'shift<=selected_conversion;bcd<=0;bits<=0;state<=CONVERT' in robot
    assert 'wire [25:0] gain_product=selected_raw[15:0]*26\'d625;' in robot
    assert 'wire [31:0] selected_conversion=fractional[field] ? {10\'d0,gain_product[25:4]} : selected_magnitude;' in robot
    assert 'fractional<=10\'b1110011100;' in robot
    assert 'if(!held_pid_view && held_view!=16 && rowfield==14)' in robot
    assert 'if(!held_pid_view && held_view!=16 && rowfield==15)' in robot
    for row,label in ((11,'H0'),(12,'H1'),(13,'S0'),(14,'S1'),(15,'TX')):
        assert f'{row}:if(held_view==16)begin formatted_id="{label} 00000000     ";formatted_numeric=0;end' in semantic
    assert robot.count('else if(!held_pid_view)begin formatted_id=')==5
    assert 'held_debug_status<=lidar_debug_status;held_last_rx_word<=lidar_last_rx_word;' in robot
    assert 'held_reply_h0<=lidar_reply_capture[127:96];held_reply_h1<=lidar_reply_capture[95:64];' in robot
    assert 'held_reply_s0<=lidar_reply_capture[63:32];held_reply_s1<=lidar_reply_capture[31:0];' in robot
    assert 'held_last_tx_word<=lidar_last_tx_word;' in robot
    c1_sidebar=semantic.split('row_ids[9]<=',1)[1].split('row_ids[10]<=',1)[0]
    assert c1_sidebar.index('lidar_error==8\'h15 ? "C1 STILL RX     "') < c1_sidebar.index('!scan_enable ? "C1 REQ OFF      "')
    assert 'if(bits==31)state<=STORE' in robot
    assert 'held_flags<={gains_ready,subpage,lidar_error[4:0],nav_fault,nav_status,map_ready,safety_stop,scan_enable,' in robot
    assert 'wire [5:0] held_view={held_flags[2:0],held_flags[30:28]};' in robot
    assert 'flags<=held_flags;text_valid<=1;state<=IDLE' in robot
    assert len(re.findall(r'adjusted=add3\(bcd\)', robot)) == 1
    assert 'digits10' not in robot and 'bcd[23:20]' not in robot
    assert 'if(!seen && negative)decimal_text[(k+1)*8+:8]="-";' in robot
    # Each named128-bit constant function must supply exactly16 visible bytes.
    for name in ('field_name','view_title','navigation_label','notice_label','unit_hint','menu_line'):
        function = re.search(r'function \[7:0\] '+name+r';(.*?)endfunction',semantic,re.S)[1]
        assert all(len(value)==16 for value in re.findall(r'"([^"\n]*)"',function)), name
    title_function = re.search(r'function \[7:0\] view_title;(.*?)endfunction',semantic,re.S)[1]
    titles = {int(index): text for index,text in re.findall(r'(\d+):view_title="([^"]*)"',title_function)}
    assert titles.items() >= ROBOT_TITLES.items()
    prefix_part = semantic.split('prefix_id="                ";',1)[1].split('body_id=prefix_id;body_numeric=1;',1)[0]
    # Numeric emission consumes five prefix bytes; a full field-name label
    # may also serve as a prefix (GAIN X100 / MAX COUNT). It stays byte-ROM.
    assert all(len(value)==16 and value.isascii() for value in re.findall(r'"([^"\n]*)"',prefix_part))
    old_prefix=re.sub(r'"([^"\n]*)"',lambda match:'"'+match[1][:5]+'"',prefix_part).replace('prefix_id','number_label')
    assert fnv1a(canonical(old_prefix)) == '3788ca6b', 'dashboard/FOLLOW numeric labels changed'
    assert 5*8 + 11*8 == 128
    # IDLE holds inputs once; EMIT never resamples live values or flags.
    emit = robot.split('EMIT:if(text_byte_ready)begin',1)[1].split('COMMIT:begin',1)[0]
    assert not re.search(r'\b(?:values|row_ids|held_\w+)(?:\[[^]]+\])?\s*<=',emit)
    assert all(name not in emit for name in ('rx_bytes','node_count','manual_speed_left','motor_selected_wheel'))
    assert 'front_mm<50 || front_mm>16000' in robot
    assert 'left_mm<50 || left_mm>16000' in robot
    assert 'right_mm<50 || right_mm>16000' in robot
    assert 'body_id="FRONT UNKNOWN   ";body_numeric=0;' in semantic
    assert 'body_id="LEFT UNKNOWN    ";body_numeric=0;' in semantic and 'body_id="RIGHT UNKNOWN   ";body_numeric=0;' in semantic
    assert 'view==9 ? (calibration_enable ? "CAL JOG ACTIVE  " : armed ? "CAL TEST ARMED  " : "CAL DISARMED    ")' in semantic
    assert '(view==33 || manual_turn_active) && nav_fault!=0 ? "NAV FAULT       "' in semantic
    # Exact full-width producer captures complement the independent numeric
    # model: especially keep all six diagnostic counters unsigned32, not slices.
    diagnostic=re.search(r'16:begin(.*?)end',robot,re.S)[1]
    assert canonical(diagnostic)==canonical('''
        values[0]<=rx_bytes;values[1]<=node_count;values[2]<=lidar_points;
        values[3]<=uart_errors;values[4]<=packet_errors;values[5]<=restart_count;
    ''')
    for token in ('values[6]<=selected_value;values[7]<=selected_step;',
                  'values[3]<=motor_selected_wheel ? speed_right : speed_left;',
                  'values[4]<=motor_selected_wheel ? position_right : position_left;',
                  "signs[5:0]<={duty_right[15],duty_left[15],position_right[31],position_left[31],speed_right[31],speed_left[31]};",
                  "35:begin values[3]<={16'd0,cruise_counts};values[4]<={16'd0,turn_counts};end"):
        assert token in robot,token
    combinational=robot.split('        prefix_id=',1)[1].split('    always @(posedge',1)[0]
    for live_name in ('page','subpage','motor_selected_wheel','speed_left','speed_right','position_left','position_right',
                      'selected_value','selected_step','lidar_error','nav_status','front_mm','left_mm','right_mm',
                      'cruise_counts','turn_counts','rx_bytes','map_points','manual_active','gyro_raw',
                      'motor_target_left','motor_target_right','speed_kp_left','speed_kp_right',
                      'speed_ki_left','speed_ki_right','speed_kd_left','speed_kd_right',
                      'motor_position_target_left','motor_position_target_right',
                      'position_kp_left','position_kp_right','position_ki_left','position_ki_right',
                      'position_kd_left','position_kd_right','lidar_reply_capture','lidar_last_tx_word',
                      'lidar_debug_status','lidar_last_rx_word'):
        assert not re.search(r'\b'+live_name+r'\b',combinational),('live input would break atomic stream',live_name)
    assert len(re.findall(r'\bbegin\b',robot)) == len(re.findall(r'\bend\b',robot))
    assert len(re.findall(r'\bcase\s*\(',robot)) == len(re.findall(r'\bendcase\b',robot))


def magnitude(value, width=32):
    value &= (1 << width) - 1
    return ((~value + 1) & ((1 << width) - 1)) if value >> (width - 1) else value


def negative(value, width=32):
    return bool((value & ((1 << width) - 1)) >> (width - 1))


def tail(value, count):
    return f'{value:012d}'[-count:]


def motor_rows(sample):
    state = ('MOTOR E-STOP    ' if sample['stop'] else 'MOTOR LINK FAULT' if sample['link'] else
             'MOTOR FAULT     ' if sample['fault_l'] or sample['fault_r'] else
             'MOTOR RUN       ' if sample['running'] else 'MOTOR ARMED     ' if sample['armed'] else 'MOTOR DISARMED  ')
    names = MOTOR_FIELD_LABELS
    f = sample['field']
    scope = ('EDIT BOTH SHARED' if 4 <= f <= 7 else
             'EDIT R POSITION ' if sample['wheel'] and sample['edit_position'] else
             'EDIT R SPEED    ' if sample['wheel'] else
             'EDIT L POSITION ' if sample['edit_position'] else 'EDIT L SPEED    ')
    mode = ('MODE POSITION   ' if sample['mode_position'] else
            'MODE SPEED OPEN ' if sample['unlocked'] else 'MODE SPEED LOCK ')
    value, step = magnitude(sample['value']), sample['step'] & MASK32
    if f < 3:
        value, step = value * 10000 // 256, step * 10000 // 256
        v, s = tail(value, 7), tail(step, 7)
        selected_row = f'VAL {v[:3]}.{v[3:]}    '
        step_row = f'STEP {s[:3]}.{s[3:]}   '
    else:
        sign = '-' if negative(sample['value']) and value else '+'
        selected_row = f'VAL {sign}{tail(value, 10)} '
        step_row = f'STEP {tail(step, 5)}      '
    rows = [state, scope, names[f] if f < len(names) else 'FIELD INVALID   ', selected_row, step_row]
    for label, key, width in (('L TGT ', 'target_l', 16), ('L SPD ', 'speed_l', 32),
                              ('R TGT ', 'target_r', 16), ('R SPD ', 'speed_r', 32),
                              ('L POS ', 'position_l', 32), ('R POS ', 'position_r', 32)):
        value = magnitude(sample[key], width)
        sign = '-' if negative(sample[key], width) and value else '+'
        digits = 'OVERFLOW' if value > 99999999 else tail(value, 8)
        rows.append(f'{label}{sign}{digits} ')
    for label, key in (('L PWM ', 'duty_l'), ('R PWM ', 'duty_r')):
        value = magnitude(sample[key], 16)
        sign = '-' if negative(sample[key], 16) and value else '+'
        rows.append(f'{label}{sign}{tail(value, 4)}/1000')
    fault = sample['fault_l'] * 1000 + sample['fault_r']
    bad = sample['invalid_l'] * 100000 + sample['invalid_r']
    rows += [f'FAULT L {tail(fault, 6)[:3]} R{tail(fault, 3)}',
             f'BAD L{tail(bad, 10)[:5]}R{tail(bad, 5)}', mode]
    return [row.encode('ascii') for row in rows]


ROBOT_TITLES = {
    0:'ROBOT HOME      ',1:'FPGA CAMERA     ',8:'MOTOR SETUP     ',9:'ENCODER / MOTOR ',
    10:'SPEED PID       ',11:'POSITION PID    ',12:'MOTOR LIMITS    ',14:'PID VALUE EDIT  ',15:'WHEEL TEST      ',
    16:'C1 DIAGNOSTICS  ',17:'LIDAR MAP       ',18:'LIDAR INSTALL   ',24:'NAV CALIBRATION ',
    25:'WHEEL GEOMETRY  ',26:'CURRENT POSE    ',27:'GOAL POSITION   ',28:'GYRO CALIBRATION',
    32:'MANUAL DRIVE    ',33:'POINT NAVIGATION',34:'MANUAL SPEEDS   ',35:'NAV SPEED SETUP ',
}
FIELD_NAMES = ('SET CURRENT X MM','SET CURRENT Y MM','SET YAW MDEG    ',
    'SET GOAL X MM   ','SET GOAL Y MM   ','WHEEL CIRC MM   ','ENC COUNT / REV ',
    'WHEEL TRACK MM  ','CRUISE COUNT/10M','TURN COUNT/10MS ','LIDAR ZERO DEG  ',
    'GYRO Z INVERT   ','LIDAR X OFF MM  ','LIDAR Y OFF MM  ')
NAV_LABELS = ('LOCAL ARMED     ','KEEP STILL GYRO ','CALIBRATE FIRST ','MANUAL DRIVING  ',
    'TURNING         ','GOING TO TARGET ','ARRIVED         ','OBSTACLE STOP   ',
    'SENSOR UNKNOWN  ','NAV FAULT       ','PLANNING        ','NO SAFE PATH    ')
NOTICE_LABELS = {1:'ARM FIRST       ',2:'TUNE BOTH GAINS ',3:'GYRO ZERO FIRST ',
                 4:'SET GEOMETRY    ',5:'CHECK LIDAR     '}
ROBOT_DEFAULTS = dict(page=0,subpage=0,field=0,value=0,step=1,x=0,y=0,heading=0,goal_x=0,goal_y=0,
    set_x=0,set_y=0,set_heading=0,points=0,score=0,baseline=0,margin=0,front=65535,left=65535,right=65535,
    geometry=False,gyro=False,stop=False,calibrating=False,gains=False,armed=False,nav_fault=0,nav_status=0,
    lidar_error=0,lidar_ready=False,scan_enable=False,map_ready=False,map_local=True,map_world_ready=False,notice=0,motor_fault=0,
    motor_armed=False,motor_running=False,motor_mode_position=False,calibration_enable=False,manual_active=False,manual_turn_active=False,
    manual_linked=False,wheel=False,enc_l=False,enc_r=False,drive_l=False,drive_r=False,gyro_invert=False,
    speed_l=0,speed_r=0,position_l=0,position_r=0,duty_l=0,duty_r=0,gyro_raw=0,
    set_l=20,set_r=20,circ=0,cpr=0,track=0,cruise=0,turn=0,lidar_zero=0,xoff=0,yoff=0,
    rx=0,node=0,uerr=0,perr=0,restarts=0,map_points=0,scans=0,map_dropped=0,rear=65535,self_points=0,
    motor_target_l=0,motor_target_r=0,kp_l=0,ki_l=0,kd_l=0,kp_r=0,ki_r=0,kd_r=0,
    pos_target_l=0,pos_target_r=0,pkp_l=0,pki_l=0,pkd_l=0,pkp_r=0,pki_r=0,pkd_r=0,
    lidar_debug_status=0,lidar_last_rx_word=0,lidar_reply_capture=0,lidar_last_tx_word=0)


def robot_flags(sample):
    s = {**ROBOT_DEFAULTS, **sample}
    return ((int(s['gains'])<<31) | ((s['subpage']&7)<<28) | ((s['lidar_error']&31)<<23)
            | ((s['nav_fault']&255)<<15) | ((s['nav_status']&15)<<11) | (int(s['map_ready'])<<10)
            | (int(s['stop'])<<9) | (int(s['scan_enable'])<<8) | (int(s['lidar_ready'])<<7)
            | (int(s['calibrating'])<<6) | (int(s['gyro'])<<5) | (int(s['geometry'])<<4)
            | (int(s['armed'])<<3) | (s['page']&7))


def robot_rows(sample):
    s = {**ROBOT_DEFAULTS, **sample}
    view = (s['page']&7)*8 + (s['subpage']&7)
    nav = NAV_LABELS[s['nav_status']] if s['nav_status'] < len(NAV_LABELS) else NAV_LABELS[0]
    field_name = FIELD_NAMES[min(s['field'],13)]

    def num(label,key,width=32,signed=True):
        value = s[key] & ((1<<width)-1)
        if signed and value & (1<<(width-1)):
            value -= 1<<width
        return f'{label}{value:11d}'

    def constnum(label,value):
        return f'{label}{int(value):11d}'

    if s['stop']: state='ALL STOP LATCHED'
    elif s['motor_fault']: state='MOTOR FAULT     '
    elif view==17:
        state=('C1 REQ OFF      ' if not s['scan_enable'] else 'C1 WAIT SCAN    ' if not s['scans'] else
               'LOCAL LIVE SCAN ' if s['map_local'] else 'WORLD MAP       ' if s['map_world_ready'] else 'WORLD MAP HOLD  ')
    elif (view==33 or s['manual_turn_active']) and s['nav_fault']: state='NAV FAULT       '
    elif s['notice']: state=NOTICE_LABELS.get(s['notice'],'CHECK SETTINGS  ')
    elif view==9:
        state=('CAL JOG ACTIVE  ' if s['calibration_enable'] else 'CAL TEST ARMED  ' if s['armed'] else 'CAL DISARMED    ')
    elif s['page']==1:
        state=('CAL JOG ACTIVE  ' if s['calibration_enable'] and s['motor_running'] else
               'MOTOR RUNNING   ' if s['motor_running'] else
               'MOTOR ARMED     ' if s['motor_armed'] else 'MOTOR DISARMED  ')
    elif s['calibrating']: state='KEEP STILL GYRO '
    elif view==32:
        state=(nav if s['manual_turn_active'] else 'RAW MANUAL RUN  ' if s['manual_active'] else
               'ARRIVED         ' if s['nav_status']==6 else
               'MANUAL ARMED    ' if s['armed'] else 'LOCAL DISARMED  ')
    elif view==34: state='SET SPEED ONLY  '
    else: state=nav if s['armed'] else 'LOCAL DISARMED  '

    if s['lidar_error']==0x15: c1='C1 STILL RX     '
    elif not s['scan_enable']: c1='C1 REQ OFF      '
    elif not s['rx']: c1='C1 NO RX        '
    elif s['lidar_error']==0x13: c1='C1 STALE        '
    elif s['lidar_ready']: c1='C1 LIVE         '
    elif s['lidar_error'] in (2,3,0x10): c1='C1 WAIT HEALTH  '
    elif s['lidar_error'] in (4,0x11): c1='C1 WAIT SCAN    '
    elif s['lidar_error'] in (0x12,0x14): c1='C1 HEALTH ERROR '
    elif s['lidar_error']==1: c1='C1 STARTING     '
    else: c1='C1 WAIT NODE    '

    hints={0:'CHOOSE ONE STEP ',1:'FPGA CAMERA LCD ',9:'CNT/10MS CNT PWM',14:'VALUE / STEP    ',
        15:'SET/VEL CNT/10MS',16:'COUNTERS UINT32 ',17:'RANGE MM / PTS  ',18:'DEG / OFFSET MM ',
        25:'MM, COUNTS/REV  ',26:'MM / YAW MDEG   ',27:'MM / YAW MDEG   ',28:'YAW MDEG Z RAW  ',
        32:'SET/VEL CNT/10MS',33:'XY MM YAW MDEG  ',34:'SET/VEL CNT/10MS',35:'BACK TO NAV     '}
    body=[' '*16]*6
    menus={
        0:['1 MOTOR DIR/PID ','2 LIDAR RX/MAP  ','3 POSE/GEOMETRY ','4 MANUAL/NAV    ','CAMERA ON FPGA  ','STOP AT ANY TIME'],
        1:['LOCAL CAMERA LCD','USB CAM ON SBC  ','LOCAL VIDEO ONLY','NO VIDEO TO SBC ','CAMERA TASK ONE ','HOME TO CONTROL '],
        8:['ENCODER / MOTOR ','SPEED PID FIRST ','POSITION PID    ','LIMITS / TARGET ','VALUE EDITOR    ','WHEEL TEST      '],
    }
    if view in menus: body=menus[view]
    elif view in (10,11):
        position=view==11
        body=[num('LTGT ','pos_target_l',32) if position else num('LTGT ','motor_target_l',16),
              num('LPOS ','position_l') if position else num('LSPD ','speed_l')]
        for label,key in (('LKP  ','kp_l'),('LKI  ','ki_l'),('LKD  ','kd_l')):
            if position:key='p'+key
            scaled=(s[key]&65535)*10000//256
            body.append(f'{label}{str(scaled//10000)+"."+str(scaled%10000).zfill(4):>11}')
        body.append(' '*16)
    elif view==12:
        body=[{10:'EDIT SPEED LOOP ',11:'EDIT POS LOOP   ',12:'EDIT PWM LIMITS '}[view],
              'OPEN VALUE EDIT ','PWM / DEADBAND  ' if view==12 else 'SET KP KI KD    ',
              'TEST AT LOW RATE','TUNE BOTH WHEELS','STOP BEFORE EDIT']
    elif view==9:
        wheel='r' if s['wheel'] else 'l'
        body=['RIGHT / MOTOR2  ' if s['wheel'] else 'LEFT / MOTOR1   ',
              f'ENC SIGN {int(s["enc_"+wheel])}'.ljust(16),f'MOTOR SIGN {int(s["drive_"+wheel])}'.ljust(16),
              num('SPD  ','speed_'+wheel),num('POS  ','position_'+wheel),num('PWM  ','duty_'+wheel,16)]
    elif view in (14,15):
        body=[num('LSPD ','speed_l'),num('RSPD ','speed_r'),num('LPOS ','position_l'),
              num('RPOS ','position_r'),num('LPWM ','duty_l',16),num('RPWM ','duty_r',16)]
    elif view==16:
        body=[num(label,key,signed=False) for label,key in
              (('RX   ','rx'),('NODE ','node'),('PTS  ','points'),('UERR ','uerr'),('PERR ','perr'),('RST  ','restarts'))]
    elif view==17:
        body=['BACK UNKNOWN    ' if not 50<=s['rear']<=16000 else num('BK   ','rear',16,False)]
        for key,prefix,title in (('front','FR   ','FRONT'),('left','LF   ','LEFT'),('right','RT   ','RIGHT')):
            body.append((title+' UNKNOWN').ljust(16) if not 50<=s[key]<=16000 else num(prefix,key,16,False))
        body += [num('SELF ','self_points',16,False),num('SCANS','scans',signed=False)]
    elif view in (18,25,26,27,35):
        body=[field_name,num('VALUE','value'),num('STEP ','step',signed=False)]
        if view==18:
            body += [num('ZERO ','lidar_zero',9,False),num('XOFF ','xoff',16),num('YOFF ','yoff',16)]
        elif view==25:
            body += [num('CIRC ','circ',16,False),num('CPR  ','cpr',16,False),num('TRACK','track',16,False)]
        elif view==26:
            body += [num('X    ','x'),num('Y    ','y'),num('YAW  ','heading')]
        elif view==27:
            body += [num('GX   ','goal_x'),num('GY   ','goal_y'),num('YAW  ','heading')]
        else:
            body += [num('CRUIS','cruise',16,False),num('TURN ','turn',16,False),'UNITS CNT/10MS  ']
    elif view==24:
        body=[num('SETX ','set_x'),num('SETY ','set_y'),num('SYAW ','set_heading'),
              num('GX   ','goal_x'),num('GY   ','goal_y'),constnum('GEO  ',s['geometry'])]
    elif view==28:
        body=['GYRO CAL OK     ' if s['gyro'] and not s['calibrating'] else 'GYRO CAL WAIT   ',
              num('YAW  ','heading'),constnum('ZINV ',s['gyro_invert']),num('ZRAW ','gyro_raw',16),
              'KEEP BODY STILL ','Z AXIS VERTICAL ']
    elif view in (32,34):
        body=[num('LSET ','set_l',16,False),num('RSET ','set_r',16,False),
              num('LSPD ','speed_l'),num('RSPD ','speed_r')]
        body += ([num('YAW  ','heading'),'NO LIDAR GUARD  '] if view==32 else
                 [num('STEP ','step',signed=False),constnum('LINK ',s['manual_linked'])])
    elif view==33:
        body=[num('X    ','x'),num('Y    ','y'),num('YAW  ','heading'),num('GX   ','goal_x'),num('GY   ','goal_y'),nav]
    else: body=[' '*15+'0']*6
    title=('POSITION TEST   ' if s['motor_mode_position'] else 'SPEED LOOP TEST ') if view==15 else ROBOT_TITLES.get(view,'SELECT A PAGE   ')
    if view==17:
        title='LOCAL LIVE SCAN ' if s['map_local'] else 'WORLD MAP       ' if s['map_world_ready'] else 'WORLD MAP HOLD  '
    rows=[title,state]+body
    rows += ['GAINS READY     ' if s['gains'] else 'NEED SPEED GAINS',c1,hints.get(view,'STOP BEFORE EDIT'),
             field_name,num('VALUE','value'),num('STEP ','step',signed=False),
             f'M {s["score"]:02X} B{s["baseline"]:02X} D{s["margin"]:02X}    ',
             f'GEO{"Y" if s["geometry"] else "N"} GYR{"Y" if s["gyro"] else "N"} C1{s["lidar_error"]:02X}  ']
    if view in (10,11):
        position=view==11
        rows[8:11]=['POSITION COUNTS ' if position else 'COUNTS / 10MS   ','VOFA PID EDIT   ','READ ONLY LCD   ']
        rows[11:16]=[num('RTGT ','pos_target_r',32) if position else num('RTGT ','motor_target_r',16),
                     num('RPOS ','position_r') if position else num('RSPD ','speed_r')]
        for label,key in (('RKP  ','kp_r'),('RKI  ','ki_r'),('RKD  ','kd_r')):
            if position:key='p'+key
            scaled=(s[key]&65535)*10000//256
            rows.append(f'{label}{str(scaled//10000)+"."+str(scaled%10000).zfill(4):>11}')
    elif view==16:
        rows[8]=f'RX {s["lidar_last_rx_word"]&MASK32:08X}     '
        rows[10]=f'ST {s["lidar_debug_status"]&MASK32:08X}     '
        rows[11:16]=[f'{label} {(s["lidar_reply_capture"]>>shift)&MASK32:08X}     '
                     for label,shift in (('H0',96),('H1',64),('S0',32),('S1',0))]
        rows.append(f'TX {s["lidar_last_tx_word"]&MASK32:08X}     ')
    elif view==14:
        rows[8:11]=['EDIT WHEN STOP  ','BACK TO PID     ','NO AUTO START   ']
    elif view==17:
        rows[12:14]=[num('MAPPT','map_points',signed=False),num('DROP ','map_dropped',signed=False)]
    return [row.encode('ascii') for row in rows]


def check_robot_semantics():
    # The reference for full-width decimal formatting is independent of text
    # row selection and covers the exact32-bit double-dabble input range.
    def dd(value):
        bcd=0
        for bit in range(31,-1,-1):
            adjusted=0
            for digit in range(10):
                nibble=(bcd>>(digit*4))&15
                adjusted |= (nibble+3 if nibble>=5 else nibble)<<(digit*4)
            bcd=((adjusted<<1)|((value>>bit)&1))&((1<<40)-1)
        return bcd

    def blank_suppressed(bcd,negative):
        chars=[' ']*11
        seen=False
        for k in range(9,-1,-1):
            nibble=(bcd>>(k*4))&15
            if nibble or seen or k==0:
                chars[10-k]=chr(48+nibble)
                if not seen and negative: chars[9-k]='-'
                seen=True
        return ''.join(chars)

    rng=random.Random(20261025)
    values=[0,1,9,10,99,100,65535,999999,1000000,2147483647,2147483648,MASK32]
    values += [rng.getrandbits(32) for _ in range(1024)]
    for value in values:
        bcd=dd(value)
        for sign in (False,True):
            # uint32 max plus a minus sign still fits the11-character helper;
            # physical signed32 inputs never exceed magnitude2147483648.
            expected=(('-' if sign else '')+str(value)).rjust(11)
            assert blank_suppressed(bcd,sign)==expected,(value,sign)
    for key in ('rx','node','points','uerr','perr','restarts'):
        rows=robot_rows(dict(page=2,subpage=0,**{key:MASK32}))
        row={'rx':2,'node':3,'points':4,'uerr':5,'perr':6,'restarts':7}[key]
        assert rows[row][5:]==b' 4294967295', (key,rows[row])
    for value in (0,1,49,16001,65535):
        rows=robot_rows(dict(page=2,subpage=1,front=value,left=value,right=value))
        assert rows[3:6]==[b'FRONT UNKNOWN   ',b'LEFT UNKNOWN    ',b'RIGHT UNKNOWN   ']
    for value in (50,12000,16000):
        assert b'UNKNOWN' not in b''.join(robot_rows(dict(page=2,subpage=1,front=value,left=value,right=value))[3:6])
    encoder=robot_rows(dict(page=1,subpage=1,wheel=True,enc_r=True,drive_r=False,speed_r=-42,position_r=-2147483648,duty_r=-350))
    assert encoder[2:5]==[b'RIGHT / MOTOR2  ',b'ENC SIGN 1      ',b'MOTOR SIGN 0    ']
    assert encoder[5].endswith(b'-42') and encoder[6].endswith(b'-2147483648') and encoder[7].endswith(b'-350')
    for motor_running,motor_armed in ((False,False),(False,True),(True,False),(True,True)):
        encoder_state=dict(page=1,subpage=1,motor_running=motor_running,motor_armed=motor_armed,
                           calibrating=True,nav_fault=1)
        assert robot_rows(encoder_state)[1]==b'CAL DISARMED    '
        assert robot_rows({**encoder_state,'armed':True})[1]==b'CAL TEST ARMED  '
        assert robot_rows({**encoder_state,'calibration_enable':True})[1]==b'CAL JOG ACTIVE  '
    assert robot_rows(dict(page=2,nav_fault=1))[1]==b'LOCAL DISARMED  '
    assert robot_rows(dict(page=4,subpage=1,nav_fault=1))[1]==b'NAV FAULT       '
    assert robot_rows(dict(page=1,subpage=7,calibrating=True,motor_armed=True))[1]==b'MOTOR ARMED     '
    assert robot_rows(dict(page=1,subpage=7,calibrating=True,motor_running=True))[1]==b'MOTOR RUNNING   '
    for view in ROBOT_TITLES:
        rows=robot_rows(dict(page=view>>3,subpage=view&7))
        assert len(rows)==16 and all(len(row)==16 for row in rows),(view,rows)
        assert rows[0].decode()==('LOCAL LIVE SCAN ' if view==17 else 'SPEED LOOP TEST ' if view==15 else ROBOT_TITLES[view])
    for local,ready,title in ((True,False,b'LOCAL LIVE SCAN '),(True,True,b'LOCAL LIVE SCAN '),
                              (False,False,b'WORLD MAP HOLD  '),(False,True,b'WORLD MAP       ')):
        assert robot_rows(dict(page=2,subpage=1,map_local=local,map_world_ready=ready))[0]==title
    assert robot_rows(dict(page=1,subpage=7,motor_mode_position=True))[0]==b'POSITION TEST   '
    assert robot_rows(dict(page=1,subpage=7,motor_mode_position=False))[0]==b'SPEED LOOP TEST '
    # The actual running loop controls TEST's header; editing scope and all
    # other views do not pretend that the position-mode unlock has succeeded.
    assert robot_rows(dict(page=1,subpage=3,motor_mode_position=False))[0]==b'POSITION PID    '
    assert robot_rows(dict(page=1,subpage=2,motor_mode_position=True))[0]==b'SPEED PID       '
    dashboard=robot_rows(dict(page=1,subpage=2,motor_target_l=42,motor_target_r=-12,
                             speed_l=39,speed_r=-10,kp_l=256,ki_l=1,kd_l=65535,
                             kp_r=128,ki_r=0,kd_r=4096,lidar_error=255,score=255))
    assert dashboard[2:7]==[b'LTGT          42',b'LSPD          39',b'LKP       1.0000',
                             b'LKI       0.0039',b'LKD     255.9960']
    assert dashboard[11:16]==[b'RTGT         -12',b'RSPD         -10',b'RKP       0.5000',
                               b'RKI       0.0000',b'RKD      16.0000']
    assert dashboard[8:11]==[b'COUNTS / 10MS   ',b'VOFA PID EDIT   ',b'READ ONLY LCD   ']
    position=robot_rows(dict(page=1,subpage=3,pos_target_l=-2147483648,pos_target_r=2147483647,
                            position_l=-2000000,position_r=2000000,pkp_l=256,pki_l=1,pkd_l=65535,
                            pkp_r=128,pki_r=0,pkd_r=4096,lidar_error=255,score=255))
    assert position[2:7]==[b'LTGT -2147483648',b'LPOS    -2000000',b'LKP       1.0000',
                            b'LKI       0.0039',b'LKD     255.9960']
    assert position[11:16]==[b'RTGT  2147483647',b'RPOS     2000000',b'RKP       0.5000',
                              b'RKI       0.0000',b'RKD      16.0000']
    assert position[8:11]==[b'POSITION COUNTS ',b'VOFA PID EDIT   ',b'READ ONLY LCD   ']
    diagnostics=robot_rows(dict(page=2,lidar_last_rx_word=0xA55A4081,lidar_debug_status=0x64001305,
                               lidar_reply_capture=0xA55A030000000600A55A050000004081,lidar_last_tx_word=0xA525A520))
    assert diagnostics[8]==b'RX A55A4081     ' and diagnostics[10]==b'ST 64001305     '
    assert diagnostics[11:16]==[b'H0 A55A0300     ',b'H1 00000600     ',b'S0 A55A0500     ',
                                 b'S1 00004081     ',b'TX A525A520     ']
    assert robot_rows(dict(scan_enable=False,lidar_error=0x15))[9]==b'C1 STILL RX     '
    assert robot_rows(dict(scan_enable=False,lidar_error=0))[9]==b'C1 REQ OFF      '
    for notice,text in NOTICE_LABELS.items():
        sample=dict(page=4,subpage=0,notice=notice,manual_active=True)
        assert robot_rows(sample)[1].decode()==text
        assert robot_rows({**sample,'stop':True})[1]==b'ALL STOP LATCHED'
        assert robot_rows({**sample,'motor_fault':1})[1]==b'MOTOR FAULT     '
    for code,text in ((2,'C1 WAIT HEALTH  '),(3,'C1 WAIT HEALTH  '),(4,'C1 WAIT SCAN    '),
                      (5,'C1 WAIT NODE    '),(0x13,'C1 STALE        ')):
        assert robot_rows(dict(scan_enable=True,rx=1,lidar_error=code))[9].decode()==text
    assert robot_rows(dict(scan_enable=True,rx=0))[9]==b'C1 NO RX        '
    assert robot_rows(dict(scan_enable=True,rx=1,lidar_ready=True))[9]==b'C1 LIVE         '
    assert robot_rows(dict(page=4,manual_active=True))[1]==b'RAW MANUAL RUN  '
    assert robot_rows(dict(page=4,manual_turn_active=True,nav_status=4))[1]==b'TURNING         '
    assert robot_rows(dict(page=4,nav_status=6))[1]==b'ARRIVED         '
    assert robot_rows(dict(page=4))[7]==b'NO LIDAR GUARD  '
    nav_speed=robot_rows(dict(page=4,subpage=3,field=8,value=120,step=10,cruise=120,turn=35))
    assert nav_speed[2]==b'CRUISE COUNT/10M' and nav_speed[3].endswith(b'120')
    assert nav_speed[4].endswith(b'10') and nav_speed[5].endswith(b'120') and nav_speed[6].endswith(b'35')
    assert nav_speed[7]==b'UNITS CNT/10MS  ' and nav_speed[10]==b'BACK TO NAV     '
    for page in range(8):
        for subpage in range(8):
            s={**ROBOT_DEFAULTS,'page':page,'subpage':subpage,'lidar_error':0xff,'nav_fault':0xa5,
               'nav_status':9,'gains':True,'map_ready':True,'stop':True,'scan_enable':True,
               'lidar_ready':True,'calibrating':True,'gyro':True,'geometry':True,'armed':True}
            flags=robot_flags(s)
            assert (flags>>28)&7==subpage and flags>>31==1 and flags&7==page
            old=((s['lidar_error']<<23)|(s['nav_fault']<<15)|(s['nav_status']<<11)|0x7f8|page)
            assert flags&((1<<28)-1)==old&((1<<28)-1)
    print(f'PASS:21 semantic robot views;{len(values)*2} full32 decimal/sign cases, '
          'UINT32 counters, unknown ranges, selected-wheel signs, notices, C1 stages and flags30:28.')


class RowTransactions:
    """Abstract row offers/accepts, with publication as a separate transaction."""
    def __init__(self, rows, previous=b'Z' * 256):
        assert len(rows) == 16 and all(len(row) == 16 for row in rows)
        self.rows = deepcopy(rows)
        self.previous = self.lcd = previous
        self.row = self.byte = 0
        self.shift = int.from_bytes(self.rows[0], 'big')
        self.accepted = []
        self.done = self.published = False

    def offer(self):
        return not self.done, self.row * 16 + self.byte, self.shift >> 120

    def service(self, ready):
        valid, index, data = self.offer()
        before = (self.row, self.byte, self.shift, self.lcd)
        if valid and ready:
            self.accepted.append((index, data))
            if self.byte == 15:
                if self.row == 15:
                    self.done = True
                else:
                    self.row += 1
                    self.byte = 0
                    self.shift = int.from_bytes(self.rows[self.row], 'big')
            else:
                self.byte += 1
                self.shift = (self.shift << 8) & ((1 << 128) - 1)
        else:
            assert before == (self.row, self.byte, self.shift, self.lcd)
        assert self.lcd == self.previous

    def publish(self):
        assert self.done and len(self.accepted) == 256 and not self.published
        assert [index for index, _ in self.accepted] == list(range(256))
        assert bytes(data for _, data in self.accepted) == b''.join(self.rows)
        self.lcd = b''.join(self.rows)
        self.published = True


def main():
    source_checks()
    check_robot_semantics()
    rng = random.Random(20261025)
    edges = (0, 1, 255, 256, 32767, 32768, 65535, 99999999, 100000000,
             (1 << 31) - 1, 1 << 31, MASK32)
    transactions = stalls = 0
    for number in range(512):
        choose = lambda: edges[number % len(edges)] if number < 48 else rng.getrandbits(32)
        motor = dict(field=number % 16, value=choose(), step=choose(),
            target_l=choose(), target_r=choose(), speed_l=choose(), speed_r=choose(),
            position_l=choose(), position_r=choose(), duty_l=choose(), duty_r=choose(),
            fault_l=rng.getrandbits(8), fault_r=rng.getrandbits(8),
            invalid_l=rng.getrandbits(16), invalid_r=rng.getrandbits(16),
            stop=bool(number & 1), link=bool(number & 2), running=bool(number & 4),
            armed=bool(number & 8), wheel=bool(number & 16), edit_position=bool(number & 32),
            mode_position=bool(number & 64), unlocked=bool(number & 128))
        robot = dict(page=number % 8, field=number % 16, value=choose(), step=choose(),
            x=choose(), y=choose(), heading=choose(), goal_x=choose(), goal_y=choose(),
            points=choose(), score=rng.getrandbits(8), baseline=rng.getrandbits(8), margin=rng.getrandbits(8),
            front=65535 if number % 3 == 0 else rng.getrandbits(16),
            left=65535 if number % 3 == 1 else rng.getrandbits(16),
            right=65535 if number % 3 == 2 else rng.getrandbits(16),
            geometry=bool(number & 1), gyro=bool(number & 2), stop=bool(number & 4),
            calibrating=bool(number & 8), gains=bool(number & 16), armed=bool(number & 32),
            nav_fault=0 if number & 64 else rng.getrandbits(8), nav_status=number % 16,
            lidar_error=rng.getrandbits(8))
        view=tuple(ROBOT_TITLES)[number%len(ROBOT_TITLES)]
        robot.update(page=view>>3,subpage=view&7,rx=choose(),node=choose(),uerr=choose(),perr=choose(),
            restarts=choose(),map_points=choose(),scans=choose(),speed_l=choose(),speed_r=choose(),
            position_l=choose(),position_r=choose(),duty_l=choose(),duty_r=choose(),gyro_raw=choose(),
            set_l=choose(),set_r=choose(),set_x=choose(),set_y=choose(),set_heading=choose(),
            motor_target_l=choose(),motor_target_r=choose(),kp_l=choose(),ki_l=choose(),kd_l=choose(),
            kp_r=choose(),ki_r=choose(),kd_r=choose(),lidar_debug_status=choose(),lidar_last_rx_word=choose(),
            pos_target_l=choose(),pos_target_r=choose(),pkp_l=choose(),pki_l=choose(),pkd_l=choose(),
            pkp_r=choose(),pki_r=choose(),pkd_r=choose(),lidar_reply_capture=rng.getrandbits(128),lidar_last_tx_word=choose(),
            circ=choose(),cpr=choose(),track=choose(),cruise=choose(),turn=choose(),lidar_zero=choose(),xoff=choose(),yoff=choose(),
            wheel=bool(number&1),enc_l=bool(number&2),enc_r=bool(number&4),drive_l=bool(number&8),drive_r=bool(number&16),
            scan_enable=bool(number&32),lidar_ready=bool(number&64),motor_armed=bool(number&128),motor_running=bool(number&256),
            notice=number%7,manual_active=bool(number&1),manual_turn_active=bool(number&2),manual_linked=bool(number&4),
            gyro_invert=bool(number&8),calibration_enable=bool(number&16),motor_fault=0 if number&32 else 1)
        for rows in (motor_rows(motor), robot_rows(robot)):
            model = RowTransactions(rows)
            while not model.done:
                # Force first/last bytes to withstand extended stalls, plus
                # random backpressure on every other byte of each row.
                if model.byte in (0, 15):
                    offered = model.offer()
                    for _ in range(3):
                        model.service(False)
                        assert model.offer() == offered
                        stalls += 1
                if rng.randrange(3) == 0:
                    model.service(False)
                    stalls += 1
                model.service(True)
                transactions += 1
            model.service(False)  # End-of-frame is still not an LCD commit.
            model.publish()
            assert model.lcd == b''.join(rows)
            # Dropping a partial frame on reset leaves no suffix capable of
            # masquerading as a new frame's byte0.
            reset_model = RowTransactions(rows)
            for _ in range(number % 256):
                reset_model.service(True)
            assert reset_model.lcd == b'Z' * 256 and not reset_model.published
            reset_model = RowTransactions(rows, previous=bytes(256))
            assert reset_model.offer() == (True, 0, rows[0][0])
    overflow = motor_rows({**motor, 'speed_l': 100000000})
    assert b'OVERFLOW' in overflow[6]
    for field in range(16):
        label = MOTOR_FIELD_LABELS[field] if field<len(MOTOR_FIELD_LABELS) else 'FIELD INVALID   '
        assert motor_rows({**motor,'field':field})[2] == label.encode('ascii')
    print(f'PASS:1024 complete16x16 motor/NAV references;{transactions} accepted bytes, '
          f'{stalls} backpressure transactions, ordered0..255 and atomic LCD publication.')
    print('PASS:motor pre-stream expressions and current robot snapshot contracts checked; '
          'two-wheel PID dashboard and frozen diagnostic hex snapshots; '
          'motor128-bit and robot88-bit numeric shifters, atomic snapshots and complete numeric widths verified.')
    print('No HDL compilation, simulation, programming or resource/timing claim.')


if __name__ == '__main__':
    main()
