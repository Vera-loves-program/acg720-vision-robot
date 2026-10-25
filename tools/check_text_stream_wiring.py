"""Read-only contracts for PID/NAV streams, UART RAM and LCD bank wiring.

Lexical source checks plus independent finite bank/arbitration mathematics.
This is not HDL compilation, elaboration, simulation or a timing/resource test.
"""
from itertools import product
from collections import Counter
from pathlib import Path
import re
from check_motor_integration import (uncomment, check_blocks, closing,
                                    module_ports, instances, SOURCE_MODULES)

ROOT=Path(__file__).resolve().parents[1]

def source(name):
    text=uncomment((ROOT/'src'/name).read_text(encoding='utf-8-sig'))
    check_blocks(text,name)
    return text

def compact(text):
    return re.sub(r'\s+','',text)

def connections(text,module,instance):
    for match in re.finditer(r'\b'+re.escape(module)+r'\s*',text):
        cursor=match.end()
        if text[cursor:cursor+1]=='#':
            opening=text.index('(',cursor)
            cursor=closing(text,opening)+1
        found=re.match(r'\s*'+re.escape(instance)+r'\s*\(',text[cursor:])
        if not found:
            continue
        opening=cursor+found.end()-1
        body=text[opening+1:closing(text,opening)]
        result={}
        for item in re.finditer(r'\.\s*(\w+)\s*\(',body):
            start=item.end()-1
            assert item[1] not in result,(module,instance,item[1])
            result[item[1]]=compact(body[start+1:closing(body,start)])
        return result
    raise AssertionError((module,instance,'not found'))

top=source('vision_robot_ui_top.v')
subsystem=source('robot_subsystem.v')
motor=source('motor_status_text.v')
robot=source('robot_status_text.v')
uart=source('sensor_demo_uart.v')
sensor=source('sensor_demo_text.v')
lcd=source('lcd1024_interaction_ui.v')

# Audit every named connection of the existing new-module catalogue, including
# helpers declared in the same file (UI hit-test, caption ROM and byte-bank
# wrappers). Merely checking the two formatter ready pins misses renamed or
# omitted ports in the new calibration/manual control path.
audited_sources={name:source(name+'.v') for name in SOURCE_MODULES}
declared={}
for owner,text in audited_sources.items():
    for module,pins in module_ports(text).items():
        assert module not in declared,('duplicate module declaration',owner,module)
        declared[module]=pins
all_instances=[]
for owner,text in audited_sources.items():
    for module,instance,pins in instances(text,declared):
        assert all(count==1 for count in Counter(pins).values()),(owner,instance,'duplicate port')
        assert set(pins)==declared[module],(owner,instance,
            'missing',sorted(declared[module]-set(pins)),
            'unknown',sorted(set(pins)-declared[module]))
        all_instances.append((owner,module,instance))
assert sum(module=='robot_ui_hit_test' for _,module,_ in all_instances)==3
assert {'robot_vofa_uart','motor_vofa_settings_adapter','sensor_demo_text'}<=set(declared)
module_count=len(declared)
motor_link=connections(top,'motor_status_text','u_motor_text')
uart_link=connections(top,'sensor_demo_uart','u_sensor_demo_uart')
robot_link=connections(top,'robot_subsystem','u_robot')
status_link=connections(subsystem,'robot_status_text','u_status')
store_link=connections(uart,'sensor_demo_text_snapshot_store','u_text_store')
lcd_link=connections(top,'lcd1024_interaction_ui','u_lcd')
lcd_streams={stream:connections(top,'lcd_text_byte_snapshot','u_'+stream+'_lcd_text')
             for stream in ('motor','robot')}

for suffix in ('valid','data','index'):
    assert motor_link['text_byte_'+suffix]=='motor_byte_'+suffix
    assert status_link['text_byte_'+suffix]=='status_byte_'+suffix
    assert robot_link['status_byte_'+suffix]=='robot_byte_'+suffix
    for stream in ('motor','robot'):
        port=stream+'_byte_'+suffix
        assert store_link[port]==port
        if suffix!='valid':
            assert uart_link[port]==port
assert uart_link['motor_byte_valid']=='!ACTUATOR_TEST&&motor_byte_valid&&(!ROBOT_ACTIVE||motor_lcd_byte_ready)'
assert uart_link['robot_byte_valid']=='ROBOT_ACTIVE&&robot_byte_valid&&robot_lcd_byte_ready'
assert motor_link['text_byte_ready']=="ACTUATOR_TEST?1'b1:(motor_byte_ready&&(!ROBOT_ACTIVE||motor_lcd_byte_ready))"
assert robot_link['status_byte_ready']=="ROBOT_ACTIVE?(robot_byte_ready&&robot_lcd_byte_ready):1'b1"
assert status_link['text_byte_ready']=='status_byte_ready'
for stream in ('motor','robot'):
    assert uart_link[stream+'_byte_ready']==stream+'_byte_ready'
    assert store_link[stream+'_byte_ready']==stream+'_byte_ready'
assert motor_link['clk']==uart_link['clk']==robot_link['clk']=='clk50M'
assert motor_link['reset_n']==uart_link['reset_n']=='ready'
assert robot_link['reset_n']=='ready&&ROBOT_ACTIVE'
assert status_link['clk']=='clk' and status_link['reset_n']=='reset_n'
assert store_link['reset_n']=='reset_n&&!ACTUATOR_TEST'
assert store_link['clk']=='clk'
# VOFA robot builds drain the character producers into LCD alone. The
# deprecated UART snapshot RAM/sender live only in the other generate branch;
# keeping them textually present must not activate a second transmitter/bank.
uart_code=compact(uart)
active_branch=uart_code.split('generateif(VOFA_MODE!=0)begin:vofa_lcd_only',1)[1].split('endelsebegin:legacy_sender',1)[0]
assert active_branch=="assignmotor_byte_ready=1'b1;assignrobot_byte_ready=1'b1;assigntx_line_count=32'd0;assigntx=1'b1;"
assert 'sensor_demo_text_snapshot_store' not in active_branch
assert 'sensor_demo_uart_tx_8n1' not in active_branch
assert 'sensor_demo_uart#(.ACTUATOR_TEST(ACTUATOR_TEST),.ROBOT_MODE(ROBOT_ACTIVE),.VOFA_MODE(ROBOT_ACTIVE))u_sensor_demo_uart(' in compact(top)
legacy_branch=uart_code.split('endelsebegin:legacy_sender',1)[1].split('endendgenerate',1)[0]
assert 'u_text_store(' in legacy_branch and 'u_tx(' in legacy_branch
assert 'wirepixel_reset_n=ready&&pixel_pll_locked;' in compact(top)
assert 'lcd1024_interaction_ui#(.BYTE_TEXT_MODE(ROBOT_ACTIVE))u_lcd(' in compact(top)
lcd_ports=module_ports(lcd)
assert set(lcd_link)==lcd_ports['lcd1024_interaction_ui']
for stream,link in lcd_streams.items():
    assert set(link)==lcd_ports['lcd_text_byte_snapshot']
    assert link['write_clk']=='clk50M' and link['read_clk']=='pixel_clk'
    assert link['reset_n']=='pixel_reset_n&&ROBOT_ACTIVE'
    assert link['read_run']=='pixel_run_pipe[1]' and link['frame_restart']=='frame_restart'
    assert link['byte_valid']=='ROBOT_ACTIVE&&'+stream+'_byte_valid&&'+stream+'_byte_ready'
    for suffix in ('data','index'):
        assert link['byte_'+suffix]==stream+'_byte_'+suffix
    assert link['byte_ready']==stream+'_lcd_byte_ready'
    assert link['frame_commit']==stream+'_text_valid'
    assert link['frame_meta']==("32'd0" if stream=='motor' else 'robot_flags')
    assert link['read_en']==stream+'_lcd_read_en'
    assert link['read_index']=='lcd_text_read_index'
    assert link['read_byte']==stream+'_lcd_character'
assert lcd_streams['robot']['read_meta']=='robot_lcd_meta'
assert lcd_link['clk']=='pixel_clk' and lcd_link['run']=='pixel_run_pipe[1]'
assert lcd_link['robot_byte_character']=='robot_lcd_character'
assert lcd_link['pid_byte_character']=='motor_lcd_character'
assert lcd_link['robot_byte_meta']=='robot_lcd_meta'
assert lcd_link['robot_byte_read_en']=='robot_lcd_read_en'
assert lcd_link['pid_byte_read_en']=='motor_lcd_read_en'
assert lcd_link['byte_text_read_index']=='lcd_text_read_index'

# Legacy 2048-bit text remains at the LCD snapshot boundary; it no longer
# re-enters either the UART module interface or its single-port RAM helper.
ports=module_ports(uart)
for old in ('motor_text_data','motor_text_valid','robot_text_data','robot_text_valid'):
    assert old not in ports['sensor_demo_uart']
    assert old not in ports['sensor_demo_text_snapshot_store']
assert motor_link['text_rows']=='motor_text' and motor_link['text_valid']=='motor_text_valid'
assert status_link['text_rows']=='status_text' and status_link['text_valid']=='status_valid'
assert status_link['flags']=='status_flags'
assert robot_link['status_text']=='robot_text' and robot_link['status_valid']=='robot_text_valid'

# An omitted ready input would strand a formatter in EMIT, suppressing its
# legacy LCD commit as well as UART. Audit every non-vendor source instance,
# including any future integration site rather than just the two known ones.
known={'motor_status_text':module_ports(motor)['motor_status_text'],
       'robot_status_text':module_ports(robot)['robot_status_text']}
found=[]
for path in sorted((ROOT/'src').glob('*.v')):
    text=uncomment(path.read_text(encoding='utf-8-sig'))
    for module,instance,pins in instances(text,known):
        link=connections(text,module,instance)
        for port in ('text_byte_ready','text_byte_valid','text_byte_data','text_byte_index'):
            assert port in pins and link.get(port),(path.name,module,instance,port,'unconnected')
        found.append((path.name,module,instance))
assert len(found)==2,found

for text,row in ((motor,'field'),(robot,'rowfield')):
    code=compact(text)
    declared=next(iter(module_ports(text).values()))
    if row=='field':
        # The default legacy branch retains its original complete formatter;
        # active ROBOT selects the separate three-row byte implementation.
        code=code.split('endelsebegin:full_formatter',1)[1].rsplit('endendgenerate',1)[0]
    assert {'text_byte_valid','text_byte_data','text_byte_index','text_byte_ready'}<=declared
    assert 'assigntext_byte_valid=reset_n&&state==EMIT;' in code
    assert 'assigntext_byte_index={'+row+',byte_in_row};' in code
    emit=code.split('EMIT:if(text_byte_ready)begin',1)[1].split('COMMIT:begin',1)[0]
    assert 'if(byte_in_row==15)begin' in emit
    assert 'if('+row+'==15)state<=COMMIT;' in emit
    assert "byte_in_row<=byte_in_row+1'b1;" in emit
    commit=code.split('COMMIT:begin',1)[1].split('default:state<=IDLE;',1)[0]
    assert 'text_valid<=1;state<=IDLE;' in commit
    if row=='field':
        assert 'assigntext_byte_data=emit_shift[127:120];' in code
        assert code.count('text_byte_ready')==1  # branch body, header audited above
        # Preserve the motor formatter's existing constant row write folding.
        for target in re.findall(r'\brows\[([^\]]+)\]\s*<=',text):
            assert target.isdecimal() or target=='i',('dynamic LCD row write',row,target)
        assert not re.search(r'\brows\[[^\]]+\]\s*<=\s*formatted_row\b',text)
        assert 'emit_shift<={emit_shift[119:0],8\'d0};' in emit
        assert 'text_rows[' in commit
    else:
        # Status constants are now byte-wide ROM reads, with an88-bit number
        # shifter for the eleven-character numeric suffix. Never update ROMQ
        # or that suffix during an unaccepted EMIT byte.
        assert 'assigntext_byte_data=emit_numeric&&byte_in_row>=5?emit_number[87:80]:decorated_byte;' in code
        assert 'wirelabel_read=state==STORE||(state==EMIT&&text_byte_ready&&byte_in_row!=15&&(!emit_numeric||byte_in_row<4));' in code
        assert "wire[3:0]label_column=state==STORE?4'd0:byte_in_row+4'd1;" in code
        label_link=connections(robot,'robot_status_label_rom','u_labels')
        assert label_link=={'clk':'clk','read_enable':'label_read','address':'{emit_id,label_column}','data':'label_byte'}
        assert 'STORE:beginemit_number<=number;byte_in_row<=0;state<=EMIT;end' in code
        assert "if(emit_numeric&&byte_in_row>=5)emit_number<={emit_number[79:0],8'd0};" in emit
        assert 'generateif(PARALLEL_COMPAT!=0)begin:parallel_compatibility' in code
        assert "endelsebegin:no_parallel_compatibilityassigntext_rows=2048'd0;endendgenerate" in code
        assert 'robot_status_text#(.PARALLEL_COMPAT(0))u_status(' in compact(subsystem)
        assert 'if(text_byte_valid&&text_byte_ready)accepted_frame<={accepted_frame[2039:0],text_byte_data};' in code
        assert 'if(state==COMMIT)published_frame<=accepted_frame;' in code
        assert 'flags<=held_flags;' in commit

motor_compact=compact(motor).split('generateif(COMPACT_ROBOT!=0)begin:compact_robot',1)[1].split('endelsebegin:full_formatter',1)[0]
assert 'motor_status_text#(.COMPACT_ROBOT(ROBOT_ACTIVE))u_motor_text(' in compact(top)
for contract in ('assigntext_byte_valid=reset_n&&compact_state==C_EMIT;',
                 'assigntext_byte_index={compact_row,compact_column};',
                 'assigntext_byte_data=compact_emit[127:120];',
                 'C_EMIT:if(text_byte_ready)begin',
                 "compact_emit<={compact_emit[119:0],8'd0};",
                 'if(compact_row==15)compact_state<=C_COMMIT;',
                 'C_COMMIT:begintext_valid<=1;compact_state<=C_IDLE;end'):
    assert contract in motor_compact,contract
assert motor_compact.count('text_rows<=0;')==2

# The status formatter still emits exactly 16x16 bytes. The paired dashboard
# reuses one BCD converter for ten fields; other pages keep the eight-field
# schedule. No whole-row writeback/extra text bank is introduced.
r=compact(robot)
assert 'reg[31:0]values[0:9];' in r
assert 'reg[2:0]state;' in r and 'reg[3:0]field;' in r and 'reg[9:0]signs,fractional;' in r
assert 'wireheld_pid_view=held_view==10||held_view==11;' in r
assert 'if(held_pid_view?((rowfield>=2&&rowfield<=6)||rowfield>=11):((rowfield>=2&&rowfield<=7)||rowfield==12||rowfield==13))begin' in r
assert "if(held_pid_view?((rowfield>=2&&rowfield<=6)||(rowfield>=11&&rowfield<=14)):((rowfield>=2&&rowfield<=7)||rowfield==12))field<=field+1'b1;" in r
assert 'held_flags<={gains_ready,subpage,lidar_error[4:0],nav_fault,nav_status,map_ready,safety_stop,scan_enable,' in r
assert 'wire[5:0]held_view={held_flags[2:0],held_flags[30:28]};' in r
numeric_fields=[]
field=0
for row in range(16):
    if 2<=row<=7 or row in (12,13):
        numeric_fields.append((row,field))
    if 2<=row<=7 or row==12:
        field+=1
assert numeric_fields==[(2,0),(3,1),(4,2),(5,3),(6,4),(7,5),(12,6),(13,7)]
assert field==7
dashboard_fields=[]
field=0
for row in range(16):
    if 2<=row<=6 or row>=11:
        dashboard_fields.append((row,field))
    if 2<=row<=6 or 11<=row<=14:
        field+=1
assert dashboard_fields==[(2,0),(3,1),(4,2),(5,3),(6,4),(11,5),(12,6),(13,7),(14,8),(15,9)]
assert field==9
# Trace the configured setpoints and all six gains from the settings owner
# through subsystem to formatter, rather than reusing target_effective=0
# while stopped. Raw C1 debug words take the same held snapshot route.
for port,signal in {'motor_target_left':'motor_target_speed_left',
                    'motor_target_right':'motor_target_speed_right',
                    'speed_kp_left':'motor_skp_l','speed_kp_right':'motor_skp_r',
                    'speed_ki_left':'motor_ski_l','speed_ki_right':'motor_ski_r',
                    'speed_kd_left':'motor_skd_l','speed_kd_right':'motor_skd_r',
                    'motor_position_target_left':'motor_target_position_left',
                    'motor_position_target_right':'motor_target_position_right',
                    'position_kp_left':'motor_pkp_l','position_kp_right':'motor_pkp_r',
                    'position_ki_left':'motor_pki_l','position_ki_right':'motor_pki_r',
                    'position_kd_left':'motor_pkd_l','position_kd_right':'motor_pkd_r'}.items():
    assert robot_link[port]==signal,(port,robot_link.get(port))
    assert status_link[port]==port,(port,status_link.get(port))
c1_link=connections(subsystem,'rplidar_c1','u_c1')
assert c1_link['debug_status']==status_link['lidar_debug_status']=='lidar_debug_status'
assert c1_link['last_rx_word']==status_link['lidar_last_rx_word']=='lidar_last_rx_word'
for port,signal in (('reply_capture','lidar_reply_capture'),('last_tx_word','lidar_last_tx_word')):
    assert c1_link[port]==status_link[signal]==signal

# The processed IMU snapshot remains available to LCD/common-format clients.
# VOFA deliberately plots only four wheel-speed fields; it must not grow a
# second raw-IMU conversion or stream navigation/battery/debug fields.
sensor_link=connections(uart,'sensor_demo_text','u_format')
for port in ('processed_axes','processed_imu_valid'):
    assert sensor_link[port]==port
assert uart_link['processed_axes']=='imu_processed_axes'
assert uart_link['processed_imu_valid']=='imu_processed_valid'
assert 'processed_axes<={processed_work[5],processed_work[4],processed_work[3],processed_work[2],processed_work[1],processed_work[0]};' in compact(sensor)
assert 'processed_imu_valid<=imu_ok;' in compact(sensor)
top_code=compact(top)
assert 'wire[127:0]vofa_sample_values;' in top_code
assert set(re.findall(r'assignvofa_sample_values\[(\d+)\*32\+:32\]',top_code))=={'0','1','2','3'}
assert 'vofa_processed_imu' not in top_code
codec=connections(top,'robot_vofa_uart','u_vofa')
adapter=connections(top,'motor_vofa_settings_adapter','u_vofa_settings')
settings=connections(top,'motor_tuning_control','u_motor_settings')
vofa_tx=connections(top,'sensor_demo_uart_tx_8n1','u_vofa_tx')
for prefix in ('cmd','rsp'):
    for suffix in ('valid','ready','seq','op','wheel','field','value'):
        port=prefix+'_'+suffix
        assert codec[port]==adapter[port]=='vofa_'+port
assert codec['rsp_status']==adapter['rsp_status']=='vofa_rsp_status'
assert codec['sample_values']=='vofa_sample_values'
assert codec['byte_valid']=='demo_rx_byte_valid' and codec['byte_data']=='demo_rx_byte'
assert codec['emergency_stop_pulse']=='vofa_emergency'
assert codec['tx_byte_valid']=='vofa_tx_byte_valid' and codec['tx_byte_data']=='vofa_tx_byte'
assert codec['tx_byte_ready']=='vofa_tx_byte_ready'
assert codec['clk']==adapter['clk']==vofa_tx['clk']=='clk50M'
assert codec['reset_n']==adapter['reset_n']==vofa_tx['reset_n']=='ready&&ROBOT_ACTIVE'
assert 'assignvofa_tx_byte_ready=!vofa_tx_busy;' in top_code
assert vofa_tx['start']=='vofa_tx_byte_valid&&vofa_tx_byte_ready'
assert vofa_tx['data']=='vofa_tx_byte' and vofa_tx['busy']=='vofa_tx_busy' and vofa_tx['tx']=='vofa_tx'
assert 'assignbluetooth_tx=ROBOT_ACTIVE?vofa_tx:legacy_bluetooth_tx;' in top_code
assert 'assignbluetooth_tx_lines=ROBOT_ACTIVE?vofa_tx_lines:legacy_bluetooth_lines;' in top_code
assert adapter['core_action_valid']=='vofa_action_valid' and adapter['core_argument']=='vofa_argument'
assert adapter['core_action']=='vofa_action' and adapter['core_live_update']=='vofa_live_update'
assert adapter['core_selected_value']==settings['selected_value']=='motor_selected_value'
assert settings['uart_action']=='vofa_action_valid?vofa_action:motor_uart_action'
assert settings['uart_argument']=='vofa_action_valid?vofa_argument:motor_uart_argument'
assert settings['uart_live_update']=='vofa_action_valid&&vofa_live_update'
assert settings['ui_action_valid']=='settings_ui_valid&&(vofa_cmd_ready||settings_ui_stop)'
assert adapter['core_stop_override']=='settings_ui_stop'
assert 'wiremotor_safety_stop=stop_latched||!reset_n||pid_emergency_stop||motor_uart_emergency||vofa_emergency||robot_emergency_stop||(ROBOT_ACTIVE&&!button4_n);' in top_code
assert adapter['takeover']=='vofa_takeover' and robot_link['remote_takeover']=='vofa_takeover'
assert codec['turn_complete_valid']==robot_link['turn_complete_valid']=='robot_turn_complete_valid'
assert codec['turn_complete_ready']==robot_link['turn_complete_ready']=='robot_turn_complete_ready'
assert codec['turn_complete_busy']==robot_link['turn_report_busy']=='robot_turn_report_busy'
for adapter_port,robot_port,signal in (
    ('turn_request','remote_turn_request','vofa_turn_request'),
    ('turn_degrees','remote_turn_degrees','vofa_turn_degrees'),
    ('turn_ready','remote_turn_ready','robot_turn_ready'),
    ('turn_active','remote_turn_active','robot_turn_active')):
    assert adapter[adapter_port]==robot_link[robot_port]==signal
adapter_code=compact(audited_sources['motor_tuning_control']).split('modulemotor_vofa_settings_adapter(',1)[1]
for settled in ('WRITE_VALUE:state<=WAIT_ONE;', 'WAIT_ONE:state<=WAIT_TWO;',
                'WAIT_TWO:state<=REPLY;',
                "rsp_value<=unknown_field?32'sd0:core_selected_value;",
                'core_selected_value!=held_value)rsp_status<=INTERRUPTED;',
                'wireblocked_now=safety_stop||core_stop_override||core_link_fault||core_fault_left!=0||core_fault_right!=0;',
                'if(cancelled||blocked_now)core_action_valid=0;',
                'if(held_op==1&&(cancelled||blocked_now))core_action_valid=0;'):
    assert settled in adapter_code, settled
# Cancellation is applied after ordinary dispatch so a coincident UI STOP
# cannot be overwritten by a later RUN substate or a premature success ACK.
assert adapter_code.index('if(cancel_new)begin')>adapter_code.index('REPLY:begin')
assert 'if(state==REPLY)rsp_valid<=0;' in adapter_code
assert '(ui_action_valid&&ui_action==10)' in compact(audited_sources['motor_tuning_control'])
codec_code=compact(audited_sources['motor_uart_commands']).split('modulerobot_vofa_uart#',1)[1]
assert 'ack_status<=rsp_status;ack_value<=rsp_value;' in codec_code
stream_indices=[row*16+byte for row in range(16) for byte in range(16)]
assert stream_indices==list(range(256))
assert all(0<=index<256 for index in stream_indices)

# Subpage metadata reaches the display atomically through the same snapshot
# commit as the characters. Both touch contacts and pixel fill use the same
# geometry module, with matching page/subpage inputs and no widened page bus.
ui=source('robot_ui_control.v')
panel=source('robot_control_panel.v')
for instance,x,y,button in (('u_hit0','x0','y0[9:0]','current_button'),
                             ('u_hit1','x1','y1[9:0]','second_button')):
    hit=connections(ui,'robot_ui_hit_test',instance)
    assert hit=={'x':x,'y':y,'page':'page','subpage':'subpage','button':button}
pixel_hit=connections(panel,'robot_ui_hit_test','u_pixel_hit')
assert pixel_hit=={'x':'x','y':'y','page':'page','subpage':'subpage','button':'hit_button'}
assert status_link['page']=='page' and status_link['subpage']=='subpage'
assert robot_link['page']=='robot_page' and robot_link['subpage']=='robot_subpage'
assert 'wire[2:0]page=flags[2:0],subpage=flags[30:28];' in compact(panel)

u=compact(uart)
helper=u.split('modulesensor_demo_text_snapshot_store',1)[1].split('endmodule',1)[0]
assert 'reg[7:0]text_ram[0:2047];' in helper
assert 'if(memory_write)text_ram[memory_address]<=memory_write_data;elseread_data<=text_ram[memory_address];' in helper
assert 'read_valid<=read_request;' in helper
assert 'held_motor_bank<=latest_motor_bank;held_robot_bank<=latest_robot_bank;' in helper
assert 'TEXT_RAM_WAIT:if(text_read_valid)begin' in u
for stream in ('motor','robot'):
    assert stream+'_byte_index==8\'d0?free_bank(latest_'+stream+'_bank,held_'+stream+'_bank,report_active):write_'+stream+'_bank' in helper
    assert "{1'b"+('0' if stream=='motor' else '1')+','+stream+'_input_bank,'+stream+'_byte_index}' in helper
    assert 'if(grant_'+stream+')begin' in helper
    accepted=helper.split('if(grant_'+stream+')begin',1)[1]
    assert "if("+stream+"_byte_index==8'd0)begin" in accepted
    assert 'write_'+stream+'_bank<='+stream+'_input_bank;' in accepted
    assert 'elseif(!'+stream+'_frame_open||'+stream+'_byte_index!='+stream+'_expected_index)begin' in accepted
    assert "elseif("+stream+"_byte_index==8'd255)begin" in accepted
    assert 'latest_'+stream+'_bank<=write_'+stream+'_bank;'+stream+'_seen<=1;' in accepted
    assert '(grant_'+stream+'&&'+stream+'_byte_order_ok)' in helper

# The LCD bank permissions must not depend on valid or the UART grant: the
# UART ready already depends on its valid, so such a dependency would form
# a combinational loop through the top's two-consumer atomic join.
l=compact(lcd)
lcd_helper=l.split('modulelcd_text_byte_snapshot(',1)[1].split('endmodule',1)[0]
bank_helper=l.split('modulelcd_text_snapshot_bank(',1)[1].split('endmodule',1)[0]
assert 'assignbyte_ready=reset_n&&!publish_pending&&!frame_complete;' in lcd_helper
assert "wireordered_byte=byte_index==8'd0||(frame_open&&byte_index==expected_index);" in lcd_helper
assert 'wirebyte_write=byte_accept&&ordered_byte;' in lcd_helper
assert 'always@(posedgewrite_clkornegedgereset_n)' in lcd_helper
assert 'always@(posedgeread_clkornegedgereset_n)' in lcd_helper
assert 'negedgeread_run' not in lcd_helper
assert 'write_bank<=1\'b1;publish_pending<=0;frame_open<=0;frame_complete<=0;' in lcd_helper
assert 'request_sync<=0;snapshot_ack<=0;active_bank<=0;read_bank_q<=0;' in lcd_helper
assert 'ack_sync<={ack_sync[1:0],snapshot_ack};' in lcd_helper
assert 'request_sync<={request_sync[1:0],snapshot_request};' in lcd_helper
assert 'if(publish_pending&&snapshot_request==ack_sync[2])beginpublish_pending<=0;write_bank<=~write_bank;end' in lcd_helper
assert "if(byte_index==8'd0)beginframe_open<=1;expected_index<=8'd1;" in lcd_helper
assert "if(byte_index==8'd255)beginframe_open<=0;frame_complete<=1;" in lcd_helper
assert 'if(frame_commit&&frame_complete&&!publish_pending)begin' in lcd_helper
assert 'snapshot_bank<=write_bank;snapshot_meta<=frame_meta;snapshot_request<=~snapshot_request;publish_pending<=1;frame_complete<=0;' in lcd_helper
assert 'if(read_run&&frame_restart&&request_sync[2]!=snapshot_ack)begin' in lcd_helper
assert 'active_bank<=snapshot_bank;read_meta<=snapshot_meta;read_seen<=1;snapshot_ack<=request_sync[2];' in lcd_helper
assert 'if(read_run&&read_seen&&read_en)read_bank_q<=active_bank;' in lcd_helper
assert "assignread_byte=read_seen&&read_run?(read_bank_q?bank1_q:bank0_q):8'h20;" in lcd_helper
for number in (0,1):
    link=connections(lcd,'lcd_text_snapshot_bank','u_bank'+str(number))
    assert set(link)==lcd_ports['lcd_text_snapshot_bank']
    assert link['reset_n']=='reset_n'
    assert link['write_clk']=='write_clk' and link['read_clk']=='read_clk'
    assert link['write_enable']=='byte_write&&'+('!' if number==0 else '')+'write_bank'
    assert link['read_enable']=='read_run&&read_seen&&read_en&&'+('!' if number==0 else '')+'active_bank'
    assert link['write_index']=='byte_index' and link['read_index']=='read_index'
    assert link['write_data']=='byte_data' and link['read_data']=='bank'+str(number)+'_q'
dpb=connections(lcd,'DPB','u_ram')
dpb_ports=set('DOA DOB DIA DIB BLKSELA BLKSELB ADA ADB WREA WREB CLKA CLKB CEA CEB OCEA OCEB RESETA RESETB'.split())
assert set(dpb)==dpb_ports
assert dpb['CEA']=='reset_n&&write_enable' and dpb['CEB']=='reset_n&&read_enable'
assert dpb['CLKA']=='write_clk' and dpb['CLKB']=='read_clk'
assert dpb['WREA']=="1'b1" and dpb['WREB']=="1'b0"
assert dpb['OCEA']==dpb['OCEB']=="1'b1"
assert dpb['RESETA']==dpb['RESETB']=="1'b0"
assert dpb['ADA']=="{3'b000,write_index,3'b000}" and dpb['ADB']=="{3'b000,read_index,3'b000}"
assert dpb['DIA']=="{8'd0,write_data}" and dpb['DIB']=="16'd0"
for parameter,value in dict(BIT_WIDTH_0='8',BIT_WIDTH_1='8',WRITE_MODE0="2'b00",WRITE_MODE1="2'b00",
                            READ_MODE0="1'b0",READ_MODE1="1'b0",RESET_MODE='"SYNC"').items():
    assert '.'+parameter+'('+value+')' in bank_helper
# The RAM output itself is the existing character A register; only the
# source selector is registered before glyph B, preserving four pixel stages.
assert 'assignbyte_text_read_index={robot_label_row_l,char_ix};' in l
assert 'assignrobot_byte_read_en=BYTE_TEXT_MODE&&run&&in_label_l&&robot_control_l&&robot_label_source_l==3\'d1;' in l
assert 'assignpid_byte_read_en=BYTE_TEXT_MODE&&run&&in_label_l&&robot_control_l&&robot_label_source_l==3\'d2;' in l
assert "robot_byte_source_a<=BYTE_TEXT_MODE&&robot_control_l?(robot_label_source_l==3'd1?2'd1:robot_label_source_l==3'd2?2'd2:2'd0):2'd0;" in l
assert 'robot_byte_source_a<=0;' in l
assert "wire[7:0]glyph_dynamic_character_a=robot_byte_source_a==2'd1?robot_byte_character:robot_byte_source_a==2'd2?pid_byte_character:character_a;" in l
assert 'font_b<=glyph(glyph_character_a);' in l
assert not re.search(r'\b\w+\s*<=\s*(?:robot|pid)_byte_character\s*;',lcd)
assert 'if(!ROBOT_ACTIVE&&robot_text_valid&&robot_status_request==robot_status_ack_sync[2])' in compact(top)
assert 'if(!ROBOT_ACTIVE&&motor_text_valid&&pid_text_request==pid_text_ack_sync[2])' in compact(top)

# Exhaust the logical ownership boundary, independently of cycle schedules.
# Starting a report and accepting byte0/255 on the same boundary is included.
def free_bank(latest,held,active):
    return next(bank for bank in range(3) if bank!=latest and (not active or bank!=held))

ownership_cases=0
for latest,held,active,begin,index in product(range(3),range(3),(False,True),(False,True),(0,1,254,255)):
    if active and begin:
        continue
    for writer in range(3):
        if writer==latest or (active and writer==held):
            continue
        incoming=free_bank(latest,held,active) if index==0 else writer
        report_bank=latest if begin else held
        assert incoming!=latest
        if active or begin:
            assert incoming!=report_bank
        # A terminal byte can publish only its existing unpublished writer.
        published=incoming if index==255 else latest
        if begin:
            assert report_bank==latest
        assert 0<=published<3
        ownership_cases+=1

arbitration_cases=0
for reset,mode,read,mvalid,rvalid,rr in product((False,True),repeat=6):
    gm=not read and mvalid and (not mode or not rvalid or not rr)
    gr=not read and mode and rvalid and (not mvalid or rr)
    mready=reset and gm
    rready=reset and (not mode or gr)
    assert not (gm and gr)
    if not reset:
        assert not mready and not rready
    if read:
        assert not gm and not gr and not mready
        if mode:
            assert not rready
    if reset and not read and mvalid and (not mode or not rvalid):
        assert mready
    if reset and not read and mode and mvalid and rvalid:
        assert mready!=rready
        # A serviced stream passes the next tie to the other one.
        next_rr=bool(gm)
        assert next_rr!=rr
    if reset and not mode:
        assert rready and not gr  # disabled NAV is explicitly drained/discarded
    arbitration_cases+=1

# Exhaust the Boolean atomic-fork contract, including the UART's valid-
# dependent ready. LCD permission is an independent registered-state input.
# These are logical transfer truth tables, not a clocked RTL simulation.
join_cases=0
for reset,test,mode,read,mvalid,rvalid,mlcd,rlcd,rr in product((False,True),repeat=9):
    active=mode and not test
    muvalid=not test and mvalid and (not active or mlcd)
    ruvalid=active and rvalid and rlcd
    gm=not read and muvalid and (not active or not ruvalid or not rr)
    gr=not read and active and ruvalid and (not muvalid or rr)
    mready=reset and not test and gm
    rready=reset and not test and (not active or gr)
    producer_mready=True if test else mready and (not active or mlcd)
    producer_rready=rready and rlcd if active else True
    motor_uart=muvalid and mready
    robot_uart=ruvalid and rready
    motor_lcd=active and mvalid and mready and mlcd
    robot_lcd=active and rvalid and rready and rlcd
    if active:
        assert motor_uart==motor_lcd==(mvalid and producer_mready)
        assert robot_uart==robot_lcd==(rvalid and producer_rready)
    elif not test:
        assert motor_uart==(mvalid and producer_mready)
        assert not motor_lcd and not robot_lcd and not robot_uart
    else:
        assert not motor_uart and not robot_uart and not motor_lcd and not robot_lcd
    join_cases+=1

# The active VOFA branch grants both drains unconditionally. A stalled LCD
# still blocks its producer, and accepted bytes occur once at that LCD only.
vofa_join_cases=0
for valid,lcd_ready in product((False,True),repeat=2):
    drain_ready=True
    producer_ready=drain_ready and lcd_ready
    lcd_accept=valid and drain_ready and lcd_ready
    drained=valid and lcd_ready and drain_ready
    assert lcd_accept==drained==(valid and producer_ready)
    vofa_join_cases+=1

# Signed19 processed values cover every raw16 conversion and survive the
# explicit13-bit sign extension; the invalid sentinel cannot be valid IMU.
imu_cases=0
for encoded in range(65536):
    raw=encoded if encoded<32768 else encoded-65536
    for processed in (raw*7,(-1 if raw<0 else 1)*((abs(raw)*122+500)//1000)):
        assert -(1<<18)<=processed<(1<<18)
        packed=processed&((1<<19)-1)
        extended=packed|(((1<<13)-1)<<19) if packed&(1<<18) else packed
        decoded=extended-(1<<32) if extended&(1<<31) else extended
        assert decoded==processed and extended!=0x80000000
        imu_cases+=1

# Physical-bank ownership across publication, paused raster, switch and ACK.
# A pause retains tokens/banks; it must never make the active bank writable.
bank_boundaries=0
for initial_reader in (0,1):
    for paused in (False,True):
        reader=initial_reader
        writer=1-reader
        assert writer!=reader
        published=writer
        can_start_next=False
        if paused:
            assert reader!=writer and not can_start_next
        reader=published
        # After the read-domain switch the writer is blocked until ACK; on
        # ACK its bank flips, so no write has both active read/write owners.
        assert not can_start_next
        writer=1-writer
        can_start_next=True
        assert writer!=reader and can_start_next
        bank_boundaries+=1

print('PASS: top/subsystem/2 formatters/UART/LCD byte wiring; dual-domain common reset; inactive-mode drain')
print(f'PASS: held EMIT, frame/meta commit, Normal RAM/DPB CE, A2-to-B3 character path; {ownership_cases} UART ownership and {arbitration_cases} arbitration boundaries')
print(f'PASS: {join_cases} two-consumer transfer truth cases and {bank_boundaries} LCD bank/pause/ACK ownership boundaries')
print(f'PASS: {module_count} module declarations/{len(all_instances)} complete named-port instances and block pairs; 3 shared UI hit tests; 8/10 numeric fields and configured speed/position PID/C1 capture routes in256 ordered bytes')
print(f'PASS: VOFA command/adapter/readback/physical TX route, legacy generate-only RAM, {vofa_join_cases} LCD-only drain cases and {imu_cases} processed IMU sign-extension cases')
print('NO HDL compilation/simulation, inferred-netlist/RAM verification, timing, resource or board validation')
