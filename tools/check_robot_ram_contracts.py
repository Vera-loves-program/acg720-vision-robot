"""Static GW5AT-60 RAM compatibility and ownership contracts.

Regression for PA2122: no read-before-write is requested by the robot RAM
templates. Checks explicit DPB mode/CE pins and the LCD bank ownership
boundaries. This does NOT compile, elaborate or simulate HDL, inspect the
encrypted netlist, or prove that synthesis/timing/hardware has passed.
"""
from pathlib import Path
import re
from check_motor_integration import uncomment, check_blocks, closing, instances

ROOT=Path(__file__).resolve().parents[1]
grid=uncomment((ROOT/'src/lidar_grid_map.v').read_text(encoding='utf-8-sig'))
planner=uncomment((ROOT/'src/lidar_path_planner.v').read_text(encoding='utf-8-sig'))
setup=uncomment((ROOT/'src/bq34z100_board_setup.v').read_text(encoding='utf-8-sig'))
uart=uncomment((ROOT/'src/sensor_demo_uart.v').read_text(encoding='utf-8-sig'))
panel=uncomment((ROOT/'src/robot_control_panel.v').read_text(encoding='utf-8-sig'))
status=uncomment((ROOT/'src/robot_status_text.v').read_text(encoding='utf-8-sig'))
check_blocks(grid,'lidar_grid_map')
check_blocks(planner,'lidar_path_planner')
check_blocks(setup,'bq34z100_board_setup')
check_blocks(uart,'sensor_demo_uart')
check_blocks(panel,'robot_control_panel')
check_blocks(status,'robot_status_text')
g=re.sub(r'\s+','',grid)
p=re.sub(r'\s+','',planner)
b=re.sub(r'\s+','',setup)
u=re.sub(r'\s+','',uart)
r=re.sub(r'\s+','',panel)
st=re.sub(r'\s+','',status)

# A write and a read on the same edge would request unsupported OLD-data
# read-before-write. A preceding read state already provides the old value.
for code,statement in (
    (g,'if(map_we)cells[map_addr]<=map_wdata;elsemap_rdata<=cells[map_addr];'),
    (g,'if(ray_we)rays[ray_waddr]<=ray_wdata;elseray_rdata<=rays[ray_raddr];'),
    (p,'if(node_write_enable)nodes[node_address]<=node_write_data;elsenode_read_data<=nodes[node_address];'),
    (p,'if(queue_write_enable)queue[queue_write_address]<=queue_write_data;elsequeue_read_data<=queue[queue_head[11:0]];'),
    (b,'if(state==AUDIT_STORE)audit_bytes[{audit_bank,df_index[4:0]}]<=audit_write_byte;elseaudit_byte<=audit_bytes[{audit_bank,df_index[4:0]}];'),
    (u,'if(memory_write)text_ram[memory_address]<=memory_write_data;elseread_data<=text_ram[memory_address];'),
):
    assert statement in code, statement
assert 'S_RAY_READ:state<=S_RAY_WRITE;' in g
assert 'S_MREQ:state<=S_MUSE;' in g
assert 'START_REQ:state<=START_CHECK;' in p
assert 'NREQ' in p and 'NREAD' in p
assert 'QWAIT:state<=QLATCH;' in p
assert 'AUDIT_READ:state<=AUDIT_COMPARE;' in b
assert 'if(!audit_valid[bank])begin' in b
assert 'audit_valid[bank]<=0;df_index<=0;state<=AUDIT_STORE;' in b
assert 'audit_valid[audit_bank]<=1;df_index<=0;state<=audit_return;' in b
assert len(re.findall(r'\baudit_bytes\s*\[',setup))==3
assert len(re.findall(r'\btext_ram\s*\[',uart))==3
assert 'always@(posedgeclk)character<=bytes[{label_id,character_index}];' in r
assert '(*syn_romstyle="block_rom"*)reg[7:0]bytes[0:2047];' in r
# The10Hz status formatter uses a separate synchronous read-only byte ROM.
# It must retain a real read-enable (Q holds under stream backpressure), no
# asynchronous reset/read, and loop-bound-safe complete SPACE initialization.
if 'labels[0:2047]' in st:
    assert '(*syn_romstyle="block_rom"*)reg[7:0]labels[0:2047];' in st
    assert 'S_REQUEST:beginlabel_byte<=labels[{current_label,column}];state<=S_SEND;end' in st
    assert "for(init_index=0;init_index<1024;init_index=init_index+1)beginlabels[init_index]=8'h20;labels[init_index+1024]=8'h20;end" in st
    assert 'labels[' not in st.split('always@(posedgeclk',1)[-1].replace('labels[{current_label,column}]','')
else:
    status_rom=st.split('modulerobot_status_label_rom(',1)[1].split('endmodule',1)[0]
    assert '(*syn_romstyle="block_rom"*)reg[7:0]memory[0:4095];' in status_rom
    assert 'inputwire[11:0]address' in status_rom
    assert 'always@(posedgeclk)if(read_enable)data<=memory[address];' in status_rom
    assert 'negedge' not in status_rom and 'always@*' not in status_rom
    assert "for(init_index=0;init_index<1024;init_index=init_index+1)beginmemory[init_index]=8'h20;memory[init_index+1024]=8'h20;memory[init_index+2048]=8'h20;memory[init_index+3072]=8'h20;end" in status_rom
assert '(*syn_ramstyle="block_ram"*)reg[7:0]text_ram[0:2047];' in u
assert 'wiregrant_motor=!read_request&&' in u and 'wiregrant_robot=!read_request&&' in u
assert 'read_valid<=read_request;' in u
# UART owns exactly one Normal port. Byte grants consume an input, but only
# sequential byte0/open+expected transfers may WRITE: after commit writebank
# equals latest, so malformed continuation bytes must never touch that bank.
uart_helper=u.split('modulesensor_demo_text_snapshot_store',1)[1].split('endmodule',1)[0]
assert 'assignmotor_byte_ready=reset_n&&grant_motor;' in uart_helper
assert 'assignrobot_byte_ready=reset_n&&(!ROBOT_MODE||grant_robot);' in uart_helper
assert 'wirememory_write=reset_n&&((grant_motor&&motor_byte_order_ok)||(grant_robot&&robot_byte_order_ok));' in uart_helper
assert "wiremotor_byte_order_ok=motor_byte_index==8'd0||(motor_frame_open&&motor_byte_index==motor_expected_index);" in uart_helper
assert "wirerobot_byte_order_ok=robot_byte_index==8'd0||(robot_frame_open&&robot_byte_index==robot_expected_index);" in uart_helper
assert "wire[1:0]motor_input_bank=motor_byte_index==8'd0?free_bank(latest_motor_bank,held_motor_bank,report_active):write_motor_bank;" in uart_helper
assert "wire[1:0]robot_input_bank=robot_byte_index==8'd0?free_bank(latest_robot_bank,held_robot_bank,report_active):write_robot_bank;" in uart_helper
assert "{1'b0,motor_input_bank,motor_byte_index}" in uart_helper
assert "{1'b1,robot_input_bank,robot_byte_index}" in uart_helper
assert '[2047:0]' not in u and 'motor_text' not in uart_helper and 'robot_text' not in uart_helper
assert '-:8' not in uart_helper and 'DPB' not in uart_helper

# The live control memory now has just its declaration, control write and
# control read. No LCD-domain read remains on the same physical memory.
assert len(re.findall(r'\bcells\s*\[',grid))==3
assert 'outputwire[1:0]rd_cell' in g
assert 'assignrd_cell=display_read_bank?snapshot_read_1:snapshot_read_0;' in g
assert 'display_read_bank<=display_active_bank;' in g
assert 'rd_valid<=rd_en&&display_valid_sync[2];' in g

# Explicit primitive parameters remove write-mode inference from the mirror.
def arguments(body):
    result={}
    for match in re.finditer(r'\.\s*(\w+)\s*\(',body):
        opening=match.end()-1
        assert match[1] not in result,match[1]
        result[match[1]]=re.sub(r'\s+','',body[opening+1:closing(body,opening)])
    return result

match=re.search(r'\bDPB\s*#\s*\(',grid)
assert match
opening=match.end()-1
end=closing(grid,opening)
parameters=arguments(grid[opening+1:end])
for name,value in dict(WRITE_MODE0="2'b00",WRITE_MODE1="2'b00",
                       READ_MODE0="1'b0",READ_MODE1="1'b0",
                       BIT_WIDTH_0='1',BIT_WIDTH_1='1',
                       BLK_SEL_0="3'b000",BLK_SEL_1="3'b000",
                       RESET_MODE='"SYNC"').items():
    assert parameters[name]==value,(name,parameters.get(name))
body_open=grid.index('(',end+1)
connections=arguments(grid[body_open+1:closing(grid,body_open)])
primitive_ports=set('DOA DOB DIA DIB BLKSELA BLKSELB ADA ADB WREA WREB CLKA CLKB CEA CEB OCEA OCEB RESETA RESETB'.split())
actual=list(instances(grid,{'DPB':primitive_ports}))
assert len(actual)==1 and set(actual[0][2])==primitive_ports
assert len(actual[0][2])==len(primitive_ports)
assert connections['CEA']=='reset_n&&write_enable'
assert connections['CEB']=='reset_n&&read_enable'
assert connections['OCEA']==connections['OCEB']=="1'b1"
assert connections['WREA']=="1'b1" and connections['WREB']=="1'b0"
assert connections['RESETA']==connections['RESETB']=="1'b0"
assert connections['ADA']=='physical_write_address'
assert connections['ADB']=='physical_read_address'
assert connections['DIA']=="{15'd0,write_data[plane]}"
assert 'for(plane=0;plane<2;plane=plane+1)' in g
assert "{{(14-ADDRESS_BITS){1'b0}},write_address}" in g

# Each bank's real CE is mutually exclusive by ownership. The OLD bank may
# become writable only after the LCD has switched and ACK has returned.
for connection in (
    '.write_enable(snapshot_copy_enable&&!snapshot_write_bank)',
    '.write_enable(snapshot_copy_enable&&snapshot_write_bank)',
    '.read_enable(display_read_enable&&!display_active_bank)',
    '.read_enable(display_read_enable&&display_active_bank)',
):
    assert connection in g,connection
assert 'snapshot_request<=0;snapshot_write_bank<=1;' in g
assert 'snapshot_ack<=0;display_active_bank<=0;' in g
assert 'snapshot_ack_sync<={snapshot_ack_sync[1:0],snapshot_ack};' in g
assert 'snapshot_request_sync<={snapshot_request_sync[1:0],snapshot_request};' in g
assert 'if(snapshot_request_sync[2]!=snapshot_ack&&!rd_en)begin' in g
assert 'display_active_bank<=snapshot_request_sync[2];snapshot_ack<=snapshot_request_sync[2];' in g
finish=g.split('S_FINISH:begin',1)[1].split('S_SNAPSHOT_READ:',1)[0]
assert 'if(snapshot_request==snapshot_ack_sync[2])begin' in finish
assert 'elsebeginbusy<=0;state<=S_IDLE;end' in finish
assert 'S_SNAPSHOT_READ:state<=S_SNAPSHOT_WRITE;' in g
assert 'snapshot_request<=!snapshot_request;snapshot_write_bank<=!snapshot_write_bank;' in g
clear=g.split('if(clear_map)begin',2)[-1].split('endelsecase(state)',1)[0]
assert 'snapshot_epoch_valid<=0;snapshot_display_valid<=0;' in clear
for variable in ('snapshot_request','snapshot_write_bank','snapshot_ack','display_active_bank'):
    assert variable+'<=' not in clear,variable
assert 'snapshot_request==snapshot_ack_sync[2]&&snapshot_epoch_valid' in g

# Independent ownership invariant for any number of publications and clears.
# Stages are logical ownership boundaries, not a model of HDL clock events.
reader=0;request=0;writer=1
for publication in range(256):
    assert writer!=reader  # copy can only touch the non-reader bank
    complete_bank=writer
    request^=1;writer^=1
    assert complete_bank==request
    # No further write is permitted while the acknowledgement is outstanding.
    for cleared in (False,True):
        # Clear hides an epoch, but must never reset the ownership tokens.
        reader_after_switch=request
        assert writer!=reader_after_switch
    reader=request  # acknowledgement has returned before the next iteration
assert 2*16384==32768
assert 32768/50_000_000<0.000656
assert 2*2*16384//8==8192
print('PASS: six Normal RAM templates including BQ/UART; two synchronous byte-ROM templates; explicit DPB mode/CE and port isolation')
print('PASS: snapshot bank ownership, clear/publish boundaries, one-cycle read selector and 0.655ms copy arithmetic')
print('NO HDL compilation/simulation, inferred-netlist verification, placement/routing or hardware validation')
