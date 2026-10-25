#!/usr/bin/env python3
"""BQ static integration and independent unit/byte arithmetic; no HDL validation."""
from pathlib import Path
import re
from check_motor_integration import uncomment

ROOT=Path(__file__).resolve().parents[1]
master=uncomment((ROOT/'src/bq34z100_i2c_master.v').read_text(encoding='utf-8'))
gauge=uncomment((ROOT/'src/bq34z100_gauge.v').read_text(encoding='utf-8'))
status=uncomment((ROOT/'src/bq34z100_status_text.v').read_text(encoding='utf-8'))
setup=uncomment((ROOT/'src/bq34z100_board_setup.v').read_text(encoding='utf-8'))
top=uncomment((ROOT/'src/vision_robot_ui_top.v').read_text(encoding='utf-8'))
lcd=uncomment((ROOT/'src/lcd1024_interaction_ui.v').read_text(encoding='utf-8'))
panel=uncomment((ROOT/'src/actuator_test_panel.v').read_text(encoding='utf-8'))
control=uncomment((ROOT/'src/actuator_test_control.v').read_text(encoding='utf-8'))

for pin in ('scl','sda'):
    assert re.search(rf'assign\s+{pin}\s*=\s*{pin}_low\s*\?\s*1\x27b0\s*:\s*1\x27bz',master)
assert "tx_byte<=8'haa" in master and "tx_byte<=8'hab" in master
assert 'BUS_FREE_CYCLES=US_CYCLES*66' in master
assert 'STRETCH_TIMEOUT_MS=50' in master and 'TRANSACTION_TIMEOUT_MS=2000' in master
assert 'scl_sync[1]' in master and 'recovery_count==8' in master
assert 'read_data[byte_index*8 +:8]<=' in master
assert 'read_count>32' in master and 'write_count>2' in master
assert 'held_write_data[15:8]' in master and 'WRITE_DATA' in master
assert re.search(r"0:begin register_address<=8'h02;read_count<=1;end",gauge)
for register in ('08','0a','0c','0e','2a','3a'):
    assert f"register_address<=8'h{register};read_count<=2" in gauge
assert 'STARTUP_DELAY_MS=300' in gauge and 'STALE_MS=3000' in gauge
assert "error_code<=8'h80" in gauge
assert "error==8'h80 || error==0" in status and 'error==1 ||' not in status
assert top.count('bq34z100_gauge u_bq_')==2
for suffix in ('a','b'):
    assert f'.scl(bq_{suffix}_scl),.sda(bq_{suffix}_sda)' in top
    assert f"assign bq_{suffix}_scl=1'bz" not in top
assert 'battery_status_mailbox<={battery_valid_flags,battery_text}' in top
assert 'frame_restart && battery_request_sync[2]!=battery_status_ack' in lcd
assert 'reg [1921:0] battery_status_mailbox' in top
assert 'wire [1919:0] battery_text' in top
assert 'input wire [1921:0] battery_status_data' in lcd
assert '.battery_rows(battery_status[1919:1280])' in lcd
assert '.diagnostic_rows(battery_status[1279:0])' in lcd
assert 'reg [2064:0] test_status_mailbox' in top
assert 'test_status_mailbox<={bq_info,test_status_flags,test_status_text}' in top
assert '.bq_info(test_status[2064])' in lcd
assert 'rect(x,y,884,996,60,88)' in control
assert 'button(884,996,60,88' in panel
assert 'action==26 && touch_ready) bq_info<=!bq_info' in control
assert 'DF/FW FROM SETUP' in panel
assert 'cfg_busy ? cfg_start : i2c_start' in gauge
assert 'if (begin_setup && !setup_busy)' in gauge
assert 'field==8' in gauge and 'candidate_internal' in gauge
# Seven standard reads plus the CONTROL_STATUS command/read must complete
# before any live PackConfiguration/ControlStatus pair is published.
assert "7:begin register_address<=8'h00;read_count<=2;poll_read<=0;end" in gauge
assert "default:begin register_address<=8'h00;read_count<=2;end" in gauge
assert '.read_not_write(cfg_busy ? cfg_read : poll_read)' in gauge
assert "cfg_busy ? cfg_write_data : 16'h0000" in gauge
assert 'control_delay_count<=0;field<=8;state<=CONTROL_DELAY' in gauge
assert 'control_delay_count>=(CLK_HZ/1000)*10-1' in gauge
assert 'state==WAIT_READ || state==CONTROL_DELAY' in gauge
assert 'pack_configuration<=candidate_pack;control_status<=read_data' in gauge
for runtime_error in ('ea','eb','ec'):
    assert f"configured<=0;runtime_error<=8'h{runtime_error}" in gauge
assert 'candidate_voltage<8000 || candidate_voltage>14000' in gauge
assert "(candidate_pack&16'h0801)!=16'h0801 || read_data[12]" in gauge
assert 'valid_a && pack_a[0]' in status and 'valid_b && pack_b[0]' in status
assert 'held_fresh<={valid_a,valid_b}' in status
assert 'held_configured<={configured_a,configured_b}' in status
assert 'valid_flags<=held_fresh & held_configured' in status
assert 'fresh ? "OK " : "CHK"' in status
assert 'output reg [1919:0] text_rows' in status
assert "raw>=16'd2732 ? raw-16'd2731 : 16'd2732-raw" in status
assert 'reg [15:0] values[0:11]' in status
assert 'reg [19:0] digits[0:11]' in status
assert 'if (bit_count==15) state<=STORE' in status
assert 'if (field==11) state<=PACK' in status
assert 'text_valid<=1;state<=IDLE' in status
assert 'result[119:112]=divider[15:8]' in setup
assert 'result[127:120]=divider[7:0]' in setup
assert 'if (df_verifying && !byte_equal) df_readback_match<=0' in setup
assert 'if (!df_readback_match)' in setup
assert 'response[7:0]!=(8\'hff-df_sum)' in setup
assert 'restore_required' in setup and "16'h0041" in setup
# Calibration permission is DF PackConfiguration bit 14, not the active
# calibration status bit 12. Check the separate read/restore paths, and final
# checksum-validated block audits after configuration/cleanup.
assert 'original_cal_permission<=block_data[6]' in setup
assert "CAL_PERMISSION_READ:issue_read(8'h3a,2" in setup
assert 'if (response[14]) state<=CAL_ENTER' in setup
assert 'response[14]!=original_cal_permission' in setup
assert 'block_data[6]!=original_cal_permission' in setup
assert 'else if (board_write_pending && error_code==0 && !recovery_only) state<=WRITE_DIVIDER' in setup
for bank,field_name,next_state in ((0,'CAL','FINAL_CONFIG'),(1,'CONFIG','FINAL_POWER'),(2,'POWER','FINAL_POWER_OK')):
    assert f'check_audit({bank},{next_state},FINAL_{field_name}_CHECK)' in setup
assert 'audit_bytes [0:127]' in setup
assert 'audit_valid[bank]<=0' in setup and 'audit_valid[audit_bank]<=1' in setup
assert 'failure_phase<=audit_failure_phase;state<=FAILURE' in setup
assert 'block_data!=expected_data' not in setup
assert 'STABILITY_MV=250' in setup
assert 'pause_ms(2000,STABLE_CONTROL)' in setup
assert 'pause_ms(1000,STABLE_CONTROL)' in setup
assert 'stable_samples==2' in setup
assert 'stable_min' in setup and 'stable_max' in setup
# These commands must remain absent: chemistry/model/learning is a separate
# commissioning procedure, never implied by the local hardware SETUP button.
assert "16'h0021" not in setup  # IT_ENABLE

# Expected wire byte order and signed conversion; no bus activity is performed.
for low,high,expected in ((0,0,0),(0xff,0,255),(0,1,256),(0xff,0x7f,32767),(0,0x80,32768),(0xff,0xff,65535)):
    raw=int.from_bytes(bytes((low,high)),'little')
    assert raw==expected
    signed=int.from_bytes(bytes((low,high)),'little',signed=True)
    assert signed==(expected if expected<32768 else expected-65536)
from check_bq34_display_math import check_display_mapping, rounded_reference
mapped_cases=check_display_mapping()
for tag,full in (('A',83),('B',88)):
    for raw in (0,1,17,18,19,78,83,88,100,255):
        soc=rounded_reference(raw,full)
        for millivolts in (0,9000,11100,12600,65535):
            cv=millivolts//10
            prefix='LOW' if soc<=20 else 'MAP'
            text=f'{tag} {prefix}{soc:03d}% {cv//100:02d}.{cv%100:02d}V'
            assert len(text)==16
    for error in (1,2,3,4,5,0x10,0x11):
        assert len(f'{tag} BQ ERR 0X{error:02X}   ')==16
assert 50000000//1000000*66==3300
assert 50000000//1000*300==15000000
assert 50000000//1000*50==2500000

# Independent hardware and wire-format arithmetic, NOT a model of the RTL FSM.
from fractions import Fraction
ratio=Fraction(240000+16500,16500)
assert round(ratio*1000)==15545
assert Fraction(12600,1000)/ratio < Fraction(9,10)
assert (15545).to_bytes(2,'big')==bytes.fromhex('3c b9')
for original_word in range(0x10000):
    updated=original_word|0x0801
    assert updated & 0x0801 == 0x0801
    assert updated & ~0x0801 == original_word & ~0x0801
for seed in range(256):
    block=bytearray((seed+i*7)&255 for i in range(32))
    before=bytes(block)
    block[14:16]=(15545).to_bytes(2,'big')
    assert block[:14]==before[:14] and block[16:]==before[16:]
    checksum=(255-sum(block))&255
    assert (sum(block)+checksum)&255 == 255
    broken=bytearray(block);broken[6]^=1
    assert (sum(broken)+checksum)&255 != 255
for raw,expected in ((2731,-0.1),(2732,0.1),(2981,25.0),(2631,-10.1)):
    centi=raw*10-27315
    tenth=(centi+5)//10 if centi>=0 else -((-centi+5)//10)
    assert tenth/10==expected
for tag in ('A','B'):
    assert len(f'{tag} N~+25.0 I+27.2')==16
assert len('A:OK  B:OK  NTC~')==16
assert len('A:CHK B:EEA NTC~')==16

# Diagnostic rows must fit the existing 16-character LCD cells at both
# numeric extremes. DF/FW values are cached setup diagnostics, not live polls.
for tag in ('A','B'):
    for value in (0,0x0801,0x3cb9,0xffff):
        diagnostic_rows=(
            f'{tag} V={value:04X} P={value:04X} ',
            f'{tag} CS={value:04X} F={value&255:02X}  ',
            f'{tag} DF={value:04X} N={value&255:02X}  ',
            f'{tag} VD={value:05d} R0X{value&255:02X}',
            f'{tag} SV{value:05d} FW{value:04X}',
        )
        assert all(len(row)==16 for row in diagnostic_rows)

# Independent acceptance-policy examples, not an HDL state-machine model.
for samples,expected in (
    ((8000,8000,8000),True),
    ((14000,14000,14000),True),
    ((11000,11250,11100),True),
    ((11000,11251,11100),False),
    ((11000,10900,11151),False),
    ((7999,8000,8001),False),
    ((13999,14000,14001),False),
):
    acceptable=all(8000<=sample<=14000 for sample in samples) and max(samples)-min(samples)<=250
    assert acceptable is expected
print(f'PASS: dual BQ integration, nine-transaction polling, diagnostics, setup guards; {mapped_cases} full-point display mappings with rounded LOW/stale/raw-hex and byte/unit arithmetic')
print('NO HDL compilation/simulation, physical I2C, battery configuration or SOC calibration validation')
