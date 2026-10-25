"""Core-profile pruning and retained I/O/math contracts, offline only."""
from pathlib import Path
import hashlib
import json
import re

ROOT=Path(__file__).resolve().parents[1]
top=(ROOT/'src/vision_robot_ui_top.v').read_text(encoding='utf-8')
gauge=(ROOT/'src/bq34z100_gauge.v').read_text(encoding='utf-8')
text=(ROOT/'src/bq34z100_status_text.v').read_text(encoding='utf-8')
assert top.count('bq34z100_gauge #(.ENABLE_SETUP(!ROBOT_ACTIVE))')==2
assert 'if(ENABLE_SETUP!=0)' in gauge
assert 'if(!ROBOT_ACTIVE)begin:legacy_clock_display' in top
assert 'if(!ROBOT_ACTIVE)begin:legacy_motor_report' in top
assert 'if(SEND_CAMERA_UDP!=0)begin:camera_udp_fifo' in top
for token in ['u_framebuffer','u_camera_init','u_dvp','u_imu_demo','u_motors','u_vofa','u_udp_rx','u_robot']:
    assert token in top,token
assert '.MINIMAL(ROBOT_ACTIVE)' in top
assert 'reg [15:0] values[0:3]' in text
assert 'configured_a' not in text.split('begin:core_voltage_soc',1)[1].split('end else begin:legacy_full_dashboard',1)[0]
assert 'bq_valid_a && bq_error_a==0' in top


def decimal16(value):
    # Reference the compact hardware's shared16-step double-dabble.
    digits=[0]*5
    for bit in range(15,-1,-1):
        for i in range(5):
            if digits[i]>=5:digits[i]+=3
        carry=(value>>bit)&1
        for i in range(4,-1,-1):
            old=digits[i];digits[i]=((old<<1)|carry)&15;carry=old>>3
    return ''.join(map(str,digits))


for mv in range(65536):
    assert decimal16(mv)==f'{mv:05d}'
for full in (83,88):
    for raw in range(256):
        count=100 if raw>=full else (raw*100+full//2)//full
        assert 0<=count<=100
        assert decimal16(count)==f'{count:05d}'
for name,digest in json.loads((ROOT/'tmp/core-route-20261025/protected.json').read_text()).items():
    assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest,name
print('PASS: core pruning explicit, FPGA camera/IMU/motor/VOFA/network retained, raw battery freshness gates low-poweroff')
print('PASS:65536 voltage/BCD values;512 relative-SOC scales; all project/constraint/process hashes unchanged')
print('Static and independent numeric checks; no service/device actions or dynamic HDL simulation.')
