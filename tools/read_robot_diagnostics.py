"""Read passive operation10 snapshots. Opens a serial port ONLY when run by user.

Close VOFA first: the same Bluetooth UART cannot be owned by two programs.
No RUN/STOP/PID/pose/clear-fault command is issued by this utility.
"""
import argparse
import json
from pathlib import Path
import re
import time

REASONS={0:'no live rejection classified; inspect mission/planner errors',1:'scan communication/config not ready',
 2:'complete scan expired',3:'gyro not valid/calibrated',4:'gyro expired',
 5:'projection/scan processing overrun',6:'required direction measurement UNKNOWN',
 7:'confirmed near obstacle',8:'virtual near contact',9:'map not world mode',
 10:'waiting map integration',11:'start UNKNOWN',12:'start OCCUPIED',
 13:'goal UNKNOWN',14:'goal OCCUPIED',15:'no known route',16:'confirmed NAV lookahead hit'}

MISSION_ERRORS={0:'none',13:'map refresh timeout',14:'pose unavailable',15:'map unavailable',
 16:'virtual contact',17:'precise final leg blocked',18:'NAV sensors stale',
 19:'NAV fault',20:'coordinates outside map',21:'motion permit unavailable'}
def signed(x,bits=32):
    x &= (1<<bits)-1
    return x-(1<<bits) if x & (1<<(bits-1)) else x

def decode_words(w):
    if len(w)!=48 or w[0]!=0x4c440101:
        raise ValueError('Snapshot header/version mismatch; load the matching FPGA build.')
    reason=w[39]&255
    return dict(reason_code=reason,reason=REASONS.get(reason,'unclassified'),
      nav_status=w[1]&15,nav_fault=w[1]>>24,mission_error=(w[1]>>16)&255,planner_error=(w[1]>>8)&255,
      sensors=dict(gyro_age_ticks=(w[2]>>10)&63,gyro_fresh=bool(w[2]&512),gyro_calibrated=bool(w[2]&256),
        calibrating=bool(w[2]&128),gyro_valid=bool(w[2]&64),lidar_ready=bool(w[2]&32),
        avoid_lidar_ready=bool(w[2]&16),body_guard_ready=bool(w[2]&8),scan_enable=bool(w[2]&4),running=bool(w[2]&2)),
      scan=dict(period_ticks=(w[3]>>16)&255,age_ticks=(w[3]>>10)&63,complete_age_ticks=(w[3]>>4)&63,
        work_bad=bool(w[3]&8),last_scan_bad=bool(w[3]&4),scan_id=w[4]>>16,integrated_id=w[4]&65535,
        raw_nodes=(w[5]>>20)&1023,usable_nodes=(w[5]>>10)&1023,invalid_nodes=w[5]&1023,
        parse_errors=(w[6]>>12)&255,overruns=(w[6]>>4)&255,
        hit_coverage=w[6]&15,direction_counts=[(w[7]>>(8*i))&255 for i in range(4)],
        sampled_visibility=(w[8]>>13)&15,required=(w[8]>>17)&15,unconfirmed=(w[8]>>8)&31),
      obstacle=dict(front=bool(w[8]&128),rear=bool(w[8]&64),slow=bool(w[8]&32),turn=bool(w[8]&16),
        contact=bool(w[8]&8),trigger_valid=bool(w[8]&4),trigger_kind=(w[9]>>16)&63,
        trigger_scan_id=w[9]&65535,raw_angle_q6=(w[10]>>16)&32767,distance_mm=w[10]&65535,
        quality=w[11]&63,body_x_mm=signed(w[12]>>16,16),body_y_mm=signed(w[12],16)),
      pose=dict(x_mm=signed(w[13]),y_mm=signed(w[14]),heading_mdeg=signed(w[15]),heading_q6=w[16]>>16),
      map=dict(local=bool(w[17]&128),ready=bool(w[17]&64),busy=bool(w[17]&32),seed_valid=bool(w[17]&16),
        seed_done=bool(w[17]&8),integrated_valid=bool(w[17]&4),frozen=bool(w[17]&2),world_capture_valid=bool(w[17]&1),
        rejected_fine_x=signed(w[18]),rejected_fine_y=signed(w[19]),reject_valid=bool(w[20]&512),
        reject_reason=(w[20]>>5)&15,rejected_cell=(w[20]>>3)&3,phase=w[20]&7),
      geometry=dict(radius_mm=w[21]&65535,front_mm=w[22]>>16,rear_mm=w[22]&65535,
        left_mm=w[23]>>16,right_mm=w[23]&65535,offset_x_mm=signed(w[24]>>16,16),offset_y_mm=signed(w[24],16),zero_q6=w[25]&32767),
      counters=dict(rx_bytes=w[26],nodes=w[27],points=w[28],uart_errors=w[29],packet_errors=w[30],
        map_dropped=w[31],map_oob=w[32],map_points=w[33],map_scans=w[44]),
      goal=[signed(w[34]),signed(w[35])],current_leg=[signed(w[36]),signed(w[37])],
      scan_seen=bool(w[46]&1),lookahead_record_valid=bool(w[46]&2),trigger_is_lookahead=bool(w[46]&4),snapshot_epoch=w[47]&65535,mixed_epoch=bool(w[47]&65536),words=w)

ACK=re.compile(r'^\[ACK (OK|ERR)\]\s*(\d+),(\d+),(\d+),(\d+),(\d+),(-?\d+)\s*$')

def format_summary(record):
    """Keep mission rejection visible even if the live-cause classifier is0."""
    scan,grid=record['scan'],record['map']
    error=record['mission_error']
    return (f"nav={record['nav_status']} fault={record['nav_fault']} "
      f"mission={error}({MISSION_ERRORS.get(error,'unclassified')}) planner={record['planner_error']} "
      f"cause={record['reason']} scan={scan['scan_id']} integrated={scan['integrated_id']} "
      f"age10ms={scan['complete_age_ticks']} bad={scan['work_bad']} overruns={scan['overruns']} "
      f"world_capture={grid['world_capture_valid']} local={grid['local']} frozen={grid['frozen']} "
      f"reject_valid={grid['reject_valid']} cell=({grid['rejected_fine_x']},{grid['rejected_fine_y']}) "
      f"raw={grid['rejected_cell']} reject_reason={grid['reject_reason']} phase={grid['phase']} "
      f"pose={record['pose']} goal={record['goal']} mixed={record['mixed_epoch']}")
def query(port,seq,field,value,timeout=3):
    port.write(f'#{seq},10,0,{field},{value}\n'.encode('ascii'))
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        line=port.readline().decode('ascii',errors='replace').strip()
        match=ACK.match(line)
        if not match or int(match[2])!=seq:continue
        if int(match[3])!=10 or match[1]!='OK' or int(match[6])!=0:
            raise RuntimeError('Diagnostic rejected (old FPGA/unsupported operation?): '+line)
        return int(match[7])&0xffffffff
    raise TimeoutError('No diagnostic ACK. Check matching bitstream/UART; close VOFA first.')

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',required=True)
    parser.add_argument('--output',type=Path,default=Path('dataset/lidar_diagnostics.jsonl'))
    parser.add_argument('--count',type=int,default=5)
    parser.add_argument('--interval',type=float,default=1)
    args=parser.parse_args()
    if args.count<1 or args.interval<1:parser.error('count>=1 and interval>=1 second required')
    import serial
    args.output.parent.mkdir(parents=True,exist_ok=True)
    seq=10000
    with serial.Serial(args.port,115200,timeout=.2) as port,args.output.open('a',encoding='utf-8') as output:
        for _ in range(args.count):
            t=time.monotonic();query(port,seq,1,0);seq+=1
            words=[]
            for index in range(48):
                words.append(query(port,seq,0,index));seq=(seq+1)&65535
            record=decode_words(words);record['wall_time']=time.strftime('%Y-%m-%dT%H:%M:%S%z')
            output.write(json.dumps(record,ensure_ascii=False)+'\n');output.flush()
            print(format_summary(record),flush=True)
            print('scan evidence',record['scan'],'trigger (may be historical)',record['obstacle'],flush=True)
            time.sleep(max(0,args.interval-(time.monotonic()-t)))

if __name__=='__main__':main()
