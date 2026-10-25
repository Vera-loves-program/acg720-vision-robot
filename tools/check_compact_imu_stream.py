"""Exhaustive compact IMU numeric/ASCII references and byte-stream model.

This checks arithmetic, source shape and an independent backpressure model;
it does not clock HDL, prove CDC hardware, predict fit or operate the board.
"""
from pathlib import Path
import random
import re
from check_sensor_text_math import bcd_six,restoring_divide,gyro_row,accel_row

ROOT=Path(__file__).resolve().parents[1]
SOURCE=(ROOT/'src/sensor_demo_text.v').read_text(encoding='utf-8').split('module sensor_demo_text_compact',1)[1]
LCD=(ROOT/'src/lcd1024_interaction_ui.v').read_text(encoding='utf-8')


def scale_magnitude(magnitude,accel):
    if not accel:return ((magnitude<<3)-magnitude)&((1<<18)-1)
    first=((magnitude<<7)-(magnitude<<2))&((1<<22)-1)
    dividend=(first-(magnitude<<1)+500)&((1<<22)-1)
    quotient,remainder=restoring_divide(dividend,22,12)
    assert (quotient,remainder)==divmod(magnitude*122+500,1000)
    return quotient


def streamed_row(field,negative,bcd,valid=True):
    # Byte cases, independently compared below to the established full-row
    # formatter and ordinary decimal formatting for every signed16 input.
    row=bytearray(b' '*16)
    row[0]=ord('G' if field<3 else 'A');row[1]=ord('XYZ'[field%3])
    if not valid:
        tail=b' ----.--DPS   ' if field<3 else b' -.---G       '
        row[2:]=tail
    elif field<3:
        row[3]=ord('-' if negative and bcd else '+')
        for column,shift in ((4,20),(5,16),(6,12),(7,8),(9,4),(10,0)):
            row[column]=48+((bcd>>shift)&15)
        row[8]=ord('.');row[11:14]=b'DPS'
    else:
        row[3]=ord('-' if negative and bcd else '+')
        for column,shift in ((4,12),(6,8),(7,4),(8,0)):
            row[column]=48+((bcd>>shift)&15)
        row[5]=ord('.');row[9]=ord('G')
    return bytes(row)


def frame_bytes(raw_axes,valid=True):
    frame=bytearray(b' '*256)
    frame[:16]=b'IMU READY       ' if valid else b'IMU INVALID     '
    for field,raw in enumerate(raw_axes):
        magnitude=abs(raw)
        scaled=scale_magnitude(magnitude,field>=3)
        frame[(field+1)*16:(field+2)*16]=streamed_row(field,raw<0,bcd_six(scaled,18),valid)
    return bytes(frame)


def main():
    gyro=[];accel=[]
    for magnitude in range(32769):
        g=scale_magnitude(magnitude,False);a=scale_magnitude(magnitude,True)
        assert g==magnitude*7
        assert a==(magnitude*122+500)//1000
        gb,ab=bcd_six(g,18),bcd_six(a,18)
        assert gb==int(f'{g:06d}',16) and ab==int(f'{a:06d}',16)
        gyro.append(gb);accel.append(ab)
    for raw in range(-32768,32768):
        magnitude=abs(raw)
        for axis in range(3):
            g=streamed_row(axis,raw<0,gyro[magnitude])
            a=streamed_row(axis+3,raw<0,accel[magnitude])
            assert g==gyro_row(axis,raw<0,gyro[magnitude])
            assert a==accel_row(axis,raw<0,accel[magnitude])
            assert len(g)==len(a)==16
    for field in range(6):
        expected=(f'G{"XYZ"[field]} ----.--DPS   ' if field<3 else f'A{"XYZ"[field-3]} -.---G       ').encode()
        assert streamed_row(field,True,0,False)==expected
    edge=frame_bytes((-32768,32767,0,-32768,32767,-1))
    assert edge[16:32]==b'GX -2293.76DPS  '
    assert edge[64:80]==b'AX -3.998G      '
    assert edge[96:112]==b'AZ +0.000G      '
    assert edge[112:]==b' '*144

    rng=random.Random(20261025)
    for sample in range(100):
        raw=tuple(rng.randrange(-32768,32768) for _ in range(6))
        expected=frame_bytes(raw,sample%3!=0)
        # A frame freezes axes once; unrelated live samples during stalls
        # cannot change a currently presented byte or enter an old snapshot.
        offered=frame_bytes(raw,sample%3!=0)
        index=0;accepted=[];previous=None;commit=False
        while index<256:
            ready=rng.randrange(4)==0
            byte=(index,offered[index])
            if previous is not None:assert byte==previous
            if ready:accepted.append(byte);index+=1;previous=None
            else:previous=byte
            _new_live_axes=tuple(rng.randrange(-32768,32768) for _ in range(6))
            assert not commit
        commit=True
        assert commit and [i for i,_ in accepted]==list(range(256))
        assert bytes(b for _,b in accepted)==expected

    compact=re.sub(r'\s+','',re.sub(r'//[^\n]*','',SOURCE))
    for token in ('reg[23:0]decimal;','reg[21:0]dividend;','reg[11:0]quotient;',
        'assigntext_byte_valid=state==EMIT;','EMIT:if(text_byte_ready)begin',
        'if(text_byte_index==255)state<=COMMIT;',
        'COMMIT:begintext_valid<=1;state<=IDLE;end'):
        assert token in compact,token
    assert 'text_rows' not in compact and 'processed_axes' not in compact and 'metadata' not in compact
    assert not re.search(r'\*\s*\d+\s*\x27d(?:7|122)\b',SOURCE)
    assert 'if (!BYTE_TEXT_MODE && frame_restart && sensor_request_sync[2]!=sensor_text_ack)' in LCD
    assert 'robot_byte_source_a==2\'d3 ? sensor_byte_character' in LCD
    assert 'robot_control_l && robot_label_source_l==3\'d3' in LCD
    print('PASS: all65536 signed raw values on all6 axes; exact legacy16-byte rows,2000dps/4g units and negative-zero suppression')
    print('PASS:32769 shift/add magnitudes,22-step /1000 and18-step six-digitBCD;100 ordered256-byte frames with random backpressure')
    print('PASS: core has no wide ASCII/metadata/processed-axis banks; LCDsource3 uses existing synchronousbyte stage')
    print('Independent numeric/source/stream model only; no HDL execution, CDC proof, device or timing/resource-fit claim.')


if __name__=='__main__':main()
