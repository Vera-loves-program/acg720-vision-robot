"""CRC algebra/frame acceptance regressions; no devices or HDL simulation."""
from pathlib import Path
import binascii
import random
import re
import struct
import zipfile

ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'src/udp_detection_feedback.v'
source=SOURCE.read_text(encoding='utf-8-sig')
function_pattern=r'function\s+\[15:0\]\s+crc_byte;.*?endfunction'
function=re.search(function_pattern,source,re.S).group()
masks={int(bit):int(mask,16) for bit,mask in re.findall(r"crc_byte\[(\d+)\]\s*=\^\(combined\s*&\s*24'h([0-9a-fA-F]+)\)",function)}
assert set(masks)==set(range(16))
assert max(mask.bit_count() for mask in masks.values())==7
assert 'combined={previous,data};' in re.sub(r'\s+','',function)
assert not re.search(r'\bfor\s*\(|\?',re.sub(r'//[^\n]*','',function))

def old_byte(previous,data):
    value=previous^(data<<8)
    for _ in range(8):value=((value<<1)^(0x1021 if value&0x8000 else 0))&0xffff
    return value

def new_byte(previous,data):
    combined=(previous<<8)|data
    return sum(((combined&mask).bit_count()&1)<<bit for bit,mask in masks.items())

def crc_frame(data,step):
    result=0xffff
    for byte in data:result=step(result,byte)
    return result

def accepts(frame,step,*,packet_error=False,payload_length=32,count=32,mailbox_free=True):
    if packet_error or payload_length!=32 or count!=32 or len(frame)!=32 or not mailbox_free:return False
    if frame[4]!=1 or crc_frame(frame[:30],step)!=int.from_bytes(frame[30:],'big'):return False
    if frame[:4] in (b'FOL1',b'TGT1',b'VEC1',b'GOL1'):return True
    if frame[:4]!=b'DET1' or frame[5] not in (0,2,3):return False
    score,sequence,width,height,x0,y0,x1,y1,age=struct.unpack('>9H',frame[6:24])
    box_valid=(x0<x1<=400 and y0<y1<=240) if frame[5]&1 else score==0 and not any(frame[14:22])
    return box_valid and score<=1000 and width==400 and height==240 and age<=1500

def with_crc(frame):
    return bytes(frame[:30])+binascii.crc_hqx(frame[:30],0xffff).to_bytes(2,'big')

def main():
    assert new_byte(0,0)==old_byte(0,0)==0
    for bit in range(24):
        combined=1<<bit
        assert new_byte(combined>>8,combined&255)==old_byte(combined>>8,combined&255)
    pairs=0
    for previous in range(65536):
        for data in (0,1,0x55,0x80,0xaa,0xff):
            assert new_byte(previous,data)==old_byte(previous,data)
            pairs+=1
    rng=random.Random(0x1021)
    for _ in range(100000):
        previous=rng.randrange(65536);data=rng.randrange(256)
        assert new_byte(previous,data)==old_byte(previous,data)==binascii.crc_hqx(bytes([data]),previous)
    assert crc_frame(b'123456789',new_byte)==0x29b1
    packets=[]
    for magic in (b'FOL1',b'TGT1',b'VEC1',b'GOL1'):
        frame=bytearray(rng.randbytes(32));frame[:4]=magic;frame[4]=1;packets.append(with_crc(frame))
    frame=bytearray(32);frame[:4]=b'DET1';frame[4:6]=bytes((1,3))
    struct.pack_into('>9H',frame,6,900,42,400,240,50,60,100,120,150)
    packets.append(with_crc(frame))
    corrupted=0
    for frame in packets:
        assert accepts(frame,new_byte) and accepts(frame,old_byte)
        for bit in range(256):
            damaged=bytearray(frame);damaged[bit//8]^=1<<(bit%8)
            assert not accepts(damaged,new_byte) and not accepts(damaged,old_byte)
            corrupted+=1
        for metadata in (dict(packet_error=True),dict(payload_length=31),dict(payload_length=33),
                         dict(count=31),dict(count=33),dict(mailbox_free=False)):
            assert not accepts(frame,new_byte,**metadata) and not accepts(frame,old_byte,**metadata)
        # Recomputed valid CRC must not bypass magic/version/DET box checks.
        damaged=bytearray(frame);damaged[4]=2;damaged=with_crc(damaged)
        assert not accepts(damaged,new_byte) and not accepts(damaged,old_byte)
    for changed_offset,new_value in ((5,1),(10,2),(19,0),(22,255)):
        invalid=bytearray(packets[-1]);invalid[changed_offset]=new_value;invalid=with_crc(invalid)
        assert not accepts(invalid,new_byte) and not accepts(invalid,old_byte)
    # Prove this focused edit leaves every non-CRC byte/framing/mailbox line
    # identical to the actualR3 source snapshot when that archive is present.
    archive=ROOT/'tmp/core-route-20261025/build-184139-sources.zip'
    if archive.exists() and 'reg packet_pending' not in source:
        with zipfile.ZipFile(archive) as z:
            name=next(name for name in z.namelist() if name.replace('\\','/').endswith('src/udp_detection_feedback.v'))
            before=z.read(name).decode('utf-8-sig')
        assert re.sub(function_pattern,'CRC_FUNCTION',before,flags=re.S)==re.sub(function_pattern,'CRC_FUNCTION',source,flags=re.S)
    print(f'PASS: 24GF2basis vectors; {pairs} exhaustive16-bit CRC states/6bytes;100000random triples includingstdlib;CCITT29B1check.')
    print(f'PASS:5protocol frames;{corrupted} single-bit corruptions;length/count/error/busy/version/DETgeometry rejection.')
    print('Independent algebra/protocol checks, not HDL simulation, timing closure or hardware validation.')

if __name__=='__main__':main()
