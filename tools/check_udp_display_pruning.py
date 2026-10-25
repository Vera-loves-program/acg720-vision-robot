"""Shared work-bank/optional display regressions; no HDL simulation/devices."""
from pathlib import Path
import random
import re
from check_idle_timer import Receiver,Timer,fresh,det
from check_udp_crc_matrix import accepts,old_byte,new_byte,with_crc

ROOT=Path(__file__).resolve().parents[1]

class LeanReceiver(Receiver):
    """Independent single-work-bank model; no second full packet copy."""
    def __init__(self,display=True):
        super().__init__();self.display=display

    def edge(self,byte=None,done=False,length=32,error=False,acks=(0,0),reset=False):
        if reset:self.__init__(self.display);return
        self.cycle+=1
        oldwork,oldcount,oldcrc=self.work,self.count,self.crc
        oldpending=self.pending
        free=[self.req[k]==self.sync[k][2] for k in range(2)]
        oldhave,oldlast,oldexpired=self.have,self.last,self.timer.expired
        commit_display=False
        if oldpending is not None:
            valid,free_done,is_display=oldpending;k=0 if is_display else 1
            if valid and free_done and free[k]:
                self.data[k]=oldwork;self.req[k]^=1
                self.events.append((self.cycle,k,oldwork))
                if is_display:self.have=True;self.last=int.from_bytes(oldwork[8:10],'big');commit_display=True
        self.pending=None
        if done and oldpending is None:
            is_display=oldwork[:4]==b'DET1'
            common=(not error and length==32 and oldcount==32 and oldwork[4]==1 and oldcrc==int.from_bytes(oldwork[30:],'big'))
            valid=common and accepts(oldwork,old_byte) and (self.display or not is_display)
            if is_display:valid=valid and fresh(int.from_bytes(oldwork[8:10],'big'),oldlast,oldhave,oldexpired)
            self.pending=(valid,free[0 if is_display else 1],is_display)
        if self.display:self.timer.edge(commit_display)
        if oldpending is not None:
            self.work=bytes(31)+(bytes([byte]) if byte is not None else b'\0')
            self.count=int(byte is not None)
            self.crc=0xffff if byte is None else new_byte(0xffff,byte)
        elif byte is not None and self.count<32:
            if self.count<30:self.crc=new_byte(self.crc,byte)
            self.work=self.work[1:]+bytes([byte]);self.count+=1
        self.sync=[[acks[k],self.sync[k][0],self.sync[k][1]] for k in range(2)]
        if not self.display:
            assert self.req[0]==0 and self.data[0] is None

def frame(magic,seed):
    rng=random.Random(seed);raw=bytearray(rng.randbytes(32));raw[:4]=magic;raw[4]=1
    return with_crc(raw)

def main():
    # Existing two-bank reference and new shared-bank variant produce the
    # same events/data with default legacy display and one-clock validation.
    old,new=Receiver(),LeanReceiver()
    raw_frames=[frame(magic,seed) for seed in range(120) for magic in (b'FOL1',b'TGT1',b'VEC1',b'GOL1')]
    raw_frames+= [det(sequence) for sequence in range(60)]
    for raw in raw_frames:
        for byte in raw:
            acks=tuple(old.req);old.edge(byte=byte,acks=acks);new.edge(byte=byte,acks=acks)
            assert old.events==new.events
        acks=tuple(old.req);old.edge(done=True,acks=acks);new.edge(done=True,acks=acks)
        assert old.events==new.events
        # No idle cycle: the NEXT frame's first byte arrives on commit.
    old.edge(acks=tuple(old.req));new.edge(acks=tuple(new.req))
    assert old.events==new.events and old.data==new.data
    assert [data for _,_,data in new.events]==raw_frames

    # Robot-only profile rejects DET while all four robot protocols and
    # exact CRC/length/count/error/free-at-done behavior remain active.
    core=LeanReceiver(False);expected=[]
    for raw in raw_frames:
        core.frame(raw);core.edge(acks=tuple(core.req))
        if raw[:4]!=b'DET1':expected.append(raw)
    assert [data for _,kind,data in core.events]==expected
    assert not any(kind==0 for _,kind,_ in core.events)
    assert core.req[0]==0 and core.data[0] is None

    # Exhaust arbitrary byte1 values exactly on the validation/clear edge.
    for first in range(256):
        for display in (False,True):
            rx=LeanReceiver(display);raw=frame(b'FOL1',first);rx.frame(raw)
            rx.edge(byte=first,acks=tuple(rx.req))
            assert rx.events[-1][2]==raw
            assert rx.count==1 and rx.work==bytes(31)+bytes([first])
            assert rx.crc==old_byte(0xffff,first)
    # Repeated done while the full bank is held cannot requeue that packet.
    for display in (False,True):
        rx=LeanReceiver(display);raw=frame(b'TGT1',1);rx.frame(raw)
        rx.edge(done=True);rx.edge(done=True);rx.edge()
        assert [data for _,_,data in rx.events]==[raw]
        # Same-edge last byte+done remains invalid, as in the original codec.
        rx=LeanReceiver(display)
        for byte in raw[:-1]:rx.edge(byte=byte)
        rx.edge(byte=raw[-1],done=True);rx.edge();assert not rx.events

    corrupt=0
    for display in (False,True):
        for raw in (frame(b'FOL1',7),det(1)):
            for bit in range(256):
                bad=bytearray(raw);bad[bit//8]^=1<<(bit%8)
                rx=LeanReceiver(display);rx.frame(bad);rx.edge();assert not rx.events;corrupt+=1
        for kwargs in (dict(length=31),dict(length=33),dict(error=True)):
            rx=LeanReceiver(display);rx.frame(frame(b'GOL1',7),**kwargs);rx.edge();assert not rx.events
        # ACK crossing the packet_done/commit boundary does not turn a
        # busy-at-done packet into a newly authorized write.
        for ack_offset in range(-6,4):
            rx=LeanReceiver(display);first=frame(b'FOL1',8);rx.frame(first);rx.edge()
            second=frame(b'VEC1',9)
            for byte in second:rx.edge(byte=byte)
            when=rx.cycle+8
            while rx.cycle<when:
                upcoming=rx.cycle+1;ack=int(upcoming>=when+ack_offset)
                allowed=rx.req[1]==rx.sync[1][2]
                rx.edge(done=upcoming==when,acks=(0,ack))
            before=len(rx.events);rx.edge(acks=(0,1))
            assert (len(rx.events)>before)==allowed
            assert rx.data[1]==(second if allowed else first)

    source=(ROOT/'src/udp_detection_feedback.v').read_text(encoding='utf-8')
    code=re.sub(r'\s+','',re.sub(r'//[^\n]*|/\*.*?\*/','',source,flags=re.S))
    assert 'parameterintegerENABLE_DISPLAY=1' in code
    assert 'reg[255:0]packet_hold' not in code
    assert len(re.findall(r'\breg\s*\[255:0\]',source))==3  # work + robot output + legacy output
    legacy=code.split('generateif(ENABLE_DISPLAY!=0)begin:legacy_display',1)[1].split('endelsebegin:core_no_detection_display',1)[0]
    core=code.split('endelsebegin:core_no_detection_display',1)[1]
    for token in ('idle_cycles','last_sequence','check_x_order','check_det_magic','reg[2:0]ack_sync','reg[255:0]data_q'):
        assert token in legacy
        assert token not in code.split('generateif(ENABLE_DISPLAY!=0)',1)[0],token
    assert "assigndisplay_request=1'b0;assigndisplay_data=256'd0;" in core
    assert "work<={248'd0,payload_byte};count<=1;crc<=crc_byte(16'hffff,payload_byte);" in code
    assert 'wirestage_capture=packet_done&&!packet_pending;' in code
    assert 'if(commit_robot)beginrobot_data<=work;robot_request<=~robot_request;end' in code
    print(f'PASS:540legacy frames vsoldheld-bank reference;480robot frames withDETcompiledout;512commit-edge byte1cases;1024corruptions;20ACKraces;repeatdone/finalbyte/error/length cases.')
    print('Structural: removes256-bit duplicate held bank; allDETtimer/sequence/predicates/data/ACK state confined toinactivegenerate in core.')
    print('Independent transaction/source checks; not HDL simulation, synthesized resource count, timing or hardware proof.')

if __name__=='__main__':main()
