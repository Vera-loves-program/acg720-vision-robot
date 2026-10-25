"""Independent timer/packet clock-edge regressions, not HDL simulation."""
from dataclasses import dataclass
from pathlib import Path
import random
import re
import struct
from check_udp_crc_matrix import old_byte,new_byte,with_crc,accepts

ROOT=Path(__file__).resolve().parents[1]
TERMINAL=250_000_000

@dataclass
class Timer:
    value:int=TERMINAL
    expired:bool=True
    c7:int=0
    c14:int=0
    c21:int=0

    @classmethod
    def at(cls,value):
        return cls(value,value==TERMINAL,int(value&127==127),
                   int(value&16383==16383),int(value&2097151==2097151))

    def edge(self,commit=False):
        value=self.value
        if commit:
            self.value=1;self.expired=False;self.c7=self.c14=self.c21=0
        elif not self.expired:
            self.value=(((value&127)+1)&127)|((((value>>7&127)+self.c7)&127)<<7)|\
                ((((value>>14&127)+self.c14)&127)<<14)|((((value>>21&127)+self.c21)&127)<<21)
            self.c7=int(value&127==126);self.c14=int(value&16383==16382)
            self.c21=int(value&2097151==2097150)
            self.expired=value==TERMINAL-1


def fresh(sequence,last,have,expired):
    delta=(sequence-last)&65535
    return not have or expired or 0<delta<32768

class Receiver:
    """One-stage parallel validation and live held-mailbox/ACK model."""
    def __init__(self):
        self.timer=Timer();self.have=False;self.last=0
        self.work=bytes(32);self.count=0;self.crc=0xffff;self.pending=None
        self.req=[0,0];self.sync=[[0,0,0],[0,0,0]];self.data=[None,None]
        self.events=[];self.cycle=0

    def edge(self,byte=None,done=False,length=32,error=False,acks=(0,0),reset=False):
        if reset:self.__init__();return
        self.cycle+=1
        old_free=[self.req[k]==self.sync[k][2] for k in range(2)]
        commit_display=False
        if self.pending is not None:
            raw,valid,free_at_done,is_display=self.pending
            k=0 if is_display else 1
            if valid and free_at_done and old_free[k]:
                self.data[k]=raw;self.req[k]^=1
                self.events.append((self.cycle,k,raw))
                if is_display:
                    self.have=True;self.last=int.from_bytes(raw[8:10],'big');commit_display=True
        # Snapshot sequence/timeout at packet_done, before its delayed commit.
        raw=self.work;count=self.count;crc=self.crc
        next_pending=None
        if done:
            is_display=raw[:4]==b'DET1'
            common=(not error and length==32 and count==32 and raw[4]==1 and crc==int.from_bytes(raw[30:],'big'))
            valid=common and accepts(raw,old_byte)
            if is_display:valid=valid and fresh(int.from_bytes(raw[8:10],'big'),self.last,self.have,self.timer.expired)
            next_pending=(raw,valid,old_free[0 if is_display else 1],is_display)
        self.timer.edge(commit_display)
        if byte is not None and self.count<32:
            if self.count<30:self.crc=new_byte(self.crc,byte)
            self.work=self.work[1:]+bytes([byte]);self.count+=1
        if done:self.work=bytes(32);self.count=0;self.crc=0xffff
        self.pending=next_pending
        self.sync=[[acks[k],self.sync[k][0],self.sync[k][1]] for k in range(2)]

    def frame(self,raw,**kwargs):
        for byte in raw:self.edge(byte=byte,acks=tuple(self.req))
        self.edge(done=True,acks=tuple(self.req),**kwargs)


def det(sequence):
    data=bytearray(32);data[:4]=b'DET1';data[4:6]=bytes((1,3))
    struct.pack_into('>9H',data,6,900,sequence,400,240,40,50,100,120,100)
    return with_crc(data)


def main():
    cases=0
    # Exhaust every lower14-bit pattern with varied upper bits; compare
    # chunks, registered carry prediction and saturation to flat arithmetic.
    for upper in (0,127,1000):
        for lower in range(16384):
            value=(upper<<14)|lower
            timer=Timer.at(value)
            for _ in range(4):
                value=min(TERMINAL,value+1);timer.edge()
                assert timer.value==value and timer.expired==(value==TERMINAL)
                cases+=1
    rng=random.Random(250000000)
    for _ in range(50000):
        value=rng.randrange(TERMINAL);timer=Timer.at(value)
        for _ in range(4):
            commit=rng.randrange(20)==0
            value=1 if commit else min(TERMINAL,value+1);timer.edge(commit)
            assert timer.value==value and timer.expired==(value==TERMINAL)
            cases+=1
    for boundary in (128,16384,2097152,TERMINAL):
        for start in range(boundary-8,boundary+1):
            timer=Timer.at(min(start,TERMINAL));value=timer.value
            for _ in range(20):
                value=min(TERMINAL,value+1);timer.edge()
                assert timer.value==value and timer.expired==(value==TERMINAL)
            timer.edge(True);assert timer.value==1 and not timer.expired
    # A source-clock accept reset used0 at N and increments1 at N+1.
    # Delayed commit loads1 at N+1, preserving the expiration edge exactly.
    for start in (0,126,127,16382,16383,2097150,2097151,TERMINAL-1,TERMINAL):
        timer=Timer.at(start);timer.edge();timer.edge(True)
        assert timer.value==1 and not timer.expired
        for _ in range(300):timer.edge()
        assert timer.value==301

    rx=Receiver()
    for magic in (b'FOL1',b'TGT1',b'VEC1',b'GOL1'):
        raw=bytearray(rng.randbytes(32));raw[:4]=magic;raw[4]=1;raw=with_crc(raw)
        before=len(rx.events);rx.frame(raw)
        assert len(rx.events)==before  # publication waits one receive clock
        done=rx.cycle;rx.edge(acks=tuple(rx.req))
        assert rx.events[-1]==(done+1,1,raw)
    # New frame capture must not replace data held for an unacknowledged
    # consumer. ACK racing just after packet_done cannot rescue a busy frame.
    ack_cases=0
    for offset in range(-6,4):
        rx=Receiver();first=with_crc(b'FOL1\1'+bytes(25));rx.frame(first);rx.edge()
        assert rx.req[1]==1
        raw=with_crc(b'TGT1\1'+bytes(25))
        for byte in raw:rx.edge(byte=byte)
        done_cycle=rx.cycle+8
        while rx.cycle<done_cycle:
            upcoming=rx.cycle+1;ack=1 if upcoming>=done_cycle+offset else 0
            free_before=rx.req[1]==rx.sync[1][2]
            rx.edge(done=upcoming==done_cycle,acks=(0,ack))
        allowed=free_before;before=len(rx.events);rx.edge(acks=(0,1))
        assert (len(rx.events)>before)==allowed
        assert rx.data[1]==(raw if allowed else first)
        ack_cases+=1
    for error,length in ((True,32),(False,31),(False,33)):
        rx=Receiver();rx.frame(det(1),error=error,length=length);rx.edge()
        assert not rx.events
    for bad_count in (0,1,30,31):
        rx=Receiver()
        for byte in det(1)[:bad_count]:rx.edge(byte=byte)
        rx.edge(done=True);rx.edge();assert not rx.events
    # All single-bit corruptions stay rejected after parallel staging.
    corrupt=0
    for raw in (det(1),with_crc(b'FOL1\1'+bytes(25))):
        for bit in range(256):
            bad=bytearray(raw);bad[bit//8]^=1<<(bit%8)
            rx=Receiver();rx.frame(bad);rx.edge();assert not rx.events;corrupt+=1
    # Sequence wrap and timeout decisions are frozen at done, not recomputed
    # on the later commit (especially expiry crossing on that same edge).
    for expired in (False,True):
        for last,sequence in ((0,0),(0,1),(65535,0),(0,32767),(0,32768),(2,1)):
            rx=Receiver();rx.have=True;rx.last=last
            rx.frame(det(sequence))
            # Rebuild precisely at the done edge using a prepared full packet.
            rx=Receiver();rx.have=True;rx.last=last;rx.work=det(sequence);rx.count=32
            rx.crc=int.from_bytes(rx.work[30:],'big');rx.timer=Timer.at(TERMINAL if expired else TERMINAL-1)
            rx.edge(done=True);rx.edge()
            assert bool(rx.events)==fresh(sequence,last,True,expired)
            if rx.events:assert rx.timer.value==1 and not rx.timer.expired
    rx=Receiver();rx.frame(det(42));rx.edge(reset=True);rx.edge()
    assert not rx.events and rx.timer.expired and rx.req==[0,0]
    # Wire-legal bursts with consumer ACKs preserve every full payload.
    rx=Receiver();gold=[]
    for sequence in range(150):
        raw=det(sequence);rx.frame(raw);rx.edge(acks=tuple(rx.req));gold.append(raw)
    assert [raw for _,kind,raw in rx.events if kind==0]==gold

    source=(ROOT/'src/udp_detection_feedback.v').read_text(encoding='utf-8')
    code=re.sub(r'\s+','',re.sub(r'//[^\n]*|/\*.*?\*/','',source,flags=re.S))
    for token in ('idle_cycles[6:0]<=idle_cycles[6:0]+7\'d1;',
                  "idle_carry7<=idle_cycles[6:0]==7'd126;", "idle_carry14<=idle_cycles[13:0]==14'd16382;",
                  "idle_carry21<=idle_cycles[20:0]==21'd2097150;", "if(idle_cycles==28'd249999999)idle_expired<=1;",
                  "idle_cycles<=28'd1;idle_expired<=0;",'wirestage_capture=packet_done&&!packet_pending;',
                  'check_sequence<=new_sequence;','robot_free_at_done&&robot_request==robot_ack_sync[2]',
                  'display_free_at_done&&request_q==ack_sync[2]',
                  'if(commit_robot)beginrobot_data<=work;robot_request<=~robot_request;end'):
        assert token in code,token
    assert "idle_cycles<=idle_cycles+1'b1" not in code
    print(f'PASS:{cases} timer state transitions; every14bit lower pattern;7/14/21bit rolls; exact250000000terminal/reset-compensation.')
    print(f'PASS:4metadata types+DET;one-clock publish;{ack_cases} ACK races;bad length/count/error;{corrupt} corruptedframes;sequence/expiry snapshot;reset pending;150 legal bursts.')
    print('Independent arithmetic/transaction model and source contracts, not HDL simulation/timing/hardware validation.')

if __name__=='__main__':main()
