#!/usr/bin/env python3
"""Render panel label/button declarations with the RTL font for visual QA.

This reads the layout declarations, not an HDL simulation. Dynamic sample
values are illustrative and are NEVER presented as board measurements.
"""
from pathlib import Path
import argparse
import re
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]

def split_args(text):
    parts=[]; start=0; depth=0; quoted=False
    for index,char in enumerate(text):
        if char=='"': quoted=not quoted
        if not quoted:
            if char in '([{': depth+=1
            elif char in ')]}': depth-=1
            elif char==',' and depth==0:
                parts.append(text[start:index].strip());start=index+1
    return parts+[text[start:].strip()]

def render(output, running=False, bq_info=False):
    source=(ROOT/'src/actuator_test_panel.v').read_text(encoding='utf-8')
    lcd=(ROOT/'src/lcd1024_interaction_ui.v').read_text(encoding='utf-8')
    glyphs={char:int(bits,16) for char,bits in re.findall(r'"(.)":glyph=40\x27h([0-9a-f]+)',lcd)}
    colors={name:int(value,16) for name,value in re.findall(r'(\w+)=16\x27h([0-9a-f]+)',source)}
    def rgb565(value):
        return ((value>>11)*255//31,((value>>5)&63)*255//63,(value&31)*255//31)
    image=Image.new('RGB',(1024,600),rgb565(colors['BG'])); draw=ImageDraw.Draw(image)
    def box(l,r,t,b,color):
        draw.rectangle((l,t,r-1,b-1),fill=rgb565(color))
    box(0,1024,0,44,colors['WHITE']);box(0,1024,44,45,colors['BORDER'])
    for shift in (0,240):
        box(16,320,56+shift,280+shift,colors['WHITE'])
        box(336,704,56+shift,280+shift,colors['WHITE'])
    box(720,1008,56,464,colors['WHITE'])
    flags=[1,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0]
    rows=['SET +00% PWM    ','L SPD WAIT      ','SET +00% PWM    ','R SPD WAIT      ']
    for _ in range(2): rows+=['US 1500 / 1500  ','CMD 0180 DEG EST','L1500 H1500 US  ','FINE    10 US   ']
    rows+=['BT NO RX YET    ','LOCAL TEST READY','RX BYTE 0000    ','TX SENT 0000    ']
    sensor=['I:WAIT  U:WAIT  ','GX ----.--DPS   ','GY ----.--DPS   ','GZ ----.--DPS   ',
            'AX -.---G       ','AY -.---G       ','AZ -.---G       ','US INVALID      ']
    if running:
        flags=[0,0,1,0,1,0,1,1,0,0,1,1,1,1,1,1]
        rows[0]='SET +15% PWM    '  # Encoder values stay WAIT, not fabricated measurements.
        rows[4]='US 1556 / 1556  ';rows[5]='CMD 0190 DEG EST';rows[6]='L0800 H2200 US  ';rows[7]='COARSE  10 DEG  '
        rows[10]='L1100 H1900 US  ';rows[11]='FINE     1 DEG  ';rows[12]='BT RX ACTIVE    '
        sensor[0]='I:OK    U:WAIT  '
        # Intentionally retain invalid axes: no fabricated sensor readings.
    diagnostics=[]
    for tag in ('A','B'):
        diagnostics += [f'{tag} V=---- P=---- ',f'{tag} CS=---- F=00  ',
                        f'{tag} DF=---- N=--  ',f'{tag} VD=-----      ',f'{tag} SV----- FW----']
    env=dict(colors,flags=flags,rows=rows,sensor_rows=sensor,battery_valid=[0,0],
             bq_info=bq_info,diagnostic_rows=diagnostics)

    def conditional(expr):
        depth=0;quoted=False;question=None;nested=0
        for index,char in enumerate(expr):
            if char=='"':quoted=not quoted
            if quoted:continue
            if char in '([{':depth+=1
            elif char in ')]}':depth-=1
            elif depth==0:
                if char=='?':
                    if question is None:question=index
                    else:nested+=1
                elif char==':' and question is not None:
                    if nested:nested-=1
                    else:return expr[:question],expr[question+1:index],expr[index+1:]
        return None

    def evaluate(expr):
        expr=expr.strip()
        choice=conditional(expr)
        if choice:
            condition,yes,no=choice
            return evaluate(yes if evaluate(condition) else no)
        if expr.startswith('"'): return expr[1:-1]
        if '==' in expr:
            left,right=expr.split('==',1)
            return evaluate(left)==evaluate(right)
        if expr.startswith('{"IMU "'): return 'IMU '+sensor[0][:8]+'    '
        battery={639:'A BQ WAIT       ',511:'B BQ WAIT       ',383:'A N~----- I-----',
                 255:'B N~----- I-----',127:'A:--- B:--- NTC~'}
        match=re.fullmatch(r'battery_rows\[(\d+):\d+\]',expr)
        if match:return battery[int(match[1])]
        match=re.fullmatch(r'(rows|sensor_rows|diagnostic_rows)\[(.+)\s+-:\s*(\d+)\]',expr)
        if match:
            start=evaluate(match[2]);width=int(match[3]);total=dict(rows=2048,sensor_rows=1024,diagnostic_rows=1280)[match[1]]
            row_index=(total-1-start)//128
            return env[match[1]][row_index][:width//8]
        return eval(expr,{'__builtins__':{}},env)

    labels=[]
    def calls(segment):
        for match in re.finditer(r'\b(label|button)\((.*?)\);',segment,re.S):
            args=[evaluate(part) for part in split_args(match[2])]
            if match[1]=='button':
                left,right,top,bottom,text,fill,ink=args
                box(left,right,top,bottom,fill)
                labels.append((left+8,right-4,top+(bottom-top-16)//2,text,ink))
            else: labels.append(tuple(args))

    body=source.split('always @* begin',1)[1]
    before,loop=body.split('for (card=0;card<2;card=card+1) begin',1)
    loop,after=loop.split('if (rect(720,1008,56,464))',1)
    calls(before)
    for card in range(2):
        env.update(card=card,top=card*240,slot=4 if card==0 else 8)
        # Assignments select the wheel flag first and servo flag afterwards.
        wheel,servo_body=loop.split('// Middle:',1)
        env['enabled']=flags[2+card];calls(wheel)
        env.update(enabled=flags[4+card],cal=flags[card],min_ok=flags[12+2*card],
                   max_ok=flags[13+2*card],limits_ok=flags[6+card])
        calls(servo_body)
    calls(after)
    for left,right,top,text,color in labels:
        assert len(text)<=16,repr(text)
        for index,char in enumerate(text):
            bits=glyphs.get(char,0)
            if char!=' ':
                assert left+index*12+10<=right,(text,left,right)
            for col in range(5):
                for row in range(7):
                    if (bits>>(32-col*8+row))&1:
                        px=left+index*12+col*2;py=top+row*2
                        if px+1<right: draw.rectangle((px,py,px+1,py+1),fill=rgb565(color))
    output.parent.mkdir(parents=True,exist_ok=True);image.save(output)
    print(f'Layout preview (illustrative, not FPGA output): {output}')

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output',type=Path);parser.add_argument('--running',action='store_true')
    parser.add_argument('--bq-info',action='store_true')
    args=parser.parse_args();render(args.output,args.running,args.bq_info)
