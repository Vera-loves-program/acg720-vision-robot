"""Generate/check the byte-ROM region from the formatter's annotated IDs.

Labels stay next to their eight-bit localparam IDs in robot_status_text.v.
Default mode checks the generated region without changing any file. Pass
--write only when intentionally editing those label definitions. No HDL tool
is invoked, and no external .mem file is needed by the Gowin project.
"""
from pathlib import Path
import argparse
import json
import re

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT/'src/robot_status_text.v'
BEGIN = '// BEGIN GENERATED STATUS LABEL ROM\n'
END = '// END GENERATED STATUS LABEL ROM'


def labels_from_source(source):
    labels = []
    for name, number, literal in re.findall(
            r'localparam \[7:0\] (TEXT\d+)=8\'d(\d+); // ("[^"\n]*")', source):
        number = int(number)
        value = json.loads(literal)
        assert name == f'TEXT{number:03d}' and number == len(labels), (name, number)
        assert len(value) == 16 and value.isascii(), (name, value)
        assert value not in labels, (name, 'duplicate label ID')
        labels.append(value)
    assert labels and labels[0] == ' '*16 and len(labels) <= 256
    return labels


def generate(labels):
    lines = ['module robot_status_label_rom(',
             '    input wire clk,read_enable,',
             '    input wire [11:0] address,',
             '    output reg [7:0] data', ');',
             '    (* syn_romstyle="block_rom" *) reg [7:0] memory[0:4095];',
             '    integer init_index;', '    initial begin',
             '        for(init_index=0;init_index<1024;init_index=init_index+1)begin',
             "            memory[init_index]=8'h20;memory[init_index+1024]=8'h20;",
             "            memory[init_index+2048]=8'h20;memory[init_index+3072]=8'h20;",
             '        end']
    for number, label in enumerate(labels):
        lines.append(f'        // TEXT{number:03d}: {json.dumps(label)}')
        for column, character in enumerate(label):
            if character != ' ':
                lines.append(f"        memory[{number*16+column}]=8'h{ord(character):02X};")
    lines += ['    end', '    always @(posedge clk)if(read_enable)data<=memory[address];',
              'endmodule']
    return '\n'.join(lines)+'\n'+generate_help()


HELP_LABELS = {
 0:'',1:'WAIT USB CAMERA',2:'CAMERA ERROR',3:'NO DOG IN VIEW',4:'DOG SCORE LOW',
 5:'WAIT DOG STABLE',6:'TARGET EXPIRED',7:'CALIBRATE GIMBAL',8:'CENTERING DOG',
 9:'CLICK TRACK DOG',10:'MOVE DOG FORWARD',11:'PI COMMAND ERROR',12:'INSTALL POWEROFF',
 13:'POWER OFF FAILED',14:'POWER OFF SENT',15:'UPDATE PI V20',16:'READY TO FOLLOW',
 17:'DOG TRACKING',18:'CAMERA TOO OLD',19:'RETRY POWER OFF',20:'SET ORIGIN FIRST',
 21:'ORIGIN SET',22:'START UNKNOWN',23:'GOAL UNKNOWN',26:'PI LINK WAIT',27:'POWER OFF WAIT',28:'WAIT LIDAR SCAN',29:'WAIT GYRO ZERO',30:'CHECK MOTOR PID',31:'GOAL OUTSIDE MAP',32:'SEARCHING DOG',33:'SEARCH TURN 90',34:'DOG NOT FOUND',35:'CAMERA FRONT SET',36:'SET CAMERA FRONT',37:'LASER AMBIGUOUS',38:'WAIT LASER MATCH',39:'DOG POSITION SET',40:'SIX TASK DEMOS',41:'1 CAMERA + IMU',42:'2 MOTOR CONTROL',
 43:'3 LIDAR AVOID',44:'4 XY NAVIGATION',45:'5 VISION TARGET',46:'6 FIND DOG',
 47:'X FRONT Y LEFT',48:'MM / DEGREES',49:'ORIGIN SET',50:'CLICK TO MOVE',51:'STOPPED',
 52:'CLICK TRACK DOG',53:'MOVE DOG FORWARD',54:'CLICK START',55:'SET ORIGIN FIRST',
 56:'NO ROUTE RETRY',57:'POWER OFF PI'
}

def generate_help():
    lines=['module robot_status_help_rom(', '    input wire clk,read_enable,',
           '    input wire [9:0] address,', '    output reg [7:0] data', ');',
           '    (* syn_romstyle="block_rom" *) reg [7:0] memory[0:1023];',
           '    integer i;', '    initial begin',
           "        for(i=0;i<1024;i=i+1)memory[i]=8'h20;"]
    for index,label in HELP_LABELS.items():
        assert len(label)<=16 and label.isascii()
        for col,char in enumerate(label):
            if char!=' ':lines.append(f"        memory[{index*16+col}]=8'h{ord(char):02X};")
    lines+=['    end', '    always @(posedge clk)if(read_enable)data<=memory[address];','endmodule']
    return '\n'.join(lines)+'\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--write', action='store_true')
    args = parser.parse_args()
    source = SOURCE.read_text(encoding='utf-8')
    if 'labels[0:2047]' in source and source.startswith('// Compact atomic text:'):
        if args.write:
            parser.error('Compact UI owns its embedded 2KB ROM; legacy regeneration is disabled. No source file was written.')
        expected=bytearray(b' '*2048)
        for number,text in re.findall(r'^\s*// (\d+): (.*)$',source,re.M):
            number=int(number)
            assert 0<=number<128 and len(text)<=16 and text.isascii()
            expected[number*16:number*16+16]=text.ljust(16).encode('ascii')
        actual=bytearray(b' '*2048)
        for address,value in re.findall(r"labels\[(\d+)\]=8'h([0-9a-fA-F]{2});",source):
            address=int(address)
            assert 0<=address<2048
            actual[address]=int(value,16)
        assert actual==expected,'Compact ROM bytes differ from annotated labels'
        assert 'init_index<1024' in source and "labels[init_index+1024]=8'h20" in source
        print('PASS: compact embedded2048x8 byte ROM; annotated labels/spaces verified; legacy --write disabled.')
        return
    head, region = source.split(BEGIN, 1)
    old_rom, tail = region.split(END, 1)
    labels = labels_from_source(head)
    generated = generate(labels)
    if args.write:
        SOURCE.write_text(head+BEGIN+generated+END+tail, encoding='utf-8', newline='\n')
    else:
        assert old_rom == generated, 'ROM differs from label definitions; regenerate intentionally with --write'
    print(f'PASS: {len(labels)} exact16-byte labels; 4096x8 synchronousROM; '
          'all unused addresses SPACE; initialization1024 iterations, below Gowin2000 limit.')


if __name__ == '__main__':
    main()
