"""Compact UI references/contracts only: no HDL execution, device or network.

The decimal references compare serial BCD with ordinary Python formatting.
Frame/backpressure checks are a protocol model, not HDL simulation.
"""
from pathlib import Path
import random
import re
import unittest

from robot_ui_spec import VIEWS, SIDEBAR, button_rects, hit_test, route_for, generate_panel

ROOT=Path(__file__).resolve().parents[1]
STATUS=(ROOT/'src/robot_status_text.v').read_text(encoding='utf-8')
CONTROL=(ROOT/'src/robot_ui_control.v').read_text(encoding='utf-8')
PANEL=(ROOT/'src/robot_control_panel.v').read_text(encoding='utf-8')


def serial_bcd(value):
    bcd=0
    for bit in range(19,-1,-1):
        for digit in range(6):
            nibble=(bcd>>(4*digit))&15
            if nibble>=5:bcd+=3<<(4*digit)
        bcd=((bcd<<1)|((value>>bit)&1))&0xffffff
    return bcd


def numeric_row(prefix,value,*,degrees=False,unknown=False):
    magnitude=abs(value)
    digits=serial_bcd(magnitude)
    if degrees:digits>>=12
    observed=''.join(str((digits>>(4*d))&15) for d in range(5,-1,-1)).lstrip('0') or '0'
    return prefix.ljust(5)+' '+(' ' if unknown else '-' if value<0 else '+')+('------' if unknown else observed.rjust(6))+'   '


def rom_bytes():
    result=bytearray(b' '*2048)
    for index,value in re.findall(r"labels\[(\d+)\]=8'h([0-9a-fA-F]{2});",STATUS):
        result[int(index)]=int(value,16)
    return bytes(result)


class Tests(unittest.TestCase):
    def test_ten_core_views_and_every_visible_hit(self):
        self.assertEqual(set(VIEWS),{(0,0),(0,1),(2,1),(2,3),(3,3),(4,0),(4,1),(5,0),(5,3),(5,4)})
        self.assertNotIn(104,SIDEBAR)
        self.assertEqual([route_for((0,0),n) for n in range(1,7)],[(0,1),(4,0),(2,3),(4,1),(5,0),(5,3)])
        for view in VIEWS:
            for button,(left,top,right,bottom) in button_rects(*view).items():
                self.assertEqual(hit_test((left+right)//2,(top+bottom)//2,*view),button)
        self.assertEqual(PANEL,generate_panel())
        self.assertEqual(route_for((4,1),5),(3,3))
        self.assertEqual(route_for((3,3),6),(4,1))
        self.assertEqual(route_for((2,1),1),(2,3))
        self.assertEqual(route_for((5,3),4),(5,4))
        self.assertIn('RADAR +90',VIEWS[(2,3)].captions)

    def test_blank_actions_and_removed_editors(self):
        for view in [(5,0),(5,4)]:
            for slot,text in enumerate(VIEWS[view].captions,1):
                if not text:
                    x=408 if slot%2 else 608;y=128+88*((slot-1)//2)
                    self.assertEqual(hit_test(x+30,y+20,*view),0)
        self.assertTrue(all(not any(x in text for x in ('ARM','PID','GEOMETRY','ENCODER','SETTINGS')) for view in VIEWS.values() for text in view.captions))
        self.assertNotIn('reg signed [32:0] requested;',CONTROL)
        self.assertNotIn('case(selected_field)',CONTROL)
        self.assertNotIn('start_delay_ticks',CONTROL)

    def test_measured_configs_are_continuous_constants(self):
        for contract in ['assign wheel_mm_per_rev=16\'d204','assign encoder_counts_per_rev=16\'d1469',
                         "assign track_mm=16'd213", "assign body_front_mm=16'd232", "assign body_rear_mm=16'd118",
                         "assign drive_margin_mm=16'd20", "assign turn_margin_mm=16'd30", "assign contact_margin_mm=16'd10",
                         "assign lidar_offset_x_mm=16'sd33", "assign lidar_offset_y_mm=-16'sd77",
                         "assign follow_gain=8'd20", "assign follow_cap=16'd5"]:
            self.assertIn(contract,CONTROL)
        self.assertIn('forward_degrees<=90;',CONTROL)
        self.assertIn('forward_degrees>=270 ? forward_degrees-270 : forward_degrees+90',CONTROL)

    def test_stop_rebase_and_goal_steps(self):
        self.assertIn('rebase_pending<=3',CONTROL)
        self.assertIn('if(rebase_pending==1)',CONTROL)
        self.assertIn('pose_set<=1;map_clear<=1;',CONTROL)
        self.assertIn("assign pose_x_mm=32'sd0;assign pose_y_mm=32'sd0;assign pose_heading_mdeg=32'sd0;",CONTROL)
        self.assertIn('stop_contact',CONTROL)
        self.assertIn('armed<=0;command<=0;command_valid<=1;',CONTROL)
        self.assertIn('if(remote_takeover)',CONTROL)
        self.assertIn('if(page==4 && subpage==0 && current_button>=1 && current_button<=4)',CONTROL)
        rng=random.Random(5)
        for _ in range(10000):
            value=rng.randint(-3900,3900);step=rng.choice((10,100,500));sign=rng.choice((-1,1))
            total=value+sign*step
            observed=3900 if total>3900 else -3900 if total< -3900 else total
            self.assertEqual(observed,max(-3900,min(3900,total)))
        self.assertIn("step_select==0 ? 32'd10 : step_select==1 ? 32'd100 : 32'd500",CONTROL)

    def test_serial_decimal_six_digits_and_degrees(self):
        rng=random.Random(6)
        for value in [-524288,-180000,-1000,-999,-1,0,1,999,1000,65535,179999,524287]+[rng.randint(-524288,524287) for _ in range(10000)]:
            for degrees in (False,True):
                row=numeric_row('X MM',value,degrees=degrees)
                expected=abs(value)//1000 if degrees else abs(value)
                self.assertEqual(len(row),16)
                self.assertEqual(int(row[7:13]),expected)
                self.assertEqual(row[6],'-' if value<0 else '+')
        self.assertEqual(numeric_row('FRONT',65535,unknown=True),'FRONT  ------   ')

    def test_one_small_formatter_and_flag_contract(self):
        self.assertIn('reg signed [19:0] values[0:5];',STATUS)
        self.assertIn('reg [23:0] bcd,render_bcd;',STATUS)
        self.assertIn('labels[0:2047]',STATUS)
        self.assertNotIn('gain_product',STATUS)
        self.assertNotIn('held_reply_',STATUS)
        self.assertNotIn('emit_number',STATUS)
        self.assertNotIn('values[0:9]',STATUS)
        self.assertIn('held_flags<={gains_ready,subpage,lidar_error[4:0],nav_fault,nav_status,map_ready,safety_stop,scan_enable,',STATUS)
        self.assertIn('lidar_ready,calibrating,gyro_calibrated,geometry_valid,armed,page};',STATUS)
        for page in range(6):
            for sub in range(8):
                f=(1<<31)|(sub<<28)|(31<<23)|(173<<15)|(11<<11)|(1<<10)|(1<<9)|(1<<8)|(1<<7)|(1<<6)|(1<<5)|(1<<4)|(1<<3)|page
                self.assertEqual(f&7,page);self.assertEqual((f>>28)&7,sub)
                self.assertEqual((f>>11)&15,11);self.assertEqual((f>>15)&255,173)

    def test_rom_labels_and_full_byte_protocol(self):
        rom=rom_bytes()
        self.assertEqual(len(rom),2048)
        for index,text in {26:'PI LINK WAIT',31:'GOAL OUTSIDE MAP',40:'SIX TASK DEMOS',43:'3 LIDAR AVOID',44:'4 XY NAVIGATION',46:'6 FIND FACE',51:'ALL STOP',65:'X MM',67:'YAW',78:'FRONT',84:'VEL',
                           85:'START BLOCKED',86:'GOAL BLOCKED',87:'NO KNOWN PATH',88:'PLAN TIMEOUT',89:'START OFF MAP',90:'GOAL OFF MAP',91:'WAIT MAP SCANS',
                           92:'MAP WAIT TIMEOUT',93:'POSE NOT READY',94:'MAP NOT READY',95:'CONTACT STOP',96:'FINAL LEG BLOCK',97:'CHECK SENSORS',98:'NAV MATH FAULT',99:'OUTSIDE MAP',
                           100:'NO FACE IN VIEW',101:'FACE SCORE LOW',102:'WAIT FACE STABLE',103:'FACE TRACKING',104:'SEARCHING FACE',105:'FACE NOT FOUND',106:'FACE POSITION',107:'HEAD TO 135'}.items():
            self.assertEqual(rom[index*16:index*16+16].decode('ascii'),text.ljust(16))
        self.assertEqual(VIEWS[(5,0)].captions[:3],('TRACK DOG','TRACK FACE','STOP TRACK'))
        self.assertIn('FIND FACE',VIEWS[(5,3)].captions)
        self.assertIn("if(follow_diagnostic>=40 && follow_diagnostic<=47)live_status=7'd100+{1'b0,follow_diagnostic[5:0]}-7'd40;",STATUS)
        self.assertIn('if((view==33 || view==43) && waiting_map)live_status=91;',STATUS)
        for error,label in {1:88,2:85,3:86,4:87,8:22,9:23,11:89,12:90,13:92,14:93,15:94,16:95,17:96,18:97,19:98,20:99}.items():
            self.assertIn(f'{error}:planner_label={label};',STATUS)
        rows=[rom[40*16:40*16+16],rom[20*16:20*16+16],numeric_row('X MM',-300).encode(),numeric_row('Y MM',120).encode()]+[b' '*16]*4+[rom[52*16:52*16+16],rom[53*16:53*16+16],rom[47*16:47*16+16]]+[b' '*16]*5
        golden=b''.join(rows);self.assertEqual(len(golden),256)
        # Independent consumer model: bytes/index stay stable while not ready,
        # including at row boundaries and final byte before frame commit.
        rng=random.Random(7);accepted=[];index=0;stalled=None
        while index<256:
            offer=(index,golden[index]);ready=rng.randrange(4)==0
            if stalled is not None:self.assertEqual(offer,stalled)
            if ready:accepted.append(offer[1]);index+=1;stalled=None
            else:stalled=offer
        self.assertEqual(bytes(accepted),golden)
        self.assertIn('assign text_byte_index={row,column};',STATUS)
        self.assertIn('S_SEND:if(text_byte_ready)begin',STATUS)
        self.assertIn('S_REQUEST:begin label_byte<=labels[{current_label,column}];state<=S_SEND;end',STATUS)
        self.assertIn('S_COMMIT:begin flags<=held_flags;text_valid<=1;state<=S_IDLE;end',STATUS)

    def test_map_two_pixel_addressing(self):
        self.assertIn("(x-11'd31)>>1",PANEL)
        self.assertIn("(10'd367-y)>>1",PANEL)
        self.assertNotIn("/11'd3",PANEL)
        for cell in range(128):
            for offset in (0,1):
                pixel=32+cell*2+offset
                previous=pixel-1
                self.assertEqual((previous-31)>>1,cell)
                y=367-cell*2-offset
                self.assertEqual((367-y)>>1,cell)
        self.assertNotIn('car_border',PANEL)


if __name__=='__main__':unittest.main()
