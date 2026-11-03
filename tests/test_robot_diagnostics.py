"""Host diagnostic protocol and decode checks; no serial device is opened."""
import importlib.util
from pathlib import Path
import unittest

spec=importlib.util.spec_from_file_location('robot_diag',Path(__file__).resolve().parents[1]/'tools/read_robot_diagnostics.py')
diag=importlib.util.module_from_spec(spec);spec.loader.exec_module(diag)

class DiagnosticsTests(unittest.TestCase):
    def test_no_live_rejection_does_not_hide_a_mission_failure(self):
        words=[0]*48;words[0]=0x4c440101
        words[1]=(17<<16)|(4<<8)|11
        result=diag.decode_words(words)
        self.assertEqual(result['reason_code'],0)
        self.assertEqual(result['mission_error'],17)
        self.assertEqual(result['planner_error'],4)
        summary=diag.format_summary(result)
        self.assertIn('mission=17(precise final leg blocked)',summary)
        self.assertIn('planner=4',summary)
        self.assertIn('nav=11',summary)
        self.assertNotIn('ready/no',summary)

    def test_record_keeps_unknown_hit_visibility_and_signed_grid_separate(self):
        words=[0]*48;words[0]=0x4c440101
        words[1]=(0x02<<24)|(14<<16)|(8<<8)|8
        words[6]=1 # front hit present, NOT sampled visibility.
        words[8]=1<<17 # front required, sampled visibility=0, no obstacle flags.
        words[9]=(32<<16)|123
        words[12]=(233<<16)|((-77)&65535)
        words[18]=(-1)&0xffffffff;words[19]=64
        words[20]=(1<<9)|(3<<5)|(0<<3)|1
        words[39]=6;words[46]=7;words[47]=65536|17
        result=diag.decode_words(words)
        self.assertEqual(result['reason_code'],6)
        self.assertEqual(result['scan']['hit_coverage'],1)
        self.assertEqual(result['scan']['sampled_visibility'],0)
        self.assertFalse(result['obstacle']['front'])
        self.assertEqual(result['obstacle']['body_y_mm'],-77)
        self.assertEqual(result['obstacle']['trigger_kind'],32)
        self.assertEqual(result['map']['rejected_fine_x'],-1)
        self.assertTrue(result['mixed_epoch'])
        self.assertEqual(result['snapshot_epoch'],17)
        with self.assertRaises(ValueError):diag.decode_words([0]*48)

    def test_only_read_operation_sent_and_samples_other_ack_ignored(self):
        class Port:
            def __init__(self):self.sent=[];self.lines=iter([b'samples:1,2,3,4\n',b'[ACK OK] 9,10,0,0,0,7\n',b'[ACK OK] 10,10,0,0,0,-77\n'])
            def write(self,data):self.sent.append(data)
            def readline(self):return next(self.lines)
        port=Port()
        self.assertEqual(diag.query(port,10,0,12),(-77)&0xffffffff)
        self.assertEqual(port.sent,[b'#10,10,0,0,12\n'])

if __name__=='__main__':unittest.main()
