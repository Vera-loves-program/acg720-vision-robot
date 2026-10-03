"""Local preview coordinates and honest target/control state; no hardware tests."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "pc"))
from udp_video_viewer import ViewerState, command_packet
from ui_telemetry import UITelemetry, parse_ui_telemetry


def telemetry(**updates):
    fields = dict(gaussian_on=True, debug=True, selected=False, touch_ready=True,
                  touch_identified=True, stop=False, zoom_code=0, contacts=0,
                  selection_counter=7, capture_counter=6, clear_counter=2,
                  camera_x=400, camera_y=240, touch_error=0, tx_sequence=10,
                  uptime_ticks=1000)
    fields.update(updates)
    return UITelemetry(**fields)


class InteractionTests(unittest.TestCase):
    def test_selection_requires_debug_and_valid_video(self):
        state = ViewerState()
        self.assertIsNone(state.select_target(420, 324))
        state.action("debug")
        self.assertIsNone(state.select_target(420, 324))
        state.has_frame = True
        target = state.select_target(420, 324)
        self.assertEqual(target.point, (200, 120))
        self.assertEqual(target.roi, (160, 80, 240, 160))
        self.assertIn("tracker is not connected", state.notice)
        # Only the explicit debug action emits a VRB1 flag request.
        self.assertEqual(state.pending_commands, [(2, 1)])

    def test_zoom_anchor_and_target_coordinates_remain_on_source(self):
        state = ViewerState(debug_mode=True, has_frame=True)
        before = state.source_point(500, 350)
        state.zoom_at(500, 350, 1)
        after = state.source_point(500, 350)
        self.assertAlmostEqual(before[0], after[0], delta=0.6)
        self.assertAlmostEqual(before[1], after[1], delta=0.6)
        target = state.select_target(500, 350)
        self.assertEqual(target.point, after)
        for _ in range(30):
            state.zoom_at(500, 350, 1)
        self.assertEqual(state.zoom, 4.0)
        self.assertEqual(state.target, target)
        state.action("reset_view")
        self.assertEqual(state.crop_bounds(), (0, 0, 400, 240))
        self.assertEqual(state.target, target)

    def test_edge_crops_and_roi_are_bounded(self):
        state = ViewerState(debug_mode=True, has_frame=True)
        for _ in range(20):
            state.zoom_at(20, 84, 1)
        self.assertEqual(state.crop_bounds(), (0, 0, 100, 60))
        target = state.select_target(20, 84)
        self.assertEqual(target.roi, (0, 0, 80, 80))
        self.assertIsNone(state.source_point(820, 564))
        self.assertIsNone(state.select_target(950, 350))
        for _ in range(20):
            state.zoom_at(20, 84, -1)
        self.assertEqual(state.zoom, 1.0)

    def test_buttons_cancel_and_unconfirmed_commands(self):
        state = ViewerState(debug_mode=True, has_frame=True)
        state.select_target(420, 324)
        state.click(900, 480)
        self.assertIsNone(state.target)
        self.assertIsNone(state.filter_requested)
        state.action("filter")
        state.action("filter")
        state.action("stop")
        state.action("clear")
        self.assertEqual(state.pending_commands, [(1, 1), (1, 0), (3, 1), (4, 0)])
        self.assertIn("unconfirmed", state.notice)
        packet = command_packet(2, 1, 256)
        self.assertEqual(packet[:7], b"VRB1\x02\x01\x00")
        checksum = 0
        for byte in packet:
            checksum ^= byte
        self.assertEqual(checksum, 0)

    def test_touch_baseline_then_new_selection_zoom_cancel(self):
        state = ViewerState()
        self.assertTrue(state.apply_telemetry(telemetry()))
        self.assertIsNone(state.target)
        self.assertNotIn("CAPTURE pressed", state.notice)
        state.apply_telemetry(telemetry(tx_sequence=11, selected=True,
                                        selection_counter=8, camera_x=798, camera_y=478, zoom_code=1))
        self.assertEqual(state.target.point, (399, 239))
        self.assertEqual(state.zoom, 2.0)
        self.assertEqual(state.pending_commands, [])
        state.apply_telemetry(telemetry(tx_sequence=12, clear_counter=3, zoom_code=1))
        self.assertIsNone(state.target)
        state.apply_telemetry(telemetry(tx_sequence=13, clear_counter=3, capture_counter=7, zoom_code=1))
        self.assertIn("CAPTURE pressed", state.notice)

    def test_telemetry_duplicate_reorder_wrap_and_reboot(self):
        state = ViewerState()
        state.apply_telemetry(telemetry(tx_sequence=0xFFFFFFFF, capture_counter=0xFFFF))
        state.apply_telemetry(telemetry(tx_sequence=0, capture_counter=0, uptime_ticks=1001))
        self.assertIn("CAPTURE pressed", state.notice)
        self.assertFalse(state.apply_telemetry(telemetry(tx_sequence=0, capture_counter=5, uptime_ticks=1001)))
        self.assertFalse(state.apply_telemetry(telemetry(tx_sequence=0xFFFFFFFF, capture_counter=5, uptime_ticks=1000)))
        # Move beyond sequence zero, then accept a new boot as a baseline.
        state.apply_telemetry(telemetry(tx_sequence=10, capture_counter=8, uptime_ticks=1100))
        state.apply_telemetry(telemetry(tx_sequence=0, capture_counter=0, uptime_ticks=0))
        self.assertIn("synchronized", state.notice)
        self.assertNotIn("CAPTURE pressed", state.notice)

    def test_lcd_zoom_uses_central_crop_with_off_center_target(self):
        state = ViewerState(debug_mode=True, has_frame=True)
        state.apply_telemetry(telemetry())
        target = state.select_source_target(20, 30)
        state.apply_telemetry(telemetry(tx_sequence=11, zoom_code=1))
        self.assertEqual(state.crop_bounds(), (100, 60, 300, 180))
        self.assertEqual(state.source_point(420, 324), (200, 120))
        self.assertEqual(state.target, target)

    def test_r3_cancel_and_reset_request_only_with_recent_telemetry(self):
        state = ViewerState()
        state.action("cancel_target")
        state.action("reset_view")
        self.assertEqual(state.pending_commands, [])
        state.apply_telemetry(telemetry())
        state.action("cancel_target")
        state.action("reset_view")
        self.assertEqual(state.pending_commands, [(5, 0), (6, 0)])
        state.pending_commands.clear()
        state.ui_seen_at -= 4.0
        state.action("cancel_target")
        state.action("reset_view")
        self.assertEqual(state.pending_commands, [])

    def test_ui_telemetry_protocol_boundaries(self):
        packet = bytearray(32)
        packet[:4] = b"VUI1"
        packet[4] = 1
        packet[5] = 0x1F
        packet[6] = 1
        packet[7] = 5
        packet[14:16] = (799).to_bytes(2, "big")
        packet[16:18] = (479).to_bytes(2, "big")
        self.assertEqual(parse_ui_telemetry(packet).camera_x, 799)
        for position, value in ((4, 2), (5, 0x80), (6, 2), (7, 6), (19, 1), (28, 1)):
            invalid = packet.copy()
            invalid[position] = value
            self.assertIsNone(parse_ui_telemetry(invalid))
        invalid = packet.copy()
        invalid[14:16] = (800).to_bytes(2, "big")
        self.assertIsNone(parse_ui_telemetry(invalid))
        invalid = packet.copy()
        invalid[16:18] = (480).to_bytes(2, "big")
        self.assertIsNone(parse_ui_telemetry(invalid))
        self.assertIsNone(parse_ui_telemetry(packet[:-1]))


if __name__ == "__main__":
    unittest.main()
