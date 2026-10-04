"""Local preview coordinates and honest target/control state; no hardware tests."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "pc"))
from udp_video_viewer import BUTTON_RECTS, ViewerState, command_packet
from ui_telemetry import UITelemetry, parse_ui_telemetry


def telemetry(**updates):
    fields = dict(gaussian_on=True, debug=True, selected=False, touch_ready=True,
                  touch_identified=True, stop=False, zoom_code=0, contacts=0,
                  selection_counter=7, capture_counter=6, clear_counter=2,
                  camera_x=400, camera_y=240, touch_error=0, tx_sequence=10,
                  uptime_ticks=1000)
    fields.update(updates)
    return UITelemetry(**fields)


def telemetry_packet_v2(step=128, origin_x=100, origin_y=60, zoom_mode=True):
    packet = bytearray(32)
    packet[:4] = b"VUI1"
    packet[4] = 2
    packet[5] = 0x1B  # Gaussian, debug, touch ready/identified; no selected point.
    packet[6] = int(step != 256)
    packet[14:16] = (400).to_bytes(2, "big")
    packet[16:18] = (240).to_bytes(2, "big")
    packet[20:24] = (10).to_bytes(4, "big")
    packed = (int(zoom_mode) << 28) | (step << 19) | (origin_y << 10) | origin_x
    packet[28:32] = packed.to_bytes(4, "big")
    return packet


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

    def test_zoom_centers_on_focus_and_target_stays_in_source_coordinates(self):
        state = ViewerState(debug_mode=True, has_frame=True)
        before = state.source_point(500, 350)
        state.zoom_at(500, 350, 1)
        centered = state.source_point(420, 324)
        self.assertAlmostEqual(before[0], centered[0], delta=0.6)
        self.assertAlmostEqual(before[1], centered[1], delta=0.6)
        self.assertEqual(state.zoom_focus, before)
        state.action("zoom_mode")
        target = state.double_click(420, 324)
        self.assertEqual(target.point, centered)
        for _ in range(30):
            state.change_zoom(1)
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
        state.action("zoom_mode")
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
        state.click(900, 515)
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

    def test_zoom_and_target_double_click_are_separate_and_repeat_cancels(self):
        state = ViewerState(debug_mode=True, has_frame=True)
        target = state.double_click(420, 324)
        state.action("zoom_mode")
        self.assertIsNone(state.double_click(500, 350))
        self.assertEqual(state.zoom_focus, (240, 133))
        self.assertEqual(state.target, target)
        self.assertEqual(state.pending_commands, [])
        state.action("zoom_in")
        self.assertAlmostEqual(state.zoom, 1.25)
        self.assertEqual(state.target, target)
        state.action("zoom_out")
        self.assertAlmostEqual(state.zoom, 1.0)
        state.action("zoom_mode")
        self.assertIsNone(state.double_click(420, 324))
        self.assertIsNone(state.target)
        self.assertIn("not connected", state.notice)
        state.debug_mode = False
        self.assertIsNone(state.double_click(420, 324))
        self.assertIsNone(state.target)

    def test_zoom_buttons_without_video_and_sidebar_are_bounded(self):
        state = ViewerState()
        state.action("zoom_in")
        self.assertEqual(state.crop_bounds(), (0, 0, 400, 240))
        self.assertIsNone(state.double_click(420, 324))
        state.has_frame = True
        state.set_zoom_focus(999, -10)
        for _ in range(20):
            state.action("zoom_in")
        self.assertEqual(state.zoom, 4.0)
        self.assertEqual(state.zoom_focus, (399.0, 0.0))
        self.assertEqual(state.crop_bounds(), (300, 0, 400, 60))
        self.assertEqual(state.source_point(20, 84), (300, 0))
        self.assertIsNone(state.source_point(820, 564))
        rects = list(BUTTON_RECTS.values())
        for i, a in enumerate(rects):
            for b in rects[i + 1:]:
                self.assertFalse(max(a[0], b[0]) < min(a[2], b[2]) and
                                 max(a[1], b[1]) < min(a[3], b[3]))

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

    def test_lcd_double_click_cancels_without_clear_counter_and_baseline_does_not_replay(self):
        state = ViewerState(debug_mode=True, has_frame=True)
        state.apply_telemetry(telemetry(selected=False, selection_counter=30))
        self.assertIn("synchronized", state.notice)
        self.assertNotIn("cancelled target by double-click", state.notice)
        state.apply_telemetry(telemetry(tx_sequence=11, selected=True, selection_counter=31))
        self.assertIsNotNone(state.target)
        state.apply_telemetry(telemetry(tx_sequence=12, selected=False, selection_counter=32))
        self.assertIsNone(state.target)
        self.assertEqual(state.ui_telemetry.clear_counter, 2)
        self.assertIn("cancelled target by double-click", state.notice)
        self.assertEqual(state.pending_commands, [])
        state.select_source_target(200, 120)
        state.apply_telemetry(telemetry(tx_sequence=13, selected=False, selection_counter=32))
        self.assertIsNotNone(state.target)  # Same counter is state, not a new cancellation.

    def test_lcd_zoom_uses_central_crop_with_off_center_target(self):
        state = ViewerState(debug_mode=True, has_frame=True)
        state.apply_telemetry(telemetry())
        target = state.select_source_target(20, 30)
        state.apply_telemetry(telemetry(tx_sequence=11, zoom_code=1))
        self.assertEqual(state.crop_bounds(), (100, 60, 300, 180))
        self.assertEqual(state.source_point(420, 324), (200, 120))
        self.assertEqual(state.target, target)

    def test_v2_actual_crop_follows_even_with_unchanged_coarse_zoom(self):
        state = ViewerState(has_frame=True)
        parsed = parse_ui_telemetry(telemetry_packet_v2())
        self.assertTrue(state.apply_telemetry(parsed))
        self.assertEqual(state.crop_bounds(), (50, 30, 250, 150))
        self.assertEqual(state.source_point(420, 324), (150, 90))
        self.assertTrue(state.zoom_mode)
        state.set_zoom_focus(300, 180)
        state.change_zoom(1)
        local_crop = state.crop_bounds()
        state.apply_telemetry(telemetry(version=2, tx_sequence=11, zoom_code=1,
                                       zoom_step_q8=128, view_origin_x=100,
                                       view_origin_y=60, zoom_mode=True))
        self.assertEqual(state.crop_bounds(), local_crop)
        state.apply_telemetry(telemetry(version=2, tx_sequence=12, zoom_code=1,
                                       zoom_step_q8=96, view_origin_x=250,
                                       view_origin_y=150, zoom_mode=True))
        self.assertEqual(state.crop_bounds(), (125, 75, 275, 165))
        self.assertAlmostEqual(state.zoom, 256 / 96)
        state.apply_telemetry(telemetry(version=2, tx_sequence=13, zoom_code=1,
                                       zoom_step_q8=96, view_origin_x=300,
                                       view_origin_y=180, zoom_mode=True))
        self.assertEqual(state.crop_bounds(), (150, 90, 300, 180))
        self.assertEqual(state.source_point(420, 324), (225, 135))

    def test_v2_fractional_footprint_and_odd_origins_cover_source_edges(self):
        parsed = parse_ui_telemetry(telemetry_packet_v2(step=65, origin_x=595, origin_y=357))
        self.assertIsNotNone(parsed)
        state = ViewerState()
        state.apply_telemetry(parsed)
        self.assertEqual(state.crop_bounds(), (297, 178, 400, 240))
        self.assertEqual(state.source_point(20, 84), (297, 178))
        self.assertEqual(state.zoom, 256 / 65)

    def test_v2_protocol_geometry_and_reserved_fields(self):
        parsed = parse_ui_telemetry(telemetry_packet_v2())
        self.assertEqual((parsed.version, parsed.zoom_step_q8, parsed.view_origin_x,
                          parsed.view_origin_y, parsed.zoom_mode), (2, 128, 100, 60, True))
        full = parse_ui_telemetry(telemetry_packet_v2(step=256, origin_x=0, origin_y=0))
        self.assertEqual(full.zoom_code, 0)
        maximum = parse_ui_telemetry(telemetry_packet_v2(step=64, origin_x=600, origin_y=360))
        self.assertIsNotNone(maximum)
        for kwargs in ({"step": 63}, {"step": 257}, {"step": 256, "origin_x": 1, "origin_y": 0},
                       {"step": 128, "origin_x": 401}, {"step": 128, "origin_y": 241},
                       {"step": 65, "origin_x": 597, "origin_y": 0},
                       {"step": 65, "origin_x": 0, "origin_y": 359}):
            self.assertIsNone(parse_ui_telemetry(telemetry_packet_v2(**kwargs)), kwargs)
        for position, value in ((4, 3), (5, 0x80), (6, 0), (19, 1), (28, 0x20)):
            invalid = telemetry_packet_v2()
            invalid[position] = value
            self.assertIsNone(parse_ui_telemetry(invalid), (position, value))

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
        self.assertEqual(parse_ui_telemetry(packet).version, 1)
        for position, value in ((4, 3), (5, 0x80), (6, 2), (7, 6), (19, 1), (28, 1)):
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
