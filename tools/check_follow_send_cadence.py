"""Exercise actual FollowLink.send at50Hz with fake UDP; no network/devices."""
from copy import deepcopy
from dataclasses import asdict
import binascii
from pathlib import Path
import struct
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'pc'))
from gimbal_control import GimbalController, default_config
from robot_follow import FollowLink, FpgaPWMBackend, packet


class FakeSocket:
    def __init__(self):
        self.sent = []
        self.attempts = 0
        self.fail = False

    def setsockopt(self, *args): pass
    def bind(self, *args): pass
    def setblocking(self, *args): pass

    def sendto(self, raw, peer):
        self.attempts += 1
        if self.fail:
            raise OSError(101, 'Network is unreachable')
        self.sent.append((raw, peer))
        return len(raw)


class SendCadence(unittest.TestCase):
    def setUp(self):
        # Opening a real socket is a test failure, including indirect opens.
        self.socket_patch = mock.patch('socket.socket', side_effect=AssertionError('real network forbidden'))
        self.socket_patch.start()
        self.addCleanup(self.socket_patch.stop)
        (ROOT / 'tmp').mkdir(exist_ok=True)
        temporary = tempfile.TemporaryDirectory(dir=ROOT / 'tmp')
        self.addCleanup(temporary.cleanup)
        self.sock = FakeSocket()
        self.backend = FpgaPWMBackend()
        self.link = FollowLink(self.backend, temporary.name, sock=self.sock,
                               initial_tilt=1750, initial_tilt_angle=135)
        self.addCleanup(self.link.log.close)
        # Isolate FOL cadence: catalogue retains its independent200ms timer.
        self.link.last_catalog = 1e9
        config = default_config()
        for axis in ('pan', 'tilt'):
            config['axes'][axis].update(
                safe_min_us=1000, safe_max_us=2000,
                calibration_points=[dict(angle_deg=0, pulse_us=1000),
                                    dict(angle_deg=180, pulse_us=2000)],
                calibration_confirmed=True)
        self.gimbal = GimbalController(config, self.backend, clock=lambda: 0,
                                       operating_angle_min={'tilt': 90})
        self.addCleanup(self.gimbal.close)
        self.gimbal.arm('pan', 1200, physical=True)
        self.gimbal.arm('tilt', 1750, physical=True)
        self.app = SimpleNamespace(gimbal=self.gimbal, config=config, latest=None,
                                   tracking_result={}, tracking=False, error=None)

    def test_actual_send_every20ms_without_duplicates_or_catchup(self):
        #100Hz control calls for1second must produce51 FOL packets, not the
        # former50ms cadence nor a float-induced30ms cadence.
        for index in range(101):
            now = 100 + index * .01
            self.link.send(self.app, now)
            self.link.send(self.app, now)  # Same timestamp cannot send twice.
        self.assertEqual(len(self.sock.sent), 51)
        for sequence, (raw, peer) in enumerate(self.sock.sent):
            self.assertEqual(peer, ('192.168.10.255', 5001))
            self.assertEqual(len(raw), 32)
            self.assertEqual(raw[:6], b'FOL1\x01\x18')
            self.assertEqual(int.from_bytes(raw[6:8], 'big'), sequence)
            expected = struct.pack('>h10H', 0, 1200, 1750, 1000, 2000,
                                   1500, 2000, 65535, 0, 0, 0)
            self.assertEqual(raw, packet(b'FOL1', 24, sequence, expected))
            self.assertEqual(binascii.crc_hqx(raw[:30], 0xffff), int.from_bytes(raw[30:], 'big'))
        self.link.send(self.app, 102)
        self.link.send(self.app, 102)
        self.assertEqual(len(self.sock.sent), 52)  # No burst after a stall.

    def test_network_loss_retries_without_mutating_calibration_or_pwm(self):
        before_config = deepcopy(self.app.config)
        before_axes = {axis: asdict(cfg) for axis, cfg in self.gimbal.axes.items()}
        before_pwm = dict(self.backend.enabled)
        self.link.follower.capture(36, .02)
        self.link.follower.forward_sign = 1
        self.link.send(self.app, 200)
        sequence = self.link.sequence
        self.sock.fail = True
        self.link.send(self.app, 200.02)
        self.assertIn('Network is unreachable', self.link.transport_error)
        self.assertIsNone(self.link.error)  # Transport failure is not PWM fault.
        self.assertEqual(self.link.sequence, sequence)
        attempts = self.sock.attempts
        self.link.send(self.app, 200.039)
        self.assertEqual(self.sock.attempts, attempts)
        self.sock.fail = False
        self.link.send(self.app, 200.04)
        self.assertIsNone(self.link.transport_error)
        self.assertEqual(self.link.sequence, sequence + 1)
        self.assertEqual(self.app.config, before_config)
        self.assertEqual({axis: asdict(cfg) for axis, cfg in self.gimbal.axes.items()}, before_axes)
        self.assertEqual(self.backend.enabled, before_pwm)
        self.assertEqual(self.link.follower.reference, (36, .02))
        self.assertEqual(self.link.follower.forward_sign, 1)

    def test_five_seconds_active_geometry_preserves_vec_goal_and_catalog_rates(self):
        self.link.report = dict(flags=128, gain=20, cap=5)
        self.link.geometry_active = True
        self.link.last_catalog = -1e9
        self.link.registry.selected = 1
        self.link.visual.config = {}
        self.link.visual.reference = dict(angle_signature='synthetic-measured-table', offset=0)
        self.link.visual.lock = (1500, 100)
        self.link.visual.lock_target = 1
        self.app._angle_signature = lambda: 'synthetic-measured-table'
        self.app._mount_upright = lambda: 180
        self.app.tracking = True
        target = dict(x1=280, y1=200, x2=360, y2=280, kind='dog', score=.9)
        self.app.tracking_result = dict(status='tracking', target=target)
        for index in range(501):
            now = 300 + index * .01
            self.link.report_at = now
            self.app.latest = SimpleNamespace(arrived_at=now, width=640, height=480,
                                              detections=[target], gimbal_at_arrival=self.gimbal.snapshot())
            self.link.registry.update([target], now, index + 1, 640, 480)
            self.link.visual.laser = dict(flags=1, x=0, y=0)
            self.link.visual.laser_at = now
            self.link.send(self.app, now)
        grouped = {magic: [raw for raw, _ in self.sock.sent if raw[:4] == magic]
                   for magic in (b'FOL1', b'VEC1', b'GOL1', b'TGT1')}
        self.assertEqual(len(grouped[b'FOL1']), 251)
        for magic in (b'VEC1', b'GOL1'):
            self.assertTrue(80 <= len(grouped[magic]) <= 101, (magic, len(grouped[magic])))
        self.assertTrue(23 <= len(grouped[b'TGT1']) <= 26)
        self.assertTrue(all(raw[5] & 1 for raw in grouped[b'VEC1']))
        for raw in grouped[b'GOL1']:
            selected, age, x, y, lock_x, lock_y, standoff = struct.unpack('>HHiiiiH', raw[8:30])
            self.assertEqual((selected, age, lock_x, lock_y, standoff), (1, 0, 1500, 100, 512))
            self.assertTrue(900 < x < 1500 and 0 < y < 100)
        for packets in grouped.values():
            for raw in packets:
                self.assertEqual(len(raw), 32)
                self.assertEqual(binascii.crc_hqx(raw[:30], 0xffff), int.from_bytes(raw[30:], 'big'))
        self.assertIsNone(self.link.error)


if __name__ == '__main__':
    unittest.main(verbosity=2)
