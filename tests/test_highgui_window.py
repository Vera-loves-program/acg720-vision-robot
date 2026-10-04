"""Regression cases for real backend differences without requiring a desktop."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "pc"))
from highgui_window import WindowCloseMonitor


class FakeCV2:
    WND_PROP_VISIBLE = 4
    WND_PROP_AUTOSIZE = 1

    class error(Exception):
        pass

    def __init__(self, visible, autosize=1):
        self.visible = visible
        self.autosize = autosize

    def getWindowProperty(self, name, prop):
        value = self.visible if prop == self.WND_PROP_VISIBLE else self.autosize
        if isinstance(value, Exception):
            raise value
        return value


class WindowCloseTests(unittest.TestCase):
    def monitor(self, cv2):
        return WindowCloseMonitor(cv2, "test", report=lambda *args, **kwargs: None)

    def test_gtk_unsupported_visibility_keeps_live_window_then_detects_close(self):
        for unsupported in (-1.0, float("nan"), FakeCV2.error("unsupported")):
            with self.subTest(unsupported=unsupported):
                cv2 = FakeCV2(unsupported)
                monitor = self.monitor(cv2)
                self.assertFalse(monitor.closed())
                self.assertFalse(monitor.closed())
                cv2.autosize = -1
                self.assertTrue(monitor.closed())

    def test_qt_visible_window_then_hidden_or_missing_exits(self):
        for missing in (0, -1, FakeCV2.error("window missing")):
            with self.subTest(missing=missing):
                cv2 = FakeCV2(1)
                monitor = self.monitor(cv2)
                self.assertFalse(monitor.closed())
                cv2.visible = missing
                self.assertTrue(monitor.closed())

    def test_initial_hidden_window_is_allowed_to_map(self):
        cv2 = FakeCV2(0)
        monitor = self.monitor(cv2)
        self.assertFalse(monitor.closed())
        cv2.visible = 1
        self.assertFalse(monitor.closed())
        cv2.visible = 0
        self.assertTrue(monitor.closed())

    def test_unknown_properties_do_not_falsely_exit(self):
        monitor = self.monitor(FakeCV2(-1, -1))
        self.assertFalse(monitor.closed())
        self.assertFalse(monitor.closed())


if __name__ == "__main__":
    unittest.main()
