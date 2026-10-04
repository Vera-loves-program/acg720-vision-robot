"""Window-close detection compatible with Qt and Ubuntu's GTK OpenCV builds.

WND_PROP_VISIBLE is not implemented by some GTK backends. A negative value
or NaN on a live window therefore cannot by itself mean the window was closed.
This module deliberately does not import cv2, so headless capture stays usable.
"""
import math


class WindowCloseMonitor:
    def __init__(self, cv2, name, report=print):
        self.cv2 = cv2
        self.name = name
        self.report = report
        self.seen_visible = False
        self.seen_autosize = False
        self.reported_fallback = False

    def _property(self, prop):
        try:
            value = self.cv2.getWindowProperty(self.name, prop)
        except self.cv2.error:
            return None
        return value if math.isfinite(value) and value >= 0 else None

    def closed(self):
        visible = self._property(self.cv2.WND_PROP_VISIBLE)
        if visible is not None and visible > 0:
            self.seen_visible = True
            return False
        if self.seen_visible:
            # A backend that previously reported visibility now reports hidden
            # or missing. Call this after waitKey, before the next imshow.
            return True

        autosize = self._property(self.cv2.WND_PROP_AUTOSIZE)
        if autosize is not None:
            self.seen_autosize = True
            if visible is None and not self.reported_fallback:
                self.report("GUI: visibility query unavailable; using window-existence check. "
                            "Press Q/Esc in the image window, or Ctrl+C in the terminal, to quit.",
                            flush=True)
                self.reported_fallback = True
            return False
        # If neither query ever worked, keep running rather than falsely quit.
        # Keyboard and Ctrl+C still work on backends with no property support.
        return self.seen_autosize


def report_gui(cv2):
    gui = next((line.strip() for line in cv2.getBuildInformation().splitlines()
                if line.strip().startswith("GUI:")), "GUI: unknown")
    print(f"OpenCV {cv2.__version__}; {gui}", flush=True)
