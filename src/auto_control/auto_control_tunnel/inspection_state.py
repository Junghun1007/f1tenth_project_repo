"""Read-only startup display state. The C++ controller authorizes movement."""
import time


class InspectionState:
    def __init__(self, expected):
        self.camera = 'WAITING | waiting for camera measurement'
        self.models = dict.fromkeys(expected, 'WAITING | not loaded')
        self.controller = 'WAITING | waiting for controller'
        self.controller_at = None
        self.error = ''

    def update_controller(self, text, now=None):
        self.controller = text
        self.controller_at = time.monotonic() if now is None else now

    def controller_fresh(self, now):
        return self.controller_at is not None and 0 <= now - self.controller_at <= 1.0

    def checks_ready(self):
        return self.camera.startswith('READY | ') and all(
            value.startswith('READY | ') for value in self.models.values())

    def can_start(self, now=None):
        now = time.monotonic() if now is None else now
        return self.controller_fresh(now) and self.checks_ready() and self.controller.startswith('READY_TO_START | ')

    def completed(self, now=None):
        now = time.monotonic() if now is None else now
        return self.controller_fresh(now) and (
            self.controller.startswith('STARTED | ') or
            (self.controller.startswith('MANUAL | ') and self.checks_ready()))
