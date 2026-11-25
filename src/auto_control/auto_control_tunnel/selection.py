"""Select the requested lane source immediately on an accepted tunnel event."""
import math
from .path_validation import stamp_ns


class LaneSelector:
    def __init__(self, max_age=.2, validation_mode=False):
        if not math.isfinite(max_age) or max_age <= 0:
            raise ValueError('Invalid lane freshness setting')
        self.validation_mode, self.max_age = validation_mode, max_age
        self.state = 'OUTSIDE'
        self.selected = self.requested = 'RGB'
        self.last_event_ns = 0
        self.last_input_ns = {'RGB': 0, 'IR': 0}
        self.reason = 'steady'
        self.last_handover = None

    @property
    def compute(self):
        return self.selected

    @property
    def camera(self):
        return self.selected

    def event(self, kind, stamp, now_ns):
        if (kind not in ('entry', 'exit') or stamp <= self.last_event_ns or
                (not self.validation_mode and not 0 <= (now_ns - stamp) / 1e9 <= self.max_age)):
            return False
        self.last_event_ns = stamp
        target = 'IR' if kind == 'entry' else 'RGB'
        if target == self.requested:
            return False
        self.selected = self.requested = target
        self.state = 'INSIDE' if target == 'IR' else 'OUTSIDE'
        self.reason = f'tcn_{kind}'
        self.last_handover = dict(target=target, reason=self.reason, total_ms=0.)
        return True

    def lane(self, source, message, now_ns):
        if source not in self.last_input_ns:
            return None
        stamp = stamp_ns(message)
        fresh = (self.validation_mode and source == 'IR') or (
            0 <= (now_ns - stamp) / 1e9 <= self.max_age)
        if stamp <= self.last_input_ns[source] or not fresh:
            return None
        self.last_input_ns[source] = stamp
        if source != self.selected:
            return None
        # Preserve all geometry and capture/performance timestamps. Source
        # identity lets the controller check ordering within each camera stream.
        message.lane_source = source
        return message
