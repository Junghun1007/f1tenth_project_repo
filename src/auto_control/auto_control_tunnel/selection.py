"""Clock-explicit immediate source selection, independent of ROS/hardware."""
import math


class LaneSelector:
    def __init__(self, max_age=.2, validation_mode=False):
        if not math.isfinite(max_age) or max_age <= 0:
            raise ValueError('Invalid lane selection freshness setting')
        self.validation_mode = validation_mode
        self.max_age = max_age
        self.state = 'OUTSIDE'
        self.selected = self.requested = 'RGB'
        self.last_event_ns = 0
        self.last_output_ns = 0
        self.last_input_ns = {'RGB': 0, 'IR': 0}

    @property
    def compute(self):
        return self.selected

    @property
    def camera(self):
        return self.selected

    def event(self, kind, stamp_ns, now_ns):
        if (kind not in ('entry', 'exit') or stamp_ns <= self.last_event_ns or
                (not self.validation_mode and not 0 <= (now_ns - stamp_ns) / 1e9 <= self.max_age)):
            return False
        self.last_event_ns = stamp_ns
        self.selected = self.requested = 'IR' if kind == 'entry' else 'RGB'
        self.state = 'INSIDE' if kind == 'entry' else 'OUTSIDE'
        return True

    def lane(self, source, stamp_ns, now_ns):
        if source not in self.last_input_ns:
            return False
        fresh = (self.validation_mode and source == 'IR') or (
            0 <= (now_ns - stamp_ns) / 1e9 <= self.max_age)
        if stamp_ns <= self.last_input_ns[source] or not fresh:
            return False
        self.last_input_ns[source] = stamp_ns
        # Route the selected source immediately, without candidate geometry or
        # confirmation gates. The controller consumes the original result.
        if source != self.selected or (
                not self.validation_mode and stamp_ns <= self.last_output_ns):
            return False
        self.last_output_ns = stamp_ns
        return True
