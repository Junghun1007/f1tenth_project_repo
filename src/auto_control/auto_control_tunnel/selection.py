"""Select the requested lane source immediately on an accepted tunnel event."""
import math
from .path_validation import PathRules, stamp_ns
from .ir_length_exit import IrLengthExit


class LaneSelector:
    def __init__(self, max_age=.2, validation_mode=False, rules=None, ir_length_exit=None):
        if not math.isfinite(max_age) or max_age <= 0:
            raise ValueError('Invalid lane freshness setting')
        self.validation_mode, self.max_age = validation_mode, max_age
        self.rules = rules or PathRules()
        self.state = 'OUTSIDE'
        self.selected = self.requested = 'RGB'
        self.last_event_ns = 0
        self.last_input_ns = {'RGB': 0, 'IR': 0}
        self.reason = 'steady'
        self.last_handover = None
        self.ir_length_exit = ir_length_exit or IrLengthExit()
        self.ir_exit_latched_at = None

    @property
    def compute(self):
        return self.selected

    @property
    def camera(self):
        return self.selected

    def observe_tcn_state(self, state, stamp, now_ns):
        # Retained TCN inside reports must not undo a length-triggered exit.
        # Require a fresh post-trigger outside acknowledgement before re-entry.
        if self.ir_exit_latched_at is None:
            return True
        if (state == 'outside' and stamp > self.ir_exit_latched_at and
                0 <= (now_ns - stamp) / 1e9 <= self.max_age):
            self.ir_exit_latched_at = None
            return True
        return False

    def event(self, kind, stamp, now_ns, by_length=False):
        if (kind not in ('entry', 'exit') or stamp <= self.last_event_ns or
                (not self.validation_mode and not 0 <= (now_ns - stamp) / 1e9 <= self.max_age)):
            return False
        if kind == 'entry' and self.ir_exit_latched_at is not None:
            return False
        self.last_event_ns = stamp
        target = 'IR' if kind == 'entry' else 'RGB'
        if target == self.requested:
            return False
        if target == 'IR':
            self.ir_length_exit.reset()
        self.selected = self.requested = target
        self.state = 'INSIDE' if target == 'IR' else 'OUTSIDE'
        self.reason = 'ir_length_drop' if by_length else f'tcn_{kind}'
        self.last_handover = dict(target=target, reason=self.reason, total_ms=0.)
        if by_length:
            self.last_handover['trigger'] = self.reason
        return True

    def lane(self, source, message, now_ns):
        if source not in self.last_input_ns:
            return None
        stamp = stamp_ns(message)
        fresh = (self.validation_mode and source == 'IR') or (
            0 <= (now_ns - stamp) / 1e9 <= self.max_age)
        if source == 'IR' and self.state == 'INSIDE':
            if self.ir_length_exit.observe(message, now_ns, self.max_age, self.rules):
                if self.event('exit', now_ns, now_ns, by_length=True):
                    self.ir_exit_latched_at = now_ns
        if stamp <= self.last_input_ns[source] or not fresh:
            return None
        self.last_input_ns[source] = stamp
        if source != self.selected:
            return None
        # Preserve all geometry and capture/performance timestamps. Source
        # identity lets the controller check ordering within each camera stream.
        message.lane_source = source
        return message
