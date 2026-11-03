"""Prepare both lane sources, then blend on new observations only."""
from dataclasses import dataclass
import math
from .blending import PathRules, metric_path, mix_paths, stamp_ns


@dataclass
class Observation:
    message: object
    stamp: int
    received: int
    path: list


class LaneSelector:
    def __init__(self, max_age=.2, validation_mode=False, blend_sec=.15,
                 transition_timeout_sec=.75, contributor_max_age_sec=.12,
                 pair_max_skew_sec=.10, maximum_offset_m=.20,
                 maximum_heading_rad=math.radians(20), rules=None):
        values = (max_age, blend_sec, transition_timeout_sec, contributor_max_age_sec,
                  pair_max_skew_sec, maximum_offset_m, maximum_heading_rad)
        if (not all(math.isfinite(v) and v > 0 for v in values) or
                blend_sec >= transition_timeout_sec or
                maximum_heading_rad >= math.pi / 2):
            raise ValueError('Invalid tunnel handover settings')
        self.validation_mode, self.max_age = validation_mode, max_age
        self.blend_sec, self.transition_timeout_sec = blend_sec, transition_timeout_sec
        self.contributor_max_age_sec = contributor_max_age_sec
        self.pair_max_skew_sec = pair_max_skew_sec
        self.maximum_offset_m, self.maximum_heading_rad = maximum_offset_m, maximum_heading_rad
        self.rules = rules or PathRules()
        self.state = 'OUTSIDE'
        self.selected = self.requested = 'RGB'
        self.last_event_ns = self.last_output_ns = 0
        self.last_input_ns = {'RGB': 0, 'IR': 0}
        self.observations = {'RGB': None, 'IR': None}
        self.started_ns = self.first_target_ns = None
        self.weight = 0.
        self.reason = 'steady'
        self.last_handover = None

    @property
    def transitioning(self):
        return self.started_ns is not None

    @property
    def compute(self):
        return 'BOTH' if self.transitioning else self.selected

    @property
    def camera(self):
        # RGB is a separate sensor. Preserve usable IR through exit blending;
        # restore stereo auto exposure/depth lights only after RGB takes over.
        return 'IR' if self.transitioning else self.selected

    def event(self, kind, stamp, now_ns):
        if (kind not in ('entry', 'exit') or stamp <= self.last_event_ns or
                (not self.validation_mode and not 0 <= (now_ns - stamp) / 1e9 <= self.max_age)):
            return False
        self.last_event_ns = stamp
        target = 'IR' if kind == 'entry' else 'RGB'
        if target == self.requested:
            return False
        self.requested = target
        if target == self.selected:
            self._finish(now_ns, 'reversed_request')
            return True
        self.started_ns, self.first_target_ns = now_ns, None
        self.observations[target] = None  # No previous tunnel visit/cached standby result.
        self.weight, self.reason = 0., 'waiting_target_path'
        self.state = 'ENTERING' if target == 'IR' else 'EXITING'
        return True

    def _finish(self, now_ns, reason):
        self.last_handover = dict(
            target=self.requested, reason=reason,
            total_ms=(now_ns - self.started_ns) / 1e6 if self.started_ns is not None else 0.,
            first_path_ms=(self.first_target_ns - self.started_ns) / 1e6
            if self.first_target_ns is not None and self.started_ns is not None else None)
        self.selected = self.requested
        self.state = 'INSIDE' if self.selected == 'IR' else 'OUTSIDE'
        self.started_ns = self.first_target_ns = None
        self.weight, self.reason = 1., reason
        self.observations = {'RGB': None, 'IR': None}

    def tick(self, now_ns):
        # Deadline changes status/compute only. Never republish a saved path.
        if self.transitioning and (now_ns < self.started_ns or
                (now_ns - self.started_ns) / 1e9 >= self.transition_timeout_sec):
            self._finish(now_ns, 'target_wait_timeout')

    def _fresh(self, source, now_ns):
        observation = self.observations[source]
        if observation is None:
            return None
        age = min(self.max_age, self.contributor_max_age_sec)
        if not (0 <= now_ns - observation.received <= age * 1e9 and
                0 <= now_ns - observation.stamp <= age * 1e9):
            return None
        return observation

    def _emit(self, message):
        # Include cross-source ordering even in validation mode. Do not rewrite
        # timestamps or refresh freshness by replaying cached observations.
        stamp = stamp_ns(message)
        if stamp <= self.last_output_ns:
            return None
        self.last_output_ns = stamp
        return message

    def lane(self, source, message, now_ns):
        self.tick(now_ns)
        if source not in self.last_input_ns:
            return None
        stamp = stamp_ns(message)
        fresh = (self.validation_mode and source == 'IR' and not self.transitioning) or (
            0 <= (now_ns - stamp) / 1e9 <= self.max_age)
        if stamp <= self.last_input_ns[source] or not fresh:
            return None
        self.last_input_ns[source] = stamp
        if not self.transitioning:
            if source != self.selected:
                return None
            path = metric_path(message, self.rules)
            self.observations[source] = Observation(message, stamp, now_ns, path) if path else None
            return self._emit(message)

        path = metric_path(message, self.rules)
        self.observations[source] = Observation(message, stamp, now_ns, path) if path else None
        old, target = self._fresh(self.selected, now_ns), self._fresh(self.requested, now_ns)
        if target is None:
            self.reason = 'waiting_target_path'
            if source == self.selected and old is not None:
                return self._emit(message)
            # Both invalid: deliver the new invalid result to the controller.
            return self._emit(message) if old is None and path is None else None
        if self.first_target_ns is None:
            self.first_target_ns = now_ns
        if old is None:
            if source == self.requested:
                self._finish(now_ns, 'target_only')
                return self._emit(message)
            return None
        self.weight = min(1., (now_ns - self.first_target_ns) / (self.blend_sec * 1e9))
        if self.weight >= 1.:
            if source == self.requested:
                self._finish(now_ns, 'blend_complete')
                return self._emit(message)
            return None
        if abs(old.stamp - target.stamp) / 1e9 > self.pair_max_skew_sec:
            result, reason = None, 'timestamp_skew'
        else:
            result, reason = mix_paths(old.message, target.message, old.path, target.path,
                self.weight, self.rules, self.maximum_offset_m, self.maximum_heading_rad)
        self.reason = reason
        if result is not None:
            return self._emit(result)
        # Large disagreement/non-overlap is not evidence for a third path.
        # Hand over only on a new valid target observation; never replay it.
        if source == self.requested:
            self._finish(now_ns, reason)
            return self._emit(message)
        return self._emit(message) if source == self.selected and old is not None else None
