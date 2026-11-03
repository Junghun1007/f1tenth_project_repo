"""Clock-explicit lane handoff policy, independent of ROS and hardware."""
import math


class LaneSelector:
    def __init__(self, max_age=.2, confirmations=3):
        if not math.isfinite(max_age) or max_age <= 0 or confirmations < 1:
            raise ValueError('Invalid lane selection freshness/confirmation settings')
        self.max_age = max_age
        self.confirmations = confirmations
        self.state = 'OUTSIDE'
        self.selected = 'RGB'
        self.requested = 'RGB'
        self.transition_ns = 0
        self.last_event_ns = 0
        self.last_output_ns = 0
        self.last_input_ns = {'RGB': 0, 'IR': 0}
        self.count = 0
        self.last_candidate_ns = 0

    @property
    def compute(self):
        return 'BOTH' if self.state in ('ENTERING', 'EXITING') else self.selected

    @property
    def camera(self):
        # Do not restore outside IR exposure while the controller still uses IR.
        return 'IR' if self.state != 'OUTSIDE' else 'RGB'

    def event(self, kind, stamp_ns, now_ns):
        if (kind not in ('entry', 'exit') or stamp_ns <= self.last_event_ns or
                not 0 <= (now_ns - stamp_ns) / 1e9 <= self.max_age):
            return False
        self.last_event_ns = stamp_ns
        # TCN events alternate. A fast opposite event may cancel an unfinished
        # handoff, so state changes cannot get stuck waiting for a lost entry.
        self.requested = 'IR' if kind == 'entry' else 'RGB'
        if self.requested == self.selected:
            self.state = 'INSIDE' if self.selected == 'IR' else 'OUTSIDE'
        else:
            self.state = 'ENTERING' if kind == 'entry' else 'EXITING'
        # Require captures AFTER the request was received, not low-rate cached
        # results produced before TCN detected the boundary.
        self.transition_ns = now_ns
        self.count = 0
        self.last_candidate_ns = 0
        return True

    def lane(self, source, stamp_ns, now_ns, ready):
        if source not in self.last_input_ns:
            return False
        fresh = 0 <= (now_ns - stamp_ns) / 1e9 <= self.max_age
        if stamp_ns <= self.last_input_ns[source] or not fresh:
            if source == self.requested:
                self.count = 0
            return False
        self.last_input_ns[source] = stamp_ns
        if source == self.requested and source != self.selected:
            if not ready or stamp_ns <= self.transition_ns:
                self.count = 0
            else:
                if stamp_ns - self.last_candidate_ns > self.max_age * 1e9:
                    self.count = 0
                self.count += 1
                self.last_candidate_ns = stamp_ns
                if self.count >= self.confirmations:
                    self.selected = source
                    self.state = 'INSIDE' if source == 'IR' else 'OUTSIDE'
        # No cached republishing: original stamp and strict monotonicity are
        # preserved across cameras. Missing/invalid paths reach controller stops.
        if source != self.selected or stamp_ns <= self.last_output_ns:
            return False
        self.last_output_ns = stamp_ns
        return True


def lane_ready(message, rules):
    """Validate candidate geometry before switching. Controller remains final judge."""
    if (not message.centerline_valid or message.centerline_sample_limit_reached or
            message.state not in (1, 2, 3) or message.header.frame_id != 'front_axle_bev' or
            message.source_width != 120 or message.source_height != 300 or
            not math.isclose(message.centerline_bev_width_m, 1.2, abs_tol=1e-5) or
            not math.isclose(message.centerline_bev_height_m, 3., abs_tol=1e-5)):
        return False
    points, supports = message.centerline_points, message.centerline_support
    if not 2 <= len(points) <= 10000 or len(points) != len(supports):
        return False
    metric = []
    for point, support in zip(points, supports):
        if (support not in (1, 2, 3, 4) or
                not all(math.isfinite(v) for v in (point.x, point.y, point.z)) or
                abs(point.z) > 1e-6 or not 0 <= point.y < 300 or
                not 0 <= point.x < 120 + message.padding_left + message.padding_right):
            return False
        x, y = 3. - (point.y + .5) * .01, .6 - (point.x - message.padding_left + .5) * .01
        metric.append((x, y))
    contiguous = []
    for point in metric:
        if not rules['minimum_x'] <= point[0] <= rules['maximum_x']:
            if contiguous:
                break
            continue
        if contiguous:
            gap = math.dist(point, contiguous[-1])
            if gap < 1e-5:
                continue
            if gap > rules['maximum_gap']:
                break
        contiguous.append(point)
    metric = contiguous
    if len(metric) < rules['minimum_points']:
        return False
    distances = [math.dist(a, b) for a, b in zip(metric, metric[1:])]
    return bool(distances) and max(distances) <= rules['maximum_gap'] and sum(distances) >= rules['minimum_span']
