"""Pure helpers for the remote preview frame (no ROS imports)."""
from collections import deque
import math

STALE_SEC = 0.5
TUNNEL_STALE_SEC = 1.0
TUNNEL_STATES = ('OUTSIDE', 'ENTERING', 'INSIDE', 'EXITING')


def _finite_positive(*values):
    return all(isinstance(v, (int, float)) and math.isfinite(v) and v > 0 for v in values)


def centerline_metric(points, source_width, source_height, padding_left,
                      bev_width_m, bev_height_m, max_points=24):
    """Padded BEV pixels -> [x_forward_m, y_left_m], same formula as the C++ controller."""
    if len(points) < 2 or not _finite_positive(source_width, source_height, bev_width_m, bev_height_m):
        return []
    scale_x, scale_y = bev_width_m / source_width, bev_height_m / source_height
    count = len(points)
    if count > max_points:
        indices = [round(i * (count - 1) / (max_points - 1)) for i in range(max_points)]
    else:
        indices = range(count)
    result = []
    for index in indices:
        u, v = points[index][0], points[index][1]
        if not (math.isfinite(u) and math.isfinite(v)):
            return []
        result.append([round(bev_height_m - (v + 0.5) * scale_y, 3),
                       round(bev_width_m / 2 - (u - padding_left + 0.5) * scale_x, 3)])
    return result


class RateWindow:
    """Events (optionally with values) in a sliding window, for FPS and mean ms."""

    def __init__(self, window_sec=1.0):
        self.window_sec = window_sec
        self.events = deque()

    def _trim(self, now):
        while self.events and now - self.events[0][0] > self.window_sec:
            self.events.popleft()

    def add(self, now, value=None):
        self.events.append((now, value))
        self._trim(now)

    def fps(self, now):
        self._trim(now)
        if len(self.events) < 2:
            return None
        span = self.events[-1][0] - self.events[0][0]
        return (len(self.events) - 1) / span if span > 0 else None

    def mean(self, now):
        self._trim(now)
        values = [v for _, v in self.events if v is not None]
        return sum(values) / len(values) if values else None

    def age_ms(self, now):
        return None if not self.events else (now - self.events[-1][0]) * 1000.0


def tunnel_state(text):
    first = (text or '').split('\n', 1)[0]
    if not first.startswith('Tunnel:'):
        return None
    state = first[len('Tunnel:'):].strip().split(' ')[0].upper()
    return state if state in TUNNEL_STATES else None


def _r(value, digits=3):
    return round(value, digits) if isinstance(value, (int, float)) and math.isfinite(value) else None


def _fresh(entry, now, limit=STALE_SEC):
    """entry is (value, received_at) or None."""
    if not entry or now - entry[1] > limit:
        return None
    return entry[0]


def _age_ms(entries, now):
    stamps = [e[1] for e in entries if e]
    return round((now - max(stamps)) * 1000.0, 1) if stamps else None


VEHICLE_KEYS = ('speed_mps', 'target_speed_mps', 'duty', 'brake_current_a', 'servo', 'steering_rad',
                'raw_steering_rad', 'corner_state', 'cte_m', 'heading_rad', 'curvature',
                'vesc_connected')
FPS_KEYS = ('camera', 'bev', 'lane', 'inference_ms', 'postprocess_ms', 'control_ms', 'traffic', 'tcn')


def build_frame(seq, now, snap):
    vehicle_in, lane_in = snap.get('vehicle', {}), snap.get('lane', {})
    vehicle = {}
    for key in VEHICLE_KEYS:
        value = _fresh(vehicle_in.get(key), now)
        vehicle[key] = value if isinstance(value, bool) or value is None else _r(value)
    vehicle['age_ms'] = _age_ms(vehicle_in.values(), now)
    points = _fresh(lane_in.get('points'), now) or []
    lane = {
        'points': points if len(points) >= 2 else [],
        'valid': bool(_fresh(lane_in.get('valid'), now)),
        'state': _fresh(lane_in.get('state'), now),
        'source': _fresh(lane_in.get('source'), now) or '',
        'stop_line_m': _r(_fresh(lane_in.get('stop_line_m'), now), 2),
        'lane_width_m': snap.get('lane_width_m', 0.65),
        'age_ms': _age_ms([lane_in.get('points')], now),
    }
    fps_in = snap.get('fps', {})
    traffic = snap.get('traffic')
    tunnel = snap.get('tunnel', {})
    return {
        'v': 1, 'seq': seq, 't': round(now, 3),
        'vehicle': vehicle,
        'lane': lane,
        'fps': {key: _r(fps_in.get(key), 2) for key in FPS_KEYS},
        'traffic': {'state': _fresh(traffic, now), 'age_ms': _age_ms([traffic], now)},
        'tunnel': {'state': _fresh(tunnel.get('state'), now, TUNNEL_STALE_SEC),
                   'lines': _fresh(tunnel.get('lines'), now, TUNNEL_STALE_SEC)},
        'system': {key: _r(snap.get('system', {}).get(key), 1)
                   for key in ('cpu_pct', 'gpu_pct', 'cpu_temp_c', 'gpu_temp_c', 'power_w')},
        'startup': snap.get('startup', {'camera': '', 'models': {}, 'controller': '',
                                        'can_start': False, 'started': False, 'message': ''}),
        'link': snap.get('link', {'clients': 0, 'heartbeat_age_ms': None, 'loss_stop_sec': 0.0}),
    }
