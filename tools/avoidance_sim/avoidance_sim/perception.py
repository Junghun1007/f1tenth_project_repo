"""인지 모델 (설계 문서 7장). 출력은 앞차축 기준 차량 좌표 (x 앞, y 왼쪽)."""
from dataclasses import dataclass, field
import math
import numpy as np

from .config import PerceptionParams
from .geometry import to_local, square_corners
from .track import Track, LANE_HALF_TAPE


@dataclass
class LaneObs:
    center: np.ndarray            # (N,2) 차선 중심선 추정 (차선 경계 판단용)
    left_tape: np.ndarray
    right_tape: np.ndarray
    path: np.ndarray = None       # line_detactor 출력 경로 (코너 바깥쪽 이동 포함). 없으면 center

    def __post_init__(self):
        if self.path is None:
            self.path = self.center


@dataclass
class ConeDetection:
    center: np.ndarray            # (2,) 차량 좌표, 13x13 사각형 중심
    front: np.ndarray             # 측정된 앞면
    bearing: float
    visible_fraction: float
    truth_index: int              # 디버그용 (플래너는 사용 금지)
    depth_source: int = -1        # 디버그용: 깊이를 준 콘 (다르면 가림에 의한 위치 오류)


@dataclass
class PerceptionFrame:
    capture_time: float
    available_time: float
    lane: LaneObs
    cones: list
    odom_pose: tuple              # 촬영 시점 오도메트리 앞차축 자세 (x, y, psi)
    true_pose: tuple              # 디버그용
    speed: float = 0.0            # 촬영 시점 측정 속도
    steer: float = 0.0            # 촬영 시점 조향 명령


def _in_bev(pts, p: PerceptionParams):
    return (pts[:, 0] >= 0.0) & (pts[:, 0] <= p.bev_x_max) & (np.abs(pts[:, 1]) <= p.bev_y_max)


def corner_confidence(track: Track, ss, s_car, p: PerceptionParams):
    """line_detactor 코너 바깥쪽 이동 가중치 (0~1). 보이는 코너와 그 진입 구간에만 적용."""
    ss = np.asarray(ss, float)
    conf = np.zeros(len(ss))
    L = track.length
    for c0, c1 in track.corner_ranges():
        for k in (-1, 0, 1):
            a, b = c0 + k * L, c1 + k * L
            if a - s_car > p.bev_x_max or b < s_car - 1.0:
                continue                      # 아직 안 보이는 코너 / 지나간 코너
            inside = (ss >= a) & (ss <= b)
            conf[inside] = 1.0
            d = a - ss
            full, entry = p.corner_entry_full_offset_distance, p.corner_entry_distance
            ramp = (d > 0) & (d < entry)
            if entry > 0:
                x = np.clip((entry - d[ramp]) / max(entry - full, 1e-6), 0.0, 1.0)
                conf[ramp] = np.maximum(conf[ramp], x * x * (3 - 2 * x))
            # 코너 직후: 코너 판별 창(약 0.4 m) 동안 서서히 해제
            d2 = ss - b
            ex = (d2 > 0) & (d2 < 0.4)
            y_ = 1.0 - d2[ex] / 0.4
            conf[ex] = np.maximum(conf[ex], y_ * y_ * (3 - 2 * y_))
    return conf


def observe_lane(track: Track, front_pose, s_hint, p: PerceptionParams, rng, camera=None) -> LaneObs:
    x, y, psi = front_pose
    ss = np.arange(s_hint - 0.6, s_hint + p.bev_x_max + 1.5, p.lane_spacing)
    noise = p.lane_noise if p.noise else 0.0
    bias = rng.normal(0.0, noise)
    out = {}
    visible = {}
    offset = -p.corner_outward_offset * corner_confidence(track, ss, s_hint, p)   # 좌회전 → 오른쪽(바깥)
    for name, l in (("left", LANE_HALF_TAPE), ("right", -LANE_HALF_TAPE), ("center", 0.0), ("path", offset)):
        world = track.points_vec(ss, l)
        local = to_local(world, (x, y), psi)
        local[:, 1] += bias + rng.normal(0.0, noise * 0.3, len(ss))
        out[name] = local
        vis = _in_bev(local, p)
        if camera is not None and p.camera_model and p.lane_camera_frustum:
            vis &= camera.ground_visible(local)
        visible[name] = vis
    any_tape = visible["left"] | visible["right"]
    # 중심선은 한쪽 테이프만 보여도 추정 가능. 카메라에 안 잡히는 차량 바로 앞은 없음
    near_ok = out["center"][:, 0] >= (_min_visible_x(camera, p) if camera is not None and p.lane_camera_frustum else 0.0)
    keep = any_tape & near_ok
    return LaneObs(center=out["center"][keep],
                   left_tape=out["left"][visible["left"]],
                   right_tape=out["right"][visible["right"]],
                   path=out["path"][keep])


def _min_visible_x(camera, p):
    if not p.camera_model:
        return 0.0
    for xq in np.arange(0.0, 1.0, 0.01):
        if camera.ground_visible(np.array([[xq, 0.0]]))[0]:
            return float(xq)
    return 0.0


def _interval_subtract(interval, covers):
    """[a,b]에서 covers 구간들을 뺀 나머지 목록."""
    parts = [interval]
    for c0, c1 in covers:
        nxt = []
        for a, b in parts:
            if c1 <= a or c0 >= b:
                nxt.append((a, b))
                continue
            if c0 > a:
                nxt.append((a, c0))
            if c1 < b:
                nxt.append((c1, b))
        parts = nxt
    return parts


def visible_cones(cone_centers_world, front_pose, p: PerceptionParams, cone_size=0.13):
    """가림 계산. 반환: [(index, near_range, (a0,a1) 보이는 가장 큰 구간, 보이는 비율)]"""
    x, y, psi = front_pose
    cam = np.array([x + p.camera_x * math.cos(psi), y + p.camera_x * math.sin(psi)])
    items = []
    for i, c in enumerate(cone_centers_world):
        corners = to_local(square_corners(c, cone_size), cam, psi)
        if corners[:, 0].max() <= 0.02:
            continue
        ang = np.arctan2(corners[:, 1], corners[:, 0])
        a0, a1 = float(ang.min()), float(ang.max())
        # 정사각형까지 최소 거리 (앞면)
        local_c = to_local(np.asarray(c)[None], cam, psi)[0]
        h = cone_size / 2
        dx = max(abs(local_c[0]) - h, 0.0)
        dy = max(abs(local_c[1]) - h, 0.0)
        items.append((math.hypot(dx, dy), i, a0, a1))
    items.sort()
    covers, result = [], []
    for near, i, a0, a1 in items:
        parts = _interval_subtract((a0, a1), covers)
        vis = sum(b - a for a, b in parts)
        frac = vis / max(a1 - a0, 1e-9)
        if parts:
            big = max(parts, key=lambda ab: ab[1] - ab[0])
            result.append((i, near, big, frac))
        covers.append((a0, a1))
    return result


def observe_cones(cone_centers_world, front_pose, p: PerceptionParams, rng, cone_size=0.13, camera=None):
    if camera is not None and p.camera_model:
        if not len(cone_centers_world):
            return []
        local = to_local(np.asarray(cone_centers_world), front_pose[:2], front_pose[2])
        dets, _ = camera.detect(local, rng, p.noise, p)
        out = []
        for i, center, front, b, frac, box, src in dets:
            if not (0.0 <= front[0] <= p.bev_x_max and abs(front[1]) <= p.bev_y_max):
                continue
            out.append(ConeDetection(center=center, front=front, bearing=b, visible_fraction=frac,
                                     truth_index=i, depth_source=src))
        return out
    return _observe_cones_simple(cone_centers_world, front_pose, p, rng, cone_size)


def expected_visible(cone_centers_world, front_pose, p: PerceptionParams, camera=None, threshold=0.8):
    """기억 콘 중 지금 보여야 하는 것의 인덱스 (자체 가림 모델)."""
    if not len(cone_centers_world):
        return set()
    if camera is not None and p.camera_model:
        local = to_local(np.asarray(cone_centers_world), front_pose[:2], front_pose[2])
        owner, ranges, areas = camera.render(local, 0.13, camera.cp.cone_height, camera.cp.cone_top)
        owner = camera.roi_view(owner)
        counts = np.bincount(owner[owner >= 0].ravel(), minlength=len(local))
        return {i for i in range(len(local)) if areas[i] > 0 and counts[i] / areas[i] >= threshold}
    return {i for i, near, ab, frac in visible_cones(cone_centers_world, front_pose, p) if frac >= threshold}


def _observe_cones_simple(cone_centers_world, front_pose, p: PerceptionParams, rng, cone_size=0.13):
    """예전 단순 모델: 바닥 사각형의 각도 구간으로만 가림 판단 (높이 무시, 무한히 높은 콘)."""
    dets = []
    noisy = p.noise
    for i, near, (a0, a1), frac in visible_cones(cone_centers_world, front_pose, p, cone_size):
        if frac < p.min_visible_fraction:
            continue
        if noisy and rng.random() > p.detect_prob:
            continue
        bearing = 0.5 * (a0 + a1)
        cam = np.array(front_pose[:2]) + p.camera_x * np.array([math.cos(front_pose[2]), math.sin(front_pose[2])])
        rng_true = _ray_square_entry(front_pose[2] + bearing, np.asarray(cone_centers_world[i]) - cam, cone_size)
        r = near if rng_true is None else rng_true
        b = bearing
        if noisy:
            r += rng.normal(0.0, p.range_noise_base + p.range_noise_per_m * r)
            b += rng.normal(0.0, math.radians(p.bearing_noise_deg))
        u = np.array([math.cos(b), math.sin(b)])
        front = np.array([p.camera_x, 0.0]) + r * u
        if not (0.0 <= front[0] <= p.bev_x_max and abs(front[1]) <= p.bev_y_max):
            continue
        dets.append(ConeDetection(center=front + (cone_size / 2) * u, front=front, bearing=b,
                                  visible_fraction=frac, truth_index=i))
    return dets


def _ray_square_entry(bearing, center, size):
    """원점에서 (월드) bearing 방향 광선이 원점 기준 축정렬 정사각형에 들어가는 거리. 없으면 None."""
    d = np.array([math.cos(bearing), math.sin(bearing)])
    h = size / 2
    lo, hi = center - h, center + h
    tmin, tmax = -math.inf, math.inf
    for k in range(2):
        if abs(d[k]) < 1e-12:
            if not (lo[k] <= 0 <= hi[k]):
                return None
            continue
        t0, t1 = (lo[k]) / d[k], (hi[k]) / d[k]
        t0, t1 = min(t0, t1), max(t0, t1)
        tmin, tmax = max(tmin, t0), min(tmax, t1)
    if tmax < max(tmin, 0.0):
        return None
    return max(tmin, 0.0)


class Odometry:
    """속도·요레이트 적분 (설계 7.4). 앞차축 자세를 odom 좌표로 추정."""

    def __init__(self, p: PerceptionParams, wheelbase, x, y, psi, rng):
        self.p = p
        self.L = wheelbase
        self.rng = rng
        self.rx, self.ry, self.psi = x - wheelbase * math.cos(psi), y - wheelbase * math.sin(psi), psi
        self.bias = math.radians(p.gyro_bias_deg_s) if p.noise else 0.0

    def update(self, v_true, yaw_rate_true, dt):
        if self.p.noise:
            v = v_true * (1.0 + self.rng.normal(0.0, self.p.odom_speed_noise))
            w = yaw_rate_true + self.bias + self.rng.normal(0.0, math.radians(self.p.gyro_noise_deg_s))
        else:
            v, w = v_true, yaw_rate_true
        self.rx += v * math.cos(self.psi) * dt
        self.ry += v * math.sin(self.psi) * dt
        self.psi += w * dt

    def front_pose(self):
        return (self.rx + self.L * math.cos(self.psi), self.ry + self.L * math.sin(self.psi), self.psi)
