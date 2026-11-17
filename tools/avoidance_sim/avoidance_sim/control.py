"""Stanley 조향과 곡률 기반 속도 계획 (1113ML auto_control control_core 이식·단순화)."""
import math
import numpy as np

from .config import ControlParams


class Path:
    """앞차축 기준 차량 좌표 점열."""

    def __init__(self, points, geometry_window=0.16):
        pts = np.asarray(points, dtype=float)
        if len(pts) >= 2:
            keep = np.concatenate([[True], np.linalg.norm(np.diff(pts, axis=0), axis=1) > 1e-6])
            pts = pts[keep]
        self.points = pts
        self.window = geometry_window
        if len(pts) >= 2:
            self.arc = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))])
        else:
            self.arc = np.zeros(len(pts))

    @classmethod
    def with_arc(cls, points, arc, geometry_window):
        """강체 변환한 같은 경로: 호 길이를 다시 계산하지 않는다 (롤아웃용)."""
        obj = cls.__new__(cls)
        obj.points, obj.arc, obj.window = points, arc, geometry_window
        return obj

    def points_at(self, arcs):
        a = np.asarray(arcs, float)
        return np.stack([np.interp(a, self.arc, self.points[:, 0]), np.interp(a, self.arc, self.points[:, 1])], -1)

    def curvatures(self, arcs):
        """curvature()의 벡터화 버전."""
        a = np.asarray(arcs, float)
        h = self.window / 2
        a0, a1 = np.maximum(0.0, a - h), np.minimum(self.length, a + h)
        p0, p1, p2 = self.points_at(a0), self.points_at(0.5 * (a0 + a1)), self.points_at(a1)
        d1, d2 = p1 - p0, p2 - p1
        cross = d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]
        denom = np.linalg.norm(d1, axis=1) * np.linalg.norm(d2, axis=1) * np.linalg.norm(p2 - p0, axis=1)
        out = np.where(denom < 1e-12, 0.0, 2.0 * cross / np.maximum(denom, 1e-12))
        return np.where(a1 - a0 < 1e-3, 0.0, out)

    @property
    def valid(self):
        return len(self.points) >= 2 and self.arc[-1] > 0.05

    @property
    def length(self):
        return float(self.arc[-1]) if len(self.arc) else 0.0

    def point(self, a):
        return np.array([np.interp(a, self.arc, self.points[:, 0]), np.interp(a, self.arc, self.points[:, 1])])

    def heading(self, a):
        h = self.window / 2
        a0, a1 = max(0.0, a - h), min(self.length, a + h)
        if a1 - a0 < 1e-6:
            a0, a1 = max(0.0, a1 - 0.02), a1
        d = self.point(a1) - self.point(a0)
        return math.atan2(d[1], d[0])

    def curvature(self, a):
        h = self.window / 2
        a0, a1 = max(0.0, a - h), min(self.length, a + h)
        if a1 - a0 < 1e-3:
            return 0.0
        p0, p1, p2 = self.point(a0), self.point(0.5 * (a0 + a1)), self.point(a1)
        d1, d2 = p1 - p0, p2 - p1
        cross = d1[0] * d2[1] - d1[1] * d2[0]
        denom = np.linalg.norm(d1) * np.linalg.norm(d2) * np.linalg.norm(p2 - p0)
        return 0.0 if denom < 1e-12 else 2.0 * cross / denom


def stanley(path: Path, speed, cp: ControlParams, max_steer):
    """auto_control stanley_control과 같은 식. 반환 (조향각, 횡오차, 방향오차)."""
    pts = path.points
    start = pts[:-1]
    delta = pts[1:] - start
    len2 = np.maximum((delta ** 2).sum(1), 1e-12)
    t = np.clip(-(start * delta).sum(1) / len2, 0.0, 1.0)
    closest = start + delta * t[:, None]
    d2 = (closest ** 2).sum(1)
    i = int(np.argmin(d2))
    length = math.sqrt(len2[i])
    cte = (-delta[i, 1] * closest[i, 0] + delta[i, 0] * closest[i, 1]) / length
    arc = path.arc[i] + t[i] * length
    heading_error = path.heading(min(path.length, arc + cp.heading_lookahead))
    correction = math.atan2(cp.stanley_gain * cte, abs(speed) + cp.softening_speed)
    thr = math.radians(cp.corner_heading_threshold_deg)
    if abs(heading_error) >= thr and heading_error * correction < 0:
        lim = abs(heading_error) * min(max(cp.corner_opposing_ratio, 0.0), 0.99)
        correction = max(-lim, min(lim, correction))
    steer = max(-max_steer, min(max_steer, heading_error + correction))
    return steer, cte, heading_error


def plan_speed(path: Path, cp: ControlParams, obstacle_limit, spacing=0.05):
    """곡률 한계 → 한 칸 팽창 → 장애물 상한 → 뒤로 감속 전파. 반환 (arc, 속도 배열)."""
    n = max(1, int(math.ceil(path.length / spacing)))
    arc = np.linspace(0.0, path.length, n + 1)
    kappa = np.abs(path.curvatures(arc))
    with np.errstate(divide="ignore"):
        lim = np.where(kappa > 1e-9, np.sqrt(cp.max_lateral_accel / np.maximum(kappa, 1e-9)), cp.max_speed)
    lim = np.clip(lim, cp.min_speed, cp.max_speed)
    dil = lim.copy()
    dil[1:] = np.minimum(dil[1:], lim[:-1])
    dil[:-1] = np.minimum(dil[:-1], lim[1:])
    v = np.minimum(dil, obstacle_limit)
    for k in range(len(v) - 1, 0, -1):
        ds = arc[k] - arc[k - 1]
        v[k - 1] = min(v[k - 1], math.sqrt(v[k] ** 2 + 2 * cp.planning_decel * ds))
    return arc, v
