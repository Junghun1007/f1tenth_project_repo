"""2D 기하 유틸리티: 좌표 변환, 사각형, SAT 충돌, 볼록 다각형 거리."""
import math
import numpy as np


def rot(psi: float) -> np.ndarray:
    c, s = math.cos(psi), math.sin(psi)
    return np.array([[c, -s], [s, c]])


def to_local(points, origin, psi) -> np.ndarray:
    """월드 점 → (origin, psi) 기준 로컬 좌표."""
    p = np.asarray(points, dtype=float) - np.asarray(origin, dtype=float)
    c, s = math.cos(psi), math.sin(psi)
    return np.stack([c * p[..., 0] + s * p[..., 1], -s * p[..., 0] + c * p[..., 1]], axis=-1)


def to_world(points, origin, psi) -> np.ndarray:
    p = np.asarray(points, dtype=float)
    c, s = math.cos(psi), math.sin(psi)
    return np.stack([c * p[..., 0] - s * p[..., 1], s * p[..., 0] + c * p[..., 1]], axis=-1) + np.asarray(origin)


def rect_corners(cx, cy, psi, length, width) -> np.ndarray:
    hl, hw = length / 2, width / 2
    local = np.array([[hl, hw], [hl, -hw], [-hl, -hw], [-hl, hw]])
    return to_world(local, (cx, cy), psi)


def square_corners(center, size) -> np.ndarray:
    h = size / 2
    x, y = center
    return np.array([[x + h, y + h], [x + h, y - h], [x - h, y - h], [x - h, y + h]])


def _axes(poly):
    edges = np.roll(poly, -1, axis=0) - poly
    normals = np.stack([-edges[:, 1], edges[:, 0]], axis=1)
    return normals / np.linalg.norm(normals, axis=1, keepdims=True)


def sat_gap(a: np.ndarray, b: np.ndarray) -> float:
    """볼록 다각형 a, b의 분리축 최대 간격. 음수면 겹침(침투 깊이 근사)."""
    best = -math.inf
    for axes in (_axes(a), _axes(b)):
        pa = a @ axes.T
        pb = b @ axes.T
        gap = np.maximum(pb.min(0) - pa.max(0), pa.min(0) - pb.max(0))
        best = max(best, float(gap.max()))
    return best


def polygons_overlap(a, b) -> bool:
    return sat_gap(a, b) < 0.0


def _point_segment_distance(p, a, b):
    ab = b - a
    t = np.clip(np.dot(p - a, ab) / max(np.dot(ab, ab), 1e-12), 0.0, 1.0)
    return float(np.linalg.norm(a + t * ab - p))


def polygon_distance(a, b) -> float:
    """볼록 다각형 사이 최소 거리. 겹치면 음수(SAT 침투 깊이)."""
    gap = sat_gap(a, b)
    if gap < 0:
        return gap
    best = math.inf
    for poly_p, poly_e in ((a, b), (b, a)):
        n = len(poly_e)
        for p in poly_p:
            for i in range(n):
                best = min(best, _point_segment_distance(p, poly_e[i], poly_e[(i + 1) % n]))
    return best


def wrap_angle(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def rect_squares_gap(cx, cy, yaw, half_len, half_wid, centers, half_sq):
    """회전 사각형(차체)과 월드 축정렬 정사각형(콘) 여러 개의 SAT 간격 (벡터화). 음수=겹침."""
    c = np.asarray(centers, float).reshape(-1, 2)
    if not len(c):
        return np.zeros(0)
    dx, dy = c[:, 0] - cx, c[:, 1] - cy
    cs, sn = math.cos(yaw), math.sin(yaw)
    ac, as_ = abs(cs), abs(sn)
    # 차체 축 (cs,sn), (-sn,cs): 정사각형 투영 반경 = h(|cs|+|sn|)
    r_sq = half_sq * (ac + as_)
    g1 = np.abs(dx * cs + dy * sn) - half_len - r_sq
    g2 = np.abs(-dx * sn + dy * cs) - half_wid - r_sq
    # 월드 축 x, y: 차체 투영 반경
    g3 = np.abs(dx) - (half_len * ac + half_wid * as_) - half_sq
    g4 = np.abs(dy) - (half_len * as_ + half_wid * ac) - half_sq
    return np.max(np.stack([g1, g2, g3, g4]), axis=0)
