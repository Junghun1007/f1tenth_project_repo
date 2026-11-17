"""핀홀 카메라 + 저해상도 래스터 z-버퍼로 콘 관측을 만든다.

- 카메라: 앞차축 기준 x=camera_x, 높이 camera_height, 아래로 pitch. 1280x800, 고정 줌 1.25
- 콘: 바닥 13x13 cm, 위 3x3 cm, 높이 cone_height 의 사각뿔대 → 영상 다각형
- 가까운 콘이 먼 콘을 픽셀 단위로 가린다 (카메라 높이와 콘 높이에 따라 위쪽은 보일 수 있음)
- YOLO 박스 = 보이는 픽셀의 경계 상자, 위치 = multiclass_yolo object_localization 방식
  (박스 가운데 60% 의 깊이 점 → 가장 가까운 충분한 거리 구간 → 시선 방향 뒤로 13 cm 사각형)
  → 가려진 콘의 박스에 앞 콘 픽셀이 많이 섞이면 앞 콘 거리로 잘못 놓이는 실제 오류도 재현
"""
import math
import numpy as np

from .geometry import to_local


class Camera:
    def __init__(self, cp):
        self.cp = cp
        s = cp.raster_scale
        self.W = int(round(cp.image_width * s))
        self.H = int(round(cp.image_height * s))
        half = math.tan(math.radians(cp.hfov_deg) / 2)
        self.f = (self.W / 2) / half * cp.zoom
        self.cx, self.cy = self.W / 2, self.H / 2
        th = math.radians(cp.pitch_deg)
        self.fwd = np.array([math.cos(th), 0.0, -math.sin(th)])
        self.right = np.array([0.0, -1.0, 0.0])
        self.up = np.array([math.sin(th), 0.0, math.cos(th)])
        self.pos = np.array([cp.camera_x, 0.0, cp.camera_height])
        self.roi_top = self.H * (1.0 - cp.roi_height_percent / 100.0)
        uu, vv = np.meshgrid(np.arange(self.W) + 0.5, np.arange(self.H) + 0.5)
        self._uu, self._vv = uu, vv

    # ------------------------------------------------------------------
    def project(self, pts3):
        """차량 좌표 3D 점 (N,3) → (u, v, z_cam). z_cam<=0 이면 카메라 뒤."""
        p = np.asarray(pts3, float) - self.pos
        z = p @ self.fwd
        x = p @ self.right
        y = -(p @ self.up)
        zz = np.where(z > 1e-3, z, 1e-3)
        return self.cx + self.f * x / zz, self.cy + self.f * y / zz, z

    def ground_visible(self, pts2):
        """지면 점 (N,2) 이 영상 안에 들어오는지."""
        pts2 = np.asarray(pts2, float).reshape(-1, 2)
        if not len(pts2):
            return np.zeros(0, bool)
        u, v, z = self.project(np.column_stack([pts2, np.zeros(len(pts2))]))
        return (z > 0.02) & (u >= 0) & (u < self.W) & (v >= 0) & (v < self.H)

    def roi_view(self, owner):
        out = owner.copy()
        out[: int(math.ceil(self.roi_top))] = -1
        return out

    def ray_bearing(self, u, v):
        d = self.fwd + ((u - self.cx) / self.f) * self.right - ((v - self.cy) / self.f) * self.up
        return math.atan2(d[1], d[0])

    # ------------------------------------------------------------------
    def render(self, cones_local, size, height, top):
        """콘들을 래스터화. 반환 (owner(HxW, -1=없음), 콘별 거리, 콘별 전체 다각형 면적 px)."""
        n = len(cones_local)
        owner = np.full((self.H, self.W), -1, dtype=np.int32)
        depth = np.full((self.H, self.W), np.inf)
        ranges = np.full(n, np.inf)
        areas = np.zeros(n)
        self.bboxes = np.zeros((n, 4), dtype=int)
        cam_g = self.pos[:2]
        hb, ht = size / 2, top / 2
        for i, c in enumerate(cones_local):
            dx = max(abs(c[0] - cam_g[0]) - hb, 0.0)
            dy = max(abs(c[1] - cam_g[1]) - hb, 0.0)
            ranges[i] = math.hypot(dx, dy)
            if c[0] - cam_g[0] < 0.02 or ranges[i] > 6.0:
                continue
            corners = []
            for hh, zz in ((hb, 0.0), (ht, height)):
                for sx, sy in ((1, 1), (1, -1), (-1, -1), (-1, 1)):
                    corners.append((c[0] + sx * hh, c[1] + sy * hh, zz))
            u, v, z = self.project(np.array(corners))
            if (z <= 0.02).any():
                continue
            hull = _convex_hull(np.column_stack([u, v]))
            areas[i] = _polygon_area(hull)
            u0, u1 = max(int(math.floor(hull[:, 0].min())), 0), min(int(math.ceil(hull[:, 0].max())), self.W)
            v0, v1 = max(int(math.floor(hull[:, 1].min())), 0), min(int(math.ceil(hull[:, 1].max())), self.H)
            if u1 <= u0 or v1 <= v0:
                continue
            uu = self._uu[v0:v1, u0:u1]
            vv = self._vv[v0:v1, u0:u1]
            inside = np.ones(uu.shape, bool)
            m = len(hull)
            for k in range(m):
                a, b = hull[k], hull[(k + 1) % m]
                inside &= (b[0] - a[0]) * (vv - a[1]) - (b[1] - a[1]) * (uu - a[0]) >= 0
            self.bboxes[i] = (u0, v0, u1, v1)
            sub_d = depth[v0:v1, u0:u1]
            win = inside & (ranges[i] < sub_d)
            sub_d[win] = ranges[i]
            owner[v0:v1, u0:u1][win] = i
        return owner, ranges, areas

    def detect(self, cones_local, rng, noise, pp):
        """콘 관측 목록 [(index, center(2), front(2), bearing, visible_frac, box)] (차량 좌표)."""
        cp = self.cp
        owner_all, ranges, areas = self.render(cones_local, pp.cone_size, cp.cone_height, cp.cone_top)
        # YOLO는 영상 아래 ROI만 본다: ROI 위로 잘린 부분은 박스·보이는 비율에 넣지 않는다
        owner = self.roi_view(owner_all)
        n = len(cones_local)
        counts = np.bincount(owner[owner >= 0].ravel(), minlength=n) if n else np.zeros(0)
        out = []
        # 콘별 깊이 바이어스 (스테레오 오차)
        bias = np.array([rng.normal(0.0, pp.range_noise_base + pp.range_noise_per_m * r) if noise and np.isfinite(r)
                         else 0.0 for r in ranges])
        for i in range(n):
            if counts[i] == 0 or areas[i] <= 0:
                continue
            frac = counts[i] / areas[i]
            bu0, bv0, bu1, bv1 = self.bboxes[i]
            bv0 = max(bv0, int(math.ceil(self.roi_top)))
            vs, us = np.nonzero(owner[bv0:bv1, bu0:bu1] == i)
            u0, u1, v0, v1 = us.min() + bu0, us.max() + 1 + bu0, vs.min() + bv0, vs.max() + 1 + bv0
            box_h_full = (v1 - v0) / cp.raster_scale
            if frac < cp.min_visible_fraction or box_h_full < cp.min_box_px:
                continue
            vc = 0.5 * (v0 + v1)
            if noise and rng.random() > pp.detect_prob:
                continue
            # 박스 가운데 60% 깊이 점 (지면·배경은 필터로 제거됨 → 콘 픽셀만)
            cf = cp.box_core_fraction
            cu0 = int(round(u0 + (u1 - u0) * (1 - cf) / 2)); cu1 = max(cu0 + 1, int(round(u1 - (u1 - u0) * (1 - cf) / 2)))
            cv0 = int(round(v0 + (v1 - v0) * (1 - cf) / 2)); cv1 = max(cv0 + 1, int(round(v1 - (v1 - v0) * (1 - cf) / 2)))
            core = owner[cv0:cv1, cu0:cu1]
            ids = core[core >= 0]
            if len(ids) < cp.min_points:
                ids = owner[v0:v1, u0:u1]
                ids = ids[ids >= 0]
            r = ranges[ids] + bias[ids]
            if noise:
                r = r + rng.normal(0.0, 0.004, len(r))
            rng_est = _nearest_peak(np.sort(r), cp)
            if rng_est is None:
                continue
            uc = 0.5 * (u0 + u1)
            b = self.ray_bearing(uc, vc)
            if noise:
                b += rng.normal(0.0, math.radians(pp.bearing_noise_deg))
            ux, uy = math.cos(b), math.sin(b)
            front = self.pos[:2] + rng_est * np.array([ux, uy])
            center = front + (pp.cone_size / 2) * np.array([ux, uy])
            uniq = np.unique(ids)
            src = int(uniq[np.argmin(np.abs(ranges[uniq] + bias[uniq] - rng_est))])   # 거리를 준 콘
            out.append((i, center, front, b, frac, (u0, v0, u1, v1), src))
        self.last_owner = owner_all
        self.last_ranges = ranges
        self.last_boxes = [(d[0], d[5], d[6]) for d in out]
        return out, owner


def _nearest_peak(r, cp):
    n = len(r)
    need = max(cp.min_points, int(math.ceil(cp.min_point_share * n)))
    if n < need:
        return None
    j = 0
    for i in range(n):
        w = cp.window_base + cp.window_per_m * r[i]
        if j < i:
            j = i
        while j < n and r[j] <= r[i] + w:
            j += 1
        if j - i >= need:
            return float(r[i + (j - i) // 2])
    return None


def _convex_hull(p):
    pts = sorted(map(tuple, p))
    if len(pts) <= 2:
        return np.array(pts)

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for q in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], q) <= 0:
            lower.pop()
        lower.append(q)
    for q in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], q) <= 0:
            upper.pop()
        upper.append(q)
    return np.array(lower[:-1] + upper[:-1])


def _polygon_area(h):
    x, y = h[:, 0], h[:, 1]
    return 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))
