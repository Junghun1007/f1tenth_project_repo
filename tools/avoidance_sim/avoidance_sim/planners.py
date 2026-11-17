"""회피 플래너 (설계 문서 9장).

공통 인터페이스
    planner.update(frame: PerceptionFrame) -> None       # 새 인지 결과 도착 시
    planner.path_for_control(odom_front_pose) -> (Path|None, obstacle_active)
    planner.debug -> dict                                  # 화면용 (차량 좌표, 촬영 시점 기준)
"""
from dataclasses import dataclass
import math
import numpy as np

from .config import SimParams
from .control import Path, stanley
from .geometry import to_local, to_world, square_corners, sat_gap, rect_corners, rect_squares_gap
from .perception import expected_visible
from .camera import Camera


# ----------------------------------------------------------------------
# 기준선 유틸

def _frenet_frame(center: np.ndarray):
    """중심선 점열 → (arc, 점, 단위접선, 단위법선(왼쪽))."""
    seg = np.diff(center, axis=0)
    seglen = np.linalg.norm(seg, axis=1)
    arc = np.concatenate([[0.0], np.cumsum(seglen)])
    tang = np.vstack([seg, seg[-1:]])
    tang = tang / np.maximum(np.linalg.norm(tang, axis=1, keepdims=True), 1e-12)
    # 부드러운 접선 (앞뒤 평균)
    sm = tang.copy()
    sm[1:-1] = tang[:-2] + tang[1:-1]
    sm = sm / np.maximum(np.linalg.norm(sm, axis=1, keepdims=True), 1e-12)
    norm = np.stack([-sm[:, 1], sm[:, 0]], axis=1)
    return arc, center, sm, norm


def _project(points, arc, center, tang, norm):
    """점 → (s, l) (가장 가까운 기준선 점 기준 선형 근사)."""
    pts = np.atleast_2d(points)
    out = np.zeros((len(pts), 2))
    for k, p in enumerate(pts):
        d = center - p
        i = int(np.argmin((d ** 2).sum(1)))
        v = p - center[i]
        out[k, 0] = arc[i] + float(v @ tang[i])
        out[k, 1] = float(v @ norm[i])
    return out


def _from_frenet(s, l, arc, center, tang, norm):
    s = np.asarray(s, dtype=float)
    l = np.asarray(l, dtype=float)
    # 기준선 구간 밖은 끝 접선으로 직선 연장
    x = np.interp(s, arc, center[:, 0])
    y = np.interp(s, arc, center[:, 1])
    tx = np.interp(s, arc, tang[:, 0])
    ty = np.interp(s, arc, tang[:, 1])
    nrm = np.hypot(tx, ty)
    tx, ty = tx / nrm, ty / nrm
    before = s < arc[0]
    after = s > arc[-1]
    x = np.where(before, center[0, 0] + (s - arc[0]) * tang[0, 0], x)
    y = np.where(before, center[0, 1] + (s - arc[0]) * tang[0, 1], y)
    x = np.where(after, center[-1, 0] + (s - arc[-1]) * tang[-1, 0], x)
    y = np.where(after, center[-1, 1] + (s - arc[-1]) * tang[-1, 1], y)
    tx = np.where(before, tang[0, 0], np.where(after, tang[-1, 0], tx))
    ty = np.where(before, tang[0, 1], np.where(after, tang[-1, 1], ty))
    return np.stack([x - l * ty, y + l * tx], axis=-1)


def _extend_backward(center: np.ndarray, length: float, spacing=0.05):
    """기준선 앞쪽 끝을 첫 구간 방향으로 직선 연장 (앞차축 뒤 영역용)."""
    if len(center) < 2:
        return center
    d = center[1] - center[0]
    d = d / max(np.linalg.norm(d), 1e-12)
    n = int(length / spacing)
    back = center[0] - d * (np.arange(n, 0, -1)[:, None] * spacing)
    return np.vstack([back, center])


def _smooth_center(center: np.ndarray):
    if len(center) < 5:
        return center
    k = np.ones(5) / 5
    pad = np.pad(center, ((2, 2), (0, 0)), mode="edge")
    return np.stack([np.convolve(pad[:, 0], k, "valid"), np.convolve(pad[:, 1], k, "valid")], axis=1)


# ----------------------------------------------------------------------

class _PlannerBase:
    def __init__(self, params: SimParams):
        self.P = params
        self.path_odom = None          # odom 좌표 경로 (제안안)
        self.path_vehicle = None       # 차량 좌표 경로 (기준안)
        self.path_time = -1.0
        self.active = False
        self.debug = {}


class BaselinePlanner(_PlannerBase):
    """현재 실차 방식: 메모리 없음, 보이는 콘만으로 중앙선 변형."""

    name = "baseline"

    def update(self, frame):
        bp, vp = self.P.baseline, self.P.vehicle
        lane = frame.lane
        if len(lane.path) < 4:
            return
        center = _smooth_center(lane.path)      # 실차처럼 line_detactor 경로(코너 이동 포함)를 변형
        arc, c, t, n = _frenet_frame(center)
        cones = [d.center for d in frame.cones]
        offsets = np.zeros(len(arc))
        in_lane = []
        if cones:
            sl = _project(np.array(cones), arc, c, t, n)
            for s_c, l_c in sl:
                if abs(l_c) >= 0.34 or s_c < -0.2 or s_c > arc[-1] + 0.2:
                    continue
                in_lane.append((s_c, l_c))
                need = vp.width / 2 + 0.065 + bp.safety_margin - abs(l_c)
                if need <= 0:
                    continue
                side = -1.0 if l_c >= 0 else 1.0          # 정확히 중앙이면 오른쪽
                mag = min(need, bp.max_offset, 0.325 - vp.width / 2 + 0.02)
                prof = _plateau(arc, s_c - bp.hold_before, s_c + bp.hold_after, bp.ramp)
                cand = side * mag * prof
                offsets = np.where(np.abs(cand) > np.abs(offsets), cand, offsets)
        path = c + offsets[:, None] * n
        self.path_vehicle = path
        self.path_time = frame.capture_time
        self.active = bool(in_lane)
        self.debug = {"reference": c, "path": path, "used_cones": np.array(cones) if cones else np.zeros((0, 2)),
                      "memory": np.zeros((0, 2)), "status": f"cones in lane={len(in_lane)}"}

    def path_for_control(self, odom_front_pose, now):
        if self.path_vehicle is None or now - self.path_time > self.P.control.path_hold_sec + 0.2:
            return None, self.active
        return Path(self.path_vehicle, self.P.control.geometry_window), self.active


def _plateau(arc, a, b, ramp):
    """[a,b] 구간 1, 앞뒤 ramp 길이 동안 코사인으로 0까지."""
    y = np.zeros_like(arc)
    y[(arc >= a) & (arc <= b)] = 1.0
    up = (arc > a - ramp) & (arc < a)
    y[up] = 0.5 - 0.5 * np.cos(np.pi * (arc[up] - (a - ramp)) / ramp)
    dn = (arc > b) & (arc < b + ramp)
    y[dn] = 0.5 + 0.5 * np.cos(np.pi * (arc[dn] - b) / ramp)
    return y


# ----------------------------------------------------------------------

@dataclass
class Track_:
    pos: np.ndarray
    hits: int
    misses: int
    last_seen: float
    weight: float = 1.0           # 누적 정보량 (1/σ² 합)

    @property
    def confirmed(self):
        return self.hits >= 3


class ConeMemory:
    """최근접 이웃 추적 (설계 9.2-2). 위치는 odom 좌표."""

    def __init__(self, params: SimParams):
        self.P = params
        self.tracks: list[Track_] = []
        self.camera = Camera(params.camera)

    def update(self, frame):
        pp = self.P.proposed
        ox, oy, opsi = frame.odom_pose
        pc = self.P.perception
        cam = np.array([self.P.camera.camera_x, 0.0])
        used = set()
        for d in frame.cones:
            o = to_world(d.center[None], (ox, oy), opsi)[0]
            # 스테레오 오차는 시선(거리) 방향으로 크다 → 시선 방향·옆 방향을 따로 본 타원 게이트
            ray = d.center - cam
            r = float(np.linalg.norm(ray))
            u_local = ray / max(r, 1e-6)
            c, s_ = math.cos(opsi), math.sin(opsi)
            u = np.array([c * u_local[0] - s_ * u_local[1], s_ * u_local[0] + c * u_local[1]])
            sig_r = pc.range_noise_base + pc.range_noise_per_m * r
            gate_r = min(pp.gate_radial_max, pp.gate + 2.5 * sig_r)
            gate_l = pp.gate_lateral
            best, bscore = None, 1.0
            for k, tr in enumerate(self.tracks):
                if k in used:
                    continue
                dv = tr.pos - o
                along = float(dv @ u)
                across = float(dv[0] * -u[1] + dv[1] * u[0])
                score = (along / gate_r) ** 2 + (across / gate_l) ** 2
                if score < bscore:
                    best, bscore = k, score
            w = 1.0 / (sig_r ** 2)
            if best is None:
                self.tracks.append(Track_(o.copy(), 1, 0, frame.capture_time, w))
                used.add(len(self.tracks) - 1)
            else:
                tr = self.tracks[best]
                tr.pos = (tr.pos * tr.weight + o * w) / (tr.weight + w)   # 가까이서 본 관측일수록 크게 반영
                tr.weight = min(tr.weight + w, 50.0 * w)                     # 너무 굳지 않게 상한
                tr.hits += 1
                tr.misses = 0
                tr.last_seen = frame.capture_time
                used.add(best)
        self._merge_duplicates()
        # 보여야 하는데 안 보인 콘만 감점 (자체 가림 모델)
        if self.tracks:
            centers = np.array([t.pos for t in self.tracks])
            for i in expected_visible(centers, frame.odom_pose, self.P.perception, self.camera):
                if i in used:
                    continue
                c = to_local(centers[i][None], (ox, oy), opsi)[0]
                pc = self.P.perception
                if 0.15 <= c[0] <= pc.bev_x_max - 0.1 and abs(c[1]) <= pc.bev_y_max - 0.05:
                    self.tracks[i].misses += 1
        behind = -(self.P.vehicle.front_axle_to_rear_bumper + pp.delete_behind)
        keep = []
        for t in self.tracks:
            c = to_local(t.pos[None], (ox, oy), opsi)[0]
            if c[0] < behind:
                continue
            if t.misses >= pp.miss_delete:
                continue
            if not t.confirmed and frame.capture_time - t.last_seen > 0.5:
                continue
            keep.append(t)
        self.tracks = keep

    def _merge_duplicates(self):
        """실제 콘은 중심 간격 13 cm 미만으로 겹칠 수 없다 → 그보다 가까운 두 기억은 하나로."""
        lim = self.P.proposed.merge_distance
        merged = True
        while merged:
            merged = False
            for a in range(len(self.tracks)):
                for b in range(a + 1, len(self.tracks)):
                    ta, tb = self.tracks[a], self.tracks[b]
                    if np.linalg.norm(ta.pos - tb.pos) < lim:
                        tot = ta.weight + tb.weight
                        ta.pos = (ta.pos * ta.weight + tb.pos * tb.weight) / tot
                        ta.weight = tot
                        ta.hits += tb.hits
                        ta.misses = min(ta.misses, tb.misses)
                        ta.last_seen = max(ta.last_seen, tb.last_seen)
                        del self.tracks[b]
                        merged = True
                        break
                if merged:
                    break

    def confirmed_positions(self):
        return np.array([t.pos for t in self.tracks if t.hits >= self.P.proposed.confirm_hits]).reshape(-1, 2)


class ProposedPlanner(_PlannerBase):
    """오도메트리 + 콘 메모리 + Frenet DP + 롤아웃 충돌검사."""

    name = "proposed"

    def __init__(self, params: SimParams):
        super().__init__(params)
        self.memory = ConeMemory(params)
        self.last_dp_ms = 0.0
        self._l_des = (np.array([0.0, 1.0]), np.array([0.0, 0.0]))   # 목표 오프셋 (기본 0)

    # 기억 방식 hook: 기본(proposed)은 월드 좌표 기억 (ConeMemory)
    def _observe(self, frame):
        self.memory.update(frame)

    def _memory_world(self, frame, arc, c, t, n):
        return self.memory.confirmed_positions()

    def update(self, frame):
        import time
        t0 = time.perf_counter()
        self._observe(frame)
        lane = frame.lane
        if len(lane.center) < 4:
            return
        pp, vp = self.P.proposed, self.P.vehicle
        ox, oy, opsi = frame.odom_pose
        center = _extend_backward(_smooth_center(lane.center), pp.backward_extension)
        arc, c, t, n = _frenet_frame(center)
        # 앞차축(원점)의 s를 0으로 맞춘다
        s0, l0 = _project(np.zeros((1, 2)), arc, c, t, n)[0]
        arc = arc - s0
        mem_world = self._memory_world(frame, arc, c, t, n)
        mem_local = to_local(mem_world, (ox, oy), opsi) if len(mem_world) else np.zeros((0, 2))
        cones_sl = _project(mem_local, arc, c, t, n) if len(mem_local) else np.zeros((0, 2))
        # 차체 뒷부분 위치(현재 자세)를 Frenet로
        rear_axle_sl = _project(np.array([[-vp.wheelbase, 0.0]]), arc, c, t, n)[0]
        s_end = min(pp.s_max, arc[-1])
        # line_detactor 경로(코너 바깥쪽 이동)를 기준선 대비 목표 오프셋으로
        drive = _smooth_center(lane.path)
        dsl = _project(drive, arc, c, t, n)
        order = np.argsort(dsl[:, 0])
        self._l_des = (dsl[order, 0], dsl[order, 1])
        # 1) 차선 안에서만 탐색 (지그재그 콘) → 2) 막혔으면 차선 이탈 허용 (유도선)
        result, mode = None, "none"
        near_mask = ((cones_sl[:, 0] > -0.6) & (cones_sl[:, 0] < s_end + 0.3)) if len(cones_sl) else np.zeros(0, bool)
        if not near_mask.any():
            # 주변에 기억 콘이 없으면 차선 중심 추종 (기준안과 동일)
            result, mode = (drive[drive[:, 0] >= -0.05], True, None), "center"
        # 차선 안(기본 여유) → 차선 안(여유 축소) → 차선 밖. 차선 밖은 차가 지나갈 틈이 아예 없을 때만
        tiers = ((True, 0.0), (True, -1.0), (False, 0.0)) if result is None else ()
        # 롤아웃 속도: 현재 속도가 아니라 곧 도달할 속도(장애물 구간 상한)까지 가정
        v_roll = max(frame.speed, self.P.control.obstacle_speed)
        for lane_hard, relax in tiers:
            found = None
            for extra in (0.0, pp.rollout_retry_margin):
                ls = self._dp(l0, rear_axle_sl, cones_sl, s_end, extra, lane_hard, relaxed=relax < 0)
                if ls is None:
                    break
                path = _from_frenet(ls[0], ls[1], arc, c, t, n)
                ok, gap = self._rollout_ok(path, mem_local, v_roll, frame.steer)
                if found is None or ok:
                    found = (path, ok, gap)
                if ok:
                    break           # 충돌 예상이면 여유를 늘려 한 번 더
            if found is None:
                continue
            result = found
            mode = "free" if not lane_hard else ("lane-tight" if relax < 0 else "lane")
            break           # 차선 안 경로가 있으면 차선 밖은 보지 않음 (롤아웃 실패 시 감속)
        if result is None:
            # 계획 실패: 이전 경로 유지, 없으면 기준선
            if self.path_odom is not None:
                self.rollout_ok = False
                self.last_dp_ms = (time.perf_counter() - t0) * 1e3
                self.debug["status"] = f"mem={len(mem_local)} 계획 실패 → 이전 경로 유지"
                return
            path, ok, gap = c[arc >= 0], False, None
        else:
            path, ok, gap = result
        self.path_odom = to_world(path, (ox, oy), opsi)
        self.path_time = frame.capture_time
        near = (cones_sl[:, 0] > -0.6) & (cones_sl[:, 0] < 3.0) & (np.abs(cones_sl[:, 1]) < 1.0) if len(cones_sl) else []
        self.active = bool(np.any(near)) if len(cones_sl) else False
        self.rollout_ok = ok
        self.last_dp_ms = (time.perf_counter() - t0) * 1e3
        self.debug = {"reference": c, "path": path, "memory": mem_local,
                      "used_cones": np.array([d.center for d in frame.cones]).reshape(-1, 2),
                      "status": f"mem={len(mem_local)} mode={mode} rollout={'OK' if ok else 'FAIL'} plan={self.last_dp_ms:.1f}ms"}

    # ------------------------------------------------------------------
    def _dp(self, l0, rear_sl, cones_sl, s_end, margin_extra, lane_hard=False, relaxed=False):
        pp, vp = self.P.proposed, self.P.vehicle
        ds, dl, l_max = (pp.ds_lane, pp.dl_lane, pp.l_max_lane) if lane_hard else (pp.ds, pp.dl, pp.l_max)
        ss = np.arange(0.0, s_end + 1e-9, ds)
        if len(ss) < 4:
            return None
        L = np.arange(-l_max, l_max + 1e-9, dl)
        nl = len(L)
        INF = 1e9
        hl, hw = vp.length / 2, vp.width / 2
        center_off = vp.length / 2 - vp.rear_overhang - vp.wheelbase   # 앞차축 → 차체 중심 (음수)
        hc = 0.065
        if relaxed:
            hard, soft = pp.relaxed_hard_clearance, pp.relaxed_soft_clearance
        elif lane_hard:
            hard, soft = pp.hard_clearance, pp.soft_clearance
        else:
            hard, soft = pp.free_hard_clearance, pp.free_soft_clearance
        hard += margin_extra
        soft += margin_extra

        # 과거 경로(앞차축 뒤): 현재 차체 직선
        def hist_l(s_query):
            # s<0 구간: 앞차축(0, l0) ~ 뒷차축(rear_sl) 직선 외삽
            sr, lr = rear_sl
            slope = (l0 - lr) / max(-sr, 1e-6)
            return l0 + slope * s_query

        cur_excess = max(0.0, max(abs(l0), abs(rear_sl[1])) + hw - pp.lane_half)
        cost = np.full(nl, INF)
        i0 = int(np.argmin(np.abs(L - l0)))
        cost[i0] = 0.0
        back = np.zeros((len(ss), nl), dtype=int)
        rear = np.zeros((len(ss), nl))          # 각 노드까지 최적 경로를 따랐을 때 뒷차축 l
        rear[0, :] = rear_sl[1]
        n_sub = 4
        for i in range(1, len(ss)):
            prev_l = L[:, None]                 # (pred, 1)
            new_l = L[None, :]                  # (1, new)
            # 뒷차축 궤적을 직접 적분 (앞차축에 끌려가는 곡선, tractrix):
            # 앞차축이 prev→new 로 ds 전진하는 동안 뒷차축은 앞차축 쪽으로 ds 만큼 끌려간다.
            # 직선이면 경로 위, 곡선이면 안쪽(내륜차), S자에서는 늦게 따라오는 효과까지 자연히 반영된다.
            lr = np.broadcast_to(rear[i - 1][:, None], (nl, nl)).copy()
            for m in range(1, n_sub + 1):
                lf = prev_l + (new_l - prev_l) * (m / n_sub)
                lr = lr + (lf - lr) * (ds / n_sub) / vp.wheelbase
            # 경로 곡률 (이번 칸 기준)
            if i >= 2:
                pprev = L[back[i - 1]][:, None]
            else:
                pprev = np.full((nl, 1), hist_l(-ds))
            curv = (new_l - 2 * prev_l + pprev) / ds ** 2
            # 차체 자세: 뒷차축 (s_r, lr) → 앞차축 (s_i, new_l)
            dy = new_l - lr                     # (pred, new)
            yaw = np.arctan2(dy, vp.wheelbase)
            cy_ = new_l + np.sin(yaw) * center_off
            cx_ = ss[i] + np.cos(yaw) * center_off
            node = np.zeros((nl, nl))
            # 장애물 (이 구간 근처 콘만, 콘 축으로 벡터화)
            if len(cones_sl):
                sel = cones_sl[(cones_sl[:, 0] >= ss[i] - 0.8) & (cones_sl[:, 0] <= ss[i] + 0.4)]
                if len(sel):
                    cs, sn = np.cos(yaw), np.sin(yaw)
                    dx_ = sel[:, 0, None, None] - cx_
                    dy_ = sel[:, 1, None, None] - cy_
                    lx = np.abs(cs * dx_ + sn * dy_) - hl
                    ly = np.abs(-sn * dx_ + cs * dy_) - hw
                    gap = np.where((lx > 0) | (ly > 0), np.hypot(np.maximum(lx, 0), np.maximum(ly, 0)),
                                   np.maximum(lx, ly)) - hc
                    node += pp.w_obstacle * (np.maximum(0.0, soft - gap) ** 2).sum(0)
                    node = np.where(gap.min(0) < hard, INF, node)
            # 차선 이탈 (soft): 차체 모서리 |l|
            edge = np.abs(cy_) + np.abs(np.sin(yaw)) * hl + np.cos(yaw) * hw
            excess = np.maximum(0.0, edge - pp.lane_half)
            w_lin = pp.w_lane_linear if lane_hard else pp.w_lane_linear_free   # 차선 밖 모드: 통로 가운데 우선
            node += pp.w_lane * np.minimum(excess, pp.lane_quad_cap) ** 2 + w_lin * excess
            if lane_hard:
                tol = pp.lane_tolerance + cur_excess * max(0.0, 1.0 - ss[i] / pp.lane_return_distance)
                node = np.where(excess > tol, INF, node)
            l_des = np.interp(ss[i], *self._l_des)
            node += pp.w_offset * (new_l - l_des) ** 2 * ds
            slope = (new_l - prev_l) / ds
            node += pp.w_slope * slope ** 2 * ds
            node += pp.w_curvature * curv ** 2 * ds
            node = np.where(np.abs(curv) > pp.max_curvature, INF, node)
            node = np.where(np.abs(slope) > 1.2, INF, node)
            total = cost[:, None] + node
            back[i] = np.argmin(total, axis=0)
            cost = total[back[i], np.arange(nl)]
            rear[i] = lr[back[i], np.arange(nl)]
        if not np.isfinite(cost).any() or cost.min() >= INF:
            return None
        j = int(np.argmin(cost))
        idx = [j]
        for i in range(len(ss) - 1, 0, -1):
            j = back[i, j]
            idx.append(j)
        ll = L[np.array(idx[::-1])]
        ll[0] = l0
        # 가벼운 평활화
        ll_s = ll.copy()
        ll_s[1:-1] = 0.25 * ll[:-2] + 0.5 * ll[1:-1] + 0.25 * ll[2:]
        ll_s[0] = l0
        return ss, ll_s

    @staticmethod
    def _trace(back, L, i_from, i_to):
        """노드 i_from의 각 l(=L 인덱스, 이전 단계 기준)에서 i_to까지 거슬러 올라간 l (pred 축 배열)."""
        idx = np.arange(len(L))
        for i in range(i_from, i_to, -1):
            idx = back[i, idx]
        return L[idx]

    def _rollout_ok(self, path_xy, cones_local, speed=1.0, steer=0.0):
        """선택 경로를 실제 제어기(Stanley + 조향 필터·변화율 제한)와 자전거 모델로 미리 굴려
        차체 사각형 충돌 확인 (차량 좌표, 지연 포함)."""
        vp, cp, pc = self.P.vehicle, self.P.control, self.P.perception
        if len(cones_local) == 0:
            return True, None
        if len(path_xy) < 2:
            return False, None
        rx, ry, psi = -vp.wheelbase, 0.0, 0.0
        v = min(max(speed, 0.3), cp.max_speed)
        delta = cmd = steer
        dt = 0.01
        rate = math.radians(vp.steer_rate_deg_s) * dt
        best = math.inf
        centers = np.asarray(cones_local)
        off = vp.length / 2 - vp.rear_overhang
        n_lat = int(pc.latency / dt)
        base = Path(path_xy, cp.geometry_window)
        arc = base.arc
        pts = base.points
        for k in range(int(self.P.proposed.rollout_sec / dt)):
            if k >= n_lat:
                fx, fy = rx + vp.wheelbase * math.cos(psi), ry + vp.wheelbase * math.sin(psi)
                raw, _, _ = stanley(Path.with_arc(to_local(pts, (fx, fy), psi), arc, cp.geometry_window),
                                    v, cp, vp.max_steer)
                cmd = cp.steering_current_weight * raw + (1 - cp.steering_current_weight) * cmd
            delta += max(-rate, min(rate, cmd - delta))
            rx += v * math.cos(psi) * dt
            ry += v * math.sin(psi) * dt
            psi += v / vp.wheelbase * math.tan(delta) * dt
            if k % 2:
                continue
            bx, by = rx + off * math.cos(psi), ry + off * math.sin(psi)
            g = rect_squares_gap(bx, by, psi, vp.length / 2, vp.width / 2, centers, 0.065)
            gm = float(g.min())
            best = min(best, gm)
            if gm < 0.0:
                return False, gm
        return True, best

    def path_for_control(self, odom_front_pose, now):
        if self.path_odom is None:
            return None, self.active
        x, y, psi = odom_front_pose
        return Path(to_local(self.path_odom, (x, y), psi), self.P.control.geometry_window), self.active


# ----------------------------------------------------------------------
# 차선 기준 단기 기억 (실차 cone_drive_test memory.mode = 1)

class LaneConeMemory:
    """콘을 차선 중심선 기준 (s, l)로만 들고 있는다.

    - 보이는 구간(s >= lane_visible_s_min): 이번 프레임 관측만 사용, 안 보이면 lane_miss_delete 번 만에 삭제
    - 사각지대(s < lane_visible_s_min): 뒷범퍼를 지날 때까지 유지 (뒷바퀴 보호)
    - 차가 움직이면 s만 바퀴 이동 거리만큼 줄인다 (자이로 미사용)
    """

    def __init__(self, params: SimParams):
        self.P = params
        self.tracks = []          # [s, l, hits, misses, weight]

    def _drop(self):
        pp, vp = self.P.proposed, self.P.vehicle
        limit = -(vp.front_axle_to_rear_bumper + pp.delete_behind)
        self.tracks = [t for t in self.tracks if t[0] >= limit and t[3] < pp.lane_miss_delete]

    def advance(self, ds):
        if ds > 0:
            for t in self.tracks:
                t[0] -= ds
            self._drop()

    def update(self, observed_sl, ranges=None):
        """ranges: 각 관측의 카메라 거리 (m). 거리 오차 σ = base + per_m·거리 → 1/σ² 가중 평균
        (화면 끝·가림으로 한 번 크게 틀린 관측이 기억 위치를 끌고 가지 않게)."""
        pp, pc = self.P.proposed, self.P.perception
        obs = np.asarray(observed_sl, float).reshape(-1, 2)
        rr = np.ones(len(obs)) if ranges is None else np.asarray(ranges, float)
        used = [False] * len(self.tracks)
        for o, r in zip(obs, rr):
            w = 1.0 / (pc.range_noise_base + pc.range_noise_per_m * r) ** 2
            best, bd = -1, pp.lane_match_radius
            for k, t in enumerate(self.tracks):
                if used[k]:
                    continue
                d = float(np.hypot(t[0] - o[0], t[1] - o[1]))
                if d < bd:
                    best, bd = k, d
            if best < 0:
                self.tracks.append([float(o[0]), float(o[1]), 1, 0, w])
                used.append(True)
            else:
                t = self.tracks[best]
                W = t[4]
                t[0] = (t[0] * W + o[0] * w) / (W + w)
                t[1] = (t[1] * W + o[1] * w) / (W + w)
                t[4] = min(W + w, 50.0 * w)            # 너무 굳지 않게 상한 (월드 기억과 같음)
                t[2] += 1
                t[3] = 0
                used[best] = True
        for k, t in enumerate(self.tracks):
            if not used[k] and t[0] >= pp.lane_visible_s_min:
                t[3] += 1                              # 사각지대는 놓침으로 세지 않음
        self._drop()

    def cones_sl(self):
        h = self.P.proposed.lane_confirm_hits
        return np.array([[t[0], t[1]] for t in self.tracks if t[2] >= h]).reshape(-1, 2)


class ProposedLanePlanner(ProposedPlanner):
    """제안안과 같은 계획(Frenet DP + 롤아웃)에 기억만 차선 기준 단기 기억으로 바꾼 것."""

    name = "proposed_lane"

    def __init__(self, params: SimParams):
        super().__init__(params)
        self.lane_memory = LaneConeMemory(params)
        self._frame = None
        self._last_odom = None

    def _observe(self, frame):
        self._frame = frame          # 차선 기준 좌표가 있어야 하므로 계획 직전에 처리

    def _memory_world(self, frame, arc, c, t, n):
        ox, oy, opsi = frame.odom_pose
        if self._last_odom is not None:   # 지난 프레임 이후 이동 거리 (바퀴 오도메트리; 방향은 쓰지 않음)
            self.lane_memory.advance(float(np.hypot(ox - self._last_odom[0], oy - self._last_odom[1])))
        self._last_odom = (ox, oy)
        obs = np.array([d.center for d in frame.cones]).reshape(-1, 2)
        ranges = np.hypot(obs[:, 0] - self.P.camera.camera_x, obs[:, 1]) if len(obs) else np.zeros(0)
        self.lane_memory.update(_project(obs, arc, c, t, n) if len(obs) else np.zeros((0, 2)), ranges)
        sl = self.lane_memory.cones_sl()
        if not len(sl):
            return np.zeros((0, 2))
        local = _from_frenet(sl[:, 0], sl[:, 1], arc, c, t, n)
        return to_world(local, (ox, oy), opsi)
