"""대회 트랙 기하 (설계 문서 5장).

월드 좌표: 원점 = 20번 매트 바깥 꼭짓점, x 오른쪽 0~13 m, y 위쪽 0~4 m.
차선 중심선 호 길이 s=0은 (1, 0.5), 진행 +x (반시계 주행).
횡오프셋 l은 진행 방향 왼쪽(+)이며, 반시계 루프에서 왼쪽 = 루프 안쪽.
"""
import math
import numpy as np

LANE_HALF_TAPE = 0.34       # 중심선 → 테이프 중심
TAPE_WIDTH = 0.03
LANE_HALF_INNER = 0.325     # 중심선 → 테이프 안쪽 끝 (주행 가능 폭 0.65 / 2)
CORNER_R = 0.5
CONE_SIZE = 0.13


class Track:
    """직선 4개 + 사분원 4개로 이루어진 차선 중심선."""

    def __init__(self):
        r = CORNER_R
        # (종류, 파라미터) ; line: (시작점, 끝점), arc: (중심, 시작각, 끝각)
        self.segments = [
            ("line", (1.0, 0.5), (12.0, 0.5)),
            ("arc", (12.0, 1.0), -math.pi / 2, 0.0),
            ("line", (12.5, 1.0), (12.5, 3.0)),
            ("arc", (12.0, 3.0), 0.0, math.pi / 2),
            ("line", (12.0, 3.5), (1.0, 3.5)),
            ("arc", (1.0, 3.0), math.pi / 2, math.pi),
            ("line", (0.5, 3.0), (0.5, 1.0)),
            ("arc", (1.0, 1.0), math.pi, 1.5 * math.pi),
        ]
        self.starts = []
        self.lengths = []
        s = 0.0
        for seg in self.segments:
            length = (math.dist(seg[1], seg[2]) if seg[0] == "line" else r * (seg[3] - seg[2]))
            self.starts.append(s)
            self.lengths.append(length)
            s += length
        self.length = s

    # ------------------------------------------------------------------
    def pose_at(self, s: float):
        """호 길이 s의 (x, y, 방향)."""
        s = s % self.length
        for seg, start, length in zip(self.segments, self.starts, self.lengths):
            if s <= start + length + 1e-12:
                t = s - start
                if seg[0] == "line":
                    a, b = np.array(seg[1]), np.array(seg[2])
                    d = (b - a) / length
                    p = a + d * t
                    return float(p[0]), float(p[1]), math.atan2(d[1], d[0])
                c, a0 = seg[1], seg[2]
                ang = a0 + t / CORNER_R
                return (c[0] + CORNER_R * math.cos(ang), c[1] + CORNER_R * math.sin(ang),
                        ang + math.pi / 2)
        raise AssertionError("unreachable")

    def point_at(self, s, l=0.0):
        x, y, psi = self.pose_at(s)
        return np.array([x - l * math.sin(psi), y + l * math.cos(psi)])

    def project(self, p, s_hint=None):
        """월드 점 → (s, l). s_hint가 있으면 그 근처 해를 우선."""
        p = np.asarray(p, dtype=float)
        best = None
        for seg, start, length in zip(self.segments, self.starts, self.lengths):
            if seg[0] == "line":
                a, b = np.array(seg[1]), np.array(seg[2])
                d = (b - a) / length
                t = float(np.clip(np.dot(p - a, d), 0.0, length))
                q = a + d * t
                n = np.array([-d[1], d[0]])
                l = float(np.dot(p - q, n))
                dist = float(np.linalg.norm(p - q))
            else:
                c, a0, a1 = np.array(seg[1]), seg[2], seg[3]
                v = p - c
                ang = math.atan2(v[1], v[0])
                # 구간 안으로 각도 정규화
                while ang < a0 - math.pi:
                    ang += 2 * math.pi
                while ang > a0 + math.pi:
                    ang -= 2 * math.pi
                ang_c = min(max(ang, a0), a1)
                t = CORNER_R * (ang_c - a0)
                q = c + CORNER_R * np.array([math.cos(ang_c), math.sin(ang_c)])
                rad = float(np.linalg.norm(v))
                l = CORNER_R - rad if ang_c == ang else None
                dist = float(np.linalg.norm(p - q))
                if l is None:
                    tangent = np.array([-math.sin(ang_c), math.cos(ang_c)])
                    normal = np.array([-tangent[1], tangent[0]])
                    l = float(np.dot(p - q, normal))
            s = start + t
            score = dist
            if s_hint is not None:
                ds = abs((s - s_hint + self.length / 2) % self.length - self.length / 2)
                score = dist + 0.05 * max(0.0, ds - 1.0)
            if best is None or score < best[0]:
                best = (score, s, l)
        return best[1] % self.length, best[2]

    def points_vec(self, ss, ls):
        """point_at의 벡터화 버전 (1 cm 표본 보간)."""
        if not hasattr(self, "_dense"):
            grid = np.arange(0.0, self.length + 0.01, 0.01)
            poses = np.array([self.pose_at(x) for x in grid])
            self._dense = (grid, poses[:, 0], poses[:, 1], np.unwrap(poses[:, 2]))
        grid, gx, gy, gp = self._dense
        s = np.mod(np.asarray(ss, float), self.length)
        x, y, psi = np.interp(s, grid, gx), np.interp(s, grid, gy), np.interp(s, grid, gp)
        ls = np.broadcast_to(np.asarray(ls, float), s.shape)
        return np.stack([x - ls * np.sin(psi), y + ls * np.cos(psi)], axis=-1)

    def sample(self, s0, s1, spacing, l=0.0):
        n = max(2, int(math.ceil((s1 - s0) / spacing)) + 1)
        ss = np.linspace(s0, s1, n)
        return ss, np.array([self.point_at(s, l) for s in ss])

    def corner_ranges(self):
        return [(st, st + ln) for seg, st, ln in zip(self.segments, self.starts, self.lengths) if seg[0] == "arc"]

    # ------------------------------------------------------------------
    # 매트 번호 → 매트가 시작되는 s (진행 방향 진입 가장자리)
    def mat_entry_s(self, mat: int) -> float:
        if mat == 1:
            return 10.0
        if 21 <= mat <= 30:
            return float(mat - 21)
        if mat in (3, 4):
            return 11.0 + math.pi / 4 + (mat - 3)
        if 6 <= mat <= 16:
            return 13.0 + math.pi / 2 + (mat - 6)
        if mat in (18, 19):
            return 24.0 + 3 * math.pi / 4 + (mat - 18)
        raise ValueError(f"corner or unknown mat {mat}")


# ----------------------------------------------------------------------
# 장애물 배치 (월드 좌표 콘 중심, (N,2))

STAGE1_GAP_DEFAULT = 0.47
STAGE1_SPACING_DEFAULT = 0.66   # 콘 묶음(2개) 사이 앞뒤 간격 (규격 0.50에서 실차 시험 배치로 변경)


def stage1_cones(gap: float = STAGE1_GAP_DEFAULT, spacing: float = STAGE1_SPACING_DEFAULT) -> np.ndarray:
    """1단계 라바콘 회피 (7, 8번). 진행 −x, 왼쪽 = 안쪽(y 감소).

    gap: 콘 지점에서 남는 차선 폭. 규격 0.47 m (바깥 콘 중심 = 테이프 중심, 안쪽 콘 붙임).
    gap을 넓히면 안쪽 콘만 테이프 쪽으로 물러난다 (두 콘이 겹치면 바깥 콘도 함께 이동).
    spacing: 묶음 사이 앞뒤 간격 (기본 0.66 m). 첫 묶음은 7번 진입 x=10.75.
    """
    inner_tape, outer_tape = 3.5 - LANE_HALF_TAPE, 3.5 + LANE_HALF_TAPE
    protrusion = 2 * LANE_HALF_INNER - gap           # 차선 안쪽 끝에서 돌출량 (규격 0.18)
    inner_center = LANE_HALF_INNER - protrusion + CONE_SIZE / 2   # 중심선 기준 안쪽 콘 중심
    outer_center = max(LANE_HALF_TAPE, inner_center + CONE_SIZE)
    cones = []
    for k, side in enumerate(("L", "R", "L", "R")):
        x = 10.75 - k * spacing
        sign = -1.0 if side == "L" else 1.0          # 왼쪽 = y 감소
        cones += [(x, 3.5 + sign * outer_center), (x, 3.5 + sign * inner_center)]
    return np.array(cones)


def guide_offset(u):
    """2단계 유도선 중심의 진행 왼쪽 오프셋 (u: 11번 시작부터 거리, m)."""
    return 0.34 * (1.0 - np.cos(2.0 * np.pi * np.asarray(u) / 3.0))


def stage2_cones(per_side: int = 19) -> np.ndarray:
    """2단계 유도선 (11~13번 + 보조 A~C). 중심선 호 길이 등간격, 법선 ±0.34 m."""
    u = np.linspace(0.0, 3.0, 30001)
    l = guide_offset(u)
    arc = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(u), np.diff(l)))])
    targets = np.linspace(0.0, arc[-1], per_side)
    uu = np.interp(targets, arc, u)
    ll = guide_offset(uu)
    dl = 0.34 * (2.0 * np.pi / 3.0) * np.sin(2.0 * np.pi * uu / 3.0)
    # 월드: x = 7 - u, y = 3.5 - l. 진행 방향 = d(x,y)/du = (-1, -dl)
    tx, ty = -np.ones_like(uu), -dl
    norm = np.hypot(tx, ty)
    tx, ty = tx / norm, ty / norm
    nx, ny = -ty, tx                      # 진행 왼쪽 법선
    cx, cy = 7.0 - uu, 3.5 - ll
    left = np.stack([cx + 0.34 * nx, cy + 0.34 * ny], axis=1)
    right = np.stack([cx - 0.34 * nx, cy - 0.34 * ny], axis=1)
    return np.concatenate([left, right])


def guide_centerline(n: int = 200) -> np.ndarray:
    u = np.linspace(0.0, 3.0, n)
    return np.stack([7.0 - u, 3.5 - guide_offset(u)], axis=1)
