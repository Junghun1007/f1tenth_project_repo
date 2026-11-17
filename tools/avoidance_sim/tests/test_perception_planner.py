import math
import numpy as np

from avoidance_sim.config import SimParams
from avoidance_sim.perception import observe_cones, PerceptionFrame, LaneObs, ConeDetection
from avoidance_sim.planners import ProposedPlanner, ConeMemory
from avoidance_sim.sim import run
from avoidance_sim.scenarios import SCENARIOS


def _params(noise=False):
    P = SimParams()
    P.perception.noise = noise
    return P


def test_cone_behind_cone_is_hidden():
    P = _params()
    cones = np.array([[1.0, 0.0], [1.6, 0.0], [1.0, 0.4]])
    dets = observe_cones(cones, (0.0, 0.0, 0.0), P.perception, np.random.default_rng(0))
    seen = sorted(d.truth_index for d in dets)
    assert seen == [0, 2]


def test_out_of_bev_not_seen():
    P = _params()
    cones = np.array([[1.0, 0.9], [3.5, 0.0], [-0.2, 0.0]])
    assert observe_cones(cones, (0.0, 0.0, 0.0), P.perception, np.random.default_rng(0)) == []


def test_measured_center_matches_truth_without_noise():
    P = _params()
    d = observe_cones(np.array([[1.5, 0.2]]), (0.0, 0.0, 0.0), P.perception, np.random.default_rng(0))[0]
    assert np.linalg.norm(d.center - [1.5, 0.2]) < 0.02


def _frame(t, pose, dets_local):
    xs = np.arange(0, 3, 0.05)
    lane = LaneObs(np.stack([xs, np.zeros_like(xs)], 1), np.zeros((0, 2)), np.zeros((0, 2)))
    dets = [ConeDetection(np.asarray(c, float), np.asarray(c, float), 0.0, 1.0, 0) for c in dets_local]
    return PerceptionFrame(t, t, lane, dets, pose, pose)


def test_memory_keeps_cone_after_it_leaves_view_and_drops_it_behind_car():
    P = _params()
    mem = ConeMemory(P)
    for k in range(3):
        mem.update(_frame(k * 0.03, (0.0, 0.0, 0.0), [[0.5, 0.25]]))
    assert len(mem.confirmed_positions()) == 1
    # 차가 0.6 m 전진: 콘은 차량 기준 x=-0.1 (시야 밖) → 유지
    mem.update(_frame(1.0, (0.6, 0.0, 0.0), []))
    pos = mem.confirmed_positions()
    assert len(pos) == 1 and np.allclose(pos[0], [0.5, 0.25], atol=1e-6)
    # 뒷범퍼(0.45) + 0.10 뒤로 지나가면 삭제
    mem.update(_frame(2.0, (1.1, 0.0, 0.0), []))
    assert len(mem.confirmed_positions()) == 0


def test_dp_leaves_lane_only_when_blocked():
    pl = ProposedPlanner(_params())
    # 차선을 가로막는 콘 벽 (l = -0.3 ~ 0.3) → 차선 안 모드 불가, 이탈 모드는 가능
    wall = np.array([[1.5, l] for l in np.arange(-0.3, 0.31, 0.12)])
    assert pl._dp(0.0, (-0.33, 0.0), wall, 3.0, 0.0, lane_hard=True) is None
    ss, ll = pl._dp(0.0, (-0.33, 0.0), wall, 3.0, 0.0, lane_hard=False)
    assert np.max(np.abs(ll)) > 0.4
    # 한쪽만 살짝 들어온 콘 → 차선 안에서 반대쪽으로 회피
    ss, ll = pl._dp(0.0, (-0.33, 0.0), np.array([[1.5, 0.21]]), 3.0, 0.0, lane_hard=True)
    assert ll[np.argmin(np.abs(ss - 1.5))] < -0.0


def test_lap_without_obstacles_is_clean_and_deterministic():
    a = run(SCENARIOS["stage0_lap"], "proposed", seed=3, record=False).metrics
    b = run(SCENARIOS["stage0_lap"], "proposed", seed=3, record=False).metrics
    assert a["finished"] and a["contacts"] == 0
    assert a["time_s"] == b["time_s"]


def _slalom(spacing):
    cones = []
    for k, side in enumerate((1, -1, 1, -1)):
        s = 1.0 + k * spacing
        cones += [[s, side * 0.21], [s, side * 0.34]]
    return np.array(cones)


def test_zigzag_stays_in_lane_for_spec_and_wider_spacing():
    """1단계처럼 차선 안 틈이 있으면 차선 안 모드에서 경로가 나와야 한다 (차선 밖으로 크게 돌지 않음)."""
    pl = ProposedPlanner(_params())
    for spacing in (0.5, 0.75):
        # 플래너와 같은 순서: 차선 안(기본 여유) → 차선 안(여유 0)
        out = pl._dp(0.0, (-0.33, 0.0), _slalom(spacing), 3.0, 0.0, lane_hard=True)
        if out is None:
            out = pl._dp(0.0, (-0.33, 0.0), _slalom(spacing), 3.0, 0.0, lane_hard=True, relaxed=True)
        assert out is not None, spacing
        assert np.max(np.abs(out[1])) < 0.2


def test_memory_merges_noisy_duplicates():
    P = SimParams()
    mem = ConeMemory(P)
    # 같은 콘을 거리 방향으로 ±6 cm 흔들려 관측 (2 m 거리의 스테레오 오차 수준)
    for k, dx in enumerate((0.0, 0.06, -0.06, 0.05, -0.04, 0.06)):
        mem.update(_frame(k * 0.03, (0.0, 0.0, 0.0), [[2.0 + dx, 0.25]]))
    assert len(mem.confirmed_positions()) == 1
