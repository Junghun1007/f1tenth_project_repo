"""제안안(차선 기준 단기 기억, proposed_lane): 실차 cone_drive_test memory.mode=1과 같은 규칙."""
import numpy as np

from avoidance_sim.config import SimParams
from avoidance_sim.planners import LaneConeMemory
from avoidance_sim.scenarios import SCENARIOS
from avoidance_sim.sim import run


def test_visible_unseen_dropped_blind_zone_held():
    m = LaneConeMemory(SimParams())
    m.update([[0.6, 0.25]])
    m.update([[0.6, 0.25]])
    assert len(m.cones_sl()) == 1
    m.advance(0.4)                      # s = 0.2: 사각지대
    for _ in range(5):
        m.update([])
    assert np.allclose(m.cones_sl(), [[0.2, 0.25]])
    m.advance(0.8)                      # s = -0.6: 뒷범퍼(0.45)+0.10 지남
    assert len(m.cones_sl()) == 0
    m.update([[1.5, 0.1]]); m.update([[1.5, 0.1]])
    for _ in range(3):
        m.update([])                    # 보이는 구간에서 안 보이면 삭제
    assert len(m.cones_sl()) == 0


def test_proposed_lane_runs_stage1():
    r = run(SCENARIOS["stage1_cones"], "proposed_lane", SimParams(), seed=1, record=False)
    assert r.metrics["finished"]


def test_lane_memory_unaffected_by_gyro_bias():
    """자이로 바이어스를 크게 줘도 차선 기준 기억은 접촉이 늘지 않아야 한다 (월드 기억은 영향을 받음)."""
    P = SimParams()
    P.perception.gyro_bias_deg_s = 8.0
    lane = run(SCENARIOS["stage1_cones"], "proposed_lane", P, seed=1, record=False).metrics
    assert lane["finished"]
    assert lane["contacts"] <= 2


def test_lane_memory_stays_in_lane_on_zigzag():
    """가림·화면 끝의 나쁜 마지막 관측을 그대로 기억하면 차선 밖으로 튀던 시드들 (가중 평균으로 해결)."""
    for seed in (1, 2, 5, 8):
        m = run(SCENARIOS["stage1_cones"], "proposed_lane", SimParams(), seed=seed, record=False).metrics
        assert (m["stage1_lane_excess_m"] or 0) <= 0.03, (seed, m["stage1_lane_excess_m"])
