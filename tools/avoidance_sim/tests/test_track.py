import math
import numpy as np
import pytest

from avoidance_sim.track import Track, stage1_cones, stage2_cones, LANE_HALF_INNER, CONE_SIZE
from avoidance_sim.geometry import rect_corners, polygons_overlap, polygon_distance, square_corners


def test_lap_length():
    assert Track().length == pytest.approx(26 + math.pi, abs=1e-6)


def test_pose_continuity_and_project_roundtrip():
    tr = Track()
    for s in np.linspace(0, tr.length - 1e-6, 400):
        for l in (-0.3, 0.0, 0.4):
            p = tr.point_at(s, l)
            s2, l2 = tr.project(p, s_hint=s)
            assert abs((s2 - s + tr.length / 2) % tr.length - tr.length / 2) < 1e-6
            assert l2 == pytest.approx(l, abs=1e-6)


def test_start_pose_mat30():
    tr = Track()
    x, y, psi = tr.pose_at(tr.mat_entry_s(30) + 0.5)
    assert (x, y) == pytest.approx((10.5, 0.5))
    assert psi == pytest.approx(0.0)


def test_stage1_free_width():
    cones = stage1_cones()
    assert len(cones) == 8
    # 왼쪽(안쪽) 지점: 안쪽 콘의 차선 쪽 끝 → 반대 테이프 안쪽 끝
    left_edge = cones[1, 1] + CONE_SIZE / 2
    lane_outer_edge = 3.5 + LANE_HALF_INNER
    assert lane_outer_edge - left_edge == pytest.approx(0.47, abs=1e-6)


def test_stage2_spacing_and_peak():
    cones = stage2_cones()
    assert len(cones) == 38
    left, right = cones[:19], cones[19:]
    gaps = np.linalg.norm(np.diff((left + right) / 2, axis=0), axis=1)
    # 중심선 호 길이 등간격 ≈ 18.6 cm
    assert np.mean(gaps) == pytest.approx(0.186, abs=0.003)
    # 시작/끝은 기존 테이프 위
    assert sorted([left[0, 1], right[0, 1]]) == pytest.approx([3.16, 3.84], abs=1e-6)
    # 최대 이탈 지점(12번 중앙 x=5.5)에서 오른쪽 줄 = 기존 안쪽 테이프 y=3.16
    mid = right[9]
    assert mid[0] == pytest.approx(5.5, abs=1e-6)
    assert mid[1] == pytest.approx(3.16, abs=1e-6)


def test_sat():
    a = rect_corners(0, 0, 0.3, 0.5, 0.3)
    assert polygons_overlap(a, square_corners((0.2, 0.0), 0.13))
    b = square_corners((0, 0.30), 0.13)
    d = polygon_distance(rect_corners(0, 0, 0.0, 0.5, 0.3), b)
    assert d == pytest.approx(0.30 - 0.065 - 0.15, abs=1e-9)


def test_stage1_pair_spacing_default_66cm():
    cones = stage1_cones()
    xs = sorted({round(float(x), 6) for x in cones[:, 0]}, reverse=True)
    assert len(xs) == 4
    assert np.diff(xs) == pytest.approx([-0.66] * 3, abs=1e-6)
    wide = stage1_cones(spacing=0.80)
    assert wide[:, 0].max() - wide[:, 0].min() == pytest.approx(2.40, abs=1e-6)
