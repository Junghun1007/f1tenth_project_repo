import numpy as np

from avoidance_sim.camera import Camera
from avoidance_sim.config import SimParams
from avoidance_sim.perception import observe_cones, observe_lane, corner_confidence
from avoidance_sim.scenarios import SCENARIOS, with_layout, save_layout, load_layout
from avoidance_sim.sim import run
from avoidance_sim.track import Track


def _P():
    P = SimParams()
    P.perception.noise = False
    return P


def test_camera_hides_cone_directly_behind_and_sees_offset_one():
    P = _P()
    cam = Camera(P.camera)
    cones = np.array([[1.0, 0.0], [1.4, 0.0], [1.4, 0.35]])
    dets = observe_cones(cones, (0.0, 0.0, 0.0), P.perception, np.random.default_rng(0), camera=cam)
    seen = sorted(d.truth_index for d in dets)
    assert 0 in seen and 2 in seen and 1 not in seen


def test_camera_near_blind_zone():
    P = _P()
    cam = Camera(P.camera)
    # 카메라 하단 화각 밖(차량 바로 앞)은 안 보인다
    assert not cam.ground_visible(np.array([[0.1, 0.0]]))[0]
    assert cam.ground_visible(np.array([[1.0, 0.0]]))[0]


def test_partially_hidden_cone_can_get_occluder_depth():
    """가려진 콘의 박스에 앞 콘 픽셀이 섞이면 앞 콘 거리로 놓일 수 있다 (multiclass_yolo와 같은 오류)."""
    P = _P()
    P.camera.min_visible_fraction = 0.2
    cam = Camera(P.camera)
    cones = np.array([[0.7, 0.0], [1.1, 0.04]])
    dets = observe_cones(cones, (0.0, 0.0, 0.0), P.perception, np.random.default_rng(0), camera=cam)
    far = [d for d in dets if d.truth_index == 1]
    assert far, "far cone should still be detected"
    assert far[0].depth_source == 0
    assert far[0].center[0] < 0.85         # 실제 1.1 m 콘이 0.7 m 콘 위치로 잘못 놓임


def test_corner_offset_points_outward_before_corner():
    tr = Track()
    P = _P()
    corner = tr.corner_ranges()[0][0]          # 첫 코너 (좌회전) 시작
    conf = corner_confidence(tr, [corner - 3.0, corner - 1.0, corner - 0.3, corner + 0.3], corner - 1.5, P.perception)
    assert conf[0] == 0.0 and 0.0 < conf[1] < 1.0 and conf[2] == 1.0 and conf[3] == 1.0
    lane = observe_lane(tr, (*tr.point_at(corner - 0.5), 0.0), corner - 0.5, P.perception, np.random.default_rng(0))
    # 좌회전 → 경로는 중심선보다 오른쪽(바깥, y<0)
    d = lane.path[:, 1] - lane.center[:, 1]
    assert d.min() < -0.07


def test_layout_roundtrip_and_custom_scenario(tmp_path):
    cones = np.array([[10.5, 3.3], [9.6, 3.7]])
    p = tmp_path / "l.json"
    save_layout(p, cones)
    assert np.allclose(load_layout(p), cones)
    sc = with_layout(SCENARIOS["stage1_cones"], cones)
    assert np.allclose(sc.cones(), cones)
    r = run(sc, "proposed", seed=0, record=False)
    assert r.metrics["finished"]
