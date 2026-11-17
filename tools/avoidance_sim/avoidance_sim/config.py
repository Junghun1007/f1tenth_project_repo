"""파라미터 모음. 수치 출처는 설계 문서와 1113ML auto_control.yaml."""
from dataclasses import dataclass, field, asdict, fields, is_dataclass
import math


@dataclass
class VehicleParams:
    wheelbase: float = 0.33
    length: float = 0.50
    width: float = 0.30
    front_overhang: float = 0.05   # 앞차축 → 앞범퍼
    rear_overhang: float = 0.12    # 뒷차축 → 뒷범퍼
    max_steer_deg: float = 30.0
    steer_rate_deg_s: float = 240.0
    speed_tau: float = 0.2
    max_accel: float = 1.0
    max_decel: float = 2.0

    @property
    def max_steer(self) -> float:
        return math.radians(self.max_steer_deg)

    @property
    def front_axle_to_rear_bumper(self) -> float:
        return self.wheelbase + self.rear_overhang


@dataclass
class PerceptionParams:
    bev_x_max: float = 3.0
    bev_y_max: float = 0.6
    camera_x: float = 0.0          # 앞차축 기준
    rate_hz: float = 30.0
    latency: float = 0.05
    lane_noise: float = 0.01
    lane_spacing: float = 0.05
    detect_prob: float = 0.95
    min_visible_fraction: float = 0.5
    range_noise_base: float = 0.01
    range_noise_per_m: float = 0.02
    bearing_noise_deg: float = 0.5
    cone_size: float = 0.13
    odom_speed_noise: float = 0.02
    gyro_bias_deg_s: float = 0.5
    gyro_noise_deg_s: float = 0.3
    noise: bool = True             # False면 모든 노이즈 OFF (가림은 유지)
    camera_model: bool = True      # False면 예전 단순 모델 (각도 구간 가림)
    # 차선에도 카메라 하단 사각지대를 적용할지. 기본 False = "차선은 BEV 0~3 m 전체에서 안다" 가정
    # (True면 경로가 앞차축 약 0.23 m 앞부터 시작 → Stanley 최근접점이 경로 시작점이 되어 일찍 꺾음)
    lane_camera_frustum: bool = False
    # line_detactor 코너 바깥쪽 이동 (1113ML line_detactor.yaml)
    corner_outward_offset: float = 0.08
    corner_entry_distance: float = 2.2
    corner_entry_full_offset_distance: float = 0.6


@dataclass
class CameraParams:
    """OAK CAM_A 콘 검출 경로 (1113ML multiclass_yolo 기준).
    위치·높이는 bev_config.yaml. multiclass_yolo는 OAK를 직접 열어 1280x800 undistort만 하고
    camera_driver의 fixed_view_zoom은 쓰지 않는다 → 줌 1.0. 기본 모델 holdout10(512x320), ROI 65 %."""
    camera_x: float = -0.09        # 앞차축 기준 (bev_config camera_x_m)
    camera_height: float = 0.20    # manual_camera_height_m
    pitch_deg: float = 13.0        # 아래로 (bev_config 주석의 실측 예시)
    hfov_deg: float = 69.0         # OAK CAM_A 원본 수평 화각
    zoom: float = 1.0              # multiclass_yolo: 줌 없음 (camera_driver의 1.25와 다름)
    image_width: int = 1280
    image_height: int = 800
    roi_height_percent: float = 65.0   # multiclass_yolo 기본 (holdout10 512x320). roi80 모델은 80
    raster_scale: float = 0.25     # 가림 계산용 해상도 배율 (320x200)
    cone_height: float = 0.18
    cone_top: float = 0.03
    min_visible_fraction: float = 0.35   # 보이는 면적 비율이 이 이상이어야 YOLO 검출
    min_box_px: float = 12.0       # 원본 해상도 기준 최소 박스 높이
    box_core_fraction: float = 0.6 # multiclass_yolo object.box_core_fraction
    min_points: int = 6
    min_point_share: float = 0.10
    window_base: float = 0.05
    window_per_m: float = 0.03


@dataclass
class ControlParams:
    rate_hz: float = 100.0
    stanley_gain: float = 1.3
    softening_speed: float = 0.3
    heading_lookahead: float = 0.05
    geometry_window: float = 0.16
    corner_heading_threshold_deg: float = 60.0
    corner_opposing_ratio: float = 0.8
    steering_current_weight: float = 0.4
    min_speed: float = 1.0
    max_speed: float = 1.8
    max_lateral_accel: float = 1.6
    planning_decel: float = 0.5
    obstacle_speed: float = 1.0
    obstacle_decel: float = 0.35
    obstacle_accel: float = 0.35
    obstacle_clear_sec: float = 1.0
    path_hold_sec: float = 0.25
    rollout_fail_speed: float = 0.5   # 제안안: 롤아웃 충돌 예상 시 속도 상한


@dataclass
class BaselineParams:
    max_age: float = 0.25
    safety_margin: float = 0.05
    max_offset: float = 0.25
    ramp: float = 0.30            # 이동량이 0→최대가 되는 길이
    hold_before: float = 0.10
    hold_after: float = 0.10


@dataclass
class ProposedParams:
    gate: float = 0.06            # 시선 방향 게이트 기본값 (+2.5σ 거리 오차)
    gate_radial_max: float = 0.17
    gate_lateral: float = 0.06    # 옆 방향 게이트 (방향 오차는 작다)
    merge_distance: float = 0.10  # 실제 콘 중심 간격(≥13 cm)보다 가까운 기억은 합친다
    confirm_hits: int = 3
    miss_delete: int = 4
    delete_behind: float = 0.10   # 뒷범퍼 뒤 여유
    s_min: float = 0.0
    s_max: float = 3.0
    ds: float = 0.15              # 차선 이탈 허용 모드 격자
    l_max: float = 1.0
    dl: float = 0.04
    ds_lane: float = 0.10         # 차선 안 모드 격자 (더 촘촘)
    l_max_lane: float = 0.36
    dl_lane: float = 0.02
    lane_half: float = 0.325
    hard_clearance: float = 0.015           # 지그재그 탐색 결과 (sim_output/sweep)
    relaxed_hard_clearance: float = 0.0     # 차선 안 2차 시도: 여유 0까지 허용 (겹침은 계획하지 않음)
    relaxed_soft_clearance: float = 0.04
    soft_clearance: float = 0.06            # 지그재그 탐색 결과: 0.10→0.06에서 무접촉 79→90 %
    # 차선 밖 모드: 폭 여유가 있어 넓은 여유가 유리 (유도선형 배치에서 0.06이면 접촉이 늘어남)
    free_hard_clearance: float = 0.005
    free_soft_clearance: float = 0.10
    w_obstacle: float = 4000.0
    w_lane: float = 400.0        # 이탈 초반(lane_quad_cap까지) 제곱 비용
    w_lane_linear: float = 40.0  # 이탈량 선형 비용: 차선이 막히면 차선 밖 통로를 고를 수 있게
    lane_quad_cap: float = 0.06
    w_lane_linear_free: float = 4.0  # 차선 밖 모드에서는 차선 쪽으로 당기는 힘을 약하게
    lane_tolerance: float = 0.03   # 차선 안 모드에서 허용하는 차체 이탈 (테이프 폭)
    lane_return_distance: float = 0.8  # 이미 차선 밖이면 이 거리 안에 복귀
    w_offset: float = 20.0
    w_slope: float = 1.0
    w_curvature: float = 0.08
    max_curvature: float = 3.5   # 격자 l'' 한계. 실제 곡률 한계 tan30/0.33=1.75는 롤아웃이 검사
    rollout_sec: float = 1.5
    rollout_retry_margin: float = 0.03   # 롤아웃 충돌 예상 시 여유를 이만큼 늘려 재계획
    backward_extension: float = 0.8
    # proposed_lane (차선 기준 단기 기억, 실차 cone_drive_test memory.mode=1과 같은 규칙)
    lane_confirm_hits: int = 2        # 이만큼 관측돼야 계획에 사용
    lane_miss_delete: int = 3         # 보이는 구간에서 연속으로 안 보이면 삭제
    lane_match_radius: float = 0.12   # 같은 콘으로 볼 (s,l) 거리
    lane_visible_s_min: float = 0.45  # 이보다 가까우면 카메라 사각지대 → 뒷범퍼 지날 때까지 유지


@dataclass
class SimParams:
    dt: float = 0.01
    max_time: float = 60.0
    seed: int = 0
    vehicle: VehicleParams = field(default_factory=VehicleParams)
    perception: PerceptionParams = field(default_factory=PerceptionParams)
    camera: CameraParams = field(default_factory=CameraParams)
    control: ControlParams = field(default_factory=ControlParams)
    baseline: BaselineParams = field(default_factory=BaselineParams)
    proposed: ProposedParams = field(default_factory=ProposedParams)


def apply_overrides(params, overrides: dict):
    """{'perception': {'noise': False}} 형태의 dict로 덮어쓴다."""
    for key, value in (overrides or {}).items():
        if not hasattr(params, key):
            raise KeyError(f"unknown parameter: {key}")
        current = getattr(params, key)
        if is_dataclass(current) and isinstance(value, dict):
            apply_overrides(current, value)
        else:
            setattr(params, key, type(current)(value) if current is not None else value)
    return params


def to_dict(params) -> dict:
    return asdict(params)
