"""시뮬레이션 루프와 지표 (설계 문서 8·11장)."""
from dataclasses import dataclass, field
import math
import numpy as np

from .config import SimParams
from .control import stanley, plan_speed
from .geometry import square_corners, sat_gap, polygon_distance, to_local, to_world, rect_squares_gap
from .perception import observe_lane, observe_cones, Odometry, PerceptionFrame
from .camera import Camera
from .planners import BaselinePlanner, ProposedPlanner, ProposedLanePlanner
from .scenarios import Scenario, STAGE1_ZONE
from .track import Track, LANE_HALF_INNER, CONE_SIZE
from .vehicle import Vehicle

PLANNERS = {"baseline": BaselinePlanner, "proposed": ProposedPlanner, "proposed_lane": ProposedLanePlanner}


@dataclass
class Frame:
    t: float
    pose: tuple                    # 뒷차축 (x, y, psi)
    footprint: np.ndarray
    v: float
    v_target: float
    delta: float
    path_world: np.ndarray         # 제어가 따라가는 경로 (실제 자세로 월드 변환)
    detections_world: np.ndarray   # 이번 인지 프레임 콘 관측 (월드)
    memory_world: np.ndarray       # 기억 중인 콘 (월드, 실제 자세 기준 변환)
    detections_local: np.ndarray
    memory_local: np.ndarray
    path_local: np.ndarray
    reference_local: np.ndarray
    lane_local: tuple
    contacts: list                 # 현재 접촉 중인 콘 인덱스
    status: str
    cam_image: np.ndarray = None   # 카메라 래스터 (콘 인덱스, -1=없음), 마지막 인지 프레임
    cam_boxes: list = None         # [(콘, (u0,v0,u1,v1), 깊이를 준 콘)]


@dataclass
class Result:
    scenario: str
    planner: str
    seed: int
    frames: list
    cones: np.ndarray
    trail: np.ndarray
    metrics: dict = field(default_factory=dict)
    view: tuple = None


def _part(cone, veh: Vehicle):
    c = to_local(np.asarray(cone)[None], veh.body_center(), veh.psi)[0]
    third = veh.p.length / 6
    return "front" if c[0] > third else ("rear" if c[0] < -third else "side")


def run(scenario: Scenario, planner_name: str, params: SimParams = None, seed: int = 0,
        record: bool = True) -> Result:
    P = params or SimParams()
    rng = np.random.default_rng(seed)
    tr = Track()
    cones = scenario.cones()
    fx, fy, fpsi = tr.pose_at(scenario.start_s)
    veh = Vehicle.from_front_axle(P.vehicle, fx, fy, fpsi, scenario.v0)
    odom = Odometry(P.perception, P.vehicle.wheelbase, fx, fy, fpsi, rng)
    planner = PLANNERS[planner_name](P)
    camera = Camera(P.camera)
    cp, vp = P.control, P.vehicle

    dt = P.dt
    perc_period = 1.0 / P.perception.rate_hz
    next_capture = 0.0
    queue = []
    t = 0.0
    s_prev = scenario.start_s
    progress = 0.0
    steer_cmd = 0.0
    obstacle_limit = cp.max_speed
    last_active = -1e9
    last_frame = None
    in_contact = set()
    contacts = []
    min_gap = math.inf
    lane_excess = 0.0
    max_steer = 0.0
    speeds = []
    plan_ms = []
    frames, trail = [], []
    next_record = 0.0
    finished = False
    off_track = False
    v_target = 0.0
    last_cam = None

    while t < P.max_time:
        # --- 인지 ---
        if t + 1e-9 >= next_capture:
            front = veh.front_axle()
            true_fp = (front[0], front[1], veh.psi)
            s_hint, _ = tr.project(front, s_hint=s_prev)
            lane = observe_lane(tr, true_fp, s_hint, P.perception, rng, camera)
            camera.last_owner = None
            dets = observe_cones(cones, true_fp, P.perception, rng, camera=camera) if len(cones) else []
            if record and getattr(camera, "last_owner", None) is not None:
                last_cam = (camera.last_owner.astype(np.int16), list(camera.last_boxes))
            queue.append(PerceptionFrame(t, t + P.perception.latency, lane, dets, odom.front_pose(), true_fp,
                                         veh.v, steer_cmd))
            next_capture += perc_period
        while queue and queue[0].available_time <= t + 1e-9:
            last_frame = queue.pop(0)
            planner.update(last_frame)
            if hasattr(planner, "last_dp_ms"):
                plan_ms.append(planner.last_dp_ms)

        # --- 제어 (100 Hz = dt) ---
        path, active = planner.path_for_control(odom.front_pose(), t)
        if active:
            last_active = t
        lim_target = cp.obstacle_speed if t - last_active <= cp.obstacle_clear_sec else cp.max_speed
        rate = cp.obstacle_decel if lim_target < obstacle_limit else cp.obstacle_accel
        obstacle_limit += max(-rate * dt, min(rate * dt, lim_target - obstacle_limit))
        if path is not None and path.valid:
            raw, _, _ = stanley(path, veh.v, cp, vp.max_steer)
            steer_cmd = cp.steering_current_weight * raw + (1 - cp.steering_current_weight) * steer_cmd
            _, vplan = plan_speed(path, cp, obstacle_limit)
            v_target = float(vplan[0])
            if planner_name.startswith("proposed") and not getattr(planner, "rollout_ok", True):
                v_target = min(v_target, P.control.rollout_fail_speed)
        else:
            v_target = 0.0
        veh.step(steer_cmd, v_target, dt)
        odom.update(veh.v, veh.yaw_rate, dt)
        t += dt

        # --- 지표 ---
        front = veh.front_axle()
        s_now, l_now = tr.project(front, s_hint=s_prev)
        ds = (s_now - s_prev + tr.length / 2) % tr.length - tr.length / 2
        progress += ds
        s_prev = s_now
        body = veh.footprint()
        bc = veh.body_center()
        now_contact = set()
        if len(cones):
            gaps = rect_squares_gap(bc[0], bc[1], veh.psi, vp.length / 2, vp.width / 2, cones, CONE_SIZE / 2)
            for i in np.flatnonzero(gaps < 0):
                now_contact.add(int(i))
                if i not in in_contact:
                    contacts.append({"t": round(t, 2), "cone": int(i), "part": _part(cones[i], veh)})
            j = int(np.argmin(gaps))
            g = float(gaps[j])
            if 0 <= g < 0.3:      # SAT 간격은 모서리끼리일 때 실제보다 작으므로 가까운 것만 정확히
                g = polygon_distance(body, square_corners(cones[j], CONE_SIZE))
            min_gap = min(min_gap, g)
        in_contact = now_contact
        if STAGE1_ZONE[0] <= s_now <= STAGE1_ZONE[1] and scenario.obstacles in ("stage1", "both"):
            for corner in body:
                _, lc = tr.project(corner, s_hint=s_now)
                lane_excess = max(lane_excess, abs(lc) - LANE_HALF_INNER)
        max_steer = max(max_steer, abs(veh.delta))
        speeds.append(veh.v)
        if abs(l_now) > 1.6:
            off_track = True

        if record and t + 1e-9 >= next_record:
            next_record += 1.0 / 30.0
            trail.append(front.copy())
            fr = _snapshot(t, veh, v_target, path, planner, last_frame, odom, in_contact)
            if last_cam is not None:
                fr.cam_image, fr.cam_boxes = last_cam
            frames.append(fr)

        if progress >= scenario.distance:
            finished = True
            break
        if off_track:
            break

    metrics = {
        "finished": finished,
        "off_track": off_track,
        "time_s": round(t, 2),
        "contacts": len(contacts),
        "contact_parts": ",".join(sorted({c["part"] for c in contacts})) or "-",
        "contact_detail": contacts,
        "min_clearance_m": round(min_gap, 3) if math.isfinite(min_gap) else None,
        "stage1_lane_excess_m": round(lane_excess, 3) if scenario.obstacles in ("stage1", "both") else None,
        "max_steer_deg": round(math.degrees(max_steer), 1),
        "avg_speed": round(float(np.mean(speeds)), 2) if speeds else 0.0,
        "max_speed": round(float(np.max(speeds)), 2) if speeds else 0.0,
        "plan_ms_avg": round(float(np.mean(plan_ms)), 1) if plan_ms else None,
        "plan_ms_max": round(float(np.max(plan_ms)), 1) if plan_ms else None,
    }
    return Result(scenario.name, planner_name, seed, frames, cones, np.array(trail), metrics, view=scenario.view)


def _snapshot(t, veh, v_target, path, planner, frame, odom, in_contact):
    front = veh.front_axle()
    fp = (front[0], front[1], veh.psi)
    empty = np.zeros((0, 2))
    path_local = path.points if path is not None else empty
    path_world = to_world(path_local, fp[:2], fp[2]) if len(path_local) else empty
    dbg = planner.debug or {}
    # 기억 콘: 플래너는 odom 좌표로 들고 있으므로 현재 odom 자세 기준으로 차량 좌표화
    mem_local = empty
    if hasattr(planner, "memory"):
        mw = planner.memory.confirmed_positions()
        if len(mw):
            ox, oy, opsi = odom.front_pose()
            mem_local = to_local(mw, (ox, oy), opsi)
    det_local = empty
    lane_local = (empty, empty)
    if frame is not None:
        det_local = np.array([d.center for d in frame.cones]).reshape(-1, 2)
        # 촬영 시점 → 현재 차량 좌표 (실제 자세 사용, 화면 표시용)
        det_world = to_world(det_local, frame.true_pose[:2], frame.true_pose[2]) if len(det_local) else empty
        det_local = to_local(det_world, fp[:2], fp[2]) if len(det_world) else empty
        lw = [to_world(x, frame.true_pose[:2], frame.true_pose[2]) if len(x) else empty
              for x in (frame.lane.left_tape, frame.lane.right_tape)]
        lane_local = tuple(to_local(x, fp[:2], fp[2]) if len(x) else empty for x in lw)
    ref = dbg.get("reference", empty)
    if frame is not None and len(ref):
        ref = to_local(to_world(ref, frame.true_pose[:2], frame.true_pose[2]), fp[:2], fp[2])
    return Frame(
        t=t, pose=(veh.x, veh.y, veh.psi), footprint=veh.footprint(), v=veh.v, v_target=v_target,
        delta=veh.delta, path_world=path_world,
        detections_world=to_world(det_local, fp[:2], fp[2]) if len(det_local) else empty,
        memory_world=to_world(mem_local, fp[:2], fp[2]) if len(mem_local) else empty,
        detections_local=det_local, memory_local=mem_local, path_local=path_local,
        reference_local=ref, lane_local=lane_local, contacts=sorted(in_contact),
        status=dbg.get("status", ""))
