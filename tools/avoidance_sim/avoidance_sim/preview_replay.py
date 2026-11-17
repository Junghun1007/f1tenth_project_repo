"""시뮬레이터 주행을 cone_drive_test 웹 프리뷰와 같은 텔레메트리 JSON으로 바꿔 재생한다.

    python -m avoidance_sim.preview_replay --export ../../src/cone_drive_test/web/demo   # 샘플 파일만 생성
    python -m avoidance_sim.preview_replay --serve 8765                                   # 브라우저로 재생

실차 없이 화면 배치를 확인하는 용도다. 단계별 시간·사용률은 그럴듯한 고정값에 약간의 흔들림을 준 가짜 값이다.
"""
import argparse
import http.server
import io
import json
import math
import os
import threading
import time

import numpy as np
from PIL import Image, ImageDraw

from .config import SimParams
from .scenarios import SCENARIOS
from .sim import run
from .track import Track, LANE_HALF_TAPE
from .geometry import to_local

WEB_ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), "../../../src/cone_drive_test/web"))
STAGES = [("receive", 0.6), ("stabilize_bev", 1.1), ("lane_infer", 3.2), ("lane_post", 1.4), ("yolo_pre", 0.5),
          ("yolo_infer", 6.8), ("yolo_post", 0.3), ("box_depth", 0.4), ("memory_plan", 1.9), ("control", 0.2), ("web", 0.3)]


def _pts(a, n=40):
    a = np.asarray(a, float).reshape(-1, 2)
    if len(a) > n:
        a = a[np.linspace(0, len(a) - 1, n).astype(int)]
    return [[round(float(x), 3), round(float(y), 3)] for x, y in a]


def _in_bev(a):
    a = np.asarray(a, float).reshape(-1, 2)
    return a[(a[:, 0] >= -0.5) & (a[:, 0] <= 3.2) & (np.abs(a[:, 1]) <= 0.8)] if len(a) else a


def frames_from_sim(scenario="stage1_cones", seed=1, mode="drive"):
    P = SimParams()
    res = run(SCENARIOS[scenario], "proposed", P, seed=seed, record=True)
    rng = np.random.default_rng(0)
    out = []
    for k, f in enumerate(res.frames):
        st = f.status
        plan_mode = "center"
        for tok in st.split():
            if tok.startswith("mode="):
                plan_mode = tok[5:]
        rollout_ok = "rollout=FAIL" not in st
        stages = []
        for name, ms in STAGES:
            m = ms * (1 + 0.08 * rng.standard_normal())
            if name == "memory_plan" and plan_mode == "center":
                m = 0.1
            stages.append({"name": name, "ms": round(m, 2), "fps": round(1000 / max(m, 1e-3), 1)})
        dets = [{"cls": "con", "score": round(float(0.82 + 0.1 * rng.random()), 2), "x": round(float(x), 3),
                 "y": round(float(y), 3), "length": 0.13, "width": 0.13, "yaw": 0.0}
                for x, y in _in_bev(f.detections_local)]
        mem = [{"x": round(float(x), 3), "y": round(float(y), 3), "confirmed": True, "id": i}
               for i, (x, y) in enumerate(_in_bev(f.memory_local))]
        left, right = f.lane_local
        out.append({
            "t": round(f.t, 3), "state": "DRIVING" if mode == "drive" else "MANUAL", "mode": mode, "reason": "",
            "readiness": {"attitude_ok": True, "lane_model_ready": True, "yolo_model_ready": True, "vesc_connected": True,
                          "erpm_fresh": True, "lane_fresh": True, "client_alive": True},
            "inspection": {"attitude": {"status": "PASS", "height_m": 0.201, "roll_deg": -0.4, "pitch_down_deg": 13.2, "message": ""},
                           "models": [{"name": "lane (Fast-SCNN)", "status": "READY", "stage": "ready", "elapsed_s": 2.1, "message": ""},
                                      {"name": "YOLOX-tiny 512x320", "status": "READY", "stage": "ready", "elapsed_s": 3.4, "message": ""}]},
            "vehicle": {"speed": round(f.v, 2), "target": round(f.v_target, 2), "steer_deg": round(math.degrees(f.delta), 1),
                        "cte": round(float(rng.normal(0, 0.01)), 3), "duty": round(0.01 + 0.05 * f.v_target, 3), "brake": 0.0},
            "lane": {"left": _pts(_in_bev(left)), "right": _pts(_in_bev(right)), "center": _pts(_in_bev(f.reference_local)),
                     "path": _pts(_in_bev(f.reference_local))},
            "plan": {"path": _pts(_in_bev(f.path_local)), "mode": plan_mode, "rollout_ok": rollout_ok},
            "detections": dets, "memory": mem,
            "stages": stages,
            "usage": {"cpu": round(38 + 4 * rng.random(), 1), "proc_cpu": round(61 + 6 * rng.random(), 1),
                      "mem": 47.5, "gpu": round(58 + 8 * rng.random(), 1)},
            "params_version": 1,
        })
    return res, out


def bev_image(res, frame_index, w=120, h=300, mpp=0.01):
    """트랙 바닥·테이프·콘을 위에서 본 120×300 그림 (실차 BEV와 같은 x 0~3 m, y ±0.6 m)."""
    f = res.frames[frame_index]
    tr = Track()
    rx, ry, psi = f.pose
    fx, fy = rx + 0.33 * math.cos(psi), ry + 0.33 * math.sin(psi)
    img = Image.new("RGB", (w, h), (92, 96, 92))
    d = ImageDraw.Draw(img)

    def px(p):
        loc = to_local(np.asarray(p, float).reshape(-1, 2), (fx, fy), psi)
        return [((0.6 - y) / mpp, (3.0 - x) / mpp) for x, y in loc]

    ss = np.arange(0, tr.length, 0.02)
    for off, col in ((LANE_HALF_TAPE, (235, 235, 235)), (-LANE_HALF_TAPE, (235, 235, 235)), (LANE_HALF_TAPE + 0.65, (235, 235, 235))):
        pts = px(tr.points_vec(ss, off))
        for a, b in zip(pts[:-1], pts[1:]):
            if abs(a[0] - b[0]) < 20 and abs(a[1] - b[1]) < 20:
                d.line([a, b], fill=col, width=4)
    for c in res.cones:
        q = px([[c[0] - 0.065, c[1] - 0.065], [c[0] + 0.065, c[1] - 0.065], [c[0] + 0.065, c[1] + 0.065], [c[0] - 0.065, c[1] + 0.065]])
        d.polygon(q, fill=(240, 120, 30))
    arr = np.asarray(img).astype(float)
    arr += np.random.default_rng(frame_index).normal(0, 6, arr.shape)
    return Image.fromarray(np.clip(arr, 0, 255).astype(np.uint8))


def camera_image(res, frame_index, w=640, h=400):
    """검출 모드 카메라 패널용 간단한 원근 그림."""
    f = res.frames[frame_index]
    img = Image.new("RGB", (w, h), (70, 74, 70))
    d = ImageDraw.Draw(img)
    d.rectangle([0, 0, w, int(h * 0.42)], fill=(150, 160, 170))
    fpx, cy, hc, pitch = 350.0, h / 2, 0.20, math.radians(13)

    def proj(x, y, z=0.0):
        xc, zc = x + 0.09, z - hc
        X = -y
        Y = -(zc * math.cos(pitch) - xc * math.sin(pitch))
        Z = xc * math.cos(pitch) + zc * math.sin(pitch)
        if Z <= 0.05:
            return None
        return (w / 2 + fpx * X / Z, cy + fpx * Y / Z)

    for side in (-1, 1):
        pts = [proj(x, side * 0.34) for x in np.linspace(0.2, 4, 40)]
        pts = [p for p in pts if p]
        d.line(pts, fill=(235, 235, 235), width=5)
    boxes = []
    for x, y in sorted(_in_bev(f.detections_local).tolist(), key=lambda p: -p[0]):
        base = proj(x, y)
        top = proj(x, y, 0.18)
        if not base or not top:
            continue
        half = fpx * 0.065 / (x + 0.09)
        d.polygon([(base[0] - half, base[1]), (base[0] + half, base[1]), (top[0] + half * 0.25, top[1]), (top[0] - half * 0.25, top[1])],
                  fill=(240, 120, 30))
        boxes.append([base[0] - half, top[1], base[0] + half, base[1]])
    d.line([(0, int(h * 0.35)), (w, int(h * 0.35))], fill=(255, 220, 0), width=2)
    for b in boxes:
        d.rectangle(b, outline=(255, 60, 60), width=2)
        d.text((b[0], b[1] - 12), "con 0.87", fill=(255, 255, 255))
    return img


def export(out_dir, scenario="stage1_cones", seed=1):
    os.makedirs(out_dir, exist_ok=True)
    res, frames = frames_from_sim(scenario, seed)
    # 지그재그 안에서 회피 중인 프레임: 기억 콘이 많고 계획 모드가 center가 아닌 것
    idx = max(range(len(frames)), key=lambda i: (frames[i]["plan"]["mode"] != "center", frames[i]["plan"]["rollout_ok"], len(frames[i]["memory"]), -i))
    keep = frames[max(0, idx - 60): idx + 60]
    with open(os.path.join(out_dir, "frames.json"), "w", encoding="utf-8") as fh:
        json.dump({"frames": keep, "focus": idx - max(0, idx - 60)}, fh, ensure_ascii=False)
    bev_image(res, idx).save(os.path.join(out_dir, "bev.jpg"), quality=80)
    camera_image(res, idx).save(os.path.join(out_dir, "camera.jpg"), quality=80)
    print(f"{len(keep)}프레임, 대표 프레임 {idx} → {out_dir}")


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--export")
    ap.add_argument("--scenario", default="stage1_cones")
    ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args(argv)
    if a.export:
        export(a.export, a.scenario, a.seed)


if __name__ == "__main__":
    main()
