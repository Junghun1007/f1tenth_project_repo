"""cone_drive_test C++ 이식 검증용 골든 데이터 생성.

시뮬레이터 코드를 바꾸지 않고 ConeMemory.update / ProposedPlanner._dp / _rollout_ok를 감싸 입력과 출력을 JSON으로 저장한다.

    python -m avoidance_sim.export_fixtures --out ../../src/cone_drive_test/test/fixtures
"""
import argparse
import dataclasses
import json
import os

import numpy as np

from . import planners as PL
from .config import SimParams
from .scenarios import SCENARIOS
from .sim import run


def _f(a):
    return np.asarray(a, float).round(9).tolist()


def _params(P: SimParams):
    return {"proposed": dataclasses.asdict(P.proposed), "vehicle": dataclasses.asdict(P.vehicle),
            "control": dataclasses.asdict(P.control),
            "perception": {k: v for k, v in dataclasses.asdict(P.perception).items() if isinstance(v, (int, float, bool))},
            "camera_x": P.camera.camera_x}


def record(scenarios=("stage1_cones", "stage2_guide"), seed=7, max_steps=300):
    mem_steps, dp_calls, rollouts = [], [], []
    orig_update, orig_dp, orig_roll = PL.ConeMemory.update, PL.ProposedPlanner._dp, PL.ProposedPlanner._rollout_ok
    orig_expected = PL.expected_visible
    state = {"expected": None, "scenario": None, "count": 0}

    def expected_wrap(centers, pose, *a, **k):
        out = orig_expected(centers, pose, *a, **k)
        state["expected"] = _f(np.asarray(centers)[sorted(out)]) if len(out) else []
        return out

    def update_wrap(self, frame):
        state["expected"] = []
        orig_update(self, frame)
        if state["count"] < max_steps:
            mem_steps.append({"scenario": state["scenario"], "pose": list(frame.odom_pose), "t": frame.capture_time,
                              "detections": [_f(d.center) for d in frame.cones], "expected_world": state["expected"],
                              "confirmed": _f(self.confirmed_positions()),
                              "tracks": [{"pos": _f(t.pos), "hits": t.hits, "misses": t.misses} for t in self.tracks]})
            state["count"] += 1

    def dp_wrap(self, l0, rear_sl, cones_sl, s_end, margin_extra, lane_hard=False, relaxed=False):
        out = orig_dp(self, l0, rear_sl, cones_sl, s_end, margin_extra, lane_hard, relaxed)
        dp_calls.append({"l0": float(l0), "rear_sl": _f(rear_sl), "cones_sl": _f(np.asarray(cones_sl).reshape(-1, 2)),
                         "s_end": float(s_end), "margin_extra": float(margin_extra), "lane_hard": bool(lane_hard),
                         "relaxed": bool(relaxed), "l_des": [_f(self._l_des[0]), _f(self._l_des[1])],
                         "result": None if out is None else {"s": _f(out[0]), "l": _f(out[1])}})
        return out

    def roll_wrap(self, path_xy, cones_local, speed=1.0, steer=0.0):
        ok, gap = orig_roll(self, path_xy, cones_local, speed, steer)
        rollouts.append({"path": _f(path_xy), "cones_local": _f(np.asarray(cones_local).reshape(-1, 2)),
                         "speed": float(speed), "steer": float(steer), "ok": bool(ok),
                         "gap": None if gap is None else float(gap)})
        return ok, gap

    PL.ConeMemory.update, PL.ProposedPlanner._dp, PL.ProposedPlanner._rollout_ok = update_wrap, dp_wrap, roll_wrap
    PL.expected_visible = expected_wrap
    try:
        for name in scenarios:
            state["scenario"], state["count"] = name, 0
            run(SCENARIOS[name], "proposed", SimParams(), seed=seed, record=False)
    finally:
        PL.ConeMemory.update, PL.ProposedPlanner._dp, PL.ProposedPlanner._rollout_ok = orig_update, orig_dp, orig_roll
        PL.expected_visible = orig_expected
    return mem_steps, dp_calls, rollouts


def curated_dp_cases(P: SimParams):
    """손으로 만든 대표 입력 (지그재그 간격·침범, 유도선 벽, 빈 입력, 콘 1개)."""
    pl = PL.ProposedPlanner(P)
    cases = []

    def add(name, cones, lane_hard, relaxed=False, l0=0.0, rear=(-0.33, 0.0), extra=0.0, s_end=3.0):
        cones = np.asarray(cones, float).reshape(-1, 2)
        out = pl._dp(l0, rear, cones, s_end, extra, lane_hard, relaxed)
        cases.append({"name": name, "l0": l0, "rear_sl": list(rear), "cones_sl": _f(cones), "s_end": s_end,
                      "margin_extra": extra, "lane_hard": lane_hard, "relaxed": relaxed,
                      "l_des": [_f(pl._l_des[0]), _f(pl._l_des[1])],
                      "result": None if out is None else {"s": _f(out[0]), "l": _f(out[1])}})

    for spacing in (0.5, 0.6, 0.75):
        for prot in (0.18, 0.20):
            inner = 0.325 - prot + 0.065
            cones = []
            for k, side in enumerate((1, -1, 1, -1)):
                s = 1.0 + k * spacing
                cones += [[s, side * inner], [s, side * 0.34]]
            for hard, rel in ((True, False), (True, True), (False, False)):
                add(f"zigzag_{spacing}_{prot}_{'lane' if hard else 'free'}{'_relaxed' if rel else ''}", cones, hard, rel)
    wall = [[1.5, l] for l in np.arange(-0.3, 0.31, 0.12)]
    add("wall_lane", wall, True)
    add("wall_free", wall, False)
    add("empty_lane", [], True)
    add("single_left_lane", [[1.5, 0.21]], True)
    add("single_center_free", [[1.2, 0.0]], False)
    add("offset_start", [[1.5, -0.2]], True, l0=0.12, rear=(-0.33, 0.08))
    add("retry_margin", [[1.4, 0.15], [2.0, -0.15]], True, extra=0.03)
    return cases


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--seed", type=int, default=7)
    a = ap.parse_args(argv)
    os.makedirs(a.out, exist_ok=True)
    P = SimParams()
    mem, dps, rolls = record(seed=a.seed)
    rng = np.random.default_rng(0)
    pick = sorted(rng.choice(len(dps), size=min(30, len(dps)), replace=False).tolist())
    dp_cases = curated_dp_cases(P) + [{"name": f"sim_{i}", **dps[i]} for i in pick]
    oks = [r for r in rolls if r["ok"] and r["cones_local"]]
    fails = [r for r in rolls if not r["ok"]]
    roll_cases = [oks[i] for i in sorted(rng.choice(len(oks), size=min(20, len(oks)), replace=False))] + \
                 [fails[i] for i in sorted(rng.choice(len(fails), size=min(10, len(fails)), replace=False))]
    with open(os.path.join(a.out, "memory.json"), "w") as fh:
        json.dump({"params": _params(P), "steps": mem}, fh)
    with open(os.path.join(a.out, "planner.json"), "w") as fh:
        json.dump({"params": _params(P), "cases": dp_cases}, fh)
    with open(os.path.join(a.out, "rollout.json"), "w") as fh:
        json.dump({"params": _params(P), "cases": roll_cases}, fh)
    nn = sum(c["result"] is None for c in dp_cases)
    print(f"memory steps {len(mem)}, dp cases {len(dp_cases)} (None {nn}), rollout {len(roll_cases)} "
          f"(ok {sum(r['ok'] for r in roll_cases)}, fail {sum(not r['ok'] for r in roll_cases)})")


if __name__ == "__main__":
    main()
