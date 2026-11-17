"""1단계 지그재그 파라미터 탐색.

배치: 1단계 위치(7번 진입 x=10.75부터), 왼·오·왼·오 4묶음.
      묶음 간격 spacing, 차선 안쪽 끝에서 침범 길이 protrusion (바깥 콘은 테이프 중심, 안쪽 콘이 침범).
파라미터 격자의 모든 조합 × 모든 배치 × 시드를 병렬로 돌려 CSV와 순위표를 만든다.

    python -m avoidance_sim.sweep --out sweep.csv                 # 기본 격자
    python -m avoidance_sim.sweep --quick                         # 빠른 확인 (작은 격자)
"""
import argparse
import csv
import itertools
import json
import math
import multiprocessing as mp
import sys
import time

import numpy as np

from .config import SimParams, apply_overrides
from .scenarios import Scenario
from .sim import run
from .track import Track, LANE_HALF_TAPE, LANE_HALF_INNER, CONE_SIZE

_TR = Track()
FIRST_X = 10.75
END_X = 7.3                         # 마지막 묶음(최대 x=8.35)을 차 전체가 지난 뒤
START_S = 13.0                      # 6번 코너 진입 전 (기존 stage1_cones와 같음)

DEFAULT_GRID = {
    "control.obstacle_speed": [0.6, 0.8, 1.0],
    "proposed.soft_clearance": [0.06, 0.10, 0.14],
    "control.stanley_gain": [1.0, 1.3, 1.6],
    "control.steering_current_weight": [0.4, 0.6],
}
QUICK_GRID = {"control.obstacle_speed": [0.6, 1.0], "proposed.soft_clearance": [0.10]}
SPACINGS = [0.60, 0.65, 0.70, 0.75, 0.80]
PROTRUSIONS = [0.18, 0.19, 0.20]


def zigzag(spacing, protrusion, first_x=FIRST_X):
    """윗줄(진행 −x, 왼쪽 = 안쪽 −y) 왼·오·왼·오 4묶음."""
    inner = LANE_HALF_INNER - protrusion + CONE_SIZE / 2          # 중심선 → 안쪽 콘 중심
    outer = max(LANE_HALF_TAPE, inner + CONE_SIZE)                # 바깥 콘은 테이프 위 (겹치면 바깥으로)
    cones = []
    for k, side in enumerate(("L", "R", "L", "R")):
        x = first_x - k * spacing
        sign = -1.0 if side == "L" else 1.0
        cones += [(x, 3.5 + sign * outer), (x, 3.5 + sign * inner)]
    return np.array(cones)


def scenario(spacing, protrusion):
    end_s = _TR.project((END_X, 3.5), s_hint=20.0)[0]
    return Scenario(f"zigzag_{spacing:.2f}_{protrusion:.2f}", START_S, end_s - START_S, 1.0, "stage1",
                    (7.0, 12.9, 1.8, 4.1), custom_cones=zigzag(spacing, protrusion))


def _job(args):
    combo, spacing, protrusion, seed, planner = args
    P = SimParams()
    over = {}
    for key, val in combo.items():
        group, name = key.split(".")
        over.setdefault(group, {})[name] = val
    apply_overrides(P, over)
    t0 = time.perf_counter()
    m = run(scenario(spacing, protrusion), planner, P, seed=seed, record=False).metrics
    return {**{k: v for k, v in combo.items()}, "planner": planner, "spacing": spacing, "protrusion": protrusion,
            "seed": seed, "contacts": m["contacts"], "min_clearance_m": m["min_clearance_m"],
            "lane_excess_m": m["stage1_lane_excess_m"], "finished": m["finished"], "time_s": m["time_s"],
            "wall_s": round(time.perf_counter() - t0, 2)}


def summarize(rows, keys):
    groups = {}
    for r in rows:
        groups.setdefault(tuple(r[k] for k in keys), []).append(r)
    table = []
    for k, g in groups.items():
        n = len(g)
        clean = sum(r["contacts"] == 0 and r["finished"] and (r["lane_excess_m"] or 0) <= 0.03 for r in g)
        gaps = [r["min_clearance_m"] for r in g if r["min_clearance_m"] is not None]
        table.append({**dict(zip(keys, k)), "runs": n, "clean_rate": clean / n,
                      "contacts_avg": float(np.mean([r["contacts"] for r in g])),
                      "min_clear_p10": float(np.percentile(gaps, 10)) if gaps else float("nan"),
                      "min_clear_avg": float(np.mean(gaps)) if gaps else float("nan"),
                      "lane_excess_max": max((r["lane_excess_m"] or 0) for r in g),
                      "time_avg": float(np.mean([r["time_s"] for r in g if r["finished"]] or [math.nan]))})
    table.sort(key=lambda r: (-r["clean_rate"], -r["min_clear_p10"], r["time_avg"]))
    return table


def main(argv=None):
    ap = argparse.ArgumentParser(description="지그재그 파라미터 탐색")
    ap.add_argument("--grid", help='JSON, 예: {"control.obstacle_speed": [0.6, 0.8]}')
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--seeds", type=int, default=4)
    ap.add_argument("--seed-base", type=int, default=100)
    ap.add_argument("--planner", default="proposed", choices=["baseline", "proposed"])
    ap.add_argument("--workers", type=int, default=mp.cpu_count())
    ap.add_argument("--out", default="sweep.csv")
    args = ap.parse_args(argv)

    grid = json.loads(args.grid) if args.grid else (QUICK_GRID if args.quick else DEFAULT_GRID)
    keys = list(grid)
    combos = [dict(zip(keys, vals)) for vals in itertools.product(*grid.values())]
    jobs = [(c, sp, pr, args.seed_base + s, args.planner)
            for c in combos for sp in SPACINGS for pr in PROTRUSIONS for s in range(args.seeds)]
    print(f"조합 {len(combos)}개 × 배치 {len(SPACINGS) * len(PROTRUSIONS)}개 × 시드 {args.seeds} = {len(jobs)}회", flush=True)
    rows = []
    t0 = time.time()
    with mp.Pool(args.workers) as pool, open(args.out, "w", newline="", encoding="utf-8") as fh:
        writer = None
        for k, row in enumerate(pool.imap_unordered(_job, jobs, chunksize=2), 1):
            rows.append(row)
            if writer is None:
                writer = csv.DictWriter(fh, fieldnames=list(row))
                writer.writeheader()
            writer.writerow(row)
            fh.flush()
            if k % 20 == 0 or k == len(jobs):
                el = time.time() - t0
                print(f"  {k}/{len(jobs)}  경과 {el / 60:.1f}분  남은 시간 약 {el / k * (len(jobs) - k) / 60:.1f}분", flush=True)

    print("\n=== 파라미터 조합 순위 (무접촉률 → 하위10% 최소여유 → 시간) ===")
    for r in summarize(rows, keys)[:15]:
        desc = "  ".join(f"{k.split('.')[1]}={r[k]}" for k in keys)
        print(f"{desc:80s} 무접촉 {r['clean_rate'] * 100:5.1f}%  평균접촉 {r['contacts_avg']:.2f}  "
              f"여유p10 {r['min_clear_p10'] * 100:+.1f}cm  차선밖 {r['lane_excess_max']:.2f}  시간 {r['time_avg']:.2f}s")
    best = summarize(rows, keys)[0]
    best_rows = [r for r in rows if all(r[k] == best[k] for k in keys)]
    print("\n=== 최적 조합의 배치별 결과 (무접촉률) ===")
    print("간격\\침범  " + "  ".join(f"{p * 100:.0f}cm" for p in PROTRUSIONS))
    for sp in SPACINGS:
        cells = []
        for pr in PROTRUSIONS:
            g = [r for r in best_rows if r["spacing"] == sp and r["protrusion"] == pr]
            cells.append(f"{sum(r['contacts'] == 0 for r in g)}/{len(g)}")
        print(f"{sp * 100:.0f}cm      " + "  ".join(f"{c:>4s}" for c in cells))
    print(f"\nCSV: {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
