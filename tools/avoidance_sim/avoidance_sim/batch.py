"""배치 실행: 시나리오 × 알고리즘 × 시드 → CSV + 요약 표.

예)
  python -m avoidance_sim.batch --scenario stage1_cones stage2_guide --planner baseline proposed --seeds 10
  python -m avoidance_sim.batch --scenario stage1_cones --stage1-gap 0.47 0.53 --seeds 5 --csv out.csv
"""
import argparse
import csv
import dataclasses
import sys

from .config import SimParams
from .scenarios import SCENARIOS
from .sim import run

FIELDS = ["scenario", "stage1_gap", "planner", "seed", "finished", "contacts", "contact_parts", "min_clearance_m",
          "stage1_lane_excess_m", "time_s", "avg_speed", "max_steer_deg", "plan_ms_avg", "plan_ms_max"]


def main(argv=None):
    ap = argparse.ArgumentParser(description="라바콘 회피 배치 실행")
    ap.add_argument("--scenario", nargs="+", default=["stage1_cones", "stage2_guide"], choices=sorted(SCENARIOS))
    ap.add_argument("--planner", nargs="+", default=["baseline", "proposed"], choices=["baseline", "proposed", "proposed_lane"])
    ap.add_argument("--seeds", type=int, default=5)
    ap.add_argument("--stage1-gap", type=float, nargs="+")
    ap.add_argument("--no-noise", action="store_true")
    ap.add_argument("--csv", default="batch_results.csv")
    args = ap.parse_args(argv)

    rows = []
    for name in args.scenario:
        base = SCENARIOS[name]
        gaps = args.stage1_gap if (args.stage1_gap and base.obstacles in ("stage1", "both")) else [base.stage1_gap]
        for gap in gaps:
            sc = dataclasses.replace(base, stage1_gap=gap)
            for pl in args.planner:
                for seed in range(args.seeds):
                    P = SimParams()
                    P.perception.noise = not args.no_noise
                    m = run(sc, pl, P, seed=seed, record=False).metrics
                    row = {"scenario": name, "stage1_gap": gap, "planner": pl, "seed": seed}
                    row.update({k: m.get(k) for k in FIELDS if k in m})
                    rows.append(row)
                    print(f"{name:13s} gap={gap:.2f} {pl:9s} seed={seed:2d} 접촉={m['contacts']} "
                          f"최소여유={m['min_clearance_m']} 완주={m['finished']}", flush=True)
    with open(args.csv, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=FIELDS)
        w.writeheader()
        w.writerows(rows)

    print("\n요약 (평균)")
    print(f"{'시나리오':13s} {'폭':>5s} {'알고리즘':9s} {'접촉없음':>7s} {'평균접촉':>7s} {'최소여유':>8s} {'완주':>5s}")
    keys = sorted({(r["scenario"], r["stage1_gap"], r["planner"]) for r in rows})
    for k in keys:
        g = [r for r in rows if (r["scenario"], r["stage1_gap"], r["planner"]) == k]
        clean = sum(r["contacts"] == 0 for r in g)
        avg_c = sum(r["contacts"] for r in g) / len(g)
        gaps_ = [r["min_clearance_m"] for r in g if r["min_clearance_m"] is not None]
        mg = min(gaps_) if gaps_ else float("nan")
        fin = sum(bool(r["finished"]) for r in g)
        print(f"{k[0]:13s} {k[1]:5.2f} {k[2]:9s} {clean:>4d}/{len(g):<2d} {avg_c:7.2f} {mg:8.3f} {fin:>3d}/{len(g)}")
    print(f"\nCSV: {args.csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
