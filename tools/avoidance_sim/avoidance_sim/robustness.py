"""과적합 점검: 튜닝에 쓰지 않은 무작위 배치로 검증.

아래쪽 직선(21~29번, 진행 +x)에 지그재그(간격·폭 무작위), 유도선(좌/우, 크기·길이 무작위),
차선 가장자리 콘을 무작위로 놓고 시드 10번대로 돌린다.

    python -m avoidance_sim.robustness [난수시드]
"""
import sys, dataclasses, numpy as np
from avoidance_sim.sim import run
from avoidance_sim.scenarios import Scenario
from avoidance_sim.track import Track
tr = Track()
VIEW = (0.5, 10.5, -0.1, 2.2)

def slalom(rng):
    sp = rng.uniform(0.6, 1.0); gap = rng.uniform(0.50, 0.60); n = rng.integers(3, 6)
    first_side = rng.choice([-1, 1]); x0 = rng.uniform(3.0, 4.0)
    prot = 0.65 - gap; inner = 0.325 - prot + 0.065
    cones = []
    for k in range(n):
        side = first_side * (-1) ** k; x = x0 + k * sp + rng.normal(0, 0.02)
        cones += [[x, 0.5 + side * max(0.34, inner + 0.13)], [x, 0.5 + side * inner + rng.normal(0, 0.01)]]
    return np.array(cones), f"slalom sp={sp:.2f} gap={gap:.2f} n={n}", "lane"

def corridor(rng):
    amp = rng.uniform(0.5, 0.75) * rng.choice([-1, 1]); length = rng.uniform(2.6, 3.6); x0 = rng.uniform(3.0, 4.0)
    u = np.linspace(0, length, 4001); l = amp / 2 * (1 - np.cos(2 * np.pi * u / length))
    arc = np.concatenate([[0], np.cumsum(np.hypot(np.diff(u), np.diff(l)))])
    m = int(round(arc[-1] / 0.186)) + 1
    uu = np.interp(np.linspace(0, arc[-1], m), arc, u); ll = amp / 2 * (1 - np.cos(2 * np.pi * uu / length))
    dl = amp / 2 * (2 * np.pi / length) * np.sin(2 * np.pi * uu / length)
    nrm = np.hypot(1, dl); nx, ny = -dl / nrm, 1 / nrm
    cx, cy = x0 + uu, 0.5 + ll
    cones = np.concatenate([np.stack([cx + 0.34 * nx, cy + 0.34 * ny], 1), np.stack([cx - 0.34 * nx, cy - 0.34 * ny], 1)])
    return cones, f"corridor amp={amp:+.2f} len={length:.2f}", "free"

def edge_cones(rng):
    # 차선 가장자리 바깥/테이프 위 콘: 차선을 벗어날 이유가 없음
    xs = np.sort(rng.uniform(3, 8, rng.integers(3, 7)))
    ys = 0.5 + rng.choice([-1, 1], len(xs)) * rng.uniform(0.40, 0.55, len(xs))
    return np.stack([xs, ys], 1), f"edge n={len(xs)}", "lane"

def make(cones):
    return Scenario("custom", 0.8, 9.0, 1.0, "custom", VIEW, custom_cones=cones)

if __name__ == "__main__":
    rng = np.random.default_rng(int(sys.argv[1]) if len(sys.argv) > 1 else 123)
    rows = []
    for gen in (slalom, corridor, edge_cones):
        for k in range(6):
            cones, desc, expect = gen(rng)
            sc = make(cones)
            for pl in ("baseline", "proposed", "proposed_lane"):
                r = run(sc, pl, seed=10 + k, record=True)
                # 차선 이탈량 (아래쪽 직선 구간 전체)
                ex = 0.0
                for f in r.frames:
                    ex = max(ex, max(abs(tr.project(c)[1]) for c in f.footprint) - 0.325)
                m = r.metrics
                rows.append((gen.__name__, pl, m["contacts"], m["min_clearance_m"], round(ex, 3), m["finished"]))
                print(f"{desc:34s} {pl:9s} 접촉 {m['contacts']} 여유 {m['min_clearance_m']} 차선밖 {ex:+.3f} 완주 {m['finished']}", flush=True)
    print("\n요약 (각 6개 배치)")
    for g in ("slalom", "corridor", "edge_cones"):
        for pl in ("baseline", "proposed"):
            rr = [x for x in rows if x[0] == g and x[1] == pl]
            print(f"{g:11s} {pl:9s} 무접촉 {sum(x[2]==0 for x in rr)}/6  평균접촉 {np.mean([x[2] for x in rr]):.1f}  "
                  f"최대 차선밖 {max(x[4] for x in rr):+.2f}  완주 {sum(x[5] for x in rr)}/6")
