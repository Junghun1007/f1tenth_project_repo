"""시뮬 실행 + GUI.

예)
  python -m avoidance_sim.run --scenario stage1_cones --planner proposed
  python -m avoidance_sim.run --scenario stage1_cones --planner baseline proposed   # 위아래 비교
  python -m avoidance_sim.run --scenario stage2_guide --planner proposed --save out.gif
  python -m avoidance_sim.run --scenario stage1_cones --no-noise
  python -m avoidance_sim.run --scenario stage1_cones --stage1-gap 0.53   # 지그재그 폭 넓히기
"""
import argparse
import json
import sys

from .config import SimParams, apply_overrides
from .scenarios import SCENARIOS
from .sim import run


def main(argv=None):
    ap = argparse.ArgumentParser(description="라바콘 회피 시뮬레이터")
    ap.add_argument("--scenario", default="stage1_cones", choices=sorted(SCENARIOS))
    ap.add_argument("--planner", nargs="+", default=["proposed"], choices=["baseline", "proposed", "proposed_lane"],
                    help="baseline=현재 실차(기억 없음), proposed=월드 기억, proposed_lane=차선 기준 단기 기억")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--stage1-gap", type=float, help="1단계 콘 지점에서 남는 차선 폭 (규격 0.47 m)")
    ap.add_argument("--stage1-spacing", type=float, help="1단계 콘 묶음 사이 앞뒤 간격 (기본 0.66 m)")
    ap.add_argument("--layout", help="에디터에서 저장한 콘 배치 JSON (시나리오 구간은 --scenario)")
    ap.add_argument("--no-noise", action="store_true", help="인지·오도메트리 노이즈 OFF (가림은 유지)")
    ap.add_argument("--config", help="파라미터 덮어쓰기 YAML (예: perception: {latency: 0.08})")
    ap.add_argument("--save", help="GUI 대신 .gif/.mp4로 저장")
    ap.add_argument("--snapshot", type=float, help="이 시각(s)의 정지 화면을 PNG로 저장 (--save 경로 사용)")
    ap.add_argument("--headless", action="store_true", help="지표만 출력")
    args = ap.parse_args(argv)

    import dataclasses
    scenario = SCENARIOS[args.scenario]
    if args.stage1_gap is not None:
        scenario = dataclasses.replace(scenario, stage1_gap=args.stage1_gap)
    if args.stage1_spacing is not None:
        scenario = dataclasses.replace(scenario, stage1_spacing=args.stage1_spacing)
    if args.layout:
        from .scenarios import with_layout, load_layout
        scenario = with_layout(scenario, load_layout(args.layout))
    results = []
    for name in args.planner:
        P = SimParams()
        if args.config:
            import yaml
            with open(args.config, encoding="utf-8") as fh:
                apply_overrides(P, yaml.safe_load(fh) or {})
        if args.no_noise:
            P.perception.noise = False
        r = run(scenario, name, P, seed=args.seed, record=not args.headless)
        m = {k: v for k, v in r.metrics.items() if k != "contact_detail"}
        print(f"[{name}] {json.dumps(m, ensure_ascii=False)}")
        for c in r.metrics["contact_detail"]:
            print(f"    접촉 t={c['t']}s 콘#{c['cone']} 부위={c['part']}")
        results.append(r)
    if args.headless:
        return 0

    if args.save or args.snapshot is not None:
        import matplotlib
        matplotlib.use("Agg")
    from .viewer import Viewer
    title = f"{args.scenario}  seed={args.seed}" + (f"  1단계 남는 폭={scenario.stage1_gap:.2f} m, 묶음 간격={scenario.stage1_spacing:.2f} m" if scenario.obstacles in ("stage1", "both") else "") + ("  (노이즈 OFF)" if args.no_noise else "")
    v = Viewer(results, title)
    if args.snapshot is not None:
        v.snapshot(args.save or "snapshot.png", args.snapshot)
    elif args.save:
        v.save(args.save)
        print(f"saved {args.save}")
    else:
        v.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
