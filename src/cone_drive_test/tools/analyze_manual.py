#!/usr/bin/env python3
"""수동 주행 녹화(<log.dir>/manual/<시각>_<상황>/)로 회피 파라미터를 찾는다.

  python3 tools/analyze_manual.py ~/cone_drive_test_data/manual/2026-11-20_*_zigzag ~/cone_drive_test_data/manual/*_guided

사람이 부딪히지 않고 지나간 주행에서 다음을 잰다 (차체 = meta.json의 길이·폭·뒤 오버행 직사각형, 콘 = cone_size 사각형, 계획기와 같은 계산):
  - 콘마다 차체와 콘의 최소 빈틈               → 최소 여유(안전 하한)·권장 여유
  - 콘 옆을 지날 때 차체가 차선 밖으로 나간 양   → 차선 안 허용 이탈 / 차선 밖 최대 이탈
  - 콘을 지난 뒤 차선 가운데로 돌아오는 데 든 거리 → 차선 복귀 거리
  - 같은 순간 계획기가 낸 경로의 빈틈           → 계획기가 사람보다 붙는지 떨어지는지
"""
import csv, json, math, os, sys
import numpy as np

LANE_HALF = 0.325   # 계획기 차선 반폭 (frenet_planner.hpp)


def pts(s):
    if not s: return np.zeros((0, 2))
    return np.array([[float(a) for a in p.split()] for p in s.split(';') if p.strip()])


def load(run):
    meta = json.load(open(os.path.join(run, 'meta.json')))
    rows = list(csv.DictReader(open(os.path.join(run, 'frames.csv'))))
    F = []
    for r in rows:
        F.append(dict(t=float(r['t']), x=float(r['pose_x']), y=float(r['pose_y']), yaw=float(r['pose_yaw']),
                      v=float(r['speed']), mode=r['plan_mode'], left=pts(r['left']), right=pts(r['right']),
                      plan=pts(r['plan']), cones=pts(r['cones'])))
    return meta, F


def to_world(p, x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.stack([x + c * p[:, 0] - s * p[:, 1], y + s * p[:, 0] + c * p[:, 1]], 1)


def body_gap(cx, cy, yaw, hl, hw, cone, hc):
    """차체 직사각형(중심 cx,cy, 방향 yaw)과 콘 사각형 사이 빈틈 (frenet_planner와 같은 식, 음수 = 겹침)"""
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = cone[0] - cx, cone[1] - cy
    lx, ly = abs(c * dx + s * dy) - hl, abs(-s * dx + c * dy) - hw
    g = math.hypot(max(lx, 0), max(ly, 0)) if (lx > 0 or ly > 0) else max(lx, ly)
    return g - hc


def line_fit(P, lo=0.1, hi=0.8):
    if len(P) == 0: return None
    m = (P[:, 0] >= lo) & (P[:, 0] <= hi)
    return np.polyfit(P[m, 0], P[m, 1], 1) if m.sum() >= 4 else None


def cluster_cones(F):
    """프레임마다 기억 콘 목록 → 콘 하나당 가장 최근 위치 (10 cm 안은 같은 콘)"""
    out = []
    for f in F:
        for c in f['cones']:
            for o in out:
                if math.hypot(o[0] - c[0], o[1] - c[1]) < 0.10: o[0], o[1] = c[0], c[1]; break
            else: out.append([c[0], c[1]])
    return np.array(out)


def analyze(run):
    meta, F = load(run)
    L, W, ro, wb, cs = meta['vehicle_length'], meta['vehicle_width'], meta['rear_overhang'], meta['wheelbase'], meta['cone_size']
    hl, hw, hc = L / 2, W / 2, cs / 2
    off = L / 2 - ro - wb   # 앞차축 → 차체 중심 (음수)
    F = [f for f in F if f['v'] > 0.1]
    if len(F) < 20: return None
    cones = cluster_cones(F)
    # 주행 거리 (앞차축)
    xy = np.array([[f['x'], f['y']] for f in F]); s_car = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(xy, axis=0).T))])
    # 프레임마다: 차선 가운데 대비 앞차축 옆 위치, 차체 차선 이탈량
    lat = np.full(len(F), np.nan); excess = np.full(len(F), np.nan)
    corners = np.array([[off + hl, hw], [off + hl, -hw], [off - hl, hw], [off - hl, -hw]])
    for i, f in enumerate(F):
        a, b = line_fit(f['left']), line_fit(f['right'])
        if a is None or b is None or not (0.3 < (a[1] - b[1]) < 1.0): continue
        mid = lambda x: 0.5 * (np.polyval(a, x) + np.polyval(b, x))
        lat[i] = -mid(0.0)                        # + = 차가 차선 가운데보다 왼쪽
        excess[i] = max(abs(cy - mid(cx)) for cx, cy in corners) - LANE_HALF
    res = []
    for k, c in enumerate(cones):
        g = np.array([body_gap(f['x'] + math.cos(f['yaw']) * off, f['y'] + math.sin(f['yaw']) * off, f['yaw'], hl, hw, c, hc) for f in F])
        i = int(np.argmin(g))
        if g[i] > 0.5: continue                   # 옆을 지나지 않은 콘
        # 사람이 지난 위치 기준 앞뒤 거리 (콘의 차량 기준 x가 0이 되는 순간)
        loc_x = np.array([math.cos(f['yaw']) * (c[0] - f['x']) + math.sin(f['yaw']) * (c[1] - f['y']) for f in F])
        near = np.abs(loc_x) < 1.5
        ex = np.nanmax(excess[near]) if np.isfinite(excess[near]).any() else np.nan
        # 계획기 경로의 빈틈: 콘이 0.5~2.0 m 앞에 있을 때 그 순간 계획 경로
        pg = []
        for j, f in enumerate(F):
            if not (0.5 < loc_x[j] < 2.0) or len(f['plan']) < 3: continue
            P = to_world(f['plan'], f['x'], f['y'], f['yaw'])
            hd = np.arctan2(np.gradient(P[:, 1]), np.gradient(P[:, 0]))
            pg.append(min(body_gap(P[q, 0] + math.cos(hd[q]) * off, P[q, 1] + math.sin(hd[q]) * off, hd[q], hl, hw, c, hc) for q in range(len(P))))
        # 복귀 거리: 콘을 지난(차량 기준 x < −뒤 오버행·축간거리) 뒤 옆 위치가 기준(콘 없는 구간 중앙값) ±3 cm로 돌아올 때까지
        base = np.nanmedian(lat[np.abs(loc_x) > 2.5]) if np.isfinite(lat[np.abs(loc_x) > 2.5]).any() else 0.0
        passed = np.where(loc_x < -(wb + ro))[0]
        ret = np.nan
        if len(passed):
            p0 = passed[0]
            back = np.where(np.abs(lat[p0:] - base) < 0.03)[0]
            if len(back): ret = s_car[p0 + back[0]] - s_car[p0]
        res.append(dict(cone=k, gap=g[i], excess=ex, plan_gap=np.median(pg) if pg else np.nan, ret=ret,
                        mode=F[i]['mode'], v=F[i]['v']))
    modes = {}
    for f in F: modes[f['mode']] = modes.get(f['mode'], 0) + 1
    return meta, res, modes


def pct(a, q):
    a = np.array([x for x in a if np.isfinite(x)])
    return float(np.percentile(a, q)) if len(a) else float('nan')


def main(runs):
    allr = []
    for run in runs:
        out = analyze(run)
        if out is None: print('%s: 주행 프레임이 모자람' % run); continue
        meta, res, modes = out
        print('\n== %s (%s) 콘 %d개 · 계획 모드 %s' % (os.path.basename(run.rstrip('/')), meta['label'], len(res),
              ', '.join('%s %d%%' % (k, 100 * v // max(1, sum(modes.values()))) for k, v in modes.items())))
        print('   콘  사람 최소 빈틈  차체 차선 이탈  계획기 빈틈  복귀 거리  속도')
        for r in res:
            print('   %2d    %5.1f cm      %+5.1f cm     %5.1f cm   %5.2f m   %.2f m/s' % (r['cone'], r['gap'] * 100, r['excess'] * 100,
                  r['plan_gap'] * 100, r['ret'], r['v']))
            r['label'] = meta['label']
        allr += res
    if not allr: return
    gap = [r['gap'] for r in allr]
    inlane = [r for r in allr if np.isfinite(r['excess']) and r['excess'] < 0.10]
    outlane = [r for r in allr if np.isfinite(r['excess']) and r['excess'] >= 0.10]
    pgap = [r['plan_gap'] for r in allr]
    print('\n== 전체 (콘 %d개 통과, 차선 안 %d / 차선 밖 %d)' % (len(allr), len(inlane), len(outlane)))
    print('   사람 최소 빈틈: 최소 %.1f, 10%% %.1f, 중앙값 %.1f cm' % (min(gap) * 100, pct(gap, 10) * 100, pct(gap, 50) * 100))
    print('   계획기 빈틈:   최소 %.1f, 중앙값 %.1f cm (사람보다 작으면 계획기가 더 붙음)' % (np.nanmin(pgap) * 100 if np.isfinite(pgap).any() else float('nan'), pct(pgap, 50) * 100))
    floor = max(0.01, round(0.5 * pct(gap, 10), 2))
    soft_in = pct([r['gap'] for r in inlane], 50); soft_out = pct([r['gap'] for r in outlane], 50)
    tol = pct([r['excess'] for r in inlane], 90); exmax = max([r['excess'] for r in outlane], default=float('nan'))
    ret = pct([r['ret'] for r in allr], 50)
    print('\n== 추천 (사람이 안전하게 지나간 값 기준)')
    print('   최소 여유 ①②③ (안전 하한)     %.2f m   ← 사람 빈틈 하위 10%%의 절반' % floor)
    if np.isfinite(soft_in): print('   ① 차선 안 권장 여유            %.2f m   ← 차선 안에서 지나간 빈틈 중앙값' % soft_in)
    if np.isfinite(soft_in): print('   ② 여유 축소 권장 여유          %.2f m   ← 하한과 ① 사이' % ((floor + soft_in) / 2))
    if np.isfinite(soft_out): print('   ③ 차선 밖 권장 여유            %.2f m   ← 차선 밖으로 지나간 빈틈 중앙값' % soft_out)
    if np.isfinite(tol): print('   차선 안 허용 이탈              %.2f m   ← 차선 안 통과의 차체 이탈 90%%' % max(0.0, tol))
    if np.isfinite(exmax): print('   차선 밖 최대 이탈              %.2f m   ← 차선 밖 통과의 최대 이탈 + 5 cm' % (exmax + 0.05))
    if np.isfinite(ret): print('   차선 복귀 거리                 %.2f m   ← 콘을 지난 뒤 가운데로 돌아온 거리 중앙값' % ret)


if __name__ == '__main__':
    if len(sys.argv) < 2: print(__doc__); sys.exit(1)
    main(sys.argv[1:])
