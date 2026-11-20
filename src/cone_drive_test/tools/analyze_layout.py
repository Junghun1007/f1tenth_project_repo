#!/usr/bin/env python3
"""콘을 실측한 위치에 놓고 녹화한 주행으로 콘 검출 위치 오차를 재고 보정값(방향·거리 배율·거리 오프셋·시간)을 맞춘다.

  python3 tools/analyze_layout.py [--cones "0.5,0.26;1.5,-0.26;2.5,0.26;3.5,-0.26"] [--cam -0.08] <녹화 폴더>...

좌표: 출발 자세(녹화 첫 프레임, 앞차축) 기준 x 앞, y 왼쪽 (m). 출발 자세의 작은 어긋남(앞뒤·옆·방향)은 녹화마다 같이 맞춘다.
결과의 보정값은 지금 파라미터에 더할 값이다: object.bearing_offset_deg += 방향, object.range_gain *= (1 + 배율),
object.range_offset_m += 오프셋, object.time_offset_ms += 시간 (녹화 당시 값이 0·1·0·0이었다면 그대로 넣으면 된다).
2026-11-20 정답 배치(콘 4개) 녹화 4회: 방향 −2.4°, 배율 −2.9%, 오프셋 −4.6 cm, 시간 +15 ms → 정답과의 오차 6.5 → 4.5 cm.
"""
import csv, glob, math, os, sys, numpy as np
args = sys.argv[1:]
TRUTH = np.array([[0.5, 0.26], [1.5, -0.26], [2.5, 0.26], [3.5, -0.26]])
CAM = -0.09   # 녹화 당시 camera.x_m (관측 local은 이 값으로 계산됨)
while args and args[0].startswith('--'):
    if args[0] == '--cones': TRUTH = np.array([[float(v) for v in p.split(',')] for p in args[1].split(';')]); args = args[2:]
    elif args[0] == '--cam': CAM = float(args[1]); args = args[2:]
    else: print(__doc__); sys.exit(1)
if not args: print(__doc__); sys.exit(1)
runs = []
for d in args:
    F = list(csv.DictReader(open(d + '/frames.csv')))
    ft = np.array([float(f['t']) for f in F]); fx = np.array([float(f['pose_x']) for f in F]); fy = np.array([float(f['pose_y']) for f in F])
    fyaw = np.unwrap(np.array([float(f['pose_yaw']) for f in F]))
    O = [o for o in csv.DictReader(open(d + '/cone_obs.csv')) if int(float(o['class'])) == 0]
    ob = np.array([[float(o['t_pose']), float(o['local_x']), float(o['local_y']), float(o['yaw_rate']), float(o['speed'])] for o in O])
    runs.append(dict(name=os.path.basename(d.rstrip('/')), ft=ft, fx=fx, fy=fy, fyaw=fyaw, ob=ob, p0=(fx[0], fy[0], fyaw[0])))

def world(run, g, start):
    """관측 → 출발 자세 기준 좌표 → 배치 좌표. g = (α, k, b, τ, cam_dx), start = (x0, y0, ψ0)"""
    a, k, b, tau, cdx = g
    ob = run['ob']; t = ob[:, 0] + tau
    vx, vy = ob[:, 1] - CAM, ob[:, 2]
    r = np.hypot(vx, vy); th = np.arctan2(vy, vx)
    r2 = r * (1 + k) + b; th2 = th + a
    lx = CAM + cdx + r2 * np.cos(th2); ly = r2 * np.sin(th2)
    px = np.interp(t, run['ft'], run['fx']); py = np.interp(t, run['ft'], run['fy']); yaw = np.interp(t, run['ft'], run['fyaw'])
    wx = px + np.cos(yaw) * lx - np.sin(yaw) * ly; wy = py + np.sin(yaw) * lx + np.cos(yaw) * ly
    x0, y0, y0yaw = run['p0']
    c, s = math.cos(-y0yaw), math.sin(-y0yaw)
    qx, qy = c * (wx - x0) - s * (wy - y0), s * (wx - x0) + c * (wy - y0)   # 출발 자세 기준
    X0, Y0, P0 = start
    cp, sp_ = math.cos(P0), math.sin(P0)
    return np.stack([X0 + cp * qx - sp_ * qy, Y0 + sp_ * qx + cp * qy], 1), r

def residuals(x, free_g, assign):
    g = np.array([0.0, 0.0, 0.0, 0.0, 0.0]); g[free_g] = x[:len(free_g)]
    res = []
    for i, run in enumerate(runs):
        st = x[len(free_g) + 3 * i: len(free_g) + 3 * i + 3]
        W, _ = world(run, g, st)
        m = assign[i] >= 0
        res.append((W[m] - TRUTH[assign[i][m]]).ravel())
    return np.concatenate(res)

def fit(free_g, iters=4):
    x = np.zeros(len(free_g) + 3 * len(runs))
    for it in range(iters):
        g = np.zeros(5); g[free_g] = x[:len(free_g)]
        assign = []
        for i, run in enumerate(runs):
            W, r = world(run, g, x[len(free_g) + 3 * i: len(free_g) + 3 * i + 3])
            dist = np.linalg.norm(W[:, None, :] - TRUTH[None], axis=2)
            a = dist.argmin(1); a[dist.min(1) > 0.30] = -1
            assign.append(a)
        for _ in range(20):   # Gauss-Newton (수치 미분)
            r0 = residuals(x, free_g, assign)
            J = np.zeros((len(r0), len(x)))
            for j in range(len(x)):
                dx = np.zeros_like(x); dx[j] = 1e-5
                J[:, j] = (residuals(x + dx, free_g, assign) - r0) / 1e-5
            step = np.linalg.lstsq(J.T @ J + 1e-6 * np.eye(len(x)), -J.T @ r0, rcond=None)[0]
            x = x + step
            if np.abs(step).max() < 1e-7: break
    r0 = residuals(x, free_g, assign)
    n = sum((a >= 0).sum() for a in assign)
    return x, math.sqrt((r0 ** 2).sum() / n), assign

names = ['방향 α(°)', '거리 배율 k(%)', '거리 오프셋 b(cm)', '시간 τ(ms)', '카메라 x(cm)']
scale = [180 / math.pi, 100, 100, 1000, 100]
print('관측 수:', [len(r['ob']) for r in runs])
for label, free in [('출발 자세만', []), ('+ 방향', [0]), ('+ 방향·거리 배율·오프셋', [0, 1, 2]), ('+ 시간', [0, 1, 2, 3]), ('+ 카메라 x', [0, 1, 2, 3, 4])]:
    x, rms, assign = fit(free)
    g = ', '.join('%s %+.2f' % (names[j], x[i] * scale[j]) for i, j in enumerate(free))
    starts = ' '.join('(%.2f,%.2f,%.1f°)' % (x[len(free) + 3 * i], x[len(free) + 3 * i + 1], math.degrees(x[len(free) + 3 * i + 2])) for i in range(len(runs)))
    print('%-22s 정답과의 거리 RMS %.1f cm | %s | 출발 자세 %s' % (label, rms * 100, g or '-', starts))
    if free == [0, 1, 2, 3]: best = (free, x, assign)
# 거리 구간별, 회전 여부별 오차 (보정 전·후)
free, x, assign = best
for tag, gg in [('보정 전', np.zeros(5)), ('보정 후', None)]:
    if gg is None: gg = np.zeros(5); gg[free] = x[:len(free)]
    rows = []
    for i, run in enumerate(runs):
        xs = x[len(free) + 3 * i: len(free) + 3 * i + 3]
        W, r = world(run, gg, xs)
        m = assign[i] >= 0
        e = W[m] - TRUTH[assign[i][m]]
        rows.append(np.column_stack([r[m], np.linalg.norm(e, axis=1), e[:, 0], e[:, 1], np.abs(run['ob'][m, 3])]))
    R = np.vstack(rows)
    out = []
    for lo, hi in [(0.3, 1.0), (1.0, 2.0), (2.0, 3.0), (3.0, 4.5)]:
        m = (R[:, 0] >= lo) & (R[:, 0] < hi)
        if m.sum(): out.append('%.0f~%.0f m: %.1f cm (앞뒤 %+.1f, 옆 %+.1f)' % (lo, hi, np.sqrt((R[m, 1] ** 2).mean()) * 100, R[m, 2].mean() * 100, R[m, 3].mean() * 100))
    turn = R[:, 4] > 0.5
    print('%s | %s | 회전 중(>0.5 rad/s) %.1f cm, 직진 %.1f cm' % (tag, ' · '.join(out), np.sqrt((R[turn, 1] ** 2).mean()) * 100 if turn.any() else float('nan'), np.sqrt((R[~turn, 1] ** 2).mean()) * 100))

print('\n# 공정 비교: 각자 맞춘 출발 자세로')
for label, free in [('보정 전 (출발 자세만)', []), ('보정 후 (방향·거리·시간)', [0, 1, 2, 3])]:
    x, rms, assign = fit(free)
    gg = np.zeros(5); gg[free] = x[:len(free)]
    rows, first = [], []
    for i, run in enumerate(runs):
        W, r = world(run, gg, x[len(free) + 3 * i: len(free) + 3 * i + 3])
        a = assign[i]; m = a >= 0
        e = W[m] - TRUTH[a[m]]
        rows.append(np.column_stack([r[m], np.linalg.norm(e, axis=1)]))
        for k in range(4):   # 콘마다 처음 3번 관측 평균 = 처음 기억한 위치
            idx = np.where(a == k)[0][:3]
            if len(idx): first.append((r[idx].mean(), np.linalg.norm(W[idx].mean(0) - TRUTH[k])))
    R = np.vstack(rows); Fs = np.array(first)
    bins = ' · '.join('%.0f~%.0f m %.1f' % (lo, hi, np.sqrt((R[(R[:, 0] >= lo) & (R[:, 0] < hi), 1] ** 2).mean()) * 100) for lo, hi in [(0.3, 1), (1, 2), (2, 3), (3, 4.5)])
    print('%-24s 전체 %.1f cm | 거리별(cm) %s | 처음 기억 위치 오차 평균 %.1f cm (최대 %.1f, 처음 본 거리 평균 %.1f m)' % (label, rms * 100, bins, Fs[:, 1].mean() * 100, Fs[:, 1].max() * 100, Fs[:, 0].mean()))
