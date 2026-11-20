#!/usr/bin/env python3
"""콘 관측 기록(cone_obs_*.csv)으로 '처음 기억한 위치와 최근 위치가 틀어지는' 원인을 가린다.

  python3 tools/analyze_cone_obs.py ~/auto_control_data/cone_obs_2026-11-20_130000.csv

같은 콘(id)을 여러 번 본 기록에서, 가장 가까이서 본 관측(카메라 1 m 안)의 평균을 그 콘의 기준 위치로 두고
각 관측의 어긋남을 시선 방향(거리)과 옆 방향으로 나눠 무엇을 따라 변하는지 본다.
  - 거리 방향 어긋남이 거리에 비례   → 스테레오 거리 치우침 (기울기 = 거리 배율 오차)
  - 옆 방향 어긋남이 거리×요레이트에 비례 → 시각 어긋남 (기울기 = 시간차 s)
  - 거리 방향 어긋남이 속도에 비례   → 시각 어긋남 (앞뒤)
  - 처음 본 위치 − 마지막 위치 (콘마다)  → 지금 화면에서 보이는 차이 그 자체
"""
import csv, math, sys
import numpy as np

def load(path):
    rows = list(csv.DictReader(open(path)))
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0].keys()} if rows else None

def fit(x, y):
    if len(x) < 10 or np.ptp(x) < 1e-6: return float('nan'), float('nan')
    A = np.vstack([x, np.ones_like(x)]).T
    (k, c), *_ = np.linalg.lstsq(A, y, rcond=None)
    return k, c

def main(path, cam_x=-0.09):
    d = load(path)
    if d is None: print('기록이 비어 있음'); return
    ids = d['id'].astype(int)
    cones = d['class'] == 0
    er_all, et_all, rng_all, rw_all, spd_all = [], [], [], [], []
    gaps = []
    for i in np.unique(ids[cones]):
        s = (ids == i) & cones
        if s.sum() < 8: continue
        o = np.argsort(d['t_rgb'][s])
        wx, wy = d['world_x'][s][o], d['world_y'][s][o]
        px, py, yaw = d['pose_x'][s][o], d['pose_y'][s][o], d['pose_yaw'][s][o]
        cx = px + cam_x * np.cos(yaw); cy = py + cam_x * np.sin(yaw)          # 카메라 위치 (월드)
        r = np.hypot(wx - cx, wy - cy)
        near = r < 1.0
        if near.sum() < 3: continue
        ref = np.array([wx[near].mean(), wy[near].mean()])
        ux, uy = (wx - cx) / r, (wy - cy) / r                                # 시선 방향 (월드)
        dx, dy = wx - ref[0], wy - ref[1]
        er = dx * ux + dy * uy                                               # + = 실제보다 멀게
        et = -dx * uy + dy * ux                                              # + = 시선 기준 왼쪽
        er_all += list(er); et_all += list(et); rng_all += list(r)
        rw_all += list(r * d['yaw_rate'][s][o]); spd_all += list(d['speed'][s][o])
        k0 = min(3, len(wx))
        first = np.array([wx[:k0].mean(), wy[:k0].mean()]); last = np.array([wx[-k0:].mean(), wy[-k0:].mean()])
        gaps.append((i, r[:k0].mean(), r[-k0:].mean(), np.linalg.norm(first - last) * 100, er[:k0].mean() * 100, et[:k0].mean() * 100, s.sum()))
    if not gaps: print('가까이(1 m 안)까지 여러 번 본 콘이 없음: 콘 옆을 지나가도록 달려 주세요'); return
    er, et, rng, rw, spd = map(np.array, (er_all, et_all, rng_all, rw_all, spd_all))
    print('콘 %d개, 관측 %d개' % (len(gaps), len(er)))
    print('\n[콘마다] 처음 본 위치와 가까이서 본 위치의 차이')
    print('  id   처음 거리  마지막 거리   차이     (거리 방향 / 옆 방향)  관측 수')
    for g in gaps: print('  %3d   %5.2f m    %5.2f m   %5.1f cm   (%+5.1f / %+5.1f cm)   %d' % g)
    gap = np.array([g[3] for g in gaps])
    print('  평균 차이 %.1f cm, 최대 %.1f cm' % (gap.mean(), gap.max()))
    k_r, c_r = fit(rng, er)
    print('\n[원인별]')
    print('  거리 방향 어긋남 = %.1f cm/m × 거리 %+.1f cm   → 스테레오 거리 배율 오차 약 %+.1f%%' % (k_r * 100, c_r * 100, k_r * 100))
    tau, k_best, a_best, sc0, sc1 = search_time_and_scale(d, cam_x)
    print('  가장 잘 모이는 보정: 방향 %+.2f° (object.bearing_offset_deg에 더할 값), 시간 %+.0f ms, 거리 배율 %+.1f%% → 같은 콘 위치의 퍼짐 %.1f → %.1f cm'
          % (a_best, tau * 1000, k_best * 100, sc0 * 100, sc1 * 100))
    print('     (방향 + = 관측을 왼쪽으로 돌림, 시간 + = 영상이 기록된 시각보다 늦은 순간의 장면)')
    print('\n[판단 기준] 거리 배율 오차가 ±1%% 넘거나 시간차가 ±10 ms 넘고, 퍼짐이 눈에 띄게 줄면 그 항목을 보정할 가치가 있다')


def search_time_and_scale(d, cam_x):
    """기록된 자세들로 궤적을 만들고, 시간차 τ와 거리 배율 k를 바꿔 가며 같은 콘 관측들이 가장 모이는 값을 찾는다"""
    t = d['t_pose']; o = np.argsort(t)
    tt, ux = np.unique(t[o], return_index=True)
    PX, PY = d['pose_x'][o][ux], d['pose_y'][o][ux]
    PYAW = np.unwrap(d['pose_yaw'][o][ux])
    sel = d['class'] == 0
    ids = d['id'][sel].astype(int)
    lx, ly, tp = d['local_x'][sel], d['local_y'][sel], d['t_pose'][sel]
    keep = np.isin(ids, [i for i in np.unique(ids) if (ids == i).sum() >= 8])
    ids, lx, ly, tp = ids[keep], lx[keep], ly[keep], tp[keep]
    rx, ry = lx - cam_x, ly
    def spread(tau, k, a=0.0):
        px = np.interp(tp + tau, tt, PX); py = np.interp(tp + tau, tt, PY); yaw = np.interp(tp + tau, tt, PYAW)
        ca, sa = math.cos(a), math.sin(a)
        sx, sy = cam_x + (ca * rx - sa * ry) / (1 + k), (sa * rx + ca * ry) / (1 + k)
        wx = px + np.cos(yaw) * sx - np.sin(yaw) * sy; wy = py + np.sin(yaw) * sx + np.cos(yaw) * sy
        tot, n = 0.0, 0
        for i in np.unique(ids):
            m = ids == i
            tot += ((wx[m] - wx[m].mean()) ** 2 + (wy[m] - wy[m].mean()) ** 2).sum(); n += m.sum()
        return math.sqrt(tot / max(1, n))
    s0 = spread(0.0, 0.0)
    best = (s0, 0.0, 0.0, 0.0)
    for a in np.arange(-6.0, 6.01, 0.5):       # 먼저 방향·거리 (시간 0)
        for k in np.arange(-0.06, 0.0601, 0.01):
            v = spread(0.0, k, math.radians(a))
            if v < best[0]: best = (v, 0.0, k, a)
    _, _, k0, a0 = best
    for tau in np.arange(-0.08, 0.0801, 0.005):   # 그 근처에서 시간까지 같이
        for da in (-0.5, -0.25, 0.0, 0.25, 0.5):
            for dk in (-0.01, -0.005, 0.0, 0.005, 0.01):
                v = spread(tau, k0 + dk, math.radians(a0 + da))
                if v < best[0]: best = (v, tau, k0 + dk, a0 + da)
    return best[1], best[2], best[3], s0, best[0]


if __name__ == '__main__':
    if len(sys.argv) < 2: print(__doc__); sys.exit(1)
    main(sys.argv[1])
