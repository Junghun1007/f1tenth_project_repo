// 새 코너 경로 계획 (속도 반영 내륜차 통로 + 최소 곡률 스플라인). 브라우저·node 공용.
// 좌표: 차량 기준 x 전방, y 왼쪽, 원점 = 앞차축 (BEV 맨 아래 줄).
(function (root) {
'use strict';
const DB = 0.05, DF = 0.01, MPP = 0.01, PAD = 30, P_SIGMA = 0.15;   // 기준선 평활화 폭 (m)
const sub = (a, b) => ({x: a.x - b.x, y: a.y - b.y}), add = (a, b) => ({x: a.x + b.x, y: a.y + b.y});
const mul = (a, s) => ({x: a.x * s, y: a.y * s}), nrm = a => Math.hypot(a.x, a.y);
const unit = a => {const n = nrm(a) || 1; return {x: a.x / n, y: a.y / n};};
const toV = p => ({x: 3.0 - (p[1] + 0.5) * MPP, y: 0.6 - (p[0] - PAD + 0.5) * MPP});

function resample(P, st) {
  if (P.length < 2) return P.slice();
  const o = [P[0]]; let acc = 0;
  for (let i = 1; i < P.length; i++) {
    let a = P[i - 1]; const b = P[i]; let l = nrm(sub(b, a));
    while (acc + l >= st) {const t = (st - acc) / l; a = add(a, mul(sub(b, a), t)); o.push(a); l = nrm(sub(b, a)); acc = 0;}
    acc += l;
  }
  return o;
}
function gauss(v, s) {
  const r = Math.ceil(3 * s), o = v.map(() => 0);
  for (let i = 0; i < v.length; i++) {
    let a = 0, ws = 0;
    for (let j = -r; j <= r; j++) {const k = Math.min(v.length - 1, Math.max(0, i + j)), g = Math.exp(-j * j / (2 * s * s)); a += v[k] * g; ws += g;}
    o[i] = a / ws;
  }
  return o;
}
// 부호 있는 곡률 (왼쪽 회전 +)
function curvSigned(P, smooth) {
  const N = P.length, k = new Array(N).fill(0);
  for (let i = 1; i < N - 1; i++) {
    const a = sub(P[i], P[i - 1]), b = sub(P[i + 1], P[i]), c = sub(P[i + 1], P[i - 1]);
    k[i] = 2 * (a.x * b.y - a.y * b.x) / (nrm(a) * nrm(b) * nrm(c) + 1e-9);
  }
  if (N > 1) {k[0] = k[1]; k[N - 1] = k[N - 2];}
  return smooth ? gauss(k, smooth) : k;
}
// 관측 차선 점 중 기준점 전후 ±4 cm 안에 있는 것들의 법선 방향 거리 최솟값 (차량 코드와 같은 방식: 점 순서 무관)
function sideDistance(p, t, n, pts, limit) {
  let best = Infinity;
  for (const q of pts) {
    const d = sub(q, p);
    if (Math.abs(d.x * t.x + d.y * t.y) > 0.04) continue;
    const lat = d.x * n.x + d.y * n.y;
    if (lat >= 0 && lat < best) best = lat;
  }
  return best <= limit ? best : null;
}

// 기준선(차선 중심) 주위의 통로: 점마다 왼쪽·오른쪽 차선까지 법선 방향 거리
function corridor(ref, left, right, laneWidth) {
  let base = ref.slice();
  // 기준선을 첫 방향 그대로 뒤로 늘려 차량 옆에서 시작 (차량 위치로 바로 꺾어 붙이면 복귀 때 휘청거림)
  {const t0 = unit(sub(base[Math.min(3, base.length - 1)], base[0])), e = base[0].x * t0.x + base[0].y * t0.y;
    if (e > 0.03) base = [sub(base[0], mul(t0, e))].concat(base);}
  base = resample(base, DB);
  // 끝점을 고정한 채 평활화 (양 끝을 점대칭으로 늘려서 가우시안) — 그냥 하면 양 끝이 안쪽으로 줄어든다
  const s = P_SIGMA / DB, r = Math.ceil(3 * s), n0 = base.length;
  const ext = [];
  for (let i = r; i >= 1; i--) ext.push(sub(mul(base[0], 2), base[Math.min(n0 - 1, i)]));
  ext.push(...base);
  for (let i = 1; i <= r; i++) ext.push(sub(mul(base[n0 - 1], 2), base[Math.max(0, n0 - 1 - i)]));
  const xs = gauss(ext.map(p => p.x), s).slice(r, r + n0), ys = gauss(ext.map(p => p.y), s).slice(r, r + n0);
  const R = resample(xs.map((x, i) => ({x, y: ys[i]})), DB), N = R.length;
  const T = R.map((p, i) => unit(sub(R[Math.min(N - 1, i + 1)], R[Math.max(0, i - 1)])));
  const nL = T.map(t => ({x: -t.y, y: t.x}));
  const lp = left.flat(), rp = right.flat();
  const dl = [], dr = [], est = [];
  for (let i = 0; i < N; i++) {
    let l = sideDistance(R[i], T[i], nL[i], lp, 1.0), r = sideDistance(R[i], T[i], mul(nL[i], -1), rp, 1.0), e = 0;
    if (l == null && r != null) {l = laneWidth - r; e = 1;}            // 안 보이는 쪽은 차선 폭으로 추정
    else if (r == null && l != null) {r = laneWidth - l; e = 2;}
    else if (l == null && r == null) {l = r = laneWidth / 2; e = 3;}
    dl.push(l); dr.push(r); est.push(e);
  }
  return {R, nL, dl, dr, est, N};
}

function speedProfile(kabs, P) {
  const N = kabs.length, lim = kabs.map(x => Math.min(P.VC, Math.sqrt(P.ALAT / Math.max(x, 1e-6))));
  for (let i = N - 2; i >= 0; i--) lim[i] = Math.min(lim[i], Math.sqrt(lim[i + 1] ** 2 + 2 * P.AD * DB));
  const v = [P.VC]; let over = false;
  for (let i = 1; i < N; i++) {
    const up = Math.sqrt(v[i - 1] ** 2 + 2 * P.AACC * DB), dn = Math.sqrt(Math.max(0, v[i - 1] ** 2 - 2 * P.AD * DB));
    const x = Math.max(dn, Math.min(lim[i], up));
    if (x > lim[i] + 0.02) over = true;
    v.push(x);
  }
  return {v, over};
}

// 경로를 따라갈 때 차체 네 모서리의 통로 안 여유 (음수 = 차선 밟음)
function trace(path, C, P) {
  const F = resample(path, DF);
  const k = curvSigned(resample(path, DB), 1.5), sp = speedProfile(k.map(Math.abs), P);
  let r = {x: F[0].x - P.L, y: F[0].y};
  let j = 0, minL = Infinity, minR = Infinity;
  const rear = [], touch = [];
  for (let i = 0; i < F.length; i++) {
    const d = unit(sub(F[i], r)); r = sub(F[i], mul(d, P.L));
    const c = Math.min(k.length - 1, Math.round(i * DF / DB));
    const left = {x: -d.y, y: d.x}, slipShift = Math.max(0, Math.abs(k[c]) * P.L * P.L / 2 * (sp.v[c] / P.V0) ** 2);
    const rs = add(r, mul(left, -Math.sign(k[c]) * slipShift));       // 슬립: 뒤축이 회전 바깥쪽으로
    const corners = [add(rs, mul(left, P.W / 2)), sub(rs, mul(left, P.W / 2)),
      add(add(F[i], mul(d, P.FOH)), mul(left, P.W / 2)), sub(add(F[i], mul(d, P.FOH)), mul(left, P.W / 2))];
    let cl = Infinity, cr = Infinity;
    for (const q of corners) {
      while (j + 1 < C.N && nrm(sub(q, C.R[j + 1])) < nrm(sub(q, C.R[j]))) j++;
      let jj = j; for (let t = Math.max(0, j - 12); t <= Math.min(C.N - 1, j + 12); t++) if (nrm(sub(q, C.R[t])) < nrm(sub(q, C.R[jj]))) jj = t;
      const lat = (q.x - C.R[jj].x) * C.nL[jj].x + (q.y - C.R[jj].y) * C.nL[jj].y;
      // 실제로 관측한 쪽 차선에 대해서만 여유를 잰다 (추정한 쪽은 판정에서 제외)
      if (C.est[jj] !== 1 && C.est[jj] !== 3) cl = Math.min(cl, C.dl[jj] - lat);
      if (C.est[jj] !== 2 && C.est[jj] !== 3) cr = Math.min(cr, C.dr[jj] + lat);
    }
    if (cl < Infinity) minL = Math.min(minL, cl);
    if (cr < Infinity) minR = Math.min(minR, cr);
    rear.push(rs); touch.push(Math.min(cl, cr) < 0);
  }
  // 최소 반경: ±0.24 m 떨어진 세 점으로 (점 잡음에 둔감하게, 웹 표시와 같은 기준)
  const G = resample(path, 0.02); let kmax = 0;
  for (let i = 12; i < G.length - 12; i++) {
    const a = G[i - 12], b = G[i], c = G[i + 12], den = nrm(sub(b, a)) * nrm(sub(c, b)) * nrm(sub(a, c));
    if (den > 1e-9) kmax = Math.max(kmax, Math.abs(2 * ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x))) / den);
  }
  let ia = 0; for (let i = 0; i < k.length; i++) if (Math.abs(k[i]) > Math.abs(k[ia])) ia = i;
  return {rear, touch, minL, minR, minClear: Math.min(minL, minR), rmin: kmax > 0 ? 1 / kmax : Infinity, vApex: sp.v[ia], over: sp.over};
}

function evalD(basis, c) {return basis.map(b => b.w[0] * c[b.m] + b.w[1] * c[b.m + 1] + b.w[2] * c[b.m + 2] + b.w[3] * c[b.m + 3]);}
function optimize(C, lo, hi, basis, ctrl, st, centerW, target) {
  const cost = c => {
    const d = evalD(basis, c), Q = C.R.map((p, i) => add(p, mul(C.nL[i], d[i])));
    let J = 0;
    for (let i = 1; i < C.N - 1; i++) {
      const a = sub(Q[i], Q[i - 1]), b = sub(Q[i + 1], Q[i]), e = sub(Q[i + 1], Q[i - 1]);
      const k = 2 * (a.x * b.y - a.y * b.x) / (nrm(a) * nrm(b) * nrm(e) + 1e-9); J += k * k * DB;
    }
    for (let i = 0; i < C.N; i++) {const v = Math.max(0, lo[i] - d[i]) + Math.max(0, d[i] - hi[i]); J += 1e5 * v * v + centerW[i] * (d[i] - target[i]) ** 2 * DB;}
    const a = sub(Q[1], Q[0]), b = sub(Q[2], Q[1]), e = sub(Q[2], Q[0]);
    const k1 = 2 * (a.x * b.y - a.y * b.x) / (nrm(a) * nrm(b) * nrm(e) + 1e-9), s0 = d[1] - d[0] - st.slope * DB;
    return J + 1e5 * ((d[0] - st.d0) ** 2 + s0 * s0) + 2.0 * (k1 - st.kv) ** 2;
  };
  let J = cost(ctrl);
  for (let h = 0.08; h > 0.0008; h /= 2) for (let sweep = 0; sweep < 40; sweep++) {
    let imp = false;
    for (let m = 0; m < ctrl.length; m++) for (const s of [h, -h]) {
      ctrl[m] += s; const j = cost(ctrl);
      if (j < J - 1e-9) {J = j; imp = true; break;}
      ctrl[m] -= s;
    }
    if (!imp) break;
  }
  return evalD(basis, ctrl);
}

// frame: line_detactor 결과 (픽셀 좌표). P: {VC, AD, AACC, ALAT, V0, Min, Mout, W, L, FOH, LW}
function planFrame(frame, P) {
  const ref = (frame.reference || []).map(toV);
  const left = ((frame.observed || [])[0] || []).map(f => f.map(toV));
  const right = ((frame.observed || [])[1] || []).map(f => f.map(toV));
  const old = (frame.centerline || []).map(toV);
  if (ref.length < 4) return {ok: false, reason: '차선 중심선 없음', old};
  const C = corridor(ref, left, right, P.LW), N = C.N;
  const DS = 0.3, nC = Math.ceil((N - 1) * DB / DS) + 3, ctrl = new Array(nC).fill(0);
  const basis = C.R.map((_, i) => {const u = i * DB / DS, m = Math.min(Math.floor(u), nC - 4), t = u - m;
    return {m, w: [(1 - t) ** 3 / 6, (3 * t ** 3 - 6 * t * t + 4) / 6, (-3 * t ** 3 + 3 * t * t + 3 * t + 1) / 6, t ** 3 / 6]};});
  // 차량 출발 상태 (기준선 좌표)와 중앙 복귀 비중 — corner_planner.cpp와 같음
  const t0 = unit(sub(C.R[1], C.R[0]));
  const st = {d0: -(C.R[0].x * C.nL[0].x + C.R[0].y * C.nL[0].y), slope: Math.min(1.5, Math.max(-1.5, C.nL[0].x / Math.max(0.2, t0.x))), kv: P.KV || 0};
  const returnDist = Math.max(P.RET_MIN || 0.8, Math.max(P.VC, 0) * (P.RET_T || 1.0)), retW = P.RET_W == null ? 100 : P.RET_W;
  const target = C.R.map((_, i) => {const u = Math.min(1, i * DB / returnDist); return st.d0 * (1 - u * u * u * (10 - 15 * u + 6 * u * u));});
  // 복귀 세기는 지금 치우친 만큼만 (5 cm 이상이면 최대) — 중앙 근처에서 코너에 들어갈 땐 경로를 묶지 않는다.
  // 복귀 거리 뒤 0.8 m까지 중앙을 붙잡아 반대편으로 넘어가지 않게
  const retScale = Math.min(1, Math.abs(st.d0) / 0.05);
  const centerW = C.R.map((_, i) => 0.2 + (i * DB < returnDist + 0.8 ? retW * retScale : 0));
  const exL = new Array(N).fill(0), exR = new Array(N).fill(0), nLw = Math.round(P.L / DB);
  let d = new Array(N).fill(0), path = C.R.slice(), lo = [], hi = [], infeas = false, tr = null;
  for (let loop = 0; loop < 4; loop++) {
    const k = curvSigned(path, 1.5), sp = speedProfile(k.map(Math.abs), P);
    const off = gauss(k.map((x, i) => Math.abs(x) * P.L * P.L / 2 * (1 - (sp.v[i] / P.V0) ** 2)), 2);
    infeas = false;
    for (let i = 0; i < N; i++) {
      const turning = Math.abs(k[i]) > 0.3, leftInner = k[i] > 0;
      const inner = turning ? P.Min + Math.max(0, off[i]) : P.Mout, outer = P.Mout + (turning ? Math.max(0, -off[i]) : 0);
      const mL = leftInner ? inner : outer, mR = leftInner ? outer : inner;
      hi[i] = C.dl[i] - (P.W / 2 + mL + exL[i]);
      lo[i] = -(C.dr[i] - (P.W / 2 + mR + exR[i]));
      if (lo[i] > hi[i]) {
        infeas = true;
        const m = Math.min(C.dl[i] - P.W / 2, Math.max(-(C.dr[i] - P.W / 2), (lo[i] + hi[i]) / 2));
        lo[i] = hi[i] = m;
      }
    }
    for (let i = 0; i < N && i * DB < 0.3; i++) {lo[i] = Math.min(lo[i], st.d0); hi[i] = Math.max(hi[i], st.d0);}
    d = optimize(C, lo, hi, basis, ctrl, st, centerW, target);
    path = C.R.map((p, i) => add(p, mul(C.nL[i], d[i])));
    tr = trace(path, C, P);
    if (loop === 3 || tr.minClear >= 0.02) break;
    // 여유가 모자란 곳은 그 지점부터 축거만큼 앞까지 통로를 좁혀 다시 맞춘다
    const F = resample(path, DF);
    const defL = new Array(N).fill(0), defR = new Array(N).fill(0);
    let r = {x: F[0].x - P.L, y: F[0].y};
    for (let i = 0; i < F.length; i += 2) {
      const dd = unit(sub(F[i], r)); r = sub(F[i], mul(dd, P.L));
      const lf = {x: -dd.y, y: dd.x};
      for (const q of [add(tr.rear[i], mul(lf, P.W / 2)), sub(tr.rear[i], mul(lf, P.W / 2))]) {
        let jj = 0, best = Infinity; for (let t = 0; t < N; t++) {const dist = nrm(sub(q, C.R[t])); if (dist < best) {best = dist; jj = t;}}
        const lat = (q.x - C.R[jj].x) * C.nL[jj].x + (q.y - C.R[jj].y) * C.nL[jj].y;
        if (C.est[jj] !== 1 && C.est[jj] !== 3) defL[jj] = Math.max(defL[jj], 0.02 - (C.dl[jj] - lat));
        if (C.est[jj] !== 2 && C.est[jj] !== 3) defR[jj] = Math.max(defR[jj], 0.02 - (C.dr[jj] + lat));
      }
    }
    for (let t = 0; t < N; t++) {
      let a = 0, b = 0;
      for (let u = Math.max(0, t - nLw); u <= t; u++) {a = Math.max(a, defL[u]); b = Math.max(b, defR[u]);}
      exL[t] = Math.min(0.15, exL[t] + Math.max(0, a)); exR[t] = Math.min(0.15, exR[t] + Math.max(0, b));
    }
  }
  const oldTr = old.length >= 4 ? trace(old, C, P) : null;
  return {ok: true, path, tr, old, oldTr, infeas, C,
    // 앞차축 중심이 갈 수 있는 한계 (차선 − 차폭/2 − 여유 − 내륜차)
    hiPts: C.R.map((p, i) => add(p, mul(C.nL[i], hi[i]))), loPts: C.R.map((p, i) => add(p, mul(C.nL[i], lo[i]))),
    estimated: C.est.filter(e => e).length / C.N};
}

const api = {planFrame, toV};
if (typeof module !== 'undefined') module.exports = api; else root.Planner = api;
})(this);
