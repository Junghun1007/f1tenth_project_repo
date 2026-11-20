// semi_slam 브라우저 화면. Jetson은 수치·좌표만 보내고 그림은 여기서 그린다.
// 데모: index.html?demo (가짜 주행으로 화면 확인)
"use strict";

const qs = new URLSearchParams(location.search);
const DEMO = qs.has("demo");
const $ = (id) => document.getElementById(id);
const STAGE_LABEL = {
  cam_rgb: "카메라 RGB 수신", receive: "영상 지연(촬영→처리)", stabilize_bev: "안정화+BEV", lane_infer: "차선 추론",
  lane_post: "차선 후처리", control: "제어", lane_side_fix: "⚠ 차선 좌우 교정(회/s)", lane_side_drop: "⚠ 차선 조각 제외(회/s)",
};
const READY_LABEL = { attitude_ok: "카메라 자세", lane_model_ready: "차선 모델", vesc_connected: "VESC", erpm_fresh: "ERPM", lane_fresh: "차선 입력", client_alive: "브라우저" };

let latest = null, params = [], edits = {}, group = "slam", mapData = null;
let clientId = "";
try { clientId = localStorage.getItem("ss_client") || ""; } catch (e) { /* 저장소 없음 */ }
if (!clientId) { clientId = Math.random().toString(36).slice(2, 10); try { localStorage.setItem("ss_client", clientId); } catch (e) { /* 없음 */ } }

// 화면에 쌓는 데이터
const laps = [[]];            // 바퀴별 궤적 [[x,y],...]
let afterTrack = [];          // 보정 후 다시 계산한 궤적
let freeTrack = [];           // 자유 적분 궤적 (비교용)
let lastLap = 0, lastRunning = false, lastFinishing = false;

// ---------- 서버 통신 ----------
async function post(path, body) {
  if (DEMO) return { ok: true, reason: "" };
  if (path === "/api/view") body = Object.assign({ client: clientId }, body || {});
  try {
    const r = await fetch(path, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body || {}) });
    return await r.json();
  } catch (e) { return { ok: false, reason: "요청 실패: " + e.message }; }
}
async function setParam(name, value) {
  const r = await post("/api/params", { changes: { [name]: value } });
  if (!r.ok) $("reason").textContent = r.reason; else loadParams();
}
function connect() {
  const es = new EventSource("/api/events");
  es.onmessage = (ev) => { onTelemetry(JSON.parse(ev.data)); setLink(true); };
  es.onerror = () => setLink(false);
  setInterval(() => post("/api/heartbeat", { client: clientId }), 200);
  post("/api/view", { preview: true, bev: $("togBev").checked, camera: false });
}
function setLink(ok) { const el = $("link"); el.textContent = ok ? "연결됨" : "연결 끊김"; el.className = "link" + (ok ? " ok" : ""); }
async function loadParams() {
  if (DEMO) { params = demoParams(); renderParams(); syncBar(); return; }
  try { params = (await (await fetch("/api/params")).json()).params; } catch (e) { params = []; }
  renderParams(); syncBar();
}
async function loadRun() {   // 재접속·보정 후: 서버가 모아 둔 궤적·매핑 점으로 다시 그림
  if (DEMO) return;
  try {
    const r = await (await fetch("/api/run")).json();
    laps.length = 0; laps.push(r.track || []);
    afterTrack = r.after || [];
    freeTrack = r.free || [];
    resetPoints(); addPoints(r.points || []);
  } catch (e) { /* 다음에 다시 */ }
}

// ---------- 트랙 캔버스 ----------
const cv = $("map"), ctx = cv.getContext("2d");
const PX = 150, M = 18;   // px/m, 여백
let bounds = { x0: 0, y0: 0, x1: 4, y1: 4 };
const ptsLayer = document.createElement("canvas"), baseLayer = document.createElement("canvas");
function P(x, y) { return [M + (x - bounds.x0) * PX, M + (bounds.y1 - y) * PX]; }
function setupCanvas() {
  const xs = mapData.outer.map((p) => p[0]), ys = mapData.outer.map((p) => p[1]);
  bounds = { x0: Math.floor(Math.min(...xs)), y0: Math.floor(Math.min(...ys)), x1: Math.ceil(Math.max(...xs)), y1: Math.ceil(Math.max(...ys)) };
  for (const c of [cv, ptsLayer, baseLayer]) { c.width = Math.round((bounds.x1 - bounds.x0) * PX + 2 * M); c.height = Math.round((bounds.y1 - bounds.y0) * PX + 2 * M); }
  drawBase(); resetPoints();
}
function line(c, pts, color, w, dash, close) {
  if (!pts || pts.length < 2) return;
  c.save(); c.strokeStyle = color; c.lineWidth = w; c.setLineDash(dash || []); c.lineJoin = "round"; c.beginPath();
  pts.forEach((p, i) => { const [u, v] = P(p[0], p[1]); i ? c.lineTo(u, v) : c.moveTo(u, v); });
  if (close) c.closePath();
  c.stroke(); c.restore();
}
function drawBase() {
  const c = baseLayer.getContext("2d");
  c.fillStyle = "#0b0e12"; c.fillRect(0, 0, baseLayer.width, baseLayer.height);
  c.strokeStyle = "#1d242c"; c.lineWidth = 1; c.fillStyle = "#55606b"; c.font = "11px sans-serif";
  for (let x = bounds.x0; x <= bounds.x1 + 1e-6; x += 0.5) { const [u] = P(x, 0); c.beginPath(); c.moveTo(u, M); c.lineTo(u, baseLayer.height - M); c.stroke(); if (Number.isInteger(x)) c.fillText(x + " m", u + 2, baseLayer.height - M - 3); }
  for (let y = bounds.y0; y <= bounds.y1 + 1e-6; y += 0.5) { const [, v] = P(0, y); c.beginPath(); c.moveTo(M, v); c.lineTo(baseLayer.width - M, v); c.stroke(); }
  line(c, mapData.outer, "#5b6573", 4, [], true);
  line(c, mapData.inner, "#5b6573", 4, [], true);
  line(c, mapData.center, "#9aa4b1", 1.5, [8, 6], true);
  line(c, mapData.stop, "#f8fafc", 4);
  line(c, mapData.start, "#ef4444", 5);
  c.font = "bold 12px sans-serif";
  { const [u, v] = P(mapData.stop[1][0] - 0.12, mapData.stop[1][1] + 0.08); c.fillStyle = "#f8fafc"; c.fillText("S", u, v); }
  { const [u, v] = P(mapData.start[1][0] + 0.03, mapData.start[1][1] + 0.08); c.fillStyle = "#ef4444"; c.fillText("T1", u, v); }
  // 출발 위치 (선택한 방향): 아래 직선 시작점, 진행 방향 화살표
  const sp = pval("slam.direction") === 1 ? mapData.start_cw : mapData.start_ccw;
  if (sp) {
    const [a, b] = sp, dx = b[0] - a[0], dy = b[1] - a[1], tip = [a[0] + 0.3 * dx, a[1] + 0.3 * dy];
    c.save(); c.strokeStyle = c.fillStyle = "#22c55e"; c.lineWidth = 3;
    const [u0, v0] = P(a[0], a[1]), [u1, v1] = P(tip[0], tip[1]);
    c.beginPath(); c.arc(u0, v0, 5, 0, 2 * Math.PI); c.fill();
    c.beginPath(); c.moveTo(u0, v0); c.lineTo(u1, v1); c.stroke();
    const ang = Math.atan2(v1 - v0, u1 - u0);
    c.beginPath(); c.moveTo(u1, v1); c.lineTo(u1 - 9 * Math.cos(ang - 0.5), v1 - 9 * Math.sin(ang - 0.5)); c.lineTo(u1 - 9 * Math.cos(ang + 0.5), v1 - 9 * Math.sin(ang + 0.5)); c.closePath(); c.fill();
    c.fillText("출발", u0 - 12, v0 + 18); c.restore();
  }
}
function resetPoints() { const c = ptsLayer.getContext("2d"); c.clearRect(0, 0, ptsLayer.width, ptsLayer.height); }
function addPoints(pts) {   // 매핑 점은 누적 레이어에 한 번만 그린다
  if (!mapData) return;
  const c = ptsLayer.getContext("2d");
  for (const p of pts) { const [u, v] = P(p[0], p[1]); c.fillStyle = p[2] === 0 ? "rgba(59,130,246,.75)" : "rgba(239,68,68,.75)"; c.fillRect(u - 1.2, v - 1.2, 2.4, 2.4); }
}
function drawMap() {
  if (!mapData) return;
  ctx.drawImage(baseLayer, 0, 0);
  ctx.drawImage(ptsLayer, 0, 0);
  line(ctx, freeTrack, "rgba(148,163,184,.45)", 1.5);
  laps.slice(0, -1).forEach((l) => line(ctx, l, "#0e7490", 2));
  line(ctx, afterTrack, "#a3e635", 2, [5, 4]);
  const cur = laps[laps.length - 1];
  line(ctx, cur, "#22d3ee", 3);
  const r = latest && latest.run;
  if (r && (r.running || cur.length)) {   // 차량 화살표
    const [x, y, yaw] = r.pose; const [u, v] = P(x, y);
    ctx.save(); ctx.translate(u, v); ctx.rotate(-yaw); ctx.fillStyle = "#facc15"; ctx.beginPath();
    ctx.moveTo(14, 0); ctx.lineTo(-10, 8); ctx.lineTo(-10, -8); ctx.closePath(); ctx.fill(); ctx.restore();
  }
}

// ---------- BEV 차선 각도 (카메라 틀어짐 확인용) ----------
// 차를 직선에 차선과 나란히 세웠을 때 0°에 가까워야 정상. 계속 몇 도씩 나오면 BEV(카메라)가 그만큼 틀어진 것
const angHist = [];
function fitLine(pts) {   // y = a·x + b, 가까운 직선 부분만 (semi_slam 정지선 기준 보정과 같은 범위)
  let n = 0, sx = 0, sy = 0, sxx = 0, sxy = 0;
  for (const [x, y] of pts) if (x >= 0.15 && x <= 0.8) { n++; sx += x; sy += y; sxx += x * x; sxy += x * y; }
  const den = n * sxx - sx * sx;
  if (n < 6 || Math.abs(den) < 1e-9) return null;
  const a = (n * sxy - sx * sy) / den;
  return { a, b: (sy - a * sx) / n };
}
function laneAngle(lane) {
  const L = fitLine(lane.left || []), R = fitLine(lane.right || []);
  const d = (f) => (f ? Math.atan(f.a) * 180 / Math.PI : null);
  const left = d(L), right = d(R);
  const both = L && R && Math.abs(L.a - R.a) < 0.1;
  const mean = both ? (left + right) / 2 : left !== null ? left : right;
  const now = performance.now() / 1000;
  if (both) angHist.push([now, mean]);
  while (angHist.length && angHist[0][0] < now - 2) angHist.shift();
  const avg = angHist.length ? angHist.reduce((s, v) => s + v[1], 0) / angHist.length : null;
  const center = both ? ((L.a + R.a) / 2) * 0.3 + (L.b + R.b) / 2 : null;
  return { left, right, mean, avg, center };
}

// ---------- BEV 미니 ----------
const bcv = $("bev"), bctx = bcv.getContext("2d");
function BP(x, y) { return [(0.6 - y) * 100, (3.0 - x) * 100]; }
function drawBev(f) {
  bctx.fillStyle = "#0a0d10"; bctx.fillRect(0, 0, 120, 300);
  const img = $("bevImg");
  if ($("togBev").checked && img.complete && img.naturalWidth > 0) bctx.drawImage(img, 0, 0, 120, 300);
  if (!f.lane) return;
  const pl = (pts, color, w, dash) => { if (!pts || pts.length < 2) return; bctx.save(); bctx.strokeStyle = color; bctx.lineWidth = w; bctx.setLineDash(dash || []);
    bctx.beginPath(); pts.forEach((p, i) => { const [u, v] = BP(p[0], p[1]); i ? bctx.lineTo(u, v) : bctx.moveTo(u, v); }); bctx.stroke(); bctx.restore(); };
  pl(f.lane.left, "#3b82f6", 3); pl(f.lane.right, "#ef4444", 3); pl(f.lane.path, "#facc15", 2, [6, 5]);
  if (f.lane.stop !== null && f.lane.stop !== undefined && isFinite(f.lane.stop)) {
    const [, v] = BP(f.lane.stop, 0); bctx.strokeStyle = "#22c55e"; bctx.lineWidth = 5; bctx.beginPath(); bctx.moveTo(10, v); bctx.lineTo(110, v); bctx.stroke();
  }
}

// ---------- 화면 갱신 ----------
function kv(el, rows) { el.innerHTML = rows.map(([k, v]) => `<span class="k">${k}</span><span class="v">${v}</span>`).join(""); }
const cm = (m) => (m * 100).toFixed(1) + " cm";
function usageRow(name, value) {
  const ok = value !== null && value !== undefined && isFinite(value), pct = ok ? Math.max(0, Math.min(100, value)) : 0;
  return `<div class="bar-row"><span>${name}</span><div class="track"><div class="fill ${pct > 85 ? "hot" : ""}" style="width:${pct}%"></div></div><span>${ok ? value.toFixed(0) + " %" : "--"}</span></div>`;
}
function onTelemetry(f) {
  latest = f;
  const r = f.run;
  if (r.running && !lastRunning) { laps.length = 0; laps.push([]); afterTrack = []; freeTrack = []; resetPoints(); lastLap = 0; }   // 새 주행
  for (const p of f.new_track || []) laps[laps.length - 1].push(p);
  for (const p of f.new_free || []) freeTrack.push(p);
  if (r.lap !== lastLap && r.running) { const last = laps[laps.length - 1]; laps.push(last.length ? [last[last.length - 1]] : []); lastLap = r.lap; }
  addPoints(f.new_points || []);
  if (lastFinishing && !r.finishing) loadRun();   // 보정 끝남 → 보정 후 궤적
  lastRunning = r.running; lastFinishing = r.finishing;
  if (f.params_version !== undefined && f.params_version !== window._pv) { window._pv = f.params_version; loadParams(); }
  render();
}
let pending = false;
function render() { if (pending) return; pending = true; requestAnimationFrame(() => { pending = false; renderNow(); }); }
function renderNow() {
  const f = latest;
  if (!f) return;
  $("stateBadge").textContent = f.state; $("stateBadge").className = "state " + f.state;
  $("reason").textContent = f.reason || "";
  const st = $("status"); st.textContent = f.run.running && f.run.waiting_s ? "가속 중 · 정지선 S를 지나면 기록·보정 시작" : (f.run.status || ""); st.className = "status" + (/실패|않음|중간/.test(f.run.status || "") ? " warn" : "");
  const stopped = ["READY", "MANUAL"].includes(f.state);
  $("btnStart").disabled = !(stopped && Object.values(f.readiness).every(Boolean)) || f.run.finishing;
  $("btnRevert").disabled = $("btnReset2").disabled = f.run.running || f.run.finishing;
  const r = f.run, v = f.vehicle;
  $("lapBig").innerHTML = `${r.lap} <small>/ ${r.laps} 바퀴</small>`;
  kv($("drive"), [["속도 / 목표", `${v.speed.toFixed(2)} / ${v.target.toFixed(2)} m/s`],
    ["이번 바퀴 진행", `${r.lap_dist.toFixed(2)} / ${r.lap_length.toFixed(2)} m (${Math.min(999, 100 * r.lap_dist / r.lap_length).toFixed(0)}%)`],
    ["중앙선 위 위치 · 옆", `${(r.s_arc ?? 0).toFixed(2)} m · ${r.lat === null || r.lat === undefined || !isFinite(r.lat) ? "-" : (r.lat >= 0 ? "왼쪽 " : "오른쪽 ") + cm(Math.abs(r.lat))}`],
    ["마지막 코너 보정", r.corner_fix === undefined ? "-" : `${r.corner_fix >= 0 ? "+" : ""}${(r.corner_fix * 100).toFixed(1)} cm`],
    ["위치 (x, y) · 방향", `${r.pose[0].toFixed(2)}, ${r.pose[1].toFixed(2)} m · ${(r.pose[2] * 180 / Math.PI).toFixed(1)}°`],
    ["duty / 단계", `${v.duty.toFixed(3)} · ${v.phase || "-"}`], ["CTE · 조향", `${cm(v.cte)} · ${v.steer_deg.toFixed(1)}°`],
    ["기록 폴더", r.dir || "-"]]);
  const fin = (x) => x !== null && x !== undefined && isFinite(x);
  const sgn = (x, d) => (x >= 0 ? "+" : "") + x.toFixed(d);
  $("lapTable").querySelector("tbody").innerHTML = f.laps.map((m) =>
    `<tr><td>${m.lap}</td><td>${m.dist.toFixed(3)} m</td><td class="${Math.abs(m.s_err) > 0.1 ? "bad" : ""}">${sgn(m.s_err * 100, 1)} cm</td>` +
    `<td class="${Math.abs(m.turn - 360) > 2 ? "bad" : ""}">${m.turn.toFixed(1)}°</td><td>${sgn(m.lat * 100, 1)} cm</td>` +
    `<td class="${Math.abs(m.corner_fix || 0) > 0.15 ? "bad" : ""}">${sgn((m.corner_fix || 0) * 100, 1)} cm</td>` +
    `<td>${sgn(m.free_along * 100, 1)} cm · ${sgn(m.free_heading, 1)}°</td></tr>`).join("");
  $("accNote").innerHTML = f.laps.length
    ? "S 도착 = 바퀴 거리 − 중앙선 한 바퀴 · 회전 = 보정 적용한 자이로 (정답 360°) · 옆 위치 + = 중앙선보다 왼쪽 · 코너 보정 = 코너 한가운데(자이로 45°·135°·225°·315°)에서 다시 잡을 때 옮긴 가장 큰 양"
    : "정지선 S를 지날 때마다 한 줄씩 채워집니다";
  const c = f.calib;
  kv($("calib"), [["자이로 배율", `${c.gyro_scale.toFixed(4)} · 누적 ${c.gyro_laps}바퀴`], ["기준 한 바퀴 (지도 중앙선)", `${(c.lap_ref || 0).toFixed(3)} m`],
    ["S 통과 판정 오프셋 (반시계 / 시계)", `${sgn((c.stop_offset || 0) * 100, 1)} cm (${c.stop_laps || 0}바퀴) / ${sgn((c.stop_offset_cw || 0) * 100, 1)} cm (${c.stop_laps_cw || 0}바퀴)`],
    ["카메라 yaw (설정)", sgn(c.camera_yaw || 0, 2) + "°"], ["정지 영점 (이번 출발)", (r.bias0 * 180 / Math.PI).toFixed(3) + " °/s"],
    [`BEV로 잰 차선 폭 (지도 ${mapData && mapData.lane_width ? cm(mapData.lane_width) : "-"})`, fin(r.lane_width) ? cm(r.lane_width) : "-"]]);
  $("wheelTable").querySelector("tbody").innerHTML = c.wheel.length ? c.wheel.map(([s, k, n, cw]) => `<tr><td>${cw ? "시계" : "반시계"}</td><td>${s.toFixed(2)} m/s</td><td>${k.toFixed(4)}</td><td>${n}</td></tr>`).join("") : `<tr><td colspan="4" style="color:var(--dim)">아직 없음 (기본 1.0)</td></tr>`;
  const res = f.result;
  kv($("lastResult"), res ? [["마지막 주행에서 구한 값", `자이로 ×${fin(res.gyro_est) ? res.gyro_est.toFixed(4) : "-"} · 거리 ×${fin(res.wheel_est) ? res.wheel_est.toFixed(4) : "-"} (${res.laps}바퀴)`],
    ["원값 한 바퀴", `${res.turn_per_lap.toFixed(1)}° · ${res.dist_per_lap.toFixed(3)} m`],
    ["S 통과 판정 오프셋 (이번)", fin(res.stop_offset) ? `${sgn(res.stop_offset * 100, 1)} cm (${res.stop_laps}바퀴, − = 판정 순간 차가 아직 S 앞)` : "-"],
    ["직선에서 차선 대비 각 (참고)", fin(res.straight_angle) ? `BEV 각 ${sgn(res.straight_angle, 1)}° · 옆 이동으로 본 실제 각 ${sgn(res.straight_slip, 1)}° (직선 ${res.straights}개). 카메라 yaw 판단에는 쓰지 말 것: 차가 비스듬히 달리는 것이 섞임` : "직선 구간 부족"],
    ["결과", res.message]] : [["마지막 주행", "아직 없음"]]);
  $("histTable").querySelector("tbody").innerHTML = f.history.slice().reverse().map((h) => h.action === "reverted"
    ? `<tr><td>${h.time.slice(11, 16)}</td><td colspan="4">이전 값으로 되돌림</td><td></td></tr>`
    : `<tr><td>${h.time.slice(11, 16)}</td><td>${h.speed.toFixed(2)}</td><td>${h.laps}</td><td>${fin(h.gyro) ? h.gyro.toFixed(4) : "-"} / ${fin(h.wheel) ? h.wheel.toFixed(4) : "-"}</td>` +
      `<td class="${h.action === "applied" ? "good" : "bad"}">${h.action === "applied" ? "반영" : "안 함"}</td></tr>`).join("");
  const seen = r.stop_seen !== null && r.stop_seen !== undefined && isFinite(r.stop_seen);
  kv($("stopInfo"), [["지금 보이는 정지선", seen ? `${r.stop_seen.toFixed(2)} m 앞 · ${r.stop_near ? "S 맞음" : "S 아님 → 무시"}` : "-"],
    ["무시한 정지선 (S와 먼 위치)", String(r.rejected_far || 0)], ["무시한 통과 (바퀴 거리 안 맞음)", String(r.rejected_lap || 0)]]);
  $("corrTable").querySelector("tbody").innerHTML = f.corrections.length ? f.corrections.map(([lap, s]) => `<tr><td>${lap}</td><td>${(s >= 0 ? "+" : "") + cm(s)}</td></tr>`).join("") : `<tr><td colspan="2" style="color:var(--dim)">${f.run.running ? "정지선 S를 지나면 표시" : "-"}</td></tr>`;
  const insp = f.inspection;
  $("checks").innerHTML = Object.entries(READY_LABEL).map(([k, n]) => `<span class="chip ${f.readiness[k] ? "ok" : ""}">${n}</span>`).join("");
  kv($("inspect"), [["카메라 자세", `${insp.attitude.status} · 높이 ${insp.attitude.height_m.toFixed(3)} m · roll ${insp.attitude.roll_deg.toFixed(1)}° · pitch ${insp.attitude.pitch_down_deg.toFixed(1)}°`],
    ["차선 모델", `${insp.model.status} ${insp.model.stage || ""}`], ["IMU 안정화", `${insp.stabilization_on ? "켜짐" : "꺼짐"} · ${insp.stabilization}`]]);
  if (f.full) {
    const ang = laneAngle(f.lane);
    const deg = (v) => (v === null ? "-" : (v >= 0 ? "+" : "") + v.toFixed(1) + "°");
    kv($("bevInfo"), [["정지선", f.lane.stop !== null && isFinite(f.lane.stop) ? f.lane.stop.toFixed(2) + " m" : "-"],
      ["차선", f.lane.left.length && f.lane.right.length ? "양쪽" : f.lane.left.length ? "왼쪽만" : f.lane.right.length ? "오른쪽만" : "없음"],
      ["차선 각도 (0.15~0.8 m, + = 왼쪽으로 기울어 보임)", `${deg(ang.mean)} · 2초 평균 ${deg(ang.avg)}`],
      ["  왼쪽 선 / 오른쪽 선", `${deg(ang.left)} / ${deg(ang.right)}`],
      ["정지선 기울기 (+ = 차선과 같은 방향)", f.lane.stop_angle === null || f.lane.stop_angle === undefined || !isFinite(f.lane.stop_angle) ? "-" : deg(f.lane.stop_angle)],
      ["차선 가운데 (0.3 m 앞, + = 차보다 왼쪽)", ang.center === null ? "-" : (ang.center * 100).toFixed(1) + " cm"],
      ["yaw 자동 측정", f.yaw_measure && f.yaw_measure.message ? f.yaw_measure.message : "정지선 S 앞 0.3~1.0 m에 나란히 세우고 [yaw 자동 측정]"]]);
    $("stages").querySelector("tbody").innerHTML = f.stages.map((s) => `<tr><td>${STAGE_LABEL[s.name] || s.name}</td><td>${s.ms.toFixed(1)}</td><td>${s.fps.toFixed(0)}</td></tr>`).join("");
    const u = f.usage; $("usage").innerHTML = usageRow("CPU", u.cpu) + usageRow("이 프로세스", u.proc_cpu) + usageRow("GPU", u.gpu) + usageRow("메모리", u.mem);
    drawBev(f);
  }
  drawMap();
}

// ---------- 상단 입력 ↔ 파라미터 ----------
function pval(name) { const p = params.find((q) => q.name === name); return p ? p.value : null; }
let lastDir = null;
function syncBar() {
  if (document.activeElement !== $("inSpeed")) $("inSpeed").value = pval("run.speed_mps") ?? "";
  if (document.activeElement !== $("inLaps")) $("inLaps").value = pval("run.laps") ?? "";
  const seg = (id, name) => $(id).querySelectorAll("button").forEach((b) => b.classList.toggle("on", Number(b.dataset.v) === pval(name)));
  seg("segStab", "camera.stabilization"); seg("segCorr", "slam.stopline_correction"); seg("segDir", "slam.direction");
  const yaw = pval("camera.yaw_deg") ?? 0;
  if (document.activeElement !== $("inYaw")) $("inYaw").value = Math.abs(yaw).toFixed(2);
  $("segYawSign").querySelectorAll("button").forEach((b) => b.classList.toggle("on", Number(b.dataset.v) === (yaw < 0 ? -1 : 1)));
  const dir = pval("slam.direction");
  if (mapData && dir !== lastDir) { lastDir = dir; drawBase(); }
}

// ---------- 파라미터 창 (cone_drive_test와 같음) ----------
function renderParams() {
  const list = params.filter((p) => p.group === group);
  let html = "", section = null;
  for (const p of list) {
    if (p.section !== section) { section = p.section; html += `<h3>${section}</h3>`; }
    const val = p.name in edits ? edits[p.name] : p.value;
    html += `<div class="prow"><label for="p_${p.name}">${p.label}${p.live ? "" : '<span class="restart">재시작 필요</span>'}<small>${p.name}</small></label>` +
      `<input id="p_${p.name}" data-name="${p.name}" value="${val}" class="${p.name in edits ? "changed" : ""}" inputmode="decimal">` +
      `<span class="unit">${p.unit || ""}</span></div>`;
  }
  $("plist").innerHTML = html;
  $("plist").querySelectorAll("input").forEach((inp) => inp.addEventListener("input", () => {
    const p = params.find((q) => q.name === inp.dataset.name), x = Number(inp.value);
    inp.classList.toggle("bad", !(isFinite(x) && x >= p.min && x <= p.max));
    if (x === p.value) delete edits[p.name]; else edits[p.name] = x;
    inp.classList.toggle("changed", p.name in edits);
    $("pmsg").textContent = Object.keys(edits).length ? `변경 ${Object.keys(edits).length}개 (적용 전)` : "";
  }));
}
function openDrawer(open) { $("drawer").hidden = !open; }

function bind() {
  $("btnStart").addEventListener("click", async () => { const r = await post("/api/start"); if (!r.ok) $("reason").textContent = r.reason; });
  $("btnStop").addEventListener("click", () => post("/api/stop"));
  $("btnRevert").addEventListener("click", async () => {
    if (!confirm("마지막으로 적용한 보정을 이전 값으로 되돌릴까요?")) return;
    const r = await post("/api/action/revert"); $("reason").textContent = r.ok ? "되돌림" : r.reason;
  });
  $("btnReset2").addEventListener("click", async () => {
    if (!confirm("보정 값과 보정 이력을 처음(기본값)으로 초기화할까요?\n지금 값과 이력은 백업 폴더로 옮겨지고, 주행 기록은 그대로 남습니다.")) return;
    const r = await post("/api/action/reset"); $("reason").textContent = r.ok ? "초기화됨 · 백업: " + r.reason : r.reason;
  });
  $("inSpeed").addEventListener("change", () => setParam("run.speed_mps", Number($("inSpeed").value)));
  $("inLaps").addEventListener("change", () => setParam("run.laps", Number($("inLaps").value)));
  $("segStab").querySelectorAll("button").forEach((b) => b.addEventListener("click", () => setParam("camera.stabilization", Number(b.dataset.v))));
  $("segCorr").querySelectorAll("button").forEach((b) => b.addEventListener("click", () => setParam("slam.stopline_correction", Number(b.dataset.v))));
  $("segDir").querySelectorAll("button").forEach((b) => b.addEventListener("click", () => setParam("slam.direction", Number(b.dataset.v))));
  // 카메라 yaw: 크기 + 방향(+ 왼쪽 / − 오른쪽)
  const yawSign = () => ((pval("camera.yaw_deg") ?? 0) < 0 ? -1 : 1);
  $("inYaw").addEventListener("change", () => { const m = Math.abs(Number($("inYaw").value)); if (isFinite(m)) setParam("camera.yaw_deg", yawSign() * m); });
  $("segYawSign").querySelectorAll("button").forEach((b) => b.addEventListener("click", () => setParam("camera.yaw_deg", Number(b.dataset.v) * Math.abs(pval("camera.yaw_deg") ?? 0))));
  $("btnYaw").addEventListener("click", async () => { const r = await post("/api/action/yaw_measure"); $("reason").textContent = r.ok ? "yaw 측정 중 (2초)" : r.reason; });
  $("togBev").addEventListener("change", () => {
    $("bevImg").src = $("togBev").checked && !DEMO ? "/api/stream/bev?client=" + clientId + "&t=" + Date.now() : "";
    post("/api/view", { preview: true, bev: $("togBev").checked, camera: false });
  });
  $("btnParams").addEventListener("click", () => openDrawer($("drawer").hidden));
  $("btnClose").addEventListener("click", () => openDrawer(false));
  $("tabs").querySelectorAll("button").forEach((b) => b.addEventListener("click", () => {
    group = b.dataset.group; $("tabs").querySelectorAll("button").forEach((c) => c.classList.toggle("on", c === b)); renderParams();
  }));
  $("btnReset").addEventListener("click", () => { edits = {}; $("pmsg").textContent = ""; renderParams(); });
  $("btnApply").addEventListener("click", async () => {
    const r = await post("/api/params", { changes: edits }); $("pmsg").textContent = r.ok ? "적용됨" : "거부: " + r.reason;
    if (r.ok) { edits = {}; await loadParams(); }
  });
  $("btnSave").addEventListener("click", async () => { const r = await post("/api/params/save"); $("pmsg").textContent = r.ok ? "저장됨: " + (r.reason || "") : "저장 실패: " + r.reason; });
}

// ---------- 데모 (서버 없이 화면 확인) ----------
function rounded(inset, rad) {
  const a = inset, b = 4 - inset, pts = [];
  const arc = (cx, cy, a0) => { for (let k = 0; k < 16; k++) { const t = a0 + (k / 16) * Math.PI / 2; pts.push([cx + rad * Math.cos(t), cy + rad * Math.sin(t)]); } };
  pts.push([2, a]); arc(b - rad, a + rad, -Math.PI / 2); arc(b - rad, b - rad, 0); arc(a + rad, b - rad, Math.PI / 2); arc(a + rad, a + rad, Math.PI);
  return pts;
}
function demoParams() {
  return [{ name: "run.speed_mps", group: "slam", section: "주행 계획", label: "일정 속도", unit: "m/s", min: 0.2, max: 2.5, value: 1.2, live: true },
    { name: "run.laps", group: "slam", section: "주행 계획", label: "목표 바퀴 수", unit: "", min: 1, max: 30, value: 10, live: true },
    { name: "camera.yaw_deg", group: "slam", section: "카메라", label: "카메라 yaw (+ = 왼쪽)", unit: "°", min: -20, max: 20, value: -2.18, live: true },
    { name: "slam.direction", group: "slam", section: "주행 계획", label: "주행 방향 (0=반시계, 1=시계)", unit: "", min: 0, max: 1, value: 0, live: true },
    { name: "camera.stabilization", group: "slam", section: "매핑·보정", label: "IMU 영상 안정화 (0=끄기, 1=전체)", unit: "", min: 0, max: 1, value: 1, live: true },
    { name: "slam.stopline_correction", group: "slam", section: "매핑·보정", label: "정지선 통과 시 위치 보정", unit: "", min: 0, max: 1, value: 1, live: true }];
}
function demo() {
  mapData = { outer: rounded(0.165, 0.62), inner: rounded(0.835, 0.174), center: rounded(0.5, 0.4), stop: [[1.97, 0.165], [1.97, 0.835]], start: [[2.03, 0.165], [2.03, 0.835]], start_ccw: [[1.0, 0.5], [2.0, 0.5]], start_cw: [[3.0, 0.5], [2.0, 0.5]], lap_length: 10.8, lane_width: 0.67 };
  setupCanvas();
  const mid = rounded(0.5, 0.4); mid.push(mid[0]);
  const seg = []; let L = 0; for (let i = 1; i < mid.length; i++) { const d = Math.hypot(mid[i][0] - mid[i - 1][0], mid[i][1] - mid[i - 1][1]); seg.push([L, i]); L += d; }
  const at = (s) => { s = ((s % L) + L) % L; let i = 1, acc = 0; for (; i < mid.length; i++) { const d = Math.hypot(mid[i][0] - mid[i - 1][0], mid[i][1] - mid[i - 1][1]); if (acc + d >= s) break; acc += d; }
    const a = mid[i - 1], b = mid[Math.min(i, mid.length - 1)], u = (s - acc) / Math.max(1e-6, Math.hypot(b[0] - a[0], b[1] - a[1])); return [a[0] + u * (b[0] - a[0]), a[1] + u * (b[1] - a[1]), Math.atan2(b[1] - a[1], b[0] - a[0])]; };
  let s = 0.03, lap = 0, lapStart = 0; const lapsM = [], corr = [];
  setLink(true); loadParams();
  setInterval(() => {
    s += 0.12; const p = at(s), drift = 0.012 * (s - lapStart);
    const x = p[0] + drift * Math.cos(p[2] + 1.2), y = p[1] + drift * Math.sin(p[2] + 1.2);
    const pts = [];
    for (let k = 0; k < 4; k++) { const q = at(s + 0.3 + k * 0.25); const l = [-Math.sin(q[2]), Math.cos(q[2])];
      pts.push([q[0] + 0.325 * l[0] + drift * 0.8, q[1] + 0.325 * l[1], 0], [q[0] - 0.325 * l[0] + drift * 0.8, q[1] - 0.325 * l[1], 1]); }
    if (s - lapStart > L) { lap++; lapsM.push({ lap, dist: L + 0.04 - 0.01 * lap, s_err: 0.04 - 0.01 * lap, turn: 360.4 - 0.1 * lap, lat: -0.03, corner_fix: 0.04, free_along: 0.12 - 0.03 * lap, free_heading: -5 + lap, rms: 0.052 }); corr.push([lap, -(0.12 - 0.03 * lap)]); lapStart = s; }
    onTelemetry({ state: "DRIVING", reason: "", full: true, readiness: { attitude_ok: true, lane_model_ready: true, vesc_connected: true, erpm_fresh: true, lane_fresh: true, client_alive: true },
      inspection: { attitude: { status: "PASS", height_m: 0.225, roll_deg: 0.2, pitch_down_deg: 6.2, message: "" }, model: { status: "READY", stage: "" }, stabilization: "적용 97%", stabilization_on: true },
      vehicle: { speed: 1.19, target: 1.2, steer_deg: 4.1, cte: 0.013, duty: 0.087, brake: 0, phase: "duty_only_track", lon_mode: 2 },
      run: { running: true, finished: false, lap, laps: 10, target_speed: 1.2, lap_dist: s - lapStart, lap_length: L, bias0: 0.0021, pose: [x, y, p[2]], s_arc: (s - lapStart) % L, lat: -0.03, stop_seen: (L - (s - lapStart)) < 1.6 ? L - (s - lapStart) : null, stop_near: true, lane_width: 0.598, rejected_far: 2, rejected_lap: 0, status: "주행 중", dir: "2026-11-19_1420_v1.20", finishing: false },
      laps: lapsM, corrections: corr, calib: { gyro_scale: 0.9957, gyro_laps: 30, latency: 0, camera_yaw: 0, lap_ref: 11.314, stop_offset: -0.158, stop_laps: 32, stop_offset_cw: -0.224, stop_laps_cw: 8, wheel: [[0.8, 1.0126, 8, false], [1.1, 1.0077, 16, false], [1.4, 0.9922, 8, false], [1.1, 1.0491, 8, true]] },
      result: { applied: true, message: "보정 반영: 자이로 ×0.9957 (누적 30바퀴), 거리 ×1.0001 @1.10 m/s.", laps: 5, gyro_est: 0.9954, wheel_est: 1.0006, turn_per_lap: 359.6, dist_per_lap: 11.307, yaw_residual: 2.3, straight_angle: 8.1, straight_slip: 5.7, straights: 2, stop_offset: -0.161, stop_laps: 8 },
      yaw_measure: { active: false, message: "차선 +2.05° · 정지선 +2.31° → 카메라 yaw −2.18° 적용 (이전 +0.00°)" },
      history: [{ time: "2026-11-19 14:02:11", action: "applied", speed: 1.0, laps: 5, gyro: 0.9953, wheel: 1.0022, yaw_res: 2.1 }, { time: "2026-11-19 14:20:40", action: "skipped", speed: 1.2, laps: 5, gyro: 0.9968, wheel: 0.9966, yaw_res: null }],
      new_track: [[x, y]], new_free: [[x + 0.05 * Math.sin(s), y + 0.04]], new_points: pts,
      lane: { left: Array.from({ length: 40 }, (_, i) => [0.2 + i * 0.05, 0.33 + i * 0.05 * 0.035]), right: Array.from({ length: 40 }, (_, i) => [0.2 + i * 0.05, -0.34 + i * 0.05 * 0.035]), path: [[0, 0], [1.5, -0.01], [2.8, -0.12]], stop_angle: 2.3, stop: (L - (s - lapStart)) < 1.6 ? L - (s - lapStart) : null },
      stages: [{ name: "cam_rgb", ms: 0, fps: 61 }, { name: "receive", ms: 43.9, fps: 63 }, { name: "stabilize_bev", ms: 1.4, fps: 63 }, { name: "lane_infer", ms: 6.0, fps: 62 }, { name: "lane_post", ms: 2.4, fps: 63 }, { name: "control", ms: 0.1, fps: 63 }],
      usage: { cpu: 38, proc_cpu: 11, gpu: 41, mem: 35 } });
  }, 100);
}

bind();
if (DEMO) demo();
else {
  fetch("/api/map").then((r) => r.json()).then((m) => { mapData = m; setupCanvas(); loadRun(); render(); });
  loadParams();
  connect();
}
