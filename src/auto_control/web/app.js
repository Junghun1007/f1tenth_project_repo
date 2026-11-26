// auto_control 브라우저 화면 (cone_drive_test/web/app.js에서 시작). Jetson은 수치·좌표만 보내고 그림은 여기서 그린다.
// 데모: index.html?demo (demo/frames.json 재생), &mode=detect, &state=INSPECTING, &params=1, &bev=1, &cam=1,
//       &scene=red (빨간불 정지 접근) | green | lead (앞차 추종) | tunnel (IR 터널), &handover=MANUAL
"use strict";

const qs = new URLSearchParams(location.search);
const DEMO = qs.has("demo");
const $ = (id) => document.getElementById(id);
// 표시 범위 (m): BEV 영상은 x 0~3 m, y ±0.6 m (120×300, 1 px = 1 cm). 아래 0.5 m는 차량 표시용 여백
const BEV = { xMax: 3.0, xMin: -0.5, yHalf: 0.6, imageXMin: 0.0, pxPerM: 200 };
const STAGE_LABEL = {
  cam_rgb: "카메라 RGB 수신", cam_depth: "카메라 Depth 수신",
  receive: "영상 지연(촬영→처리)", stabilize_bev: "안정화+BEV", lane_infer: "차선 추론", lane_post: "중앙선",
  lane_pre: "차선 전처리(H2D)", lane_geom: "차선 형상(경계·중앙선)", lane_latency: "차선 지연(촬영→제어)",
  yolo_pre: "YOLO 전처리", yolo_infer: "YOLO 추론", yolo_post: "YOLO 후처리", box_depth: "Depth 정합",
  memory_plan: "기억+회피계획", control: "제어", web: "웹 전송",
  lane_side_fix: "⚠ 차선 좌우 교정(회/s)", lane_side_drop: "⚠ 차선 조각 제외(회/s)",
  lane_reject: "차선 검사 모드 2 · 입력 보류(회/s)",
  depth_miss: "⚠ 깊이 짝 없음(회/s)", loc_fail: "⚠ 위치 계산 실패(회/s)", ground_reject: "⚠ 지면 거리 불일치로 버림(회/s)",
};
const READY_LABEL = {
  attitude_ok: "카메라 자세", lane_model_ready: "차선 모델", yolo_model_ready: "YOLO 모델",
  vesc_connected: "VESC", erpm_fresh: "ERPM", lane_fresh: "차선 입력", client_alive: "브라우저",
  ir_model_ready: "IR 차선 모델", handover_ready: "안전 전환",
};
const STATE_LABEL = { BOOT: "시작 중", INSPECTING: "출발 전 점검", READY: "출발 대기", DRIVING: "자동 주행", STOPPING: "정지 중",
  MANUAL: "수동 (조이스틱)", REFITTING: "카메라 재측정" };
const SIGNAL_NAME = ["UNKNOWN", "RED", "GREEN"];
const SIGNAL_KO = ["없음", "빨강", "초록"];
const PLAN_LABEL = { center: "중앙선 추종", lane: "차선 안 회피", "lane-tight": "차선 안(여유 0)", free: "차선 밖 회피", fallback: "대체 경로" };

let latest = null;
let forceIrRequest = false;
let cameraResetRequest = false;
let cameraResetReconnect = false;
let cameraResetPending = false;
let connected = false;
let params = [];
let edits = {};
let group = "obstacle";
let clientId = "";
try { clientId = localStorage.getItem("cdt_client") || ""; } catch (e) { /* 저장소 없음 */ }
if (!clientId) {
  clientId = Math.random().toString(36).slice(2, 10);
  try { localStorage.setItem("cdt_client", clientId); } catch (e) { /* 저장소 없음 */ }
}

// ---------- 서버 통신 ----------
async function post(path, body) {
  if (DEMO) return { ok: true, reason: "" };
  if (path === "/api/view") body = Object.assign({ client: clientId }, body || {});
  try {
    const r = await fetch(path, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body || {}) });
    return await r.json();
  } catch (e) {
    return { ok: false, reason: "요청 실패: " + e.message };
  }
}

async function restartCamera(path, body) {
  if (cameraResetRequest) return { ok: false, reason: "카메라 초기화 중입니다" };
  cameraResetRequest = true;
  $("btnCameraReset").disabled = true;
  $("cameraResetStatus").textContent = "카메라 재시작 요청 중";
  const result = await post(path, body);
  if (!cameraResetReconnect && cameraResetRequest) {
    if (!result.ok) cameraResetRequest = false;
    $("cameraResetStatus").textContent = result.ok ? "카메라 재시작 중" : result.reason;
  }
  return result;
}

function connect() {
  const es = new EventSource("/api/events");
  es.onmessage = (ev) => { latest = JSON.parse(ev.data); setLink(true); render(); };
  es.onerror = () => setLink(false);
  setInterval(() => post("/api/heartbeat", { client: clientId }), 200);
  // 이 브라우저의 보기 설정을 서버에 알림 (서버는 켠 클라이언트가 있을 때만 프리뷰 데이터를 만든다)
  post("/api/view", { preview: $("togPreview").checked, bev: $("togBev").checked, camera: $("togCam").checked });
}

function setLink(ok) {
  if (!ok && cameraResetRequest) cameraResetReconnect = true;
  if (ok && cameraResetReconnect) {
    cameraResetRequest = cameraResetReconnect = cameraResetPending = false;
    $("cameraResetStatus").textContent = "카메라 시작 보정 중";
    post("/api/view", { preview: $("togPreview").checked, bev: $("togBev").checked, camera: $("togCam").checked });
    for (const [toggle, img] of [["togBev", "bevImg"], ["togCam", "camImg"]]) {
      if ($(toggle).checked) {
        $(img).removeAttribute("src");
        $(toggle).dispatchEvent(new Event("change"));
      }
    }
    if (!$("drawer").hidden) loadParams();
  }
  connected = ok;
  if (!ok) {
    $("btnForceIr").disabled = true;
    $("btnCameraReset").disabled = true;
    $("tcnStatus").textContent = "연결 끊김";
  }
  const el = $("link");
  el.textContent = ok ? "연결됨" : "연결 끊김";
  el.className = "link" + (ok ? " ok" : "");
}

async function loadParams() {
  const url = DEMO ? "demo/params.json" : "/api/params";
  try { params = (await (await fetch(url, { cache: "no-store" })).json()).params; } catch (e) { params = []; }
  renderParams();
}

// ---------- BEV 캔버스 ----------
const cv = $("bev");
const ctx = cv.getContext("2d");
const scale = BEV.pxPerM;                                 // 오버레이·배경 공통 축척 (px/m)
cv.width = Math.round(2 * BEV.yHalf * scale);             // 캔버스 크기를 표시 범위에서 계산 (CSS로 늘이지 않음)
cv.height = Math.round((BEV.xMax - BEV.xMin) * scale);
function P(x, y) { return [(BEV.yHalf - y) * scale, (BEV.xMax - x) * scale]; }
// 배경 BEV 영상을 그릴 캔버스 영역: 오버레이와 같은 P로 계산해 항상 겹친다
function bevImageRect() {
  const [x0, y0] = P(BEV.xMax, BEV.yHalf), [x1, y1] = P(BEV.imageXMin, -BEV.yHalf);
  return { x: x0, y: y0, w: x1 - x0, h: y1 - y0 };
}

function polyline(pts, color, width, dash) {
  if (!pts || pts.length < 2) return;
  ctx.save();
  ctx.strokeStyle = color; ctx.lineWidth = width; ctx.lineJoin = "round"; ctx.setLineDash(dash || []);
  ctx.beginPath();
  pts.forEach(([x, y], i) => { const [u, v] = P(x, y); i ? ctx.lineTo(u, v) : ctx.moveTo(u, v); });
  ctx.stroke(); ctx.restore();
}

function lanePoints(points, color) {
  if (!points?.length) return;
  const radius = 2;
  ctx.fillStyle = color;
  ctx.beginPath();
  for (const [x, y] of points) {
    const [u, v] = P(x, y);
    ctx.moveTo(u + radius, v);
    ctx.arc(u, v, radius, 0, 2 * Math.PI);
  }
  ctx.fill();
}

function box(x, y, len, wid, yaw, fill, stroke, dash) {
  const c = Math.cos(yaw), s = Math.sin(yaw);
  const pts = [[-1, -1], [1, -1], [1, 1], [-1, 1]].map(([a, b]) => [x + a * len / 2 * c - b * wid / 2 * s, y + a * len / 2 * s + b * wid / 2 * c]);
  ctx.save();
  ctx.beginPath();
  pts.forEach(([px, py], i) => { const [u, v] = P(px, py); i ? ctx.lineTo(u, v) : ctx.moveTo(u, v); });
  ctx.closePath();
  if (fill) { ctx.fillStyle = fill; ctx.fill(); }
  if (stroke) { ctx.strokeStyle = stroke; ctx.lineWidth = 2; ctx.setLineDash(dash || []); ctx.stroke(); }
  ctx.restore();
}

function drawBev(f) {
  const bg = $("togBev").checked;
  ctx.fillStyle = "#0a0d10";
  ctx.fillRect(0, 0, cv.width, cv.height);
  const img = $("bevImg");
  if (bg && img.complete && img.naturalWidth > 0) {   // MJPEG의 현재 프레임을 같은 좌표계에 그림
    const r = bevImageRect();
    ctx.drawImage(img, r.x, r.y, r.w, r.h);
  }
  // 0.5 m 격자
  ctx.save();
  ctx.strokeStyle = bg ? "rgba(255,255,255,.18)" : "#1d242c"; ctx.fillStyle = bg ? "rgba(255,255,255,.7)" : "#55606b";
  ctx.font = "11px sans-serif"; ctx.lineWidth = 1;
  for (let x = 0; x <= BEV.xMax + 1e-6; x += 0.5) {
    const [, v] = P(x, 0); ctx.beginPath(); ctx.moveTo(0, v); ctx.lineTo(cv.width, v); ctx.stroke();
    ctx.fillText(x.toFixed(1) + " m", 4, v - 3);
  }
  for (let y = -0.5; y <= 0.5; y += 0.25) { const [u] = P(0, y); ctx.beginPath(); ctx.moveTo(u, 0); ctx.lineTo(u, cv.height); ctx.stroke(); }
  ctx.restore();
  if (!f) return;
  const lanePrefix = f.tunnel?.source === "IR" ? "ir_lane." : "lane.";
  const laneOpacity = params.find((p) => p.name === lanePrefix + "overlay_alpha")?.value ?? 0.75;
  ctx.save();
  ctx.globalAlpha = laneOpacity;
  lanePoints(f.lane.left, "#3b82f6");
  lanePoints(f.lane.right, "#ef4444");
  ctx.restore();
  polyline(f.lane.path, "#facc15", 2, [8, 6]);
  for (const m of f.memory) box(m.x, m.y, 0.13, 0.13, 0, null, "#f472b6", [4, 3]);
  // 화면 밖 기억 콘: 가장자리에 화살표 + 거리 (계획이 왜 켜졌는지 보이게)
  let off = 0;
  for (const m of f.memory) {
    const inside = m.x >= BEV.xMin && m.x <= BEV.xMax && Math.abs(m.y) <= BEV.yHalf;
    if (inside) continue;
    off++;
    const cx = Math.min(BEV.xMax - 0.05, Math.max(BEV.xMin + 0.05, m.x)), cy = Math.min(BEV.yHalf - 0.03, Math.max(-BEV.yHalf + 0.03, m.y));
    const [u, v] = P(cx, cy);
    const [u0, v0] = P(0, 0);
    const a = Math.atan2(v - v0, u - u0);
    ctx.save(); ctx.fillStyle = "#f472b6"; ctx.beginPath();
    ctx.moveTo(u + 7 * Math.cos(a), v + 7 * Math.sin(a));
    ctx.lineTo(u + 7 * Math.cos(a + 2.4), v + 7 * Math.sin(a + 2.4)); ctx.lineTo(u + 7 * Math.cos(a - 2.4), v + 7 * Math.sin(a - 2.4));
    ctx.closePath(); ctx.fill();
    ctx.font = "10px sans-serif"; ctx.fillText(`${m.x.toFixed(1)},${m.y.toFixed(1)}`, Math.min(Math.max(u - 18, 2), cv.width - 46), v + (v > cv.height / 2 ? -9 : 16));
    ctx.restore();
  }
  for (const d of f.detections) {
    const veh = d.cls === "truck" || d.cls === "sedan";   // 차량은 빨강, 콘은 주황
    box(d.x, d.y, d.length, d.width, d.yaw || 0, veh ? "rgba(244,63,94,.55)" : "rgba(251,146,60,.85)", veh ? "#f43f5e" : "#fb923c");
    const [u, v] = P(d.x + d.length / 2, d.y);
    ctx.fillStyle = "#fff"; ctx.font = "11px sans-serif"; ctx.fillText(d.score.toFixed(2), u + 8, v);
  }
  if (f.traffic && f.traffic.enabled && f.traffic.line > -90) {   // 정지선 (앞범퍼 기준 거리 → 앞차축 +5 cm)
    const [, v] = P(0.05 + f.traffic.line, 0);
    ctx.save(); ctx.strokeStyle = "#22c55e"; ctx.lineWidth = 5; ctx.beginPath(); ctx.moveTo(0, v); ctx.lineTo(cv.width, v); ctx.stroke();
    ctx.fillStyle = "#22c55e"; ctx.font = "bold 12px sans-serif";
    ctx.fillText(`정지선 ${f.traffic.line.toFixed(2)} m${f.traffic.holding ? " · 정지" : f.traffic.committed ? " · 정지 중" : ""}`, 6, v - 6); ctx.restore();
  }
  if (f.lead && f.lead.active) {   // 차량 추종: 앞범퍼(앞차축 +5 cm)에서 거리만큼 앞에 빨간 정지선
    const x = 0.05 + f.lead.gap, [, v] = P(x, 0);
    ctx.save(); ctx.strokeStyle = f.lead.seen ? "#f43f5e" : "#fda4af"; ctx.lineWidth = 3; ctx.setLineDash(f.lead.seen ? [] : [6, 4]);
    ctx.beginPath(); ctx.moveTo(0, v); ctx.lineTo(cv.width, v); ctx.stroke();
    ctx.fillStyle = "#f43f5e"; ctx.font = "bold 12px sans-serif";
    ctx.fillText(`앞차 ${f.lead.gap.toFixed(2)} m${f.lead.holding ? " · 정지 유지" : ""}`, cv.width - 150, v - 5); ctx.restore();
  }
  if (f.plan && f.plan.path && f.plan.mode !== "center") {
    polyline(f.plan.path, "#22d3ee", 3, f.plan.rollout_ok ? [] : [6, 5]);   // 롤아웃 실패 = 점선
    if (!f.plan.rollout_ok) { const e = f.plan.path[f.plan.path.length - 1]; const [u, v] = P(e[0], e[1]);
      ctx.fillStyle = "#22d3ee"; ctx.font = "bold 12px sans-serif"; ctx.fillText("롤아웃 FAIL", Math.min(u, cv.width - 80), v + 14); }
  }
  {   // 지금 계획 모드와 기억 콘 수 (화면 밖 포함)
    const mode = f.plan ? (PLAN_LABEL[f.plan.mode] || f.plan.mode) : "-";
    ctx.save(); ctx.font = "bold 12px sans-serif";
    ctx.fillStyle = f.plan && f.plan.mode !== "center" ? "#22d3ee" : "rgba(255,255,255,.7)";
    ctx.fillText(`계획: ${mode}`, 6, cv.height - 24);
    ctx.fillStyle = "#f472b6";
    ctx.fillText(`기억 콘 ${f.memory.length}${off ? ` (화면 밖 ${off})` : ""} · 관측 ${f.detections.length}`, 6, cv.height - 8);
    ctx.restore();
  }
  // 차량 외곽 (앞차축 원점, 길이 0.50, 폭 0.30, 앞 오버행 0.05)
  box(0.05 - 0.25, 0, 0.5, 0.3, 0, "rgba(59,130,246,.25)", "#60a5fa");
  const steer = (f.vehicle.steer_deg || 0) * Math.PI / 180;
  polyline([[0, 0], [0.25 * Math.cos(steer), 0.25 * Math.sin(steer)]], "#fff", 2);
}

// ---------- 패널 ----------
function kv(el, rows) {
  el.innerHTML = rows.map(([k, v]) => `<span class="k">${k}</span><span class="v">${v}</span>`).join("");
}

function usageRow(name, value) {
  const ok = value !== null && value !== undefined;
  const pct = ok ? Math.max(0, Math.min(100, value)) : 0;
  return `<div class="bar-row"><span>${name}</span><div class="track"><div class="fill ${pct > 85 ? "hot" : ""}" style="width:${pct}%"></div></div><span>${ok ? value.toFixed(0) + " %" : "--"}</span></div>`;
}

// ---------- 터널 TCN ----------
const tunnelChart = $("tcnChart");
const tcnCtx = tunnelChart.getContext("2d");
const chartStyle = getComputedStyle($("tcnPanel"));
const tcnColors = ["--tcn-entry", "--tcn-exit", "--tcn-switch", "--dim", "--line"]
  .map((name) => chartStyle.getPropertyValue(name).trim());
const tcnWindow = 12;
let tcnTime = 0;
let tcnSamples = [];
let laneSwitches = [];

function drawTunnelChart(threshold) {
  const width = tunnelChart.clientWidth, height = tunnelChart.clientHeight;
  if (!width || !height) return;
  const ratio = window.devicePixelRatio || 1;
  const pixelWidth = Math.round(width * ratio), pixelHeight = Math.round(height * ratio);
  if (tunnelChart.width !== pixelWidth || tunnelChart.height !== pixelHeight) {
    tunnelChart.width = pixelWidth;
    tunnelChart.height = pixelHeight;
  }
  const c = tcnCtx;
  c.setTransform(ratio, 0, 0, ratio, 0, 0);
  c.clearRect(0, 0, width, height);
  const left = 33, right = width - 18, top = 13, bottom = height - 23;
  const start = tcnTime - tcnWindow;
  const x = (t) => left + (t - start) / tcnWindow * (right - left);
  const y = (score) => bottom - score * (bottom - top);
  const [entryColor, exitColor, switchColor, labelColor, gridColor] = tcnColors;
  const line = (x1, y1, x2, y2) => {
    c.beginPath(); c.moveTo(x1, y1); c.lineTo(x2, y2); c.stroke();
  };

  c.lineWidth = 1;
  c.strokeStyle = gridColor;
  c.fillStyle = labelColor;
  c.font = "11px system-ui";
  c.textAlign = "right";
  c.textBaseline = "middle";
  for (const score of [0, 0.5, 1]) {
    line(left, y(score), right, y(score));
    c.fillText(score.toFixed(1), left - 7, y(score));
  }
  c.textAlign = "center";
  for (const offset of [12, 9, 6, 3, 0]) {
    const pos = x(tcnTime - offset);
    line(pos, top, pos, bottom);
    c.fillText(offset ? `−${offset}s` : "현재", pos, bottom + 12);
  }
  if (Number.isFinite(threshold)) {
    c.strokeStyle = labelColor;
    c.setLineDash([3, 4]);
    line(left, y(threshold), right, y(threshold));
    c.setLineDash([]);
  }

  c.save();
  c.beginPath();
  c.rect(left, top - 5, right - left, bottom - top + 10);
  c.clip();
  for (const [index, color] of [[1, entryColor], [2, exitColor]]) {
    c.strokeStyle = color;
    c.lineWidth = 1.7;
    c.beginPath();
    let lastTime = -Infinity;
    for (const sample of tcnSamples) {
      const [t] = sample;
      if (t - lastTime > 0.5) c.moveTo(x(t), y(sample[index]));
      else c.lineTo(x(t), y(sample[index]));
      lastTime = t;
    }
    c.stroke();
  }
  for (const [t, entry, exit, event, forced] of tcnSamples) {
    if (!event) continue;
    const isEntry = event === "entry";
    c.strokeStyle = c.fillStyle = isEntry ? entryColor : exitColor;
    c.save();
    c.translate(x(t), y(isEntry ? entry : exit));
    c.rotate(Math.PI / 4);
    if (forced) c.strokeRect(-3.6, -3.6, 7.2, 7.2);
    else c.fillRect(-3.6, -3.6, 7.2, 7.2);
    c.restore();
  }
  c.setLineDash([4, 4]);
  c.lineWidth = 1.5;
  c.strokeStyle = c.fillStyle = switchColor;
  for (const [t, source] of laneSwitches) {
    const pos = x(t);
    line(pos, top + 11, pos, bottom);
    c.textAlign = pos > right - 34 ? "right" : "left";
    c.fillText(source, pos + (c.textAlign === "left" ? 4 : -4), top + 4);
  }
  c.restore();
}

function selectedLaneSource(tunnel) {
  if (tunnel.state === "INSIDE") return "IR";
  if (tunnel.state === "OUTSIDE") return "RGB";
  return tunnel.source || "RGB";
}

function renderTunnel(tunnel, chart, laneFresh) {
  $("tcnPanel").hidden = !tunnel.enabled;
  if (!tunnel.enabled) return;
  if (chart) {
    if (chart.now < tcnTime) { tcnSamples = []; laneSwitches = []; }
    tcnTime = chart.now;
    for (const sample of chart.samples) {
      if (!tcnSamples.length || sample[0] > tcnSamples.at(-1)[0]) tcnSamples.push(sample);
    }
    if (Number.isFinite(tunnel.source_changed_s) &&
        (!laneSwitches.length || tunnel.source_changed_s > laneSwitches.at(-1)[0])) {
      laneSwitches.push([tunnel.source_changed_s, tunnel.source]);
    }
    const cutoff = tcnTime - tcnWindow;
    tcnSamples = tcnSamples.filter(([t]) => t >= cutoff - 0.5);
    laneSwitches = laneSwitches.filter(([t]) => t >= cutoff);
  }
  const status = chart?.status || "WAITING";
  const last = tcnSamples.at(-1);
  const fresh = last && tcnTime - last[0] <= 0.5;
  $("tcnStatus").textContent = !connected ? "연결 끊김" : { WAITING: "결과 대기", STALE: "결과 수신 끊김" }[status] || status;
  $("tcnEntry").textContent = fresh ? last[1].toFixed(2) : "—";
  $("tcnExit").textContent = fresh ? last[2].toFixed(2) : "—";
  $("tcnThreshold").textContent = Number.isFinite(chart?.threshold) ? chart.threshold.toFixed(2) : "—";
  const decision = fresh ? ({ inside: "터널 안", outside: "터널 밖" }[chart?.state] || "—") : "—";
  $("tcnDecision").textContent = "TCN 판정 " + decision + (fresh && chart?.state_override ? " (강제)" : "");

  const source = tunnel.source || "RGB";
  const selected = selectedLaneSource(tunnel);
  const waiting = tunnel.state === "ENTERING" && source !== "IR" ? "유효 IR 경로 대기"
    : selected !== source ? `첫 ${selected} 결과 수신 대기` : "";
  const lane = $("tcnLane");
  const irStatus = source === "IR" && selected === "IR" ? tunnel.ir_lane : null;
  const detail = waiting || ({
    stale: "IR 결과 수신 끊김", invalid: "IR 중앙선 무효", valid: laneFresh ? "IR 중앙선 유효" : "유효 제어 경로 없음",
  }[irStatus] || "");
  lane.dataset.pending = Boolean(waiting || (irStatus && (irStatus !== "valid" || !laneFresh)));
  lane.textContent = [tunnel.state || "—", `차선 선택 ${selected}`, detail].filter(Boolean).join(" · ");
  drawTunnelChart(chart?.threshold);
}

function renderCameraExposure(values, tunnelEnabled) {
  const rgbModes = { auto: "자동", manual: "수동", fixed_iso: "노출 자동 · ISO 고정",
    restoring: "출구 노출 복원 중", auto_pending: "자동 복귀 중" };
  for (const [id, reading, infrared] of [
    ["exposureRgb", values?.rgb, false],
    ["exposureRgbStartup", values?.rgb_startup, false],
    ["exposureIrLeft", values?.ir_left, true],
    ["exposureIrRight", values?.ir_right, true],
  ]) {
    const el = $(id);
    const ms = Number.isFinite(reading?.ms) ? `${reading.ms.toFixed(2)} ms` : "—";
    const showIso = infrared || id === "exposureRgbStartup" || reading?.mode === "fixed_iso";
    const iso = showIso && Number.isFinite(reading?.iso) ? `ISO ${reading.iso}` : "";
    el.parentElement.hidden = infrared && !tunnelEnabled;
    el.textContent = [ms, iso, rgbModes[reading?.mode], reading?.stale ? "갱신 없음" : ""].filter(Boolean).join(" · ");
    el.classList.toggle("stale", Boolean(reading?.stale));
  }
  $("exposureIrFlood").textContent = Number.isFinite(values?.ir_flood) ? `${values.ir_flood.toFixed(2)} · 명령값` : "—";
  $("exposureIrFlood").parentElement.hidden = !tunnelEnabled;
}

function render() {
  const f = latest;
  if (!f) return;
  const previewOn = $("togPreview").checked;
  const tunnel = f.tunnel || {};
  $("main").hidden = !previewOn && !tunnel.enabled && !f.camera_exposure;
  $("main").classList.toggle("preview-off", !previewOn);
  $("offNote").hidden = previewOn;
  $("togBev").disabled = $("togCam").disabled = !previewOn;
  const st = $("stateBadge");
  st.textContent = f.state; st.className = "state " + f.state;
  $("reason").textContent = f.reason || "";
  if (f.traffic) {   // 신호등 칩 (켜져 있을 때만)
    const chip = $("sigChip"), name = ["UNKNOWN", "RED", "GREEN"][f.traffic.signal] || "UNKNOWN";
    chip.hidden = !f.traffic.enabled;
    chip.className = "sig " + name;
    chip.textContent = "신호 " + ["-", "빨강", "초록"][f.traffic.signal] + (f.traffic.committed ? " · 정지" : "");
  }
  if (f.handover !== undefined) {   // safety_node 모드: AUTO / STOPPING / MANUAL
    const h = $("handoverChip"); h.className = "chipx " + f.handover;
    h.textContent = "전환 " + ({ AUTO: "AUTO", STOPPING: "제동 중", MANUAL: "MANUAL" }[f.handover] || f.handover);
  }
  const src = tunnel.source || "RGB";
  const forceIr = $("btnForceIr");
  const stopped = ["READY", "MANUAL"].includes(f.state);
  const irPreparing = tunnel.force_ir && (src !== "IR" || tunnel.state !== "INSIDE");
  forceIr.hidden = $("forceIrStatus").hidden = !tunnel.enabled;
  forceIr.disabled = !connected || !stopped || forceIrRequest || !tunnel.force_ir_change_allowed;
  forceIr.setAttribute("aria-pressed", Boolean(tunnel.force_ir));
  forceIr.classList.toggle("go", Boolean(tunnel.force_ir));
  forceIr.textContent = forceIrRequest ? "터널 IR 요청 중" : irPreparing ? "터널 IR 준비 중" :
    tunnel.force_ir ? "터널 IR 강제 ON" : "터널 IR 강제 OFF";
  const selected = selectedLaneSource(tunnel);
  $("sourceChip").className = "chipx " + selected;
  $("sourceChip").textContent = selected === "IR" ? "IR 터널" : "RGB";
  renderTunnel(tunnel, f.tcn, f.readiness.lane_fresh);
  $("cameraExposurePanel").hidden = !f.camera_exposure;
  renderCameraExposure(f.camera_exposure, tunnel.enabled);
  renderTunnelMode(tunnel);
  document.querySelectorAll("#modeSeg button").forEach((b) => {
    b.classList.toggle("on", b.dataset.mode === f.mode);
    b.disabled = !stopped;
  });
  $("btnStart").disabled = forceIrRequest || irPreparing || cameraResetRequest || f.camera_reset_pending ||
    !(f.mode === "drive" && stopped && Object.values(f.readiness).every(Boolean));
  $("btnStop").disabled = !["DRIVING", "STOPPING"].includes(f.state);
  $("btnRefit").disabled = !["READY", "MANUAL", "DRIVING"].includes(f.state);
  $("btnRefit").hidden = f.refit_supported === false;   // auto_control: 카메라 자세는 시작할 때 bev_processor가 측정
  $("btnCameraReset").hidden = !f.camera_reset_supported;
  $("btnCameraReset").disabled = !connected || !f.camera_reset_allowed || cameraResetRequest || f.camera_reset_pending;
  if (cameraResetRequest && cameraResetPending && !f.camera_reset_pending) {
    cameraResetRequest = false;
    $("cameraResetStatus").textContent = "카메라 초기화 취소됨";
  }
  cameraResetPending = Boolean(f.camera_reset_pending);
  if (f.inspection?.attitude?.status === "PASS" && $("cameraResetStatus").textContent === "카메라 시작 보정 중") {
    $("cameraResetStatus").textContent = "카메라 시작 보정 완료";
  }
  if (f.record) {
    $("btnRec").disabled = f.record.active; $("btnRecStop").disabled = !f.record.active; $("recLabel").disabled = f.record.active;
    if (f.record.active) $("recStatus").textContent = `● 녹화 중 ${f.record.seconds.toFixed(0)} s · ${f.record.frames} 프레임`;
  }
  $("camPanel").hidden = !$("togCam").checked;

  const chips = Object.entries(f.readiness).map(([k, v]) => `<span class="chip ${v ? "ok" : ""}">${READY_LABEL[k] || k}</span>`).join("");
  $("mini").innerHTML = chips;
  if (!previewOn || !f.vehicle) return;          // 프리뷰 꺼짐: 서버가 상태만 보냄
  $("checks").innerHTML = Object.entries(f.readiness).map(([k, v]) => `<span class="chip ${v ? "ok" : ""}">${READY_LABEL[k] || k}</span>`).join("");
  const a = f.inspection.attitude;
  $("models").innerHTML =
    `<div><span>카메라 자세</span><b class="${a.status === "PASS" ? "READY" : a.status === "FAIL" ? "FAILED" : "LOADING"}">${a.status}</b>` +
    `<span>${a.status === "PASS" ? `높이 ${a.height_m.toFixed(3)} m · roll ${a.roll_deg.toFixed(1)}° · pitch ${a.pitch_down_deg.toFixed(1)}°` : a.message}</span></div>` +
    f.inspection.models.map((m) => `<div><span>${m.name}</span><b class="${m.status}">${m.status}</b><span>${m.status === "READY" ? (m.stage && m.stage !== "ready" ? m.stage : m.elapsed_s.toFixed(1) + " s") : m.stage + " · " + m.elapsed_s.toFixed(0) + " s"}</span></div>`).join("") +
    (f.stabilization === "camera_driver" ? "" :
    `<div><span>IMU 영상 안정화</span><b class="${(f.stabilization || "").startsWith("적용") ? "READY" : "LOADING"}">${(f.stabilization || "").startsWith("적용") ? "ON" : "…"}</b><span>${f.stabilization || "-"}</span></div>`);

  renderDriveState(f);
  renderTraffic(f);
  const v = f.vehicle;
  kv($("vehicle"), [["속도", v.speed.toFixed(2) + " m/s"], ["목표", v.target.toFixed(2) + " m/s"], ["조향", v.steer_deg.toFixed(1) + "°"],
    ["CTE", (v.cte * 100).toFixed(1) + " cm"], ["duty", v.duty.toFixed(3)], ["제동", v.brake.toFixed(1) + " A"],
    ["종방향", f.lon ? (f.lon.mode === 2 ? "duty-only" : "staged") + (f.lon.requested !== f.lon.mode ? " (정지 시 전환)" : "") : "-"], ["단계", f.lon ? f.lon.phase : "-"]]);
  const plan = f.plan || {};
  const planMs = (f.stages.find((s) => s.name === "memory_plan") || {}).ms;
  kv($("avoid"), [["계획", PLAN_LABEL[plan.mode] || plan.mode || "-"], ["롤아웃", plan.mode === "center" ? "-" : plan.rollout_ok ? "OK" : "FAIL → 감속"],
    ["관측 콘", f.detections.length], ["기억 콘", f.memory.length], ["기억 방식", f.memory_mode === 2 ? (f.lock_m > 0 ? `보이는 콘 · 고정 ${f.lock_m.toFixed(2)} m` : "보이는 콘") : f.memory_mode === 3 ? "기억 없음" : f.memory_mode === 1 ? "차선 기준 단기" : "월드"], ["계획 시간", planMs !== undefined ? planMs.toFixed(1) + " ms" : "-"],
    ["앞차", f.lead && f.lead.active ? `${f.lead.gap.toFixed(2)} m · ${f.lead.speed.toFixed(2)} m/s${f.lead.seen ? "" : " (놓침·유지)"}` : "-"],
    ["신호", f.traffic && f.traffic.enabled ? ["-", "빨강", "초록"][f.traffic.signal] + (f.traffic.committed ? (f.traffic.holding ? " · 정지 유지" : " · 정지 중") : "") : "꺼짐"],
    ["정지선", f.traffic && f.traffic.line > -90 ? f.traffic.line.toFixed(2) + " m" : "-"],
    ["추종 상한", f.lead && f.lead.active && f.lead.cap >= 0 ? (f.lead.holding ? "정지 유지" : f.lead.cap.toFixed(2) + " m/s") : "-"]]);
  $("stages").querySelector("tbody").innerHTML = f.stages.map((s) =>
    `<tr><td>${STAGE_LABEL[s.name] || s.name}</td><td>${s.ms.toFixed(1)}</td><td>${s.fps.toFixed(0)}</td></tr>`).join("");
  const u = f.usage;
  $("usage").innerHTML = usageRow("CPU 전체", u.cpu) + usageRow("이 프로세스", u.proc_cpu) + usageRow("GPU", u.gpu) + usageRow("메모리", u.mem);
  let detectionRows = f.detections.map((d) =>
    `<tr><td>${d.cls}</td><td>${d.score.toFixed(2)}</td><td>${d.x.toFixed(2)}</td><td>${d.y.toFixed(2)}</td></tr>`).join("");
  if (!detectionRows && f.stages.some((s) => s.name === "depth_miss")) {
    const receivingDepth = f.stages.some((s) => s.name === "cam_depth");
    const reason = receivingDepth ? "RGB와 깊이 촬영 시각을 맞추지 못함" : "최근 1초간 스테레오 깊이 수신 없음";
    detectionRows = `<tr><td colspan="4">거리 미측정 · ${reason}</td></tr>`;
  }
  $("detTable").querySelector("tbody").innerHTML = detectionRows;
  drawBev(f);
}

// ---------- 주행 상태 · 신호등 ----------
function renderDriveState(f) {
  const ds = $("dsState");
  ds.textContent = STATE_LABEL[f.state] || f.state; ds.className = "state big " + f.state;
  const cap = [];
  if (f.traffic && f.traffic.committed) cap.push("신호등 정지");
  if (f.lead && f.lead.active) cap.push("앞차 추종");
  if (f.plan && f.plan.mode && f.plan.mode !== "center") cap.push("콘 회피");
  $("dsReason").textContent = f.reason || (cap.length ? "속도 제한: " + cap.join(" · ") : "");
  const t = f.tunnel || {};
  kv($("driveKv"), [["모드", f.mode === "drive" ? "자동 주행" : "검출 테스트"], ["안전 전환", f.handover || "-"],
    ["차선 입력", selectedLaneSource(t) === "IR" ? "IR (터널)" : "RGB"], ["터널", t.enabled ? (t.state || "-") : "꺼짐"],
    ["조이스틱", f.joystick ? "연결됨" : "없음 (X 정지 불가)"], ["주행 시간", f.drive_s !== undefined ? f.drive_s.toFixed(0) + " s" : "-"]]);
}

function renderTraffic(f) {
  const tr = Object.assign({ enabled: false, signal: 0, line: -99 }, f.traffic || {});
  if (f.yolo) { tr.score = f.yolo.signal_score; tr.fps = f.yolo.fps; }   // 실제 서버는 YOLO 항목에 따로 보냄
  const name = SIGNAL_NAME[tr.signal] || "UNKNOWN";
  $("lamp").className = "lamp " + (tr.enabled ? name : "");
  const ir = f.tunnel && f.tunnel.source === "IR";
  const phase = ir ? "터널 구간 (YOLO 쉼)" : !tr.enabled ? "꺼짐" : tr.holding ? "정지 유지 (초록 대기)" : tr.committed ? "정지 접근"
    : tr.passing ? "통과 (못 설 거리)" : tr.red_pending ? "빨간불 · 정지선 찾는 중" : tr.line > -90 ? "정지선 추적" : "대기";
  kv($("trafficKv"), [["신호", tr.enabled ? SIGNAL_KO[tr.signal] + (tr.score ? ` (${tr.score.toFixed(2)})` : "") : "꺼짐"],
    ["단계", phase], ["정지선", tr.line > -90 ? tr.line.toFixed(2) + " m (앞범퍼)" : "-"],
    ["속도 상한", tr.committed && tr.cap >= 0 ? (tr.holding ? "0 (정지)" : tr.cap.toFixed(2) + " m/s") : "-"],
    ["검출 FPS", !ir && tr.fps !== undefined ? tr.fps.toFixed(0) : "-"]]);
  // 정지선 막대: 0~2 m를 왼쪽→오른쪽, 초록 = 허용 정차 범위(정지선 앞 0~stop_range), 파랑 = 앞범퍼
  const span = 2.0, range = tr.stop_range || 0.25;
  const track = $("stopTrack"), car = $("stopCar"), zone = $("stopZone");
  const ok = tr.enabled && tr.line > -90;
  track.style.opacity = ok ? 1 : 0.35;
  zone.style.left = ((span - range) / span * 100) + "%"; zone.style.width = (range / span * 100) + "%";
  const pos = ok ? Math.max(0, Math.min(span, span - tr.line)) : 0;
  car.style.left = `calc(${pos / span * 100}% - 6px)`;
  $("stopLabel").textContent = ok ? `앞범퍼 → 정지선 ${tr.line.toFixed(2)} m · 초록 구간 안에서 정차` : "정지선 없음";
}

// ---------- 파라미터 창 ----------
function renderTunnelMode(tunnel) {
  const el = $("tunnelModeNow");
  if (!el) return;
  el.textContent = tunnel?.enabled && typeof tunnel.validation_mode === "boolean"
    ? `현재 실행 중 ${tunnel.validation_mode}` : tunnel?.enabled === false ? "터널 기능 꺼짐" : "실행 상태 확인 중";
}

function renderParams() {
  const panel = $("plist");
  const previouslyOpen = new Set([...panel.querySelectorAll("details[open]")].map((el) => el.dataset.section));
  const sections = new Map();
  for (const p of params.filter((p) => p.group === group)) {
    if (!sections.has(p.section)) sections.set(p.section, []);
    sections.get(p.section).push(p);
  }
  const parameterRow = (p) => {
    const val = p.name in edits ? edits[p.name] : p.value;
    const attrs = `id="p_${p.name}" data-name="${p.name}" class="${p.name in edits ? "changed" : ""}"`;
    const control = p.options
      ? `<select ${attrs}>${p.options.map((o) => `<option value="${o.value}"${o.value === val ? " selected" : ""}>${o.label}</option>`).join("")}</select>`
      : `<input ${attrs} value="${val}" inputmode="decimal">`;
    return `<div class="prow"><label for="p_${p.name}">${p.label}${p.live ? "" : '<span class="restart">재시작 필요</span>'}<small>${p.name}</small></label>` + control +
      (p.options ? "" : `<span class="unit">${p.unit || ""}</span>`) + "</div>";
  };
  panel.innerHTML = [...sections].map(([section, list]) => {
    const rows = list.map(parameterRow).join("");
    if (list.some((p) => p.name === "tunnel.validation_mode")) {
      return `<h3>${section}</h3>${rows}<div class="param-help">
        <strong id="tunnelModeNow"></strong>
        <p><b>true</b> · 독립 IR 테스트 기준. 새 IR 프레임을 바로 처리합니다.</p>
        <p><b>false</b> · 노출 변경 후 150 ms를 기다린 뒤, 실제 노출이 ±500 µs, ISO가 ±5% 범위 안인 프레임을 사용합니다.</p>
        <p>적용을 누르면 실행 중에 반영됩니다. 다음 실행에서도 쓰려면 YAML을 저장하고 <code>drive_params_file</code>로 지정합니다.</p>
      </div>`;
    }
    if (group !== "lane") return `<h3>${section}</h3>${rows}`;
    const open = previouslyOpen.has(section) || (!previouslyOpen.size && section === "RGB · 검출");
    const weight = list.find((p) => p.name.endsWith(".corner_outer_weight"));
    const preset = weight ? `<button type="button" class="btn lane-preset" data-lane-preset="${weight.name.split(".")[0]}">곡률 반영 강화</button>` : "";
    return `<details class="lane-param-section" data-section="${section}"${open ? " open" : ""}><summary>${section}</summary>${rows}${preset}</details>`;
  }).join("");
  renderTunnelMode(latest?.tunnel);
  const rgbMode = panel.querySelector('[data-name="rgb_cam.auto_exposure"]');
  if (rgbMode) {
    const exposure = panel.querySelector('[data-name="rgb_cam.exposure_us"]');
    const iso = panel.querySelector('[data-name="rgb_cam.iso"]');
    const updateRgbInputs = () => {
      exposure.disabled = rgbMode.value !== "0";
      iso.disabled = rgbMode.value === "1";
    };
    updateRgbInputs();
    rgbMode.addEventListener("input", updateRgbInputs);
  }
  panel.querySelectorAll("[data-name]").forEach((inp) => inp.addEventListener("input", () => {
    const p = params.find((q) => q.name === inp.dataset.name);
    const x = Number(inp.value);
    inp.classList.toggle("bad", !(isFinite(x) && x >= p.min && x <= p.max));
    if (x === p.value) delete edits[p.name]; else edits[p.name] = x;
    inp.classList.toggle("changed", p.name in edits);
    $("pmsg").textContent = Object.keys(edits).length ? `변경 ${Object.keys(edits).length}개 (적용 전)` : "";
  }));
  panel.querySelectorAll("[data-lane-preset]").forEach((button) => button.addEventListener("click", () => {
    const prefix = button.dataset.lanePreset + ".";
    for (const [key, value] of Object.entries({
      corner_outer_weight: 1, corner_outer_min_turn_deg: 6, corner_outer_full_turn_deg: 18,
    })) {
      const name = prefix + key;
      if (params.find((p) => p.name === name).value === value) delete edits[name];
      else edits[name] = value;
    }
    $("pmsg").textContent = Object.keys(edits).length ? `변경 ${Object.keys(edits).length}개 (적용 전)` : "";
    renderParams();
  }));
}

function openDrawer(open) {
  $("drawer").hidden = !open;
  if (open) {
    loadParams();
  }
}

// ---------- 이벤트 ----------
function bind() {
  document.querySelectorAll("#modeSeg button").forEach((b) => b.addEventListener("click", async () => {
    const r = await post("/api/mode", { mode: b.dataset.mode }); if (!r.ok) $("reason").textContent = r.reason;
  }));
  $("btnStart").addEventListener("click", async () => { const r = await post("/api/start"); if (!r.ok) $("reason").textContent = r.reason; });
  $("btnStop").addEventListener("click", () => post("/api/stop"));
  $("btnForceIr").addEventListener("click", async () => {
    if ($("btnForceIr").disabled) return;
    const enabled = !latest.tunnel.force_ir;
    forceIrRequest = true;
    $("forceIrStatus").textContent = "";
    render();
    const reply = await post("/api/tunnel/force-ir", { enabled });
    forceIrRequest = false;
    if (reply.ok) latest.tunnel.force_ir = enabled;
    else $("forceIrStatus").textContent = reply.reason;
    render();
  });
  // 수동 주행 녹화: 사람이 달린 경로 + 같은 순간 계획기 경로·차선·콘을 남겨 파라미터를 찾는 데 쓴다
  $("btnRec").addEventListener("click", async () => {
    const r = await post("/api/record/start", { label: $("recLabel").value });
    $("recStatus").textContent = r.ok ? "녹화 중" : r.reason;
  });
  $("btnRecStop").addEventListener("click", async () => {
    const r = await post("/api/record/stop");
    $("recStatus").textContent = r.ok ? "저장: " + r.reason : r.reason;
  });
  $("btnRefit").addEventListener("click", async () => {
    if (!confirm("Refit: 주행 중이면 먼저 멈춘 뒤, 카메라 높이·기울기를 다시 측정하고 콘·차량 기억을 모두 지웁니다.\n평평한 바닥에 세우고 몇 초간 움직이지 마세요.")) return;
    const r = await post("/api/refit"); if (!r.ok) $("reason").textContent = r.reason;
  });
  $("btnCameraReset").addEventListener("click", () => restartCamera("/api/camera/reset"));
  $("togBev").addEventListener("change", () => {
    const on = $("togBev").checked;
    const img = $("bevImg");
    img.src = on ? (DEMO ? "demo/bev.jpg" : "/api/stream/bev") : "";
    post("/api/view", { bev: on }); render();
  });
  $("togPreview").addEventListener("change", () => {
    const on = $("togPreview").checked;
    try { localStorage.setItem("cdt_preview", on ? "1" : "0"); } catch (e) { /* 저장소 없음 */ }
    if (!on) for (const id of ["togBev", "togCam"]) if ($(id).checked) { $(id).checked = false; $(id).dispatchEvent(new Event("change")); }
    post("/api/view", { preview: on }); render();
  });
  $("togCam").addEventListener("change", () => {
    const on = $("togCam").checked;
    $("camImg").src = on ? (DEMO ? "demo/camera.jpg" : "/api/stream/camera") : "";
    post("/api/view", { camera: on }); render();
  });
  $("btnParams").addEventListener("click", () => openDrawer($("drawer").hidden));
  $("btnClose").addEventListener("click", () => openDrawer(false));
  $("tabs").querySelectorAll("button").forEach((b) => b.addEventListener("click", () => {
    group = b.dataset.group;
    $("tabs").querySelectorAll("button").forEach((c) => c.classList.toggle("on", c === b));
    renderParams();
  }));
  $("btnReset").addEventListener("click", () => { edits = {}; $("pmsg").textContent = ""; renderParams(); });
  $("btnApply").addEventListener("click", async () => {
    const changed = params.filter((p) => p.name in edits);
    const r = await post("/api/params", { changes: edits });
    let message = "적용됨";
    if (changed.some((p) => !p.live)) message = "변경 저장 대기 · YAML 저장 후 재실행";
    else if (changed.some((p) => p.name.startsWith("tcn."))) {
      message = latest?.tunnel?.enabled ? "TCN 설정 전송됨" : "TCN 설정 변경됨 · YAML 저장 후 터널 실행";
    }
    $("pmsg").textContent = r.ok ? message : "거부: " + r.reason;
    if (r.ok) { edits = {}; await loadParams(); }
  });
  $("btnSave").addEventListener("click", async () => {
    if (Object.keys(edits).length) { $("pmsg").textContent = "변경값을 적용한 뒤 저장하세요"; return; }
    const r = await post("/api/params/save", {});
    $("pmsg").textContent = r.ok ? "저장됨 · " + r.reason : "저장 실패 · " + r.reason;
  });
}

// ---------- 데모 재생 ----------
async function demo() {
  const data = await (await fetch("demo/frames.json")).json();
  const frames = data.frames;
  let i = qs.has("still") ? data.focus : 0;
  const tweak = (f) => {
    const g = JSON.parse(JSON.stringify(f));
    if (qs.get("mode") === "detect") {
      g.mode = "detect"; g.state = "MANUAL"; g.plan = { path: [], mode: "center", rollout_ok: true };
      g.vehicle.target = 0; g.vehicle.duty = 0.04;
    }
    if (qs.get("state") === "INSPECTING") {
      g.state = "INSPECTING"; g.lane = { left: [], right: [], center: [], path: [] }; g.detections = []; g.memory = [];
      g.plan = { path: [], mode: "center", rollout_ok: true };
      Object.assign(g.readiness, { attitude_ok: false, yolo_model_ready: false, lane_fresh: false });
      g.inspection.attitude = { status: "MEASURING", height_m: 0, roll_deg: 0, pitch_down_deg: 0, message: "지면 측정 중 31/45 프레임" };
      g.inspection.models[1] = { name: "YOLOX-tiny 512x320", status: "LOADING", stage: "TensorRT 엔진 생성", elapsed_s: 48, message: "" };
      g.vehicle = { speed: 0, target: 0, steer_deg: 0, cte: 0, duty: 0, brake: 0 };
    }
    const scene = qs.get("scene");
    g.handover = qs.get("handover") || (g.state === "DRIVING" || g.state === "STOPPING" ? "AUTO" : g.state === "MANUAL" ? "MANUAL" : "AUTO");
    g.joystick = !qs.has("nojoy");
    g.drive_s = g.t;
    g.tunnel = { enabled: true, source: "RGB", state: "OUTSIDE", validation_mode: true };
    g.readiness.handover_ready = true;
    g.lon = { mode: 2, requested: 2, phase: g.vehicle.brake > 0 ? "제동" : g.vehicle.duty > 0 ? "구동" : "관성" };
    g.stabilization = "camera_driver";
    g.refit_supported = false;
    g.stages = [
      { name: "lane_latency", ms: 9.8, fps: 70 }, { name: "lane_pre", ms: 0.9, fps: 70 }, { name: "lane_infer", ms: 3.2, fps: 70 },
      { name: "lane_geom", ms: 1.6, fps: 70 }, { name: "yolo_pre", ms: 0.5, fps: 40 }, { name: "yolo_infer", ms: 7.0, fps: 40 },
      { name: "yolo_post", ms: 0.3, fps: 40 }, { name: "box_depth", ms: 0.8, fps: 40 }, { name: "memory_plan", ms: 0.7, fps: 70 },
      { name: "control", ms: 0.1, fps: 70 }, { name: "web", ms: 0.4, fps: 20 }];
    g.traffic = { enabled: true, signal: 0, committed: false, holding: false, line: -99, cap: -1, stop_range: 0.25 };
    g.yolo = { fps: 40, signal_score: 0 };
    if (scene === "red" || scene === "green") {
      g.detections = []; g.memory = []; g.plan = { path: [], mode: "center", rollout_ok: true };
      const red = scene === "red";
      g.traffic = { enabled: true, signal: red ? 1 : 2, committed: red, holding: false, line: red ? 0.62 : 1.4, cap: red ? 0.98 : -1,
        stop_range: 0.25 };
      g.yolo = { fps: 40, signal_score: 0.91 };
      g.vehicle.speed = red ? 0.74 : 1.0; g.vehicle.target = red ? 0.62 : 1.05; g.vehicle.steer_deg = 2.1; g.vehicle.duty = red ? 0.0 : 0.06;
    } else if (scene === "lead") {
      g.memory = []; g.plan = { path: [], mode: "center", rollout_ok: true };
      g.detections = [{ cls: "sedan", score: 0.88, x: 1.15, y: 0.02, length: 0.42, width: 0.2, yaw: 0 }];
      g.lead = { active: true, seen: true, gap: 0.68, speed: 0.55, cap: 0.52, holding: false };
      g.vehicle.speed = 0.57; g.vehicle.target = 0.52;
    } else if (scene === "tunnel") {
      const tick = 50 + Math.floor(i / 2), now = tick / 10;
      const state = tick < 62 || tick >= 91 ? "OUTSIDE" : "INSIDE";
      g.tunnel = { enabled: true, source: tick >= 63 && tick < 92 ? "IR" : "RGB", state, ir_lane: "valid", validation_mode: true,
        source_changed_s: tick >= 92 ? 9.2 : tick >= 63 ? 6.3 : null };
      const peak = (j, center, width) => Math.exp(-0.5 * ((j - center) / width) ** 2);
      const samples = Array.from({ length: tick + 1 }, (_, j) => [
        j / 10, 0.06 + 0.87 * peak(j, 63, 1.6), 0.04 + 0.83 * peak(j, 92, 1.5),
        j === 62 ? "entry" : j === 91 ? "exit" : null,
      ]);
      g.tcn = { now, samples, threshold: 0.5, status: "LIVE", state: tick >= 62 && tick < 91 ? "inside" : "outside" };
      g.camera_exposure = { rgb: { ms: 8.2, mode: "auto" }, rgb_startup: { ms: 6.1, iso: 260 },
        ir_left: { ms: 19.98, iso: 500 }, ir_right: { ms: 20, iso: 500 }, ir_flood: 1 };
      g.detections = []; g.memory = [];
      g.plan = { path: [], mode: "center", rollout_ok: true }; g.traffic.enabled = false;
      g.stages = g.stages.filter((x) => !x.name.startsWith("yolo") && x.name !== "box_depth");
      g.inspection.models.push({ name: "IR 차선", status: "READY", stage: "ready", elapsed_s: 4.2, message: "" });
      g.readiness.ir_model_ready = true;
    }
    return g;
  };
  const step = () => { latest = tweak(frames[i]); render(); if (!qs.has("still")) i = (i + 1) % frames.length; };
  setLink(true); step();
  if (!qs.has("still")) setInterval(step, 50);
}

bind();
new ResizeObserver(() => drawTunnelChart(latest?.tcn?.threshold)).observe(tunnelChart);
$("bevImg").addEventListener("load", () => render());   // 첫 영상이 늦게 와도 배경을 바로 그림
try { if (localStorage.getItem("cdt_preview") === "0" || qs.get("preview") === "0") $("togPreview").checked = false; } catch (e) { /* 저장소 없음 */ }
if (qs.get("bev") === "1") { $("togBev").checked = true; $("togBev").dispatchEvent(new Event("change")); }
if (qs.get("cam") === "1") { $("togCam").checked = true; $("togCam").dispatchEvent(new Event("change")); }
if (qs.get("params") === "1") openDrawer(true);
else loadParams();
if (DEMO) demo(); else connect();
