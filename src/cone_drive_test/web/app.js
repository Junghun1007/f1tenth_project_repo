// cone_drive_test 브라우저 프리뷰. Jetson은 수치·좌표만 보내고 그림은 여기서 그린다.
// 데모: index.html?demo (demo/frames.json 재생), &mode=detect, &state=INSPECTING, &params=1, &bev=1, &cam=1
"use strict";

const qs = new URLSearchParams(location.search);
const DEMO = qs.has("demo");
const $ = (id) => document.getElementById(id);
// 표시 범위 (m): BEV 영상은 x 0~3 m, y ±0.6 m (120×300, 1 px = 1 cm). 아래 0.5 m는 차량 표시용 여백
const BEV = { xMax: 3.0, xMin: -0.5, yHalf: 0.6, imageXMin: 0.0, pxPerM: 200 };
const STAGE_LABEL = {
  cam_rgb: "카메라 RGB 수신", cam_depth: "카메라 Depth 수신",
  receive: "영상 지연(촬영→처리)", stabilize_bev: "안정화+BEV", lane_infer: "차선 추론", lane_post: "중앙선",
  yolo_pre: "YOLO 전처리", yolo_infer: "YOLO 추론", yolo_post: "YOLO 후처리", box_depth: "Depth 정합",
  memory_plan: "기억+회피계획", control: "제어", web: "웹 전송",
  lane_side_fix: "⚠ 차선 좌우 교정(회/s)", lane_side_drop: "⚠ 차선 조각 제외(회/s)",
  depth_miss: "⚠ 깊이 짝 없음(회/s)", loc_fail: "⚠ 위치 계산 실패(회/s)",
};
const READY_LABEL = {
  attitude_ok: "카메라 자세", lane_model_ready: "차선 모델", yolo_model_ready: "YOLO 모델",
  vesc_connected: "VESC", erpm_fresh: "ERPM", lane_fresh: "차선 입력", client_alive: "브라우저",
};
const PLAN_LABEL = { center: "중앙선 추종", lane: "차선 안 회피", "lane-tight": "차선 안(여유 0)", free: "차선 밖 회피", fallback: "대체 경로" };

let latest = null;
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

function connect() {
  const es = new EventSource("/api/events");
  es.onmessage = (ev) => { latest = JSON.parse(ev.data); setLink(true); render(); };
  es.onerror = () => setLink(false);
  setInterval(() => post("/api/heartbeat", { client: clientId }), 200);
  // 이 브라우저의 보기 설정을 서버에 알림 (서버는 켠 클라이언트가 있을 때만 프리뷰 데이터를 만든다)
  post("/api/view", { preview: $("togPreview").checked, bev: $("togBev").checked, camera: $("togCam").checked });
}

function setLink(ok) {
  const el = $("link");
  el.textContent = ok ? "연결됨" : "연결 끊김";
  el.className = "link" + (ok ? " ok" : "");
}

async function loadParams() {
  const url = DEMO ? "demo/params.json" : "/api/params";
  try { params = (await (await fetch(url)).json()).params; } catch (e) { params = []; }
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
  polyline(f.lane.left, "#3b82f6", 3);
  polyline(f.lane.right, "#ef4444", 3);
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
    box(d.x, d.y, d.length, d.width, d.yaw || 0, "rgba(251,146,60,.85)", "#fb923c");
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

function render() {
  const f = latest;
  if (!f) return;
  const previewOn = $("togPreview").checked;
  $("main").hidden = !previewOn; $("offNote").hidden = previewOn;
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
  document.querySelectorAll("#modeSeg button").forEach((b) => {
    b.classList.toggle("on", b.dataset.mode === f.mode);
    b.disabled = !["READY", "MANUAL"].includes(f.state);
  });
  const stopped = ["READY", "MANUAL"].includes(f.state);
  $("btnStart").disabled = !(f.mode === "drive" && stopped && Object.values(f.readiness).every(Boolean));
  $("btnStop").disabled = !["DRIVING", "STOPPING"].includes(f.state);
  $("btnRefit").disabled = !["READY", "MANUAL", "DRIVING"].includes(f.state);
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
    f.inspection.models.map((m) => `<div><span>${m.name}</span><b class="${m.status}">${m.status}</b><span>${m.status === "READY" ? m.elapsed_s.toFixed(1) + " s" : m.stage + " · " + m.elapsed_s.toFixed(0) + " s"}</span></div>`).join("") +
    `<div><span>IMU 영상 안정화</span><b class="${(f.stabilization || "").startsWith("적용") ? "READY" : "LOADING"}">${(f.stabilization || "").startsWith("적용") ? "ON" : "…"}</b><span>${f.stabilization || "-"}</span></div>`;

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
  $("detTable").querySelector("tbody").innerHTML = f.detections.map((d) =>
    `<tr><td>${d.cls}</td><td>${d.score.toFixed(2)}</td><td>${d.x.toFixed(2)}</td><td>${d.y.toFixed(2)}</td></tr>`).join("");
  drawBev(f);
}

// ---------- 파라미터 창 ----------
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
    const p = params.find((q) => q.name === inp.dataset.name);
    const x = Number(inp.value);
    inp.classList.toggle("bad", !(isFinite(x) && x >= p.min && x <= p.max));
    if (x === p.value) delete edits[p.name]; else edits[p.name] = x;
    inp.classList.toggle("changed", p.name in edits);
    $("pmsg").textContent = Object.keys(edits).length ? `변경 ${Object.keys(edits).length}개 (적용 전)` : "";
  }));
}

function openDrawer(open) { $("drawer").hidden = !open; }

// ---------- 이벤트 ----------
function bind() {
  document.querySelectorAll("#modeSeg button").forEach((b) => b.addEventListener("click", async () => {
    const r = await post("/api/mode", { mode: b.dataset.mode }); if (!r.ok) $("reason").textContent = r.reason;
  }));
  $("btnStart").addEventListener("click", async () => { const r = await post("/api/start"); if (!r.ok) $("reason").textContent = r.reason; });
  $("btnStop").addEventListener("click", () => post("/api/stop"));
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
    if (!confirm("차량을 정지한 뒤 카메라 자세를 다시 측정합니다. 평평한 바닥에 세워 두세요.")) return;
    const r = await post("/api/refit"); if (!r.ok) $("reason").textContent = r.reason;
  });
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
    const r = await post("/api/params", { changes: edits });
    $("pmsg").textContent = r.ok ? "적용됨" : "거부: " + r.reason;
    if (r.ok) { edits = {}; await loadParams(); }
  });
  $("btnSave").addEventListener("click", async () => {
    const r = await post("/api/params/save"); $("pmsg").textContent = r.ok ? "저장됨: " + (r.reason || "") : "저장 실패: " + r.reason;
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
    return g;
  };
  const step = () => { latest = tweak(frames[i]); render(); if (!qs.has("still")) i = (i + 1) % frames.length; };
  setLink(true); step();
  if (!qs.has("still")) setInterval(step, 50);
}

bind();
$("bevImg").addEventListener("load", () => render());   // 첫 영상이 늦게 와도 배경을 바로 그림
try { if (localStorage.getItem("cdt_preview") === "0" || qs.get("preview") === "0") $("togPreview").checked = false; } catch (e) { /* 저장소 없음 */ }
if (qs.get("bev") === "1") { $("togBev").checked = true; $("togBev").dispatchEvent(new Event("change")); }
if (qs.get("cam") === "1") { $("togCam").checked = true; $("togCam").dispatchEvent(new Event("change")); }
loadParams().then(() => { if (qs.get("params") === "1") openDrawer(true); });
if (DEMO) demo(); else connect();
