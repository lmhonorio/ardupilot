const PAGES = [
  ["logs", "Logs"],
  ["mensagens", "Mensagens"],
  ["modos", "Modos"],
  ["parametros", "Parâmetros"],
  ["editor-missao", "Missão"],
  ["scripts", "Scripts"],
  ["ensaios", "Ensaios"],
];

const CHARTS = [
  {
    title: "Corrente", unit: "A",
    series: [
      ["ESC1_I", "M1", "#073e73", []],
      ["ESC2_I", "M2", "#2d6da3", []],
      ["ESC3_I", "M3", "#6aa0c8", []],
      ["ESC4_I", "M4", "#0e2c4f", []],
      ["ESC1_ICMD", "M1 cmd", "#073e73", [4, 3]],
      ["ESC2_ICMD", "M2 cmd", "#2d6da3", [4, 3]],
      ["ESC3_ICMD", "M3 cmd", "#6aa0c8", [4, 3]],
      ["ESC4_ICMD", "M4 cmd", "#0e2c4f", [4, 3]],
      ["BAT_I", "bateria", "#122033", [2, 3]],
    ],
  },
  {
    title: "Rotação", unit: "rpm",
    series: [
      ["ESC1_RPM", "M1", "#073e73", []],
      ["ESC2_RPM", "M2", "#2d6da3", []],
      ["ESC3_RPM", "M3", "#6aa0c8", []],
      ["ESC4_RPM", "M4", "#0e2c4f", []],
      ["ESC_TGT_L", "alvo esq", "#122033", [2, 2]],
      ["ESC_TGT_R", "alvo dir", "#122033", [6, 3]],
    ],
  },
  {
    title: "Taxa de guinada", unit: "graus/s",
    series: [
      ["PIDS_des", "desejada", "#073e73", []],
      ["PIDS_ach", "real", "#2d6da3", []],
      ["ATT_yawrate", "giro", "#8aa4b8", []],
    ],
  },
  {
    title: "Desvio da trilha", unit: "m",
    series: [["NAV_xtrack", "xtrack", "#073e73", []]],
  },
  {
    title: "Temperatura dos motores", unit: "°C",
    series: [
      ["ESC1_T", "M1", "#073e73", []],
      ["ESC2_T", "M2", "#2d6da3", []],
      ["ESC3_T", "M3", "#6aa0c8", []],
      ["ESC4_T", "M4", "#0e2c4f", []],
    ],
  },
];

let catalog = null;
let paramDocs = {};
let state = {};
let telem = { t: 0, window: 20, series: {} };
let telemAt = 0;
let shown = {};
let frameAt = 0;
let page = "logs";
let confirm = null;
let scriptBusy = false;
let scriptWanted = false;
let scriptXferTimer = null;
let sdlogBusy = false;
let sdlogWanted = false;
let sdlogXferTimer = null;
let sdlogCache = [];
let missionBusy = false;
let missionDirty = false;
let missionSelected = null;
let missionItems = [];
let missionStats = { total_m: 0, legs: [], waypoints: 0 };
let missionXferTimer = null;
let missionMap = null;
let missionMarkers = {};
let missionLines = null;
let missionRoverMarker = null;
let missionHeadingLayer = null;
let missionClickMode = "add"; // add | pan
let missionPlanMode = "map"; // map | relative
let missionCtx = null; // {x,y,lat,lon,seq}
let missionCtxIgnoreClick = false; // próximo clique no mapa só fecha o menu
let missionRelOrigin = null; // {lat, lon, hdg} fixed start for relative steps
let missionRelPreview = null; // leaflet layers
let missionRelPreviewPt = null; // [lat, lon]
const CMD_WAYPOINT = 16;
const CMD_RTL = 20;
const CMD_LOITER_UNLIM = 17;
const CMD_LOITER_TIME = 19;
const CMD_DO_JUMP = 177;
const NAV_WITH_POSITION = new Set([16, 17, 19, 21, 22, 82]);
const DEFAULT_MAP = [-15.78, -47.93];

function applyTheme(theme) {
  document.documentElement.dataset.theme = theme;
  localStorage.setItem("grin-theme", theme);
  const btn = document.getElementById("theme");
  if (btn) btn.textContent = theme === "dark" ? "Claro" : "Escuro";
  const argo = document.querySelector(".logo.argo");
  const grin = document.querySelector(".logo.grin");
  if (argo) argo.src = theme === "dark" ? "/argo-dark.png" : "/argo.png";
  if (grin) grin.src = theme === "dark" ? "/grin-dark.png" : "/grin.png";
}
applyTheme(localStorage.getItem("grin-theme") === "dark" ? "dark" : "light");
document.getElementById("theme").addEventListener("click", () => {
  applyTheme(document.documentElement.dataset.theme === "dark" ? "light" : "dark");
});

function cssVar(name) {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
}

const view = document.getElementById("view");
const nav = document.getElementById("nav");

function fmt(v, digits) {
  if (v == null || Number.isNaN(Number(v))) return "—";
  const n = Number(v);
  if (Math.abs(n - Math.round(n)) < 1e-5) return String(Math.round(n));
  const d = digits == null ? 4 : digits;
  return String(Math.round(n * 10 ** d) / 10 ** d);
}

function pageId() {
  const id = (location.hash || "#logs").slice(1);
  if (["direcao", "velocidade", "motores", "missao", "script"].includes(id)) return "parametros";
  return PAGES.some(([k]) => k === id) ? id : "logs";
}

function renderNav() {
  nav.innerHTML = PAGES.map(([id, label]) =>
    `<a href="#${id}" ${id === page ? 'aria-current="page"' : ""}>${label}</a>`).join("");
}

async function post(url, body) {
  const res = await fetch(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
  let data = {};
  try { data = await res.json(); } catch (e) { /* resposta vazia */ }
  if (data.note) document.getElementById("note").textContent = data.note;
  pollState();
  return res.ok;
}

function armButton() {
  const btn = document.getElementById("armbtn");
  if (!btn) return;
  const seen = !!state.seen;
  const armed = !!state.armed;
  const pending = confirm && confirm.kind === "arm" && Date.now() < confirm.until;
  btn.disabled = !seen;
  if (armed) {
    btn.textContent = "Desarmar";
    btn.className = "cmd solid";
  } else if (pending) {
    btn.textContent = "Confirmar armação";
    btn.className = "cmd solid";
  } else {
    btn.textContent = "Armar";
    btn.className = "cmd";
  }
}

function paint() {
  if (confirm && confirm.el && Date.now() >= confirm.until) {
    confirm.el.textContent = confirm.label || "Preparar";
    confirm.el.className = confirm.className || "cmd proc";
    confirm = null;
  }
  const alive = !!state.alive;
  document.getElementById("link").textContent = state.link || "—";
  const mode = document.getElementById("mode");
  mode.textContent = state.mode || "—";
  mode.style.color = alive || !state.seen ? cssVar("--ink") : cssVar("--muted");
  const armed = document.getElementById("armed");
  if (!state.seen || state.armed == null) {
    armed.textContent = "—";
    armed.className = "";
  } else if (state.armed) {
    armed.textContent = "ARMADO";
    armed.className = "on";
  } else {
    armed.textContent = "DESARMADO";
    armed.className = "";
  }
  const problem = latestProblem();
  const problemEl = document.getElementById("problem");
  if (problemEl) problemEl.textContent = problem ? problem.text : "";
  if (problem) armed.classList.add("bad");
  const note = document.getElementById("note");
  if (document.activeElement !== note) note.textContent = state.note || "";
  armButton();
  paintRecording();
  const last = document.getElementById("lasttext");
  if (last) last.textContent = state.text || "";
  paintMessages();
  takeEpoch();
  if (page === "parametros") paintParams();
  for (const el of document.querySelectorAll("[data-live]")) {
    const input = el.parentElement.querySelector("input");
    const name = el.dataset.live;
    const value = state.params ? state.params[name] : undefined;
    const article = el.closest("article");
    el.textContent = "na placa " + fmt(value);
    if (name in pending && value != null && closeValue(value, pending[name])) {
      delete pending[name];
    }
    if (name in pending) {
      if (input && document.activeElement !== input) input.value = fmt(pending[name]);
      if (input) input.dataset.touched = "1";
      if (article) article.classList.add("dirty");
    } else if (article) {
      article.classList.remove("dirty");
      if (input && document.activeElement !== input && value != null && input.dataset.touched !== "1") {
        input.value = fmt(value);
      }
    }
  }
  const blocked = new Set(state.blocked_modes || []);
  for (const btn of document.querySelectorAll("button.mode")) {
    btn.disabled = !state.seen || blocked.has(btn.dataset.mode);
    btn.classList.toggle("on", alive && btn.dataset.mode === state.mode);
  }
  if (state.reset_inputs) {
    pending = {};
    for (const input of document.querySelectorAll("input[data-touched]")) input.dataset.touched = "";
  }
  for (const btn of document.querySelectorAll("button.apply, button.proc, button.scriptact, button.sdlogact, button.missionact")) {
    if (btn.id === "armbtn") continue;
    const pending = confirm && confirm.el === btn && Date.now() < confirm.until;
    const busy = (btn.classList.contains("scriptact") && scriptBusy)
      || (btn.classList.contains("sdlogact") && sdlogBusy)
      || (btn.classList.contains("missionact") && missionBusy);
    const needsDisarm = btn.dataset.danger === "1";
    const needsLink = !btn.classList.contains("missionlocal");
    if (!pending) {
      btn.disabled = (needsLink && !state.seen) || busy || (!!state.armed && needsDisarm);
    }
  }
  if (page === "editor-missao") {
    paintRoverOnMap();
    if (missionPlanMode === "relative") paintRelativePreview();
    const dirty = document.getElementById("missiondirty");
    if (dirty && !missionBusy) dirty.textContent = missionDirty ? "alterações locais não enviadas" : "";
  }
  paintTools();
  if (page === "scripts" && scriptWanted && state.seen && !scriptBusy) {
    scriptWanted = false;
    loadScripts();
  }
  if (page === "logs" && sdlogWanted && state.seen && !sdlogBusy) {
    sdlogWanted = false;
    loadSdLogs();
  }
}

function drawChart(canvas, spec, data) {
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth || 300;
  const h = canvas.clientHeight || 168;
  const pw = Math.max(1, Math.floor(w * dpr));
  const ph = Math.max(1, Math.floor(h * dpr));
  if (canvas.width !== pw || canvas.height !== ph) {
    canvas.width = pw;
    canvas.height = ph;
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);
  const L = 48, R = 10, T = 8, B = 18;
  ctx.strokeStyle = cssVar("--plot");
  ctx.strokeRect(L, T, Math.max(0, w - L - R), Math.max(0, h - T - B));
  const now = viewTime();
  const drawn = spec.series
    .map(([name, label, color, dash]) => ({
      name, label, color, dash,
      pts: smoothTrace((data.series && data.series[name]) || [], now),
    }))
    .filter((s) => s.pts.length);
  const legend = canvas.parentElement.querySelector(".legend");
  if (legend) {
    const legendText = drawn.map((s) => s.label).join("   ");
    if (legend.textContent !== legendText) legend.textContent = legendText;
  }
  if (!drawn.length || !data.t) {
    ctx.fillStyle = cssVar("--muted");
    ctx.font = "14px Segoe UI";
    ctx.fillText("sem dados", L + 12, T + 28);
    return;
  }
  const t1 = now || data.t;
  const t0 = t1 - (data.window || 20);
  let ymin = Infinity;
  let ymax = -Infinity;
  for (const s of drawn) {
    for (const p of s.pts) {
      if (p[1] < ymin) ymin = p[1];
      if (p[1] > ymax) ymax = p[1];
    }
  }
  // At rest the CSV is ±0.3 rpm and ±0.2 A. A tight scale draws that as a full-height trace.
  const minSpan = { A: 4, rpm: 20, "graus/s": 20, m: 1, "°C": 10 }[spec.unit] || 0;
  if (!Number.isFinite(ymin) || !Number.isFinite(ymax) || ymin === ymax) {
    ymin = -minSpan / 2;
    ymax = minSpan / 2;
  }
  if (ymax - ymin < minSpan) {
    const mid = (ymin <= 0 && ymax >= 0) ? 0 : (ymin + ymax) / 2;
    ymin = mid - minSpan / 2;
    ymax = mid + minSpan / 2;
  } else {
    const pad = (ymax - ymin) * 0.12;
    ymin -= pad;
    ymax += pad;
  }
  const X = (t) => L + ((t - t0) / (t1 - t0)) * (w - L - R);
  const Y = (v) => T + ((ymax - v) / (ymax - ymin)) * (h - T - B);
  ctx.fillStyle = cssVar("--muted");
  ctx.font = "11px Segoe UI";
  ctx.fillText(ymax.toFixed(1), 2, T + 11);
  ctx.fillText(ymin.toFixed(1), 2, h - B);
  if (ymin < 0 && ymax > 0) {
    ctx.strokeStyle = cssVar("--plot-zero");
    ctx.beginPath();
    ctx.moveTo(L, Y(0));
    ctx.lineTo(w - R, Y(0));
    ctx.stroke();
  }
  for (const s of drawn) {
    ctx.beginPath();
    ctx.lineWidth = 1.5;
    ctx.strokeStyle = chartColor(s.color);
    ctx.setLineDash(s.dash);
    let on = false;
    for (const p of s.pts) {
      const x = X(p[0]);
      const y = Y(p[1]);
      if (!on) { ctx.moveTo(x, y); on = true; }
      else ctx.lineTo(x, y);
    }
    ctx.stroke();
  }
  ctx.setLineDash([]);
}

function lastOf(name) {
  const pts = telem.series && telem.series[name];
  if (!pts || !pts.length) return null;
  return pts[pts.length - 1][1];
}

function viewTime() {
  if (!telem.t) return 0;
  const extra = telemAt ? Math.min(0.4, (performance.now() - telemAt) / 1000) : 0;
  return telem.t + extra;
}

function ease(name, target, dt) {
  if (target == null || Number.isNaN(Number(target))) {
    delete shown[name];
    return null;
  }
  const goal = Number(target);
  const cur = shown[name];
  if (cur == null || !Number.isFinite(cur)) {
    shown[name] = goal;
    return goal;
  }
  const a = 1 - Math.exp(-Math.max(0, dt) / 0.08);
  shown[name] = cur + (goal - cur) * a;
  return shown[name];
}

function smoothTrace(pts, now) {
  if (!pts.length) return pts;
  const last = pts[pts.length - 1];
  if (now > last[0] + 0.02) return pts.concat([[now, last[1]]]);
  return pts;
}

const DARK_SERIES = {
  "#073e73": "#8ebbe0",
  "#2d6da3": "#a9cce6",
  "#6aa0c8": "#d5e7f4",
  "#0e2c4f": "#7eafd6",
  "#122033": "#efeae3",
};

function chartColor(hex) {
  if (document.documentElement.dataset.theme !== "dark") return hex;
  return DARK_SERIES[hex] || hex;
}

function trackColor(amount) {
  const t = Math.max(0, Math.min(1, amount));
  const dark = document.documentElement.dataset.theme === "dark";
  const from = dark ? [140, 196, 156] : [62, 106, 74];
  const to = dark ? [236, 156, 122] : [140, 84, 74];
  const c = from.map((v, i) => Math.round(v + (to[i] - v) * t));
  return `rgb(${c[0]}, ${c[1]}, ${c[2]})`;
}

function setNum(id, value, ref, scale) {
  const el = document.querySelector(`[data-num="${id}"]`);
  if (!el) return;
  const b = el.querySelector("b");
  const r = el.querySelector(".ref");
  if (value == null) {
    b.textContent = "—";
    b.style.color = cssVar("--muted");
    r.textContent = "";
    return;
  }
  const text = fmt(value, 1);
  if (b.textContent !== text) b.textContent = text;
  if (ref == null || scale == null) {
    b.style.color = cssVar("--ink");
    if (r.textContent) r.textContent = "";
    return;
  }
  b.style.color = trackColor(Math.abs(value - ref) / scale);
  const refText = "ref " + fmt(ref, 1);
  if (r.textContent !== refText) r.textContent = refText;
}

function wheelTarget(motor) {
  const mask = state.params && state.params.ESC_LMASK != null ? Number(state.params.ESC_LMASK) : 6;
  const left = (mask & (1 << (motor - 1))) !== 0;
  return lastOf(left ? "ESC_TGT_L" : "ESC_TGT_R");
}

function paintNumbers(dt) {
  for (let m = 1; m <= 4; m++) {
    const measured = ease(`i${m}`, lastOf(`ESC${m}_I`), dt);
    const commanded = ease(`ic${m}`, lastOf(`ESC${m}_ICMD`), dt);
    setNum(`i${m}`, measured, commanded, commanded == null ? null : Math.max(1.5, Math.abs(commanded)));
    const rpm = ease(`r${m}`, lastOf(`ESC${m}_RPM`), dt);
    const tgt = ease(`rt${m}`, wheelTarget(m), dt);
    setNum(`r${m}`, rpm, tgt, tgt == null ? null : Math.max(8, Math.abs(tgt) * 0.2));
    setNum(`t${m}`, ease(`t${m}`, lastOf(`ESC${m}_T`), dt), null, null);
  }
  const yawRaw = lastOf("PIDS_ach") != null ? lastOf("PIDS_ach") : lastOf("ATT_yawrate");
  const yaw = ease("yaw", yawRaw, dt);
  const yawRef = ease("yawref", lastOf("PIDS_des"), dt);
  setNum("yaw", yaw, yawRef, yawRef == null ? null : Math.max(8, Math.abs(yawRef) * 0.25));
  const x = ease("x", lastOf("NAV_xtrack"), dt);
  setNum("x", x, x == null ? null : 0, 0.5);
}

function paintReadouts(dt) {
  const speed = document.querySelector('[data-read="VFR_gspd"]');
  if (speed) {
    const text = fmt(ease("gspd", state.latest && state.latest.VFR_gspd, dt), 2);
    if (speed.textContent !== text) speed.textContent = text;
  }
  const xt = document.querySelector('[data-read="NAV_xtrack"]');
  if (xt) {
    const text = fmt(ease("xread", state.latest && state.latest.NAV_xtrack, dt), 2);
    if (xt.textContent !== text) xt.textContent = text;
  }
  const bat = document.querySelector('[data-read="BAT"]');
  if (bat) {
    const v = ease("batv", state.latest && state.latest.BAT_V, dt);
    const a = ease("bata", state.latest && state.latest.BAT_I, dt);
    const text = (v == null && a == null) ? "—" : `${fmt(v, 1)} V  ${fmt(a, 1)} A`;
    if (bat.textContent !== text) bat.textContent = text;
  }
}

const NUM_ROWS = [
  [1, 2, 3, 4].map((m) => `<div class="num" data-num="i${m}"><small>M${m}</small><b>—</b><small class="ref"></small></div>`).join(""),
  [1, 2, 3, 4].map((m) => `<div class="num" data-num="r${m}"><small>M${m}</small><b>—</b><small class="ref"></small></div>`).join(""),
  `<div class="num" data-num="yaw"><small>real</small><b>—</b><small class="ref"></small></div>`,
  `<div class="num" data-num="x"><small>desvio</small><b>—</b><small class="ref"></small></div>`,
  [1, 2, 3, 4].map((m) => `<div class="num" data-num="t${m}"><small>M${m}</small><b>—</b><small class="ref"></small></div>`).join(""),
];

function formatLogUtc(sec) {
  const n = Number(sec) || 0;
  if (n <= 0) return "sem data";
  const d = new Date(n * 1000);
  if (Number.isNaN(d.getTime())) return "sem data";
  const p = (x) => String(x).padStart(2, "0");
  return d.getFullYear() + "-" + p(d.getMonth() + 1) + "-" + p(d.getDate())
    + " " + p(d.getHours()) + ":" + p(d.getMinutes());
}

function fillSdLogList(logs, note) {
  const list = document.getElementById("sdloglist");
  const status = document.getElementById("sdlogstatus");
  if (status) status.textContent = note || "";
  if (!list) return;
  if (!logs.length) {
    list.innerHTML = `<p class="paramcount">${escText(note || "Nenhum log no cartão.")}</p>`;
    return;
  }
  list.innerHTML = logs.map((log) => `
    <div class="scriptrow">
      <div class="scriptinfo">
        <code>LOG${String(log.id).padStart(3, "0")}.BIN</code>
        <span class="unit">${formatBytes(log.size)} · ${formatLogUtc(log.time_utc)}</span>
      </div>
      <div class="scriptops">
        <button type="button" class="sdlogdl sdlogact" data-danger="1" data-id="${log.id}" data-size="${log.size}">Baixar</button>
      </div>
    </div>`).join("");
}

async function loadSdLogs() {
  const status = document.getElementById("sdlogstatus");
  if (!status) return;
  status.textContent = "Lendo a lista de logs…";
  sdlogBusy = true;
  paint();
  let data = {};
  try {
    data = await (await fetch("/sdlogs")).json();
  } catch (e) {
    data = { ok: false, logs: [], note: "sem resposta da página" };
  }
  sdlogBusy = false;
  if (page !== "logs" || !document.getElementById("sdlogstatus")) {
    paint();
    return;
  }
  sdlogCache = data.logs || [];
  fillSdLogList(sdlogCache, data.note || "");
  paint();
}

function showSdlogXfer(show) {
  const box = document.getElementById("sdlogxfer");
  if (!box) return;
  box.hidden = !show;
  if (!show) {
    const fill = document.getElementById("sdlogxferfill");
    if (fill) fill.style.width = "0%";
  }
}

function paintSdlogXfer(prog) {
  const box = document.getElementById("sdlogxfer");
  if (!box || !prog || !prog.active) return;
  box.hidden = false;
  const total = Number(prog.total) || 0;
  const done = Number(prog.done) || 0;
  let pct = total > 0 ? Math.min(100, Math.round((100 * done) / total)) : 0;
  if (prog.phase === "pronto") pct = 100;
  if (prog.phase === "listar" || prog.phase === "apagar") pct = Math.max(pct, 2);
  const fill = document.getElementById("sdlogxferfill");
  const label = document.getElementById("sdlogxferlabel");
  const meta = document.getElementById("sdlogxfermeta");
  if (fill) fill.style.width = pct + "%";
  if (label) {
    if (prog.action === "erase") label.textContent = "Apagando logs do cartão…";
    else if (prog.action === "list") label.textContent = "Listando logs…";
    else label.textContent = prog.name || "baixando";
  }
  const parts = [];
  if (prog.phase === "baixar") parts.push("baixando");
  else if (prog.phase === "listar") parts.push("listando");
  else if (prog.phase === "apagar") parts.push("apagando");
  else if (prog.phase === "pronto") parts.push("pronto");
  else if (prog.phase) parts.push(prog.phase);
  parts.push(pct + "%");
  if (total > 0) parts.push(formatBytes(done) + " / " + formatBytes(total));
  if (prog.eta != null && prog.phase === "baixar") parts.push("faltam ~" + formatSeconds(prog.eta));
  else if (prog.elapsed) parts.push(formatSeconds(prog.elapsed));
  if (meta) meta.textContent = parts.join(" · ");
  const status = document.getElementById("sdlogstatus");
  if (status && prog.phase === "baixar") status.textContent = "Baixando " + (prog.name || "") + "…";
}

function stopSdlogXferPoll() {
  if (sdlogXferTimer) {
    clearInterval(sdlogXferTimer);
    sdlogXferTimer = null;
  }
}

function startSdlogXferPoll() {
  stopSdlogXferPoll();
  showSdlogXfer(true);
  const tick = async () => {
    try {
      const prog = await (await fetch("/sdlogs/progress")).json();
      paintSdlogXfer(prog);
    } catch (e) { /* ignore */ }
  };
  tick();
  sdlogXferTimer = setInterval(tick, 250);
}

async function downloadSdLog(id, size) {
  if (sdlogBusy) return;
  if (state.armed) {
    const status = document.getElementById("sdlogstatus");
    if (status) status.textContent = "desarme antes de baixar logs do cartão";
    return;
  }
  sdlogBusy = true;
  paint();
  startSdlogXferPoll();
  const status = document.getElementById("sdlogstatus");
  if (status) status.textContent = "Baixando LOG" + String(id).padStart(3, "0") + ".BIN…";
  let note = "";
  try {
    const res = await fetch("/sdlogs/get", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ id: Number(id), size: Number(size) }),
    });
    const ctype = (res.headers.get("Content-Type") || "").toLowerCase();
    if (!res.ok || ctype.includes("application/json")) {
      let data = {};
      try { data = await res.json(); } catch (e) { data = {}; }
      note = data.note || "não foi possível baixar";
    } else {
      const blob = await res.blob();
      const disp = res.headers.get("Content-Disposition") || "";
      const match = /filename=\"([^\"]+)\"/i.exec(disp);
      const name = (match && match[1]) || ("LOG" + String(id).padStart(3, "0") + ".BIN");
      const a = document.createElement("a");
      a.href = URL.createObjectURL(blob);
      a.download = name;
      a.click();
      a.remove();
      URL.revokeObjectURL(a.href);
      note = "baixou " + name + " (" + formatBytes(blob.size) + ")";
    }
  } catch (err) {
    note = String(err && err.message ? err.message : err);
  }
  stopSdlogXferPoll();
  showSdlogXfer(false);
  sdlogBusy = false;
  paint();
  if (status) status.textContent = note;
}

async function eraseSdLogs() {
  if (sdlogBusy) return;
  if (state.armed) {
    const status = document.getElementById("sdlogstatus");
    if (status) status.textContent = "desarme antes de apagar logs do cartão";
    return;
  }
  sdlogBusy = true;
  paint();
  startSdlogXferPoll();
  const status = document.getElementById("sdlogstatus");
  if (status) status.textContent = "Apagando logs do cartão…";
  let data;
  try {
    data = await scriptPost("/sdlogs/erase", {});
  } catch (err) {
    data = { ok: false, note: String(err && err.message ? err.message : err) };
  }
  stopSdlogXferPoll();
  showSdlogXfer(false);
  sdlogBusy = false;
  if (data.ok) {
    sdlogCache = [];
    fillSdLogList([], data.note || "logs do cartão apagados");
  } else if (status) {
    status.textContent = data.note || "não foi possível apagar";
  }
  paint();
}

function renderLogs() {
  view.innerHTML = `
    <h1 class="page">Logs</h1>
    <div class="arm">
      <div>
        <p>Comando</p>
        <strong>Armar e desarmar</strong>
      </div>
      <button type="button" class="cmd" id="armbtn">Armar</button>
    </div>
    <section class="record">
      <div class="who">
        <p>CSV</p>
        <strong>Telemetria, mensagens e parâmetros</strong>
      </div>
      <input id="logname" type="text" maxlength="60" placeholder="opcional → esc_log_…" autocomplete="off" spellcheck="false">
      <button type="button" class="cmd" id="logstart">Iniciar</button>
      <button type="button" class="cmd" id="logstop">Parar</button>
      <p class="status" id="logstatus">Nada é gravado até você iniciar.</p>
    </section>
    <section class="record sdlogs">
      <div class="who">
        <p>Cartão</p>
        <strong>Dataflash da Pixhawk</strong>
      </div>
      <button type="button" class="cmd sdlogact" id="sdlogrefresh" data-danger="1">Atualizar</button>
      <button type="button" class="cmd sdlogact" id="sdlogerase" data-danger="1">Apagar todos</button>
      <p class="status" id="sdlogstatus">Desarme para listar, baixar ou apagar os .BIN do cartão.</p>
      <div class="scriptxfer" id="sdlogxfer" hidden>
        <div class="scriptxfer-label" id="sdlogxferlabel"></div>
        <div class="scriptxfer-track" aria-hidden="true">
          <div class="scriptxfer-fill" id="sdlogxferfill"></div>
        </div>
        <div class="scriptxfer-meta" id="sdlogxfermeta"></div>
      </div>
      <div id="sdloglist" class="sdloglist"></div>
    </section>
    <div class="reads">
      <div><span>Velocidade</span><strong data-read="VFR_gspd">—</strong><em>m/s</em></div>
      <div><span>Desvio</span><strong data-read="NAV_xtrack">—</strong><em>m</em></div>
      <div><span>Bateria</span><strong data-read="BAT">—</strong></div>
    </div>
    <p class="last" id="lasttext"></p>
    ${CHARTS.map((c, i) => `<section class="chart"><h2>${c.title} (${c.unit})</h2><div class="nums">${NUM_ROWS[i]}</div><div class="legend"></div><canvas data-chart="${i}"></canvas></section>`).join("")}`;
  const name = document.getElementById("logname");
  name.addEventListener("keydown", (ev) => {
    if (ev.key !== "Enter") return;
    ev.preventDefault();
    const start = document.getElementById("logstart");
    if (start && !start.disabled) start.click();
  });
  sdlogWanted = false;
  if (state.seen && !state.armed) loadSdLogs();
  else if (!state.seen) {
    sdlogWanted = true;
    document.getElementById("sdlogstatus").textContent = "Aguardando o rover.";
  } else {
    document.getElementById("sdlogstatus").textContent = "Desarme para listar os logs do cartão.";
  }
}

function formatBytes(n) {
  n = Number(n) || 0;
  if (n < 1024) return n + " B";
  if (n < 1048576) return Math.round(n / 1024) + " kB";
  return (n / 1048576).toFixed(1) + " MB";
}

function paintRecording() {
  const status = document.getElementById("logstatus");
  if (!status) return;
  const rec = state.recording || {};
  const start = document.getElementById("logstart");
  const stop = document.getElementById("logstop");
  const input = document.getElementById("logname");
  if (start) start.disabled = !!rec.active;
  if (stop) stop.disabled = !rec.active;
  if (input) input.disabled = !!rec.active;
  if (rec.active) {
    status.textContent = "Gravando " + rec.name + " · " + rec.rows + " linhas · " + formatBytes(rec.bytes);
  } else {
    status.textContent = "Nada é gravado até você iniciar. Vazio → esc_log_AAAAAMMDD_HHMMSS.csv na pasta logs.";
  }
}

function latestProblem() {
  const all = state.messages || [];
  if (!all.length || all[all.length - 1].sev > 4) return null;
  return all[all.length - 1];
}

const SEV_NAME = ["Emergência", "Alerta", "Crítico", "Erro", "Aviso", "Nota", "Info", "Debug"];

function escText(s) {
  return String(s).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", "\"": "&quot;" }[c]));
}

function sevClass(sev) {
  if (sev <= 3) return "err";
  if (sev === 4) return "warn";
  return "";
}

function fillMessages(el, items, stick) {
  if (!el) return;
  const atBottom = el.scrollHeight - el.scrollTop - el.clientHeight < 48;
  const last = items.length ? items[items.length - 1] : null;
  const key = items.length + "|" + (last ? last.t + last.sev + last.text : "");
  if (el.dataset.key === key) return;
  el.dataset.key = key;
  el.innerHTML = items.map((m) =>
    `<p class="msg ${sevClass(m.sev)}"><time>${escText(m.t)}</time>${escText(m.text)}</p>`
  ).join("") || `<p class="msg">Nenhuma mensagem ainda.</p>`;
  if (stick && (atBottom || el.scrollTop === 0)) el.scrollTop = el.scrollHeight;
}

function paintMessages() {
  const all = state.messages || [];
  fillMessages(document.getElementById("messagelist"), all, true);
}

function renderMessages() {
  view.innerHTML = `
    <h1 class="page">Mensagens</h1>
    <div class="msglist page" id="messagelist"></div>`;
  paintMessages();
}

function onArm() {
  if (!state.seen) return;
  if (state.armed) {
    post("/arm", { arm: false });
    confirm = null;
    return;
  }
  if (confirm && confirm.kind === "arm" && Date.now() < confirm.until) {
    confirm = null;
    post("/arm", { arm: true });
    return;
  }
  confirm = { kind: "arm", until: Date.now() + 4000 };
  armButton();
}

function renderModes() {
  const buttons = (catalog.modes || []).map((m) =>
    `<button type="button" class="mode" data-mode="${m.name}">${m.name}</button>`).join("");
  view.innerHTML = `<h1 class="page">Modos</h1><p class="last">O interruptor do rádio não se mexe. Qualquer modo pode ser escolhido. Se a placa recusar, volta o modo anterior.</p><div class="modes">${buttons}</div>`;
}

function renderParametros() {
  view.innerHTML = `
    <h1 class="page">Parâmetros</h1>
    <div class="paramtools">
      <button type="button" class="cmd tool" data-tool="store">Gravar na placa</button>
      <button type="button" class="cmd tool" data-tool="refresh">Atualizar</button>
      <button type="button" class="cmd tool" data-tool="reset" data-danger="1">Padrão de fábrica</button>
      <button type="button" class="cmd tool" data-tool="load">Carregar arquivo</button>
      <button type="button" class="cmd tool" data-tool="save">Salvar arquivo</button>
      <button type="button" class="cmd tool" data-tool="reboot" data-danger="1">Reiniciar</button>
      <input id="parmfile" type="file" accept=".parm,.param,.txt" hidden>
    </div>
    <p class="last">Carregar arquivo só coloca os valores na tela. Grave na placa para enviar o que mudou. Padrão de fábrica zera a memória e reinicia.</p>
    <p class="paramcount" id="parambanner"></p>
    <input id="paramsearch" type="search" placeholder="Buscar pelo nome ou descrição" autocomplete="off" value="">
    <p class="paramcount" id="paramcount"></p>
    <div id="paramsections"></div>`;
  const search = document.getElementById("paramsearch");
  search.addEventListener("input", () => {
    paramQuery = search.value;
    paramSig = "";
    paintParams();
  });
  const file = document.getElementById("parmfile");
  file.addEventListener("change", () => {
    const chosen = file.files && file.files[0];
    if (!chosen) return;
    chosen.text().then((text) => {
      file.value = "";
      stageParm(text);
    });
  });
  paramSig = "";
  paintParams();
}

const SECTION_ORDER = [
  ["direcao", "Direção"],
  ["velocidade", "Velocidade"],
  ["motores", "Motores"],
  ["missao", "Missão"],
  ["script", "Script"],
];
let paramQuery = "";
let paramSig = "";
let pending = {};
let paramEpoch = null;

const PARM_SKIP = new Set([
  "SYSID_SW_MREV", "WP_TOTAL", "CMD_TOTAL", "FENCE_TOTAL", "SYS_NUM_RESETS",
  "ARSPD_OFFSET", "GND_ABS_PRESS", "GND_TEMP", "CMD_INDEX", "LOG_LASTFILE", "FORMAT_VERSION",
]);

function closeValue(a, b) {
  return Math.abs(a - b) <= Math.max(1e-4, Math.abs(b) * 1e-4);
}

function parmNumber(value) {
  if (Math.abs(value - Math.round(value)) < 1e-6) return String(Math.round(value));
  return String(Math.round(value * 1e7) / 1e7);
}

function parseParmText(text) {
  const ordered = [];
  const seen = new Set();
  for (const raw of text.split(/\r?\n/)) {
    let line = raw.trim();
    if (!line || line.startsWith("#") || line.startsWith("//")) continue;
    line = line.split("#")[0].trim();
    let name, val;
    if (line.includes(",")) {
      const parts = line.split(",");
      name = parts[0];
      val = parts[1];
    } else {
      const bits = line.split(/\s+/);
      if (bits.length < 2) continue;
      name = bits[0];
      val = bits[1];
    }
    name = name.trim();
    if (!/^[A-Za-z][A-Za-z0-9_]{0,15}$/.test(name)) continue;
    const value = Number(val);
    if (!Number.isFinite(value)) continue;
    if (seen.has(name)) {
      const at = ordered.findIndex((pair) => pair[0] === name);
      if (at >= 0) ordered.splice(at, 1);
    }
    seen.add(name);
    ordered.push([name, value]);
  }
  return ordered;
}

function noteText(text) {
  const note = document.getElementById("note");
  if (note) note.textContent = text;
}

function stageParm(text) {
  const pairs = parseParmText(text);
  const known = state.params || {};
  let changed = 0;
  const missing = [];
  for (const [name, value] of pairs) {
    if (PARM_SKIP.has(name)) continue;
    if (!(name in known)) {
      missing.push(name);
      continue;
    }
    if (closeValue(known[name], value)) {
      delete pending[name];
      continue;
    }
    pending[name] = value;
    changed += 1;
  }
  applyPending();
  let msg = changed
    ? changed + " alterações na tela. Grave na placa para enviar."
    : "arquivo igual ao que está na placa";
  if (missing.length) {
    const shown = missing.slice(0, 8).join(", ");
    msg += " Sem este nome: " + shown + (missing.length > 8 ? "…" : "");
  }
  const banner = document.getElementById("parambanner");
  if (banner) banner.textContent = msg;
}

function applyPending() {
  for (const input of document.querySelectorAll("#paramsections input")) {
    const name = input.getAttribute("aria-label");
    const article = input.closest("article");
    if (name in pending) {
      if (document.activeElement !== input) input.value = fmt(pending[name]);
      input.dataset.touched = "1";
      if (article) article.classList.add("dirty");
    }
  }
}

function takeEpoch() {
  if (state.param_epoch == null) return;
  if (paramEpoch === null) {
    paramEpoch = state.param_epoch;
    return;
  }
  if (state.param_epoch !== paramEpoch) {
    paramEpoch = state.param_epoch;
    pending = {};
    paramSig = "";
    const banner = document.getElementById("parambanner");
    if (banner) banner.textContent = "";
  }
}

function paramMeta() {
  const meta = {};
  const groupOf = {};
  for (const group of catalog.groups || []) {
    for (const param of group.params) {
      meta[param.name] = param;
      groupOf[param.name] = group.id;
    }
  }
  return { meta, groupOf };
}

function paramInfo(name, meta) {
  if (meta[name]) return meta[name];
  const doc = paramDocs[name];
  if (doc) {
    return {
      step: doc.s || "any",
      unit: doc.u || "",
      desc: doc.d || "Parâmetro da placa.",
    };
  }
  return { step: "any", unit: "", desc: "Parâmetro da placa." };
}

function paintTools() {
  const hasParams = !!(state.params && Object.keys(state.params).length);
  for (const btn of document.querySelectorAll("button.tool")) {
    const pendingClick = confirm && confirm.el === btn && Date.now() < confirm.until;
    const action = btn.dataset.tool;
    if (action === "save" || action === "load") {
      btn.disabled = !hasParams;
      continue;
    }
    if (action === "store" || action === "refresh") {
      btn.disabled = !state.seen;
      continue;
    }
    const needsDisarm = btn.dataset.danger === "1";
    btn.disabled = !state.seen || (!!state.armed && needsDisarm && !pendingClick);
  }
}

function saveParm() {
  const params = state.params || {};
  const names = Object.keys(params).sort();
  const lines = names.map((name) => {
    const value = name in pending ? pending[name] : params[name];
    return name + "," + parmNumber(value);
  });
  const blob = new Blob([lines.join("\n") + "\n"], { type: "text/plain" });
  const a = document.createElement("a");
  const stamp = new Date();
  const p = (n) => String(n).padStart(2, "0");
  const name = `rover_${stamp.getFullYear()}${p(stamp.getMonth() + 1)}${p(stamp.getDate())}_${p(stamp.getHours())}${p(stamp.getMinutes())}${p(stamp.getSeconds())}.parm`;
  a.href = URL.createObjectURL(blob);
  a.download = name;
  document.body.appendChild(a);
  a.click();
  a.remove();
  URL.revokeObjectURL(a.href);
}

function runTool(action) {
  if (action === "save") {
    saveParm();
    return;
  }
  if (action === "load") {
    const input = document.getElementById("parmfile");
    if (input) input.click();
    return;
  }
  if (action === "store") {
    const changes = Object.assign({}, pending);
    post("/params/write", { changes });
    return;
  }
  post("/params/" + action, {});
}

function toolNeedsConfirm(tool) {
  const action = tool.dataset.tool;
  if (tool.dataset.danger === "1") return true;
  if ((action === "store" || action === "refresh") && Object.keys(pending).length > 0) return true;
  return false;
}

function onTool(tool) {
  const action = tool.dataset.tool;
  if (toolNeedsConfirm(tool)) {
    if (confirm && confirm.el === tool && Date.now() < confirm.until) {
      const label = confirm.label;
      confirm = null;
      tool.textContent = label;
      tool.className = "cmd tool";
      runTool(action);
      return;
    }
    confirm = {
      kind: "tool",
      el: tool,
      until: Date.now() + 4000,
      label: tool.textContent,
      className: "cmd tool",
    };
    tool.textContent = "Confirmar";
    tool.className = "cmd tool solid";
    return;
  }
  runTool(action);
}

function paintParams() {
  const box = document.getElementById("paramsections");
  if (!box) return;
  const q = paramQuery.trim().toLowerCase();
  const { meta, groupOf } = paramMeta();
  const names = Object.keys(state.params || {}).filter((name) => {
    if (!q) return true;
    if (name.toLowerCase().includes(q)) return true;
    const info = paramInfo(name, meta);
    return (info.desc || "").toLowerCase().includes(q);
  });
  names.sort();
  const sig = q + "\n" + names.join("\n");
  const count = document.getElementById("paramcount");
  if (count) {
    const total = Object.keys(state.params || {}).length;
    count.textContent = total ? `${names.length} de ${total}` : "Baixando a lista da placa…";
  }
  if (sig === paramSig) return;
  paramSig = sig;
  const buckets = {};
  for (const name of names) {
    const id = groupOf[name] || ("px:" + (name.includes("_") ? name.split("_")[0] : "OUTROS"));
    if (!buckets[id]) buckets[id] = [];
    buckets[id].push(name);
  }
  const sections = [];
  for (const [id, title] of SECTION_ORDER) {
    if (buckets[id]) {
      sections.push({ title, names: buckets[id] });
      delete buckets[id];
    }
  }
  for (const id of Object.keys(buckets).sort()) {
    sections.push({ title: id.startsWith("px:") ? id.slice(3) : id, names: buckets[id] });
  }
  box.innerHTML = sections.map((section) => `
    <h2 class="section">${escText(section.title)}</h2>
    ${section.names.map((name) => {
      const info = paramInfo(name, meta);
      return `<article class="param">
        <div>
          <h2>${escText(name)}</h2>
          <div class="ctrl">
            <input type="number" step="${escText(info.step)}" inputmode="decimal" autocomplete="off" aria-label="${escText(name)}">
            <span class="unit">${escText(info.unit || "")}</span>
            <button type="button" class="apply" data-param="${escText(name)}">Aplicar</button>
            <span class="live" data-live="${escText(name)}">na placa —</span>
          </div>
        </div>
        <p class="desc">${escText(info.desc)}</p>
      </article>`;
    }).join("")}`).join("") || `<p class="paramcount">Nenhum parâmetro com esse nome.</p>`;
  for (const input of box.querySelectorAll("input")) {
    input.addEventListener("wheel", (ev) => ev.preventDefault(), { passive: false });
    input.addEventListener("input", () => {
      const name = input.getAttribute("aria-label");
      const value = Number(input.value);
      const board = state.params ? state.params[name] : undefined;
      const article = input.closest("article");
      if (!Number.isFinite(value) || (board != null && closeValue(board, value))) {
        delete pending[name];
        input.dataset.touched = "";
        if (article) article.classList.remove("dirty");
      } else {
        pending[name] = value;
        input.dataset.touched = "1";
        if (article) article.classList.add("dirty");
      }
    });
  }
  applyPending();
}

function bytesToBase64(bytes) {
  let bin = "";
  const step = 4096;
  for (let i = 0; i < bytes.length; i += step) {
    bin += String.fromCharCode.apply(null, bytes.subarray(i, i + step));
  }
  return btoa(bin);
}

function fillScriptColumn(folder, files, note, blocked) {
  const list = document.getElementById(folder + "-list");
  const hint = document.getElementById(folder + "-note");
  const count = document.getElementById(folder + "-count");
  if (hint) hint.textContent = note || "";
  if (count) {
    count.textContent = files.length
      ? (files.length === 1 ? "1 script" : files.length + " scripts")
      : "";
  }
  if (!list) return;
  if (!files.length) {
    list.innerHTML = (note || blocked) ? "" : `<p class="paramcount">Nenhum script nesta pasta.</p>`;
    return;
  }
  list.innerHTML = files.map((file) => `
    <div class="scriptrow">
      <div class="scriptinfo">
        <code>${escText(file.name)}</code>
        <span class="unit">${formatBytes(file.size)}</span>
      </div>
      <div class="scriptops">
        <button type="button" class="scriptcopy scriptact" data-folder="${escText(folder)}" data-name="${escText(file.name)}">Copiar</button>
        <button type="button" class="scriptdel scriptact" data-folder="${escText(folder)}" data-name="${escText(file.name)}">Excluir</button>
      </div>
    </div>`).join("");
}

function base64ToBytes(text) {
  const bin = atob(text);
  const out = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i);
  return out;
}

function saveScriptFile(name, bytes) {
  const blob = new Blob([bytes], { type: "text/plain;charset=utf-8" });
  const a = document.createElement("a");
  a.href = URL.createObjectURL(blob);
  a.download = name;
  a.click();
  a.remove();
  URL.revokeObjectURL(a.href);
}

async function loadScripts() {
  const status = document.getElementById("scriptstatus");
  if (!status) return "";
  status.textContent = "Lendo o cartão…";
  scriptBusy = true;
  paint();
  let data = {};
  try {
    data = await (await fetch("/scripts")).json();
  } catch (e) {
    data = { note: "sem resposta da página" };
  }
  scriptBusy = false;
  if (page !== "scripts" || !document.getElementById("scriptstatus")) {
    paint();
    return "";
  }
  fillScriptColumn("scripts", data.scripts || [], data.scripts_note || "", data.note);
  fillScriptColumn("modules", data.modules || [], data.modules_note || "", data.note);
  status.textContent = data.note || "";
  paint();
  return data.note || "";
}

async function scriptPost(url, body) {
  const res = await fetch(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
  try {
    return await res.json();
  } catch (e) {
    return { ok: false, note: "sem resposta da página" };
  }
}

async function reloadScripts(sticky) {
  await loadScripts();
  const status = document.getElementById("scriptstatus");
  if (sticky && status && page === "scripts") status.textContent = sticky;
}

function formatSeconds(n) {
  n = Math.max(0, Number(n) || 0);
  if (n < 10) return n.toFixed(1) + " s";
  return Math.round(n) + " s";
}

function scriptPhaseLabel(phase) {
  if (phase === "preparar" || phase === "criar") return "abrindo o arquivo";
  if (phase === "enviar") return "enviando";
  if (phase === "crc") return "conferindo CRC";
  if (phase === "pronto") return "pronto";
  if (phase === "listar") return "listando pasta";
  if (phase === "apagar") return "apagando";
  return phase || "enviando";
}

function showScriptXfer(show) {
  const box = document.getElementById("scriptxfer");
  if (!box) return;
  box.hidden = !show;
  if (!show) {
    const fill = document.getElementById("scriptxferfill");
    if (fill) fill.style.width = "0%";
  }
}

function paintScriptXfer(prog) {
  const box = document.getElementById("scriptxfer");
  if (!box || !prog || !prog.active) return;
  box.hidden = false;
  const total = Number(prog.total) || 0;
  const done = Number(prog.done) || 0;
  const clearing = prog.action === "clear";
  let pct = total > 0 ? Math.min(100, Math.round((100 * done) / total)) : 0;
  if (prog.phase === "crc") pct = Math.max(pct, 97);
  if (prog.phase === "pronto") pct = 100;
  if (prog.phase === "preparar" || prog.phase === "criar") pct = Math.max(pct, 2);
  if (prog.phase === "listar") pct = Math.max(pct, 2);
  const fill = document.getElementById("scriptxferfill");
  const label = document.getElementById("scriptxferlabel");
  const meta = document.getElementById("scriptxfermeta");
  if (fill) fill.style.width = pct + "%";
  let which;
  if (clearing) {
    which = total > 0
      ? `Limpando STRG_BAK (${done} de ${total})`
      : "Limpando STRG_BAK…";
  } else {
    which = (prog.count > 1)
      ? `Arquivo ${prog.index} de ${prog.count}: ${prog.name || ""}`
      : (prog.name || "enviando");
  }
  if (label) label.textContent = which;
  const parts = [scriptPhaseLabel(prog.phase), pct + "%"];
  if (total > 0) {
    parts.push(clearing ? (done + " / " + total) : (formatBytes(done) + " / " + formatBytes(total)));
  }
  if (prog.eta != null && (prog.phase === "enviar" || prog.phase === "apagar")) {
    parts.push("faltam ~" + formatSeconds(prog.eta));
  } else if (prog.elapsed) {
    parts.push(formatSeconds(prog.elapsed));
  }
  if (meta) meta.textContent = parts.join(" · ");
  const status = document.getElementById("scriptstatus");
  if (status && clearing) {
    status.textContent = which;
  } else if (status && prog.phase === "enviar") {
    status.textContent = "Enviando " + (prog.name || "") + "…";
  } else if (status && prog.phase === "crc") {
    status.textContent = "Conferindo " + (prog.name || "") + "…";
  }
}

function stopScriptXferPoll() {
  if (scriptXferTimer) {
    clearInterval(scriptXferTimer);
    scriptXferTimer = null;
  }
}

function startScriptXferPoll() {
  stopScriptXferPoll();
  showScriptXfer(true);
  const tick = async () => {
    try {
      const prog = await (await fetch("/scripts/progress")).json();
      paintScriptXfer(prog);
    } catch (e) { /* ignore poll errors while upload runs */ }
  };
  tick();
  scriptXferTimer = setInterval(tick, 200);
}

async function onScriptFiles(input) {
  const folder = input.dataset.folder;
  const files = Array.from(input.files || []);
  input.value = "";
  if (!files.length || scriptBusy) return;
  scriptBusy = true;
  paint();
  startScriptXferPoll();
  const status = document.getElementById("scriptstatus");
  let sticky = "";
  for (let i = 0; i < files.length; i++) {
    const file = files[i];
    if (status) status.textContent = "Enviando " + file.name + " (" + (i + 1) + " de " + files.length + ")…";
    paintScriptXfer({
      active: true,
      action: "put",
      name: file.name,
      folder,
      phase: "preparar",
      done: 0,
      total: file.size,
      index: i + 1,
      count: files.length,
      elapsed: 0,
      eta: null,
    });
    if (file.size > 400000) {
      sticky = file.name + " é grande demais";
      break;
    }
    const bytes = new Uint8Array(await file.arrayBuffer());
    const data = await scriptPost("/scripts/put", {
      folder,
      name: file.name,
      data: bytesToBase64(bytes),
      index: i + 1,
      count: files.length,
    });
    if (!data.ok) {
      sticky = data.note || "não foi possível enviar";
      break;
    }
    sticky = data.note || "";
  }
  stopScriptXferPoll();
  showScriptXfer(false);
  scriptBusy = false;
  await reloadScripts(sticky);
}

async function copyScript(folder, name) {
  scriptBusy = true;
  paint();
  const status = document.getElementById("scriptstatus");
  if (status) status.textContent = "Copiando " + name + "…";
  const data = await scriptPost("/scripts/get", { folder, name });
  scriptBusy = false;
  paint();
  if (!data.ok || !data.data) {
    if (status) status.textContent = data.note || "não foi possível copiar";
    return;
  }
  try {
    saveScriptFile(data.name || name, base64ToBytes(data.data));
    if (status) status.textContent = data.note || (name + " copiado");
  } catch (e) {
    if (status) status.textContent = "não foi possível salvar " + name;
  }
}

async function removeScript(folder, name) {
  scriptBusy = true;
  paint();
  const status = document.getElementById("scriptstatus");
  if (status) status.textContent = "Excluindo " + name + "…";
  const data = await scriptPost("/scripts/remove", { folder, name });
  scriptBusy = false;
  await reloadScripts(data.note || (data.ok ? "" : "não foi possível excluir"));
}

async function clearStorageBak() {
  if (scriptBusy) return;
  scriptBusy = true;
  paint();
  const status = document.getElementById("scriptstatus");
  if (status) status.textContent = "Limpando STRG_BAK…";
  startScriptXferPoll();
  let data;
  try {
    data = await scriptPost("/storage/clear-bak", {});
  } catch (err) {
    data = { ok: false, note: String(err && err.message ? err.message : err) };
  }
  stopScriptXferPoll();
  showScriptXfer(false);
  scriptBusy = false;
  paint();
  if (status) status.textContent = data.note || (data.ok ? "STRG_BAK limpa" : "não foi possível limpar");
}

async function formatSdCard() {
  if (scriptBusy) return;
  if (state.armed) {
    const status = document.getElementById("scriptstatus");
    if (status) status.textContent = "desarme antes de formatar o cartão SD";
    return;
  }
  scriptBusy = true;
  paint();
  const status = document.getElementById("scriptstatus");
  if (status) status.textContent = "Formatando o cartão SD…";
  let data;
  try {
    data = await scriptPost("/scripts/format", {});
  } catch (err) {
    data = { ok: false, note: String(err && err.message ? err.message : err) };
  }
  scriptBusy = false;
  paint();
  if (status) {
    status.textContent = data.note || (data.ok
      ? "cartão SD formatado — reinicie a placa e reenvie os scripts"
      : "não foi possível formatar");
  }
  if (data.ok) loadScripts();
}

function renderScripts() {
  view.innerHTML = `
    <h1 class="page">Scripts</h1>
    <p class="last">Lista os .lua que estão no cartão da Pixhawk. Copiar baixa o arquivo para este computador. Excluir apaga do cartão. Adicionar envia um .lua novo ou substitui o de mesmo nome. A placa só passa a usar a lista nova depois de reiniciar. Limpar STRG_BAK apaga os backups de parâmetros na microSD sem mostrar a lista. Formatar SD apaga scripts, logs e terreno (parâmetros e missão ficam); desarme antes e reenvie os scripts depois.</p>
    <div class="paramtools">
      <button type="button" class="cmd scriptact" id="scriptrefresh">Atualizar</button>
      <button type="button" class="cmd scriptact" id="scriptclearbak">Limpar STRG_BAK</button>
      <button type="button" class="cmd scriptact" id="scriptformat" data-danger="1">Formatar SD</button>
      <button type="button" class="cmd scriptact" id="scriptreboot">Reiniciar a placa</button>
    </div>
    <p class="paramcount" id="scriptstatus"></p>
    <div class="scriptxfer" id="scriptxfer" hidden>
      <div class="scriptxfer-label" id="scriptxferlabel"></div>
      <div class="scriptxfer-track" aria-hidden="true">
        <div class="scriptxfer-fill" id="scriptxferfill"></div>
      </div>
      <div class="scriptxfer-meta" id="scriptxfermeta"></div>
    </div>
    <div class="scriptgrid">
      <section class="scriptbox">
        <div class="scripthead">
          <div>
            <h2>APM/scripts</h2>
            <p class="paramcount" id="scripts-count"></p>
          </div>
          <button type="button" class="scriptact scriptadd" data-folder="scripts">Adicionar</button>
          <input class="scriptfile" data-folder="scripts" type="file" accept=".lua,text/plain" multiple hidden>
        </div>
        <p class="paramcount" id="scripts-note"></p>
        <div id="scripts-list"></div>
      </section>
      <section class="scriptbox">
        <div class="scripthead">
          <div>
            <h2>APM/scripts/modules</h2>
            <p class="paramcount" id="modules-count"></p>
          </div>
          <button type="button" class="scriptact scriptadd" data-folder="modules">Adicionar</button>
          <input class="scriptfile" data-folder="modules" type="file" accept=".lua,text/plain" multiple hidden>
        </div>
        <p class="paramcount" id="modules-note"></p>
        <div id="modules-list"></div>
      </section>
    </div>`;
  for (const input of document.querySelectorAll("input.scriptfile")) {
    input.addEventListener("change", () => onScriptFiles(input));
  }
  scriptWanted = false;
  if (state.seen) loadScripts();
  else {
    scriptWanted = true;
    document.getElementById("scriptstatus").textContent = "Aguardando o rover.";
  }
}

function missionCommands() {
  return (catalog && catalog.mission_commands) || [];
}

function missionCmdSpec(id) {
  return missionCommands().find((c) => c.id === Number(id));
}

function missionPlainItems() {
  return missionItems.map((it) => ({
    seq: it.seq,
    frame: it.frame,
    command: it.command,
    current: it.current,
    autocontinue: it.autocontinue,
    p1: it.p1,
    p2: it.p2,
    p3: it.p3,
    p4: it.p4,
    lat: it.lat,
    lon: it.lon,
    z: it.z,
  }));
}

function gpsLatLon() {
  const g = state.gps || {};
  if (g.GPI_lat != null && g.GPI_lon != null) return [g.GPI_lat, g.GPI_lon];
  return null;
}

function gpsHdg() {
  const h = state.gps && state.gps.GPI_hdg;
  return Number.isFinite(Number(h)) ? Number(h) : null;
}

/** Move distM meters at bearingDeg (0 = north, clockwise). */
function offsetLatLon(lat, lon, bearingDeg, distM) {
  const R = 6371000.0;
  const brng = (((bearingDeg % 360) + 360) % 360) * Math.PI / 180;
  const φ1 = lat * Math.PI / 180;
  const λ1 = lon * Math.PI / 180;
  const δ = Number(distM) / R;
  const φ2 = Math.asin(Math.sin(φ1) * Math.cos(δ) + Math.cos(φ1) * Math.sin(δ) * Math.cos(brng));
  const λ2 = λ1 + Math.atan2(
    Math.sin(brng) * Math.sin(δ) * Math.cos(φ1),
    Math.cos(δ) - Math.sin(φ1) * Math.sin(φ2),
  );
  return [φ2 * 180 / Math.PI, ((λ2 * 180 / Math.PI + 540) % 360) - 180];
}

function distMeters(lat1, lon1, lat2, lon2) {
  const R = 6371000.0;
  const φ1 = lat1 * Math.PI / 180;
  const φ2 = lat2 * Math.PI / 180;
  const Δφ = (lat2 - lat1) * Math.PI / 180;
  const Δλ = (lon2 - lon1) * Math.PI / 180;
  const a = Math.sin(Δφ / 2) ** 2 + Math.cos(φ1) * Math.cos(φ2) * Math.sin(Δλ / 2) ** 2;
  return 2 * R * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
}

function bearingBetween(lat1, lon1, lat2, lon2) {
  const φ1 = lat1 * Math.PI / 180;
  const φ2 = lat2 * Math.PI / 180;
  const Δλ = (lon2 - lon1) * Math.PI / 180;
  const y = Math.sin(Δλ) * Math.cos(φ2);
  const x = Math.cos(φ1) * Math.sin(φ2) - Math.sin(φ1) * Math.cos(φ2) * Math.cos(Δλ);
  return ((Math.atan2(y, x) * 180 / Math.PI) + 360) % 360;
}

function lastPositionalSeq() {
  for (let i = missionItems.length - 1; i >= 0; i--) {
    const it = missionItems[i];
    if (it.has_pos && (it.lat || it.lon)) return i;
  }
  return null;
}

function relativeOriginPoint() {
  const src = (document.getElementById("missionrelsrc") || {}).value || "rover";
  if (src === "fixed" && missionRelOrigin) {
    return { lat: missionRelOrigin.lat, lon: missionRelOrigin.lon, hdg: missionRelOrigin.hdg, label: "origem fixa" };
  }
  if (src === "selected" && missionSelected != null && missionItems[missionSelected]
      && missionItems[missionSelected].has_pos) {
    const it = missionItems[missionSelected];
    return { lat: it.lat, lon: it.lon, hdg: null, label: `WP ${missionSelected}` };
  }
  if (src === "last" || src === "chain") {
    const i = lastPositionalSeq();
    if (i != null) {
      const it = missionItems[i];
      let hdg = null;
      if (i > 0) {
        for (let j = i - 1; j >= 0; j--) {
          if (missionItems[j].has_pos && (missionItems[j].lat || missionItems[j].lon)) {
            hdg = bearingBetween(missionItems[j].lat, missionItems[j].lon, it.lat, it.lon);
            break;
          }
        }
      }
      return { lat: it.lat, lon: it.lon, hdg, label: i === 0 ? "HOME" : `WP ${i}` };
    }
  }
  const gps = gpsLatLon();
  if (gps) return { lat: gps[0], lon: gps[1], hdg: gpsHdg(), label: "rover" };
  if (missionRelOrigin) {
    return { lat: missionRelOrigin.lat, lon: missionRelOrigin.lon, hdg: missionRelOrigin.hdg, label: "origem fixa" };
  }
  return null;
}

function relativeAbsoluteBearing(angleInput) {
  const ang = Number(String(angleInput).replace(",", "."));
  if (!Number.isFinite(ang)) return null;
  const ref = (document.getElementById("missionrelref") || {}).value || "north";
  const origin = relativeOriginPoint();
  if (ref === "north") return ((ang % 360) + 360) % 360;
  if (ref === "rover") {
    const hdg = gpsHdg();
    if (hdg == null && !(origin && origin.hdg != null)) return null;
    const base = hdg != null ? hdg : origin.hdg;
    return ((base + ang) % 360 + 360) % 360;
  }
  // relative to last leg / origin heading
  if (origin && origin.hdg != null) return ((origin.hdg + ang) % 360 + 360) % 360;
  const hdg = gpsHdg();
  if (hdg == null) return null;
  return ((hdg + ang) % 360 + 360) % 360;
}

function computeRelativeStep() {
  const distEl = document.getElementById("missionreldist");
  const angEl = document.getElementById("missionrelang");
  if (!distEl || !angEl) return null;
  const dist = Number(String(distEl.value).replace(",", "."));
  if (!Number.isFinite(dist) || dist < 0) return { error: "distância inválida" };
  const origin = relativeOriginPoint();
  if (!origin) return { error: "sem origem (GPS do rover ou waypoint)" };
  const bearing = relativeAbsoluteBearing(angEl.value);
  if (bearing == null) return { error: "ângulo inválido (falta rumo do rover?)" };
  const [lat, lon] = offsetLatLon(origin.lat, origin.lon, bearing, dist);
  return { origin, bearing, dist, lat, lon };
}

function clearRelativePreview() {
  if (missionRelPreview && missionMap) {
    missionMap.removeLayer(missionRelPreview);
  }
  missionRelPreview = null;
  missionRelPreviewPt = null;
}

function paintRelativePreview() {
  if (missionPlanMode !== "relative" || !missionMap) {
    clearRelativePreview();
    return;
  }
  const step = computeRelativeStep();
  const info = document.getElementById("missionrelinfo");
  if (!step || step.error) {
    clearRelativePreview();
    if (info) info.textContent = step && step.error ? step.error : "";
    return;
  }
  if (info) {
    info.textContent = `de ${step.origin.label} → ${step.dist.toFixed(2)} m a ${step.bearing.toFixed(0)}° `
      + `(${step.lat.toFixed(6)}, ${step.lon.toFixed(6)})`;
  }
  missionRelPreviewPt = [step.lat, step.lon];
  if (!missionRelPreview) missionRelPreview = L.layerGroup().addTo(missionMap);
  missionRelPreview.clearLayers();
  L.polyline([[step.origin.lat, step.origin.lon], [step.lat, step.lon]], {
    color: "#c25e00",
    weight: 3,
    dashArray: "4 6",
  }).addTo(missionRelPreview);
  L.circleMarker([step.lat, step.lon], {
    radius: 8,
    color: "#c25e00",
    fillColor: "#ffcc66",
    fillOpacity: 0.9,
    weight: 2,
  }).addTo(missionRelPreview);
  L.circleMarker([step.origin.lat, step.origin.lon], {
    radius: 5,
    color: "#c25e00",
    fillColor: "#fff",
    fillOpacity: 1,
    weight: 2,
  }).addTo(missionRelPreview);
}

async function addRelativeStep() {
  if (missionBusy) return;
  const step = computeRelativeStep();
  const status = document.getElementById("missionstatus");
  if (!step || step.error) {
    if (status) status.textContent = (step && step.error) || "passo inválido";
    return;
  }
  const alignPrev = !!(document.getElementById("missionrelalign") || {}).checked;
  const chain = !!(document.getElementById("missionrelchain") || {}).checked;
  const setRumo = !!(document.getElementById("missionrelrumo") || {}).checked;
  const prevSeq = lastPositionalSeq();

  missionBusy = true;
  paint();
  try {
    if (!missionItems.length) {
      const home = makeMissionItem(CMD_WAYPOINT, step.origin.lat, step.origin.lon, { z: 0 });
      home.current = 1;
      home.frame = 0;
      home.name = "WAYPOINT";
      missionItems = [home];
    }

    // Rumo do WP anterior = direção deste trecho (frente do rover), como no mission_editor.
    if (alignPrev && prevSeq != null && prevSeq > 0 && missionItems[prevSeq].command === CMD_WAYPOINT) {
      missionItems[prevSeq].z = Math.round(step.bearing * 10) / 10;
    }

    const z = setRumo ? Math.round(step.bearing * 10) / 10 : -1;
    const item = makeMissionItem(CMD_WAYPOINT, step.lat, step.lon, { z });
    missionItems.push(item);
    missionItems.forEach((it, i) => { it.seq = i; });
    missionSelected = missionItems.length - 1;
    missionDirty = true;
    await refreshMissionAnalyse(missionSelected);

    if (chain) {
      const src = document.getElementById("missionrelsrc");
      if (src) src.value = "last";
      missionRelOrigin = { lat: step.lat, lon: step.lon, hdg: step.bearing };
    }
    paintRelativePreview();
    if (status) {
      status.textContent = `passo +${step.dist.toFixed(2)} m / ${step.bearing.toFixed(0)}°`
        + (setRumo ? ` (rumo do WP = ${Math.round(z)}°)` : " (passa direto)");
    }
  } finally {
    missionBusy = false;
    paint();
  }
}

function captureRelativeOriginFromRover() {
  const gps = gpsLatLon();
  if (!gps) {
    const status = document.getElementById("missionstatus");
    if (status) status.textContent = "sem GPS do rover";
    return;
  }
  missionRelOrigin = { lat: gps[0], lon: gps[1], hdg: gpsHdg() };
  const src = document.getElementById("missionrelsrc");
  if (src) src.value = "fixed";
  const status = document.getElementById("missionstatus");
  if (status) {
    status.textContent = `origem fixa no rover: ${gps[0].toFixed(6)}, ${gps[1].toFixed(6)}`
      + (missionRelOrigin.hdg != null ? ` · rumo ${missionRelOrigin.hdg.toFixed(0)}°` : "");
  }
  paintRelativePreview();
}

function setMissionPlanMode(mode) {
  missionPlanMode = mode === "relative" ? "relative" : "map";
  missionClickMode = missionPlanMode === "map" ? "add" : "pan";
  const mapBtn = document.getElementById("missionmodemap");
  const relBtn = document.getElementById("missionmoderel");
  const panel = document.getElementById("missionrelpanel");
  if (mapBtn) mapBtn.classList.toggle("on", missionPlanMode === "map");
  if (relBtn) relBtn.classList.toggle("on", missionPlanMode === "relative");
  if (panel) panel.hidden = missionPlanMode !== "relative";
  const hint = document.querySelector(".mission-hint");
  if (hint) {
    hint.textContent = missionPlanMode === "relative"
      ? "modo relativo: distância + ângulo a partir da origem"
      : "clique = adicionar · arraste = mover · direito = menu";
  }
  if (missionPlanMode !== "relative") clearRelativePreview();
  else paintRelativePreview();
}

function acceptMissionPayload(data, { dirty = true, selected = missionSelected, fit = false } = {}) {
  if (!data || !Array.isArray(data.items)) return;
  missionItems = data.items;
  missionStats = data.stats || { total_m: 0, legs: [], waypoints: 0 };
  missionDirty = dirty;
  if (selected != null && selected >= 0 && selected < missionItems.length) {
    missionSelected = selected;
  } else if (missionSelected == null || missionSelected >= missionItems.length) {
    missionSelected = missionItems.findIndex((it, i) => i > 0 && it.command === CMD_WAYPOINT);
    if (missionSelected < 0) missionSelected = missionItems.length ? 0 : null;
  }
  paintMissionEditor({ fit });
}

async function missionPost(url, body) {
  const res = await fetch(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body || {}),
  });
  let data = {};
  try { data = await res.json(); } catch (e) { /* empty */ }
  if (data.note) document.getElementById("note").textContent = data.note;
  const status = document.getElementById("missionstatus");
  if (status && data.note) status.textContent = data.note;
  pollState();
  return { ok: res.ok && data.ok !== false, data };
}

function startMissionXferPoll() {
  stopMissionXferPoll();
  missionXferTimer = setInterval(async () => {
    if (page !== "editor-missao") return;
    try {
      const prog = await (await fetch("/mission/progress")).json();
      paintMissionXfer(prog);
    } catch (e) { /* ignore */ }
  }, 200);
}

function stopMissionXferPoll() {
  if (missionXferTimer) {
    clearInterval(missionXferTimer);
    missionXferTimer = null;
  }
  showMissionXfer(false);
}

function showMissionXfer(on) {
  const box = document.getElementById("missionxfer");
  if (box) box.hidden = !on;
}

function paintMissionXfer(prog) {
  if (!prog || !prog.active) {
    showMissionXfer(false);
    return;
  }
  showMissionXfer(true);
  const label = document.getElementById("missionxferlabel");
  const fill = document.getElementById("missionxferfill");
  const meta = document.getElementById("missionxfermeta");
  const done = prog.done || 0;
  const total = prog.total || 0;
  const pct = total > 0 ? Math.min(100, Math.round(100 * done / total)) : 0;
  if (label) label.textContent = `${prog.action || ""} ${prog.phase || ""}`.trim();
  if (fill) fill.style.width = pct + "%";
  if (meta) {
    meta.textContent = total
      ? `${done}/${total} · ${prog.elapsed || 0}s`
      : `${prog.elapsed || 0}s`;
  }
}

async function refreshMissionAnalyse(selected = missionSelected) {
  if (!missionItems.length) return;
  const result = await missionPost("/mission/analyse", { items: missionPlainItems() });
  if (result.ok) acceptMissionPayload(result.data, { dirty: missionDirty, selected });
}

async function downloadMission() {
  if (missionBusy) return;
  if (missionDirty && !window.confirm("Descartar as alterações não enviadas?")) return;
  missionBusy = true;
  paint();
  const status = document.getElementById("missionstatus");
  if (status) status.textContent = "Baixando missão…";
  startMissionXferPoll();
  let result;
  try {
    result = await missionPost("/mission/download", {});
  } catch (err) {
    result = { ok: false, data: { note: String(err && err.message ? err.message : err) } };
  }
  stopMissionXferPoll();
  missionBusy = false;
  if (result.ok) acceptMissionPayload(result.data, { dirty: false, fit: true });
  else paint();
  if (status) status.textContent = (result.data && result.data.note) || (result.ok ? "" : "falha ao baixar");
}

async function uploadMission() {
  if (missionBusy) return;
  if (!missionItems.length) {
    const status = document.getElementById("missionstatus");
    if (status) status.textContent = "Nenhuma missão carregada";
    return;
  }
  if (state.armed) {
    const status = document.getElementById("missionstatus");
    if (status) status.textContent = "desarme antes de enviar a missão";
    return;
  }
  const warns = missionItems.filter((it) => it.warn).length;
  let msg = `Enviar ${missionItems.length} itens ao rover? A missão atual será substituída.`;
  if (warns) msg += `\n\nATENÇÃO: ${warns} alerta(s) na lista.`;
  if (!window.confirm(msg)) return;
  missionBusy = true;
  paint();
  const status = document.getElementById("missionstatus");
  if (status) status.textContent = "Enviando missão…";
  startMissionXferPoll();
  let result;
  try {
    result = await missionPost("/mission/upload", { items: missionPlainItems() });
  } catch (err) {
    result = { ok: false, data: { note: String(err && err.message ? err.message : err) } };
  }
  stopMissionXferPoll();
  missionBusy = false;
  if (result.ok) missionDirty = false;
  paintMissionEditor();
  if (status) status.textContent = (result.data && result.data.note) || (result.ok ? "" : "falha ao enviar");
}

async function newMission() {
  if (missionBusy) return;
  if (missionDirty && !window.confirm("Descartar as alterações não enviadas?")) return;
  const gps = gpsLatLon() || DEFAULT_MAP;
  missionBusy = true;
  paint();
  const result = await missionPost("/mission/new", { lat: gps[0], lon: gps[1] });
  missionBusy = false;
  if (result.ok) acceptMissionPayload(result.data, { dirty: true, fit: true });
  else paint();
}

/** Missão de teste: quadrado sideM×sideM a partir do GPS, alinhado à frente do rover. */
async function makeSquareTestMission(sideM = 1) {
  if (missionBusy) return;
  const gps = gpsLatLon();
  const status = document.getElementById("missionstatus");
  if (!gps) {
    if (status) status.textContent = "precisa do GPS do rover para montar o quadrado";
    return;
  }
  if (missionDirty && !window.confirm("Substituir a missão local pelo quadrado de teste?")) return;
  const hdg = gpsHdg();
  const base = Number.isFinite(hdg) ? hdg : 0;
  const side = Math.max(0.2, Number(sideM) || 1);
  const p0 = gps;
  const p1 = offsetLatLon(p0[0], p0[1], base, side);
  const p2 = offsetLatLon(p1[0], p1[1], base + 90, side);
  const p3 = offsetLatLon(p2[0], p2[1], base + 180, side);
  const p4 = offsetLatLon(p3[0], p3[1], base + 270, side);

  const home = makeMissionItem(CMD_WAYPOINT, p0[0], p0[1], { z: 0 });
  home.current = 1;
  home.frame = 0;
  home.name = "WAYPOINT";
  // Quatro cantos; Alt = −1 (passa direto). Fecha perto do HOME.
  const corners = [p1, p2, p3, p4].map((p) => makeMissionItem(CMD_WAYPOINT, p[0], p[1], { z: -1 }));
  missionItems = [home, ...corners];
  missionItems.forEach((it, i) => { it.seq = i; });
  missionSelected = 1;
  missionDirty = true;
  missionBusy = true;
  paint();
  try {
    await refreshMissionAnalyse(1);
    paintMissionEditor({ fit: true });
    if (status) {
      status.textContent = `quadrado ${side.toFixed(1)} m × ${side.toFixed(1)} m `
        + `(frente ${base.toFixed(0)}°)`
        + (Number.isFinite(hdg) ? "" : " · sem rumo GPS, usei norte")
        + " · revise no mapa e Escrever no rover";
    }
  } finally {
    missionBusy = false;
    paint();
  }
}

async function importMissionFile(file) {
  if (!file || missionBusy) return;
  const text = await file.text();
  missionBusy = true;
  paint();
  const result = await missionPost("/mission/import", { text });
  missionBusy = false;
  if (result.ok) acceptMissionPayload(result.data, { dirty: true, fit: true });
  else paint();
}

async function exportMissionFile() {
  if (!missionItems.length || missionBusy) return;
  const res = await fetch("/mission/export", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ items: missionPlainItems() }),
  });
  if (!res.ok) {
    const status = document.getElementById("missionstatus");
    if (status) status.textContent = "não foi possível exportar";
    return;
  }
  const blob = await res.blob();
  const a = document.createElement("a");
  a.href = URL.createObjectURL(blob);
  a.download = "mission.waypoints";
  a.click();
  URL.revokeObjectURL(a.href);
}

async function missionEdit(url, extra) {
  if (missionBusy || !missionItems.length) return;
  missionBusy = true;
  paint();
  const result = await missionPost(url, { items: missionPlainItems(), ...(extra || {}) });
  missionBusy = false;
  if (result.ok) {
    acceptMissionPayload(result.data, {
      dirty: true,
      selected: result.data.selected != null ? result.data.selected : missionSelected,
    });
  } else {
    paint();
    const status = document.getElementById("missionstatus");
    if (status) status.textContent = (result.data && result.data.note) || "falha";
  }
}

function makeMissionItem(command, lat, lon, extra = {}) {
  const spec = missionCmdSpec(command);
  const hasPos = !spec || spec.has_pos;
  return {
    seq: 0,
    frame: 3,
    command: Number(command),
    current: 0,
    autocontinue: 1,
    p1: extra.p1 || 0,
    p2: extra.p2 || 0,
    p3: extra.p3 || 0,
    p4: extra.p4 || 0,
    lat: hasPos ? lat : 0,
    lon: hasPos ? lon : 0,
    // WAYPOINT: Alt = rumo; -1 = passa direto (regra do mixer / mission_editor).
    z: extra.z != null ? extra.z : (Number(command) === CMD_WAYPOINT ? -1 : 0),
    name: (spec && spec.name) || ("CMD_" + command),
    has_pos: !!hasPos,
    direction: "",
    effect: "",
    warn: "",
    reverse: false,
  };
}

async function ensureMissionHome(lat, lon) {
  if (missionItems.length) return false;
  const gps = gpsLatLon();
  const home = makeMissionItem(
    CMD_WAYPOINT,
    gps ? gps[0] : lat,
    gps ? gps[1] : lon,
    { z: 0 },
  );
  home.current = 1;
  home.frame = 0;
  home.name = "WAYPOINT";
  missionItems = [home];
  // Sem GPS, o 1º clique só define HOME (como o Mission Planner separa home dos WPs).
  return !gps;
}

async function addMissionAt(lat, lon, command = CMD_WAYPOINT) {
  if (missionBusy) return;
  missionBusy = true;
  paint();
  try {
    const homeOnly = await ensureMissionHome(lat, lon);
    if (homeOnly) {
      missionSelected = 0;
      missionDirty = true;
      await refreshMissionAnalyse(0);
      const status = document.getElementById("missionstatus");
      if (status) status.textContent = "HOME definido — clique de novo para o 1º WAYPOINT";
      return;
    }
    const item = makeMissionItem(command, lat, lon);
    missionItems.push(item);
    missionItems.forEach((it, i) => { it.seq = i; });
    missionSelected = missionItems.length - 1;
    missionDirty = true;
    await refreshMissionAnalyse(missionSelected);
  } finally {
    missionBusy = false;
    paint();
  }
}

async function insertMissionCommand(command, lat, lon, atSeq) {
  if (missionBusy) return;
  missionBusy = true;
  paint();
  try {
    const homeOnly = await ensureMissionHome(lat, lon);
    if (homeOnly && command === CMD_WAYPOINT) {
      missionSelected = 0;
      missionDirty = true;
      await refreshMissionAnalyse(0);
      return;
    }
    if (!missionItems.length) {
      // comando não-WP sem home: cria home no ponto e segue
      await ensureMissionHome(lat, lon);
    }
    const item = makeMissionItem(command, lat, lon);
    let idx = atSeq != null ? Number(atSeq) + 1 : missionItems.length;
    idx = Math.max(1, Math.min(idx, missionItems.length));
    missionItems.splice(idx, 0, item);
    missionItems.forEach((it, i) => { it.seq = i; });
    missionSelected = idx;
    missionDirty = true;
    await refreshMissionAnalyse(missionSelected);
  } finally {
    missionBusy = false;
    paint();
  }
}

async function deleteMissionSeq(seq) {
  if (seq == null || seq <= 0 || seq >= missionItems.length || missionBusy) return;
  missionBusy = true;
  paint();
  try {
    missionItems.splice(seq, 1);
    missionItems.forEach((it, i) => { it.seq = i; });
    missionSelected = Math.min(seq, missionItems.length - 1);
    missionDirty = true;
    await refreshMissionAnalyse(missionSelected);
  } finally {
    missionBusy = false;
    paint();
  }
}

async function clearMissionLocal() {
  if (!missionItems.length || missionBusy) return;
  if (!window.confirm("Limpar a missão nesta tela? (não apaga a do rover até enviar)")) return;
  missionBusy = true;
  paint();
  try {
    const home = missionItems[0];
    missionItems = [home];
    missionItems[0].seq = 0;
    missionSelected = 0;
    missionDirty = true;
    await refreshMissionAnalyse(0);
  } finally {
    missionBusy = false;
    paint();
  }
}

function setMissionZ(z) {
  const i = missionSelected;
  if (i == null || !missionItems[i] || missionItems[i].command !== CMD_WAYPOINT || i === 0) return;
  missionItems[i].z = z;
  missionDirty = true;
  refreshMissionAnalyse(i);
}

function applyMissionAngle() {
  const input = document.getElementById("missionangle");
  if (!input) return;
  const ang = Number(String(input.value).replace(",", "."));
  if (!Number.isFinite(ang)) {
    const status = document.getElementById("missionstatus");
    if (status) status.textContent = "Digite um número de 0 a 359";
    return;
  }
  setMissionZ(((ang % 360) + 360) % 360);
}

function applyMissionFields() {
  const i = missionSelected;
  if (i == null || !missionItems[i]) return;
  const it = missionItems[i];
  const cmdEl = document.getElementById("missioncmd");
  // HOME (seq 0): só lat/lon — comando fica WAYPOINT/current=1.
  if (i > 0 && cmdEl) {
    const cmd = Number(cmdEl.value);
    if (Number.isFinite(cmd) && cmd !== it.command) {
      const spec = missionCmdSpec(cmd);
      it.command = cmd;
      it.name = (spec && spec.name) || ("CMD_" + cmd);
      it.has_pos = !spec || !!spec.has_pos;
      if (!it.has_pos) { it.lat = 0; it.lon = 0; }
      if (cmd === CMD_WAYPOINT && (it.z > 360 || it.z < -1)) it.z = -1;
    }
  }
  const keys = i === 0 ? ["lat", "lon"] : ["p1", "p2", "p3", "p4", "lat", "lon", "z"];
  for (const key of keys) {
    const el = document.getElementById("missionf-" + key);
    if (!el || el.disabled) continue;
    const v = Number(String(el.value).replace(",", "."));
    if (Number.isFinite(v)) it[key] = v;
  }
  missionDirty = true;
  refreshMissionAnalyse(i);
}

function destroyMissionMap() {
  clearRelativePreview();
  if (missionMap) {
    missionMap.remove();
    missionMap = null;
  }
  missionMarkers = {};
  missionLines = null;
  missionRoverMarker = null;
  missionHeadingLayer = null;
}

function wpIcon(seq, selected, warn, reverse, isHome) {
  const color = warn ? "#c00000" : (isHome ? "#c25e00" : (reverse ? "#e67e22" : "#073e73"));
  const ring = selected ? "3px solid #efeae3" : "2px solid #fff";
  return L.divIcon({
    className: "mission-marker",
    html: `<div class="mission-pin" style="background:${color};box-shadow:0 0 0 ${ring}">${seq}</div>`,
    iconSize: [28, 28],
    iconAnchor: [14, 14],
  });
}

function roverIcon(hdg) {
  const rot = Number.isFinite(hdg) ? hdg : 0;
  return L.divIcon({
    className: "mission-marker",
    html: `<div class="mission-rover" style="transform:rotate(${rot}deg)">▲</div>`,
    iconSize: [24, 24],
    iconAnchor: [12, 12],
  });
}

const CLEAR_TILE = "data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7";

function makeLocalGrid() {
  const Grid = L.GridLayer.extend({
    createTile(coords) {
      const tile = document.createElement("canvas");
      const size = this.getTileSize();
      tile.width = size.x;
      tile.height = size.y;
      const ctx = tile.getContext("2d");
      ctx.fillStyle = "#e7efe0";
      ctx.fillRect(0, 0, size.x, size.y);
      ctx.strokeStyle = "rgba(7, 62, 115, 0.16)";
      ctx.lineWidth = 1;
      ctx.beginPath();
      for (let i = 32; i < size.x; i += 32) {
        ctx.moveTo(i + 0.5, 0);
        ctx.lineTo(i + 0.5, size.y);
        ctx.moveTo(0, i + 0.5);
        ctx.lineTo(size.x, i + 0.5);
      }
      ctx.stroke();
      ctx.strokeStyle = "rgba(7, 62, 115, 0.4)";
      ctx.beginPath();
      ctx.moveTo(0.5, 0);
      ctx.lineTo(0.5, size.y);
      ctx.moveTo(0, 0.5);
      ctx.lineTo(size.x, 0.5);
      ctx.stroke();
      if (this._map) {
        const nw = this._map.unproject(L.point(coords.x * size.x, coords.y * size.y), coords.z);
        ctx.fillStyle = "#073e73";
        ctx.font = "12px Consolas, monospace";
        ctx.fillText(`${nw.lat.toFixed(5)}, ${nw.lng.toFixed(5)}`, 8, 16);
      }
      return tile;
    },
  });
  return new Grid({ maxZoom: 22, attribution: "grade local" });
}

function watchMapTiles(layer) {
  let ok = 0;
  let bad = 0;
  layer.on("tileload", () => { ok += 1; });
  layer.on("tileerror", () => {
    bad += 1;
    if (ok > 0 || bad < 4) return;
    const status = document.getElementById("missionstatus");
    if (!status || status.dataset.offline || status.textContent) return;
    status.dataset.offline = "1";
    status.textContent = "Sem internet: grade local. Clique, arraste e o rover continuam no mapa.";
  });
}

function ensureMissionMap() {
  if (typeof L === "undefined") {
    const status = document.getElementById("missionstatus");
    if (status) status.textContent = "Mapa não iniciou";
    return null;
  }
  const el = document.getElementById("missionleaflet");
  if (!el) return null;
  if (missionMap) {
    setTimeout(() => missionMap.invalidateSize(), 50);
    return missionMap;
  }
  const start = gpsLatLon() || DEFAULT_MAP;
  missionMap = L.map(el, { zoomControl: true, attributionControl: true }).setView(start, 18);
  const grid = makeLocalGrid();
  const tileOpts = { maxZoom: 22, maxNativeZoom: 19, errorTileUrl: CLEAR_TILE };
  const streets = L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
    ...tileOpts,
    attribution: "&copy; OpenStreetMap",
  });
  const sat = L.tileLayer("https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}", {
    ...tileOpts,
    attribution: "Esri",
  });
  watchMapTiles(streets);
  watchMapTiles(sat);
  grid.addTo(missionMap);
  streets.addTo(missionMap);
  L.control.layers({ Grade: L.layerGroup(), Mapa: streets, Satélite: sat }, null, { position: "topright" }).addTo(missionMap);
  L.control.scale({ imperial: false, position: "bottomleft" }).addTo(missionMap);
  missionLines = L.layerGroup().addTo(missionMap);
  missionHeadingLayer = L.layerGroup().addTo(missionMap);

  missionMap.on("click", async (ev) => {
    if (missionCtx || missionCtxIgnoreClick) {
      hideMissionCtx();
      return;
    }
    if (missionBusy || missionClickMode !== "add") return;
    await addMissionAt(ev.latlng.lat, ev.latlng.lng, CMD_WAYPOINT);
  });
  missionMap.on("contextmenu", (ev) => {
    L.DomEvent.preventDefault(ev.originalEvent);
    L.DomEvent.stopPropagation(ev);
    if (missionCtx) {
      // segundo botão direito: sai do menu em vez de ficar preso nele
      hideMissionCtx();
      return;
    }
    showMissionCtx(ev.originalEvent.clientX, ev.originalEvent.clientY, ev.latlng.lat, ev.latlng.lng, null);
  });
  missionMap.on("mousemove", (ev) => {
    const cur = document.getElementById("missioncursor");
    if (cur) cur.textContent = `${ev.latlng.lat.toFixed(6)}, ${ev.latlng.lng.toFixed(6)}`;
  });
  return missionMap;
}

function fitMissionBounds() {
  if (!missionMap) return;
  const pts = [];
  for (const it of missionItems) {
    if (it.has_pos && (it.lat || it.lon)) pts.push([it.lat, it.lon]);
  }
  const gps = gpsLatLon();
  if (gps) pts.push(gps);
  if (!pts.length) return;
  if (pts.length === 1) missionMap.setView(pts[0], 19);
  else missionMap.fitBounds(pts, { padding: [40, 40], maxZoom: 20 });
}

function syncMissionMapLayers() {
  if (!ensureMissionMap()) return;
  if (!missionLines) missionLines = L.layerGroup().addTo(missionMap);
  if (!missionHeadingLayer) missionHeadingLayer = L.layerGroup().addTo(missionMap);
  for (const key of Object.keys(missionMarkers)) {
    missionMap.removeLayer(missionMarkers[key]);
  }
  missionMarkers = {};
  missionLines.clearLayers();
  missionHeadingLayer.clearLayers();

  const path = [];
  for (let i = 0; i < missionItems.length; i++) {
    const it = missionItems[i];
    if (!(it.has_pos && (it.lat || it.lon))) continue;
    path.push({ i, it, latlng: [it.lat, it.lon] });
  }
  for (let k = 1; k < path.length; k++) {
    const a = path[k - 1];
    const b = path[k];
    const meters = distMeters(a.latlng[0], a.latlng[1], b.latlng[0], b.latlng[1]);
    const color = b.it.direction === "re" ? "#e67e22" : "#2ca02c";
    L.polyline([a.latlng, b.latlng], {
      color,
      weight: 5,
      dashArray: b.it.direction === "re" ? "8 6" : null,
      opacity: 0.95,
    }).addTo(missionLines);
    const mid = [(a.latlng[0] + b.latlng[0]) / 2, (a.latlng[1] + b.latlng[1]) / 2];
    const label = meters < 10 ? `${meters.toFixed(2)} m` : `${meters.toFixed(1)} m`;
    L.marker(mid, {
      interactive: false,
      keyboard: false,
      icon: L.divIcon({
        className: "mission-leglabel",
        html: `<span style="--leg:${color}">${label}${b.it.direction === "re" ? " ré" : ""}</span>`,
        iconSize: [0, 0],
        iconAnchor: [0, 0],
      }),
    }).addTo(missionLines);
  }
  for (const { i, it, latlng } of path) {
    const marker = L.marker(latlng, {
      draggable: true,
      icon: wpIcon(i, i === missionSelected, !!it.warn, it.direction === "re", i === 0),
      zIndexOffset: i === missionSelected ? 600 : 400,
    }).addTo(missionMap);
    marker.on("click", (ev) => {
      L.DomEvent.stopPropagation(ev);
      hideMissionCtx();
      missionSelected = i;
      paintMissionEditor();
    });
    marker.on("contextmenu", (ev) => {
      L.DomEvent.stopPropagation(ev);
      L.DomEvent.preventDefault(ev.originalEvent);
      if (missionCtx && missionCtx.seq === i) {
        hideMissionCtx();
        return;
      }
      missionSelected = i;
      showMissionCtx(ev.originalEvent.clientX, ev.originalEvent.clientY, it.lat, it.lon, i);
      paintMissionEditor();
    });
    marker.on("dragend", async () => {
      const p = marker.getLatLng();
      missionItems[i].lat = p.lat;
      missionItems[i].lon = p.lng;
      missionDirty = true;
      await refreshMissionAnalyse(i);
    });
    missionMarkers[i] = marker;
    if (i > 0 && it.command === CMD_WAYPOINT && it.z !== -1) {
      const ang = (((it.z % 360) + 360) % 360) * Math.PI / 180;
      const len = 0.00012;
      const tip = [it.lat + len * Math.cos(ang), it.lon + len * Math.sin(ang) / Math.cos(it.lat * Math.PI / 180)];
      L.polyline([latlng, tip], { color: "#d62728", weight: 3 }).addTo(missionHeadingLayer);
    }
  }
  paintRoverOnMap();
  if (missionPlanMode === "relative") paintRelativePreview();
}

function paintRoverOnMap() {
  if (!missionMap) return;
  const gps = gpsLatLon();
  if (!gps) {
    if (missionRoverMarker) {
      missionMap.removeLayer(missionRoverMarker);
      missionRoverMarker = null;
    }
    return;
  }
  const hdg = state.gps && state.gps.GPI_hdg;
  if (!missionRoverMarker) {
    missionRoverMarker = L.marker(gps, { icon: roverIcon(hdg), zIndexOffset: 1000, interactive: false }).addTo(missionMap);
  } else {
    missionRoverMarker.setLatLng(gps);
    missionRoverMarker.setIcon(roverIcon(hdg));
  }
}

function showMissionCtx(x, y, lat, lon, seq) {
  missionCtxIgnoreClick = false;
  missionCtx = { x, y, lat, lon, seq };
  const menu = document.getElementById("missionctx");
  if (!menu) return;
  menu.hidden = false;
  menu.style.left = Math.min(window.innerWidth - 220, Math.max(8, x)) + "px";
  menu.style.top = Math.min(window.innerHeight - 280, Math.max(8, y)) + "px";
  const del = menu.querySelector("[data-act='delete']");
  if (del) del.disabled = !(seq > 0);
}

function hideMissionCtx() {
  if (missionCtx) missionCtxIgnoreClick = true;
  missionCtx = null;
  const menu = document.getElementById("missionctx");
  if (menu) menu.hidden = true;
  // limpa o flag após o ciclo de clique atual (pointerdown → click)
  if (missionCtxIgnoreClick) {
    setTimeout(() => { missionCtxIgnoreClick = false; }, 0);
  }
}

async function onMissionCtx(act) {
  const ctx = missionCtx;
  hideMissionCtx();
  if (!ctx) return;
  if (act === "add") {
    if (ctx.seq != null) await insertMissionCommand(CMD_WAYPOINT, ctx.lat, ctx.lon, ctx.seq);
    else await addMissionAt(ctx.lat, ctx.lon, CMD_WAYPOINT);
  }
  else if (act === "loiter") await insertMissionCommand(CMD_LOITER_UNLIM, ctx.lat, ctx.lon, ctx.seq);
  else if (act === "loiter-time") await insertMissionCommand(CMD_LOITER_TIME, ctx.lat, ctx.lon, ctx.seq);
  else if (act === "rtl") await insertMissionCommand(CMD_RTL, ctx.lat, ctx.lon, ctx.seq);
  else if (act === "jump") await insertMissionCommand(CMD_DO_JUMP, ctx.lat, ctx.lon, ctx.seq);
  else if (act === "delete") await deleteMissionSeq(ctx.seq);
  else if (act === "here") {
    const gps = gpsLatLon();
    if (gps) await addMissionAt(gps[0], gps[1], CMD_WAYPOINT);
  }
  else if (act === "clear") await clearMissionLocal();
}

function paintMissionEditor(opts = {}) {
  const status = document.getElementById("missionstatus");
  if (!status && page === "editor-missao") return;
  if (page !== "editor-missao") return;
  const dirty = document.getElementById("missiondirty");
  const stats = document.getElementById("missionstats");
  if (dirty) dirty.textContent = missionDirty ? "alterações locais não enviadas" : "";
  if (stats) {
    const total = missionStats.total_m || 0;
    const n = missionStats.waypoints || 0;
    stats.textContent = missionItems.length
      ? `${missionItems.length} itens · ${n} pontos · ${total.toFixed(1)} m`
      : "sem missão";
  }
  const list = document.getElementById("missionlist");
  if (list) {
    if (!missionItems.length) {
      list.innerHTML = `<p class="paramcount">Nova missão, baixe do rover ou abra um .waypoints. Clique no mapa para adicionar WAYPOINT (Alt/rumo = −1).</p>`;
    } else {
      list.innerHTML = missionItems.map((it, i) => {
        const title = i === 0 ? "0 HOME" : `${it.seq} ${it.name || ("CMD_" + it.command)}`;
        let rumo = "";
        if (i > 0 && it.command === CMD_WAYPOINT) {
          rumo = it.z === -1 ? "rumo −1" : `rumo ${Math.round((((it.z % 360) + 360) % 360))}°`;
        }
        const bits = [it.direction, rumo || it.effect, it.warn].filter(Boolean).join(" · ");
        return `<button type="button" class="missionrow${i === missionSelected ? " on" : ""}${it.warn ? " warn" : ""}" data-seq="${i}">
          <span class="missionrow-title">${escText(title)}</span>
          <span class="missionrow-meta">${escText(bits)}</span>
        </button>`;
      }).join("");
      const on = list.querySelector(".missionrow.on");
      if (on) on.scrollIntoView({ block: "nearest" });
    }
  }
  paintMissionDetail();
  syncMissionMapLayers();
  if (opts.fit) fitMissionBounds();
}

function paintMissionDetail() {
  const box = document.getElementById("missioneditor");
  const selLabel = document.getElementById("missionsel");
  if (!box) return;
  const i = missionSelected;
  const it = i != null ? missionItems[i] : null;
  if (!it) {
    box.classList.add("off");
    if (selLabel) selLabel.textContent = "Selecione um item";
    box.querySelector("#missionfields").innerHTML = "";
    drawMissionCompass(null, false);
    return;
  }
  box.classList.remove("off");
  if (selLabel) selLabel.textContent = i === 0 ? "HOME" : `${it.name} #${i}`;
  const cmdEl = document.getElementById("missioncmd");
  if (cmdEl) {
    cmdEl.disabled = i === 0 || missionBusy;
    const opts = missionCommands();
    if (!cmdEl.dataset.ready && opts.length) {
      cmdEl.innerHTML = opts.map((c) => `<option value="${c.id}">${c.name}</option>`).join("");
      cmdEl.dataset.ready = "1";
    }
    cmdEl.value = String(it.command);
  }
  const fields = document.getElementById("missionfields");
  const spec = missionCmdSpec(it.command);
  const params = (spec && spec.params) || [];
  const showLatLon = i === 0 || it.has_pos;
  let html = "";
  if (showLatLon) {
    html += fieldRow("lat", "Lat", it.lat, 7);
    html += fieldRow("lon", "Lon", it.lon, 7);
  }
  for (const p of params) {
    // z em WAYPOINT = rumo (graus), nunca altitude neste rover.
    html += fieldRow(p.key, p.label, it[p.key], p.key === "z" && it.command === CMD_WAYPOINT ? 1 : 4, p.hint);
  }
  fields.innerHTML = html;

  const wpTools = document.getElementById("missionwptools");
  const isWp = i > 0 && it.command === CMD_WAYPOINT;
  if (wpTools) wpTools.hidden = !isWp;
  const pass = document.getElementById("missionpass");
  const rev = document.getElementById("missionrev");
  const angle = document.getElementById("missionangle");
  const revInfo = document.getElementById("missionrevinfo");
  if (isWp) {
    if (pass) pass.checked = it.z === -1;
    if (angle) {
      angle.value = it.z === -1 ? "" : String(Math.round((((it.z % 360) + 360) % 360)));
      angle.disabled = it.z === -1 || missionBusy;
    }
    if (rev) rev.checked = !!it.reverse;
    if (revInfo) {
      const nxt = missionItems.findIndex((row, j) => j > i && row.command < 100);
      revInfo.textContent = (nxt >= 0 && missionItems[nxt].reverse)
        ? "próximo trecho de ré: rumo = traseira (+180)"
        : "";
    }
    drawMissionCompass(it.z === -1 ? null : (((it.z % 360) + 360) % 360), true);
  } else {
    drawMissionCompass(null, false);
  }
}

function fieldRow(key, label, value, digits, hint) {
  const v = value == null ? "" : (Number.isFinite(Number(value))
    ? String(Math.round(Number(value) * 10 ** digits) / 10 ** digits)
    : "");
  return `<label class="missionfield"><span>${escText(label)}${hint ? ` <em>${escText(hint)}</em>` : ""}</span>
    <input id="missionf-${key}" data-key="${key}" type="number" step="any" value="${escText(v)}"></label>`;
}

function drawMissionCompass(angle, enabled) {
  const canvas = document.getElementById("missioncompass");
  if (!canvas) return;
  const dpr = window.devicePixelRatio || 1;
  const size = Math.min(canvas.clientWidth || 220, 260);
  const pw = Math.max(1, Math.floor(size * dpr));
  if (canvas.width !== pw || canvas.height !== pw) {
    canvas.width = pw;
    canvas.height = pw;
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, size, size);
  const c = size / 2;
  const r = size / 2 - 28;
  const fg = enabled ? cssVar("--ink") : cssVar("--muted");
  ctx.strokeStyle = fg;
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.arc(c, c, r, 0, Math.PI * 2);
  ctx.stroke();
  for (let d = 0; d < 360; d += 15) {
    const a = d * Math.PI / 180;
    const r0 = r - (d % 90 === 0 ? 10 : d % 45 === 0 ? 6 : 3);
    ctx.beginPath();
    ctx.moveTo(c + r0 * Math.sin(a), c - r0 * Math.cos(a));
    ctx.lineTo(c + r * Math.sin(a), c - r * Math.cos(a));
    ctx.lineWidth = d % 45 === 0 ? 2 : 1;
    ctx.stroke();
  }
  ctx.fillStyle = fg;
  ctx.font = "bold 11px Segoe UI";
  ctx.textAlign = "center";
  ctx.fillText("N", c, c - r - 6);
  if (enabled && angle != null) {
    const a = angle * Math.PI / 180;
    ctx.strokeStyle = "#d62728";
    ctx.fillStyle = "#d62728";
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(c, c);
    ctx.lineTo(c + (r - 12) * Math.sin(a), c - (r - 12) * Math.cos(a));
    ctx.stroke();
    ctx.font = "bold 13px Segoe UI";
    ctx.textAlign = "left";
    ctx.fillText(`${Math.round(angle)}°`, 6, 16);
  } else if (enabled) {
    ctx.fillStyle = cssVar("--muted");
    ctx.font = "13px Segoe UI";
    ctx.textAlign = "center";
    ctx.fillText("passa direto", c, c);
  }
}

function onMissionCompassClick(ev) {
  const canvas = document.getElementById("missioncompass");
  const i = missionSelected;
  if (!canvas || i == null || !missionItems[i] || missionItems[i].command !== CMD_WAYPOINT || i === 0) return;
  if (missionItems[i].z === -1) return;
  const rect = canvas.getBoundingClientRect();
  const size = Math.min(rect.width, rect.height);
  const c = size / 2;
  const x = ev.clientX - rect.left;
  const y = ev.clientY - rect.top;
  let ang = Math.atan2(x - c, c - y) * 180 / Math.PI;
  ang = ((ang % 360) + 360) % 360;
  const step = ev.shiftKey ? 1 : 5;
  ang = Math.round(ang / step) * step % 360;
  setMissionZ(ang);
}

function renderMission() {
  destroyMissionMap();
  hideMissionCtx();
  view.innerHTML = `
    <div class="missionplan">
      <aside class="mission-side">
        <div class="mission-side-head">
          <h1 class="page">Missão</h1>
          <p class="last">O mapa mostra a missão local: linhas = trechos, números = waypoints, ▲ = rover.
            <b>Ler</b> traz a do rover; <b>Escrever</b> grava o que está no mapa.
            Clique = WAYPOINT (Alt = rumo, −1 passa direto). Arraste para mover; botão direito = menu.</p>
        </div>
        <div class="mission-toolbar">
          <button type="button" class="missionact" id="missiondownload">Ler</button>
          <button type="button" class="missionact" id="missionupload" data-danger="1">Escrever</button>
          <button type="button" class="missionact missionlocal" id="missionnew">Nova</button>
          <button type="button" class="missionact" id="missionsquare" title="HOME no GPS + 4 waypoints de 1 m alinhados à frente do rover">Quadrado 1 m</button>
          <button type="button" class="missionact missionlocal" id="missionopen">Abrir</button>
          <button type="button" class="missionact missionlocal" id="missionsave">Salvar</button>
          <button type="button" class="missionact missionlocal" id="missionfit">Enquadrar</button>
          <button type="button" class="missionact" id="missioncenter">Rover</button>
          <button type="button" class="missionact missionlocal" id="missionallnext">Rumos→trecho</button>
          <button type="button" class="missionact missionlocal" id="missionfix">Corrigir rumo</button>
          <input id="missionfile" type="file" accept=".waypoints,.txt,text/plain" hidden>
        </div>
        <div class="mission-modes">
          <button type="button" class="missionmode on" id="missionmodemap">Mapa</button>
          <button type="button" class="missionmode" id="missionmoderel">Relativo</button>
        </div>
        <section class="missionbox" id="missionrelpanel" hidden>
          <h2>Passo relativo</h2>
          <p class="paramcount">A partir da origem: distância (m) + ângulo. Ex.: 1&nbsp;m à frente do rover, depois 2&nbsp;m a 90°.</p>
          <div class="missionfields">
            <label class="missionfield"><span>Origem</span>
              <select id="missionrelsrc" class="mission-select">
                <option value="rover">GPS do rover</option>
                <option value="fixed">Origem fixa (capturada)</option>
                <option value="last">Último WP / HOME</option>
                <option value="selected">WP selecionado</option>
              </select>
            </label>
            <label class="missionfield"><span>Ângulo relativo a</span>
              <select id="missionrelref" class="mission-select">
                <option value="rover">Frente do rover (0° = frente)</option>
                <option value="north">Norte (0° = N)</option>
                <option value="leg">Trecho / origem</option>
              </select>
            </label>
            <label class="missionfield"><span>Distância <em>m</em></span>
              <input id="missionreldist" type="number" step="0.1" min="0" value="1">
            </label>
            <label class="missionfield"><span>Ângulo <em>graus</em></span>
              <input id="missionrelang" type="number" step="1" value="0">
            </label>
          </div>
          <label class="missioncheck"><input type="checkbox" id="missionrelchain" checked> Depois do passo, origem = novo ponto</label>
          <label class="missioncheck"><input type="checkbox" id="missionrelalign" checked> Alinhar rumo do WP anterior para este trecho</label>
          <label class="missioncheck"><input type="checkbox" id="missionrelrumo"> Novo WP com rumo = ângulo do passo (senão −1)</label>
          <p class="paramcount" id="missionrelinfo"></p>
          <div class="mission-editor-actions">
            <button type="button" class="missionact" id="missionrelcapture">Capturar origem no rover</button>
            <button type="button" class="missionact missionlocal" id="missionrelpreview">Atualizar preview</button>
            <button type="button" class="missionact missionlocal solid" id="missionreladd">Adicionar passo</button>
          </div>
        </section>
        <p class="paramcount" id="missionstatus"></p>
        <p class="paramcount" id="missiondirty"></p>
        <p class="paramcount" id="missionstats"></p>
        <div class="scriptxfer" id="missionxfer" hidden>
          <div class="scriptxfer-label" id="missionxferlabel"></div>
          <div class="scriptxfer-track" aria-hidden="true"><div class="scriptxfer-fill" id="missionxferfill"></div></div>
          <div class="scriptxfer-meta" id="missionxfermeta"></div>
        </div>
        <div id="missionlist" class="missionlist"></div>
        <section class="missionbox" id="missioneditor">
          <div class="mission-editor-head">
            <h2 id="missionsel"></h2>
            <select id="missioncmd" class="mission-select"></select>
          </div>
          <div id="missionfields" class="missionfields"></div>
          <div class="mission-editor-actions">
            <button type="button" class="missionact missionlocal" id="missionapplyfields">Aplicar campos</button>
            <button type="button" class="missionact missionlocal" id="missiondelitem">Apagar item</button>
          </div>
          <div id="missionwptools">
            <label class="missioncheck"><input type="checkbox" id="missionpass"> Passa direto (rumo = −1)</label>
            <label class="missioncheck"><input type="checkbox" id="missionrev"> Chegar de ré</label>
            <p class="paramcount" id="missionrevinfo"></p>
            <div class="ctrl">
              <input id="missionangle" type="number" step="1" min="0" max="359" placeholder="rumo °">
              <button type="button" class="missionact missionlocal" id="missionapply">Aplicar rumo</button>
            </div>
            <button type="button" class="missionact missionlocal" id="missionnextleg">Rumo do próximo trecho</button>
            <canvas id="missioncompass" class="missioncanvas" width="220" height="220"></canvas>
          </div>
        </section>
      </aside>
      <div class="mission-mapwrap">
        <div id="missionleaflet" class="mission-leaflet"></div>
        <div class="mission-mapbar">
          <span id="missioncursor">—</span>
          <span class="mission-hint">clique = adicionar · arraste = mover · direito = menu</span>
        </div>
      </div>
    </div>
    <div id="missionctx" class="missionctx" hidden>
      <button type="button" data-act="add">Inserir WAYPOINT</button>
      <button type="button" data-act="loiter">Loiter aqui</button>
      <button type="button" data-act="loiter-time">Loiter tempo</button>
      <button type="button" data-act="rtl">RTL</button>
      <button type="button" data-act="jump">DO_JUMP</button>
      <button type="button" data-act="here">WP na posição do rover</button>
      <button type="button" data-act="delete">Apagar WP</button>
      <button type="button" data-act="clear">Limpar missão</button>
    </div>`;

  document.getElementById("missionfile").addEventListener("change", () => {
    const input = document.getElementById("missionfile");
    const f = input.files && input.files[0];
    input.value = "";
    if (f) importMissionFile(f);
  });
  const compass = document.getElementById("missioncompass");
  if (compass) compass.addEventListener("pointerdown", onMissionCompassClick);
  const angle = document.getElementById("missionangle");
  if (angle) angle.addEventListener("keydown", (ev) => { if (ev.key === "Enter") applyMissionAngle(); });
  const pass = document.getElementById("missionpass");
  if (pass) pass.addEventListener("change", () => { if (pass.checked) setMissionZ(-1); else setMissionZ(0); });
  const rev = document.getElementById("missionrev");
  if (rev) rev.addEventListener("change", () => {
    if (missionSelected == null) return;
    missionEdit("/mission/rebuild", { seq: missionSelected, reverse: !!rev.checked });
  });
  document.getElementById("missionctx").addEventListener("click", (ev) => {
    const btn = ev.target.closest("button[data-act]");
    if (btn) onMissionCtx(btn.dataset.act);
  });
  for (const id of ["missionreldist", "missionrelang", "missionrelsrc", "missionrelref"]) {
    const el = document.getElementById(id);
    if (!el) continue;
    el.addEventListener("input", paintRelativePreview);
    el.addEventListener("change", paintRelativePreview);
  }
  for (const id of ["missionreldist", "missionrelang"]) {
    const el = document.getElementById(id);
    if (!el) continue;
    el.addEventListener("keydown", (ev) => {
      if (ev.key === "Enter") addRelativeStep();
    });
  }
  document.removeEventListener("pointerdown", hideMissionCtxOnOutside, true);
  document.removeEventListener("keydown", hideMissionCtxOnKey);
  document.addEventListener("pointerdown", hideMissionCtxOnOutside, true);
  document.addEventListener("keydown", hideMissionCtxOnKey);
  ensureMissionMap();
  setMissionPlanMode(missionPlanMode);
  paintMissionEditor({ fit: !!missionItems.length });
}

function hideMissionCtxOnOutside(ev) {
  const menu = document.getElementById("missionctx");
  if (!menu || menu.hidden || !missionCtx) return;
  if (menu.contains(ev.target)) return;
  // botão direito no mapa: o handler do Leaflet cuida (toggle)
  if (ev.button === 2) return;
  hideMissionCtx();
}

function hideMissionCtxOnKey(ev) {
  if (ev.key === "Escape") hideMissionCtx();
}

function renderProcesses() {
  const cards = (catalog.processes || []).map((p) => `
    <article class="card">
      <h2>${p.title}</h2>
      <p>${p.text}</p>
      <p class="sets">${p.sets}</p>
      <button type="button" class="cmd proc" data-proc="${p.id}">Preparar</button>
    </article>`).join("");
  view.innerHTML = `<h1 class="page">Ensaios</h1><div class="cards">${cards}</div>`;
}

view.addEventListener("click", (ev) => {
  if (ev.target.closest("#armbtn")) {
    onArm();
    return;
  }
  if (ev.target.closest("#logstart")) {
    const name = (document.getElementById("logname").value || "").trim();
    post("/log", { action: "start", name });
    return;
  }
  if (ev.target.closest("#logstop")) {
    post("/log", { action: "stop" });
    return;
  }
  if (ev.target.closest("#sdlogrefresh")) {
    const btn = ev.target.closest("#sdlogrefresh");
    if (!btn.disabled) loadSdLogs();
    return;
  }
  const sdErase = ev.target.closest("#sdlogerase");
  if (sdErase && !sdErase.disabled) {
    if (confirm && confirm.el === sdErase && Date.now() < confirm.until) {
      confirm = null;
      sdErase.textContent = "Apagar todos";
      sdErase.className = "cmd sdlogact";
      eraseSdLogs();
      return;
    }
    confirm = {
      el: sdErase,
      until: Date.now() + 4000,
      label: "Apagar todos",
      className: "cmd sdlogact",
    };
    sdErase.textContent = "Apaga todos os .BIN — confirmar";
    sdErase.className = "cmd sdlogact solid";
    return;
  }
  const sdDl = ev.target.closest("button.sdlogdl");
  if (sdDl && !sdDl.disabled) {
    downloadSdLog(sdDl.dataset.id, sdDl.dataset.size);
    return;
  }
  const modeBtn = ev.target.closest("button.mode");
  if (modeBtn && !modeBtn.disabled) {
    post("/mode", { name: modeBtn.dataset.mode });
    return;
  }
  const apply = ev.target.closest("button.apply");
  if (apply && !apply.disabled) {
    const input = apply.parentElement.querySelector("input");
    const value = Number(input.value);
    if (!Number.isFinite(value)) return;
    input.dataset.touched = "";
    delete pending[apply.dataset.param];
    apply.closest("article").classList.remove("dirty");
    post("/param", { name: apply.dataset.param, value });
    return;
  }
  const tool = ev.target.closest("button.tool");
  if (tool && !tool.disabled) {
    onTool(tool);
    return;
  }
  if (ev.target.closest("#scriptrefresh")) {
    const btn = ev.target.closest("#scriptrefresh");
    if (!btn.disabled) loadScripts();
    return;
  }
  const add = ev.target.closest("button.scriptadd");
  if (add && !add.disabled) {
    const input = add.parentElement.querySelector("input.scriptfile");
    if (input) input.click();
    return;
  }
  const copy = ev.target.closest("button.scriptcopy");
  if (copy && !copy.disabled) {
    copyScript(copy.dataset.folder, copy.dataset.name);
    return;
  }
  const del = ev.target.closest("button.scriptdel");
  if (del && !del.disabled) {
    if (confirm && confirm.el === del && Date.now() < confirm.until) {
      const folder = del.dataset.folder;
      const name = del.dataset.name;
      confirm = null;
      del.textContent = "Excluir";
      del.className = "scriptdel scriptact";
      removeScript(folder, name);
      return;
    }
    confirm = {
      el: del,
      until: Date.now() + 4000,
      label: "Excluir",
      className: "scriptdel scriptact",
    };
    del.textContent = "Confirmar";
    del.className = "scriptdel scriptact solid";
    return;
  }
  const clearBak = ev.target.closest("#scriptclearbak");
  if (clearBak && !clearBak.disabled) {
    if (confirm && confirm.el === clearBak && Date.now() < confirm.until) {
      confirm = null;
      clearBak.textContent = "Limpar STRG_BAK";
      clearBak.className = "cmd scriptact";
      clearStorageBak();
      return;
    }
    confirm = {
      el: clearBak,
      until: Date.now() + 4000,
      label: "Limpar STRG_BAK",
      className: "cmd scriptact",
    };
    clearBak.textContent = "Confirmar";
    clearBak.className = "cmd scriptact solid";
    return;
  }
  const formatSd = ev.target.closest("#scriptformat");
  if (formatSd && !formatSd.disabled) {
    if (confirm && confirm.el === formatSd && Date.now() < confirm.until) {
      confirm = null;
      formatSd.textContent = "Formatar SD";
      formatSd.className = "cmd scriptact";
      formatSdCard();
      return;
    }
    confirm = {
      el: formatSd,
      until: Date.now() + 4000,
      label: "Formatar SD",
      className: "cmd scriptact",
    };
    formatSd.textContent = "Apaga o cartão — confirmar";
    formatSd.className = "cmd scriptact solid";
    return;
  }
  const reboot = ev.target.closest("#scriptreboot");
  if (reboot && !reboot.disabled) {
    if (confirm && confirm.el === reboot && Date.now() < confirm.until) {
      confirm = null;
      reboot.textContent = "Reiniciar a placa";
      reboot.className = "cmd scriptact";
      post("/params/reboot", {});
      return;
    }
    confirm = {
      el: reboot,
      until: Date.now() + 4000,
      label: "Reiniciar a placa",
      className: "cmd scriptact",
    };
    reboot.textContent = "Confirmar";
    reboot.className = "cmd scriptact solid";
    return;
  }
  if (ev.target.closest("#missiondownload")) {
    const btn = ev.target.closest("#missiondownload");
    if (!btn.disabled) downloadMission();
    return;
  }
  if (ev.target.closest("#missionupload")) {
    const btn = ev.target.closest("#missionupload");
    if (!btn.disabled) {
      if (confirm && confirm.el === btn && Date.now() < confirm.until) {
        confirm = null;
        btn.textContent = "Escrever";
        btn.className = "missionact";
        uploadMission();
        return;
      }
      confirm = {
        el: btn,
        until: Date.now() + 4000,
        label: "Escrever",
        className: "missionact",
      };
      btn.textContent = "Substitui — confirmar";
      btn.className = "missionact solid";
    }
    return;
  }
  if (ev.target.closest("#missionmodemap")) {
    setMissionPlanMode("map");
    return;
  }
  if (ev.target.closest("#missionmoderel")) {
    setMissionPlanMode("relative");
    return;
  }
  if (ev.target.closest("#missionrelcapture")) {
    if (!ev.target.closest("#missionrelcapture").disabled) captureRelativeOriginFromRover();
    return;
  }
  if (ev.target.closest("#missionrelpreview")) {
    paintRelativePreview();
    return;
  }
  if (ev.target.closest("#missionreladd")) {
    if (!ev.target.closest("#missionreladd").disabled) addRelativeStep();
    return;
  }
  if (ev.target.closest("#missionnew")) {
    if (!ev.target.closest("#missionnew").disabled) newMission();
    return;
  }
  if (ev.target.closest("#missionsquare")) {
    if (!ev.target.closest("#missionsquare").disabled) makeSquareTestMission(1);
    return;
  }
  if (ev.target.closest("#missionopen")) {
    const input = document.getElementById("missionfile");
    if (input && !missionBusy) input.click();
    return;
  }
  if (ev.target.closest("#missionsave")) {
    if (!ev.target.closest("#missionsave").disabled) exportMissionFile();
    return;
  }
  if (ev.target.closest("#missionfit")) {
    fitMissionBounds();
    return;
  }
  if (ev.target.closest("#missioncenter")) {
    const gps = gpsLatLon();
    if (gps && missionMap) missionMap.setView(gps, Math.max(missionMap.getZoom(), 18));
    return;
  }
  if (ev.target.closest("#missionallnext")) {
    if (!ev.target.closest("#missionallnext").disabled) missionEdit("/mission/all-next");
    return;
  }
  if (ev.target.closest("#missionfix")) {
    if (!ev.target.closest("#missionfix").disabled) missionEdit("/mission/fix");
    return;
  }
  if (ev.target.closest("#missionapply")) {
    if (!ev.target.closest("#missionapply").disabled) applyMissionAngle();
    return;
  }
  if (ev.target.closest("#missionapplyfields")) {
    if (!ev.target.closest("#missionapplyfields").disabled) applyMissionFields();
    return;
  }
  if (ev.target.closest("#missiondelitem")) {
    if (!ev.target.closest("#missiondelitem").disabled) deleteMissionSeq(missionSelected);
    return;
  }
  if (ev.target.closest("#missionnextleg")) {
    if (!ev.target.closest("#missionnextleg").disabled) {
      missionEdit("/mission/heading", { seq: missionSelected });
    }
    return;
  }
  const row = ev.target.closest("button.missionrow");
  if (row) {
    missionSelected = Number(row.dataset.seq);
    paintMissionEditor();
    return;
  }
  const proc = ev.target.closest("button.proc");
  if (!proc || proc.disabled) return;
  if (confirm && confirm.el === proc && Date.now() < confirm.until) {
    const id = proc.dataset.proc;
    confirm = null;
    proc.textContent = "Preparar";
    proc.className = "cmd proc";
    post("/process", { id });
    return;
  }
  confirm = { kind: "proc", el: proc, until: Date.now() + 4000 };
  proc.textContent = "Confirmar";
  proc.className = "cmd proc solid";
});

function route() {
  page = pageId();
  confirm = null;
  if (page !== "scripts") stopScriptXferPoll();
  if (page !== "logs") stopSdlogXferPoll();
  if (page !== "editor-missao") {
    stopMissionXferPoll();
    destroyMissionMap();
    document.removeEventListener("pointerdown", hideMissionCtxOnOutside, true);
    document.removeEventListener("keydown", hideMissionCtxOnKey);
    hideMissionCtx();
  }
  renderNav();
  if (!catalog) { view.textContent = "Carregando."; return; }
  if (page === "logs") renderLogs();
  else if (page === "mensagens") renderMessages();
  else if (page === "modos") renderModes();
  else if (page === "parametros") renderParametros();
  else if (page === "editor-missao") renderMission();
  else if (page === "scripts") renderScripts();
  else if (page === "ensaios") renderProcesses();
  paint();
}

async function pollState() {
  try { state = await (await fetch("/state")).json(); }
  catch (e) { return; }
  paint();
}

async function pollTelem() {
  if (page !== "logs") return;
  try { telem = await (await fetch("/telemetry")).json(); }
  catch (e) { return; }
  telemAt = performance.now();
}

function liveFrame(now) {
  const dt = frameAt ? Math.min(0.05, (now - frameAt) / 1000) : 0.016;
  frameAt = now;
  if (page === "logs" && document.querySelector("canvas[data-chart]")) {
    paintReadouts(dt);
    paintNumbers(dt);
    for (const canvas of document.querySelectorAll("canvas[data-chart]")) {
      if (canvas.closest(".off")) continue;
      drawChart(canvas, CHARTS[Number(canvas.dataset.chart)], telem);
    }
  }
  requestAnimationFrame(liveFrame);
}
requestAnimationFrame(liveFrame);

window.addEventListener("hashchange", route);
Promise.all([
  fetch("/catalog").then((r) => r.json()),
  fetch("/param_docs.json").then((r) => (r.ok ? r.json() : {})).catch(() => ({})),
]).then(([data, docs]) => {
  catalog = data;
  paramDocs = docs || {};
  if (!location.hash) location.replace("#logs");
  route();
  pollState();
  setInterval(pollState, 250);
  setInterval(pollTelem, 120);
}).catch(() => { view.textContent = "Não foi possível abrir o catálogo."; });
