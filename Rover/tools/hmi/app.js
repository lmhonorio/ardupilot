const PAGES = [
  ["logs", "Logs"],
  ["mensagens", "Mensagens"],
  ["modos", "Modos"],
  ["parametros", "Parâmetros"],
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
  for (const btn of document.querySelectorAll("button.apply, button.proc, button.scriptact, button.sdlogact")) {
    if (btn.id === "armbtn") continue;
    const pending = confirm && confirm.el === btn && Date.now() < confirm.until;
    const busy = (btn.classList.contains("scriptact") && scriptBusy)
      || (btn.classList.contains("sdlogact") && sdlogBusy);
    const needsDisarm = btn.dataset.danger === "1";
    if (!pending) {
      btn.disabled = !state.seen || busy || (!!state.armed && needsDisarm);
    }
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
  renderNav();
  if (!catalog) { view.textContent = "Carregando."; return; }
  if (page === "logs") renderLogs();
  else if (page === "mensagens") renderMessages();
  else if (page === "modos") renderModes();
  else if (page === "parametros") renderParametros();
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
