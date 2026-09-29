// ARCNET dashboard. Renders backend state only, never moves robots itself.
const API = location.protocol === "file:" ? "http://localhost:8000" : "";
const WSURL = API ? "ws://localhost:8000/ws" : (location.protocol === "https:" ? "wss://" : "ws://") + location.host + "/ws";
const $ = id => document.getElementById(id);
const COL = { normal: "#e0a84a", caution: "#ff6a3d", hard_stop: "#e5484d", failed: "#6b625b" };
const HINT = {
  view: "Watch the fleet. Pick a mission on the left to load a layout.",
  dispatch: "Click a robot, then click a cell to send it there as a new task.",
  block: "Click a cell to drop an obstacle in the aisle.",
  wifi: "Click a cell to create a communication dead zone.",
  kill: "Click a robot to fail it and watch the fleet recover."
};
let S = null, tool = "view", picked = null, view = "2d", wasRunning = false, taskSig = "", L = null, T = null;

const safe = r => (!r.active || r.mode === "failed") ? "failed" : (r.safety || "normal");
const cellOf = (x, y) => [Math.floor(x / S.grid.cell_size), Math.floor(y / S.grid.cell_size)];
const xy = c => Array.isArray(c) ? c : [c.x, c.y];

function toast(msg) {
  const t = $("toast");
  t.textContent = msg; t.hidden = false;
  clearTimeout(toast.h);
  toast.h = setTimeout(() => t.hidden = true, 2600);
}

async function api(path, method = "POST", body = null) {
  try {
    const r = await fetch(API + path, {
      method, headers: { "Content-Type": "application/json" },
      body: body ? JSON.stringify(body) : undefined
    });
    if (!r.ok) throw new Error(r.status);
    return await r.json();
  } catch (e) {
    toast("Request failed: " + path);
    return null;
  }
}

function connect() {
  const ws = new WebSocket(WSURL);
  ws.onopen = () => setPill("Connected", "ok");
  ws.onclose = () => { setPill("Disconnected", ""); setTimeout(connect, 2000); };
  ws.onmessage = e => { S = JSON.parse(e.data); update(); };
}
function setPill(text, cls) { const p = $("pill"); p.textContent = text; p.className = "pill " + cls; }

function update() {
  const m = S.metrics || {};
  const total = S.tasks.length, done = S.tasks.filter(t => t.completed).length;
  $("hTime").textContent = S.time.toFixed(1);
  $("hTasks").textContent = done + "/" + total;
  $("hCol").textContent = m.collisions ?? 0;
  $("hNear").textContent = m.near_misses ?? 0;
  $("hDead").textContent = m.deadlocks ?? 0;
  $("hCol").parentElement.classList.toggle("bad", (m.collisions ?? 0) > 0);
  if (S.running) setPill("Running", "run"); else if ($("pill").textContent === "Running") setPill("Connected", "ok");

  renderFleet(); renderGoals(total, done, m); renderEvents(); renderTasks();
  document.querySelectorAll(".mis").forEach(b => b.classList.toggle("on", b.dataset.id === S.scenario));

  if (S.running) $("banner").hidden = true;
  if (wasRunning && !S.running && total > 0 && done === total) showBanner(done);
  wasRunning = S.running;
  if (view === "2d") draw2d();
}

function renderFleet() {
  $("nFleet").textContent = S.robots.length;
  $("fleet").innerHTML = S.robots.map(r => {
    const st = safe(r), bat = Math.max(0, Math.min(100, r.battery ?? 0));
    const line = st === "failed" ? "Failed"
      : r.mode === "idle" ? (r.task_id ? "Task " + r.task_id : "Idle")
      : (r.task_id ? r.task_id + ": " : "") + (r.reason || r.mode);
    const why = r.waiting_for ? `Waiting for ${r.waiting_for}, priority tier ${r.priority_tier}, clock ${r.logical_clock}` : "";
    return `<div class="rb ${picked === r.id ? "pick" : ""} ${st === "failed" ? "dead" : ""}" data-id="${r.id}">
      <div><strong>${r.id}</strong><span class="chip ${st}">${st.replace("_", " ")}</span></div>
      <small>${line}</small>${why ? `<small class="why">${why}</small>` : ""}
      <div class="bar"><i style="width:${bat}%"></i></div></div>`;
  }).join("") || '<p class="mute">No robots loaded.</p>';
}

function renderGoals(total, done, m) {
  const all = total > 0 && done === total;
  const clean = (m.near_misses ?? 0) === 0 && (m.deadlocks ?? 0) === 0 && (m.collisions ?? 0) === 0;
  const rows = [
    ["Zero collisions", (m.collisions ?? 0) > 0 ? "lost" : all ? "earned" : "holding"],
    ["Delivered " + done + "/" + total, all ? "earned" : "holding"],
    ["Smooth run", !clean ? "lost" : all ? "earned" : "holding"]
  ];
  $("goals").innerHTML = rows.map(([a, s]) => `<span class="gl ${s}">${a}</span>`).join("");
}

const KIND = { SAFETY: "Safety", DEADLOCK: "Deadlock", YIELD: "Yield", TASK: "Task", FAULT: "Fault", REROUTE: "Reroute", SYSTEM: "System", SCENARIO: "System" };
const GROUP = { all: null, safety: ["SAFETY"], conflict: ["DEADLOCK", "YIELD"], task: ["TASK"], fault: ["FAULT", "REROUTE"] };
let logFilter = "all";

function renderEvents() {
  const g = GROUP[logFilter];
  const list = [...S.events].reverse().filter(e => !g || g.includes(e.kind)).slice(0, 60);
  $("events").innerHTML = list.map(e =>
    `<div class="lg ${e.kind || ""}"><div class="lh"><time>${Number(e.time).toFixed(1)}s</time><b>${KIND[e.kind] || e.kind}</b><span>${e.source}</span></div><p>${e.message}</p></div>`
  ).join("") || '<p class="mute">Nothing here yet.</p>';
}

function renderTasks() {
  $("nTasks").textContent = S.tasks.filter(t => !t.completed).length;
  const sig = JSON.stringify(S.tasks) + S.robots.map(r => r.id).join();
  if (sig === taskSig) return;
  taskSig = sig;
  const opts = sel => '<option value="">unassigned</option>' +
    S.robots.map(r => `<option ${r.id === sel ? "selected" : ""}>${r.id}</option>`).join("");
  $("taskList").innerHTML = S.tasks.map(t => {
    const label = t.completed ? "completed" : (t.phase || t.status);
    return `<div class="tk"><div><b>${t.id}</b><small>(${t.start.join(", ")}) to (${t.destination.join(", ")})</small></div>
      <span class="chip ${t.completed ? "completed" : t.status}">${label}</span>
      <select data-task="${t.id}">${opts(t.assigned_robot)}</select></div>`;
  }).join("") || '<p class="mute">No tasks.</p>';
  $("tRobot").innerHTML = opts("");
}

function showBanner(done) {
  const m = S.metrics || {}, clean = !(m.collisions || m.near_misses || m.deadlocks);
  $("banner").innerHTML = `<h3>Mission complete</h3><p>${done} tasks in ${S.time.toFixed(1)} s</p>
    <p class="mute">${m.collisions ? m.collisions + " collision(s) recorded" : "Zero collisions"}${clean ? ", smooth run" : ""}</p>`;
  $("banner").hidden = false;
}

function draw2d() {
  const cv = $("c2d"), box = cv.parentElement.getBoundingClientRect(), d = devicePixelRatio || 1;
  cv.width = box.width * d; cv.height = box.height * d;
  const g = cv.getContext("2d"); g.setTransform(d, 0, 0, d, 0, 0);
  const { width: gw, height: gh, cell_size: m } = S.grid;
  const cs = Math.min((box.width - 30) / gw, (box.height - 50) / gh);
  const ox = (box.width - cs * gw) / 2, oy = (box.height - cs * gh) / 2 + 8;
  L = { cs, ox, oy, gw, gh };
  const cx = c => ox + (c + 0.5) * cs, cy = c => oy + (c + 0.5) * cs;

  g.fillStyle = "#130d0a"; g.fillRect(ox, oy, cs * gw, cs * gh);
  g.strokeStyle = "#241812"; g.lineWidth = 1; g.beginPath();
  for (let i = 0; i <= gw; i++) { g.moveTo(ox + i * cs, oy); g.lineTo(ox + i * cs, oy + gh * cs); }
  for (let j = 0; j <= gh; j++) { g.moveTo(ox, oy + j * cs); g.lineTo(ox + gw * cs, oy + j * cs); }
  g.stroke();

  (S.wifi || []).forEach(z => { g.fillStyle = "#e0a84a22"; g.fillRect(ox + z.x * cs, oy + z.y * cs, cs, cs); });
  S.obstacles.forEach(o => {
    g.fillStyle = o.temporary ? "#7a1f26" : "#3a1a1e";
    g.fillRect(ox + o.x * cs + 1, oy + o.y * cs + 1, cs - 2, cs - 2);
  });

  S.tasks.filter(t => !t.completed).forEach(t => {
    g.strokeStyle = "#e0a84a"; g.setLineDash([3, 3]);
    g.strokeRect(ox + t.start[0] * cs + 3, oy + t.start[1] * cs + 3, cs - 6, cs - 6); g.setLineDash([]);
    g.fillStyle = "#ff6a3d"; g.beginPath();
    const x = cx(t.destination[0]), y = cy(t.destination[1]), r = cs * 0.22;
    g.moveTo(x, y - r); g.lineTo(x + r, y); g.lineTo(x, y + r); g.lineTo(x - r, y); g.fill();
  });

  S.robots.forEach(r => {
    if (!r.plan || r.plan.length < 2) return;
    g.strokeStyle = COL[safe(r)] + "88"; g.lineWidth = 2; g.setLineDash([5, 4]); g.beginPath();
    r.plan.forEach((c, i) => { const [a, b] = xy(c); i ? g.lineTo(cx(a), cy(b)) : g.moveTo(cx(a), cy(b)); });
    g.stroke(); g.setLineDash([]);
  });

  const size = S.robot_size || [0.9, 0.64];
  S.robots.forEach(r => {
    const c = COL[safe(r)], px = ox + r.x / m * cs, py = oy + r.y / m * cs;
    const len = size[0] / m * cs, wid = size[1] / m * cs;
    const gr = g.createRadialGradient(px, py, 0, px, py, cs * 1.1);
    gr.addColorStop(0, c + "55"); gr.addColorStop(1, c + "00");
    g.fillStyle = gr; g.beginPath(); g.arc(px, py, cs * 1.1, 0, 7); g.fill();
    g.save(); g.translate(px, py); g.rotate(r.theta || 0);
    g.fillStyle = "#241914"; g.strokeStyle = c; g.lineWidth = picked === r.id ? 3 : 2;
    g.beginPath(); g.roundRect(-len / 2, -wid / 2, len, wid, 4); g.fill(); g.stroke();
    g.fillStyle = c; g.fillRect(len / 2 - 4, -wid / 4, 4, wid / 2);
    g.restore();
    g.fillStyle = "#efe6da"; g.font = "600 12px Inter,sans-serif"; g.textAlign = "center";
    g.fillText(r.id, px, py - wid / 2 - 6);
  });
}

function onStageClick(e) {
  if (!S || !L || view !== "2d") return;
  const b = $("c2d").getBoundingClientRect();
  const cx = Math.floor((e.clientX - b.left - L.ox) / L.cs), cy = Math.floor((e.clientY - b.top - L.oy) / L.cs);
  if (cx < 0 || cy < 0 || cx >= L.gw || cy >= L.gh) return;
  const near = S.robots.map(r => [r, Math.hypot(r.x / S.grid.cell_size - cx - 0.5, r.y / S.grid.cell_size - cy - 0.5)])
    .sort((a, c) => a[1] - c[1])[0];
  const hit = near && near[1] < 0.9 ? near[0] : null;
  if (tool === "block") api("/api/fault", "POST", { kind: "blocked_cell", x: cx, y: cy });
  else if (tool === "wifi") api("/api/fault", "POST", { kind: "wifi_dead_zone", x: cx, y: cy });
  else if (tool === "kill" && hit) api("/api/fault", "POST", { kind: "robot_failure", target: hit.id });
  else if (tool === "dispatch") {
    if (hit) { picked = hit.id; toast("Picked " + hit.id + ". Now click a goal cell."); return; }
    if (!picked) return toast("Click a robot first.");
    const r = S.robots.find(q => q.id === picked), s = cellOf(r.x, r.y);
    let n = S.tasks.length + 1, id;
    do { id = "T" + String(n++).padStart(3, "0"); } while (S.tasks.some(t => t.id === id));
    api("/api/task", "POST", { id, start: s, destination: [cx, cy], assigned_robot: picked });
  }
}

// 3D twin: drag to orbit, wheel to zoom. Positions come from backend state.
function init3d() {
  const el = $("c3d"), m = S.grid.cell_size, W = S.grid.width * m, H = S.grid.height * m;
  const r = new THREE.WebGLRenderer({ antialias: true });
  r.setPixelRatio(devicePixelRatio); r.setSize(el.clientWidth, el.clientHeight); el.appendChild(r.domElement);
  const sc = new THREE.Scene(); sc.background = new THREE.Color(0x0b0908);
  const cam = new THREE.PerspectiveCamera(45, el.clientWidth / el.clientHeight, 0.1, 300);
  sc.add(new THREE.AmbientLight(0xffffff, 0.6));
  const dl = new THREE.DirectionalLight(0xffb27a, 0.9); dl.position.set(W / 2, 30, H / 2); sc.add(dl);
  const fl = new THREE.Mesh(new THREE.PlaneGeometry(W, H), new THREE.MeshStandardMaterial({ color: 0x140e0b }));
  fl.rotation.x = -Math.PI / 2; fl.position.set(W / 2, 0, H / 2); sc.add(fl);
  T = { r, sc, cam, c: new THREE.Vector3(W / 2, 0, H / 2), a: 0.6, e: 0.85, d: Math.max(W, H) * 1.1, rob: {}, obs: new THREE.Group(), key: "" };
  sc.add(T.obs);
  let drag = null;
  r.domElement.onmousedown = e => drag = [e.clientX, e.clientY];
  addEventListener("mouseup", () => drag = null);
  addEventListener("mousemove", e => {
    if (!drag) return;
    T.a -= (e.clientX - drag[0]) * 0.008;
    T.e = Math.min(1.5, Math.max(0.2, T.e + (e.clientY - drag[1]) * 0.006));
    drag = [e.clientX, e.clientY];
  });
  r.domElement.onwheel = e => { e.preventDefault(); T.d = Math.min(90, Math.max(6, T.d * (1 + e.deltaY * 0.001))); };
}

function loop3d() {
  if (view !== "3d") return;
  requestAnimationFrame(loop3d);
  if (!S || !T) return;
  const m = S.grid.cell_size, key = JSON.stringify(S.obstacles);
  if (key !== T.key) {
    T.key = key; T.obs.clear();
    S.obstacles.forEach(o => {
      const b = new THREE.Mesh(new THREE.BoxGeometry(m, 1.2, m),
        new THREE.MeshStandardMaterial({ color: o.temporary ? 0x7a1f26 : 0x3a2a22 }));
      b.position.set((o.x + 0.5) * m, 0.6, (o.y + 0.5) * m); T.obs.add(b);
    });
  }
  const size = S.robot_size || [0.9, 0.64];
  S.robots.forEach(r => {
    let mesh = T.rob[r.id];
    if (!mesh) {
      mesh = new THREE.Mesh(new THREE.BoxGeometry(size[0], 0.35, size[1]), new THREE.MeshStandardMaterial({}));
      const nose = new THREE.Mesh(new THREE.BoxGeometry(0.12, 0.2, size[1] * 0.5), new THREE.MeshBasicMaterial({ color: 0xefe6da }));
      nose.position.x = size[0] / 2; mesh.add(nose);
      T.sc.add(mesh); T.rob[r.id] = mesh;
    }
    mesh.position.set(r.x, 0.25, r.y); mesh.rotation.y = -(r.theta || 0);
    const c = new THREE.Color(COL[safe(r)]);
    mesh.material.color = c; mesh.material.emissive = c.clone().multiplyScalar(0.35);
  });
  const { a, e, d, c } = T;
  T.cam.position.set(c.x + d * Math.cos(e) * Math.sin(a), d * Math.sin(e), c.z + d * Math.cos(e) * Math.cos(a));
  T.cam.lookAt(c); T.r.render(T.sc, T.cam);
}

function setView(v) {
  view = v;
  document.querySelectorAll(".vw").forEach(b => b.classList.toggle("on", b.dataset.view === v));
  $("c2d").style.display = v === "2d" ? "block" : "none";
  $("c3d").style.display = v === "3d" ? "block" : "none";
  if (v === "3d" && S) { if (!T) init3d(); loop3d(); } else if (v === "2d" && S) draw2d();
}

async function loadScenarios() {
  const list = await api("/api/scenarios", "GET");
  if (!list) return setTimeout(loadScenarios, 3000);
  $("missions").innerHTML = list.map(s =>
    `<button class="mis" data-id="${s.id}">${s.name}<small>${s.description || ""}</small></button>`).join("");
}

// board-start (pure helpers, no DOM)
const fin = r => r.tasks_failed === 0 && r.tasks_completed === r.tasks_total;
const avg = a => a.length ? a.reduce((x, y) => x + y, 0) / a.length : null;
const sum = (a, k) => a.reduce((n, r) => n + (r[k] || 0), 0);

function boardStats(rows, limit) {
  const by = {};
  rows.forEach(r => (by[r.scenario] = by[r.scenario] || {})[r.strategy] = r);
  const pairs = Object.values(by).filter(p => p.stop_and_wait && p.arcnet);
  const cmp = pairs.filter(p => fin(p.stop_and_wait) && fin(p.arcnet));
  const gain = p => (p.stop_and_wait.completion_time - p.arcnet.completion_time) / p.stop_and_wait.completion_time * 100;
  const at = r => fin(r) ? r.completion_time : limit;
  const A = rows.filter(r => r.strategy === "arcnet"), B = rows.filter(r => r.strategy === "stop_and_wait");
  return {
    pairs, cmp, gain, n: pairs.length, A, B,
    mean: avg(cmp.map(gain)),
    bound: avg(pairs.map(p => (at(p.stop_and_wait) - at(p.arcnet)) / at(p.stop_and_wait) * 100)),
    doneA: A.filter(fin).length, doneB: B.filter(fin).length,
    colA: sum(A, "collisions"), colB: sum(B, "collisions"),
    worst: cmp.length ? cmp.reduce((w, p) => gain(p) < gain(w) ? p : w) : null
  };
}

function aggRows(st) {
  const cA = st.cmp.map(p => p.arcnet), cB = st.cmp.map(p => p.stop_and_wait);
  const per = (rs, f) => avg(rs.map(f));
  const gaps = rs => { const g = rs.map(r => r.min_gap_m).filter(v => v != null); return g.length ? Math.min(...g) : null; };
  return [
    ["Time to finish all tasks (s)", per(cB, r => r.completion_time), per(cA, r => r.completion_time), "low", "runs both finished"],
    ["Average delivery time (s)", per(cB, r => r.avg_task_s), per(cA, r => r.avg_task_s), "low", "runs both finished"],
    ["Throughput (tasks per minute)", per(cB, r => r.tasks_completed / r.completion_time * 60), per(cA, r => r.tasks_completed / r.completion_time * 60), "high", "runs both finished"],
    ["Standing time per robot with a job (s)", per(cB, r => r.wait_s / r.robots), per(cA, r => r.wait_s / r.robots), "low", "runs both finished"],
    ["Distance per delivered task (m)", per(cB, r => r.distance_m / r.tasks_completed), per(cA, r => r.distance_m / r.tasks_completed), "low", "runs both finished"],
    ["Stops", sum(st.B, "stops"), sum(st.A, "stops"), "low", "all runs"],
    ["Replans", sum(st.B, "replans"), sum(st.A, "replans"), "info", "all runs"],
    ["Deadlocks detected", sum(st.B, "deadlocks"), sum(st.A, "deadlocks"), "low", "all runs"],
    ["Deadlocks resolved", sum(st.B, "deadlocks_resolved"), sum(st.A, "deadlocks_resolved"), "high", "all runs"],
    ["Yield negotiations", sum(st.B, "yields"), sum(st.A, "yields"), "info", "all runs"],
    ["Near misses", sum(st.B, "near_misses"), sum(st.A, "near_misses"), "low", "all runs"],
    ["Collisions", st.colB, st.colA, "low", "all runs"],
    ["Closest approach (m)", gaps(st.B), gaps(st.A), "high", "all runs"]
  ];
}
// board-end

const f1 = v => v == null ? "-" : Number.isInteger(v) ? v : v.toFixed(1);
let lastRows = [], valTimer = null;

function renderBoard(rows, limit) {
  const st = boardStats(rows, limit), out = $("vout");
  if (!st.n) { out.innerHTML = '<p class="mute">No results yet.</p>'; return; }
  const met = st.mean !== null && st.mean >= 20;
  const head = st.mean === null ? "No run finished under both strategies, so no time comparison is possible."
    : (met ? "Target met: " : "Below the 20 percent target: ") + `ARCNET was ${st.mean.toFixed(1)}% faster on average across ${st.cmp.length} runs both strategies finished.`;
  const worst = st.worst && st.gain(st.worst) < 0 ? st.gain(st.worst) : null;
  const kpi = (v, l, c) => `<div class="kpi ${c || ""}"><b>${v}</b><span>${l}</span></div>`;

  const ids = [...new Set(rows.map(r => r.scenario_id))];
  const val = r => fin(r) ? r.completion_time : limit;
  const bars = ids.map(id => {
    const b = st.B.filter(r => r.scenario_id === id), a = st.A.filter(r => r.scenario_id === id);
    const mb = avg(b.map(val)), ma = avg(a.map(val)), dnf = b.filter(r => !fin(r)).length, dnfA = a.filter(r => !fin(r)).length;
    const w = v => Math.min(100, v / limit * 100).toFixed(1) + "%";
    return `<div class="br"><span>${id.replace(/_/g, " ")}</span><div>
      <div class="tr"><i class="b ${dnf ? "dnf" : ""}" style="width:${w(mb)}"></i><em>Stop and wait ${mb.toFixed(1)} s${dnf ? ", " + dnf + " of " + b.length + " did not finish" : ""}</em></div>
      <div class="tr"><i class="a" style="width:${w(ma)}"></i><em>ARCNET ${ma.toFixed(1)} s${dnfA ? ", " + dnfA + " did not finish" : ""}</em></div></div></div>`;
  }).join("");

  const agg = aggRows(st).map(([name, b, a, dir, basis]) => {
    let d = "-", cls = "";
    if (typeof b === "number" && typeof a === "number" && b !== 0) {
      const pc = (a - b) / b * 100;
      d = (pc > 0 ? "+" : "") + pc.toFixed(0) + "%";
      if (dir !== "info" && Math.abs(pc) >= 1) cls = ((dir === "low") === (pc < 0)) ? "good" : "worse";
    }
    return `<tr><td>${name}</td><td>${f1(b)}</td><td>${f1(a)}</td><td class="${cls}">${d}</td><td class="mute">${basis}</td></tr>`;
  }).join("");

  const detail = st.pairs.map(p => {
    const b = p.stop_and_wait, a = p.arcnet, ok = fin(a) && fin(b), g = ok ? st.gain(p) : null;
    return `<tr><td>${a.scenario}</td><td>${fin(b) ? b.completion_time + " s" : "did not finish"}</td><td>${fin(a) ? a.completion_time + " s" : "did not finish"}</td>
      <td class="${g === null ? "" : g >= 20 ? "good" : g < 0 ? "worse" : ""}">${g === null ? "not comparable" : g.toFixed(1) + "%"}</td>
      <td>${b.wait_s} / ${a.wait_s}</td><td>${b.distance_m} / ${a.distance_m}</td><td>${b.collisions} / ${a.collisions}</td></tr>`;
  }).join("");

  out.innerHTML = `
    <div class="verdict ${met ? "ok" : "warn"}"><b>${head}</b>
      <span class="mute">ARCNET finished ${st.doneA} of ${st.n} runs and stop and wait finished ${st.doneB}. ARCNET collisions: ${st.colA}.</span></div>
    <div class="kpis">
      ${kpi(st.doneA + " / " + st.n, "Runs finished by ARCNET (stop and wait: " + st.doneB + ")", st.doneA > st.doneB ? "good" : "")}
      ${kpi(st.mean === null ? "-" : st.mean.toFixed(1) + "%", "Time saved, runs both finished (" + st.cmp.length + ")", met ? "good" : "")}
      ${kpi(st.bound === null ? "-" : st.bound.toFixed(1) + "%", "Time saved if unfinished runs count as " + limit + " s (lower bound)")}
      ${kpi(st.colA, "ARCNET collisions (stop and wait: " + st.colB + ")", st.colA === 0 ? "good" : "bad")}
      ${kpi(worst === null ? "None" : worst.toFixed(1) + "%", worst === null ? "Scenarios where ARCNET was slower" : "Worst case: " + st.worst.arcnet.scenario, worst === null ? "good" : "bad")}
    </div>
    <div class="panel"><h2>Time to finish by scenario</h2>
      <div class="key"><span>Shorter is better</span><span>Hatched: some runs did not finish, counted at ${limit} s</span></div>${bars}</div>
    <div class="panel"><h2>All measured metrics</h2>
      <table><thead><tr><th>Metric</th><th>Stop and wait</th><th>ARCNET</th><th>Change</th><th>Basis</th></tr></thead><tbody>${agg}</tbody></table></div>
    <div class="panel"><details><summary>Every run (${st.n} scenario layouts)</summary>
      <table><thead><tr><th>Scenario</th><th>Stop and wait</th><th>ARCNET</th><th>Change</th><th>Standing s (base / ARCNET)</th><th>Distance m</th><th>Collisions</th></tr></thead><tbody>${detail}</tbody></table></details></div>`;
}

async function pollVal() {
  const v = await api("/api/validation/status", "GET");
  if (!v) return;
  const running = v.state === "running";
  $("vbtn").disabled = running;
  $("vprog").hidden = !running;
  if (running) {
    $("vbar").style.width = (v.done / v.total * 100) + "%";
    $("vtext").textContent = "Running " + v.done + " of " + v.total + " runs";
    clearTimeout(valTimer); valTimer = setTimeout(pollVal, 800);
  } else if (v.state === "error") $("vout").textContent = "Validation failed: " + v.error;
  else if (v.state === "done") { lastRows = v.rows; renderBoard(v.rows, v.limit); $("vcsv").hidden = false; }
}

async function runValidation() { await api("/api/validation/run"); pollVal(); }

function exportCsv() {
  if (!lastRows.length) return;
  const keys = Object.keys(lastRows[0]);
  const csv = [keys.join(",")].concat(lastRows.map(r => keys.map(k => JSON.stringify(r[k] ?? "")).join(","))).join("\n");
  const a = document.createElement("a");
  a.href = URL.createObjectURL(new Blob([csv], { type: "text/csv" }));
  a.download = "arcnet_validation.csv"; a.click();
}

document.addEventListener("click", e => {
  const t = e.target.closest("[data-tab],[data-tool],[data-view],[data-dock],[data-filter],.mis,.rb");
  if (!t) return;
  if (t.dataset.dock) {
    document.querySelectorAll(".dt").forEach(b => b.classList.toggle("on", b === t));
    document.querySelectorAll(".dp").forEach(p => p.classList.toggle("on", p.id === "dock-" + t.dataset.dock));
    return;
  }
  if (t.dataset.filter) {
    logFilter = t.dataset.filter;
    document.querySelectorAll(".fc").forEach(b => b.classList.toggle("on", b === t));
    if (S) renderEvents();
    return;
  }
  if (t.dataset.tab) {
    document.querySelectorAll(".tab").forEach(b => b.classList.toggle("on", b === t));
    document.querySelectorAll(".page").forEach(p => p.classList.toggle("on", p.id === t.dataset.tab));
    if (t.dataset.tab === "monitor" && S) setTimeout(() => setView(view), 0);
  } else if (t.dataset.tool) {
    tool = t.dataset.tool; picked = null; $("hint").textContent = HINT[tool];
    document.querySelectorAll(".tool").forEach(b => b.classList.toggle("on", b === t));
  } else if (t.dataset.view) setView(t.dataset.view);
  else if (t.classList.contains("mis")) api("/api/scenario/" + t.dataset.id);
  else if (t.classList.contains("rb") && tool === "dispatch") { picked = t.dataset.id; renderFleet(); }
});
document.addEventListener("change", e => {
  const id = e.target.dataset.task;
  if (id && e.target.value) api("/api/task/" + id + "/assign", "POST", { robot_id: e.target.value });
});
$("c2d").addEventListener("click", onStageClick);
$("bStart").onclick = () => api("/api/run/start");
$("bStop").onclick = () => api("/api/run/stop");
$("bReset").onclick = async () => { await api("/api/run/reset"); $("banner").hidden = true; };
$("vbtn").onclick = runValidation;
$("vcsv").onclick = exportCsv;
$("bNew").onclick = () => { $("taskForm").hidden = !$("taskForm").hidden; };
$("bTask").onclick = () => {
  const v = id => Number($(id).value);
  if (!S) return;
  let n = S.tasks.length + 1, id;
  do { id = "T" + String(n++).padStart(3, "0"); } while (S.tasks.some(t => t.id === id));
  api("/api/task", "POST", { id, start: [v("sx"), v("sy")], destination: [v("dx"), v("dy")], assigned_robot: $("tRobot").value || null });
};
addEventListener("resize", () => {
  if (T) { const el = $("c3d"); T.r.setSize(el.clientWidth, el.clientHeight); T.cam.aspect = el.clientWidth / el.clientHeight; T.cam.updateProjectionMatrix(); }
  if (S && view === "2d") draw2d();
});

$("hint").textContent = HINT.view;
connect();
loadScenarios();
pollVal();