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

  renderFleet(); renderBadges(total, done, m); renderEvents(); renderTasks();
  document.querySelectorAll(".mis").forEach(b => b.classList.toggle("on", b.dataset.id === S.scenario));

  if (S.running) $("banner").hidden = true;
  if (wasRunning && !S.running && total > 0 && done === total) showBanner(done);
  wasRunning = S.running;
  if (view === "2d") draw2d();
}

function renderFleet() {
  $("fleet").innerHTML = S.robots.map(r => {
    const st = safe(r), bat = Math.max(0, Math.min(100, r.battery ?? 0));
    return `<div class="rb ${picked === r.id ? "pick" : ""} ${st === "failed" ? "dead" : ""}" data-id="${r.id}">
      <div><strong>${r.id}</strong><span class="chip ${st}">${st.replace("_", " ")}</span></div>
      <small>${r.task_id ? "Task " + r.task_id : "Idle"}${r.reason ? " - " + r.reason : ""}</small>
      <div class="bar"><i style="width:${bat}%"></i></div></div>`;
  }).join("") || '<p class="mute">No robots loaded.</p>';
}

function renderBadges(total, done, m) {
  const all = total > 0 && done === total;
  const clean = (m.near_misses ?? 0) === 0 && (m.deadlocks ?? 0) === 0 && (m.collisions ?? 0) === 0;
  const rows = [
    ["Zero collisions", "No robot touches another", (m.collisions ?? 0) > 0 ? "lost" : all ? "earned" : "holding"],
    ["All tasks delivered", done + " of " + total + " done", all ? "earned" : "holding"],
    ["Smooth run", "No near misses or deadlocks", !clean ? "lost" : all ? "earned" : "holding"]
  ];
  $("badges").innerHTML = rows.map(([a, b, s]) =>
    `<div class="bd ${s}"><div><b>${a}</b><br><span>${b}</span></div><span>${s}</span></div>`).join("");
}

function renderEvents() {
  $("events").innerHTML = [...S.events].reverse().slice(0, 40).map(e =>
    `<div class="ev ${e.kind || ""}"><time>${e.time}s</time>${e.message}</div>`).join("")
    || '<p class="mute">Nothing yet. Press Start.</p>';
}

function renderTasks() {
  const sig = JSON.stringify(S.tasks) + S.robots.map(r => r.id).join();
  if (sig === taskSig) return;
  taskSig = sig;
  const opts = sel => '<option value="">unassigned</option>' +
    S.robots.map(r => `<option ${r.id === sel ? "selected" : ""}>${r.id}</option>`).join("");
  $("taskRows").innerHTML = S.tasks.map(t =>
    `<tr><td>${t.id}</td><td>(${t.start.join(", ")})</td><td>(${t.destination.join(", ")})</td>
     <td><select data-task="${t.id}">${opts(t.assigned_robot)}</select></td>
     <td><span class="chip ${t.status}">${t.status}</span></td></tr>`).join("");
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

async function runValidation() {
  const btn = $("vbtn"), out = $("vout");
  btn.disabled = true; out.textContent = "Running both strategies on every scenario...";
  const res = await api("/api/validation/run");
  btn.disabled = false;
  if (!res) { out.textContent = "The validation endpoint is not available on the backend yet."; return; }
  const rows = res.rows || res, by = {};
  rows.forEach(r => (by[r.scenario] = by[r.scenario] || {})[r.strategy] = r);
  const fin = r => r && r.tasks_failed === 0 && r.tasks_completed === r.tasks_total;
  const gains = [];
  const body = Object.entries(by).map(([name, p]) => {
    const b = p.stop_and_wait, a = p.arcnet, ok = fin(a) && fin(b);
    const gain = ok ? (b.completion_time - a.completion_time) / b.completion_time * 100 : null;
    if (ok) gains.push(gain);
    return `<tr><td>${name}</td><td>${fin(b) ? b.completion_time.toFixed(1) + " s" : "did not finish"}</td>
      <td>${fin(a) ? a.completion_time.toFixed(1) + " s" : "did not finish"}</td>
      <td class="${gain === null ? "" : gain >= 20 ? "win" : gain < 0 ? "bad" : ""}">${gain === null ? "not comparable" : gain.toFixed(1) + "%"}</td>
      <td>${b ? b.collisions : "-"} / ${a ? a.collisions : "-"}</td></tr>`;
  }).join("");
  const mean = gains.length ? gains.reduce((x, y) => x + y, 0) / gains.length : null;
  const arcCol = Object.values(by).reduce((n, p) => n + (p.arcnet ? p.arcnet.collisions : 0), 0);
  out.innerHTML = `<p><strong>${mean === null ? "No comparable scenarios yet." :
    "Mean improvement " + mean.toFixed(1) + "% across " + gains.length + " of " + Object.keys(by).length + " scenarios."}</strong>
    ARCNET collisions: ${arcCol}.</p>
    <table><thead><tr><th>Scenario</th><th>Stop and wait</th><th>ARCNET</th><th>Change</th><th>Collisions (base / ARCNET)</th></tr></thead><tbody>${body}</tbody></table>`;
}

document.addEventListener("click", e => {
  const t = e.target.closest("[data-tab],[data-tool],[data-view],.mis,.rb");
  if (!t) return;
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