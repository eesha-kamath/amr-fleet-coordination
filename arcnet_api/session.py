# Adapter between the API and CoordinationRuntime. The backend owns all state.
import csv
from math import hypot
import threading
import time
import traceback

from arcnet_core.robot import RobotSpec
from arcnet_core.grid import Grid
from arcnet_core.obstacle import Obstacle
from arcnet_core.task import Task, TaskStatus
from arcnet_core.scenarios import ScenarioLibrary
from arcnet_core.runtime import CoordinationRuntime, RuntimeConfig

DT = 0.1
MAX_SIM_TIME = 300.0
GRID = (20, 12, 1.0)

# Faults that belong to a scenario. Used by the live view and by validation.
# kill: (sim seconds, robot id).  wifi: grid cells where a robot's broadcast is lost.
SCENARIO_FAULTS = {
    "robot_failure": {"kill": [(3.0, "R2")]},
    "wifi_dead_zone": {"wifi": [(4, 5), (5, 5), (6, 5)]},
}
VARIANTS = ("base", "mirror", "shift")


def make_spec(robot_id):
    return RobotSpec(robot_id, "rectangle", 0.9, 0.64, 0.19, 70, 0.5, 0.08, 1.0, 0.5, 0.8, 0.1)


def default_scenario():
    return {
        "id": "default",
        "name": "Default fleet",
        "description": "Three robots on a shared floor.",
        "robots": [
            {"id": "R1", "x": 2.5, "y": 3.5},
            {"id": "R2", "x": 17.5, "y": 3.5},
            {"id": "R3", "x": 10.5, "y": 8.5},
        ],
        "tasks": [
            Task("T001", (2, 3), (17, 3), "R1"),
            Task("T002", (17, 3), (2, 3), "R2"),
            Task("T003", (10, 8), (10, 2), "R3"),
        ],
        "obstacles": [],
    }


def build_runtime(data, strategy="arcnet"):
    grid = Grid(*GRID)
    specs = {r["id"]: make_spec(r["id"]) for r in data["robots"]}
    rt = CoordinationRuntime(grid, specs, list(data.get("obstacles", [])), RuntimeConfig(strategy=strategy))
    for r in data["robots"]:
        rt.add_robot(r["id"], r["x"], r["y"])
    for t in data["tasks"]:
        rt.add_task(Task(t.id, t.start, t.destination, t.assigned_robot))
    return rt


def val(x):
    return getattr(x, "value", x)


def settled(rt):
    """True when every task is either completed or failed."""
    tasks = list(rt.tasks.values())
    return bool(tasks) and all(t.completed or val(t.status) == "failed" for t in tasks)


# Fault helpers. The runtime's own methods are used when present.
def kill_robot(rt, robot_id):
    fn = getattr(rt, "kill_robot", None)
    if fn:
        fn(robot_id)
        return
    rt.sim.kill_robot(robot_id)
    agent = rt.agents.get(robot_id)
    if agent:
        agent.mode = "failed"


def block_cell(rt, x, y):
    fn = getattr(rt, "block_cell", None)
    if fn:
        fn(x, y)
        return
    cell = Obstacle(x, y, True)
    for lst in {id(rt.obstacles): rt.obstacles, id(rt.sim.obstacles): rt.sim.obstacles}.values():
        if cell not in lst:
            lst.append(cell)


class Session:
    def __init__(self):
        self.lock = threading.RLock()
        self.library = ScenarioLibrary()
        self.strategy = "arcnet"
        self.running = False
        self.scenario_id = "default"
        self.events = []
        self.wifi = []
        self.pending_kills = []
        self.rt = None
        self.load("default")
        threading.Thread(target=self._loop, daemon=True).start()

    # scenarios and run control
    def scenarios(self):
        out = [{"id": "default", "name": "Default fleet", "description": "Three robots on a shared floor."}]
        for sid, build in self.library.all().items():
            d = build()
            out.append({"id": sid, "name": d.get("name", sid), "description": d.get("description", "")})
        return out

    def load(self, scenario_id):
        data = default_scenario() if scenario_id == "default" else self.library.get(scenario_id)
        faults = SCENARIO_FAULTS.get(scenario_id, {})
        with self.lock:
            self.running = False
            self.scenario_id = scenario_id
            self.events = []
            self.rt = build_runtime(data, self.strategy)
            self.wifi = [{"x": x, "y": y} for x, y in faults.get("wifi", [])]
            self.pending_kills = list(faults.get("kill", []))
            self._apply_wifi()
            self.log("SCENARIO", "SYSTEM", f"Loaded {scenario_id}")

    def reset(self):
        self.load(self.scenario_id)

    def start(self):
        with self.lock:
            if settled(self.rt):
                self.log("SYSTEM", "SYSTEM", "Run already finished. Reset to run again.")
                return
            self.running = True
            self.log("SYSTEM", "SYSTEM", "Simulation started")

    def stop(self):
        with self.lock:
            self.running = False
            self.log("SYSTEM", "SYSTEM", "Simulation stopped")

    def set_strategy(self, name):
        if name not in ("arcnet", "stop_and_wait"):
            raise ValueError("Unknown strategy")
        self.strategy = name
        self.reset()

    def _apply_wifi(self):
        fn = getattr(self.rt, "set_wifi_zones", None)
        if fn:
            fn({(z["x"], z["y"]) for z in self.wifi})

    def _loop(self):
        while True:
            if not self.running:
                time.sleep(0.05)
                continue
            t0 = time.time()
            with self.lock:
                if self.running:
                    try:
                        while self.pending_kills and self.rt.time >= self.pending_kills[0][0]:
                            _, rid = self.pending_kills.pop(0)
                            if rid in self.rt.sim.robots:
                                kill_robot(self.rt, rid)
                        self.rt.step()
                        self._check_done()
                    except Exception as e:
                        self.running = False
                        traceback.print_exc()
                        self.log("SYSTEM", "SYSTEM", f"Runtime error: {e}")
            time.sleep(max(0.0, DT - (time.time() - t0)))

    def _check_done(self):
        if settled(self.rt):
            self.running = False
            done = all(t.completed for t in self.rt.tasks.values())
            self.log("SYSTEM", "SYSTEM", "All tasks completed" if done else "Run ended with failed tasks")
        elif self.rt.time >= MAX_SIM_TIME:
            self.running = False
            self.log("SYSTEM", "SYSTEM", "Run timed out before all tasks finished")

    # tasks and faults
    def add_task(self, data):
        with self.lock:
            tid = data["id"]
            if tid in self.rt.tasks:
                raise ValueError("Task id already exists")
            start, dest = tuple(data["start"]), tuple(data["destination"])
            g = self.rt.grid
            for c in (start, dest):
                if not (0 <= c[0] < g.width and 0 <= c[1] < g.height):
                    raise ValueError("Cell outside the grid")
            robot = data.get("assigned_robot")
            if robot and robot not in self.rt.sim.robots:
                raise ValueError("Robot not found")
            self.rt.add_task(Task(tid, start, dest, robot))
            self.log("TASK", "SYSTEM", f"Added {tid}" + (f" for {robot}" if robot else ""))
            return {"ok": True, "task": tid}

    def assign_task(self, task_id, robot_id):
        with self.lock:
            task = self.rt.tasks.get(task_id)
            robot = self.rt.sim.robots.get(robot_id)
            if not task:
                raise ValueError("Task not found")
            if not robot or not robot.active:
                raise ValueError("Robot not available")
            task.assigned_robot = robot_id
            task.status = TaskStatus.PENDING
            task.completed = False
            self.log("TASK", "SYSTEM", f"{task_id} assigned to {robot_id}")
            return {"ok": True}

    def add_fault(self, data):
        kind = data.get("kind")
        with self.lock:
            before = len(self.rt.events)
            if kind == "robot_failure":
                rid = data.get("target")
                if rid not in self.rt.sim.robots:
                    raise ValueError("Robot not found")
                kill_robot(self.rt, rid)
                note = f"{rid} failed"
            elif kind == "blocked_cell":
                x, y = int(data["x"]), int(data["y"])
                block_cell(self.rt, x, y)
                note = f"Cell ({x},{y}) blocked"
            elif kind == "wifi_dead_zone":
                zone = {"x": int(data["x"]), "y": int(data["y"])}
                if zone not in self.wifi:
                    self.wifi.append(zone)
                self._apply_wifi()
                live = hasattr(self.rt, "set_wifi_zones")
                note = f"Dead zone at ({zone['x']},{zone['y']})" + ("" if live else ", recorded only")
                before = -1
            else:
                raise ValueError("Unknown fault kind")
            # the runtime logs its own fault events; add ours only if it did not
            if len(self.rt.events) == before or before == -1:
                self.log("FAULT", "SYSTEM", note)
            return {"ok": True}

    def log(self, kind, source, message):
        t = round(self.rt.time, 2) if self.rt else 0.0
        self.events.append({"time": t, "kind": kind, "source": source, "message": message})
        self.events = self.events[-200:]

    # state for the dashboard
    def state(self):
        with self.lock:
            rt = self.rt
            robots = []
            for r in rt.sim.robots.values():
                a = rt.agents.get(r.id)
                failed = (not r.active) or (a is not None and getattr(a, "mode", "") == "failed")
                robots.append({
                    "id": r.id, "x": r.x, "y": r.y, "theta": r.theta, "vx": r.vx, "vy": r.vy,
                    "battery": r.battery, "active": not failed,
                    "task_id": getattr(a, "task_id", None),
                    "mode": getattr(a, "mode", "idle"),
                    "safety": "failed" if failed else (val(getattr(a, "safety", None)) or "normal"),
                    "reason": getattr(a, "reason", ""),
                    "silent": bool(getattr(a, "silent", False)),
                    "waiting_for": getattr(a, "waiting_for", None),
                    "wait_time": round(getattr(a, "wait_time", 0.0), 1),
                    "priority_tier": rt.arbiter.aging_tier(getattr(a, "wait_time", 0.0)) if a else 0,
                    "logical_clock": getattr(a, "req_clock", 0),
                    "plan": [list(c) for c in (getattr(a, "plan", None) or [])],
                })
            tasks = [{
                "id": t.id, "start": list(t.start), "destination": list(t.destination),
                "assigned_robot": t.assigned_robot, "completed": t.completed, "status": val(t.status),
                "phase": rt.phase.get(t.id),
            } for t in rt.tasks.values()]
            m = rt.metrics
            spec = next(iter(rt.sim.robot_specs.values()), None)
            events = sorted(self.events + list(rt.events[-150:]), key=lambda e: e.get("time", 0))[-100:]
            return {
                "time": round(rt.time, 2),
                "running": self.running,
                "scenario": self.scenario_id,
                "strategy": self.strategy,
                "grid": {"width": rt.grid.width, "height": rt.grid.height, "cell_size": rt.grid.cell_size},
                "robot_size": [spec.length, spec.width] if spec else [0.9, 0.64],
                "robots": robots,
                "tasks": tasks,
                "obstacles": [{"x": o.x, "y": o.y, "temporary": o.temporary} for o in rt.sim.obstacles],
                "wifi": list(self.wifi),
                "events": events,
                "metrics": {
                    "collisions": m.collisions, "near_misses": m.near_misses, "stops": m.stops,
                    "replans": m.replans, "deadlocks": m.deadlocks_detected,
                    "deadlocks_resolved": m.deadlocks_resolved,
                    "tasks_completed": m.tasks_completed, "tasks_failed": m.tasks_failed,
                },
            }


# Validation. Each scenario runs in three layouts (base, mirrored, shifted) and two strategies.
def make_transform(variant, width):
    """Returns (cell transform, position transform) for a layout variant."""
    if variant == "mirror":
        return (lambda x, y: (width - 1 - x, y)), (lambda x, y: (width - x, y))
    if variant == "shift":
        return (lambda x, y: (x + 5, y + 1)), (lambda x, y: (x + 5, y + 1))
    return (lambda x, y: (x, y)), (lambda x, y: (x, y))


def variant_data(data, variant):
    tc, tp = make_transform(variant, GRID[0])
    robots = []
    for r in data["robots"]:
        x, y = tp(r["x"], r["y"])
        robots.append({**r, "x": x, "y": y})
    tasks = [Task(t.id, tc(*t.start), tc(*t.destination), t.assigned_robot) for t in data["tasks"]]
    obstacles = [Obstacle(*tc(o.x, o.y), o.temporary) for o in data.get("obstacles", [])]
    return {**data, "robots": robots, "tasks": tasks, "obstacles": obstacles}


def run_instrumented(rt, limit, kills):
    """Steps the runtime and measures extra metrics from ground truth."""
    counts = {}
    orig = rt._log

    def counting(kind, source, message, *a, **k):
        last = rt.events[-1] if rt.events else None
        orig(kind, source, message, *a, **k)
        if rt.events and rt.events[-1] is not last:
            counts[kind] = counts.get(kind, 0) + 1

    rt._log = counting
    dt = rt.sim.config.dt
    prev = {r.id: (r.x, r.y) for r in rt.sim.robots.values()}
    dist = wait = 0.0
    done_at = {}

    while rt.time < limit and not rt.finished():
        while kills and rt.time >= kills[0][0]:
            kill_robot(rt, kills.pop(0)[1])
        rt.step()
        for r in rt.sim.robots.values():
            moved = hypot(r.x - prev[r.id][0], r.y - prev[r.id][1])
            prev[r.id] = (r.x, r.y)
            dist += moved
            agent = rt.agents[r.id]
            if agent.mode != "failed" and rt._task_for(agent) and moved < 0.005:
                wait += dt
        for t in rt.tasks.values():
            if t.completed and t.id not in done_at:
                done_at[t.id] = rt.time

    rt._log = orig
    m = rt.metrics
    m.completion_time = rt._finished_at if rt._finished_at is not None else rt.time
    gap = rt.judge.min_gap
    return {
        "completion_time": round(m.completion_time, 1),
        "distance_m": round(dist, 1),
        "wait_s": round(wait, 1),
        "avg_task_s": round(sum(done_at.values()) / len(done_at), 1) if done_at else None,
        "min_gap_m": None if gap == float("inf") else round(gap, 2),
        "yields": counts.get("YIELD", 0),
        "reroutes": counts.get("REROUTE", 0),
        "robots": len(rt.agents),
    }


def run_scenario(sid, variant, strategy, limit):
    tc, _ = make_transform(variant, GRID[0])
    faults = SCENARIO_FAULTS.get(sid, {})
    rt = build_runtime(variant_data(ScenarioLibrary().get(sid), variant), strategy)
    zones = {tc(x, y) for x, y in faults.get("wifi", [])}
    if zones and hasattr(rt, "set_wifi_zones"):
        rt.set_wifi_zones(zones)
    extra = run_instrumented(rt, limit, list(faults.get("kill", [])))
    return rt, rt.metrics, extra


# Validation runs in a background thread so the UI can show progress.
VAL = {"state": "idle", "done": 0, "total": 0, "limit": 120.0, "rows": [], "error": None}
_val_lock = threading.Lock()


def validation_status():
    return dict(VAL)


def start_validation(limit=120.0):
    with _val_lock:
        if VAL["state"] == "running":
            raise ValueError("Validation is already running")
        total = len(ScenarioLibrary().all()) * len(VARIANTS) * 2
        VAL.update(state="running", done=0, total=total, limit=limit, rows=[], error=None)
    threading.Thread(target=_validation_worker, args=(limit,), daemon=True).start()


def _validation_worker(limit, csv_path="validation_results.csv"):
    try:
        rows = []
        for sid in ScenarioLibrary().all():
            for n, variant in enumerate(VARIANTS, start=1):
                for strategy in ("stop_and_wait", "arcnet"):
                    rt, m, extra = run_scenario(sid, variant, strategy, limit)
                    rows.append({
                        "scenario": f"{sid} ({variant})", "scenario_id": sid, "variant": variant,
                        "run": n, "strategy": strategy,
                        "collisions": m.collisions, "near_misses": m.near_misses, "stops": m.stops,
                        "replans": m.replans, "deadlocks": m.deadlocks_detected,
                        "deadlocks_resolved": m.deadlocks_resolved, "tasks_total": len(rt.tasks),
                        "tasks_completed": m.tasks_completed, "tasks_failed": m.tasks_failed,
                        **extra,
                    })
                    VAL["done"] += 1
        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
        VAL["rows"] = rows
        VAL["state"] = "done"
    except Exception as e:
        traceback.print_exc()
        VAL.update(state="error", error=str(e))