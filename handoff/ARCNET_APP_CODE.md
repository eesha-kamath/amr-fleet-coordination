# ARCNET APPLICATION CODE SNAPSHOT


============================================================
FILE: arcnet_api\main.py
============================================================

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from .session import SimulationSession
import asyncio

app = FastAPI(title="ARCNET API")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

session = SimulationSession()


@app.get("/api/state")
def state():
    return session.state()


@app.get("/api/scenarios")
def scenarios():
    return session.scenarios()


@app.post("/api/run/start")
def start():
    session.start()
    return {"ok": True, "running": True}


@app.post("/api/run/stop")
def stop():
    session.stop()
    return {"ok": True, "running": False}


@app.post("/api/run/reset")
def reset():
    session.reset()
    return {"ok": True}


@app.post("/api/scenario/{scenario_id}")
def scenario(scenario_id: str):
    session.load_scenario(scenario_id)
    return {"ok": True, "scenario": scenario_id}


@app.post("/api/task")
def task(data: dict):
    return session.add_task(data)


@app.post("/api/task/{task_id}/assign")
def assign(task_id: str, data: dict):
    return session.assign_task(
        task_id,
        data.get("robot_id")
    )


@app.post("/api/fault")
def fault(data: dict):
    return session.add_fault(data)


@app.websocket("/ws")
async def websocket(websocket: WebSocket):
    await websocket.accept()

    try:
        while True:
            await websocket.send_json(session.state())
            await asyncio.sleep(0.1)
    except WebSocketDisconnect:
        pass


============================================================
FILE: arcnet_api\models.py
============================================================

from pydantic import BaseModel


class TaskInput(BaseModel):
    id: str
    start: tuple[int, int]
    destination: tuple[int, int]
    assigned_robot: str | None = None


class AssignInput(BaseModel):
    robot_id: str


class FaultInput(BaseModel):
    kind: str
    target: str | None = None
    x: int | None = None
    y: int | None = None


class ScenarioInput(BaseModel):
    scenario_id: str


============================================================
FILE: arcnet_api\session.py
============================================================

import asyncio
from math import sqrt

from arcnet_core.robot import RobotSpec
from arcnet_core.grid import Grid
from arcnet_core.obstacle import Obstacle
from arcnet_core.config import SafetyConfig
from arcnet_core.engine import ArcnetEngine
from arcnet_core.simulator import WarehouseSimulator
from arcnet_core.task import Task, TaskStatus
from arcnet_core.motion import MotionCommand
from arcnet_core.scenarios import ScenarioLibrary
from arcnet_core.faults import FaultManager


class SimulationSession:
    def __init__(self):
        self.scenario_id = "default"
        self.running = False
        self.loop_task = None
        self.events = []
        self.faults = FaultManager()
        self.library = ScenarioLibrary()

        self.grid = Grid(20, 12, 1.0)

        self.specs = {
            "R1": self._spec("R1"),
            "R2": self._spec("R2"),
            "R3": self._spec("R3")
        }

        self.paths = {}

        self.reset()

    def _spec(self, robot_id):
        return RobotSpec(
            robot_id,
            "rectangle",
            0.9,
            0.64,
            0.19,
            70,
            0.5,
            0.08,
            1.0,
            0.5,
            0.8,
            0.1
        )

    def _create(self, obstacles=None):
        obstacles = obstacles or []

        self.sim = WarehouseSimulator(
            self.grid,
            obstacles,
            self.specs
        )

        self.engine = ArcnetEngine(
            self.specs,
            self.grid,
            obstacles,
            SafetyConfig()
        )

    def reset(self):
        self.running = False
        self.events.clear()
        self.faults.clear()
        self.paths.clear()

        self._create()

        self.sim.add_robot("R1", 2.5, 3.5)
        self.sim.add_robot("R2", 17.5, 3.5)
        self.sim.add_robot("R3", 10.5, 8.5)

        self.sim.add_task(
            Task("T001", (2, 3), (17, 3), "R1")
        )

        self.sim.add_task(
            Task("T002", (17, 3), (2, 3), "R2")
        )

        self.sim.add_task(
            Task("T003", (10, 8), (10, 2), "R3")
        )

        self.log("SYSTEM", "Simulation reset")

    def scenarios(self):
        result = []

        all_scenarios = self.library.all()

        for scenario_id, builder in all_scenarios.items():
            try:
                data = builder()

                result.append({
                    "id": scenario_id,
                    "name": data.get(
                        "name",
                        scenario_id
                    ),
                    "description": data.get(
                        "description",
                        ""
                    )
                })
            except Exception:
                result.append({
                    "id": scenario_id,
                    "name": scenario_id,
                    "description": ""
                })

        return result

    def load_scenario(self, scenario_id):
        self.running = False
        self.scenario_id = scenario_id
        self.events.clear()
        self.faults.clear()
        self.paths.clear()

        data = self.library.get(scenario_id)

        if not data:
            raise ValueError(
                f"Unknown scenario: {scenario_id}"
            )

        obstacles = list(
            data.get("obstacles", [])
        )

        self._create(obstacles)

        for robot in data.get("robots", []):
            self.sim.add_robot(
                robot["id"],
                robot["x"],
                robot["y"],
                robot.get("theta", 0.0),
                robot.get("battery", 100.0)
            )

        for task in data.get("tasks", []):
            if isinstance(task, Task):
                self.sim.add_task(task)
            else:
                self.sim.add_task(
                    Task(
                        task["id"],
                        tuple(task["start"]),
                        tuple(task["destination"]),
                        task.get("assigned_robot")
                    )
                )

        self.log(
            "SCENARIO",
            f"Loaded {scenario_id}"
        )

    def start(self):
        if self.running:
            return

        self.running = True

        try:
            loop = asyncio.get_running_loop()

            if (
                self.loop_task is None
                or self.loop_task.done()
            ):
                self.loop_task = loop.create_task(
                    self._run_loop()
                )

        except RuntimeError:
            pass

        self.log(
            "SYSTEM",
            "Simulation started"
        )

    def stop(self):
        self.running = False

        for robot in self.sim.robots.values():
            robot.vx = 0.0
            robot.vy = 0.0

        self.log(
            "SYSTEM",
            "Simulation stopped"
        )

    async def _run_loop(self):
        while self.running:
            self.step()
            await asyncio.sleep(0.1)

    def step(self):
        if not self.running:
            return

        self._update_peers()

        commands = {}

        for robot_id, robot in self.sim.robots.items():

            if not robot.active:
                continue

            task = self._get_task(robot_id)

            if not task:
                commands[robot_id] = MotionCommand(
                    0.0,
                    0.0,
                    0.0,
                    True
                )
                continue

            if task.status == TaskStatus.PENDING:
                task.activate()

            goal = task.destination

            current = self.grid.to_cell(
                robot.x,
                robot.y
            )

            if current == goal:
                commands[robot_id] = MotionCommand(
                    0.0,
                    0.0,
                    0.0,
                    True
                )
                continue

            path = self.paths.get(robot_id)

            if (
                not path
                or current not in path
            ):
                path = self.engine.plan_robot(
                    robot_id,
                    current,
                    goal
                )

                self.paths[robot_id] = path

            if not path:
                task.fail()

                self.log(
                    robot_id,
                    f"Task {task.id}: no path"
                )

                commands[robot_id] = MotionCommand(
                    0.0,
                    0.0,
                    0.0,
                    True
                )

                continue

            try:
                index = path.index(current)
            except ValueError:
                index = 0

            if index + 1 >= len(path):
                target = goal
            else:
                target = path[index + 1]

            tx, ty = self.grid.to_position(
                target[0],
                target[1]
            )

            dx = tx - robot.x
            dy = ty - robot.y

            d = sqrt(dx * dx + dy * dy)

            if d < 0.08:
                robot.x = tx
                robot.y = ty

                commands[robot_id] = MotionCommand(
                    0.0,
                    0.0,
                    0.0,
                    True
                )

                continue

            vx = dx / d
            vy = dy / d

            peers = self._get_peers(
                robot_id
            )

            for peer in peers:
                self.engine.set_peer_state(
                    peer["id"],
                    peer["x"],
                    peer["y"],
                    peer["vx"],
                    peer["vy"]
                )

            nearest = self._nearest_distance(
                robot,
                peers
            )

            level, command = (
                self.engine.command_for_distance(
                    robot_id,
                    nearest,
                    vx,
                    vy
                )
            )

            commands[robot_id] = command

        self.sim.step(commands)

        self._check_tasks()
        self._check_safety()

    def _update_peers(self):
        for robot_id, robot in self.sim.robots.items():

            if not robot.active:
                continue

            for other_id, other in self.sim.robots.items():

                if (
                    other_id == robot_id
                    or not other.active
                ):
                    continue

                self.engine.set_peer_state(
                    other_id,
                    other.x,
                    other.y,
                    other.vx,
                    other.vy
                )

    def _get_peers(self, robot_id):
        result = []

        for other_id, robot in self.sim.robots.items():

            if (
                other_id == robot_id
                or not robot.active
            ):
                continue

            result.append({
                "id": other_id,
                "x": robot.x,
                "y": robot.y,
                "vx": robot.vx,
                "vy": robot.vy
            })

        return result

    def _nearest_distance(self, robot, peers):
        if not peers:
            return float("inf")

        best = float("inf")

        for peer in peers:
            dx = robot.x - peer["x"]
            dy = robot.y - peer["y"]

            d = sqrt(
                dx * dx +
                dy * dy
            )

            if d < best:
                best = d

        return best

    def _get_task(self, robot_id):
        for task in self.sim.tasks.values():

            if (
                task.assigned_robot == robot_id
                and not task.completed
                and task.status != TaskStatus.FAILED
            ):
                return task

        return None

    def _check_tasks(self):
        for task in self.sim.tasks.values():

            if (
                task.completed
                or not task.assigned_robot
            ):
                continue

            robot = self.sim.get_robot(
                task.assigned_robot
            )

            if not robot or not robot.active:
                continue

            cell = self.grid.to_cell(
                robot.x,
                robot.y
            )

            if cell == task.destination:
                task.complete()

                self.paths.pop(
                    robot.id,
                    None
                )

                self.log(
                    robot.id,
                    f"Task {task.id} completed"
                )

        if (
            self.sim.tasks
            and all(
                task.completed
                for task in self.sim.tasks.values()
            )
        ):
            self.running = False

            self.log(
                "SYSTEM",
                "All tasks completed"
            )

    def _check_safety(self):
        robots = [
            r for r in self.sim.robots.values()
            if r.active
        ]

        for i in range(len(robots)):
            for j in range(i + 1, len(robots)):

                a = robots[i]
                b = robots[j]

                dx = a.x - b.x
                dy = a.y - b.y

                d = sqrt(
                    dx * dx +
                    dy * dy
                )

                ra = self.specs[a.id].radius
                rb = self.specs[b.id].radius

                if d <= ra + rb:
                    self.log(
                        "SAFETY",
                        f"COLLISION {a.id}/{b.id}"
                    )

    def add_task(self, data):
        task = Task(
            data["id"],
            tuple(data["start"]),
            tuple(data["destination"]),
            data.get("assigned_robot")
        )

        self.sim.add_task(task)

        self.log(
            "TASK",
            f"Added {task.id}"
        )

        return {
            "ok": True,
            "task": task.id
        }

    def assign_task(self, task_id, robot_id):
        task = self.sim.tasks.get(task_id)

        if not task:
            return {
                "ok": False,
                "error": "Task not found"
            }

        if robot_id not in self.sim.robots:
            return {
                "ok": False,
                "error": "Robot not found"
            }

        task.assigned_robot = robot_id
        task.status = TaskStatus.PENDING
        task.completed = False

        self.paths.pop(
            robot_id,
            None
        )

        self.log(
            "TASK",
            f"{task_id} assigned to {robot_id}"
        )

        return {"ok": True}

    def add_fault(self, data):
        kind = data.get("kind")

        if kind == "robot_failure":

            robot_id = data.get("target")

            self.faults.kill_robot(
                robot_id
            )

            robot = self.sim.get_robot(
                robot_id
            )

            if robot:
                self.sim.kill_robot(
                    robot_id
                )

            self.log(
                "FAULT",
                f"Robot {robot_id} failed"
            )

        elif kind == "blocked_cell":

            x = int(data["x"])
            y = int(data["y"])

            self.faults.blocked_cell(
                x,
                y
            )

            obstacle = Obstacle(x, y)

            if obstacle not in self.sim.obstacles:
                self.sim.obstacles.append(
                    obstacle
                )

            self.paths.clear()

            self.log(
                "FAULT",
                f"Blocked cell ({x},{y})"
            )

        elif kind == "wifi_dead_zone":

            x = int(data["x"])
            y = int(data["y"])

            self.faults.wifi_dead_zone(
                x,
                y
            )

            self.log(
                "FAULT",
                f"Wi-Fi dead zone ({x},{y})"
            )

        return {"ok": True}

    def log(self, source, message):
        self.events.append({
            "time": round(
                self.sim.time,
                2
            ),
            "source": source,
            "message": message
        })

        if len(self.events) > 200:
            self.events = self.events[-200:]

    def state(self):
        robots = []

        for robot in self.sim.robots.values():

            task = self._get_task(
                robot.id
            )

            robots.append({
                "id": robot.id,
                "x": robot.x,
                "y": robot.y,
                "theta": robot.theta,
                "vx": robot.vx,
                "vy": robot.vy,
                "battery": robot.battery,
                "active": robot.active,
                "task_id":
                    task.id if task else None
            })

        tasks = []

        for task in self.sim.tasks.values():

            tasks.append({
                "id": task.id,
                "start": list(task.start),
                "destination":
                    list(task.destination),
                "assigned_robot":
                    task.assigned_robot,
                "completed":
                    task.completed,
                "status":
                    task.status.value
            })

        obstacles = [
            {
                "x": obstacle.x,
                "y": obstacle.y
            }
            for obstacle in self.sim.obstacles
        ]

        return {
            "time": round(
                self.sim.time,
                2
            ),
            "running": self.running,
            "scenario": self.scenario_id,
            "grid": {
                "width": self.grid.width,
                "height": self.grid.height,
                "cell_size":
                    self.grid.cell_size
            },
            "robots": robots,
            "tasks": tasks,
            "obstacles": obstacles,
            "events":
                self.events[-100:]
        }


============================================================
FILE: arcnet_api\__init__.py
============================================================



============================================================
FILE: frontend\app.js
============================================================

const API = "http://localhost:8000";

let state = {
    robots: [],
    tasks: [],
    obstacles: [],
    events: []
};


function showTab(id) {
    document.querySelectorAll(".tab").forEach(x => {
        x.classList.remove("active-tab");
    });

    document.getElementById(id).classList.add("active-tab");

    document.querySelectorAll(".nav").forEach(x => {
        x.classList.remove("active");
    });

    const buttons = document.querySelectorAll(".nav");

    const names = [
        "live",
        "tasks",
        "validation",
        "scenarios",
        "setup",
        "architecture"
    ];

    const i = names.indexOf(id);

    if (i >= 0) {
        buttons[i].classList.add("active");
    }

    const titles = {
        live: "Live Monitor",
        tasks: "Task Queue",
        validation: "Validation",
        scenarios: "Scenario Library",
        setup: "Fleet Setup",
        architecture: "System Architecture"
    };

    document.getElementById("title").innerText = titles[id];
}


async function api(path, method = "POST", body = null) {
    const options = {
        method,
        headers: {
            "Content-Type": "application/json"
        }
    };

    if (body) {
        options.body = JSON.stringify(body);
    }

    const res = await fetch(API + path, options);
    return await res.json();
}


function connect() {
    const ws = new WebSocket(
        "ws://localhost:8000/ws"
    );

    ws.onopen = () => {
        document.getElementById("connection").innerText =
            "Connected";

        document.getElementById("dot").style.background =
            "#65d48b";
    };

    ws.onclose = () => {
        document.getElementById("connection").innerText =
            "Disconnected";

        document.getElementById("dot").style.background =
            "#888";

        setTimeout(connect, 2000);
    };

    ws.onmessage = e => {
        state = JSON.parse(e.data);
        render();
    };
}


function render() {
    document.getElementById("time").innerText =
        state.time.toFixed(1) + " s";

    document.getElementById("robotCount").innerText =
        state.robots.length;

    document.getElementById("taskCount").innerText =
        state.tasks.filter(
            x => !x.completed
        ).length;

    document.getElementById("scenario").innerText =
        state.scenario;

    renderRobots();
    renderTasks();
    renderEvents();
    drawWarehouse();
}


function renderRobots() {
    const root =
        document.getElementById("robots");

    root.innerHTML = "";

    state.robots.forEach(r => {

        const div =
            document.createElement("div");

        div.className = "robot";

        div.innerHTML = `
            <div class="robot-name">
                ${r.id}
            </div>

            <div class="robot-meta">
                Position:
                ${r.x.toFixed(2)},
                ${r.y.toFixed(2)}
            </div>

            <div class="robot-meta">
                Battery:
                ${r.battery.toFixed(1)}%
            </div>

            <div class="robot-meta">
                Task:
                ${r.task_id || "Idle"}
            </div>

            <div class="robot-meta">
                Status:
                ${r.active ? "ACTIVE" : "FAILED"}
            </div>
        `;

        root.appendChild(div);
    });
}


function renderTasks() {
    const root =
        document.getElementById("taskTable");

    root.innerHTML = "";

    state.tasks.forEach(t => {

        const row =
            document.createElement("tr");

        row.innerHTML = `
            <td>${t.id}</td>
            <td>${t.start}</td>
            <td>${t.destination}</td>
            <td>${t.assigned_robot || "-"}</td>
            <td>${t.status}</td>
        `;

        root.appendChild(row);
    });
}


function renderEvents() {
    const root =
        document.getElementById("events");

    root.innerHTML = "";

    [...state.events]
        .reverse()
        .slice(0, 15)
        .forEach(e => {

            const div =
                document.createElement("div");

            div.className = "event";

            div.innerHTML = `
                <span>
                    ${e.time}s
                </span>

                <strong>
                    ${e.kind}
                </strong>

                ${e.message}
            `;

            root.appendChild(div);
        });
}


function drawWarehouse() {
    const canvas =
        document.getElementById("warehouse");

    const ctx =
        canvas.getContext("2d");

    const rect =
        canvas.getBoundingClientRect();

    canvas.width = rect.width;
    canvas.height = rect.height;

    const gw = state.grid.width;
    const gh = state.grid.height;

    const cw = canvas.width / gw;
    const ch = canvas.height / gh;

    ctx.clearRect(
        0,
        0,
        canvas.width,
        canvas.height
    );

    ctx.strokeStyle = "#26313a";
    ctx.lineWidth = 1;

    for (let x = 0; x <= gw; x++) {
        ctx.beginPath();
        ctx.moveTo(x * cw, 0);
        ctx.lineTo(x * cw, canvas.height);
        ctx.stroke();
    }

    for (let y = 0; y <= gh; y++) {
        ctx.beginPath();
        ctx.moveTo(0, y * ch);
        ctx.lineTo(canvas.width, y * ch);
        ctx.stroke();
    }

    state.obstacles.forEach(o => {
        ctx.fillStyle = "#4a525a";

        ctx.fillRect(
            o.x * cw,
            o.y * ch,
            cw,
            ch
        );
    });

    state.tasks.forEach(t => {

        const sx = t.start[0] * cw + cw / 2;
        const sy = t.start[1] * ch + ch / 2;

        const dx =
            t.destination[0] * cw + cw / 2;

        const dy =
            t.destination[1] * ch + ch / 2;

        ctx.strokeStyle = "#586674";
        ctx.setLineDash([4, 4]);

        ctx.beginPath();
        ctx.moveTo(sx, sy);
        ctx.lineTo(dx, dy);
        ctx.stroke();

        ctx.setLineDash([]);
    });

    state.robots.forEach(r => {

        const x = r.x / state.grid.cell_size * cw;
        const y = r.y / state.grid.cell_size * ch;

        ctx.save();

        ctx.translate(x, y);
        ctx.rotate(r.theta);

        ctx.fillStyle =
            r.active ? "#dbe7f2" : "#555";

        ctx.fillRect(
            -cw * 0.28,
            -ch * 0.20,
            cw * 0.56,
            ch * 0.40
        );

        ctx.fillStyle = "#111";

        ctx.fillRect(
            cw * 0.05,
            -ch * 0.08,
            cw * 0.25,
            ch * 0.16
        );

        ctx.restore();

        ctx.fillStyle = "#e8edf3";
        ctx.font = "11px Arial";

        ctx.fillText(
            r.id,
            x + 7,
            y - 7
        );
    });
}


async function createTask() {

    const id =
        document.getElementById("taskId").value;

    const sx =
        Number(document.getElementById("sx").value);

    const sy =
        Number(document.getElementById("sy").value);

    const dx =
        Number(document.getElementById("dx").value);

    const dy =
        Number(document.getElementById("dy").value);

    const robot =
        document.getElementById("taskRobot").value;

    await api(
        "/api/tasks",
        "POST",
        {
            id,
            start: [sx, sy],
            destination: [dx, dy],
            assigned_robot: robot || null
        }
    );
}


async function fault(
    kind,
    target = null,
    x = null,
    y = null
) {

    await api(
        "/api/faults",
        "POST",
        {
            kind,
            target,
            x,
            y
        }
    );
}


async function loadScenarios() {

    const scenarios =
        await api(
            "/api/scenarios",
            "GET"
        );

    const root =
        document.getElementById("scenarioGrid");

    root.innerHTML = "";

    scenarios.forEach(s => {

        const div =
            document.createElement("div");

        div.className = "scenario";

        div.innerHTML = `
            <strong>${s.name}</strong>
            <small>
                Load scenario
            </small>
        `;

        div.onclick = async () => {

            await api(
                "/api/scenarios/load",
                "POST",
                {
                    scenario_id: s.id
                }
            );
        };

        root.appendChild(div);
    });
}


async function runValidation() {

    const root =
        document.getElementById(
            "validationResult"
        );

    root.innerHTML =
        "Validation runner will execute the selected scenario against both strategies.";
}


connect();
loadScenarios();


============================================================
FILE: frontend\index.html
============================================================

<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ARCNET Fleet Control</title>
    <link rel="stylesheet" href="style.css">
</head>

<body>

<div class="app">

    <aside class="sidebar">
        <div class="logo">ARCNET</div>
        <div class="sub">DECENTRALIZED FLEET CONTROL</div>

        <button class="nav active" onclick="showTab('live')">
            Live Monitor
        </button>

        <button class="nav" onclick="showTab('tasks')">
            Tasks
        </button>

        <button class="nav" onclick="showTab('validation')">
            Validation
        </button>

        <button class="nav" onclick="showTab('scenarios')">
            Scenario Library
        </button>

        <button class="nav" onclick="showTab('setup')">
            Setup
        </button>

        <button class="nav" onclick="showTab('architecture')">
            Architecture
        </button>

        <div class="connection">
            <span id="dot"></span>
            <span id="connection">Connecting</span>
        </div>
    </aside>

    <main>

        <header>
            <div>
                <h1 id="title">Live Monitor</h1>
                <p>Autonomous Mobile Robot Coordination</p>
            </div>

            <div class="controls">
                <button onclick="api('/api/run/start')">
                    Start
                </button>

                <button onclick="api('/api/run/stop')">
                    Stop
                </button>

                <button onclick="api('/api/run/reset')">
                    Reset
                </button>
            </div>
        </header>

        <section id="live" class="tab active-tab">

            <div class="stats">

                <div class="stat">
                    <span>Simulation Time</span>
                    <strong id="time">0.0 s</strong>
                </div>

                <div class="stat">
                    <span>Robots</span>
                    <strong id="robotCount">0</strong>
                </div>

                <div class="stat">
                    <span>Active Tasks</span>
                    <strong id="taskCount">0</strong>
                </div>

                <div class="stat">
                    <span>Scenario</span>
                    <strong id="scenario">-</strong>
                </div>

            </div>

            <div class="monitor">

                <div class="warehouse-card">
                    <div class="card-title">
                        Warehouse State
                    </div>

                    <canvas id="warehouse"></canvas>
                </div>

                <div class="side-card">

                    <div class="card-title">
                        Robot Status
                    </div>

                    <div id="robots"></div>

                    <div class="card-title events-title">
                        Decision / Event Log
                    </div>

                    <div id="events"></div>

                </div>

            </div>

        </section>


        <section id="tasks" class="tab">

            <div class="panel">

                <div class="card-title">
                    Task Queue
                </div>

                <table>
                    <thead>
                    <tr>
                        <th>Task</th>
                        <th>Start</th>
                        <th>Destination</th>
                        <th>Robot</th>
                        <th>Status</th>
                    </tr>
                    </thead>

                    <tbody id="taskTable"></tbody>
                </table>

            </div>

            <div class="panel form-panel">

                <div class="card-title">
                    Create Task
                </div>

                <input id="taskId" placeholder="Task ID">

                <input id="sx" type="number" placeholder="Start X">
                <input id="sy" type="number" placeholder="Start Y">

                <input id="dx" type="number" placeholder="Destination X">
                <input id="dy" type="number" placeholder="Destination Y">

                <select id="taskRobot">
                    <option value="">Unassigned</option>
                    <option value="R1">R1</option>
                    <option value="R2">R2</option>
                    <option value="R3">R3</option>
                </select>

                <button onclick="createTask()">
                    Create Task
                </button>

            </div>

        </section>


        <section id="validation" class="tab">

            <div class="panel">

                <div class="card-title">
                    Validation
                </div>

                <p>
                    Compare identical scenarios using the
                    Stop-and-Wait baseline and ARCNET coordination.
                </p>

                <button onclick="runValidation()">
                    Run Validation
                </button>

                <div id="validationResult"></div>

            </div>

        </section>


        <section id="scenarios" class="tab">

            <div class="scenario-grid" id="scenarioGrid"></div>

            <div class="panel">

                <div class="card-title">
                    Fault Injection
                </div>

                <div class="fault-buttons">

                    <button onclick="fault('robot_failure','R1')">
                        Kill R1
                    </button>

                    <button onclick="fault('robot_failure','R2')">
                        Kill R2
                    </button>

                    <button onclick="fault('blocked_cell',null,8,3)">
                        Block Aisle
                    </button>

                    <button onclick="fault('wifi_dead_zone',null,8,3)">
                        Wi-Fi Dead Zone
                    </button>

                </div>

            </div>

        </section>


        <section id="setup" class="tab">

            <div class="panel">

                <div class="card-title">
                    Fleet Setup
                </div>

                <div class="setup-grid">

                    <div>
                        <label>Robot ID</label>
                        <input value="R1">
                    </div>

                    <div>
                        <label>Shape</label>
                        <select>
                            <option>Rectangle</option>
                            <option>Circle</option>
                        </select>
                    </div>

                    <div>
                        <label>Length</label>
                        <input value="0.9">
                    </div>

                    <div>
                        <label>Width</label>
                        <input value="0.64">
                    </div>

                    <div>
                        <label>Max Speed</label>
                        <input value="1.0">
                    </div>

                    <div>
                        <label>Braking</label>
                        <input value="0.8">
                    </div>

                </div>

            </div>

        </section>


        <section id="architecture" class="tab">

            <div class="architecture">

                <div class="arch-node">USER</div>
                <div class="arrow">↓</div>

                <div class="arch-node">2D WEB DASHBOARD</div>
                <div class="arrow">↓ HTTPS / WebSocket</div>

                <div class="arch-node">FASTAPI</div>
                <div class="arrow">↓</div>

                <div class="arch-node">SESSION MANAGER</div>
                <div class="arrow">↓</div>

                <div class="arch-node">ROS 2</div>
                <div class="arrow">↓</div>

                <div class="arch-node">HEADLESS GAZEBO</div>
                <div class="arrow">↕</div>

                <div class="arch-node highlight">
                    ARCNET CORE
                </div>

            </div>

        </section>

    </main>

</div>

<script src="app.js"></script>

</body>
</html>


============================================================
FILE: frontend\style.css
============================================================

* {
    box-sizing: border-box;
}

body {
    margin: 0;
    font-family: Arial, sans-serif;
    background: #0b0f14;
    color: #e8edf3;
}

.app {
    display: flex;
    min-height: 100vh;
}

.sidebar {
    width: 240px;
    background: #10161d;
    border-right: 1px solid #27313c;
    padding: 28px 18px;
}

.logo {
    font-size: 28px;
    font-weight: 800;
    letter-spacing: 3px;
}

.sub {
    color: #788594;
    font-size: 9px;
    margin-top: 5px;
    margin-bottom: 40px;
}

.nav {
    width: 100%;
    border: 0;
    background: transparent;
    color: #8e9aa8;
    padding: 14px;
    text-align: left;
    border-radius: 8px;
    margin-bottom: 5px;
    cursor: pointer;
    font-size: 14px;
}

.nav:hover,
.nav.active {
    background: #1c2630;
    color: white;
}

.connection {
    position: absolute;
    bottom: 25px;
    font-size: 12px;
    color: #8e9aa8;
}

#dot {
    display: inline-block;
    width: 8px;
    height: 8px;
    border-radius: 50%;
    background: #888;
    margin-right: 7px;
}

main {
    flex: 1;
    padding: 30px;
    overflow: auto;
}

header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    margin-bottom: 25px;
}

h1 {
    margin: 0;
    font-size: 28px;
}

header p {
    color: #7d8996;
    margin-top: 5px;
}

button {
    background: #202b36;
    color: white;
    border: 1px solid #344250;
    border-radius: 7px;
    padding: 10px 16px;
    cursor: pointer;
}

button:hover {
    background: #2b3946;
}

.stats {
    display: grid;
    grid-template-columns: repeat(4, 1fr);
    gap: 15px;
    margin-bottom: 20px;
}

.stat,
.panel,
.warehouse-card,
.side-card {
    background: #111820;
    border: 1px solid #27313c;
    border-radius: 10px;
}

.stat {
    padding: 18px;
}

.stat span {
    display: block;
    color: #788594;
    font-size: 12px;
    margin-bottom: 10px;
}

.stat strong {
    font-size: 22px;
}

.monitor {
    display: grid;
    grid-template-columns: 2fr 1fr;
    gap: 20px;
}

.warehouse-card {
    padding: 18px;
}

.side-card {
    padding: 18px;
}

.card-title {
    font-weight: bold;
    margin-bottom: 15px;
}

#warehouse {
    width: 100%;
    height: 580px;
    background: #0b1015;
    border-radius: 7px;
}

.robot {
    padding: 12px;
    border-bottom: 1px solid #25303a;
}

.robot-name {
    font-weight: bold;
}

.robot-meta {
    color: #7d8996;
    font-size: 11px;
    margin-top: 5px;
}

.events-title {
    margin-top: 25px;
}

.event {
    padding: 7px;
    border-bottom: 1px solid #202932;
    font-size: 11px;
}

.event span {
    color: #697785;
    margin-right: 5px;
}

.tab {
    display: none;
}

.active-tab {
    display: block;
}

.panel {
    padding: 22px;
    margin-bottom: 20px;
}

table {
    width: 100%;
    border-collapse: collapse;
}

th,
td {
    text-align: left;
    padding: 13px;
    border-bottom: 1px solid #26313b;
    font-size: 13px;
}

th {
    color: #84909d;
}

.form-panel {
    max-width: 700px;
}

input,
select {
    background: #0c1218;
    color: white;
    border: 1px solid #303c48;
    border-radius: 6px;
    padding: 11px;
    margin: 5px;
}

.scenario-grid {
    display: grid;
    grid-template-columns: repeat(4, 1fr);
    gap: 14px;
    margin-bottom: 20px;
}

.scenario {
    background: #111820;
    border: 1px solid #27313c;
    border-radius: 9px;
    padding: 18px;
    cursor: pointer;
}

.scenario:hover {
    border-color: #6c7c8d;
}

.scenario strong {
    display: block;
    margin-bottom: 8px;
}

.scenario small {
    color: #788594;
}

.fault-buttons {
    display: flex;
    gap: 10px;
    flex-wrap: wrap;
}

.setup-grid {
    display: grid;
    grid-template-columns: repeat(3, 1fr);
}

.setup-grid label {
    display: block;
    color: #7f8b98;
    font-size: 12px;
    margin: 5px;
}

.architecture {
    display: flex;
    flex-direction: column;
    align-items: center;
    gap: 8px;
    padding: 40px;
}

.arch-node {
    width: 300px;
    padding: 20px;
    text-align: center;
    border: 1px solid #35424f;
    border-radius: 8px;
    background: #121a22;
}

.highlight {
    border-color: #71869a;
}

.arrow {
    color: #788594;
}


# EXISTING GAZEBO / BCR TREE

Folder PATH listing for volume New Volume
Volume serial number is 0000024E EA57:9F03
E:\OTHERS\HACKATHONS\SIH'26\AMR-FLEET-COORDINATION\SRC
+---aws-robomaker-small-warehouse-world
|       README.md
|       
\---bcr_bot
    |   .gitignore
    |   CHANGELOG.rst
    |   CMakeLists.txt
    |   Dockerfile
    |   LICENSE
    |   package.xml
    |   README.md
    |   
    +---config
    |       bcr_map.pgm
    |       bcr_map.yaml
    |       mapper_params_online_async.yaml
    |       nav2_params.yaml
    |       
    +---launch
    |       bcr_bot_gazebo_spawn.launch.py
    |       bcr_bot_gz_spawn.launch.py
    |       bcr_bot_ign_spawn.launch.py
    |       bcr_bot_multi_spawn.launch.py
    |       fleet_bringup.launch.py
    |       gazebo.launch.py
    |       gz.launch.py
    |       ign.launch.py
    |       mapping.launch.py
    |       nav2.launch.py
    |       rviz.launch.py
    |       world_bringup.launch.py
    |       
    +---meshes
    |   |   bcr_bot_mesh.dae
    |   |   logo.png
    |   |   realsense_texture.png
    |   |   
    |   \---kinect
    |           d415.dae
    |           kinect.dae
    |           kinect.jpg
    |           kinect.tga
    |           
    +---models
    |   +---aws_robomaker_warehouse_Bucket_01
    |   |   |   .DS_Store
    |   |   |   model.config
    |   |   |   model.sdf
    |   |   |   
    |   |   +---materials
    |   |   |   |   .DS_Store
    |   |   |   |   
    |   |   |   \---textures
    |   |   |           aws_robomaker_warehouse_Bucket_01.png
    |   |   |           
    |   |   \---meshes
    |   |           aws_robomaker_warehouse_Bucket_01_collision.DAE
    |   |           aws_robomaker_warehouse_Bucket_01_visual.DAE
    |   |           
    |   +---aws_robomaker_warehouse_ClutteringA_01
    |   |   |   .DS_Store
    |   |   |   model.config
    |   |   |   model.sdf
    |   |   |   
    |   |   +---materials
    |   |   |   |   .DS_Store
    |   |   |   |   
    |   |   |   \---textures
    |   |   |           .DS_Store
    |   |   |           aws_robomaker_warehouse_ClutteringA_01.png
    |   |   |           aws_robomaker_warehouse_ClutteringA_02.png
    |   |   |           aws_robomaker_warehouse_ClutteringA_03.png
    |   |   |           
    |   |   \---meshes
    |   |           aws_robomaker_warehouse_ClutteringA_01_collision.DAE
    |   |           aws_robomaker_warehouse_ClutteringA_01_visual.DAE
    |   |           
    |   +---aws_robomaker_warehouse_ClutteringC_01
    |   |   |   model.config
    |   |   |   model.sdf
    |   |   |   
    |   |   +---materials
    |   |   |   \---textures
    |   |   |           aws_robomaker_warehouse_ClutteringC_01.png
    |   |   |           aws_robomaker_warehouse_ClutteringC_02.png
    |   |   |           
    |   |   \---meshes
    |   |           aws_robomaker_warehouse_ClutteringC_01_collision.DAE
    |   |           aws_robomaker_warehouse_ClutteringC_01_visual.DAE
    |   |           
    |   +---aws_robomaker_warehouse_ClutteringD_01
    |   |   |   model.config
    |   |   |   model.sdf
    |   |   |   
    |   |   +---materials
    |   |   |   \---textures
    |   |   |           aws_robomaker_warehouse_ClutteringD_01.png
    |   |   |           aws_robomaker_warehouse_ClutteringD_02.png
    |   |   |           
    |   |   \---meshes
    |   |           aws_robomaker_warehouse_ClutteringD_01_collision.DAE
    |   |           aws_robomaker_warehouse_ClutteringD_01_visual.DAE
    |   |           
    |   +---aws_robomaker_warehouse_DeskC_01
    |   |   |   model.config
    |   |   |   model.sdf
    |   |   |   
    |   |   +---materials
    |   |   |   |   .DS_Store
    |   |   |   |   
    |   |   |   \---textures
    |   |   |           aws_robomaker_warehouse_DeskC_01.png
    |   |   |           aws_robomaker_warehouse_DeskC_02.png
    |   |   |           aws_robomaker_warehouse_DeskC_03.png
    |   |   |           aws_robomaker_warehouse_DeskC_03.psd
    |   |   |           aws_robomaker_warehouse_DeskC_04.png
    |   |   |           
    |   |   \---meshes
    |   |           aws_robomaker_warehouse_DeskC_01_collision.DAE
    |   |           aws_robomaker_warehouse_DeskC_01_visual.DAE
    |   |           
    |   +---aws_robomaker_warehouse_GroundB_01
    |   |   |   model.config
    |   |   |   model.sdf
    |   |   |   
    |   |   +---materials
    |   |   |   \---textures
    |   |   |           aws_robomaker_warehouse_GroundB_01.png
    |   |   |           aws_robomaker_warehouse_GroundB_02.png
    |   |   |           
    |   |   \---meshes
    |   |           aws_robomaker_warehouse_GroundB_01_collision.DAE
    |   |           aws_robomaker_warehouse_GroundB_01_visual.DAE
    |   |           
    |   +---aws_robomaker_warehouse_Lamp_01
    |   |   |   model.config
    |   |   |   model.sdf
    |   |   |   
    |   |   +---materials
    |   |   |   \---textures
    |   |   |           aws_robomaker_warehouse_Lamp_01.png
    |   |   |           
    |   |   \---meshes
    |   |           aws_robomaker_warehouse_Lamp_01_collision.DAE
    |   |           aws_robomaker_warehouse_Lamp_01_visual.DAE
    |   |           
    |   +---aws_robomaker_warehouse_PalletJackB_01
    |   |   |   .DS_Store
    |   |   |   model.config
    |   |   |   model.sdf
    |   |   |   
    |   |   +---materials
    |   |   |   |   .DS_Store
    |   |   |   |   
    |   |   |   \---textures
    |   |   |           aws_robomaker_warehouse_PalletJackB_01.png
    |   |   |           aws_robomaker_warehouse_PalletJackB_02.png
    |   |   |           
    |   |   \---meshes
    |   |           aws_robomaker_warehouse_PalletJackB_01_collision.DAE
    |   |           aws_robomaker_warehouse_PalletJackB_01_visual.DAE
    |   |           
    |   +---aws_robomaker_warehouse_RoofB_01
    |   |   |   model.config
    |   |   |   model.sdf
    |   |   |   
    |   |   +---materials
    |   |   |   \---textures
    |   |   |           aws_robomaker_warehouse_RoofB_01.png
    |   |   |           
    |   |   \---meshes
    |   |           aws_robomaker_warehouse_RoofB_01_collision.DAE
    |   |           aws_robomaker_warehouse_RoofB_01_visual.DAE
    |   |           
    |   +---aws_robomaker_warehouse_ShelfD_01
    |   |   |   model.config
    |   |   |   model.sdf
    |   |   |   
    |   |   +---materials
    |   |   |   \---textures
    |   |   |           aws_robomaker_warehouse_ShelfD_01.png
    |   |   |           aws_robomaker_warehouse_ShelfD_02.png
    |   |   |           aws_robomaker_warehouse_ShelfD_03.png
    |   |   |           aws_robomaker_warehouse_ShelfD_04.png
    |   |   |           
    |   |   \---meshes
    |   |           aws_robomaker_warehouse_ShelfD_01_collision.DAE
    |   |           aws_robomaker_warehouse_ShelfD_01_visual.DAE
    |   |           
    |   +---aws_robomaker_warehouse_ShelfE_01
    |   |   |   .DS_Store
    |   |   |   model.config
    |   |   |   model.sdf
    |   |   |   
    |   |   +---materials
    |   |   |   |   .DS_Store
    |   |   |   |   
    |   |   |   \---textures
    |   |   |           aws_robomaker_warehouse_ShelfE_01.png
    |   |   |           aws_robomaker_warehouse_ShelfE_02.png
    |   |   |           aws_robomaker_warehouse_ShelfE_03.png
    |   |   |           aws_robomaker_warehouse_ShelfE_04.png
    |   |   |           
    |   |   \---meshes
    |   |           aws_robomaker_warehouse_ShelfE_01_collision.DAE
    |   |           aws_robomaker_warehouse_ShelfE_01_visual.DAE
    |   |           
    |   +---aws_robomaker_warehouse_ShelfF_01
    |   |   |   model.config
    |   |   |   model.sdf
    |   |   |   
    |   |   +---materials
    |   |   |   \---textures
    |   |   |           aws_robomaker_warehouse_ShelfF_01.png
    |   |   |           aws_robomaker_warehouse_ShelfF_02.png
    |   |   |           aws_robomaker_warehouse_ShelfF_03.png
    |   |   |           
    |   |   \---meshes
    |   |           aws_robomaker_warehouse_ShelfF_01_collision.DAE
    |   |           aws_robomaker_warehouse_ShelfF_01_visual.DAE
    |   |           
    |   +---aws_robomaker_warehouse_TrashCanC_01
    |   |   |   model.config
    |   |   |   model.sdf
    |   |   |   
    |   |   +---materials
    |   |   |   \---textures
    |   |   |           aws_robomaker_warehouse_TrashCanC_01.png
    |   |   |           
    |   |   \---meshes
    |   |           aws_robomaker_warehouse_TrashCanC_01_collision.DAE
    |   |           aws_robomaker_warehouse_TrashCanC_01_visual.DAE
    |   |           
    |   \---aws_robomaker_warehouse_WallB_01
    |       |   model.config
    |       |   model.sdf
    |       |   
    |       +---materials
    |       |   \---textures
    |       |           aws_robomaker_warehouse_WallB_01.png
    |       |           
    |       \---meshes
    |               aws_robomaker_warehouse_WallB_01_collision.DAE
    |               aws_robomaker_warehouse_WallB_01_visual.DAE
    |               
    +---res
    |       gz.jpg
    |       isaac.jpg
    |       rviz.jpg
    |       
    +---rviz
    |       entire_setup.rviz
    |       map.rviz
    |       
    +---scripts
    |       remapper.py
    |       
    +---urdf
    |       bcr_bot.xacro
    |       gazebo.xacro
    |       gz.xacro
    |       ign.xacro
    |       macros.xacro
    |       materials.xacro
    |       
    +---usd
    |       ActionGraphFull.usd
    |       bcr_bot.usd
    |       scene.usd
    |       warehouse_scene.usd
    |       
    \---worlds
            empty.sdf
            small_warehouse.sdf
            
