# UI-only preview with fake data. Not part of the product.
import asyncio
import math
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles

app = FastAPI()
sim = {"t": 0.0, "on": False}


def state():
    p = sim["t"] % 15
    return {
        "time": round(sim["t"], 1),
        "running": sim["on"],
        "scenario": "head_on",
        "grid": {"width": 20, "height": 12, "cell_size": 1.0},
        "robots": [
            {"id": "R1", "x": 2.5 + p, "y": 3.5, "theta": 0, "battery": 82,
             "active": True, "task_id": "T001", "mode": "moving", "safety": "normal",
             "plan": [[i, 3] for i in range(int(2.5 + p), 18)]},
            {"id": "R2", "x": 17.5 - p, "y": 5.5, "theta": math.pi, "battery": 64,
             "active": True, "task_id": "T002", "mode": "moving", "safety": "caution",
             "reason": "R1 ahead", "plan": [[i, 5] for i in range(2, int(17.5 - p) + 1)]},
            {"id": "R3", "x": 10.5, "y": 8.5, "theta": -1.57, "battery": 40,
             "active": True, "task_id": None, "mode": "idle", "safety": "hard_stop"},
        ],
        "tasks": [
            {"id": "T001", "start": [2, 3], "destination": [17, 3], "assigned_robot": "R1",
             "completed": False, "status": "active"},
            {"id": "T002", "start": [17, 5], "destination": [2, 5], "assigned_robot": "R2",
             "completed": False, "status": "active"},
        ],
        "obstacles": [{"x": x, "y": 7, "temporary": False} for x in range(4, 9)]
                     + [{"x": 12, "y": 4, "temporary": True}],
        "events": [
            {"time": 1.0, "kind": "TASK", "source": "R1", "message": "R1 started T001"},
            {"time": 2.4, "kind": "YIELD", "source": "R2", "message": "R2 waits for R1"},
            {"time": 3.1, "kind": "SAFETY", "source": "R3", "message": "R3 hard stop"},
        ],
        "metrics": {"collisions": 0, "near_misses": 1, "deadlocks": 0},
    }


@app.get("/api/scenarios")
def scenarios():
    return [{"id": "head_on", "name": "Head-on", "description": "Two robots, one aisle"},
            {"id": "four_way_choke", "name": "4-way choke", "description": "Four robots, one crossing"}]


@app.post("/api/run/start")
def start():
    sim["on"] = True
    return {"ok": True}


@app.post("/api/run/stop")
def stop():
    sim["on"] = False
    return {"ok": True}


@app.post("/api/run/reset")
def reset():
    sim["on"] = False
    sim["t"] = 0.0
    return {"ok": True}


@app.post("/api/{rest:path}")
def anything(rest: str, data: dict = None):
    return {"ok": True}


@app.websocket("/ws")
async def ws(w: WebSocket):
    await w.accept()
    try:
        while True:
            if sim["on"]:
                sim["t"] += 0.1
            await w.send_json(state())
            await asyncio.sleep(0.1)
    except WebSocketDisconnect:
        pass


app.mount("/", StaticFiles(directory="frontend", html=True), name="ui")