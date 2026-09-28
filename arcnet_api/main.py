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