import asyncio
from pathlib import Path

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles

from .session import Session, start_validation, validation_status

app = FastAPI(title="ARCNET API")
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])

session = Session()


def guard(fn, *args):
    try:
        return fn(*args)
    except (ValueError, KeyError) as e:
        raise HTTPException(status_code=400, detail=str(e))


@app.get("/api/state")
def state():
    return session.state()


@app.get("/api/scenarios")
def scenarios():
    return session.scenarios()


@app.post("/api/run/start")
def start():
    session.start()
    return {"ok": True}


@app.post("/api/run/stop")
def stop():
    session.stop()
    return {"ok": True}


@app.post("/api/run/reset")
def reset():
    session.reset()
    return {"ok": True}


@app.post("/api/scenario/{scenario_id}")
def scenario(scenario_id: str):
    guard(session.load, scenario_id)
    return {"ok": True, "scenario": scenario_id}


@app.post("/api/strategy/{name}")
def strategy(name: str):
    guard(session.set_strategy, name)
    return {"ok": True, "strategy": name}


@app.post("/api/task")
def task(data: dict):
    return guard(session.add_task, data)


@app.post("/api/task/{task_id}/assign")
def assign(task_id: str, data: dict):
    return guard(session.assign_task, task_id, data.get("robot_id"))


@app.post("/api/fault")
def fault(data: dict):
    return guard(session.add_fault, data)


@app.post("/api/validation/run")
def validation_run():
    guard(start_validation)
    return validation_status()


@app.get("/api/validation/status")
def validation_state():
    return validation_status()


@app.websocket("/ws")
async def ws(w: WebSocket):
    await w.accept()
    try:
        while True:
            await w.send_json(session.state())
            await asyncio.sleep(0.1)
    except (WebSocketDisconnect, RuntimeError):
        pass


FRONTEND = Path(__file__).resolve().parent.parent / "frontend"
app.mount("/", StaticFiles(directory=FRONTEND, html=True), name="ui")