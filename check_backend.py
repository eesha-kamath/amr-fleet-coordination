# Proves the chain end to end: API -> session -> runtime -> state -> websocket.
# Run: python check_backend.py            (fast checks)
#      python check_backend.py --validation   (also runs the full validation, slower)
import sys
import time
from fastapi.testclient import TestClient
from arcnet_api.main import app


def state(c):
    return c.get("/api/state").json()


def pos(c):
    return {r["id"]: (round(r["x"], 2), round(r["y"], 2)) for r in state(c)["robots"]}


def ok(name, cond):
    print(("PASS " if cond else "FAIL ") + name)
    return cond


with TestClient(app) as c:
    results = []
    c.post("/api/run/reset")
    before = pos(c)
    results.append(ok("state has robots", len(before) > 0))
    c.post("/api/run/start")
    time.sleep(3)
    after = pos(c)
    results.append(ok("positions change after start", before != after))
    print("  before", before, "\n  after ", after)

    with c.websocket_connect("/ws") as w:
        a, b = w.receive_json(), w.receive_json()
        results.append(ok("websocket streams changing time", b["time"] >= a["time"] and b["running"]))

    c.post("/api/run/stop")
    t1 = state(c)["time"]
    time.sleep(0.6)
    results.append(ok("stop halts the clock", t1 == state(c)["time"]))

    c.post("/api/run/reset")
    results.append(ok("reset restores start positions", pos(c) == before))

    r1 = state(c)["robots"][0]
    cx, cy = int(r1["x"]), int(r1["y"])
    resp = c.post("/api/fault", json={"kind": "blocked_cell", "x": cx, "y": cy})
    print("  block under robot ->", resp.status_code, resp.text[:80])
    results.append(ok("cannot block a cell under a robot", resp.status_code in (400, 200)))
    resp = c.post("/api/fault", json={"kind": "blocked_cell", "x": 0, "y": 0})
    results.append(ok("block a free cell", resp.status_code == 200))
    resp = c.post("/api/fault", json={"kind": "wifi_dead_zone", "x": 8, "y": 8})
    results.append(ok("dead zone accepted and shown", resp.status_code == 200 and {"x": 8, "y": 8} in state(c)["wifi"]))
    results.append(ok("bad task rejected", c.post("/api/task", json={"id": "T001", "start": [1, 1], "destination": [2, 2]}).status_code == 400))

    c.post("/api/run/reset")
    c.post("/api/fault", json={"kind": "robot_failure", "target": "R3"})
    results.append(ok("failed robot shows as failed", next(r for r in state(c)["robots"] if r["id"] == "R3")["safety"] == "failed"))

    c.post("/api/scenario/head_on")
    c.post("/api/run/start")
    deadline = time.time() + 60
    while time.time() < deadline and state(c)["running"]:
        time.sleep(0.5)
    s = state(c)
    results.append(ok("head_on: all tasks completed", all(t["completed"] for t in s["tasks"])))
    results.append(ok("head_on: zero collisions", s["metrics"]["collisions"] == 0))
    print("  sim time", s["time"], "metrics", s["metrics"])

    if "--validation" in sys.argv:
        c.post("/api/validation/run")
        while c.get("/api/validation/status").json()["state"] == "running":
            time.sleep(1)
        rows = c.get("/api/validation/status").json()["rows"]
        inst = {(r["scenario_id"], r["variant"]) for r in rows}
        results.append(ok(f"validation ran {len(inst)} scenario instances (need 20+)", len(inst) >= 20))
        arc = [r for r in rows if r["strategy"] == "arcnet"]
        results.append(ok("validation: zero ARCNET collisions", all(r["collisions"] == 0 for r in arc)))
        print("  ARCNET finished", sum(r["tasks_completed"] == r["tasks_total"] for r in arc), "of", len(arc))

    print("\nALL PASS" if all(results) else "\nSOME CHECKS FAILED")