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