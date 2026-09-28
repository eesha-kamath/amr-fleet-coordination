from dataclasses import dataclass
from enum import Enum


class TaskStatus(Enum):
    PENDING = "pending"
    ACTIVE = "active"
    COMPLETED = "completed"
    FAILED = "failed"


@dataclass
class Task:
    id: str
    start: tuple[int, int]
    destination: tuple[int, int]
    assigned_robot: str | None = None
    completed: bool = False
    status: TaskStatus = TaskStatus.PENDING

    def activate(self):
        if self.status == TaskStatus.PENDING:
            self.status = TaskStatus.ACTIVE

    def complete(self):
        self.completed = True
        self.status = TaskStatus.COMPLETED

    def fail(self):
        self.status = TaskStatus.FAILED
        self.completed = False

    def release(self):
        self.assigned_robot = None
        self.status = TaskStatus.PENDING
        self.completed = False