from dataclasses import dataclass


@dataclass
class Event:
    time: float
    kind: str
    robot_id: str | None = None
    task_id: str | None = None
    peer_id: str | None = None
    data: dict | None = None


class EventLog:

    def __init__(self):
        self.events = []

    def add(
        self,
        time,
        kind,
        robot_id=None,
        task_id=None,
        peer_id=None,
        data=None
    ):
        self.events.append(
            Event(
                time,
                kind,
                robot_id,
                task_id,
                peer_id,
                data or {}
            )
        )

    def all(self):
        return list(self.events)

    def clear(self):
        self.events.clear()