from dataclasses import dataclass, field


@dataclass
class Metrics:
    completion_time: float = 0.0
    collisions: int = 0
    near_misses: int = 0
    stops: int = 0
    replans: int = 0
    deadlocks_detected: int = 0
    deadlocks_resolved: int = 0
    tasks_completed: int = 0
    tasks_failed: int = 0
    events: list = field(default_factory=list)

    def record_collision(self):
        self.collisions += 1

    def record_near_miss(self):
        self.near_misses += 1

    def record_stop(self):
        self.stops += 1

    def record_replan(self):
        self.replans += 1

    def record_deadlock(self):
        self.deadlocks_detected += 1

    def record_deadlock_resolution(self):
        self.deadlocks_resolved += 1

    def event(self, kind, **data):
        self.events.append({
            "time": data.pop("time", 0.0),
            "kind": kind,
            **data
        })

    def as_dict(self):
        return {
            "completion_time": self.completion_time,
            "collisions": self.collisions,
            "near_misses": self.near_misses,
            "stops": self.stops,
            "replans": self.replans,
            "deadlocks_detected": self.deadlocks_detected,
            "deadlocks_resolved": self.deadlocks_resolved,
            "tasks_completed": self.tasks_completed,
            "tasks_failed": self.tasks_failed
        }