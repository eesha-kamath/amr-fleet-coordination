from dataclasses import dataclass
import csv


@dataclass
class ValidationResult:
    scenario: str
    strategy: str
    completion_time: float
    collisions: int
    near_misses: int
    stops: int
    replans: int
    deadlocks: int
    deadlocks_resolved: int
    tasks_completed: int
    tasks_failed: int


class ValidationRunner:

    def __init__(self, factory):
        self.factory = factory

    def run(self, scenario_id, strategy):
        simulator, engine, runner = self.factory(
            scenario_id,
            strategy
        )

        metrics = runner.run()

        return ValidationResult(
            scenario=scenario_id,
            strategy=strategy,
            completion_time=metrics.completion_time,
            collisions=metrics.collisions,
            near_misses=metrics.near_misses,
            stops=metrics.stops,
            replans=metrics.replans,
            deadlocks=metrics.deadlocks_detected,
            deadlocks_resolved=metrics.deadlocks_resolved,
            tasks_completed=metrics.tasks_completed,
            tasks_failed=metrics.tasks_failed
        )

    def compare(self, scenario_id):
        return [
            self.run(
                scenario_id,
                "stop_and_wait"
            ),
            self.run(
                scenario_id,
                "arcnet"
            )
        ]

    def run_many(self, scenario_ids):
        results = []

        for scenario_id in scenario_ids:
            results.extend(
                self.compare(scenario_id)
            )

        return results


def write_results(results, path):
    if not results:
        return

    fields = [
        "scenario",
        "strategy",
        "completion_time",
        "collisions",
        "near_misses",
        "stops",
        "replans",
        "deadlocks",
        "deadlocks_resolved",
        "tasks_completed",
        "tasks_failed"
    ]

    with open(
        path,
        "w",
        newline="",
        encoding="utf-8"
    ) as f:

        writer = csv.DictWriter(
            f,
            fieldnames=fields
        )

        writer.writeheader()

        for result in results:
            writer.writerow({
                field: getattr(
                    result,
                    field
                )
                for field in fields
            })