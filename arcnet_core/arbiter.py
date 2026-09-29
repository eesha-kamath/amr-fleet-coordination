from .intent import RobotIntent


class ConflictArbiter:
    """Deterministic priority: aging first, then Lamport clock, then id.

    aging_step=None keeps the original behaviour (raw wait time compared).
    With aging_step set, waiting time counts in whole tiers. A robot that has
    waited another aging_step seconds moves up one tier, which prevents
    starvation without flipping priority on every tick.
    """

    def __init__(self, aging_step=None):
        self.aging_step = aging_step

    def aging_tier(self, wait_time):
        if not self.aging_step:
            return wait_time

        return int(wait_time // self.aging_step)

    def priority_key(self, intent: RobotIntent):
        """Smaller key means higher priority."""
        return (
            -self.aging_tier(intent.wait_time),
            intent.logical_clock,
            intent.id
        )

    def decide(self, a: RobotIntent, b: RobotIntent):
        return a if self.priority_key(a) <= self.priority_key(b) else b

    def compare(self, a: RobotIntent, b: RobotIntent):
        winner = self.decide(a, b)

        if winner.id == a.id:
            return 1

        return -1

    def highest_priority(self, intents):
        if not intents:
            return None

        return min(intents, key=self.priority_key)