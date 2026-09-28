from .intent import RobotIntent


class ConflictArbiter:

    def decide(self, a: RobotIntent, b: RobotIntent):

        if a.wait_time != b.wait_time:
            return a if a.wait_time > b.wait_time else b

        if a.logical_clock != b.logical_clock:
            return a if a.logical_clock < b.logical_clock else b

        return a if a.id < b.id else b

    def compare(self, a: RobotIntent, b: RobotIntent):
        winner = self.decide(a, b)

        if winner.id == a.id:
            return 1

        return -1

    def highest_priority(self, intents):
        if not intents:
            return None

        winner = intents[0]

        for intent in intents[1:]:
            winner = self.decide(winner, intent)

        return winner