class WaitForGraph:

    def __init__(self):
        self.waiting = {}

    def add_wait(self, robot_id, waiting_for):
        self.waiting[robot_id] = waiting_for

    def remove_wait(self, robot_id):
        self.waiting.pop(robot_id, None)

    def clear(self):
        self.waiting.clear()

    def has_deadlock(self):
        return self.get_cycle() is not None

    def get_cycle(self):
        for start in self.waiting:

            seen = set()
            cur = start

            while cur in self.waiting:

                if cur in seen:
                    cycle = []
                    node = cur

                    while True:
                        cycle.append(node)
                        node = self.waiting[node]

                        if node == cur:
                            break

                    return cycle

                seen.add(cur)
                cur = self.waiting[cur]

        return None

    def choose_victim(self, intents, arbiter):
        cycle = self.get_cycle()

        if not cycle:
            return None

        cycle_intents = [
            intent
            for intent in intents
            if intent.id in cycle
        ]

        if not cycle_intents:
            return None

        winner = arbiter.highest_priority(cycle_intents)

        if not winner:
            return None

        return next(
            intent
            for intent in cycle_intents
            if intent.id != winner.id
        )

    def dependencies(self):
        return dict(self.waiting)