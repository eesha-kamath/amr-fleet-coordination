class LamportClock:
    def __init__(self):
        self.value = 0

    def tick(self):
        self.value += 1
        return self.value

    def receive(self, other):
        self.value = max(self.value, other) + 1
        return self.value