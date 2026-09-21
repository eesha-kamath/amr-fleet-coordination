"""
lamport_clock.py
Minimal Lamport logical clock (Lamport, 1978). Gives every robot a
deterministic, total ordering of events across the fleet without
needing synchronized real-world clocks.
"""

class LamportClock:
    def __init__(self):
        self.time = 0

    def increment(self):
        self.time += 1
        return self.time

    def update(self, received_time):
        self.time = max(self.time, received_time) + 1
        return self.time
