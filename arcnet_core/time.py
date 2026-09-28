from abc import ABC, abstractmethod
import time


class Clock(ABC):
    @abstractmethod
    def now(self):
        pass


class WallClock(Clock):
    def now(self):
        return time.time()