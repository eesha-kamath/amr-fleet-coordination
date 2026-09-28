from abc import ABC, abstractmethod


class CoordinationStrategy(ABC):

    @abstractmethod
    def plan(self, state, peers):
        pass

    @abstractmethod
    def decide_motion(self, state, peers):
        pass


class StopAndWait(CoordinationStrategy):

    def plan(self, state, peers):
        return []

    def decide_motion(self, state, peers):
        if peers:
            return {
                "vx": 0.0,
                "vy": 0.0,
                "stop": True
            }

        return {
            "vx": state.get("vx", 0.0),
            "vy": state.get("vy", 0.0),
            "stop": False
        }


class PriorityOnly(CoordinationStrategy):

    def plan(self, state, peers):
        return state.get("path", [])

    def decide_motion(self, state, peers):
        if not peers:
            return {
                "vx": state.get("vx", 0.0),
                "vy": state.get("vy", 0.0),
                "stop": False
            }

        my_id = state.get("id", "")

        for peer in peers:
            if peer.get("id", "") < my_id:
                return {
                    "vx": 0.0,
                    "vy": 0.0,
                    "stop": True
                }

        return {
            "vx": state.get("vx", 0.0),
            "vy": state.get("vy", 0.0),
            "stop": False
        }


class ArcnetFull(CoordinationStrategy):

    def plan(self, state, peers):
        return state.get("path", [])

    def decide_motion(self, state, peers):
        return {
            "vx": state.get("vx", 0.0),
            "vy": state.get("vy", 0.0),
            "stop": state.get("stop", False)
        }


STRATEGIES = {
    "stop_and_wait": StopAndWait,
    "priority_only": PriorityOnly,
    "arcnet": ArcnetFull
}