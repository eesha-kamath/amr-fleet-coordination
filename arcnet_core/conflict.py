from dataclasses import dataclass


@dataclass
class Conflict:
    robot_a: str
    robot_b: str
    position: tuple[int, int]
    time: int
    kind: str


class ConflictDetector:

    def detect(self, robot_id, path, peer_paths):
        conflicts = []

        for peer_id, peer_path in peer_paths.items():

            if peer_id == robot_id:
                continue

            n = min(len(path), len(peer_path))

            for t in range(n):

                if path[t] == peer_path[t]:
                    conflicts.append(
                        Conflict(
                            robot_id,
                            peer_id,
                            path[t],
                            t,
                            "vertex"
                        )
                    )

                if t > 0:
                    if (
                        path[t] == peer_path[t - 1]
                        and
                        path[t - 1] == peer_path[t]
                    ):
                        conflicts.append(
                            Conflict(
                                robot_id,
                                peer_id,
                                path[t],
                                t,
                                "edge"
                            )
                        )

        return conflicts

    def has_conflict(self, robot_id, path, peer_paths):
        return bool(
            self.detect(
                robot_id,
                path,
                peer_paths
            )
        )