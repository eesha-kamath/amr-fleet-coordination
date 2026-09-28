from math import sqrt


class StopAndWaitBaseline:

    def __init__(self, distance_threshold):
        self.distance_threshold = distance_threshold

    def command(
        self,
        robot,
        vx,
        vy,
        peers
    ):
        for peer in peers:

            dx = peer.x - robot.x
            dy = peer.y - robot.y

            d = sqrt(
                dx * dx +
                dy * dy
            )

            if d <= self.distance_threshold:
                return 0.0, 0.0

        return vx, vy