from dataclasses import dataclass
from enum import Enum
from .config import LivenessConfig


class PeerStatus(Enum):
    UNKNOWN = "unknown"
    ACTIVE = "active"
    LEFT_NEIGHBORHOOD = "left_neighborhood"
    SILENT_WARNING = "silent_warning"
    OUT_OF_RANGE = "out_of_range"
    PRESUMED_FAILED = "presumed_failed"


@dataclass
class Peer:
    id: str
    x: float
    y: float
    vx: float
    vy: float
    theta: float
    last_local_broadcast: float
    last_global_heartbeat: float
    max_speed: float = 0.0
    status: PeerStatus = PeerStatus.UNKNOWN


class PeerManager:
    def __init__(self, config):
        self.config = config
        self.peers = {}

    def add(self, peer):
        self.peers[peer.id] = peer

    def update_state(
        self,
        robot_id,
        x,
        y,
        vx,
        vy,
        theta,
        now,
        local=True,
        global_heartbeat=True
    ):
        peer = self.peers.get(robot_id)

        if not peer:
            peer = Peer(
                robot_id,
                x,
                y,
                vx,
                vy,
                theta,
                now,
                now,
                0.0,
                PeerStatus.ACTIVE
            )
            self.peers[robot_id] = peer
        else:
            peer.x = x
            peer.y = y
            peer.vx = vx
            peer.vy = vy
            peer.theta = theta

            if local:
                peer.last_local_broadcast = now

            if global_heartbeat:
                peer.last_global_heartbeat = now

            peer.status = PeerStatus.ACTIVE

        return peer

    def get(self, robot_id):
        return self.peers.get(robot_id)

    def remove(self, robot_id):
        self.peers.pop(robot_id, None)

    def uncertainty_radius(self, robot_id, now):
        peer = self.peers.get(robot_id)

        if not peer:
            return 0.0

        silence = max(
            0.0,
            now - peer.last_local_broadcast
        )

        return silence * peer.max_speed + self.config.margin

    def update(
        self,
        robot_id,
        now,
        local_visible,
        global_visible
    ):
        peer = self.peers.get(robot_id)

        if not peer:
            return None

        local_age = now - peer.last_local_broadcast
        global_age = now - peer.last_global_heartbeat

        if local_visible:
            peer.status = PeerStatus.ACTIVE
        elif global_visible:
            peer.status = PeerStatus.LEFT_NEIGHBORHOOD
        elif (
            local_age > self.config.failure_timeout
            and global_age > self.config.failure_timeout
        ):
            peer.status = PeerStatus.PRESUMED_FAILED
        elif local_age > self.config.warning_timeout:
            peer.status = PeerStatus.SILENT_WARNING

        return peer.status

    def update_all(self, now, local_visible):
        for robot_id in self.peers:
            self.update(
                robot_id,
                now,
                robot_id in local_visible,
                True
            )

    def active_peers(self):
        return [
            p for p in self.peers.values()
            if p.status == PeerStatus.ACTIVE
        ]

    def failed_peers(self):
        return [
            p for p in self.peers.values()
            if p.status == PeerStatus.PRESUMED_FAILED
        ]