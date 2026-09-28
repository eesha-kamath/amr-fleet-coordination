from dataclasses import dataclass


@dataclass
class SafetyConfig:
    reaction_latency: float = 0.1
    margin: float = 0.1


@dataclass
class LivenessConfig:
    warning_timeout: float = 1.0
    failure_timeout: float = 3.0
    margin: float = 0.1


@dataclass
class NetworkConfig:
    cell_size: float = 10.0
    heartbeat_frequency: float = 1.0
    local_frequency: float = 10.0


@dataclass
class ArcnetConfig:
    safety: SafetyConfig
    liveness: LivenessConfig
    network: NetworkConfig