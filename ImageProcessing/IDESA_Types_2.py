"""Shared types for the parallel IDESA navigation stack."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class NavState2(str, Enum):
    """Python-side navigation state machine."""

    ALIGNING = "ALIGNING"
    TRAVELING = "TRAVELING"
    WAITING = "WAITING"


@dataclass(frozen=True)
class NavCommand2:
    """Single outbound navigation command destined for Simulink."""

    distance_error_mm: float
    angle_error_deg: float
    enable: float
    seq: int = 0
    timestamp: float = 0.0


@dataclass(frozen=True)
class NavAck2:
    """Inbound ACK metadata from Simulink."""

    seq: int
    received_time: float
    rtt_ms: float


@dataclass(frozen=True)
class TargetTrack2:
    """Vision result for an individual ArUco marker."""

    x_mm: float = 0.0
    y_mm: float = 0.0
    detected: bool = False
    last_seen_time: float = 0.0
