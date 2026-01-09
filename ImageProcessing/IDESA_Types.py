"""Shared lightweight dataclasses for IDESA modules."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class TargetTrack:
    """Last-known information for a single ArUco target marker."""

    x_mm: float = 0.0
    y_mm: float = 0.0
    detected: bool = False
    last_seen_time: float = 0.0
