"""Shared IDESA state container.

This module provides the `IDESAState` dataclass plus a small thread-safe store
so multiple threads (vision + UDP sender + GUI) can coordinate without race
conditions.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from threading import Lock
from typing import Dict, Any


@dataclass(frozen=True)
class IDESAState:
    robot_x_mm: float = 0.0
    robot_y_mm: float = 0.0
    robot_yaw_deg: float = 0.0
    target_x_mm: float = 0.0
    target_y_mm: float = 0.0
    robot_detected: bool = False
    target_detected: bool = False
    fps: float = 0.0
    last_update_time: float = 0.0


class IDESAStateStore:
    """Thread-safe wrapper that owns the mutable IDESAState instance."""

    def __init__(self) -> None:
        self._state = IDESAState()
        self._lock = Lock()

    def update(self, **kwargs: Any) -> IDESAState:
        """Atomically update fields and return the new state snapshot."""
        with self._lock:
            self._state = replace(self._state, **kwargs)
            return self._state

    def snapshot(self) -> IDESAState:
        """Return a safe copy of the current state."""
        with self._lock:
            return IDESAState(**self._state.__dict__)

    def as_dict(self) -> Dict[str, Any]:
        """Convenience helper for GUI rendering."""
        snap = self.snapshot()
        return snap.__dict__.copy()
