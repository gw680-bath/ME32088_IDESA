"""Shared IDESA state container.

This module provides the `IDESAState` dataclass plus a small thread-safe store
so multiple threads (vision + UDP sender + GUI) can coordinate without race
conditions.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from threading import Lock
from typing import Dict, Any, Tuple

from IDESA_Types import TargetTrack


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
    active_target_id: int = -1
    distance_to_active_mm: float = 0.0
    switch_radius_mm: float = 50.0
    selected_target_ids: Tuple[int, ...] = ()
    queue_order: Tuple[int, ...] = ()
    targets_by_id: Dict[int, TargetTrack] = field(default_factory=dict)


class IDESAStateStore:
    """Thread-safe wrapper that owns the mutable IDESAState instance."""

    def __init__(self) -> None:
        self._state = IDESAState()
        self._lock = Lock()

    def update(self, **kwargs: Any) -> IDESAState:
        """Atomically update fields and return the new state snapshot."""
        safe_kwargs = self._sanitize_kwargs(kwargs)
        with self._lock:
            self._state = replace(self._state, **safe_kwargs)
            return self._state

    def snapshot(self) -> IDESAState:
        """Return a safe copy of the current state."""
        with self._lock:
            return self._clone_state(self._state)

    def as_dict(self) -> Dict[str, Any]:
        """Convenience helper for GUI rendering."""
        snap = self.snapshot()
        return snap.__dict__.copy()

    @staticmethod
    def _sanitize_kwargs(kwargs: Dict[str, Any]) -> Dict[str, Any]:
        safe: Dict[str, Any] = {}
        for key, value in kwargs.items():
            if key in {"selected_target_ids", "queue_order"}:
                safe[key] = tuple(int(v) for v in value) if value is not None else ()
            elif key == "targets_by_id" and isinstance(value, dict):
                safe[key] = {int(k): v for k, v in value.items() if isinstance(k, int)}
            else:
                safe[key] = value
        return safe

    @staticmethod
    def _clone_state(state: IDESAState) -> IDESAState:
        data = state.__dict__.copy()
        data["selected_target_ids"] = tuple(state.selected_target_ids)
        data["queue_order"] = tuple(state.queue_order)
        data["targets_by_id"] = {k: v for k, v in state.targets_by_id.items()}
        return IDESAState(**data)
