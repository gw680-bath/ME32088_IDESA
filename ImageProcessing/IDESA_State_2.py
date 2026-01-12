"""Thread-safe state container for the parallel IDESA stack."""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from threading import Lock
from typing import Any, Dict, Tuple

from IDESA_Types_2 import NavState2, TargetTrack2


@dataclass(frozen=True)
class IDESAState2:
    """Snapshot of everything the Python controller cares about."""

    robot_x_mm: float = 0.0
    robot_y_mm: float = 0.0
    robot_yaw_deg: float = 0.0
    target_x_mm: float = 0.0
    target_y_mm: float = 0.0
    active_target_id: int = -1
    distance_error_mm: float = 0.0
    angle_error_deg: float = 0.0
    enable: float = 0.0
    python_state: NavState2 = NavState2.WAITING
    vision_lost: bool = True
    comms_lost: bool = True
    robot_detected: bool = False
    target_detected: bool = False
    last_udp_send_time: float = 0.0
    last_ack_time: float = 0.0
    last_sent_seq: int = 0
    last_ack_seq: int = 0
    last_rtt_ms: float = 0.0
    available_target_ids: Tuple[int, ...] = ()
    selected_target_ids: Tuple[int, ...] = ()
    queue_order: Tuple[int, ...] = ()
    switch_radius_mm: float = 0.0
    targets_by_id: Dict[int, TargetTrack2] = field(default_factory=dict)


class IDESAStateStore2:
    """Owns the mutable state and provides atomic updates."""

    def __init__(self) -> None:
        self._state = IDESAState2()
        self._lock = Lock()

    def update(self, **kwargs: Any) -> IDESAState2:
        safe_kwargs = self._sanitize(kwargs)
        with self._lock:
            self._state = replace(self._state, **safe_kwargs)
            return self._state

    def snapshot(self) -> IDESAState2:
        with self._lock:
            return self._clone(self._state)

    def as_dict(self) -> Dict[str, Any]:
        snap = self.snapshot()
        result: Dict[str, Any] = snap.__dict__.copy()
        result["python_state"] = snap.python_state.value
        return result

    @staticmethod
    def _sanitize(kwargs: Dict[str, Any]) -> Dict[str, Any]:
        safe: Dict[str, Any] = {}
        for key, value in kwargs.items():
            if key in {"available_target_ids", "selected_target_ids", "queue_order"}:
                safe[key] = tuple(int(v) for v in value) if value is not None else ()
            elif key == "targets_by_id" and isinstance(value, dict):
                safe[key] = {
                    int(k): v if isinstance(v, TargetTrack2) else TargetTrack2()
                    for k, v in value.items()
                }
            elif key == "python_state" and isinstance(value, str):
                safe[key] = NavState2(value)
            else:
                safe[key] = value
        return safe

    @staticmethod
    def _clone(state: IDESAState2) -> IDESAState2:
        data = state.__dict__.copy()
        data["available_target_ids"] = tuple(state.available_target_ids)
        data["selected_target_ids"] = tuple(state.selected_target_ids)
        data["queue_order"] = tuple(state.queue_order)
        data["switch_radius_mm"] = state.switch_radius_mm
        data["targets_by_id"] = dict(state.targets_by_id)
        data["python_state"] = state.python_state
        return IDESAState2(**data)
