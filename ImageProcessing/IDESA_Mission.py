"""Mission logic for sequencing multiple ArUco targets."""

from __future__ import annotations

import math
import time
from threading import Event, Lock, Thread
from typing import List, Optional, Sequence, Tuple

from IDESA_State import IDESAStateStore
from IDESA_Types import TargetTrack


class TargetQueueController:
    """Maintains a cyclic queue of target IDs and switches on proximity."""

    def __init__(self, state_store: IDESAStateStore) -> None:
        self._state_store = state_store
        self._lock = Lock()
        self._stop_event = Event()
        self._thread: Optional[Thread] = None

        self._selected_target_ids: Tuple[int, ...] = ()
        self._queue: List[int] = []
        self._switch_radius_mm: float = 100.0
        self._last_switch_time: float = 0.0
        self._last_target_coords: Tuple[float, float] = (0.0, 0.0)

        self._state_store.update(switch_radius_mm=self._switch_radius_mm)

    # ------------------------------------------------------------------
    # Configuration from GUI
    # ------------------------------------------------------------------
    def set_target_ids(self, target_ids: Sequence[int]) -> Tuple[int, ...]:
        cleaned = tuple(dict.fromkeys(int(tid) for tid in target_ids if tid is not None))
        if not (2 <= len(cleaned) <= 6):
            raise ValueError("Select between 2 and 6 target IDs.")
        with self._lock:
            self._selected_target_ids = cleaned
            self._queue = list(cleaned)
            self._last_switch_time = 0.0
            self._last_target_coords = (0.0, 0.0)
        self._state_store.update(selected_target_ids=cleaned, queue_order=cleaned)
        return cleaned

    def get_selected_target_ids(self) -> Tuple[int, ...]:
        with self._lock:
            return self._selected_target_ids

    def set_switch_radius(self, radius_mm: float) -> float:
        radius_mm = max(float(radius_mm), 1.0)
        with self._lock:
            self._switch_radius_mm = radius_mm
        self._state_store.update(switch_radius_mm=radius_mm)
        return radius_mm

    def get_switch_radius(self) -> float:
        with self._lock:
            return self._switch_radius_mm

    # ------------------------------------------------------------------
    # Thread lifecycle
    # ------------------------------------------------------------------
    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        with self._lock:
            if len(self._queue) < 2:
                raise ValueError("Configure at least two target IDs before starting.")
        self._stop_event.clear()
        self._thread = Thread(target=self._run, name="TargetQueueController", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        if not self._thread:
            return
        self._stop_event.set()
        self._thread.join(timeout=2.0)
        self._thread = None
        self._stop_event.clear()
        self._state_store.update(active_target_id=-1)

    def is_running(self) -> bool:
        thread = self._thread
        return bool(thread and thread.is_alive())

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _get_queue_snapshot(self) -> Tuple[int, ...]:
        with self._lock:
            return tuple(self._queue)

    def _rotate_queue(self) -> None:
        with self._lock:
            if self._queue:
                head = self._queue.pop(0)
                self._queue.append(head)

    def _run(self) -> None:
        while not self._stop_event.is_set():
            queue = self._get_queue_snapshot()
            if not queue:
                self._stop_event.wait(0.1)
                continue

            active_target_id = queue[0]
            selected_ids = self.get_selected_target_ids()
            switch_radius = self.get_switch_radius()
            snap = self._state_store.snapshot()
            target_track: Optional[TargetTrack] = snap.targets_by_id.get(active_target_id)

            if target_track:
                target_x = target_track.x_mm
                target_y = target_track.y_mm
                target_detected = target_track.detected
                self._last_target_coords = (target_x, target_y)
            else:
                target_x, target_y = self._last_target_coords
                target_detected = False

            robot_x = snap.robot_x_mm
            robot_y = snap.robot_y_mm
            distance = math.hypot(target_x - robot_x, target_y - robot_y)
            self._state_store.update(
                target_x_mm=target_x,
                target_y_mm=target_y,
                target_detected=target_detected,
                active_target_id=active_target_id,
                distance_to_active_mm=distance,
                queue_order=tuple(queue),
                selected_target_ids=selected_ids,
                switch_radius_mm=switch_radius,
            )

            should_rotate = (
                target_track is not None
                and target_track.detected
                and distance <= switch_radius
            )
            if should_rotate:
                now = time.time()
                if now - self._last_switch_time >= 1.0:
                    self._rotate_queue()
                    self._last_switch_time = now

            self._stop_event.wait(0.05)