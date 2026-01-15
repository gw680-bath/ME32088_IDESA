"""Mission controller for the parallel IDESA navigation stack."""

from __future__ import annotations

import math
import time
from threading import Event, Lock, Thread
from typing import Sequence, Tuple

from IDESA_State_2 import IDESAStateStore2


class TargetQueueController2:
    """Maintains a cyclic list of target IDs and exposes the active target."""

    def __init__(
        self,
        state_store: IDESAStateStore2,
        switch_radius_mm: float = 150.0,
        available_target_ids: Sequence[int] | None = None,
    ) -> None:
        self._state_store = state_store
        self._lock = Lock()
        self._stop_event = Event()
        self._thread: Thread | None = None

        self._selected_target_ids: Tuple[int, ...] = ()
        self._queue: list[int] = []
        self._switch_radius_mm = max(float(switch_radius_mm), 1.0)
        self._last_switch_time = 0.0

        self._available_target_ids: Tuple[int, ...] = self._sanitize_ids(available_target_ids) or tuple(range(2, 8))

        self._state_store.update(
            switch_radius_mm=self._switch_radius_mm,
            available_target_ids=self._available_target_ids,
        )

    # ------------------------------------------------------------------
    # Configuration API
    # ------------------------------------------------------------------
    def set_target_ids(self, target_ids: Sequence[int]) -> Tuple[int, ...]:
        cleaned = tuple(dict.fromkeys(int(tid) for tid in target_ids if tid is not None))
        if len(cleaned) < 1:
            raise ValueError("Provide at least one target ID.")
        invalid = tuple(tid for tid in cleaned if tid not in self._available_target_ids)
        if invalid:
            raise ValueError(
                f"Target IDs {invalid} are not in the available set {self._available_target_ids}."
            )
        with self._lock:
            self._selected_target_ids = cleaned
            self._queue = list(cleaned)
            self._last_switch_time = 0.0
        self._state_store.update(
            available_target_ids=self._available_target_ids,
            selected_target_ids=cleaned,
            queue_order=cleaned,
        )
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
            if not self._queue:
                raise ValueError("Call set_target_ids before starting the mission thread.")
        self._stop_event.clear()
        self._thread = Thread(target=self._run, name="TargetQueueController2", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        if not self._thread:
            return
        self._stop_event.set()
        self._thread.join(timeout=2.0)
        self._thread = None
        self._stop_event.clear()

    def is_running(self) -> bool:
        thread = self._thread
        return bool(thread and thread.is_alive())

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _snapshot_queue(self) -> Tuple[int, ...]:
        with self._lock:
            return tuple(self._queue)

    def _rotate(self) -> None:
        with self._lock:
            if not self._queue:
                return
            head = self._queue.pop(0)
            self._queue.append(head)

    @staticmethod
    def _sanitize_ids(target_ids: Sequence[int] | None) -> Tuple[int, ...]:
        if not target_ids:
            return ()
        return tuple(dict.fromkeys(int(tid) for tid in target_ids if tid is not None))

    def _run(self) -> None:
        while not self._stop_event.is_set():
            queue = self._snapshot_queue()
            if not queue:
                self._stop_event.wait(0.25)
                continue

            active_target_id = queue[0]
            snap = self._state_store.snapshot()
            track = snap.targets_by_id.get(active_target_id)
            target_detected = bool(track and track.detected)
            target_x = track.x_mm if track else snap.target_x_mm
            target_y = track.y_mm if track else snap.target_y_mm

            robot_x = snap.robot_x_mm
            robot_y = snap.robot_y_mm
            distance = math.hypot(target_x - robot_x, target_y - robot_y)

            self._state_store.update(
                active_target_id=active_target_id,
                target_x_mm=target_x,
                target_y_mm=target_y,
                target_detected=target_detected,
                queue_order=tuple(queue),
                selected_target_ids=self.get_selected_target_ids(),
            )

            should_rotate = target_detected and distance <= self.get_switch_radius()
            if should_rotate:
                now = time.time()
                if now - self._last_switch_time >= 1.0:
                    self._rotate()
                    self._last_switch_time = now

            self._stop_event.wait(0.05)
