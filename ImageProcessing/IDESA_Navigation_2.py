"""Navigation state machine for the parallel IDESA stack."""

from __future__ import annotations

import math
import time
from threading import Event, Lock, Thread

from IDESA_State_2 import IDESAStateStore2
from IDESA_Types_2 import NavCommand2, NavState2


def _wrap_to_180(angle_deg: float) -> float:
    wrapped = (angle_deg + 180.0) % 360.0 - 180.0
    if wrapped == -180.0:
        return 180.0
    return wrapped


class NavigationController2:
    """Computes distance/angle errors and owns the nav state machine."""

    def __init__(
        self,
        state_store: IDESAStateStore2,
        angle_tol_deg: float = 5.0,
        dist_tol_mm: float = 100.0,
        update_hz: float = 30.0,
    ) -> None:
        self._state_store = state_store
        self._angle_tol_deg = max(float(angle_tol_deg), 0.1)
        self._dist_tol_mm = max(float(dist_tol_mm), 1.0)
        self._update_period = 1.0 / max(float(update_hz), 1.0)

        self._current_state = NavState2.WAITING
        self._last_target_id = -999

        self._stop_event = Event()
        self._thread: Thread | None = None
        self._command_lock = Lock()
        self._latest_command = NavCommand2(0.0, 0.0, 0.0, seq=0, timestamp=time.time())
        self._robot_stop = Event()
        now = time.time()
        self._last_robot_seen = now
        self._last_target_seen = now
        self._loss_timeout_s = 2.0

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = Thread(target=self._loop, name="NavigationController2", daemon=True)
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

    def get_command(self) -> NavCommand2:
        with self._command_lock:
            return self._latest_command

    def engage_robot_stop(self) -> None:
        """Force outbound commands to zero without stopping other subsystems."""

        self._robot_stop.set()
        self._publish_manual_stop_state()

    def release_robot_stop(self) -> None:
        self._robot_stop.clear()

    def is_robot_stop_active(self) -> bool:
        return self._robot_stop.is_set()

    # ------------------------------------------------------------------
    # Internal logic
    # ------------------------------------------------------------------
    def _loop(self) -> None:
        while not self._stop_event.is_set():
            snap = self._state_store.snapshot()
            command = self._compute_command(snap)
            with self._command_lock:
                self._latest_command = command
            self._stop_event.wait(self._update_period)

    def _compute_command(self, snap) -> NavCommand2:
        dx = snap.target_x_mm - snap.robot_x_mm
        dy = snap.target_y_mm - snap.robot_y_mm
        distance = math.hypot(dx, dy)
        desired_heading = math.degrees(math.atan2(dx, dy)) if dx or dy else 0.0
        angle_error = _wrap_to_180(desired_heading - snap.robot_yaw_deg)

        now = time.time()
        if snap.robot_detected:
            self._last_robot_seen = now
        if snap.target_detected:
            self._last_target_seen = now
        robot_stale = (now - self._last_robot_seen) >= self._loss_timeout_s
        target_stale = (now - self._last_target_seen) >= self._loss_timeout_s
        simultaneous_loss = robot_stale and target_stale
        vision_issue = snap.vision_lost or not snap.robot_detected or not snap.target_detected
        comms_issue = snap.comms_lost

        state = self._current_state
        if snap.active_target_id != self._last_target_id:
            self._last_target_id = snap.active_target_id
            if not vision_issue and not comms_issue:
                state = NavState2.ALIGNING

        if vision_issue or comms_issue:
            state = NavState2.WAITING
        elif state == NavState2.WAITING and snap.target_detected:
            state = NavState2.ALIGNING
        elif state == NavState2.ALIGNING and abs(angle_error) < self._angle_tol_deg:
            state = NavState2.TRAVELING
        elif state == NavState2.TRAVELING and distance < self._dist_tol_mm:
            state = NavState2.WAITING

        if simultaneous_loss:
            state = NavState2.WAITING
            distance = 0.0
            angle_error = 0.0

        manual_stop_active = self._robot_stop.is_set()
        if manual_stop_active:
            state = NavState2.WAITING
            distance = 0.0
            angle_error = 0.0

        enable = 1.0 if state != NavState2.WAITING else 0.0
        if manual_stop_active:
            enable = 0.0

        self._current_state = state
        self._state_store.update(
            distance_error_mm=distance,
            angle_error_deg=angle_error,
            enable=enable,
            python_state=state,
        )

        return NavCommand2(
            distance_error_mm=distance,
            angle_error_deg=angle_error,
            enable=enable,
            seq=0,
            timestamp=now,
        )

    def _publish_manual_stop_state(self) -> None:
        now = time.time()
        zero_command = NavCommand2(0.0, 0.0, 0.0, seq=0, timestamp=now)
        self._state_store.update(
            distance_error_mm=0.0,
            angle_error_deg=0.0,
            enable=0.0,
            python_state=NavState2.WAITING,
        )
        with self._command_lock:
            self._latest_command = zero_command
