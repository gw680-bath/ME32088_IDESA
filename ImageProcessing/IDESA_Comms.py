"""UDP communications helpers for the modular IDESA stack."""

from __future__ import annotations

import socket
import struct
import threading
import time
from typing import Callable, Optional

from IDESA_State import IDESAState


def pack_5floats(robot_x: float, robot_y: float, robot_yaw_deg: float,
                 target_x: float, target_y: float) -> bytes:
    """Pack the five-float payload exactly like the legacy script."""
    return struct.pack(
        "<5f",
        float(robot_x),
        float(robot_y),
        float(robot_yaw_deg),
        float(target_x),
        float(target_y),
    )


class UDPSender:
    """Background thread that repeatedly sends IDESA state snapshots."""

    def __init__(self, ip: str, port: int) -> None:
        if not ip:
            raise ValueError("UDP sender requires a destination IP address.")
        self._addr = (ip, int(port))
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        self._state_supplier: Optional[Callable[[], IDESAState]] = None
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._send_period = 1.0 / 30.0
        self._lock = threading.Lock()

    def start(self, state_supplier: Callable[[], IDESAState], send_hz: float) -> None:
        """Launch the sender thread (idempotent)."""
        if state_supplier is None:
            raise ValueError("state_supplier cannot be None")

        with self._lock:
            self._state_supplier = state_supplier
            self._send_period = 1.0 / max(send_hz, 1e-6)
            if self._thread and self._thread.is_alive():
                return

            self._stop_event.clear()
            self._thread = threading.Thread(target=self._run, name="UDPSender", daemon=True)
            self._thread.start()

    def stop(self) -> None:
        with self._lock:
            if not self._thread:
                return
            self._stop_event.set()
            self._thread.join(timeout=2.0)
            self._thread = None
            self._stop_event.clear()

    def close(self) -> None:
        self.stop()
        try:
            self._sock.close()
        except OSError:
            pass

    def is_running(self) -> bool:
        thread = self._thread
        return bool(thread and thread.is_alive())

    def _run(self) -> None:
        next_send_ts = time.time()
        while not self._stop_event.is_set():
            supplier = self._state_supplier
            state: Optional[IDESAState] = None
            if supplier:
                try:
                    state = supplier()
                except Exception:
                    # Supplier exceptions should not kill the sender loop.
                    state = None

            if state is not None:
                payload = pack_5floats(
                    state.robot_x_mm,
                    state.robot_y_mm,
                    state.robot_yaw_deg,
                    state.target_x_mm,
                    state.target_y_mm,
                )
                try:
                    self._sock.sendto(payload, self._addr)
                except OSError:
                    # Ignore transient socket errors; the loop keeps running.
                    pass

            next_send_ts += self._send_period
            sleep_duration = max(0.0, next_send_ts - time.time())
            if sleep_duration == 0.0:
                next_send_ts = time.time() + self._send_period
                sleep_duration = self._send_period
            self._stop_event.wait(timeout=sleep_duration)

    def __del__(self) -> None:
        self.close()
