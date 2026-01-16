"""
IDESA_Comms_3.py
UDP comms:
- Sends three float32 (distance_mm, angle_deg, state_flag) as struct '<3f' to port 50001
- Sends two float32 (state_flag, estop_ok) as struct '<2f' to port 50003 every tick
- Distance/angle obey sending_enabled; state_flag and estop packet is always sent
"""

from __future__ import annotations

import socket
import struct
import time
from threading import Lock


class UDPComms3:
    def __init__(
        self,
        state,
        state_lock: Lock,
        pi_ip: str,
        pi_port: int,
        pi_state_port: int | None = 50003,
        hz: float = 20.0,
    ) -> None:
        self.state = state
        self.lock = state_lock
        self.pi_ip = str(pi_ip)
        self.pi_port = int(pi_port)
        self.pi_state_port = int(pi_state_port) if pi_state_port is not None else None
        self.period = 1.0 / max(float(hz), 1.0)

        self._sock: socket.socket | None = None
        self._state_sock: socket.socket | None = None
        self._last_send = 0.0

    def ensure_socket(self) -> None:
        if self._sock is not None:
            return
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setblocking(False)
        self._sock = sock

    def ensure_state_socket(self) -> None:
        if self.pi_state_port is None or self._state_sock is not None:
            return
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setblocking(False)
        self._state_sock = sock

    def shutdown(self) -> None:
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None
        if self._state_sock:
            try:
                self._state_sock.close()
            except Exception:
                pass
            self._state_sock = None

    def tick(self, now: float) -> None:
        # Rate limit
        if (now - self._last_send) < self.period:
            return
        self._last_send = now

        with self.lock:
            enabled = bool(getattr(self.state, "sending_enabled", False))
            dist = float(getattr(self.state, "cmd_distance_mm", 0.0)) if enabled else 0.0
            ang = float(getattr(self.state, "cmd_angle_deg", 0.0)) if enabled else 0.0
            flag = float(getattr(self.state, "cmd_state_flag", 0.0))
            estop_val = 1.0 if bool(getattr(self.state, "estop_ok", True)) else 0.0

        self._send_cmd(dist, ang, flag)
        self._send_state(flag, estop_val)

    def send_zeros_burst(self, count: int = 5, spacing_s: float = 0.05) -> None:
        # A short burst helps ensure Simulink sees the stop command
        for _ in range(max(1, int(count))):
            with self.lock:
                flag = float(getattr(self.state, "cmd_state_flag", 0.0))
                estop_val = 1.0 if bool(getattr(self.state, "estop_ok", True)) else 0.0
            self._send_cmd(0.0, 0.0, 0.0)
            self._send_state(flag, estop_val)
            time.sleep(max(0.0, float(spacing_s)))

    def _send_cmd(self, dist_mm: float, ang_deg: float, state_flag: float) -> None:
        self.ensure_socket()
        payload = struct.pack("<3f", float(dist_mm), float(ang_deg), float(state_flag))
        try:
            assert self._sock is not None
            self._sock.sendto(payload, (self.pi_ip, self.pi_port))
        except Exception:
            # Keep silent to avoid spamming; GUI shows behaviour via zeros anyway
            pass

    def _send_state(self, state_flag: float, estop_val: float) -> None:
        if self.pi_state_port is None:
            return
        self.ensure_state_socket()
        payload = struct.pack("<2f", float(state_flag), float(estop_val))
        try:
            assert self._state_sock is not None
            self._state_sock.sendto(payload, (self.pi_ip, self.pi_state_port))
        except Exception:
            pass
