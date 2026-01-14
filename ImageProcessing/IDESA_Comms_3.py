"""
IDESA_Comms_3.py
UDP comms:
- Sends two float32: (distance_mm, angle_deg) as struct '<2f'
- Destination: Raspberry Pi 138.38.226.147:50001
- Sends only when sending_enabled=True; otherwise sends zeros (burst on stop/camera off)
"""

from __future__ import annotations

import socket
import struct
import time
from threading import Lock


class UDPComms3:
    def __init__(self, state, state_lock: Lock, pi_ip: str, pi_port: int, hz: float = 20.0) -> None:
        self.state = state
        self.lock = state_lock
        self.pi_ip = str(pi_ip)
        self.pi_port = int(pi_port)
        self.period = 1.0 / max(float(hz), 1.0)

        self._sock: socket.socket | None = None
        self._last_send = 0.0

    def ensure_socket(self) -> None:
        if self._sock is not None:
            return
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setblocking(False)
        self._sock = sock

    def shutdown(self) -> None:
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None

    def tick(self, now: float) -> None:
        # Rate limit
        if (now - self._last_send) < self.period:
            return
        self._last_send = now

        with self.lock:
            enabled = bool(getattr(self.state, "sending_enabled", False))
            dist = float(getattr(self.state, "cmd_distance_mm", 0.0)) if enabled else 0.0
            ang = float(getattr(self.state, "cmd_angle_deg", 0.0)) if enabled else 0.0

        self._send(dist, ang)

    def send_zeros_burst(self, count: int = 5, spacing_s: float = 0.05) -> None:
        # A short burst helps ensure Simulink sees the stop command
        for _ in range(max(1, int(count))):
            self._send(0.0, 0.0)
            time.sleep(max(0.0, float(spacing_s)))

    def _send(self, dist_mm: float, ang_deg: float) -> None:
        self.ensure_socket()
        payload = struct.pack("<2f", float(dist_mm), float(ang_deg))
        try:
            assert self._sock is not None
            self._sock.sendto(payload, (self.pi_ip, self.pi_port))
        except Exception:
            # Keep silent to avoid spamming; GUI shows behaviour via zeros anyway
            pass
