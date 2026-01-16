"""IDESA_Comms_4.py

UDP comms for the demo build.

Packets:
  - Command (port 50001): (distance_mm, angle_deg) as 2 x float32, '<2f'
  - Status (default port 50002): (state_flag, estop_flag) as 2 x float32, '<2f'

Behaviour:
  - Both packets are sent at the same refresh rate (hz).
  - distance/angle are forced to 0,0 when sending_enabled is False.
  - the status packet is always sent.
"""

from __future__ import annotations

import socket
import struct
import time
from threading import Lock


class UDPComms4:
    def __init__(
        self,
        state,
        state_lock: Lock,
        pi_ip: str,
        cmd_port: int = 50001,
        status_port: int = 50002,
        hz: float = 2.0,
    ) -> None:
        self.state = state
        self.lock = state_lock
        self.pi_ip = str(pi_ip)
        self.cmd_port = int(cmd_port)
        self.status_port = int(status_port)
        self.period = 1.0 / max(float(hz), 1e-6)

        self._cmd_sock: socket.socket | None = None
        self._status_sock: socket.socket | None = None
        self._last_send_ts = 0.0

    def ensure_sockets(self) -> None:
        if self._cmd_sock is None:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setblocking(False)
            self._cmd_sock = s
        if self._status_sock is None:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setblocking(False)
            self._status_sock = s

    def shutdown(self) -> None:
        for attr in ("_cmd_sock", "_status_sock"):
            sock = getattr(self, attr)
            if sock is not None:
                try:
                    sock.close()
                except Exception:
                    pass
                setattr(self, attr, None)

    def tick(self, now: float) -> None:
        if (now - self._last_send_ts) < self.period:
            return
        self._last_send_ts = now

        with self.lock:
            # Encoder reset pulse: overrides command output for exactly one
            # comms-cycle. This is used to reset the Simulink/encoder
            # integrator when key UI buttons are pressed.
            reset_cycles = int(getattr(self.state, "reset_pulse_cycles", 0))
            reset_dist = float(getattr(self.state, "encoder_reset_distance_mm", 10.0))

            enabled = bool(getattr(self.state, "sending_enabled", False))
            if reset_cycles > 0:
                dist = reset_dist
                ang = 0.0
                # Consume exactly one cycle here (tied to the comms send rate).
                try:
                    self.state.reset_pulse_cycles = max(0, reset_cycles - 1)
                except Exception:
                    pass
            else:
                dist = float(getattr(self.state, "cmd_distance_mm", 0.0)) if enabled else 0.0
                ang = float(getattr(self.state, "cmd_angle_deg", 0.0)) if enabled else 0.0
            state_flag = float(getattr(self.state, "cmd_state_flag", 0.0))
            estop_pressed = bool(getattr(self.state, "estop_pressed", False))

            # Allow tuning what number gets sent (e.g., 1 or 255) without touching Simulink.
            estop_on_value = float(getattr(self.state, "estop_on_value", 1.0))
            estop_off_value = float(getattr(self.state, "estop_off_value", 0.0))
            estop_val = estop_on_value if estop_pressed else estop_off_value

        self.ensure_sockets()
        self._send_cmd(dist, ang)
        self._send_status(state_flag, estop_val)

    def send_zeros_burst(self, count: int = 4, spacing_s: float = 0.03) -> None:
        """Short burst helps Simulink see a clean stop."""
        count = max(1, int(count))
        spacing_s = max(0.0, float(spacing_s))
        for _ in range(count):
            with self.lock:
                state_flag = float(getattr(self.state, "cmd_state_flag", 0.0))
                estop_pressed = bool(getattr(self.state, "estop_pressed", False))
                estop_on_value = float(getattr(self.state, "estop_on_value", 1.0))
                estop_off_value = float(getattr(self.state, "estop_off_value", 0.0))
                estop_val = estop_on_value if estop_pressed else estop_off_value

            self.ensure_sockets()
            self._send_cmd(0.0, 0.0)
            self._send_status(state_flag, estop_val)
            time.sleep(spacing_s)

    def _send_cmd(self, dist_mm: float, ang_deg: float) -> None:
        payload = struct.pack("<2f", float(dist_mm), float(ang_deg))
        try:
            assert self._cmd_sock is not None
            self._cmd_sock.sendto(payload, (self.pi_ip, self.cmd_port))
        except Exception:
            pass

    def _send_status(self, state_flag: float, estop_val: float) -> None:
        payload = struct.pack("<2f", float(state_flag), float(estop_val))
        try:
            assert self._status_sock is not None
            self._status_sock.sendto(payload, (self.pi_ip, self.status_port))
        except Exception:
            pass
