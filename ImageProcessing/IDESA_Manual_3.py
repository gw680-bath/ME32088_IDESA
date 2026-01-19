"""
IDESA_Manual_3.py
Manual arrow-key controller (one-shot pulses):
- Up: distance=200mm, angle=0
- Left: distance=0, angle=+90
- Right: distance=0, angle=-90

After a key press, the command is sent ONCE and then returns to zeros
because Main clears manual_pulse_pending after consuming it.
"""

from __future__ import annotations

import time
import tkinter as tk
from threading import Lock


class ManualController3:
    def __init__(self, state, state_lock: Lock) -> None:
        self.state = state
        self.lock = state_lock

    def bind_to_tk(self, root: tk.Tk) -> None:
        # Bind keys globally
        root.bind("<Up>", self._on_up)
        root.bind("<Left>", self._on_left)
        root.bind("<Right>", self._on_right)

    def _arm_pulse(self, dist_mm: float, ang_deg: float) -> None:
        with self.lock:
            # Only generate pulses when in MANUAL mode
            if str(getattr(self.state, "control_mode", "AUTO")).upper() != "MANUAL":
                return
            # Only meaningful when Start is enabled (otherwise ignored anyway)
            if not bool(getattr(self.state, "sending_enabled", False)):
                return
            self.state.manual_pulse_distance_mm = float(dist_mm)
            self.state.manual_pulse_angle_deg = float(ang_deg)
            self.state.manual_pulse_pending = True
            now = time.time()
            self.state.last_manual_distance_mm = float(dist_mm)
            self.state.last_manual_angle_deg = float(ang_deg)
            self.state.last_manual_timestamp = now

    def _on_up(self, _event) -> None:
        self._arm_pulse(2000.0, 0.0)

    def _on_left(self, _event) -> None:
        self._arm_pulse(10.0, -90.0)

    def _on_right(self, _event) -> None:
        self._arm_pulse(10.0, +90.0)
