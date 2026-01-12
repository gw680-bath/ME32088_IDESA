"""Minimal telemetry GUI for the parallel IDESA control stack."""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk
from typing import Dict

from IDESA_State_2 import IDESAStateStore2

DISPLAY_FIELDS = (
    ("Python State", "python_state"),
    ("Distance Error [mm]", "distance_error_mm"),
    ("Angle Error [deg]", "angle_error_deg"),
    ("Enable", "enable"),
    ("Last RTT [ms]", "last_rtt_ms"),
    ("Comms Lost", "comms_lost"),
    ("Vision Lost", "vision_lost"),
)


class IDESAGuiApp2(tk.Tk):
    """Lightweight dashboard that polls IDESAStateStore2."""

    def __init__(self, state_store: IDESAStateStore2, poll_ms: int = 250) -> None:
        super().__init__()
        self._state_store = state_store
        self._poll_ms = max(int(poll_ms), 50)

        self.title("IDESA Mission Control 2")
        self.geometry("420x260")
        self.resizable(False, False)

        container = ttk.Frame(self, padding=12)
        container.pack(fill=tk.BOTH, expand=True)

        self._labels: Dict[str, ttk.Label] = {}
        for row, (label_text, key) in enumerate(DISPLAY_FIELDS):
            ttk.Label(container, text=label_text, font=("Space Grotesk", 11, "bold")).grid(
                row=row, column=0, sticky=tk.W, pady=4
            )
            value_label = ttk.Label(container, text="-", font=("Space Mono", 11))
            value_label.grid(row=row, column=1, sticky=tk.E, pady=4)
            self._labels[key] = value_label

        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(self._poll_ms, self._refresh)

    def _refresh(self) -> None:
        snap = self._state_store.snapshot()
        values = {
            "python_state": snap.python_state.value,
            "distance_error_mm": f"{snap.distance_error_mm:8.1f}",
            "angle_error_deg": f"{snap.angle_error_deg:8.1f}",
            "enable": f"{snap.enable:4.1f}",
            "last_rtt_ms": f"{snap.last_rtt_ms:6.1f}",
            "comms_lost": "YES" if snap.comms_lost else "NO",
            "vision_lost": "YES" if snap.vision_lost else "NO",
        }
        for key, label in self._labels.items():
            label.configure(text=values.get(key, "-"))
        self.after(self._poll_ms, self._refresh)

    def run(self) -> None:
        self.mainloop()

    def _on_close(self) -> None:
        self.destroy()


def create_gui(state_store: IDESAStateStore2) -> IDESAGuiApp2:
    """Factory used by IDESA_Main_2."""
    return IDESAGuiApp2(state_store)
