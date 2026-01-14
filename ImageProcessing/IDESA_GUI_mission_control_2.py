"""Minimal telemetry GUI for the parallel IDESA control stack."""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk
from typing import Callable, Dict, Sequence, Tuple

from IDESA_Mission_2 import TargetQueueController2
from IDESA_State_2 import IDESAStateStore2

MIN_TARGETS = 2
MAX_TARGETS = 6

DISPLAY_FIELDS = (
    ("Python State", "python_state"),
    ("Current Target", "active_target_id"),
    ("Distance Error [mm]", "distance_error_mm"),
    ("Angle Error [deg]", "angle_error_deg"),
    ("Enable", "enable"),
    ("Last RTT [ms]", "last_rtt_ms"),
    ("Comms Lost", "comms_lost"),
    ("Vision Lost", "vision_lost"),
)


class IDESAGuiApp2(tk.Tk):
    """Lightweight dashboard that polls IDESAStateStore2."""

    def __init__(
        self,
        state_store: IDESAStateStore2,
        mission: TargetQueueController2,
        start_callback: Callable[[], None],
        robot_stop_callback: Callable[[], None],
        stop_callback: Callable[[], None],
        poll_ms: int = 250,
    ) -> None:
        super().__init__()
        self._state_store = state_store
        self._mission = mission
        self._start_cb = start_callback
        self._robot_stop_cb = robot_stop_callback
        self._stop_cb = stop_callback
        self._poll_ms = max(int(poll_ms), 50)

        self.title("IDESA Mission Control 2")
        self.geometry("1920, 1080")
        self.resizable(False, False)

        container = ttk.Frame(self, padding=12)
        container.pack(fill=tk.BOTH, expand=True)

        button_frame = ttk.Frame(container)
        button_frame.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 12))
        ttk.Button(button_frame, text="Start", command=self._on_start).pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(button_frame, text="Robot Stop", command=self._on_robot_stop).pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(button_frame, text="Shutdown", command=self._on_shutdown).pack(side=tk.LEFT)

        targets_frame = ttk.LabelFrame(container, text=f"Target Selection ({MIN_TARGETS}-{MAX_TARGETS})")
        targets_frame.grid(row=1, column=0, sticky="nsew", padx=(0, 12))
        self._targets_container = ttk.Frame(targets_frame)
        self._targets_container.pack(fill=tk.BOTH, expand=True, pady=6)
        self._target_vars: Dict[int, tk.BooleanVar] = {}
        self._target_boxes: Dict[int, ttk.Checkbutton] = {}
        self._selection_blocked = False
        snap = self._state_store.snapshot()
        self._known_target_ids: Tuple[int, ...] = (
            tuple(snap.available_target_ids)
            or tuple(snap.selected_target_ids)
            or tuple(range(2, 8))
        )
        self._build_target_controls(self._known_target_ids, snap.selected_target_ids)

        self._radius_blocked = False
        self._switch_var = tk.DoubleVar(value=max(float(snap.switch_radius_mm or 100.0), 1.0))
        radius_frame = ttk.Frame(targets_frame)
        radius_frame.pack(fill=tk.X, pady=(8, 0))
        ttk.Label(radius_frame, text="Switch radius [mm]").pack(anchor="w")
        self._radius_spin = tk.Spinbox(
            radius_frame,
            from_=10,
            to=5000,
            increment=10,
            textvariable=self._switch_var,
            width=8,
            command=self._on_switch_radius_change,
        )
        self._switch_var.trace_add("write", lambda *_: self._on_switch_radius_change())
        self._radius_spin.pack(anchor="w", pady=(2, 0))

        stats_frame = ttk.LabelFrame(container, text="Live Telemetry")
        stats_frame.grid(row=1, column=1, sticky="nsew")

        self._labels: Dict[str, ttk.Label] = {}
        for row, (label_text, key) in enumerate(DISPLAY_FIELDS):
            ttk.Label(stats_frame, text=label_text, font=("Space Grotesk", 11, "bold")).grid(
                row=row, column=0, sticky=tk.W, pady=4
            )
            value_label = ttk.Label(stats_frame, text="-", font=("Space Mono", 11))
            value_label.grid(row=row, column=1, sticky=tk.E, pady=4)
            self._labels[key] = value_label

        container.columnconfigure(0, weight=1)
        container.columnconfigure(1, weight=1)

        self._status_var = tk.StringVar(value="Idle")
        status_label = ttk.Label(container, textvariable=self._status_var, anchor="w")
        status_label.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(12, 0))

        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(self._poll_ms, self._refresh)

    # ------------------------------------------------------------------
    # Button callbacks
    # ------------------------------------------------------------------
    def _on_start(self) -> None:
        try:
            self._start_cb()
        except Exception as exc:
            self._status_var.set(f"Start failed: {exc}")
        else:
            self._status_var.set("Subsystems running")

    def _on_robot_stop(self) -> None:
        try:
            self._robot_stop_cb()
        except Exception as exc:
            self._status_var.set(f"Robot stop failed: {exc}")
        else:
            self._status_var.set("Robot halted; vision/comms active")

    def _on_shutdown(self) -> None:
        try:
            self._stop_cb()
        except Exception as exc:
            self._status_var.set(f"Shutdown failed: {exc}")
        else:
            self._status_var.set("Subsystems stopped")

    # ------------------------------------------------------------------
    # Target selection helpers
    # ------------------------------------------------------------------
    def _build_target_controls(self, target_ids: Sequence[int], selected: Sequence[int]) -> None:
        for child in self._targets_container.winfo_children():
            child.destroy()
        self._target_vars.clear()
        self._target_boxes.clear()
        for tid in target_ids:
            var = tk.BooleanVar(value=tid in selected)
            chk = ttk.Checkbutton(
                self._targets_container,
                text=f"ID {tid}",
                variable=var,
                command=lambda tid=tid: self._on_target_toggle(tid),
            )
            chk.pack(anchor="w", pady=2)
            self._target_vars[tid] = var
            self._target_boxes[tid] = chk

    def _apply_target_selection_to_ui(self, selection: Sequence[int]) -> None:
        lookup = set(selection)
        for tid, var in self._target_vars.items():
            var.set(tid in lookup)

    def _on_target_toggle(self, tid: int) -> None:
        if self._selection_blocked:
            return
        selected = tuple(t for t, var in self._target_vars.items() if var.get())
        if not (MIN_TARGETS <= len(selected) <= MAX_TARGETS):
            self._status_var.set(f"Select {MIN_TARGETS}-{MAX_TARGETS} targets.")
            self._selection_blocked = True
            self._apply_target_selection_to_ui(self._state_store.snapshot().selected_target_ids)
            self._selection_blocked = False
            return
        try:
            sanitized = self._mission.set_target_ids(selected)
        except ValueError as exc:
            self._status_var.set(str(exc))
            self._selection_blocked = True
            self._apply_target_selection_to_ui(self._state_store.snapshot().selected_target_ids)
            self._selection_blocked = False
            return
        self._selection_blocked = True
        self._apply_target_selection_to_ui(sanitized)
        self._selection_blocked = False
        self._status_var.set(f"Targets -> {sanitized}")

    def _on_switch_radius_change(self) -> None:
        if self._radius_blocked:
            return
        try:
            value = float(self._switch_var.get())
        except (tk.TclError, ValueError):
            return
        radius = self._mission.set_switch_radius(value)
        self._radius_blocked = True
        self._switch_var.set(radius)
        self._radius_blocked = False
        self._status_var.set(f"Switch radius -> {radius:.1f} mm")

    def _refresh(self) -> None:
        snap = self._state_store.snapshot()
        available = tuple(snap.available_target_ids)
        if available and available != self._known_target_ids:
            self._known_target_ids = available
            self._selection_blocked = True
            self._build_target_controls(self._known_target_ids, snap.selected_target_ids)
            self._selection_blocked = False
        else:
            self._selection_blocked = True
            self._apply_target_selection_to_ui(snap.selected_target_ids)
            self._selection_blocked = False
        if not self._radius_blocked:
            self._radius_blocked = True
            self._switch_var.set(snap.switch_radius_mm)
            self._radius_blocked = False
        values = {
            "python_state": snap.python_state.value,
            "active_target_id": f"ID {snap.active_target_id}" if snap.active_target_id >= 0 else "-",
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


def create_gui(
    state_store: IDESAStateStore2,
    mission: TargetQueueController2,
    start_callback: Callable[[], None],
    robot_stop_callback: Callable[[], None],
    stop_callback: Callable[[], None],
) -> IDESAGuiApp2:
    """Factory used by IDESA_Main_2."""
    return IDESAGuiApp2(state_store, mission, start_callback, robot_stop_callback, stop_callback)
