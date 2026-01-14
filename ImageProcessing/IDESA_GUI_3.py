"""
IDESA_GUI_3.py
Tkinter GUI: Camera On/Off, Start/Stop, Auto/Manual, target selection, radius,
and live telemetry display.
"""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk
from threading import Lock
from typing import Callable

# Type hints kept loose to avoid importing Main (prevents circular dependencies)


class IDESAGuiApp3(tk.Tk):
    def __init__(
        self,
        state,
        state_lock: Lock,
        on_camera_on: Callable[[], None],
        on_camera_off: Callable[[], None],
        on_start: Callable[[], None],
        on_stop: Callable[[], None],
        on_bind_keys: Callable[[tk.Tk], None],
        poll_ms: int = 200,
    ) -> None:
        super().__init__()
        self.state = state
        self.lock = state_lock
        self._on_camera_on = on_camera_on
        self._on_camera_off = on_camera_off
        self._on_start = on_start
        self._on_stop = on_stop
        self._poll_ms = max(int(poll_ms), 50)

        self.title("IDESA Mission Control 3")
        self.geometry("620x420")
        self.resizable(False, False)

        # Bind arrow keys (manual control) through the Manual module
        on_bind_keys(self)

        # Layout
        root = ttk.Frame(self, padding=12)
        root.pack(fill=tk.BOTH, expand=True)

        # Row 0: Camera + Start/Stop
        top = ttk.Frame(root)
        top.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 10))

        ttk.Button(top, text="Camera ON", command=self._camera_on).pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(top, text="Camera OFF", command=self._camera_off).pack(side=tk.LEFT, padx=(0, 18))
        ttk.Button(top, text="Start", command=self._start).pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(top, text="Stop", command=self._stop).pack(side=tk.LEFT)

        # Row 1 col 0: Controls panel
        controls = ttk.LabelFrame(root, text="Controls")
        controls.grid(row=1, column=0, sticky="nsew", padx=(0, 10))

        # Mode selection
        mode_frame = ttk.Frame(controls)
        mode_frame.pack(fill=tk.X, pady=(6, 8))
        ttk.Label(mode_frame, text="Control Mode").pack(anchor="w")

        self._mode_var = tk.StringVar(value="AUTO")
        ttk.Radiobutton(mode_frame, text="AUTO", value="AUTO", variable=self._mode_var, command=self._on_mode).pack(
            side=tk.LEFT, padx=(0, 10)
        )
        ttk.Radiobutton(mode_frame, text="MANUAL", value="MANUAL", variable=self._mode_var, command=self._on_mode).pack(
            side=tk.LEFT
        )

        # Target selection
        targets_frame = ttk.LabelFrame(controls, text="Targets (2–7)")
        targets_frame.pack(fill=tk.BOTH, expand=False, padx=6, pady=(0, 8))

        self._target_vars = {}
        for tid in range(2, 8):
            var = tk.BooleanVar(value=(tid in getattr(self.state, "selected_targets", [])))
            cb = ttk.Checkbutton(targets_frame, text=f"ID {tid}", variable=var, command=self._on_targets)
            cb.pack(anchor="w")
            self._target_vars[tid] = var

        # Switch radius
        radius_frame = ttk.Frame(controls)
        radius_frame.pack(fill=tk.X, padx=6, pady=(0, 8))
        ttk.Label(radius_frame, text="Switch radius [mm]").pack(anchor="w")

        self._radius_var = tk.DoubleVar(value=float(getattr(self.state, "switch_radius_mm", 150.0)))
        self._radius_spin = tk.Spinbox(
            radius_frame,
            from_=10,
            to=5000,
            increment=10,
            textvariable=self._radius_var,
            width=8,
            command=self._on_radius,
        )
        self._radius_var.trace_add("write", lambda *_: self._on_radius())
        self._radius_spin.pack(anchor="w", pady=(2, 0))

        # Row 1 col 1: Telemetry panel
        telem = ttk.LabelFrame(root, text="Live Telemetry")
        telem.grid(row=1, column=1, sticky="nsew")

        self._labels = {}
        fields = [
            ("Robot visible", "robot_visible"),
            ("Target visible", "target_visible"),
            ("Active target ID", "active_target_id"),
            ("Distance [mm]", "distance_mm"),
            ("Angle error [deg]", "angle_deg"),
            ("Mode", "control_mode"),
            ("Sending enabled", "sending_enabled"),
        ]
        for r, (label, key) in enumerate(fields):
            ttk.Label(telem, text=label, font=("Space Grotesk", 11, "bold")).grid(row=r, column=0, sticky="w", pady=4)
            v = ttk.Label(telem, text="-", font=("Space Mono", 11))
            v.grid(row=r, column=1, sticky="e", pady=4)
            self._labels[key] = v

        # Status bar
        self._status = tk.StringVar(value="Idle")
        ttk.Label(root, textvariable=self._status, anchor="w").grid(row=2, column=0, columnspan=2, sticky="ew", pady=(10, 0))

        root.columnconfigure(0, weight=1)
        root.columnconfigure(1, weight=1)

        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(self._poll_ms, self._refresh)

    # -------------------
    # Button callbacks
    # -------------------
    def _camera_on(self) -> None:
        try:
            self._on_camera_on()
            with self.lock:
                self.state.camera_on = True
            self._status.set("Camera ON")
        except Exception as exc:
            self._status.set(f"Camera ON failed: {exc}")

    def _camera_off(self) -> None:
        try:
            self._on_camera_off()
            self._status.set("Camera OFF (and sending disabled)")
        except Exception as exc:
            self._status.set(f"Camera OFF failed: {exc}")

    def _start(self) -> None:
        try:
            self._on_start()
            self._status.set("Sending enabled")
        except Exception as exc:
            self._status.set(f"Start failed: {exc}")

    def _stop(self) -> None:
        try:
            self._on_stop()
            self._status.set("Sending disabled (zeros)")
        except Exception as exc:
            self._status.set(f"Stop failed: {exc}")

    # -------------------
    # Controls callbacks
    # -------------------
    def _on_mode(self) -> None:
        with self.lock:
            self.state.control_mode = self._mode_var.get().upper()

    def _on_targets(self) -> None:
        selected = [tid for tid, var in self._target_vars.items() if var.get()]
        # Enforce 1–6 targets (your spec)
        if not (1 <= len(selected) <= 6):
            self._status.set("Select 1–6 targets.")
            return
        with self.lock:
            self.state.selected_targets = selected

    def _on_radius(self) -> None:
        try:
            v = float(self._radius_var.get())
        except Exception:
            return
        if v <= 0:
            return
        with self.lock:
            self.state.switch_radius_mm = v

    def _on_close(self) -> None:
        # Trigger camera off (also stops sending + sends zeros)
        try:
            self._on_camera_off()
        except Exception:
            pass
        self.destroy()

    # -------------------
    # Telemetry refresh
    # -------------------
    def _refresh(self) -> None:
        with self.lock:
            robot_visible = bool(self.state.robot_visible)
            active_id = self.state.active_target_id
            active_visible = False
            if active_id is not None:
                last_seen = self.state.targets_last_seen.get(active_id, 0.0)
                active_visible = (time.time() - last_seen) <= 2.0

            dist = float(getattr(self.state, "nav_distance_mm", 0.0))
            ang = float(getattr(self.state, "nav_angle_deg", 0.0))
            mode = str(getattr(self.state, "control_mode", "AUTO")).upper()
            sending = bool(getattr(self.state, "sending_enabled", False))

        self._labels["robot_visible"].configure(text="YES" if robot_visible else "NO")
        self._labels["target_visible"].configure(text="YES" if active_visible else "NO")
        self._labels["active_target_id"].configure(text=str(active_id) if active_id is not None else "-")
        self._labels["distance_mm"].configure(text=f"{dist:.1f}")
        self._labels["angle_deg"].configure(text=f"{ang:.1f}")
        self._labels["control_mode"].configure(text=mode)
        self._labels["sending_enabled"].configure(text="YES" if sending else "NO")

        self.after(self._poll_ms, self._refresh)
