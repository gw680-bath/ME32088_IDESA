"""IDESA_Mission_Control_4.py

Mission control GUI for the demo build.

Required controls:
- Camera ON / Camera OFF
- Start / Stop
- Emergency Stop / Reset E-Stop
- Switch radius + target selection
- Control mode: Autonomous / Manual

If ttkbootstrap is installed, it is used automatically for nicer styling.

Encoder-reset pulse rule:
- When any UI button is pressed (except camera on/off and target selection),
  request a 1-cycle encoder reset pulse (distance=10, angle=0) on UDP 50001.
  The actual one-cycle behaviour is implemented in Comms (tied to send rate).
"""

from __future__ import annotations

import time
from threading import Lock
from typing import Callable

import tkinter as tk
from tkinter import ttk

# Optional styling
TTKBOOTSTRAP_AVAILABLE = False
try:
    import ttkbootstrap as tb  # type: ignore

    TTKBOOTSTRAP_AVAILABLE = True
except Exception:
    tb = None  # type: ignore


BaseTk = tb.Window if TTKBOOTSTRAP_AVAILABLE else tk.Tk  # type: ignore


class IDESAMissionControl4(BaseTk):
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
        # ttkbootstrap Window takes a theme name; Tk ignores it.
        if TTKBOOTSTRAP_AVAILABLE:
            super().__init__(themename="superhero")  # type: ignore[misc]
        else:
            super().__init__()

        self.state = state
        self.lock = state_lock
        self._on_camera_on = on_camera_on
        self._on_camera_off = on_camera_off
        self._on_start = on_start
        self._on_stop = on_stop
        self._poll_ms = max(int(poll_ms), 50)

        self.title("IDESA Mission Control 4")
        self.geometry("720x460")
        self.resizable(False, False)

        # Bind arrow keys via Manual module
        on_bind_keys(self)

        # -----------------
        # Layout
        # -----------------
        root = ttk.Frame(self, padding=12)
        root.pack(fill=tk.BOTH, expand=True)

        # Top row buttons
        top = ttk.Frame(root)
        top.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 10))

        ttk.Button(top, text="Camera ON", command=self._camera_on).pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(top, text="Camera OFF", command=self._camera_off).pack(side=tk.LEFT, padx=(0, 18))
        ttk.Button(top, text="Start", command=self._start).pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(top, text="Stop", command=self._stop).pack(side=tk.LEFT, padx=(0, 18))
        ttk.Button(top, text="E-Stop", command=self._estop).pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(top, text="Reset E-Stop", command=self._reset_estop).pack(side=tk.LEFT)

        # Left panel: controls
        controls = ttk.LabelFrame(root, text="Controls")
        controls.grid(row=1, column=0, sticky="nsew", padx=(0, 10))

        # Control mode selection
        mode_frame = ttk.Frame(controls)
        mode_frame.pack(fill=tk.X, padx=8, pady=(8, 6))
        ttk.Label(mode_frame, text="Control mode").pack(anchor="w")

        initial_mode_raw = str(getattr(self.state, "control_mode", "AUTOMATIC")).upper()
        initial_mode = "AUTOMATIC" if initial_mode_raw in {"AUTO", "AUTOMATIC"} else "MANUAL"
        self._mode_var = tk.StringVar(value=initial_mode)
        ttk.Radiobutton(
            mode_frame,
            text="Automatic",
            value="AUTOMATIC",
            variable=self._mode_var,
            command=self._on_mode,
        ).pack(side=tk.LEFT, padx=(0, 10))
        ttk.Radiobutton(
            mode_frame,
            text="Manual",
            value="MANUAL",
            variable=self._mode_var,
            command=self._on_mode,
        ).pack(side=tk.LEFT)

        # Automatic sub-mode selection
        sub_frame = ttk.Frame(controls)
        sub_frame.pack(fill=tk.X, padx=8, pady=(0, 6))
        ttk.Label(sub_frame, text="Automatic sub-mode").pack(anchor="w")

        auto_sub_raw = str(getattr(self.state, "auto_sub_mode", "MISSION")).upper()
        initial_auto_sub = "RECOVERY" if auto_sub_raw == "RECOVERY" else "MISSION"
        self._auto_sub_var = tk.StringVar(value=initial_auto_sub)
        self._auto_sub_buttons: list[ttk.Radiobutton] = []
        for text, value in (("Mission", "MISSION"), ("Recovery", "RECOVERY")):
            btn = ttk.Radiobutton(
                sub_frame,
                text=text,
                value=value,
                variable=self._auto_sub_var,
                command=self._on_auto_sub_mode,
            )
            btn.pack(side=tk.LEFT, padx=(0, 10))
            self._auto_sub_buttons.append(btn)

        # (Optional placeholder for extra autonomous options)
        self._auto_hint = ttk.Label(
            controls,
            text="Automatic Mission = vision targets\nAutomatic Recovery = safe/manual override\nManual = arrow keys (Up/Left/Right)",
            justify="left",
        )
        self._auto_hint.pack(fill=tk.X, padx=8, pady=(0, 8))

        # Target selection
        targets_frame = ttk.LabelFrame(controls, text="Targets (2–7)")
        targets_frame.pack(fill=tk.BOTH, expand=False, padx=8, pady=(0, 8))
        self._target_vars: dict[int, tk.BooleanVar] = {}
        for tid in range(2, 8):
            var = tk.BooleanVar(value=(tid in getattr(self.state, "selected_targets", [])))
            cb = ttk.Checkbutton(targets_frame, text=f"ID {tid}", variable=var, command=self._on_targets)
            cb.pack(anchor="w")
            self._target_vars[tid] = var

        # Switch radius
        radius_frame = ttk.Frame(controls)
        radius_frame.pack(fill=tk.X, padx=8, pady=(0, 8))
        ttk.Label(radius_frame, text="Switch radius [mm]").pack(anchor="w")

        self._radius_var = tk.DoubleVar(value=float(getattr(self.state, "switch_radius_mm", 150.0)))
        spin = tk.Spinbox(
            radius_frame,
            from_=10,
            to=5000,
            increment=10,
            textvariable=self._radius_var,
            width=10,
            command=self._on_radius,
        )
        self._radius_var.trace_add("write", lambda *_: self._on_radius())
        spin.pack(anchor="w", pady=(2, 0))

        # Right panel: telemetry
        telem = ttk.LabelFrame(root, text="Live Telemetry")
        telem.grid(row=1, column=1, sticky="nsew")

        self._labels: dict[str, ttk.Label] = {}
        fields = [
            ("Robot visible", "robot_visible"),
            ("Active target visible", "target_visible"),
            ("Active target ID", "active_target_id"),
            ("Cmd distance [mm]", "distance_mm"),
            ("Cmd angle [deg]", "angle_deg"),
            ("Mode", "mode"),
            ("Auto sub-mode", "auto_sub"),
            ("E-Stop", "estop"),
            ("Sending enabled", "sending"),
        ]
        for r, (label, key) in enumerate(fields):
            ttk.Label(telem, text=label).grid(row=r, column=0, sticky="w", padx=8, pady=4)
            v = ttk.Label(telem, text="-")
            v.grid(row=r, column=1, sticky="e", padx=8, pady=4)
            self._labels[key] = v

        # Status bar
        self._status_var = tk.StringVar(value="Idle")
        ttk.Label(root, textvariable=self._status_var, anchor="w").grid(
            row=2, column=0, columnspan=2, sticky="ew", pady=(10, 0)
        )

        root.columnconfigure(0, weight=1)
        root.columnconfigure(1, weight=1)

        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(self._poll_ms, self._refresh)
        self._update_auto_sub_controls()

    # -------------------
    # Internal helpers
    # -------------------
    def _arm_encoder_reset_pulse(self) -> None:
        with self.lock:
            # Only needs one cycle.
            self.state.reset_pulse_cycles = 1

    def _update_auto_sub_controls(self) -> None:
        enabled = self._mode_var.get().upper() == "AUTOMATIC"
        for btn in self._auto_sub_buttons:
            if enabled:
                btn.state(["!disabled"])
            else:
                btn.state(["disabled"])

    # -------------------
    # Button callbacks
    # -------------------
    def _camera_on(self) -> None:
        try:
            self._on_camera_on()
            with self.lock:
                self.state.camera_on = True
            self._status_var.set("Camera ON")
        except Exception as exc:
            self._status_var.set(f"Camera ON failed: {exc}")

    def _camera_off(self) -> None:
        try:
            self._on_camera_off()
            self._status_var.set("Camera OFF (and sending disabled)")
        except Exception as exc:
            self._status_var.set(f"Camera OFF failed: {exc}")

    def _start(self) -> None:
        try:
            self._on_start()
            self._arm_encoder_reset_pulse()
            self._status_var.set("Start: sending enabled")
        except Exception as exc:
            self._status_var.set(f"Start failed: {exc}")

    def _stop(self) -> None:
        try:
            self._on_stop()
            self._arm_encoder_reset_pulse()
            self._status_var.set("Stop: sending disabled")
        except Exception as exc:
            self._status_var.set(f"Stop failed: {exc}")

    def _estop(self) -> None:
        with self.lock:
            self.state.estop_pressed = True
            self.state.sending_enabled = False
        self._arm_encoder_reset_pulse()
        self._status_var.set("E-Stop PRESSED")

    def _reset_estop(self) -> None:
        with self.lock:
            self.state.estop_pressed = False
        self._arm_encoder_reset_pulse()
        self._status_var.set("E-Stop RESET")

    # -------------------
    # Controls callbacks
    # -------------------
    def _on_mode(self) -> None:
        new_mode = self._mode_var.get().upper()
        new_mode = "AUTOMATIC" if new_mode in {"AUTO", "AUTOMATIC"} else "MANUAL"
        with self.lock:
            self.state.control_mode = new_mode
        # Mode change counts as a UI button press (encoder reset pulse)
        self._arm_encoder_reset_pulse()
        self._update_auto_sub_controls()

    def _on_auto_sub_mode(self) -> None:
        new_sub = self._auto_sub_var.get().upper()
        new_sub = "RECOVERY" if new_sub == "RECOVERY" else "MISSION"
        with self.lock:
            self.state.auto_sub_mode = new_sub
        self._arm_encoder_reset_pulse()

    def _on_targets(self) -> None:
        # Targets are explicitly excluded from encoder reset pulse.
        selected = [tid for tid, var in self._target_vars.items() if var.get()]
        if not (1 <= len(selected) <= 6):
            self._status_var.set("Select 1–6 targets")
            return
        with self.lock:
            self.state.selected_targets = selected

    def _on_radius(self) -> None:
        # Radius change is a config action (not a button), so do not reset encoder.
        try:
            v = float(self._radius_var.get())
        except Exception:
            return
        if v <= 0:
            return
        with self.lock:
            self.state.switch_radius_mm = v

    def _on_close(self) -> None:
        try:
            self._on_camera_off()
        except Exception:
            pass
        self.destroy()

    # -------------------
    # Telemetry refresh
    # -------------------
    def _refresh(self) -> None:
        now = time.time()
        with self.lock:
            robot_visible = bool(getattr(self.state, "robot_visible", False))
            active_id = getattr(self.state, "active_target_id", None)
            active_visible = False
            if active_id is not None:
                last_seen = float(getattr(self.state, "targets_last_seen", {}).get(active_id, 0.0))
                active_visible = (now - last_seen) <= 2.0

            cmd_dist = float(getattr(self.state, "cmd_distance_mm", 0.0))
            cmd_ang = float(getattr(self.state, "cmd_angle_deg", 0.0))
            mode = str(getattr(self.state, "control_mode", "AUTOMATIC")).upper()
            auto_sub_mode = str(getattr(self.state, "auto_sub_mode", "MISSION")).upper()
            sending = bool(getattr(self.state, "sending_enabled", False))
            estop = bool(getattr(self.state, "estop_pressed", False))

            # Manual display: show last pulse briefly even if cmd already returned to zero
            last_manual_dist = float(getattr(self.state, "last_manual_distance_mm", 0.0))
            last_manual_ang = float(getattr(self.state, "last_manual_angle_deg", 0.0))
            last_manual_time = float(getattr(self.state, "last_manual_timestamp", 0.0))

        display_dist = cmd_dist
        display_ang = cmd_ang
        if mode == "MANUAL":
            if (now - last_manual_time) <= 1.0 and abs(display_dist) < 1e-6 and abs(display_ang) < 1e-6:
                display_dist = last_manual_dist
                display_ang = last_manual_ang

        auto_mode_active = mode in {"AUTO", "AUTOMATIC"}
        mode_text = "Automatic" if auto_mode_active else "Manual"
        if auto_mode_active:
            auto_sub_text = "Mission" if auto_sub_mode == "MISSION" else "Recovery"
        else:
            auto_sub_text = "-"

        self._labels["robot_visible"].configure(text="YES" if robot_visible else "NO")
        self._labels["target_visible"].configure(text="YES" if active_visible else "NO")
        self._labels["active_target_id"].configure(text=str(active_id) if active_id is not None else "-")
        self._labels["distance_mm"].configure(text=f"{display_dist:.1f}")
        self._labels["angle_deg"].configure(text=f"{display_ang:.1f}")
        self._labels["mode"].configure(text=mode_text)
        self._labels["auto_sub"].configure(text=auto_sub_text)
        self._labels["estop"].configure(text="PRESSED" if estop else "OK")
        self._labels["sending"].configure(text="YES" if sending else "NO")

        self.after(self._poll_ms, self._refresh)
