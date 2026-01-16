"""
IDESA_Main_3.py
Main orchestrator: owns shared state, starts GUI, runs update loop,
arbitrates AUTO vs MANUAL commands, and feeds Comms.

Run:
    python IDESA_Main_3.py
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from threading import Lock

from IDESA_GUI_3 import IDESAGuiApp3
from IDESA_Vision_3 import VisionSystem3
from IDESA_Navigation_3 import NavigationSystem3
from IDESA_Manual_3 import ManualController3
from IDESA_Comms_3 import UDPComms3


# -----------------------------
# Shared state (owned by Main)
# -----------------------------
@dataclass
class IDESAState3:
    # GUI toggles
    camera_on: bool = False
    sending_enabled: bool = False  # Start/Stop controls UDP send
    control_mode: str = "AUTO"  # "AUTO", "MANUAL", or "RECOVERY"

    # Targets
    selected_targets: list[int] = field(default_factory=lambda: [2, 3])
    switch_radius_mm: float = 150.0
    robot_id: int = 1

    # Vision outputs (world units: mm)
    robot_visible: bool = False
    robot_xy_mm: tuple[float, float] | None = None
    robot_yaw_deg: float = 0.0  # yaw in degrees (see Navigation for 0-along +Y convention)
    robot_last_seen: float = 0.0

    # Targets detected (id -> (x_mm, y_mm)), plus last seen times
    targets_xy_mm: dict[int, tuple[float, float]] = field(default_factory=dict)
    targets_last_seen: dict[int, float] = field(default_factory=dict)

    # Active target chosen by Navigation/mission logic
    active_target_id: int | None = None

    # Navigation outputs
    nav_distance_mm: float = 0.0
    nav_angle_deg: float = 0.0
    hold_zero_until: float = 0.0  # time until which AUTO outputs forced to zero

    # Manual “pulse” outputs (one-shot)
    manual_pulse_pending: bool = False
    manual_pulse_distance_mm: float = 0.0
    manual_pulse_angle_deg: float = 0.0
    last_manual_distance_mm: float = 0.0
    last_manual_angle_deg: float = 0.0
    last_manual_timestamp: float = 0.0

    # Final commanded outputs (what Comms will send)
    cmd_distance_mm: float = 0.0
    cmd_angle_deg: float = 0.0
    cmd_state_flag: float = 0.0


def main() -> None:
    state = IDESAState3()
    state_lock = Lock()

    # Subsystems
    vision = VisionSystem3(state, state_lock)
    navigation = NavigationSystem3(state, state_lock)
    comms = UDPComms3(state, state_lock, pi_ip="138.38.226.147", pi_port=50001, hz=2.0)
    manual = ManualController3(state, state_lock)

    # GUI (Tk mainloop lives here) 
    app = IDESAGuiApp3(
        state=state,
        state_lock=state_lock,
        on_camera_on=vision.start,
        on_camera_off=lambda: _camera_off_all(state, state_lock, vision, comms),
        on_start=lambda: _start_sending(state, state_lock, comms),
        on_stop=lambda: _stop_sending(state, state_lock, comms),
        on_bind_keys=manual.bind_to_tk,
    )

    # Periodic update loop (runs inside Tk via after())
    def tick() -> None:
        now = time.time()

        # Navigation update (AUTO only)
        navigation.update(now)

        # Arbitration: pick what gets sent
        with state_lock:
            state.cmd_state_flag = 0.0
            if not state.sending_enabled:
                state.cmd_distance_mm = 0.0
                state.cmd_angle_deg = 0.0
                state.cmd_state_flag = 0.0
            else:
                mode = str(state.control_mode).upper()
                if mode == "RECOVERY":
                    state.cmd_distance_mm = 0.0
                    state.cmd_angle_deg = 0.0
                    state.cmd_state_flag = 1.0
                elif mode == "MANUAL":
                    if state.manual_pulse_pending:
                        # One-shot pulse, then revert to zeros automatically
                        state.cmd_distance_mm = float(state.manual_pulse_distance_mm)
                        state.cmd_angle_deg = float(state.manual_pulse_angle_deg)
                        state.manual_pulse_pending = False
                    else:
                        state.cmd_distance_mm = 0.0
                        state.cmd_angle_deg = 0.0
                else:
                    # AUTO
                    state.cmd_distance_mm = float(state.nav_distance_mm)
                    state.cmd_angle_deg = float(state.nav_angle_deg)
                    state.cmd_state_flag = 0.0

        # Comms tick (sends at fixed rate if enabled)
        comms.tick(now)

        # Schedule next tick
        app.after(50, tick)  # 20 Hz main loop

    app.after(50, tick)
    app.mainloop()

    # On GUI close
    try:
        vision.stop()
    except Exception:
        pass
    try:
        comms.shutdown()
    except Exception:
        pass


def _start_sending(state: IDESAState3, lock: Lock, comms: UDPComms3) -> None:
    with lock:
        state.sending_enabled = True
    comms.ensure_socket()


def _stop_sending(state: IDESAState3, lock: Lock, comms: UDPComms3) -> None:
    with lock:
        state.sending_enabled = False
        state.cmd_distance_mm = 0.0
        state.cmd_angle_deg = 0.0
        state.cmd_state_flag = 0.0
    comms.send_zeros_burst()


def _camera_off_all(state: IDESAState3, lock: Lock, vision: VisionSystem3, comms: UDPComms3) -> None:
    # Camera off implies: stop vision and ensure robot doesn't move
    try:
        vision.stop()
    except Exception:
        pass
    with lock:
        state.camera_on = False
        state.sending_enabled = False
        state.cmd_distance_mm = 0.0
        state.cmd_angle_deg = 0.0
        state.cmd_state_flag = 0.0
    comms.send_zeros_burst()


if __name__ == "__main__":
    main()
