"""
IDESA_Main_4.py
Main orchestrator: owns shared state, starts GUI, runs update loop,
arbitrates AUTO vs MANUAL commands, and feeds Comms.

Run:
    python IDESA_Main_4.py
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from threading import Lock

from IDESA_Mission_Control_4 import IDESAMissionControl4
from IDESA_Vision_4 import VisionSystem4
from IDESA_Navigation_4 import NavigationSystem4
from IDESA_Manual_4 import ManualController4
from IDESA_Comms_4 import UDPComms4


# -----------------------------
# Shared state (owned by Main)
# -----------------------------
@dataclass
class IDESAState4:
    # GUI toggles
    camera_on: bool = False
    sending_enabled: bool = False  # Start/Stop controls UDP send
    control_mode: str = "AUTOMATIC"  # "AUTOMATIC" or "MANUAL"
    auto_sub_mode: str = "MISSION"  # "MISSION" or "RECOVERY"
    estop_pressed: bool = False
    estop_on_value: float = 1.0
    estop_off_value: float = 0.0

    # Encoder reset pulse (sent on cmd UDP for exactly 1 comms-cycle)
    reset_pulse_cycles: int = 0
    encoder_reset_distance_mm: float = 10.0

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

    # Internal automation helpers
    lost_robot_recovery_latched: bool = False


def main() -> None:
    state = IDESAState4()
    state_lock = Lock()

    # Subsystems
    # ---- User-tweakable config (keep it simple for demo) ----
    PI_IP = "138.38.226.147"
    CMD_PORT = 50001
    STATUS_PORT = 50003
    SEND_HZ = 2

    # Camera settings
    # NOTE (Windows): camera indices are often unstable; index=1 is NOT reliably the USB camera.
    # Use CAMERA_INDEX="AUTO" to try 1..6 first (likely USB/external), then fall back to 0.
    CAMERA_INDEX = 1
    CAMERA_RESOLUTION = (1280, 720)

    vision = VisionSystem4(state, state_lock, camera_index=CAMERA_INDEX, preferred_res=CAMERA_RESOLUTION)
    navigation = NavigationSystem4(state, state_lock)
    LOG_UDP = True  # Print every packet to the terminal for debugging
    comms = UDPComms4(
        state,
        state_lock,
        pi_ip=PI_IP,
        cmd_port=CMD_PORT,
        status_port=STATUS_PORT,
        hz=SEND_HZ,
        log_packets=LOG_UDP,
    )
    manual = ManualController4(state, state_lock)

    # GUI (Tk mainloop lives here) 
    app = IDESAMissionControl4(
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
            mode_raw = str(state.control_mode).upper()
            mode = "MANUAL" if mode_raw == "MANUAL" else "AUTOMATIC"
            auto_sub_raw = str(getattr(state, "auto_sub_mode", "MISSION")).upper()
            auto_sub_mode = "RECOVERY" if auto_sub_raw == "RECOVERY" else "MISSION"

            # Lost-robot auto fallback: switch to Recovery Rover after 10 s without vision.
            if mode == "AUTOMATIC" and auto_sub_mode == "MISSION" and state.sending_enabled and not state.estop_pressed:
                last_seen = float(getattr(state, "robot_last_seen", 0.0))
                if last_seen > 0.0:
                    if (now - last_seen) >= 10.0:
                        if not state.lost_robot_recovery_latched:
                            state.auto_sub_mode = "RECOVERY"
                            auto_sub_mode = "RECOVERY"
                            state.reset_pulse_cycles = max(1, int(getattr(state, "reset_pulse_cycles", 0)) or 1)
                            state.lost_robot_recovery_latched = True
                    else:
                        state.lost_robot_recovery_latched = False
                else:
                    state.lost_robot_recovery_latched = False
            else:
                state.lost_robot_recovery_latched = False

            # Refresh auto_sub_mode in case the fallback adjusted it above.
            auto_sub_raw = str(getattr(state, "auto_sub_mode", "MISSION")).upper()
            auto_sub_mode = "RECOVERY" if auto_sub_raw == "RECOVERY" else "MISSION"
            # Control-state flag logic (pure status value sent on UDP 50003).
            manual_active = mode == "MANUAL"
            auto_mission_active = mode == "AUTOMATIC" and auto_sub_mode == "MISSION"

            if manual_active or auto_mission_active:
                state.cmd_state_flag = 0.0
            else:
                # Recovery, stopped, or any other condition should advertise 1.
                state.cmd_state_flag = 1.0

            # E-stop always forces commanded motion to zero.
            if state.estop_pressed:
                state.cmd_distance_mm = 0.0
                state.cmd_angle_deg = 0.0
            elif not state.sending_enabled:
                state.cmd_distance_mm = 0.0
                state.cmd_angle_deg = 0.0
            elif mode == "MANUAL":
                if state.manual_pulse_pending:
                    state.cmd_distance_mm = float(state.manual_pulse_distance_mm)
                    state.cmd_angle_deg = float(state.manual_pulse_angle_deg)
                    state.manual_pulse_pending = False
                else:
                    # IMPORTANT: go back to zeros after the pulse
                    state.cmd_distance_mm = 0.0
                    state.cmd_angle_deg = 0.0
            else:
                # AUTO (mission): distance+angle from navigation
                state.cmd_distance_mm = float(state.nav_distance_mm)
                state.cmd_angle_deg = float(state.nav_angle_deg)

        # Comms tick (sends at fixed rate)
        comms.tick(now)

        # Schedule next tick
        app.after(25, tick)  # 20 Hz main loop

    app.after(25, tick)
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


def _start_sending(state: IDESAState4, lock: Lock, comms: UDPComms4) -> None:
    with lock:
        state.sending_enabled = True
    comms.ensure_sockets()


def _stop_sending(state: IDESAState4, lock: Lock, comms: UDPComms4) -> None:
    with lock:
        state.sending_enabled = False
        state.cmd_distance_mm = 0.0
        state.cmd_angle_deg = 0.0
    comms.send_zeros_burst()


def _camera_off_all(state: IDESAState4, lock: Lock, vision: VisionSystem4, comms: UDPComms4) -> None:
    # Camera off implies: stop vision and ensure robot doesn't move
    try:
        vision.stop()
    except Exception:
        pass
    with lock:
        state.camera_on = False
        state.sending_enabled = False
        state.reset_pulse_cycles = 0
        state.robot_visible = False
        state.robot_xy_mm = None
        state.robot_last_seen = 0.0
        state.cmd_distance_mm = 0.0
        state.cmd_angle_deg = 0.0
    comms.send_zeros_burst()


if __name__ == "__main__":
    main()
