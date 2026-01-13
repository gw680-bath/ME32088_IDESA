"""IDESA entry point.

Usage (from ImageProcessing folder):
    python IDESA_Main.py

Edit the CONFIG dictionary below to change camera index, UDP destination, marker
size, ArUco dictionary, send frequency, or preview preference without typing
command-line arguments. Robot/target IDs can be adjusted live in the GUI.

This spins up the modular system:
  * Vision thread captures from the selected camera and keeps the shared state up to date
  * UDP thread sends the 20-byte payload at the requested frequency
  * GUI manages start/stop plus live telemetry

`vision_test_1_onboard_udp.py` remains untouched as a reference script.
"""

from __future__ import annotations

from typing import Optional

from GUI_mission_control_1 import create_gui
from IDESA_Comms import UDPSender
from IDESA_Mission import TargetQueueController
from IDESA_State import IDESAStateStore
from IDESA_Vision import VisionSystem


CONFIG = {
    "camera_index": 1,
    "camera_resolution": [1920, 1080],
    "marker_size_mm": 40.0,
    "aruco_dict": "DICT_4X4_50",
    "robot_id": 1,
    "default_target_ids": [2, 3, 4],
    "available_target_ids": [2, 3, 4, 5, 6, 7],
    "switch_radius_mm": 100.0,
    "udp_ip": "138.38.226.147",
    "udp_port": 50001,
    "send_hz": 30.0,
    "display_preview": True,
    "preview_window_scale": 0.6,
}


def main() -> int:
    udp_ip = CONFIG.get("udp_ip", "").strip()
    if not udp_ip:
        raise SystemExit("Set 'udp_ip' inside CONFIG before running IDESA_Main.py")

    state_store = IDESAStateStore()

    available_target_ids = tuple(int(t) for t in CONFIG.get("available_target_ids", (2, 3, 4, 5, 6, 7)))
    if len(available_target_ids) < 2:
        raise SystemExit("CONFIG must list at least two 'available_target_ids'.")

    default_target_ids = tuple(int(t) for t in CONFIG.get("default_target_ids", available_target_ids[:2]))
    if len(default_target_ids) < 2:
        default_target_ids = available_target_ids[:2]

    switch_radius_mm = float(CONFIG.get("switch_radius_mm", 100.0))
    resolution = CONFIG.get("camera_resolution")
    frame_width: Optional[int] = None
    frame_height: Optional[int] = None
    if isinstance(resolution, (list, tuple)) and len(resolution) == 2:
        try:
            frame_width = int(resolution[0])
            frame_height = int(resolution[1])
        except (TypeError, ValueError):
            frame_width = frame_height = None

    vision = VisionSystem(
        state_store=state_store,
        camera_index=int(CONFIG.get("camera_index", 0)),
        marker_size_mm=float(CONFIG.get("marker_size_mm", 40.0)),
        dict_name=str(CONFIG.get("aruco_dict", "DICT_4X4_50")),
        robot_id=int(CONFIG.get("robot_id", 1)),
        target_id=int(default_target_ids[0]),
        display_preview=bool(CONFIG.get("display_preview", True)),
        frame_width=frame_width,
        frame_height=frame_height,
        preview_scale=float(CONFIG.get("preview_window_scale", 0.6)),
    )
    vision.set_tracked_target_ids(default_target_ids)

    mission = TargetQueueController(state_store)
    mission.set_switch_radius(switch_radius_mm)
    mission.set_target_ids(default_target_ids)

    udp_sender = UDPSender(udp_ip, int(CONFIG.get("udp_port", 50001)))

    gui = create_gui(
        vision,
        mission,
        udp_sender,
        state_store.snapshot,
        float(CONFIG.get("send_hz", 30.0)),
        available_target_ids,
        default_target_ids,
        switch_radius_mm,
    )

    try:
        gui.run()
        return 0
    finally:
        try:
            gui.stop()  # type: ignore[attr-defined]
        except Exception:
            pass
        mission.stop()
        udp_sender.stop()
        udp_sender.close()
        vision.stop_tracking()
        vision.stop_camera()


if __name__ == "__main__":
    raise SystemExit(main())
