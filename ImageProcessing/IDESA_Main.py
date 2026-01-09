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

from GUI_mission_control_1 import create_gui
from IDESA_Comms import UDPSender
from IDESA_State import IDESAStateStore
from IDESA_Vision import VisionSystem


CONFIG = {
    "camera_index": 0,
    "marker_size_mm": 40.0,
    "aruco_dict": "DICT_4X4_50",
    "robot_id": 1,
    "target_id": 3,
    "udp_ip": "138.38.226.147",
    "udp_port": 50001,
    "send_hz": 30.0,
    "display_preview": True,
}


def main() -> int:
    udp_ip = CONFIG.get("udp_ip", "").strip()
    if not udp_ip:
        raise SystemExit("Set 'udp_ip' inside CONFIG before running IDESA_Main.py")

    state_store = IDESAStateStore()
    vision = VisionSystem(
        state_store=state_store,
        camera_index=int(CONFIG.get("camera_index", 0)),
        marker_size_mm=float(CONFIG.get("marker_size_mm", 40.0)),
        dict_name=str(CONFIG.get("aruco_dict", "DICT_4X4_50")),
        robot_id=int(CONFIG.get("robot_id", 1)),
        target_id=int(CONFIG.get("target_id", 3)),
        display_preview=bool(CONFIG.get("display_preview", True)),
    )
    udp_sender = UDPSender(udp_ip, int(CONFIG.get("udp_port", 50001)))

    gui = create_gui(vision, udp_sender, state_store.snapshot, float(CONFIG.get("send_hz", 30.0)))

    try:
        gui.run()
        return 0
    finally:
        try:
            gui.stop()  # type: ignore[attr-defined]
        except Exception:
            pass
        udp_sender.stop()
        udp_sender.close()
        vision.stop_tracking()
        vision.stop_camera()


if __name__ == "__main__":
    raise SystemExit(main())
