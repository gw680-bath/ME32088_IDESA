"""IDESA entry point.

Usage (from ImageProcessing folder):
    python IDESA_Main.py --udp-ip 192.168.1.50 --camera 0 --marker-size-mm 40 \
        --robot-id 1 --target-id 3 --udp-port 50001 --send-hz 30 [--no-preview]

This spins up the modular system:
  * Vision thread captures from the selected camera and keeps the shared state up to date
  * UDP thread sends the 20-byte payload at the requested frequency
  * GUI manages start/stop plus live telemetry

`vision_test_1_onboard_udp.py` remains untouched as a reference script.
"""

from __future__ import annotations

import argparse
import sys

from GUI_mission_control_1 import create_gui
from IDESA_Comms import UDPSender
from IDESA_State import IDESAStateStore
from IDESA_Vision import VisionSystem


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Modular IDESA vision + UDP stack")
    parser.add_argument("--camera", type=int, default=0, help="Camera index (try 0 or 1)")
    parser.add_argument("--marker-size-mm", type=float, default=40.0, help="Marker size in mm")
    parser.add_argument("--dict", type=str, default="DICT_4X4_50", help="ArUco dictionary name")
    parser.add_argument("--robot-id", type=int, default=1, help="Robot ArUco ID")
    parser.add_argument("--target-id", type=int, default=3, help="Target ArUco ID")
    parser.add_argument("--udp-ip", type=str, default="", help="Destination IP for Raspberry Pi")
    parser.add_argument("--udp-port", type=int, default=50001, help="Destination UDP port")
    parser.add_argument("--send-hz", type=float, default=30.0, help="UDP send frequency")
    parser.add_argument("--no-preview", action="store_true", help="Disable OpenCV preview window")
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)

    if not args.udp_ip:
        parser.error("You must provide --udp-ip <destination>")

    state_store = IDESAStateStore()
    vision = VisionSystem(
        state_store=state_store,
        camera_index=args.camera,
        marker_size_mm=args.marker_size_mm,
        dict_name=args.dict,
        robot_id=args.robot_id,
        target_id=args.target_id,
        display_preview=not args.no_preview,
    )
    udp_sender = UDPSender(args.udp_ip, args.udp_port)

    gui = create_gui(vision, udp_sender, state_store.snapshot, args.send_hz)

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
