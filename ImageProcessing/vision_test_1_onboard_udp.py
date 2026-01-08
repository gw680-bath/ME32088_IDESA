#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""vision_test_1_onboard_udp.py

Two-marker ArUco tracker (Robot ID=1, Target ID=3) + binary UDP sender.

Recommended architecture for your robot:
  Laptop (Python/OpenCV)  --> UDP -->  Raspberry Pi (Simulink model running on-board)

UDP payload (fixed 20 bytes):
  [robot_x_mm, robot_y_mm, robot_yaw_deg, target_x_mm, target_y_mm]
Packed as little-endian float32: struct '<5f'

Requirements:
  pip install numpy opencv-contrib-python

Calibration:
  Place CalibrationGantry.npz in the SAME folder as this script.
  It must contain arrays: 'CM' and 'dist_coef'

Quit:
  Press 'q' in the OpenCV window.
"""

import os
import time
import math
import argparse
import struct
import socket

import cv2
import cv2.aruco as aruco
import numpy as np


def yaw_deg_from_rvec(rvec: np.ndarray) -> float:
    """Convert rvec to yaw angle in degrees."""
    R, _ = cv2.Rodrigues(rvec.reshape(3, 1))
    yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    return float(yaw)


def load_calibration_npz(npz_path: str):
    data = np.load(npz_path)
    CM = data["CM"]
    dist_coef = data["dist_coef"]
    return CM, dist_coef


def pack_5floats(robot_x: float, robot_y: float, robot_yaw_deg: float,
                 target_x: float, target_y: float) -> bytes:
    """Pack 5 float32 values into 20 bytes, little-endian."""
    return struct.pack("<5f", float(robot_x), float(robot_y), float(robot_yaw_deg),
                       float(target_x), float(target_y))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", type=int, default=0, help="Camera index (try 0 or 1)")
    parser.add_argument("--marker-size-mm", type=float, default=40.0, help="Marker size in mm")
    parser.add_argument("--dict", type=str, default="DICT_4X4_50", help="ArUco dictionary name")
    parser.add_argument("--robot-id", type=int, default=1, help="Robot ArUco ID")
    parser.add_argument("--target-id", type=int, default=3, help="Target ArUco ID")

    # Printing and sending rates are independent
    parser.add_argument("--print-hz", type=float, default=1.0, help="Terminal print rate (Hz)")
    parser.add_argument("--send-hz", type=float, default=30.0, help="UDP send rate (Hz)")

    parser.add_argument("--udp-ip", type=str, default="", help="138.38.226.147")
    parser.add_argument("--udp-port", type=int, default=50001, help="50001")

    args = parser.parse_args()

    if not args.udp_ip:
        raise SystemExit(
            "You must pass --udp-ip <PI_IP>. Example: --udp-ip 192.168.1.50"
        )

    # UDP socket
    udp_addr = (args.udp_ip, int(args.udp_port))
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    print(f"UDP destination: {udp_addr[0]}:{udp_addr[1]} (payload: 5 x float32 = 20 bytes)")

    # Load calibration
    script_dir = os.path.dirname(os.path.abspath(__file__))
    calib_path = os.path.join(script_dir, "CalibrationGantry.npz")
    if not os.path.exists(calib_path):
        raise FileNotFoundError(
            f"Calibration file not found: {calib_path}\n"
            "Put CalibrationGantry.npz in the same folder as this script."
        )
    CM, dist_coef = load_calibration_npz(calib_path)

    # ArUco config
    if not hasattr(aruco, args.dict):
        raise ValueError(f"Unknown ArUco dictionary: {args.dict}")
    aruco_dict = aruco.getPredefinedDictionary(getattr(aruco, args.dict))
    parameters = aruco.DetectorParameters()

    # Camera
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera index {args.camera}")

    print("Camera opened. Starting vision loop... Press 'q' to quit.")

    # Timers
    send_period = 1.0 / max(args.send_hz, 1e-6)
    print_period = 1.0 / max(args.print_hz, 1e-6)
    last_send_t = 0.0
    last_print_t = 0.0

    # Last known readings (keep sending even if a frame misses)
    last_robot_yaw = 0.0
    last_robot_x = 0.0
    last_robot_y = 0.0
    last_target_x = 0.0
    last_target_y = 0.0

    fps = 0.0
    last_frame_t = 0.0

    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            continue

        now = time.time()
        if last_frame_t > 0:
            dt = now - last_frame_t
            if dt > 0:
                inst = 1.0 / dt
                fps = inst if fps == 0 else (0.9 * fps + 0.1 * inst)
        last_frame_t = now

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        annotated = frame.copy()
        robot_detected = False
        target_detected = False

        if ids is not None and len(ids) > 0:
            ids_flat = ids.flatten().astype(int)
            aruco.drawDetectedMarkers(annotated, corners, ids)

            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, args.marker_size_mm, CM, dist_coef
            )

            for i, mid in enumerate(ids_flat):
                rvec = rvecs[i].reshape(3, 1)
                tvec = tvecs[i].reshape(3, 1)
                cv2.drawFrameAxes(annotated, CM, dist_coef, rvec, tvec, args.marker_size_mm * 0.75)

                x = float(tvec[0, 0])
                y = float(tvec[1, 0])

                if mid == args.robot_id:
                    last_robot_yaw = yaw_deg_from_rvec(rvec)
                    last_robot_x = x
                    last_robot_y = y
                    robot_detected = True

                if mid == args.target_id:
                    last_target_x = x
                    last_target_y = y
                    target_detected = True

        # Overlay
        robot_text = (f"Robot (ID {args.robot_id}): yaw={last_robot_yaw:6.1f}°  x={last_robot_x:8.2f}  y={last_robot_y:8.2f}  [mm]"
                      if robot_detected else f"Robot (ID {args.robot_id}): NOT DETECTED (sending last)")
        target_text = (f"Target (ID {args.target_id}): x={last_target_x:8.2f}  y={last_target_y:8.2f}  [mm]"
                       if target_detected else f"Target (ID {args.target_id}): NOT DETECTED (sending last)")

        cv2.putText(annotated, robot_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (40, 255, 40), 2)
        cv2.putText(annotated, target_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (40, 255, 40), 2)
        cv2.putText(annotated, f"FPS: {fps:.1f}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (40, 255, 40), 2)

        # UDP send at send_hz (independent of print rate)
        if (now - last_send_t) >= send_period:
            payload = pack_5floats(last_robot_x, last_robot_y, last_robot_yaw,
                                   last_target_x, last_target_y)
            try:
                sock.sendto(payload, udp_addr)
            except OSError as e:
                # Keep running; printing once per second will show if this persists
                pass
            last_send_t = now

        # Print at print_hz
        if (now - last_print_t) >= print_period:
            print(f"Robot: angle={last_robot_yaw:.1f}, x={last_robot_x:.2f}, y={last_robot_y:.2f} (aruco {args.robot_id})")
            print(f"Target: x={last_target_x:.2f}, y={last_target_y:.2f} (aruco {args.target_id})")
            print(f"Send: {args.send_hz:.1f} Hz to {udp_addr[0]}:{udp_addr[1]} | FPS: {fps:.1f}")
            print("-" * 60)
            last_print_t = now

        cv2.imshow("Vision - Onboard UDP", annotated)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
