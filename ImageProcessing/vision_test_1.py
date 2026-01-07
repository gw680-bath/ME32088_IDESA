#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Vision_test_1.py

Simple ArUco vision test:
- Track ONLY TWO ArUcos:
    Robot marker:  ID 1
    Target marker: ID 3
- Print once per second:
    Robot:  yaw_deg, x, y
    Target: x, y
- Display annotated camera view (OpenCV window)
- Optional Tkinter GUI (--gui) showing video + same values

Based on the same working method as CV_Python_Aruco_4_10_v2.py:
  detectMarkers -> estimatePoseSingleMarkers -> drawFrameAxes -> print

Dependencies:
  pip install numpy opencv-contrib-python pillow

Calibration:
  Expects a CalibrationGantry.npz in the SAME folder as this script
  with arrays:
    CM
    dist_coef
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

# Optional GUI imports (only used if --gui)
try:
    import tkinter as tk
    from tkinter import ttk
    from PIL import Image, ImageTk
    GUI_AVAILABLE = True
except Exception:
    GUI_AVAILABLE = False



def send_udp_data(sock, udp_ip, udp_port, data_array):
    """
    Send a numpy array (e.g., matrix) as binary float32 little-endian via UDP.
    Assumes data_array is a 1D or 2D numpy array of floats.
    """
    if not isinstance(data_array, np.ndarray):
        data_array = np.array(data_array, dtype=np.float32)
    packed_data = data_array.astype(np.float32).tobytes()
    sock.sendto(packed_data, (udp_ip, udp_port))


# --------------------------
# Helpers
# --------------------------
def yaw_deg_from_rvec(rvec: np.ndarray) -> float:
    """
    Convert rvec -> rotation matrix -> yaw angle in degrees.
    Yaw extraction assumes a standard camera coordinate convention.
    """
    R, _ = cv2.Rodrigues(rvec.reshape(3, 1))
    yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    return float(yaw)


def load_calibration_npz(npz_path: str):
    data = np.load(npz_path)
    # Matches your working script names:
    CM = data["CM"]
    dist_coef = data["dist_coef"]
    return CM, dist_coef


# --------------------------
# Optional minimal GUI
# --------------------------
class SimpleVisionGUI:
    def __init__(self, title="Vision Test 1"):
        self.root = tk.Tk()
        self.root.title(title)

        self.robot_var = tk.StringVar(value="Robot (ID 1): -")
        self.target_var = tk.StringVar(value="Target (ID 3): -")
        self.fps_var = tk.StringVar(value="FPS: -")

        top = ttk.Frame(self.root, padding=8)
        top.pack(fill="x")

        ttk.Label(top, textvariable=self.robot_var).pack(anchor="w")
        ttk.Label(top, textvariable=self.target_var).pack(anchor="w")
        ttk.Label(top, textvariable=self.fps_var).pack(anchor="w")

        self.video_label = ttk.Label(self.root, text="Camera feed...")
        self.video_label.pack(fill="both", expand=True, padx=8, pady=8)

        self._imgtk = None

    def set_text(self, robot_text: str, target_text: str, fps: float):
        self.robot_var.set(robot_text)
        self.target_var.set(target_text)
        self.fps_var.set(f"FPS: {fps:.1f}")

    def set_frame(self, frame_bgr: np.ndarray):
        # Convert BGR to RGB for Tk
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(frame_rgb)
        self._imgtk = ImageTk.PhotoImage(image=img)
        self.video_label.configure(image=self._imgtk)

    def run_loop(self, tick_fn, tick_ms=15):
        def _loop():
            tick_fn()
            self.root.after(tick_ms, _loop)
        _loop()
        self.root.mainloop()


# --------------------------
# Main Vision Loop
# --------------------------
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", type=int, default=0, help="Camera index (try 0 or 1)")
    parser.add_argument("--marker-size-mm", type=float, default=40.0, help="Marker size in mm (must match print)")
    parser.add_argument("--dict", type=str, default="DICT_4X4_50", help="ArUco dictionary name")
    parser.add_argument("--robot-id", type=int, default=1, help="Robot ArUco ID")
    parser.add_argument("--target-id", type=int, default=3, help="Target ArUco ID")
    parser.add_argument("--print-hz", type=float, default=1.0, help="Terminal print rate (Hz)")
    parser.add_argument("--gui", action="store_true", help="Use Tkinter GUI (requires pillow)")
    parser.add_argument("--udp-ip", type=str, default="127.0.0.1", help="UDP target IP address")
    parser.add_argument("--udp-port", type=int, default=50001, help="UDP target port")
    args = parser.parse_args()

    if args.gui and not GUI_AVAILABLE:
        print("GUI requested but Tkinter/PIL not available. Install pillow or run without --gui.")
        return

    # UDP setup
    UDP_IP = args.udp_ip
    UDP_PORT = args.udp_port
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"UDP target IP: {UDP_IP}, port: {UDP_PORT}")

    # Load calibration (same style as your working script)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    calib_path = os.path.join(script_dir, "CalibrationGantry.npz")
    if not os.path.exists(calib_path):
        raise FileNotFoundError(
            f"Calibration file not found: {calib_path}\n"
            "Put CalibrationGantry.npz in the same folder as Vision_test_1.py"
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

    print("Camera opened successfully. Starting vision loop... Press 'q' to quit.")

    # Timing
    print_period = 1.0 / max(args.print_hz, 1e-6)
    last_print_t = 0.0

    # For FPS estimate
    last_frame_t = 0.0
    fps = 0.0

    # Keep last known readings (so you can still output if one frame misses)
    last_robot = [0.0, 0.0, 0.0]  # yaw, x, y
    last_target = [0.0, 0.0]      # x, y
    robot_detected = False
    target_detected = False

    # Optional GUI
    gui = SimpleVisionGUI() if args.gui else None

    def tick():
        nonlocal last_print_t, last_frame_t, fps, last_robot, last_target, robot_detected, target_detected

        robot_detected = False
        target_detected = False

        ok, frame = cap.read()
        if not ok or frame is None:
            return

        # FPS update
        now = time.time()
        if last_frame_t > 0:
            dt = now - last_frame_t
            if dt > 0:
                inst = 1.0 / dt
                fps = inst if fps == 0 else (0.9 * fps + 0.1 * inst)
        last_frame_t = now

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _rej = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        # Annotate detections
        annotated = frame.copy()
        robot_found = False
        target_found = False

        if ids is not None and len(ids) > 0:
            ids_flat = ids.flatten().astype(int)
            aruco.drawDetectedMarkers(annotated, corners, ids)

            # Pose estimation (tvec in same unit as marker-size input, here mm)
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, args.marker_size_mm, CM, dist_coef
            )

            # Loop detections and pick out robot + target
            for i, mid in enumerate(ids_flat):
                rvec = rvecs[i].reshape(3, 1)
                tvec = tvecs[i].reshape(3, 1)

                # draw axes for every detected marker (helpful debug)
                cv2.drawFrameAxes(annotated, CM, dist_coef, rvec, tvec, args.marker_size_mm * 0.75)

                x = float(tvec[0, 0])
                y = float(tvec[1, 0])

                if mid == args.robot_id:
                    yaw = yaw_deg_from_rvec(rvec)
                    last_robot = [yaw, x, y]
                    robot_detected = True

                if mid == args.target_id:
                    last_target = [x, y]
                    target_detected = True

        # Build strings (either live, or “NOT DETECTED”)
        if robot_detected:
            ryaw, rx, ry = last_robot
            robot_text = f"Robot (ID {args.robot_id}): yaw={ryaw:6.1f}°  x={rx:8.2f}  y={ry:8.2f}  [mm]"
        else:
            robot_text = f"Robot (ID {args.robot_id}): NOT DETECTED"

        if target_detected:
            tx, ty = last_target
            target_text = f"Target (ID {args.target_id}): x={tx:8.2f}  y={ty:8.2f}  [mm]"
        else:
            target_text = f"Target (ID {args.target_id}): NOT DETECTED"

        # Overlay text on the video too
        cv2.putText(annotated, robot_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (40, 255, 40), 2)
        cv2.putText(annotated, target_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (40, 255, 40), 2)
        cv2.putText(annotated, f"FPS: {fps:.1f}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (40, 255, 40), 2)

        # Print once per second (or your chosen print rate)
        if (now - last_print_t) >= print_period:
            # Send UDP data as a 1D array of 5 float32: [robot_x, robot_y, robot_yaw, target_x, target_y]
            reordered_data = [last_robot[1], last_robot[2], last_robot[0]] + last_target
            data_array = np.array(reordered_data, dtype=np.float32)
            send_udp_data(sock, UDP_IP, UDP_PORT, data_array)

            # IMPORTANT: print exactly in the style you requested
            if robot_detected:
                ryaw, rx, ry = last_robot
                print(f"Robots: angle={ryaw:.1f}, x={rx:.2f}, y={ry:.2f} (aruco {args.robot_id})")
            else:
                print(f"Robots: NOT DETECTED (aruco {args.robot_id})")

            if target_detected:
                tx, ty = last_target
                print(f"Targets: x={tx:.2f}, y={ty:.2f} (aruco {args.target_id})")
            else:
                print(f"Targets: NOT DETECTED (aruco {args.target_id})")

            print("-" * 60)
            last_print_t = now

        # Display
        if gui is not None:
            gui.set_text(robot_text, target_text, fps)
            gui.set_frame(annotated)
        else:
            cv2.imshow("Vision_test_1 - Annotated", annotated)

    try:
        if gui is not None:
            gui.run_loop(tick_fn=tick, tick_ms=15)
        else:
            print("Press 'q' to quit.")
            while True:
                tick()
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
