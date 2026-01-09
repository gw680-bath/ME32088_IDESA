# main.py
"""
Main orchestrator (glue):
- Creates GUI
- Creates ArUco tracker
- Runs update loop: read camera -> detect -> compute 5 values -> send UDP -> update GUI
- Sends UDP packet constantly while "Start" is active

Why your current main shows no coordinates:
- In your current main, CALIB_NPZ=None so pose estimation never runs.
  That means poses_cam is empty and compute_packet() returns invalid, so your packet stays NaN.
- Also, compute_packet() requires HOME_ID to be visible; if HOME_ID is wrong or not in view,
  it will also return invalid (even if robot/target are visible).

This patched version:
- Auto-loads CalibrationGantry.npz (same format used in CV_Python_Aruco_4_10_v2.py: CM/dist_coef)
- Falls back to CAMERA-frame coordinates if HOME marker isn't visible (for debugging)
- Keeps the rest of your GUI/UDP structure the same

UDP payload:
- 5 x float32, little-endian, order:
  robot_x, robot_y, robot_yaw_deg, target_x, target_y
= 20 bytes per packet
"""

from __future__ import annotations
import os
import socket
import struct
import time
import tkinter as tk
from tkinter import ttk, messagebox

import numpy as np

# Use the patched vision module (supports CM/dist_coef keys + cam-frame fallback)
from vision_aruco_patched import ArucoTracker
from gui_mission_control import MissionControlGUI, ManualCmd


# ======================
# CONFIG (edit to match your setup)
# ======================
# Your working CV script used camera index 1 for the external camera.
CAMERA_INDEX = 1
FRAME_W, FRAME_H = 1280, 720

ARUCO_DICT = "DICT_4X4_50"

# IMPORTANT:
# - OpenCV returns tvecs in the same units as MARKER_LENGTH.
# - Set this to your REAL marker size.
MARKER_LENGTH_M = 0.04  # 4cm (matches your CV script's marker_size=40)

# Choose IDs that actually exist in view.
# From your screenshot you have ID 1 and ID 2 markers.
HOME_ID = 0
ROBOT_ID = 3
TARGET_ID = 1  # change to your "target marker" if different from home

# Calibration .npz
# If left as None, we auto-try to load "CalibrationGantry.npz" from this folder.
CALIB_NPZ = None  # e.g. "CalibrationGantry.npz"

# UDP to Simulink
SIMULINK_IP = "127.0.0.1"  # change to Simulink PC IP
SIMULINK_PORT = 25000      # change to Simulink UDP Receive port
SEND_HZ = 30.0             # send rate while running

# When markers are lost, keep sending last known values (usually better than NaNs for control)
HOLD_LAST_ON_LOSS = True

# If True: require HOME marker visible to produce valid coords (best for a stable world frame).
# If False: show/send CAMERA-frame coords when HOME isn't visible (best for debugging).
REQUIRE_HOME_MARKER = False


class App:
    def __init__(self, root: tk.Tk):
        self.root = root

        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_addr = (SIMULINK_IP, SIMULINK_PORT)

        # Load calibration (auto-detect like your working CV script)
        cam_mtx = dist = None
        calib_path = CALIB_NPZ
        if calib_path is None:
            # try to find CalibrationGantry.npz beside this script
            here = os.path.dirname(os.path.abspath(__file__))
            candidate = os.path.join(here, "CalibrationGantry.npz")
            if os.path.exists(candidate):
                calib_path = candidate

        if calib_path:
            try:
                cam_mtx, dist = ArucoTracker.load_calibration_npz(calib_path)
            except Exception as e:
                messagebox.showwarning(
                    "Calibration",
                    f"Failed to load calibration from:\n{calib_path}\n\n{e}\n\nContinuing without pose (no tvecs).",
                )
        else:
            messagebox.showwarning(
                "Calibration",
                "No calibration file found (CalibrationGantry.npz).\n"
                "Pose estimation needs calibration; without it you will NOT get tvecs/coordinates.",
            )

        # Vision
        self.tracker = ArucoTracker(
            camera_index=CAMERA_INDEX,
            width=FRAME_W,
            height=FRAME_H,
            aruco_dict_name=ARUCO_DICT,
            marker_length=MARKER_LENGTH_M,
            home_id=HOME_ID,
            camera_matrix=cam_mtx,
            dist_coeffs=dist,
        )

        # Runtime state
        self.camera_active = False
        self.running = False

        self.last_frame_t = 0.0
        self.fps = 0.0
        self.last_send_t = 0.0

        # last known packet values
        self.last_packet = (np.nan, np.nan, np.nan, np.nan, np.nan)

        # GUI
        self.gui = MissionControlGUI(
            root=root,
            on_activate_camera=self.activate_camera,
            on_start=self.start,
            on_stop=self.stop,
            on_manual_cmd=self.on_manual_cmd,
        )

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    # ======================
    # GUI callbacks
    # ======================
    def activate_camera(self):
        if self.camera_active:
            self.gui.set_status("Status: Camera already active")
            return
        try:
            self.tracker.start()
        except Exception as e:
            messagebox.showerror("Camera", str(e))
            return

        self.camera_active = True
        self.gui.set_status("Status: Camera active")
        self.tick()

    def start(self):
        self.running = True
        self.gui.set_status("Status: RUNNING (sending UDP)")

    def stop(self):
        self.running = False
        self.gui.set_status("Status: STOPPED")

    def on_manual_cmd(self, cmd: ManualCmd):
        # Placeholder (you can route this to Simulink on a different UDP port if you want)
        pass

    # ======================
    # Main update loop
    # ======================
    def tick(self):
        if not self.camera_active:
            return

        # Read + detect
        try:
            frame = self.tracker.read_frame()
            annotated, poses_cam = self.tracker.detect(frame)
        except Exception as e:
            self.gui.set_status(f"Status: Camera read/detect failed: {e}")
            self.root.after(200, self.tick)
            return

        # FPS
        now = time.time()
        if self.last_frame_t > 0:
            dt = now - self.last_frame_t
            if dt > 0:
                inst = 1.0 / dt
                self.fps = inst if self.fps == 0 else (0.9 * self.fps + 0.1 * inst)
        self.last_frame_t = now
        self.gui.set_fps(self.fps)

        # Compute the 5-value packet
        robot_pose, target_xy = self.tracker.compute_packet(
            poses_cam,
            robot_id=ROBOT_ID,
            target_id=TARGET_ID,
            require_home=REQUIRE_HOME_MARKER,
        )

        if robot_pose.valid and target_xy.valid:
            packet = (robot_pose.x, robot_pose.y, robot_pose.yaw_deg, target_xy.x, target_xy.y)
            self.last_packet = packet
        else:
            packet = self.last_packet if HOLD_LAST_ON_LOSS else (np.nan, np.nan, np.nan, np.nan, np.nan)
            self.last_packet = packet

        # GUI text
        rx, ry, ryaw, tx, ty = packet
        frame_name = robot_pose.frame if robot_pose.valid else ("home" if REQUIRE_HOME_MARKER else "cam/home")
        self.gui.set_robot_text(
            f"Robot (ID {ROBOT_ID}, frame={frame_name}): x={rx:.3f} y={ry:.3f} yaw={ryaw:.1f}°"
        )
        self.gui.set_target_text(
            f"Target (ID {TARGET_ID}, frame={target_xy.frame}): x={tx:.3f} y={ty:.3f}"
        )

        # Show video
        self.gui.set_frame(annotated)

        # UDP send (constant while running)
        self.maybe_send_udp(packet, now)

        # schedule next tick
        self.root.after(15, self.tick)

    def maybe_send_udp(self, packet, now: float):
        if not self.running:
            return

        min_dt = 1.0 / SEND_HZ if SEND_HZ and SEND_HZ > 0 else 0.0
        if (now - self.last_send_t) < min_dt:
            return

        rx, ry, ryaw, tx, ty = packet
        payload = struct.pack("<5f", float(rx), float(ry), float(ryaw), float(tx), float(ty))

        try:
            self.sock.sendto(payload, self.udp_addr)
            self.last_send_t = now
        except Exception as e:
            self.gui.set_status(f"Status: UDP send error: {e}")

    def on_close(self):
        try:
            self.stop()
            try:
                self.tracker.stop()
            except Exception:
                pass
            try:
                self.sock.close()
            except Exception:
                pass
        finally:
            self.root.destroy()


def main():
    root = tk.Tk()
    try:
        ttk.Style().theme_use("clam")
    except Exception:
        pass
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
