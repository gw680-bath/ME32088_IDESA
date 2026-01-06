# main.py
"""
Main orchestrator (glue):
- Creates GUI
- Creates ArUco tracker
- Runs update loop: read camera -> detect -> compute 5 values -> send UDP -> update GUI
- Sends UDP packet constantly while "Start" is active

UDP payload:
- 5 x float32, little-endian, order:
  robot_x, robot_y, robot_yaw_deg, target_x, target_y
= 20 bytes per packet

Simulink tip:
- UDP Receive as uint8[20]
- Byte Unpack: 5x single, little-endian

Dependencies:
  pip install numpy pillow opencv-contrib-python
"""

from __future__ import annotations
import socket
import struct
import time
import tkinter as tk
from tkinter import ttk, messagebox

import numpy as np

from vision_aruco import ArucoTracker
from gui_mission_control import MissionControlGUI, ManualCmd


# ======================
# CONFIG (edit to match your setup)
# ======================
CAMERA_INDEX = 0
FRAME_W, FRAME_H = 1280, 720

ARUCO_DICT = "DICT_4X4_50"
MARKER_LENGTH_M = 0.05  # 5cm. Must match your printed marker size.

HOME_ID = 0
ROBOT_ID = 2
TARGET_ID = 1

# Calibration .npz containing camera_matrix + dist_coeffs (recommended)
CALIB_NPZ = None  # e.g. "calibration.npz"

# UDP to Simulink
SIMULINK_IP = "127.0.0.1"  # change to Simulink PC IP
SIMULINK_PORT = 25000      # change to Simulink UDP Receive port
SEND_HZ = 30.0             # send rate while running

# When markers are lost, keep sending last known values (usually better than NaNs for control)
HOLD_LAST_ON_LOSS = True


class App:
    def __init__(self, root: tk.Tk):
        self.root = root

        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_addr = (SIMULINK_IP, SIMULINK_PORT)

        # Load calibration
        cam_mtx = dist = None
        if CALIB_NPZ:
            try:
                cam_mtx, dist = ArucoTracker.load_calibration_npz(CALIB_NPZ)
            except Exception as e:
                messagebox.showwarning("Calibration", f"Failed to load calibration: {e}\nContinuing without pose.")

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

        # close handler
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
        self.tick()  # start update loop

    def start(self):
        self.running = True
        self.gui.set_status("Status: RUNNING (sending UDP)")

    def stop(self):
        self.running = False
        self.gui.set_status("Status: STOPPED")

    def on_manual_cmd(self, cmd: ManualCmd):
        # Placeholder: manual command path to Simulink or to a separate channel
        # If you decide manual should override target tracking, you can change behavior here.
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
        robot_pose, target_xy = self.tracker.compute_packet(poses_cam, robot_id=ROBOT_ID, target_id=TARGET_ID)

        if robot_pose.valid and target_xy.valid:
            packet = (robot_pose.x, robot_pose.y, robot_pose.yaw_deg, target_xy.x, target_xy.y)
            self.last_packet = packet
        else:
            if not HOLD_LAST_ON_LOSS:
                packet = (np.nan, np.nan, np.nan, np.nan, np.nan)
                self.last_packet = packet
            else:
                packet = self.last_packet

        # GUI text
        rx, ry, ryaw, tx, ty = packet
        self.gui.set_robot_text(f"Robot (rel HOME {HOME_ID}, ID {ROBOT_ID}): x={rx:.3f} y={ry:.3f} yaw={ryaw:.1f}°")
        self.gui.set_target_text(f"Target (rel HOME {HOME_ID}, ID {TARGET_ID}): x={tx:.3f} y={ty:.3f}")

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
