# gui_mission_control.py
"""
Mission control GUI (Tkinter) that:
- Activates camera
- Shows live camera view in the GUI
- Displays ArUco pose relative to a home marker (if calibrated)
- Start/Stop mission state
- Keyboard manual control (Up/Down/Left/Right + WASD)

Run:
  python gui_mission_control.py

Dependencies:
  pip install pillow numpy opencv-contrib-python
"""

from __future__ import annotations
import time
from typing import Optional, Dict

import tkinter as tk
from tkinter import ttk, messagebox

import cv2
import numpy as np
from PIL import Image, ImageTk

from vision_aruco import ArucoTracker, MarkerDetection


class MissionControlGUI:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Mission Control")

        # ====== CONFIG (edit these) ======
        self.camera_index = 0
        self.frame_w = 1280
        self.frame_h = 720

        self.home_marker_id = 0
        self.marker_length_m = 0.05  # must match printed marker side length

        # If you have calibration, set path here (npz with camera_matrix, dist_coeffs)
        self.calib_npz_path: Optional[str] = None  # e.g. "calibration.npz"

        # Manual control tuning
        self.linear_speed = 0.25    # units are up to you (e.g. m/s)
        self.angular_speed = 1.0    # rad/s (or deg/s), up to you

        # ====== state ======
        self.mission_running = False
        self.last_frame_time = 0.0
        self.fps = 0.0

        self.keys_down: Dict[str, bool] = {"Up": False, "Down": False, "Left": False, "Right": False,
                                           "w": False, "s": False, "a": False, "d": False}

        # ====== Vision tracker ======
        camera_matrix = dist_coeffs = None
        if self.calib_npz_path:
            try:
                camera_matrix, dist_coeffs = ArucoTracker.load_calibration_npz(self.calib_npz_path)
            except Exception as e:
                messagebox.showwarning("Calibration load failed", f"Could not load calibration: {e}")

        self.tracker = ArucoTracker(
            camera_index=self.camera_index,
            width=self.frame_w,
            height=self.frame_h,
            aruco_dict_name="DICT_4X4_50",
            home_id=self.home_marker_id,
            marker_length=self.marker_length_m,
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
        )

        # ====== UI Layout ======
        self._build_ui()

        # Keyboard bindings (must focus root)
        self.root.bind("<KeyPress>", self._on_key_press)
        self.root.bind("<KeyRelease>", self._on_key_release)
        self.root.focus_set()

        # Periodic tasks
        self._schedule_manual_control_tick()

        # Clean exit
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self):
        container = ttk.Frame(self.root, padding=10)
        container.grid(row=0, column=0, sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        # Top controls
        controls = ttk.Frame(container)
        controls.grid(row=0, column=0, sticky="ew")
        container.columnconfigure(0, weight=1)

        self.btn_activate = ttk.Button(controls, text="Activate Camera", command=self.activate_camera)
        self.btn_activate.grid(row=0, column=0, padx=5, pady=5)

        self.btn_start = ttk.Button(controls, text="Start", command=self.start_mission)
        self.btn_start.grid(row=0, column=1, padx=5, pady=5)

        self.btn_stop = ttk.Button(controls, text="Stop", command=self.stop_mission)
        self.btn_stop.grid(row=0, column=2, padx=5, pady=5)

        self.status_var = tk.StringVar(value="Status: Idle")
        ttk.Label(controls, textvariable=self.status_var).grid(row=0, column=3, padx=10)

        # Video display
        video_frame = ttk.Frame(container)
        video_frame.grid(row=1, column=0, sticky="nsew", pady=(10, 0))
        container.rowconfigure(1, weight=1)

        self.video_label = ttk.Label(video_frame, text="Camera feed will appear here")
        self.video_label.grid(row=0, column=0, sticky="nsew")
        video_frame.rowconfigure(0, weight=1)
        video_frame.columnconfigure(0, weight=1)

        # Pose readout / manual control hint
        bottom = ttk.Frame(container)
        bottom.grid(row=2, column=0, sticky="ew", pady=(10, 0))

        self.pose_var = tk.StringVar(value="Pose: (no detections)")
        ttk.Label(bottom, textvariable=self.pose_var).grid(row=0, column=0, sticky="w")

        self.ctrl_var = tk.StringVar(value="Manual: Up/Down/Left/Right or WASD")
        ttk.Label(bottom, textvariable=self.ctrl_var).grid(row=1, column=0, sticky="w")

        self.fps_var = tk.StringVar(value="FPS: -")
        ttk.Label(bottom, textvariable=self.fps_var).grid(row=0, column=1, sticky="e", padx=10)
        bottom.columnconfigure(0, weight=1)
        bottom.columnconfigure(1, weight=0)

    # ====== Button actions ======

    def activate_camera(self):
        if self.tracker.is_running():
            self.status_var.set("Status: Camera already active")
            return
        try:
            self.tracker.start()
        except Exception as e:
            messagebox.showerror("Camera error", str(e))
            return

        self.status_var.set("Status: Camera active")
        self._update_video_loop()

    def start_mission(self):
        self.mission_running = True
        self.status_var.set("Status: RUNNING")
        # Here you would also start your mission logic / comms loop.

    def stop_mission(self):
        self.mission_running = False
        self.status_var.set("Status: STOPPED")
        # Here you would send a stop command to the robot / state machine.

    # ====== Video + CV update loop ======

    def _update_video_loop(self):
        if not self.tracker.is_running():
            return

        try:
            frame_bgr, detections = self.tracker.read()
        except Exception as e:
            self.status_var.set(f"Status: Camera read failed: {e}")
            return

        # FPS estimate
        now = time.time()
        if self.last_frame_time > 0:
            dt = now - self.last_frame_time
            if dt > 0:
                self.fps = 0.9 * self.fps + 0.1 * (1.0 / dt) if self.fps > 0 else (1.0 / dt)
        self.last_frame_time = now
        self.fps_var.set(f"FPS: {self.fps:.1f}")

        # Update pose readout (prefer relative pose if available)
        self._update_pose_label(detections, frame_bgr.shape[1], frame_bgr.shape[0])

        # Convert frame to Tk image
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(frame_rgb)
        imgtk = ImageTk.PhotoImage(image=img)

        # Keep a reference, otherwise Tk may garbage-collect it
        self.video_label.imgtk = imgtk
        self.video_label.configure(image=imgtk)

        # Schedule next update (~30-60fps). Adjust as needed.
        self.root.after(15, self._update_video_loop)

    def _update_pose_label(self, detections, w, h):
        if not detections:
            self.pose_var.set("Pose: (no detections)")
            return

        # Pick a non-home marker to display if possible, else display home itself
        non_home = [d for d in detections if d.marker_id != self.home_marker_id]
        d = non_home[0] if non_home else detections[0]

        # If we have rel pose (needs calibration + home marker in view)
        if d.rel_tvec is not None:
            x, y, z = d.rel_tvec.flatten().tolist()
            yaw = d.rel_yaw_deg
            self.pose_var.set(
                f"Pose (rel to home ID {self.home_marker_id}): "
                f"ID {d.marker_id}  x={x:.3f}  y={y:.3f}  z={z:.3f}  yaw={yaw:.1f}°"
            )
            return

        # Otherwise fall back to pixel coords, with origin at bottom-left of the *image*
        # (bottom-left = (0, h) in image coordinates if you invert y)
        cx, cy = d.center_px
        x_px = cx
        y_px = (h - cy)  # invert so "up" in image increases y
        self.pose_var.set(
            f"Pose (pixels, origin bottom-left of image): "
            f"ID {d.marker_id}  x={x_px:.1f}px  y={y_px:.1f}px"
            "  [Tip: add calibration + a home marker to get real-world coords]"
        )

    # ====== Manual keyboard control ======

    def _on_key_press(self, event: tk.Event):
        key = event.keysym
        if key in self.keys_down:
            self.keys_down[key] = True
        # WASD as well (keysym is lower-case on most systems, but not always)
        if key.lower() in self.keys_down:
            self.keys_down[key.lower()] = True

    def _on_key_release(self, event: tk.Event):
        key = event.keysym
        if key in self.keys_down:
            self.keys_down[key] = False
        if key.lower() in self.keys_down:
            self.keys_down[key.lower()] = False

    def _schedule_manual_control_tick(self):
        # Run at ~20Hz
        self._manual_control_tick()
        self.root.after(50, self._schedule_manual_control_tick)

    def _manual_control_tick(self):
        """
        Converts keys into a velocity command.
        Replace `self.send_manual_command(v, w)` with your UDP / motor interface.
        """
        # Only send manual control when mission is running (change if you want)
        if not self.mission_running:
            return

        forward = self.keys_down["Up"] or self.keys_down["w"]
        back = self.keys_down["Down"] or self.keys_down["s"]
        left = self.keys_down["Left"] or self.keys_down["a"]
        right = self.keys_down["Right"] or self.keys_down["d"]

        v = 0.0
        w = 0.0

        if forward and not back:
            v = self.linear_speed
        elif back and not forward:
            v = -self.linear_speed

        if left and not right:
            w = self.angular_speed
        elif right and not left:
            w = -self.angular_speed

        # Update label for visibility
        self.ctrl_var.set(f"Manual: v={v:.2f}, w={w:.2f}   (Up/Down/Left/Right or WASD)")

        # Send command to robot (currently a placeholder)
        self.send_manual_command(v, w)

    def send_manual_command(self, v: float, w: float):
        """
        TODO: Replace with your actual robot command pipeline.
        Example options:
          - UDP packet to Raspberry Pi
          - Write to a shared variable consumed by your control loop
          - Call into a motor controller module

        For now, we just print occasionally (comment out if spammy).
        """
        # print(f"[MANUAL] v={v:.2f}, w={w:.2f}")
        pass

    # ====== cleanup ======

    def _on_close(self):
        try:
            self.stop_mission()
            self.tracker.stop()
        finally:
            self.root.destroy()


def main():
    root = tk.Tk()
    # nicer default styling on some platforms
    try:
        ttk.Style().theme_use("clam")
    except Exception:
        pass

    app = MissionControlGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
