# gui_mission_control.py
"""
GUI module (no vision, no UDP):
- Buttons: Activate Camera, Start, Stop
- Video display area
- Text labels (robot pose + target)
- Keyboard manual control: Up/Down/Left/Right + WASD

GUI calls callbacks you pass in from main.py.

Dependencies:
  pip install pillow
"""

from __future__ import annotations
from dataclasses import dataclass
from typing import Callable, Optional, Dict

import tkinter as tk
from tkinter import ttk

import cv2
from PIL import Image, ImageTk


@dataclass
class ManualCmd:
    v: float
    w: float


class MissionControlGUI:
    def __init__(
        self,
        root: tk.Tk,
        on_activate_camera: Callable[[], None],
        on_start: Callable[[], None],
        on_stop: Callable[[], None],
        on_manual_cmd: Callable[[ManualCmd], None],
        linear_speed: float = 0.25,
        angular_speed: float = 1.0,
    ):
        self.root = root
        self.root.title("Mission Control")

        self.on_activate_camera = on_activate_camera
        self.on_start = on_start
        self.on_stop = on_stop
        self.on_manual_cmd = on_manual_cmd

        self.linear_speed = linear_speed
        self.angular_speed = angular_speed

        self.status_var = tk.StringVar(value="Status: Idle")
        self.robot_var = tk.StringVar(value="Robot: -")
        self.target_var = tk.StringVar(value="Target: -")
        self.fps_var = tk.StringVar(value="FPS: -")
        self.manual_var = tk.StringVar(value="Manual: v=0.00 w=0.00 (Up/Down/Left/Right or WASD)")

        self.keys_down: Dict[str, bool] = {
            "Up": False, "Down": False, "Left": False, "Right": False,
            "w": False, "s": False, "a": False, "d": False
        }

        self._build_ui()

        # keyboard
        self.root.bind("<KeyPress>", self._on_key_press)
        self.root.bind("<KeyRelease>", self._on_key_release)
        self.root.focus_set()

        # drive manual commands at ~20 Hz
        self._manual_tick()

    def _build_ui(self):
        container = ttk.Frame(self.root, padding=10)
        container.grid(row=0, column=0, sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        controls = ttk.Frame(container)
        controls.grid(row=0, column=0, sticky="ew")
        container.columnconfigure(0, weight=1)

        ttk.Button(controls, text="Activate Camera", command=self.on_activate_camera).grid(row=0, column=0, padx=5, pady=5)
        ttk.Button(controls, text="Start", command=self.on_start).grid(row=0, column=1, padx=5, pady=5)
        ttk.Button(controls, text="Stop", command=self.on_stop).grid(row=0, column=2, padx=5, pady=5)

        ttk.Label(controls, textvariable=self.status_var).grid(row=0, column=3, padx=10)

        # video
        video_frame = ttk.Frame(container)
        video_frame.grid(row=1, column=0, sticky="nsew", pady=(10, 0))
        container.rowconfigure(1, weight=1)

        self.video_label = ttk.Label(video_frame, text="Camera feed will appear here")
        self.video_label.grid(row=0, column=0, sticky="nsew")
        video_frame.rowconfigure(0, weight=1)
        video_frame.columnconfigure(0, weight=1)

        # bottom text
        bottom = ttk.Frame(container)
        bottom.grid(row=2, column=0, sticky="ew", pady=(10, 0))
        bottom.columnconfigure(0, weight=1)

        ttk.Label(bottom, textvariable=self.robot_var).grid(row=0, column=0, sticky="w")
        ttk.Label(bottom, textvariable=self.target_var).grid(row=1, column=0, sticky="w")
        ttk.Label(bottom, textvariable=self.manual_var).grid(row=2, column=0, sticky="w")
        ttk.Label(bottom, textvariable=self.fps_var).grid(row=0, column=1, sticky="e", padx=10)

    # ==== GUI update methods called by main ====
    def set_status(self, text: str) -> None:
        self.status_var.set(text)

    def set_robot_text(self, text: str) -> None:
        self.robot_var.set(text)

    def set_target_text(self, text: str) -> None:
        self.target_var.set(text)

    def set_fps(self, fps: float) -> None:
        self.fps_var.set(f"FPS: {fps:.1f}")

    def set_frame(self, frame_bgr) -> None:
        # Convert to Tk image
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(frame_rgb)
        imgtk = ImageTk.PhotoImage(image=img)
        self.video_label.imgtk = imgtk
        self.video_label.configure(image=imgtk)

    # ==== keyboard -> manual command ====
    def _on_key_press(self, event: tk.Event):
        k = event.keysym
        if k in self.keys_down:
            self.keys_down[k] = True
        kl = k.lower()
        if kl in self.keys_down:
            self.keys_down[kl] = True

    def _on_key_release(self, event: tk.Event):
        k = event.keysym
        if k in self.keys_down:
            self.keys_down[k] = False
        kl = k.lower()
        if kl in self.keys_down:
            self.keys_down[kl] = False

    def _manual_tick(self):
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

        self.manual_var.set(f"Manual: v={v:.2f} w={w:.2f} (Up/Down/Left/Right or WASD)")
        self.on_manual_cmd(ManualCmd(v=v, w=w))

        self.root.after(50, self._manual_tick)  # ~20 Hz
