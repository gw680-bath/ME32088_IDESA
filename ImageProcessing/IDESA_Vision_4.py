"""
IDESA_Vision_4.py
OpenCV ArUco vision system:
- Detects robot ID=1 and GUI-selected targets (2–7)
- Updates shared state with positions (mm) + yaw (deg) + last seen timestamps
- Preview window shows full FOV, labels markers with ID and yaw, highlights robot + active target,
  and draws a heading arrow for robot.
"""

from __future__ import annotations

import math
import time
from pathlib import Path
from threading import Event, Lock, Thread
from typing import Iterable

import cv2
import cv2.aruco as aruco
import numpy as np


def _load_calibration_npz() -> tuple[np.ndarray, np.ndarray]:
    # Look for Calibration.npz in common locations (same approach as your v2 stack)
    here = Path(__file__).resolve().parent
    candidates = [
        here / "Calibration.npz",
        here.parent / "Calibration.npz",
        Path.cwd() / "Calibration.npz",
    ]
    calib = next((p for p in candidates if p.exists()), None)
    if not calib:
        raise FileNotFoundError(
            "Calibration.npz not found. Place it next to the scripts or project root."
        )
    data = np.load(str(calib))
    return data["CM"], data["dist_coef"]


def yaw_deg_from_rvec(rvec: np.ndarray) -> float:
    R, _ = cv2.Rodrigues(rvec.reshape(3, 1))
    yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    return float(yaw)


class VisionSystem4:
    def __init__(
        self,
        state,
        state_lock: Lock,
        camera_index: int | str = 1,
        marker_size_mm: float = 100.0,
        dict_name: str = "DICT_4X4_50",
        preview_window_name: str = "IDESA Camera",
        preferred_res: tuple[int, int] = (1280, 720),
    ) -> None:
        self.state = state
        self.lock = state_lock
        self.camera_index = camera_index
        self.marker_size_mm = float(marker_size_mm)
        self.preview_window_name = preview_window_name
        self.preferred_res = preferred_res
        self._camera_source_variants = self._build_camera_source_variants(camera_index)
        self._backend_priority: tuple[int, ...] = (
            getattr(cv2, "CAP_DSHOW", cv2.CAP_ANY),
            getattr(cv2, "CAP_MSMF", cv2.CAP_ANY),
            cv2.CAP_ANY,
        )
        self._active_backend: int | None = None
        self._active_source: int | str | None = None

        if not hasattr(aruco, dict_name):
            raise ValueError(f"Unknown ArUco dict: {dict_name}")
        self.aruco_dict = aruco.getPredefinedDictionary(getattr(aruco, dict_name))
        self.parameters = aruco.DetectorParameters()

        self.CM, self.dist = _load_calibration_npz()

        self._thread: Thread | None = None
        self._stop = Event()

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = Thread(target=self._loop, name="VisionSystem4", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        if not self._thread:
            return
        self._stop.set()
        self._thread.join(timeout=2.0)
        self._thread = None
        try:
            cv2.destroyWindow(self.preview_window_name)
        except Exception:
            pass
        with self.lock:
            self.state.robot_visible = False
            self.state.robot_xy_mm = None

    def _loop(self) -> None:
        cap = self._open_camera_capture()

        # Try set wide resolution
        w, h = self.preferred_res
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(w))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(h))

        while not self._stop.is_set():
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.05)
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _rej = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

            now = time.time()

            # Defaults each frame: keep last_seen times but update visibility based on detections
            robot_detected_this_frame = False
            robot_xy = None
            robot_yaw = 0.0
            detected_targets: dict[int, tuple[float, float]] = {}

            if ids is not None and len(ids) > 0:
                frame = aruco.drawDetectedMarkers(frame, corners, ids)
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    corners, self.marker_size_mm, self.CM, self.dist
                )

                # Draw axes and labels
                for i, (mid,) in enumerate(ids):
                    rvec = rvecs[i]
                    tvec = tvecs[i]

                    cv2.drawFrameAxes(frame, self.CM, self.dist, rvec, tvec, 60)
                    yaw = yaw_deg_from_rvec(rvec)

                    # Position in mm (tvec returned in same units as marker size)
                    x_mm = float(tvec[0][0])
                    y_mm = float(tvec[0][1])

                    # Label near marker center
                    c = corners[i][0]
                    cx = int(c[:, 0].mean())
                    cy = int(c[:, 1].mean())
                    cv2.putText(
                        frame,
                        f"ID {mid}  yaw {yaw:+.1f}",
                        (cx - 40, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (255, 255, 255),
                        2,
                    )

                    if int(mid) == int(getattr(self.state, "robot_id", 1)):
                        robot_detected_this_frame = True
                        robot_xy = (x_mm, y_mm)
                        robot_yaw = yaw
                    else:
                        # keep only GUI-selected targets
                        with self.lock:
                            sel = set(self.state.selected_targets)
                        if int(mid) in sel:
                            detected_targets[int(mid)] = (x_mm, y_mm)

                # Heading arrow for robot (draw in image space using pose axis projection)
                if robot_detected_this_frame:
                    # Find robot index again to use its rvec/tvec for projected arrow
                    idx = None
                    robot_id = int(getattr(self.state, "robot_id", 1))
                    for i, (mid,) in enumerate(ids):
                        if int(mid) == robot_id:
                            idx = i
                            break
                    if idx is not None:
                        rvec = rvecs[idx]
                        tvec = tvecs[idx]
                        # project two 3D points: origin and a forward point on marker x-axis
                        pts3 = np.float32([[0, 0, 0], [80, 0, 0]]).reshape(-1, 3)
                        imgpts, _ = cv2.projectPoints(pts3, rvec, tvec, self.CM, self.dist)
                        p0 = tuple(imgpts[0].ravel().astype(int))
                        p1 = tuple(imgpts[1].ravel().astype(int))
                        cv2.arrowedLine(frame, p0, p1, (0, 255, 255), 3, tipLength=0.2)

            # Update shared state
            with self.lock:
                # Active target highlight needs active_target_id from Navigation
                active_id = self.state.active_target_id

                if robot_detected_this_frame and robot_xy is not None:
                    self.state.robot_visible = True
                    self.state.robot_xy_mm = robot_xy
                    self.state.robot_yaw_deg = float(robot_yaw)
                    self.state.robot_last_seen = now
                else:
                    # do not wipe last seen; just mark not visible
                    self.state.robot_visible = False

                # Update targets dictionaries and timestamps
                for tid, xy in detected_targets.items():
                    self.state.targets_xy_mm[tid] = xy
                    self.state.targets_last_seen[tid] = now

            # Highlight robot + active target with rectangles (image-space)
            if ids is not None and len(ids) > 0:
                for i, (mid,) in enumerate(ids):
                    mid = int(mid)
                    c = corners[i][0]
                    x_min, y_min = int(c[:, 0].min()), int(c[:, 1].min())
                    x_max, y_max = int(c[:, 0].max()), int(c[:, 1].max())

                    if mid == int(getattr(self.state, "robot_id", 1)):
                        cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 3)
                    elif active_id is not None and mid == int(active_id):
                        cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (255, 0, 0), 3)

            cv2.imshow(self.preview_window_name, frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                # allow quitting preview with 'q'
                break

        cap.release()
        try:
            cv2.destroyWindow(self.preview_window_name)
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Camera helpers
    # ------------------------------------------------------------------
    def _build_camera_source_variants(self, camera_index: int | str) -> tuple[int | str, ...]:
        if isinstance(camera_index, str):
            normalized = camera_index.strip()
            variants: list[int | str] = []
            if normalized and not normalized.lower().startswith("video="):
                variants.append(f"video={normalized}")
            variants.append(normalized)
            return tuple(variants)
        return (int(camera_index),)

    def _open_camera_capture(self) -> cv2.VideoCapture:
        attempts: list[str] = []
        for source in self._camera_source_variants:
            for backend in self._backend_priority:
                cap = cv2.VideoCapture(source, backend)
                if cap.isOpened():
                    self._active_backend = backend
                    self._active_source = source
                    backend_name = self._backend_name(backend)
                    print(
                        f"[VisionSystem4] Using camera source {source!r} via {backend_name}."
                    )
                    return cap
                cap.release()
                attempts.append(f"{source!r} via {self._backend_name(backend)}")
        raise RuntimeError(
            "Could not open any camera source. Tried: " + ", ".join(attempts)
        )

    @staticmethod
    def _backend_name(backend: int | None) -> str:
        mapping = {
            getattr(cv2, "CAP_DSHOW", -1): "CAP_DSHOW",
            getattr(cv2, "CAP_MSMF", -1): "CAP_MSMF",
            getattr(cv2, "CAP_V4L2", -1): "CAP_V4L2",
            getattr(cv2, "CAP_ANY", -1): "CAP_ANY",
        }
        return mapping.get(backend, str(backend))
