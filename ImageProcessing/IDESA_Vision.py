"""Vision subsystem that mirrors the original vision_test_1_onboard_udp.py logic."""

from __future__ import annotations

import math
import os
import time
from threading import Event, Lock, Thread
from typing import Optional, Tuple

import cv2
import cv2.aruco as aruco
import numpy as np

from IDESA_State import IDESAStateStore, IDESAState


def yaw_deg_from_rvec(rvec: np.ndarray) -> float:
    """Convert a rotation vector into a yaw angle in degrees."""
    R, _ = cv2.Rodrigues(rvec.reshape(3, 1))
    yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    return float(yaw)


def load_calibration_npz(npz_path: str):
    data = np.load(npz_path)
    return data["CM"], data["dist_coef"]


class VisionSystem:
    """Encapsulates camera capture, ArUco detection, and state updates."""

    def __init__(
        self,
        state_store: IDESAStateStore,
        camera_index: int = 0,
        marker_size_mm: float = 40.0,
        dict_name: str = "DICT_4X4_50",
        robot_id: int = 1,
        target_id: int = 3,
        display_preview: bool = True,
    ) -> None:
        self._state_store = state_store
        self._camera_index = camera_index
        self._marker_size_mm = marker_size_mm
        self._dict_name = dict_name
        self._robot_id = robot_id
        self._target_id = target_id
        self._ids_lock = Lock()
        self._display_preview = display_preview
        self._preview_window_name = "Vision - Onboard UDP"

        self._cap: Optional[cv2.VideoCapture] = None
        self._cap_lock = Lock()

        self._tracking_thread: Optional[Thread] = None
        self._stop_event = Event()

        script_dir = os.path.dirname(os.path.abspath(__file__))
        calib_path = os.path.join(script_dir, "Calibration.npz")
        if not os.path.exists(calib_path):
            raise FileNotFoundError(
                f"Calibration file not found: {calib_path}\n"
                "Place Calibration.npz (with arrays 'CM' and 'dist_coef') next to IDESA_Vision.py."
            )
        self._camera_matrix, self._dist_coef = load_calibration_npz(calib_path)

        if not hasattr(aruco, dict_name):
            raise ValueError(f"Unknown ArUco dictionary: {dict_name}")
        self._aruco_dict = aruco.getPredefinedDictionary(getattr(aruco, dict_name))
        self._parameters = aruco.DetectorParameters()

    def start_camera(self) -> None:
        with self._cap_lock:
            if self._cap and self._cap.isOpened():
                return
            cap = cv2.VideoCapture(self._camera_index)
            if not cap.isOpened():
                cap.release()
                raise RuntimeError(f"Could not open camera index {self._camera_index}")
            self._cap = cap

    def stop_camera(self) -> None:
        with self._cap_lock:
            if self._cap:
                try:
                    self._cap.release()
                except Exception:
                    pass
                self._cap = None

    def start_tracking(self) -> None:
        if self._tracking_thread and self._tracking_thread.is_alive():
            return
        self.start_camera()
        self._stop_event.clear()
        self._tracking_thread = Thread(target=self._tracking_loop, name="VisionTracking", daemon=True)
        self._tracking_thread.start()

    def stop_tracking(self) -> None:
        if not self._tracking_thread:
            return
        self._stop_event.set()
        self._tracking_thread.join(timeout=2.0)
        self._tracking_thread = None
        self._stop_event.clear()

    def get_state_snapshot(self) -> IDESAState:
        return self._state_store.snapshot()

    def is_tracking(self) -> bool:
        thread = self._tracking_thread
        return bool(thread and thread.is_alive())

    def set_robot_id(self, robot_id: int) -> None:
        with self._ids_lock:
            self._robot_id = int(robot_id)

    def set_target_id(self, target_id: int) -> None:
        with self._ids_lock:
            self._target_id = int(target_id)

    def set_marker_ids(self, robot_id: int, target_id: int) -> None:
        with self._ids_lock:
            self._robot_id = int(robot_id)
            self._target_id = int(target_id)

    def get_marker_ids(self) -> Tuple[int, int]:
        with self._ids_lock:
            return self._robot_id, self._target_id

    def _tracking_loop(self) -> None:
        last_robot_x = 0.0
        last_robot_y = 0.0
        last_robot_yaw = 0.0
        last_target_x = 0.0
        last_target_y = 0.0
        fps = 0.0
        last_frame_t = 0.0

        while not self._stop_event.is_set():
            with self._ids_lock:
                robot_id = self._robot_id
                target_id = self._target_id

            with self._cap_lock:
                cap = self._cap
            if cap is None:
                time.sleep(0.05)
                continue

            ok, frame = cap.read()
            if not ok or frame is None:
                time.sleep(0.005)
                continue

            now = time.time()
            if last_frame_t > 0.0:
                dt = now - last_frame_t
                if dt > 0.0:
                    inst = 1.0 / dt
                    fps = inst if fps == 0.0 else (0.9 * fps + 0.1 * inst)
            last_frame_t = now

            annotated = frame.copy() if self._display_preview else frame

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = aruco.detectMarkers(gray, self._aruco_dict, parameters=self._parameters)

            robot_detected = False
            target_detected = False

            if ids is not None and len(ids) > 0:
                ids_flat = ids.flatten().astype(int)
                if self._display_preview:
                    aruco.drawDetectedMarkers(annotated, corners, ids)
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    corners, self._marker_size_mm, self._camera_matrix, self._dist_coef
                )

                for i, marker_id in enumerate(ids_flat):
                    rvec = rvecs[i].reshape(3, 1)
                    tvec = tvecs[i].reshape(3, 1)
                    x = float(tvec[0, 0])
                    y = float(tvec[1, 0])

                    if self._display_preview:
                        cv2.drawFrameAxes(
                            annotated,
                            self._camera_matrix,
                            self._dist_coef,
                            rvec,
                            tvec,
                            self._marker_size_mm * 0.75,
                        )

                    if marker_id == robot_id:
                        last_robot_yaw = yaw_deg_from_rvec(rvec)
                        last_robot_x = x
                        last_robot_y = y
                        robot_detected = True

                    if marker_id == target_id:
                        last_target_x = x
                        last_target_y = y
                        target_detected = True

            self._state_store.update(
                robot_x_mm=last_robot_x,
                robot_y_mm=last_robot_y,
                robot_yaw_deg=last_robot_yaw,
                target_x_mm=last_target_x,
                target_y_mm=last_target_y,
                robot_detected=robot_detected,
                target_detected=target_detected,
                fps=fps,
                last_update_time=now,
            )

            if self._display_preview:
                robot_text = (
                    f"Robot (ID {robot_id}): yaw={last_robot_yaw:6.1f}\u00b0  x={last_robot_x:8.2f}  y={last_robot_y:8.2f}  [mm]"
                    if robot_detected
                    else f"Robot (ID {robot_id}): NOT DETECTED (sending last)"
                )
                target_text = (
                    f"Target (ID {target_id}): x={last_target_x:8.2f}  y={last_target_y:8.2f}  [mm]"
                    if target_detected
                    else f"Target (ID {target_id}): NOT DETECTED (sending last)"
                )
                cv2.putText(
                    annotated,
                    robot_text,
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (40, 255, 40),
                    2,
                )
                cv2.putText(
                    annotated,
                    target_text,
                    (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (40, 255, 40),
                    2,
                )
                cv2.putText(
                    annotated,
                    f"FPS: {fps:.1f}",
                    (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (40, 255, 40),
                    2,
                )

                cv2.imshow(self._preview_window_name, annotated)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    self._stop_event.set()

        # ensure we leave things in a known state when loop exits
        self._state_store.update(last_update_time=time.time())
        if self._display_preview:
            try:
                cv2.destroyWindow(self._preview_window_name)
            except cv2.error:
                pass
