"""Vision subsystem that mirrors the original vision_test_1_onboard_udp.py logic."""

from __future__ import annotations

import math
import os
import time
from threading import Event, Lock, Thread
from typing import Dict, Optional, Sequence, Tuple

import cv2
import cv2.aruco as aruco
import numpy as np

from IDESA_State import IDESAState, IDESAStateStore
from IDESA_Types import TargetTrack


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
        camera_index: int = 1,
        marker_size_mm: float = 100.0,
        dict_name: str = "DICT_4X4_50",
        robot_id: int = 1,
        target_id: int = 3,
        display_preview: bool = True,
        frame_width: Optional[int] = None,
        frame_height: Optional[int] = None,
        share_preview_frames: bool = True,
        preview_scale: float = 1.0,
    ) -> None:
        self._state_store = state_store
        self._camera_index = camera_index
        self._marker_size_mm = marker_size_mm
        self._dict_name = dict_name
        self._robot_id = robot_id
        self._display_preview = display_preview
        self._frame_width = frame_width
        self._frame_height = frame_height
        self._share_preview_frames = share_preview_frames
        self._preview_window_name = "Vision - Onboard UDP"
        try:
            preview_scale = float(preview_scale)
        except (TypeError, ValueError):
            preview_scale = 1.0
        self._preview_scale = preview_scale if preview_scale > 0.0 else 1.0

        self._cap: Optional[cv2.VideoCapture] = None
        self._cap_lock = Lock()

        self._tracking_thread: Optional[Thread] = None
        self._stop_event = Event()

        self._ids_lock = Lock()
        self._tracked_target_ids: Tuple[int, ...] = ()
        self._targets_by_id: Dict[int, TargetTrack] = {}

        self._preview_lock = Lock()
        self._latest_frame: Optional[np.ndarray] = None

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

        # Seed initial target selection (can be replaced later via GUI).
        self.set_tracked_target_ids([target_id])

    # ------------------------------------------------------------------
    # Camera + tracking lifecycle
    # ------------------------------------------------------------------
    def start_camera(self) -> None:
        with self._cap_lock:
            if self._cap and self._cap.isOpened():
                return
            cap = cv2.VideoCapture(self._camera_index)
            if self._frame_width:
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(self._frame_width))
            if self._frame_height:
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self._frame_height))
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

    def is_tracking(self) -> bool:
        thread = self._tracking_thread
        return bool(thread and thread.is_alive())

    # ------------------------------------------------------------------
    # Shared state helpers
    # ------------------------------------------------------------------
    def get_state_snapshot(self) -> IDESAState:
        return self._state_store.snapshot()

    def set_robot_id(self, robot_id: int) -> None:
        with self._ids_lock:
            self._robot_id = int(robot_id)

    def set_target_id(self, target_id: int) -> Tuple[int, ...]:
        """Backward-compatible helper that selects a single target."""
        return self.set_tracked_target_ids([int(target_id)])

    def set_tracked_target_ids(self, target_ids: Sequence[int]) -> Tuple[int, ...]:
        cleaned = tuple(dict.fromkeys(int(tid) for tid in target_ids if tid is not None))
        with self._ids_lock:
            previous = self._targets_by_id
            new_tracks: Dict[int, TargetTrack] = {}
            for tid in cleaned:
                new_tracks[tid] = previous.get(tid, TargetTrack())
            self._targets_by_id = new_tracks
            self._tracked_target_ids = cleaned
            return self._tracked_target_ids

    def get_tracked_target_ids(self) -> Tuple[int, ...]:
        with self._ids_lock:
            return self._tracked_target_ids

    def _prepare_preview_frame(self, frame: np.ndarray) -> np.ndarray:
        scale = self._preview_scale
        if not scale or abs(scale - 1.0) < 1e-3:
            return frame
        interpolation = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
        return cv2.resize(frame, dsize=None, fx=scale, fy=scale, interpolation=interpolation)

    # ------------------------------------------------------------------
    # Internal vision loop
    # ------------------------------------------------------------------
    def _tracking_loop(self) -> None:
        last_robot_x = 0.0
        last_robot_y = 0.0
        last_robot_yaw = 0.0
        fps = 0.0
        last_frame_t = 0.0

        while not self._stop_event.is_set():
            with self._ids_lock:
                robot_id = self._robot_id
                tracked_ids = tuple(self._tracked_target_ids)

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

            overlay_enabled = self._display_preview or self._share_preview_frames
            annotated = frame.copy() if overlay_enabled else frame

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = aruco.detectMarkers(gray, self._aruco_dict, parameters=self._parameters)

            robot_detected = False
            target_updates: Dict[int, TargetTrack] = {}

            if ids is not None and len(ids) > 0:
                ids_flat = ids.flatten().astype(int)
                if overlay_enabled:
                    aruco.drawDetectedMarkers(annotated, corners, ids)
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    corners, self._marker_size_mm, self._camera_matrix, self._dist_coef
                )

                for i, marker_id in enumerate(ids_flat):
                    rvec = rvecs[i].reshape(3, 1)
                    tvec = tvecs[i].reshape(3, 1)
                    x = float(tvec[0, 0])
                    y = float(tvec[1, 0])

                    if overlay_enabled:
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

                    elif marker_id in tracked_ids:
                        target_updates[marker_id] = TargetTrack(
                            x_mm=x,
                            y_mm=y,
                            detected=True,
                            last_seen_time=now,
                        )

            overlay_summary: Tuple[Tuple[int, TargetTrack], ...] = ()
            targets_snapshot: Dict[int, TargetTrack] = {}
            if tracked_ids:
                with self._ids_lock:
                    for tid, track in target_updates.items():
                        self._targets_by_id[tid] = track
                    summary_list = []
                    for tid in tracked_ids:
                        current = self._targets_by_id.get(tid)
                        if tid not in target_updates:
                            if current:
                                self._targets_by_id[tid] = TargetTrack(
                                    x_mm=current.x_mm,
                                    y_mm=current.y_mm,
                                    detected=False,
                                    last_seen_time=current.last_seen_time,
                                )
                            else:
                                self._targets_by_id[tid] = TargetTrack()
                        summary_list.append((tid, self._targets_by_id[tid]))
                    targets_snapshot = {tid: self._targets_by_id[tid] for tid in tracked_ids}
                    overlay_summary = tuple(summary_list)

            self._state_store.update(
                robot_x_mm=last_robot_x,
                robot_y_mm=last_robot_y,
                robot_yaw_deg=last_robot_yaw,
                robot_detected=robot_detected,
                fps=fps,
                last_update_time=now,
                targets_by_id=targets_snapshot,
            )

            if overlay_enabled:
                robot_text = (
                    f"Robot (ID {robot_id}): yaw={last_robot_yaw:6.1f}\u00b0  x={last_robot_x:8.2f}  y={last_robot_y:8.2f}  [mm]"
                    if robot_detected
                    else f"Robot (ID {robot_id}): NOT DETECTED (sending last)"
                )
                if overlay_summary:
                    summary_bits = []
                    for tid, track in overlay_summary:
                        flag = "Y" if track.detected else "n"
                        summary_bits.append(f"{tid}:{flag}")
                    target_text = "Targets " + ", ".join(summary_bits)
                else:
                    target_text = "Targets: none selected"
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

                if self._display_preview:
                    preview_frame = self._prepare_preview_frame(annotated)
                    cv2.imshow(self._preview_window_name, preview_frame)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        self._stop_event.set()

            if self._share_preview_frames:
                with self._preview_lock:
                    self._latest_frame = annotated.copy() if overlay_enabled else frame.copy()

        # ensure we leave things in a known state when loop exits
        self._state_store.update(last_update_time=time.time())
        if self._display_preview:
            try:
                cv2.destroyWindow(self._preview_window_name)
            except cv2.error:
                pass

    def get_latest_frame(self) -> Optional[np.ndarray]:
        with self._preview_lock:
            if self._latest_frame is None:
                return None
            return self._latest_frame.copy()
