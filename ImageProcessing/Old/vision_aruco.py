# vision_aruco.py
"""
Vision module (no GUI, no UDP):
- Opens camera
- Detects ArUco markers
- Optionally estimates pose (needs calibration)
- Computes robot pose (x,y,yaw) and target (x,y) relative to a HOME marker

Outputs exactly what your Simulink UDP packet needs: 5 values.

Dependencies:
  pip install numpy opencv-contrib-python
"""

from __future__ import annotations
from dataclasses import dataclass
from typing import Optional, Tuple, List, Dict

import cv2
import numpy as np


@dataclass
class Pose2D:
    x: float
    y: float
    yaw_deg: float
    valid: bool


@dataclass
class XY:
    x: float
    y: float
    valid: bool


def _T_from_rvec_tvec(rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    """Rodrigues rvec + tvec -> 4x4 transform."""
    R, _ = cv2.Rodrigues(rvec.reshape(3, 1))
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = tvec.reshape(3)
    return T


def _yaw_deg_from_R(R: np.ndarray) -> float:
    # A reasonable yaw extraction. Depending on your axis conventions, you may adjust.
    return float(np.degrees(np.arctan2(R[1, 0], R[0, 0])))


class ArucoTracker:
    def __init__(
        self,
        camera_index: int = 0,
        width: int = 1280,
        height: int = 720,
        aruco_dict_name: str = "DICT_4X4_50",
        marker_length: float = 0.05,
        home_id: int = 0,
        camera_matrix: Optional[np.ndarray] = None,
        dist_coeffs: Optional[np.ndarray] = None,
    ):
        self.camera_index = camera_index
        self.width = width
        self.height = height

        self.marker_length = marker_length
        self.home_id = home_id

        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs

        self.cap: Optional[cv2.VideoCapture] = None

        if not hasattr(cv2.aruco, aruco_dict_name):
            raise ValueError(f"Unknown ArUco dictionary: {aruco_dict_name}")

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, aruco_dict_name))
        self.detector_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.detector_params)

    def start(self) -> None:
        if self.cap is not None:
            return
        cap = cv2.VideoCapture(self.camera_index)
        if not cap.isOpened():
            raise RuntimeError(f"Could not open camera index {self.camera_index}")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap = cap

    def stop(self) -> None:
        if self.cap is not None:
            self.cap.release()
            self.cap = None

    def is_running(self) -> bool:
        return self.cap is not None

    @staticmethod
    def load_calibration_npz(npz_path: str) -> Tuple[np.ndarray, np.ndarray]:
        data = np.load(npz_path)
        return data["camera_matrix"], data["dist_coeffs"]

    def read_frame(self) -> np.ndarray:
        if self.cap is None:
            raise RuntimeError("Camera not started. Call start() first.")
        ok, frame = self.cap.read()
        if not ok or frame is None:
            raise RuntimeError("Failed to read frame from camera.")
        return frame

    def detect(self, frame_bgr: np.ndarray) -> Tuple[np.ndarray, Dict[int, Tuple[np.ndarray, np.ndarray]]]:
        """
        Returns:
          annotated_frame_bgr
          poses_cam: dict {id: (rvec(3,1), tvec(3,1))} only if calibration provided
        """
        annotated = frame_bgr.copy()
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)

        corners, ids, _rej = self.detector.detectMarkers(gray)
        poses_cam: Dict[int, Tuple[np.ndarray, np.ndarray]] = {}

        if ids is None or len(ids) == 0:
            return annotated, poses_cam

        ids_flat = ids.flatten().astype(int)
        cv2.aruco.drawDetectedMarkers(annotated, corners, ids)

        have_pose = self.camera_matrix is not None and self.dist_coeffs is not None
        if have_pose:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs
            )
            for i, mid in enumerate(ids_flat):
                rvec = rvecs[i].reshape(3, 1)
                tvec = tvecs[i].reshape(3, 1)
                poses_cam[int(mid)] = (rvec, tvec)

                # draw axes
                cv2.drawFrameAxes(
                    annotated,
                    self.camera_matrix,
                    self.dist_coeffs,
                    rvec,
                    tvec,
                    self.marker_length * 0.5,
                )

        return annotated, poses_cam

    def compute_packet(
        self,
        poses_cam: Dict[int, Tuple[np.ndarray, np.ndarray]],
        robot_id: int,
        target_id: int,
    ) -> Tuple[Pose2D, XY]:
        """
        Compute robot pose and target position relative to home marker.
        Requires calibration AND home marker visible.

        Returns:
          robot_pose: (x,y,yaw_deg,valid)
          target_xy:  (x,y,valid)

        Units:
          Same as marker_length (meters if marker_length is meters).
        """
        have_pose = len(poses_cam) > 0
        if not have_pose:
            return Pose2D(np.nan, np.nan, np.nan, False), XY(np.nan, np.nan, False)

        if self.home_id not in poses_cam:
            return Pose2D(np.nan, np.nan, np.nan, False), XY(np.nan, np.nan, False)

        rvec_h, tvec_h = poses_cam[self.home_id]
        T_cam_home = _T_from_rvec_tvec(rvec_h, tvec_h)
        T_home_cam = np.linalg.inv(T_cam_home)

        # Robot
        if robot_id in poses_cam:
            rvec_r, tvec_r = poses_cam[robot_id]
            T_cam_r = _T_from_rvec_tvec(rvec_r, tvec_r)
            T_home_r = T_home_cam @ T_cam_r
            rx, ry, _rz = T_home_r[:3, 3].tolist()
            yaw = _yaw_deg_from_R(T_home_r[:3, :3])
            robot_pose = Pose2D(rx, ry, yaw, True)
        else:
            robot_pose = Pose2D(np.nan, np.nan, np.nan, False)

        # Target
        if target_id in poses_cam:
            rvec_t, tvec_t = poses_cam[target_id]
            T_cam_t = _T_from_rvec_tvec(rvec_t, tvec_t)
            T_home_t = T_home_cam @ T_cam_t
            tx, ty, _tz = T_home_t[:3, 3].tolist()
            target_xy = XY(tx, ty, True)
        else:
            target_xy = XY(np.nan, np.nan, False)

        return robot_pose, target_xy
