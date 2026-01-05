# vision_aruco.py
"""
Aruco vision module designed to be *called by a GUI* (no cv2.imshow / no infinite loop).

Features:
- Open camera
- Detect ArUco markers
- Optionally estimate pose if camera intrinsics are provided
- Compute marker pose relative to a "home" marker (best for real-world coords)
- Return an annotated frame + structured detection data

Requires:
- opencv-contrib-python (for cv2.aruco)
- numpy
"""

from __future__ import annotations
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple, Any

import cv2
import numpy as np


@dataclass
class MarkerDetection:
    marker_id: int
    corners_px: np.ndarray  # shape (4,2)
    center_px: Tuple[float, float]
    rvec: Optional[np.ndarray] = None  # shape (3,1)
    tvec: Optional[np.ndarray] = None  # shape (3,1)
    # Pose relative to home marker (same units as tvec, usually meters or mm depending on marker_length)
    rel_tvec: Optional[np.ndarray] = None  # shape (3,1)
    rel_yaw_deg: Optional[float] = None


def _rvec_tvec_to_T(rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    """Convert Rodrigues rvec + tvec to 4x4 transform matrix."""
    R, _ = cv2.Rodrigues(rvec.reshape(3, 1))
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = tvec.reshape(3)
    return T


def _yaw_from_R(R: np.ndarray) -> float:
    """
    Extract yaw from rotation matrix.
    Convention depends on your coordinate system; this gives a reasonable 'heading-like' yaw.
    """
    # yaw around Z if Z is up; many camera frames differ, so treat as indicative.
    yaw = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
    return float(yaw)


class ArucoTracker:
    def __init__(
        self,
        camera_index: int = 0,
        width: int = 1280,
        height: int = 720,
        aruco_dict_name: str = "DICT_4X4_50",
        home_id: int = 0,
        marker_length: float = 0.05,  # meters (e.g., 0.05 = 5cm). Must match your printed marker size.
        camera_matrix: Optional[np.ndarray] = None,
        dist_coeffs: Optional[np.ndarray] = None,
    ):
        self.camera_index = camera_index
        self.width = width
        self.height = height

        self.home_id = home_id
        self.marker_length = marker_length

        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs

        self.cap: Optional[cv2.VideoCapture] = None

        self.aruco_dict = self._get_aruco_dict(aruco_dict_name)
        self.detector_params = cv2.aruco.DetectorParameters()

        # Newer OpenCV has ArucoDetector
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.detector_params)

    def _get_aruco_dict(self, name: str):
        if not hasattr(cv2.aruco, name):
            raise ValueError(f"Unknown ArUco dict name: {name}")
        return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))

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

    def read(self) -> Tuple[np.ndarray, List[MarkerDetection]]:
        """
        Returns:
          frame_bgr: annotated frame
          detections: list of MarkerDetection objects
        """
        if self.cap is None:
            raise RuntimeError("Camera not started. Call start() first.")

        ok, frame = self.cap.read()
        if not ok or frame is None:
            raise RuntimeError("Failed to read frame from camera.")

        detections = self._detect_and_annotate(frame)
        return frame, detections

    def _detect_and_annotate(self, frame_bgr: np.ndarray) -> List[MarkerDetection]:
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)

        corners, ids, _rejected = self.detector.detectMarkers(gray)

        detections: List[MarkerDetection] = []
        if ids is None or len(ids) == 0:
            return detections

        ids_flat = ids.flatten().astype(int)

        # Draw detected marker borders
        cv2.aruco.drawDetectedMarkers(frame_bgr, corners, ids)

        # Prepare pose estimation if intrinsics exist
        have_pose = self.camera_matrix is not None and self.dist_coeffs is not None

        rvecs = tvecs = None
        if have_pose:
            rvecs, tvecs, _obj = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs
            )

        # Build detections
        for i, marker_id in enumerate(ids_flat):
            c = corners[i].reshape(4, 2)
            center = (float(np.mean(c[:, 0])), float(np.mean(c[:, 1])))

            det = MarkerDetection(
                marker_id=marker_id,
                corners_px=c,
                center_px=center,
            )

            if have_pose and rvecs is not None and tvecs is not None:
                det.rvec = rvecs[i].reshape(3, 1)
                det.tvec = tvecs[i].reshape(3, 1)
                # draw axis
                cv2.drawFrameAxes(
                    frame_bgr,
                    self.camera_matrix,
                    self.dist_coeffs,
                    det.rvec,
                    det.tvec,
                    self.marker_length * 0.5,
                )

            detections.append(det)

        # Compute relative poses wrt home marker if possible
        if have_pose:
            home = next((d for d in detections if d.marker_id == self.home_id and d.rvec is not None), None)
            if home is not None:
                T_cam_home = _rvec_tvec_to_T(home.rvec, home.tvec)
                T_home_cam = np.linalg.inv(T_cam_home)

                for d in detections:
                    if d.rvec is None or d.tvec is None:
                        continue
                    T_cam_d = _rvec_tvec_to_T(d.rvec, d.tvec)
                    T_home_d = T_home_cam @ T_cam_d
                    d.rel_tvec = T_home_d[:3, 3].reshape(3, 1)
                    d.rel_yaw_deg = _yaw_from_R(T_home_d[:3, :3])

                    # annotate relative position on image (for quick sanity)
                    txt = f"ID {d.marker_id} rel x={d.rel_tvec[0,0]:.3f} y={d.rel_tvec[1,0]:.3f} yaw={d.rel_yaw_deg:.1f}"
                    cv2.putText(
                        frame_bgr,
                        txt,
                        (int(d.center_px[0]) + 10, int(d.center_px[1]) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        1,
                        cv2.LINE_AA,
                    )

        return detections

    @staticmethod
    def load_calibration_npz(npz_path: str) -> Tuple[np.ndarray, np.ndarray]:
        """
        Load camera intrinsics from a .npz containing:
          - camera_matrix
          - dist_coeffs
        """
        data = np.load(npz_path)
        return data["camera_matrix"], data["dist_coeffs"]
