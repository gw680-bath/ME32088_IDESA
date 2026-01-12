"""Vision bridge that mirrors the legacy detector into the parallel stack."""

from __future__ import annotations

import time
from threading import Event, Thread
from typing import Optional, Sequence

from IDESA_State import IDESAStateStore as LegacyStateStore
from IDESA_Vision import VisionSystem as LegacyVision
from IDESA_State_2 import IDESAStateStore2
from IDESA_Types_2 import TargetTrack2


class VisionSystem2:
    """Wraps the original VisionSystem and feeds data into IDESAStateStore2."""

    def __init__(
        self,
        state_store: IDESAStateStore2,
        camera_index: int = 1,
        marker_size_mm: float = 100.0,
        dict_name: str = "DICT_4X4_50",
        robot_id: int = 1,
        target_ids: Optional[Sequence[int]] = None,
        display_preview: bool = True,
        preview_scale: float = 1.0,
        mirror_hz: float = 30.0,
        vision_timeout_s: float = 0.5,
    ) -> None:
        self._state_store = state_store
        self._legacy_store = LegacyStateStore()
        self._legacy_vision = LegacyVision(
            self._legacy_store,
            camera_index=camera_index,
            marker_size_mm=marker_size_mm,
            dict_name=dict_name,
            robot_id=robot_id,
            target_id=target_ids[0] if target_ids else 3,
            display_preview=display_preview,
            preview_scale=preview_scale,
        )
        if target_ids:
            self._legacy_vision.set_tracked_target_ids(target_ids)

        self._mirror_period = 1.0 / max(float(mirror_hz), 1.0)
        self._vision_timeout_s = max(float(vision_timeout_s), 0.1)
        self._stop_event = Event()
        self._mirror_thread = Thread(target=self._mirror_loop, name="VisionMirror2", daemon=True)
        self._mirror_thread.start()

    # ------------------------------------------------------------------
    # Legacy Vision delegation
    # ------------------------------------------------------------------
    def start_camera(self) -> None:
        self._legacy_vision.start_camera()

    def stop_camera(self) -> None:
        self._legacy_vision.stop_camera()

    def start_tracking(self) -> None:
        self._legacy_vision.start_tracking()

    def stop_tracking(self) -> None:
        self._legacy_vision.stop_tracking()

    def is_tracking(self) -> bool:
        return self._legacy_vision.is_tracking()

    def set_tracked_target_ids(self, target_ids: Sequence[int]) -> Sequence[int]:
        return self._legacy_vision.set_tracked_target_ids(target_ids)

    def get_tracked_target_ids(self) -> Sequence[int]:
        return self._legacy_vision.get_tracked_target_ids()

    def get_state_snapshot(self):  # pragma: no cover - debugging helper
        return self._state_store.snapshot()

    # ------------------------------------------------------------------
    # Mirror loop
    # ------------------------------------------------------------------
    def _mirror_loop(self) -> None:
        while not self._stop_event.is_set():
            legacy = self._legacy_store.snapshot()
            now = time.time()
            vision_lost = (now - legacy.last_update_time) > self._vision_timeout_s or not legacy.robot_detected

            targets = {
                tid: TargetTrack2(
                    x_mm=track.x_mm,
                    y_mm=track.y_mm,
                    detected=track.detected,
                    last_seen_time=track.last_seen_time,
                )
                for tid, track in legacy.targets_by_id.items()
            }

            self._state_store.update(
                robot_x_mm=legacy.robot_x_mm,
                robot_y_mm=legacy.robot_y_mm,
                robot_yaw_deg=legacy.robot_yaw_deg,
                robot_detected=legacy.robot_detected,
                targets_by_id=targets,
                vision_lost=vision_lost,
            )

            active_state = self._state_store.snapshot()
            active_track = targets.get(active_state.active_target_id)
            if active_track:
                self._state_store.update(
                    target_detected=active_track.detected,
                )

            self._stop_event.wait(self._mirror_period)

    def stop(self) -> None:
        self._legacy_vision.stop_tracking()
        self._legacy_vision.stop_camera()
        self._stop_event.set()
        if self._mirror_thread.is_alive():
            self._mirror_thread.join(timeout=2.0)

    def __del__(self) -> None:  # pragma: no cover - best-effort cleanup
        self.stop()
