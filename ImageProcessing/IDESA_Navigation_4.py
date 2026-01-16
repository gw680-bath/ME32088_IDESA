"""IDESA_Navigation_4.py

AUTO navigation:
 - Chooses active target from GUI-selected list
 - Computes distance (mm) and signed angle error (deg) robot->target
 - Outputs 0,0 when vision is stale (robot or target not seen recently)
 - When within switch radius: outputs 0,0 for hold_zero_s then advances target

Yaw convention:
 - Robot yaw is treated such that 0 deg points along +Y (up) in the arena.
"""

from __future__ import annotations

import math
from threading import Lock


class NavigationSystem4:
    def __init__(self, state, state_lock: Lock, vision_timeout_s: float = 2.0, hold_zero_s: float = 2.0) -> None:
        self.state = state
        self.lock = state_lock
        self.vision_timeout_s = float(vision_timeout_s)
        self.hold_zero_s = float(hold_zero_s)

        self._queue_index = 0
        # Light smoothing to reduce jitter without lagging too much.
        self._alpha_dist = 0.35
        self._alpha_ang = 0.45
        self._dist_filt = 0.0
        self._ang_filt = 0.0
        self._filt_init = False

    def update(self, now: float) -> None:
        with self.lock:
            selected = list(self.state.selected_targets)
            if not selected:
                self.state.active_target_id = None
                self.state.nav_distance_mm = 0.0
                self.state.nav_angle_deg = 0.0
                self._filt_init = False
                return

            if self._queue_index >= len(selected):
                self._queue_index = 0

            active_id = self.state.active_target_id
            if active_id not in selected:
                active_id = selected[self._queue_index]
                self.state.active_target_id = active_id

            if now < float(self.state.hold_zero_until):
                self.state.nav_distance_mm = 0.0
                self.state.nav_angle_deg = 0.0
                self._filt_init = False
                return

            robot_ok = (now - float(self.state.robot_last_seen)) <= self.vision_timeout_s and self.state.robot_xy_mm is not None
            target_last = float(self.state.targets_last_seen.get(active_id, 0.0))
            target_ok = (now - target_last) <= self.vision_timeout_s and (active_id in self.state.targets_xy_mm)

            if not (robot_ok and target_ok):
                self.state.nav_distance_mm = 0.0
                self.state.nav_angle_deg = 0.0
                self._filt_init = False
                return

            rx, ry = self.state.robot_xy_mm
            tx, ty = self.state.targets_xy_mm[active_id]

            dx = tx - rx
            dy = ty - ry
            dist = math.hypot(dx, dy)

            # Bearing to target in math coords (0 along +X)
            bearing_deg_math = math.degrees(math.atan2(dy, dx))

            # Convert BOTH yaw and bearing so that 0 is along +Y.
            robot_yaw_deg_math = float(self.state.robot_yaw_deg)
            robot_yaw_0y = robot_yaw_deg_math - 90.0
            bearing_0y = bearing_deg_math - 90.0

            ang_err = _wrap_deg(bearing_0y - robot_yaw_0y)

            if dist <= float(self.state.switch_radius_mm):
                self.state.nav_distance_mm = 0.0
                self.state.nav_angle_deg = 0.0
                self.state.hold_zero_until = now + self.hold_zero_s
                self._filt_init = False

                self._queue_index = (selected.index(active_id) + 1) % len(selected)
                self.state.active_target_id = selected[self._queue_index]
                return

            if not self._filt_init:
                self._dist_filt = float(dist)
                self._ang_filt = float(ang_err)
                self._filt_init = True
            else:
                self._dist_filt = self._alpha_dist * float(dist) + (1.0 - self._alpha_dist) * self._dist_filt
                delta = _wrap_deg(float(ang_err) - self._ang_filt)
                self._ang_filt = _wrap_deg(self._ang_filt + self._alpha_ang * delta)

            self.state.nav_distance_mm = float(self._dist_filt)
            self.state.nav_angle_deg = float(self._ang_filt)


def _wrap_deg(a: float) -> float:
    """Wrap angle to [-180, 180)."""
    return float((a + 180.0) % 360.0 - 180.0)
