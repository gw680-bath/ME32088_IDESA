"""
IDESA_Navigation_3.py
AUTO navigation:
- Chooses active target from GUI-selected list
- Computes distance and angle error (robot->target)
- Only valid if BOTH robot + active target seen within last 2 seconds
- If within switch radius: outputs zeros for 2 seconds, then advances target
"""

from __future__ import annotations

import math
from threading import Lock


class NavigationSystem3:
    def __init__(self, state, state_lock: Lock, vision_timeout_s: float = 10, hold_zero_s: float = 10) -> None:
        self.state = state
        self.lock = state_lock
        self.vision_timeout_s = float(vision_timeout_s)
        self.hold_zero_s = float(hold_zero_s)

        self._queue_index = 0
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

            # Ensure queue index valid
            if self._queue_index >= len(selected):
                self._queue_index = 0

            active_id = self.state.active_target_id
            if active_id not in selected:
                active_id = selected[self._queue_index]
                self.state.active_target_id = active_id

            # Hold-zero window after reaching target
            if now < float(self.state.hold_zero_until):
                self.state.nav_distance_mm = 0.0
                self.state.nav_angle_deg = 0.0
                self._filt_init = False
                return

            # Check "seen within last 2s" rule for robot + active target
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

            # Desired bearing: angle of vector robot->target
            bearing_deg_math = math.degrees(math.atan2(dy, dx))  # 0 along +X

            # Robot yaw convention requirement:
            # "robot yaw 0 on vertical +Y axis"
            # Convert robot_yaw (assumed 0 along +X) into 0 along +Y by subtracting 90 deg,
            # OR if your rvec yaw already matches +X, this conversion is correct.
            robot_yaw_deg_math = float(self.state.robot_yaw_deg)
            robot_yaw_0y = robot_yaw_deg_math - 90.0

            # Convert bearing also to 0 along +Y
            bearing_0y = bearing_deg_math - 90.0

            # Angle error = target bearing - robot yaw (both in same convention)
            ang_err = _wrap_deg(bearing_0y - robot_yaw_0y)

            # If within switch radius: hold zeros then advance
            if dist <= float(self.state.switch_radius_mm):
                self.state.nav_distance_mm = 0.0
                self.state.nav_angle_deg = 0.0
                self.state.hold_zero_until = now + self.hold_zero_s
                self._filt_init = False

                # advance target for next cycle
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
    x = (a + 180.0) % 360.0 - 180.0
    return float(x)
