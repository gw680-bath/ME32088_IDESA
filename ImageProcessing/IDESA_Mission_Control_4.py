"""IDESA_Mission_Control_4.py

Mission Control now serves a Mars-themed web dashboard on localhost instead of
an on-device Tk window. The class keeps the same public surface expected by the
rest of the stack (`after()` + `mainloop()` methods) so no other module needs
changes.

Key points:
- Buttons and toggles mirror the legacy logic (encoder reset rules, target
  validation, manual pulses, and state/status updates).
- A lightweight HTTP server (standard library) hosts the UI and JSON API.
- The manual controller still binds its callbacks; Mission Control calls them
  when web actions request pulses.
"""

from __future__ import annotations

import json
import threading
import time
import webbrowser
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from threading import Lock
from typing import Any, Callable, Dict, Tuple


_HTML_TEMPLATE_CACHE: str | None = None


def _load_html_template() -> str:
    global _HTML_TEMPLATE_CACHE
    if _HTML_TEMPLATE_CACHE is None:
        template_path = Path(__file__).with_name("Mission_control_ui_4.html")
        try:
            _HTML_TEMPLATE_CACHE = template_path.read_text(encoding="utf-8")
        except OSError as exc:
            raise RuntimeError(
                f"Mission Control template missing or unreadable: {template_path}"  # pragma: no cover
            ) from exc
    return _HTML_TEMPLATE_CACHE


class IDESAMissionControl4:
    TARGET_IDS = tuple(range(2, 8))

    class _KeyBindingProxy:
        """Collects manual key callbacks without requiring a Tk root."""

        def __init__(self) -> None:
            self.handlers: dict[str, Callable] = {}

        def bind(self, sequence: str, callback: Callable) -> None:
            self.handlers[sequence] = callback

    def __init__(
        self,
        state,
        state_lock: Lock,
        on_camera_on: Callable[[], None],
        on_camera_off: Callable[[], None],
        on_start: Callable[[], None],
        on_stop: Callable[[], None],
        on_bind_keys: Callable[[Any], None],
        poll_ms: int = 200,
        host: str = "127.0.0.1",
        port: int = 8050,
        auto_open_browser: bool = True,
    ) -> None:
        self.state = state
        self.lock = state_lock
        self._on_camera_on = on_camera_on
        self._on_camera_off = on_camera_off
        self._on_start = on_start
        self._on_stop = on_stop
        self._poll_ms = max(int(poll_ms), 50)
        self._host = host
        self._port = port
        self._base_url = f"http://{host}:{port}"
        self._auto_open_browser = auto_open_browser
        self._status_message = "Idle"
        self._status_timestamp = time.time()

        self._key_proxy = self._KeyBindingProxy()
        on_bind_keys(self._key_proxy)

        self._server: ThreadingHTTPServer | None = None
        self._server_thread: threading.Thread | None = None
        self._running = False
        self._timers: set[threading.Timer] = set()

        self._handler_cls = self._build_handler_class()

    # ------------------------------------------------------------------
    # Public Tk-like hooks consumed by IDESA_Main_4
    # ------------------------------------------------------------------
    def after(self, delay_ms: int, callback: Callable[[], None]) -> threading.Timer:
        delay_s = max(int(delay_ms), 0) / 1000.0

        def runner() -> None:
            try:
                callback()
            except Exception as exc:  # pragma: no cover - telemetry only
                print(f"[Mission Control] after() callback failed: {exc}")
            finally:
                self._timers.discard(timer)

        timer = threading.Timer(delay_s, runner)
        timer.daemon = True
        self._timers.add(timer)
        timer.start()
        return timer

    def mainloop(self) -> None:
        if self._running:
            return
        self._running = True
        self._launch_server()
        try:
            while self._running:
                time.sleep(0.1)
        except KeyboardInterrupt:
            self._set_status("Mission Control interrupted")
        finally:
            self.destroy()

    def destroy(self) -> None:
        self._running = False
        try:
            self._on_camera_off()
        except Exception:
            pass
        self._shutdown_server()
        for timer in list(self._timers):
            timer.cancel()
        self._timers.clear()

    # ------------------------------------------------------------------
    # HTTP server plumbing
    # ------------------------------------------------------------------
    def _build_handler_class(self):
        outer = self

        class MissionControlHandler(BaseHTTPRequestHandler):
            def do_GET(self) -> None:  # noqa: N802 (BaseHTTPRequestHandler API)
                if self.path in {"/", "/index.html"}:
                    outer._serve_index(self)
                elif self.path.startswith("/api/state"):
                    outer._serve_state(self)
                else:
                    self.send_error(HTTPStatus.NOT_FOUND, "Not Found")

            def do_POST(self) -> None:  # noqa: N802
                if self.path == "/api/action":
                    outer._handle_action_request(self)
                else:
                    self.send_error(HTTPStatus.NOT_FOUND, "Not Found")

            def log_message(self, _format: str, *_args: Any) -> None:  # quiet server logs
                return

        return MissionControlHandler

    def _launch_server(self) -> None:
        if self._server is not None:
            return
        try:
            self._server = ThreadingHTTPServer((self._host, self._port), self._handler_cls)
        except OSError as exc:
            raise RuntimeError(f"Mission Control web server failed to bind: {exc}") from exc
        self._server.daemon_threads = True
        self._server_thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._server_thread.start()
        print(f"[Mission Control] Web UI → {self._base_url}")
        if self._auto_open_browser:
            webbrowser.open(self._base_url)

    def _shutdown_server(self) -> None:
        if self._server is None:
            return
        try:
            self._server.shutdown()
            self._server.server_close()
        finally:
            self._server = None
            self._server_thread = None

    def _serve_index(self, handler: BaseHTTPRequestHandler) -> None:
        html = self._build_html_document()
        data = html.encode("utf-8")
        handler.send_response(HTTPStatus.OK)
        handler.send_header("Content-Type", "text/html; charset=utf-8")
        handler.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        handler.send_header("Content-Length", str(len(data)))
        handler.end_headers()
        handler.wfile.write(data)

    def _serve_state(self, handler: BaseHTTPRequestHandler) -> None:
        payload = self._collect_state_snapshot()
        self._send_json(handler, HTTPStatus.OK, payload)

    def _handle_action_request(self, handler: BaseHTTPRequestHandler) -> None:
        length = int(handler.headers.get("Content-Length", "0"))
        raw = handler.rfile.read(length) if length else b""
        try:
            body = json.loads(raw.decode("utf-8") or "{}")
        except json.JSONDecodeError:
            self._send_json(handler, HTTPStatus.BAD_REQUEST, {"ok": False, "status": "Invalid JSON"})
            return

        action = str(body.get("action") or "").strip()
        payload = body.get("payload") or {}
        ok, message = self._dispatch_action(action, payload)
        status_code = HTTPStatus.OK if ok else HTTPStatus.BAD_REQUEST
        response = {"ok": ok, "status": message, "state": self._collect_state_snapshot()}
        self._send_json(handler, status_code, response)

    def _send_json(self, handler: BaseHTTPRequestHandler, status: HTTPStatus, payload: Dict[str, Any]) -> None:
        data = json.dumps(payload).encode("utf-8")
        handler.send_response(status)
        handler.send_header("Content-Type", "application/json")
        handler.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        handler.send_header("Content-Length", str(len(data)))
        handler.end_headers()
        handler.wfile.write(data)

    # ------------------------------------------------------------------
    # Action dispatch
    # ------------------------------------------------------------------
    def _dispatch_action(self, action: str, payload: Dict[str, Any]) -> Tuple[bool, str]:
        action_key = action.lower()
        try:
            if action_key == "camera_on":
                return self._camera_on()
            if action_key == "camera_off":
                return self._camera_off()
            if action_key == "start":
                return self._start()
            if action_key == "stop":
                return self._stop()
            if action_key == "estop":
                return self._estop()
            if action_key == "reset_estop":
                return self._reset_estop()
            if action_key == "mode":
                return self._change_mode(str(payload.get("value", "")))
            if action_key == "auto_sub":
                return self._change_auto_sub(str(payload.get("value", "")))
            if action_key == "targets":
                return self._change_targets(payload.get("selected"))
            if action_key == "radius":
                return self._change_radius(payload.get("value"))
            if action_key == "manual_pulse":
                return self._manual_pulse(str(payload.get("direction", "")))
            if action_key == "shutdown":
                self._running = False
                self._set_status("Mission Control shutting down")
                return True, self._status_message
        except Exception as exc:
            message = f"Action '{action}' failed: {exc}"
            self._set_status(message)
            return False, message
        message = f"Unknown action '{action}'"
        self._set_status(message)
        return False, message

    # ------------------------------------------------------------------
    # Button + control operations (mirrors legacy Tk logic)
    # ------------------------------------------------------------------
    def _camera_on(self) -> Tuple[bool, str]:
        try:
            self._on_camera_on()
            with self.lock:
                self.state.camera_on = True
            message = "Camera ON"
        except Exception as exc:
            message = f"Camera ON failed: {exc}"
            self._set_status(message)
            return False, message
        self._set_status(message)
        return True, message

    def _camera_off(self) -> Tuple[bool, str]:
        try:
            self._on_camera_off()
            message = "Camera OFF (sending disabled)"
        except Exception as exc:
            message = f"Camera OFF failed: {exc}"
            self._set_status(message)
            return False, message
        self._set_status(message)
        return True, message

    def _start(self) -> Tuple[bool, str]:
        try:
            self._on_start()
            self._arm_encoder_reset_pulse()
            message = "Start: sending enabled"
        except Exception as exc:
            message = f"Start failed: {exc}"
            self._set_status(message)
            return False, message
        self._set_status(message)
        return True, message

    def _stop(self) -> Tuple[bool, str]:
        try:
            self._on_stop()
            self._arm_encoder_reset_pulse()
            message = "Stop: sending disabled"
        except Exception as exc:
            message = f"Stop failed: {exc}"
            self._set_status(message)
            return False, message
        self._set_status(message)
        return True, message

    def _estop(self) -> Tuple[bool, str]:
        with self.lock:
            self.state.estop_pressed = True
            self.state.sending_enabled = False
        self._arm_encoder_reset_pulse()
        message = "E-Stop PRESSED"
        self._set_status(message)
        return True, message

    def _reset_estop(self) -> Tuple[bool, str]:
        with self.lock:
            self.state.estop_pressed = False
        self._arm_encoder_reset_pulse()
        message = "E-Stop RESET"
        self._set_status(message)
        return True, message

    def _change_mode(self, value: str) -> Tuple[bool, str]:
        new_mode = value.upper()
        new_mode = "AUTOMATIC" if new_mode in {"AUTO", "AUTOMATIC"} else "MANUAL"
        with self.lock:
            self.state.control_mode = new_mode
        self._arm_encoder_reset_pulse()
        message = f"Mode: {new_mode.title()}"
        self._set_status(message)
        return True, message

    def _change_auto_sub(self, value: str) -> Tuple[bool, str]:
        new_sub = value.upper()
        new_sub = "RECOVERY" if new_sub == "RECOVERY" else "MISSION"
        with self.lock:
            self.state.auto_sub_mode = new_sub
        self._arm_encoder_reset_pulse()
        message = f"Automatic sub-mode: {new_sub.title()}"
        self._set_status(message)
        return True, message

    def _change_targets(self, selected: Any) -> Tuple[bool, str]:
        if not isinstance(selected, list):
            message = "Targets payload must be a list"
            self._set_status(message)
            return False, message
        cleaned: list[int] = []
        for tid in selected:
            try:
                tid_int = int(tid)
            except Exception:
                continue
            if tid_int in self.TARGET_IDS and tid_int not in cleaned:
                cleaned.append(tid_int)
        if not (1 <= len(cleaned) <= 6):
            message = "Select 1–6 valid targets"
            self._set_status(message)
            return False, message
        with self.lock:
            self.state.selected_targets = cleaned
        message = f"Targets set: {cleaned}"
        self._set_status(message)
        return True, message

    def _change_radius(self, value: Any) -> Tuple[bool, str]:
        try:
            radius = float(value)
        except Exception:
            message = "Radius must be numeric"
            self._set_status(message)
            return False, message
        if radius <= 0:
            message = "Radius must be > 0"
            self._set_status(message)
            return False, message
        with self.lock:
            self.state.switch_radius_mm = radius
        message = f"Switch radius: {radius:.1f} mm"
        self._set_status(message)
        return True, message

    def _manual_pulse(self, direction: str) -> Tuple[bool, str]:
        direction_key = direction.upper()
        event_lookup = {"UP": "<Up>", "LEFT": "<Left>", "RIGHT": "<Right>"}
        if direction_key not in event_lookup:
            message = "Manual direction must be Up/Left/Right"
            self._set_status(message)
            return False, message

        with self.lock:
            mode_ok = str(getattr(self.state, "control_mode", "AUTOMATIC")).upper() == "MANUAL"
            sending_ok = bool(getattr(self.state, "sending_enabled", False))
            estop_ok = not bool(getattr(self.state, "estop_pressed", False))

        if not mode_ok:
            message = "Manual pulses require MANUAL mode"
            self._set_status(message)
            return False, message
        if not sending_ok:
            message = "Start sending before manual pulses"
            self._set_status(message)
            return False, message
        if not estop_ok:
            message = "Release E-Stop to move"
            self._set_status(message)
            return False, message

        handler = self._key_proxy.handlers.get(event_lookup[direction_key])
        if handler is None:
            message = "Manual bindings unavailable"
            self._set_status(message)
            return False, message
        handler(None)
        message = f"Manual pulse: {direction_key.title()}"
        self._set_status(message)
        return True, message

    # ------------------------------------------------------------------
    # State helpers
    # ------------------------------------------------------------------
    def _arm_encoder_reset_pulse(self) -> None:
        with self.lock:
            self.state.reset_pulse_cycles = 1

    def _set_status(self, text: str) -> None:
        self._status_message = text
        self._status_timestamp = time.time()

    def _collect_state_snapshot(self) -> Dict[str, Any]:
        now = time.time()
        with self.lock:
            robot_visible = bool(getattr(self.state, "robot_visible", False))
            active_id = getattr(self.state, "active_target_id", None)
            last_seen_map = getattr(self.state, "targets_last_seen", {})
            last_seen = float(last_seen_map.get(active_id, 0.0)) if active_id is not None else 0.0
            active_visible = active_id is not None and (now - last_seen) <= 2.0

            cmd_dist = float(getattr(self.state, "cmd_distance_mm", 0.0))
            cmd_ang = float(getattr(self.state, "cmd_angle_deg", 0.0))
            mode = str(getattr(self.state, "control_mode", "AUTOMATIC")).upper()
            auto_sub = str(getattr(self.state, "auto_sub_mode", "MISSION")).upper()
            sending = bool(getattr(self.state, "sending_enabled", False))
            estop = bool(getattr(self.state, "estop_pressed", False))
            selected_targets = list(getattr(self.state, "selected_targets", []))
            radius = float(getattr(self.state, "switch_radius_mm", 150.0))
            camera_on = bool(getattr(self.state, "camera_on", False))

            # Manual pulse visibility mirrors historic Tk behaviour
            last_manual_dist = float(getattr(self.state, "last_manual_distance_mm", 0.0))
            last_manual_ang = float(getattr(self.state, "last_manual_angle_deg", 0.0))
            last_manual_time = float(getattr(self.state, "last_manual_timestamp", 0.0))

        display_dist = cmd_dist
        display_ang = cmd_ang
        if mode == "MANUAL" and (now - last_manual_time) <= 1.0 and abs(cmd_dist) < 1e-6 and abs(cmd_ang) < 1e-6:
            display_dist = last_manual_dist
            display_ang = last_manual_ang

        auto_active = mode in {"AUTO", "AUTOMATIC"}
        telemetry = {
            "robot_visible": robot_visible,
            "active_target_visible": active_visible,
            "active_target_id": active_id,
            "cmd_distance_mm": round(display_dist, 2),
            "cmd_angle_deg": round(display_ang, 2),
            "mode": "Automatic" if auto_active else "Manual",
            "auto_sub_mode": "Mission" if auto_sub == "MISSION" else "Recovery Rover",
            "estop": "PRESSED" if estop else "OK",
            "sending": sending,
            "camera_on": camera_on,
        }

        return {
            "timestamp": now,
            "poll_interval_ms": self._poll_ms,
            "telemetry": telemetry,
            "control_mode": mode,
            "auto_sub_mode": auto_sub,
            "auto_controls_enabled": auto_active,
            "selected_targets": selected_targets,
            "switch_radius_mm": radius,
            "estop_pressed": estop,
            "camera_on": camera_on,
            "sending_enabled": sending,
            "manual_ready": bool(self._key_proxy.handlers),
            "status": {
                "message": self._status_message,
                "timestamp": self._status_timestamp,
            },
            "targets_supported": list(self.TARGET_IDS),
        }

    # ------------------------------------------------------------------
    # HTML (Mars theme)
    # ------------------------------------------------------------------
    def _build_html_document(self) -> str:
        targets_json = json.dumps(list(self.TARGET_IDS))
        template = _load_html_template()
        return template.replace("__TARGETS__", targets_json).replace("__POLL_MS__", str(self._poll_ms))

