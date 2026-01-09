"""Graphical mission control panel for the modular IDESA system."""

from __future__ import annotations

from typing import Callable, Dict

from IDESA_Comms import UDPSender
from IDESA_State import IDESAState
from IDESA_Vision import VisionSystem

try:
    from PySide6 import QtCore, QtGui, QtWidgets  # type: ignore

    PYSIDE_AVAILABLE = True
except Exception:  # pragma: no cover - import guarded for environments without PySide6
    PYSIDE_AVAILABLE = False

if not PYSIDE_AVAILABLE:
    import tkinter as tk
    from tkinter import ttk


class _ControllerMixin:
    def __init__(
        self,
        vision: VisionSystem,
        sender: UDPSender,
        state_supplier: Callable[[], IDESAState],
        send_hz: float,
    ) -> None:
        self._vision = vision
        self._sender = sender
        self._state_supplier = state_supplier
        self._send_hz = send_hz

    def _start_all(self) -> None:
        try:
            self._vision.start_tracking()
            self._sender.start(self._state_supplier, self._send_hz)
        except Exception as exc:
            self._handle_error(str(exc))

    def _stop_all(self) -> None:
        self._sender.stop()
        self._vision.stop_tracking()
        self._vision.stop_camera()

    def _handle_error(self, message: str) -> None:
        # Subclasses override to surface errors.
        pass

    def _set_robot_id(self, value: int) -> None:
        self._vision.set_robot_id(int(value))

    def _set_target_id(self, value: int) -> None:
        self._vision.set_target_id(int(value))

    def _get_marker_ids(self) -> tuple[int, int]:
        return self._vision.get_marker_ids()


if PYSIDE_AVAILABLE:

    class MissionControlWindow(QtWidgets.QMainWindow, _ControllerMixin):
        def __init__(
            self,
            vision: VisionSystem,
            sender: UDPSender,
            state_supplier: Callable[[], IDESAState],
            send_hz: float,
        ) -> None:
            QtWidgets.QMainWindow.__init__(self)
            _ControllerMixin.__init__(self, vision, sender, state_supplier, send_hz)

            self.setWindowTitle("GUI_mission_control_1")
            self.resize(520, 360)

            central = QtWidgets.QWidget()
            self.setCentralWidget(central)
            layout = QtWidgets.QVBoxLayout(central)
            layout.setSpacing(16)

            button_font = QtGui.QFont("Space Mono", 11)
            info_font = QtGui.QFont("Space Grotesk", 10)

            button_row = QtWidgets.QHBoxLayout()
            layout.addLayout(button_row)

            self._btn_start = QtWidgets.QPushButton("Start")
            self._btn_start.setFont(button_font)
            self._btn_start.clicked.connect(self._start_all)

            self._btn_stop = QtWidgets.QPushButton("Stop")
            self._btn_stop.setFont(button_font)
            self._btn_stop.clicked.connect(self._stop_all)

            for btn in (self._btn_start, self._btn_stop):
                btn.setCursor(QtCore.Qt.PointingHandCursor)
                button_row.addWidget(btn, stretch=1)

            ids_group = QtWidgets.QGroupBox("Marker IDs")
            ids_layout = QtWidgets.QFormLayout(ids_group)
            ids_layout.setLabelAlignment(QtCore.Qt.AlignLeft)
            ids_layout.setFormAlignment(QtCore.Qt.AlignTop)

            robot_id, target_id = self._get_marker_ids()

            self._robot_spin = QtWidgets.QSpinBox()
            self._robot_spin.setRange(0, 4096)
            self._robot_spin.setValue(robot_id)
            self._robot_spin.valueChanged.connect(self._set_robot_id)

            self._target_spin = QtWidgets.QSpinBox()
            self._target_spin.setRange(0, 4096)
            self._target_spin.setValue(target_id)
            self._target_spin.valueChanged.connect(self._set_target_id)

            ids_layout.addRow("Robot ID:", self._robot_spin)
            ids_layout.addRow("Target ID:", self._target_spin)

            layout.addWidget(ids_group)

            stats_group = QtWidgets.QGroupBox("Live Telemetry")
            stats_layout = QtWidgets.QFormLayout(stats_group)
            stats_layout.setLabelAlignment(QtCore.Qt.AlignLeft)
            stats_layout.setFormAlignment(QtCore.Qt.AlignTop)

            self._value_labels: Dict[str, QtWidgets.QLabel] = {}
            fields = [
                ("Robot X [mm]", "robot_x_mm"),
                ("Robot Y [mm]", "robot_y_mm"),
                ("Robot Yaw [deg]", "robot_yaw_deg"),
                ("Target X [mm]", "target_x_mm"),
                ("Target Y [mm]", "target_y_mm"),
                ("Robot Detected", "robot_detected"),
                ("Target Detected", "target_detected"),
                ("FPS", "fps"),
            ]

            for label_text, key in fields:
                lbl = QtWidgets.QLabel("-")
                lbl.setFont(info_font)
                self._value_labels[key] = lbl
                stats_layout.addRow(label_text + ":", lbl)

            layout.addWidget(stats_group)

            self._status_bar = QtWidgets.QStatusBar()
            self.setStatusBar(self._status_bar)

            self._timer = QtCore.QTimer(self)
            self._timer.timeout.connect(self._refresh_values)
            self._timer.start(100)

        def _refresh_values(self) -> None:
            state = self._state_supplier()
            self._value_labels["robot_x_mm"].setText(f"{state.robot_x_mm:8.2f}")
            self._value_labels["robot_y_mm"].setText(f"{state.robot_y_mm:8.2f}")
            self._value_labels["robot_yaw_deg"].setText(f"{state.robot_yaw_deg:7.2f}")
            self._value_labels["target_x_mm"].setText(f"{state.target_x_mm:8.2f}")
            self._value_labels["target_y_mm"].setText(f"{state.target_y_mm:8.2f}")
            self._value_labels["robot_detected"].setText("YES" if state.robot_detected else "no")
            self._value_labels["target_detected"].setText("YES" if state.target_detected else "no")
            self._value_labels["fps"].setText(f"{state.fps:5.1f}")

        def _handle_error(self, message: str) -> None:  # type: ignore[override]
            self._status_bar.showMessage(message, 4000)

        def closeEvent(self, event):  # type: ignore[override]
            self._stop_all()
            self._vision.stop_camera()
            super().closeEvent(event)


    class PySideAppWrapper:
        def __init__(
            self,
            vision: VisionSystem,
            sender: UDPSender,
            state_supplier: Callable[[], IDESAState],
            send_hz: float,
        ) -> None:
            self._vision = vision
            self._sender = sender
            self._state_supplier = state_supplier
            self._send_hz = send_hz

        def run(self) -> None:
            import sys

            app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
            window = MissionControlWindow(self._vision, self._sender, self._state_supplier, self._send_hz)
            window.show()
            app.exec()

        def stop(self) -> None:
            self._sender.stop()
            self._vision.stop_tracking()
            self._vision.stop_camera()


else:

    class TkMissionControl(tk.Tk, _ControllerMixin):
        def __init__(
            self,
            vision: VisionSystem,
            sender: UDPSender,
            state_supplier: Callable[[], IDESAState],
            send_hz: float,
        ) -> None:
            tk.Tk.__init__(self)
            _ControllerMixin.__init__(self, vision, sender, state_supplier, send_hz)

            self.title("GUI_mission_control_1")
            self.geometry("540x360")

            self.configure(background="#101820")

            style = ttk.Style(self)
            style.theme_use("clam")
            style.configure("TButton", font=("Futura", 11), padding=6)
            style.configure("TLabel", background="#101820", foreground="#E1E1E1", font=("Futura", 11))

            button_frame = ttk.Frame(self)
            button_frame.pack(fill="x", pady=12, padx=12)

            self._value_labels: Dict[str, ttk.Label] = {}

            ttk.Button(button_frame, text="Start", command=self._start_all).grid(row=0, column=0, sticky="nsew", padx=4)
            ttk.Button(button_frame, text="Stop", command=self._stop_all).grid(row=0, column=1, sticky="nsew", padx=4)
            for i in range(2):
                button_frame.columnconfigure(i, weight=1)

            ids_frame = ttk.LabelFrame(self, text="Marker IDs")
            ids_frame.pack(fill="x", padx=12, pady=(0, 12))

            robot_id, target_id = self._get_marker_ids()
            self._robot_id_var = tk.IntVar(value=robot_id)
            self._target_id_var = tk.IntVar(value=target_id)

            robot_spin = tk.Spinbox(
                ids_frame,
                from_=0,
                to=4096,
                textvariable=self._robot_id_var,
                width=6,
                command=self._on_robot_spin,
            )
            target_spin = tk.Spinbox(
                ids_frame,
                from_=0,
                to=4096,
                textvariable=self._target_id_var,
                width=6,
                command=self._on_target_spin,
            )

            ttk.Label(ids_frame, text="Robot ID:").grid(row=0, column=0, sticky="w", padx=8, pady=4)
            robot_spin.grid(row=0, column=1, sticky="w", padx=8, pady=4)
            ttk.Label(ids_frame, text="Target ID:").grid(row=1, column=0, sticky="w", padx=8, pady=4)
            target_spin.grid(row=1, column=1, sticky="w", padx=8, pady=4)

            stats_frame = ttk.LabelFrame(self, text="Live Telemetry")
            stats_frame.pack(fill="both", expand=True, padx=12, pady=12)

            fields = [
                ("Robot X [mm]", "robot_x_mm"),
                ("Robot Y [mm]", "robot_y_mm"),
                ("Robot Yaw [deg]", "robot_yaw_deg"),
                ("Target X [mm]", "target_x_mm"),
                ("Target Y [mm]", "target_y_mm"),
                ("Robot Detected", "robot_detected"),
                ("Target Detected", "target_detected"),
                ("FPS", "fps"),
            ]

            for idx, (label_text, key) in enumerate(fields):
                ttk.Label(stats_frame, text=label_text).grid(row=idx, column=0, sticky="w", padx=8, pady=4)
                value_lbl = ttk.Label(stats_frame, text="-")
                value_lbl.grid(row=idx, column=1, sticky="e", padx=8, pady=4)
                self._value_labels[key] = value_lbl

            self._status_var = tk.StringVar(value="Idle")
            status_bar = ttk.Label(self, textvariable=self._status_var, anchor="w")
            status_bar.pack(fill="x", padx=12, pady=(0, 8))

            self.protocol("WM_DELETE_WINDOW", self._on_close)
            self._schedule_refresh()

        def _handle_error(self, message: str) -> None:  # type: ignore[override]
            self._status_var.set(message)

        def _schedule_refresh(self) -> None:
            self._refresh_values()
            self.after(100, self._schedule_refresh)

        def _refresh_values(self) -> None:
            state = self._state_supplier()
            self._value_labels["robot_x_mm"].config(text=f"{state.robot_x_mm:8.2f}")
            self._value_labels["robot_y_mm"].config(text=f"{state.robot_y_mm:8.2f}")
            self._value_labels["robot_yaw_deg"].config(text=f"{state.robot_yaw_deg:7.2f}")
            self._value_labels["target_x_mm"].config(text=f"{state.target_x_mm:8.2f}")
            self._value_labels["target_y_mm"].config(text=f"{state.target_y_mm:8.2f}")
            self._value_labels["robot_detected"].config(text="YES" if state.robot_detected else "no")
            self._value_labels["target_detected"].config(text="YES" if state.target_detected else "no")
            self._value_labels["fps"].config(text=f"{state.fps:5.1f}")

        def _on_robot_spin(self) -> None:
            self._set_robot_id(self._robot_id_var.get())

        def _on_target_spin(self) -> None:
            self._set_target_id(self._target_id_var.get())

        def _on_close(self) -> None:
            self._stop_all()
            self._vision.stop_camera()
            self.destroy()

        def run(self) -> None:
            self.mainloop()

        def stop(self) -> None:
            self._stop_all()
            self._vision.stop_camera()


def create_gui(
    vision: VisionSystem,
    sender: UDPSender,
    state_supplier: Callable[[], IDESAState],
    send_hz: float,
):
    """Factory that hides whether PySide6 or Tkinter is backing the GUI."""
    if PYSIDE_AVAILABLE:
        return PySideAppWrapper(vision, sender, state_supplier, send_hz)
    return TkMissionControl(vision, sender, state_supplier, send_hz)
