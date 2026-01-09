"""Graphical mission control panel for the modular IDESA system."""

from __future__ import annotations

from typing import Callable, Dict, Sequence, Tuple

from IDESA_Comms import UDPSender
from IDESA_Mission import TargetQueueController
from IDESA_State import IDESAState
from IDESA_Vision import VisionSystem

try:
    from PySide6 import QtCore, QtGui, QtWidgets  # type: ignore

    PYSIDE_AVAILABLE = True
except Exception:  # pragma: no cover - PySide6 optional
    PYSIDE_AVAILABLE = False

if not PYSIDE_AVAILABLE:
    import tkinter as tk
    from tkinter import ttk


class _ControllerMixin:
    def __init__(
        self,
        vision: VisionSystem,
        mission: TargetQueueController,
        sender: UDPSender,
        state_supplier: Callable[[], IDESAState],
        send_hz: float,
        available_target_ids: Sequence[int],
        initial_target_ids: Sequence[int],
        initial_switch_radius: float,
    ) -> None:
        self._vision = vision
        self._mission = mission
        self._sender = sender
        self._state_supplier = state_supplier
        self._send_hz = send_hz

        self._available_target_ids = tuple(dict.fromkeys(int(tid) for tid in available_target_ids)) or tuple(range(2, 8))
        if initial_target_ids:
            self._current_target_ids = tuple(dict.fromkeys(int(tid) for tid in initial_target_ids))
        else:
            self._current_target_ids = self._available_target_ids[:2]
        self._current_switch_radius = max(float(initial_switch_radius), 1.0)

        # Ensure backend modules are aligned with the GUI defaults.
        self._mission.set_switch_radius(self._current_switch_radius)
        try:
            sanitized = self._mission.set_target_ids(self._current_target_ids)
        except ValueError:
            sanitized = self._mission.set_target_ids(self._available_target_ids[:2])
        self._current_target_ids = sanitized
        self._vision.set_tracked_target_ids(sanitized)

    # ------------------------------------------------------------------
    # Button callbacks
    # ------------------------------------------------------------------
    def _activate_camera(self) -> None:
        try:
            self._vision.start_camera()
        except Exception as exc:
            self._handle_error(str(exc))

    def _deactivate_camera(self) -> None:
        self._stop_all()
        self._vision.stop_camera()

    def _start_all(self) -> None:
        try:
            self._vision.start_tracking()
            self._mission.start()
            self._sender.start(self._state_supplier, self._send_hz)
        except Exception as exc:
            self._handle_error(str(exc))

    def _stop_all(self) -> None:
        self._sender.stop()
        self._mission.stop()
        self._vision.stop_tracking()

    # ------------------------------------------------------------------
    # Shared helpers for target/switch configuration
    # ------------------------------------------------------------------
    def _try_update_targets(self, ids: Sequence[int]) -> bool:
        ordered = tuple(dict.fromkeys(int(tid) for tid in ids if tid is not None))
        if not (2 <= len(ordered) <= 6):
            self._handle_error("Select 2-6 target IDs.")
            return False
        try:
            sanitized = self._mission.set_target_ids(ordered)
        except ValueError as exc:
            self._handle_error(str(exc))
            return False
        self._vision.set_tracked_target_ids(sanitized)
        self._current_target_ids = sanitized
        return True

    def _update_switch_radius(self, value: float) -> None:
        radius = self._mission.set_switch_radius(value)
        self._current_switch_radius = radius

    # These UI hooks are implemented per-framework to reset selections when needed.
    def _apply_target_selection_to_ui(self, selection: Tuple[int, ...]) -> None:
        raise NotImplementedError

    def _handle_error(self, message: str) -> None:
        # Subclasses override to surface errors.
        pass


if PYSIDE_AVAILABLE:

    class MissionControlWindow(QtWidgets.QMainWindow, _ControllerMixin):
        def __init__(
            self,
            vision: VisionSystem,
            mission: TargetQueueController,
            sender: UDPSender,
            state_supplier: Callable[[], IDESAState],
            send_hz: float,
            available_target_ids: Sequence[int],
            initial_target_ids: Sequence[int],
            initial_switch_radius: float,
        ) -> None:
            QtWidgets.QMainWindow.__init__(self)
            _ControllerMixin.__init__(
                self,
                vision,
                mission,
                sender,
                state_supplier,
                send_hz,
                available_target_ids,
                initial_target_ids,
                initial_switch_radius,
            )

            self.setWindowTitle("GUI_mission_control_1")
            self.resize(640, 480)

            central = QtWidgets.QWidget()
            self.setCentralWidget(central)
            layout = QtWidgets.QVBoxLayout(central)
            layout.setSpacing(12)

            button_font = QtGui.QFont("Space Mono", 10)
            info_font = QtGui.QFont("Space Grotesk", 10)

            button_row = QtWidgets.QHBoxLayout()
            layout.addLayout(button_row)

            buttons = [
                ("Activate Camera", self._activate_camera),
                ("Deactivate Camera", self._deactivate_camera),
                ("Start", self._start_all),
                ("Stop", self._stop_all),
            ]
            for text, handler in buttons:
                btn = QtWidgets.QPushButton(text)
                btn.setFont(button_font)
                btn.clicked.connect(handler)
                btn.setCursor(QtCore.Qt.PointingHandCursor)
                button_row.addWidget(btn, stretch=1)

            control_group = QtWidgets.QGroupBox("Mission Configuration")
            control_layout = QtWidgets.QGridLayout(control_group)
            control_layout.setColumnStretch(0, 2)
            control_layout.setColumnStretch(1, 1)

            self._target_list = QtWidgets.QListWidget()
            self._target_list.setSelectionMode(QtWidgets.QAbstractItemView.MultiSelection)
            self._target_list.setMinimumHeight(120)
            for tid in self._available_target_ids:
                item = QtWidgets.QListWidgetItem(str(tid))
                self._target_list.addItem(item)
            self._target_list.itemSelectionChanged.connect(self._on_target_selection_changed)
            control_layout.addWidget(QtWidgets.QLabel("Select targets (2-6):"), 0, 0)
            control_layout.addWidget(self._target_list, 1, 0)

            self._radius_spin = QtWidgets.QDoubleSpinBox()
            self._radius_spin.setRange(10.0, 5000.0)
            self._radius_spin.setSingleStep(10.0)
            self._radius_spin.setSuffix(" mm")
            self._radius_spin.valueChanged.connect(self._update_switch_radius)
            control_layout.addWidget(QtWidgets.QLabel("Switch radius:"), 0, 1)
            control_layout.addWidget(self._radius_spin, 1, 1)

            layout.addWidget(control_group)

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
                ("Active Target ID", "active_target_id"),
                ("Distance -> Active [mm]", "distance_to_active_mm"),
                ("Switch Radius [mm]", "switch_radius_mm"),
                ("Queue Order", "queue_order"),
            ]

            for label_text, key in fields:
                lbl = QtWidgets.QLabel("-")
                lbl.setFont(info_font)
                self._value_labels[key] = lbl
                stats_layout.addRow(label_text + ":", lbl)

            layout.addWidget(stats_group)

            self._status_bar = QtWidgets.QStatusBar()
            self.setStatusBar(self._status_bar)

            self._selection_blocked = False
            self._apply_target_selection_to_ui(self._current_target_ids)
            self._radius_spin.setValue(self._current_switch_radius)

            self._timer = QtCore.QTimer(self)
            self._timer.timeout.connect(self._refresh_values)
            self._timer.start(100)

        def _on_target_selection_changed(self) -> None:
            if self._selection_blocked:
                return
            selected = [int(item.text()) for item in self._target_list.selectedItems()]
            if not self._try_update_targets(selected):
                self._apply_target_selection_to_ui(self._current_target_ids)

        def _apply_target_selection_to_ui(self, selection: Tuple[int, ...]) -> None:  # type: ignore[override]
            self._selection_blocked = True
            try:
                self._target_list.clearSelection()
                for idx in range(self._target_list.count()):
                    item = self._target_list.item(idx)
                    if int(item.text()) in selection:
                        item.setSelected(True)
            finally:
                self._selection_blocked = False

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
            self._value_labels["active_target_id"].setText(str(state.active_target_id))
            self._value_labels["distance_to_active_mm"].setText(f"{state.distance_to_active_mm:8.2f}")
            self._value_labels["switch_radius_mm"].setText(f"{state.switch_radius_mm:8.2f}")
            queue_text = ", ".join(str(tid) for tid in state.queue_order) if state.queue_order else "-"
            self._value_labels["queue_order"].setText(queue_text)

        def _handle_error(self, message: str) -> None:  # type: ignore[override]
            self._status_bar.showMessage(message, 4000)

        def closeEvent(self, event):  # type: ignore[override]
            self._deactivate_camera()
            super().closeEvent(event)


    class PySideAppWrapper:
        def __init__(
            self,
            vision: VisionSystem,
            mission: TargetQueueController,
            sender: UDPSender,
            state_supplier: Callable[[], IDESAState],
            send_hz: float,
            available_target_ids: Sequence[int],
            initial_target_ids: Sequence[int],
            initial_switch_radius: float,
        ) -> None:
            self._vision = vision
            self._mission = mission
            self._sender = sender
            self._state_supplier = state_supplier
            self._send_hz = send_hz
            self._available_target_ids = available_target_ids
            self._initial_target_ids = initial_target_ids
            self._initial_switch_radius = initial_switch_radius

        def run(self) -> None:
            import sys

            app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
            window = MissionControlWindow(
                self._vision,
                self._mission,
                self._sender,
                self._state_supplier,
                self._send_hz,
                self._available_target_ids,
                self._initial_target_ids,
                self._initial_switch_radius,
            )
            window.show()
            app.exec()

        def stop(self) -> None:
            self._sender.stop()
            self._mission.stop()
            self._vision.stop_tracking()
            self._vision.stop_camera()


else:

    class TkMissionControl(tk.Tk, _ControllerMixin):
        def __init__(
            self,
            vision: VisionSystem,
            mission: TargetQueueController,
            sender: UDPSender,
            state_supplier: Callable[[], IDESAState],
            send_hz: float,
            available_target_ids: Sequence[int],
            initial_target_ids: Sequence[int],
            initial_switch_radius: float,
        ) -> None:
            tk.Tk.__init__(self)
            _ControllerMixin.__init__(
                self,
                vision,
                mission,
                sender,
                state_supplier,
                send_hz,
                available_target_ids,
                initial_target_ids,
                initial_switch_radius,
            )

            self.title("GUI_mission_control_1")
            self.geometry("700x520")
            self.configure(background="#0f172a")

            style = ttk.Style(self)
            style.theme_use("clam")
            style.configure("TButton", font=("Futura", 11), padding=6)
            style.configure("TLabel", background="#0f172a", foreground="#e2e8f0", font=("Futura", 11))

            button_frame = ttk.Frame(self)
            button_frame.pack(fill="x", pady=10, padx=12)

            buttons = [
                ("Activate Camera", self._activate_camera),
                ("Deactivate Camera", self._deactivate_camera),
                ("Start", self._start_all),
                ("Stop", self._stop_all),
            ]
            for col, (text, handler) in enumerate(buttons):
                ttk.Button(button_frame, text=text, command=handler).grid(row=0, column=col, sticky="nsew", padx=4)
                button_frame.columnconfigure(col, weight=1)

            config_frame = ttk.LabelFrame(self, text="Mission Configuration")
            config_frame.pack(fill="x", padx=12, pady=(0, 12))

            ttk.Label(config_frame, text="Select targets (2-6):").grid(row=0, column=0, sticky="w", padx=8, pady=4)
            self._selection_blocked = False
            self._target_list = tk.Listbox(config_frame, selectmode=tk.MULTIPLE, exportselection=False, height=6)
            for idx, tid in enumerate(self._available_target_ids):
                self._target_list.insert(idx, str(tid))
            self._target_list.bind("<<ListboxSelect>>", self._on_target_list_changed)
            self._target_list.grid(row=1, column=0, padx=8, pady=4, sticky="nsew")

            ttk.Label(config_frame, text="Switch radius [mm]:").grid(row=0, column=1, sticky="w", padx=8, pady=4)
            self._radius_var = tk.DoubleVar(value=self._current_switch_radius)
            self._radius_spin = tk.Spinbox(
                config_frame,
                from_=10,
                to=5000,
                increment=10,
                textvariable=self._radius_var,
                width=8,
                command=self._on_radius_spin,
            )
            self._radius_var.trace_add("write", lambda *_: self._on_radius_spin())
            self._radius_spin.grid(row=1, column=1, padx=8, pady=4, sticky="w")

            config_frame.columnconfigure(0, weight=2)
            config_frame.columnconfigure(1, weight=1)

            stats_frame = ttk.LabelFrame(self, text="Live Telemetry")
            stats_frame.pack(fill="both", expand=True, padx=12, pady=12)

            self._value_labels: Dict[str, ttk.Label] = {}
            fields = [
                ("Robot X [mm]", "robot_x_mm"),
                ("Robot Y [mm]", "robot_y_mm"),
                ("Robot Yaw [deg]", "robot_yaw_deg"),
                ("Target X [mm]", "target_x_mm"),
                ("Target Y [mm]", "target_y_mm"),
                ("Robot Detected", "robot_detected"),
                ("Target Detected", "target_detected"),
                ("FPS", "fps"),
                ("Active Target ID", "active_target_id"),
                ("Distance -> Active [mm]", "distance_to_active_mm"),
                ("Switch Radius [mm]", "switch_radius_mm"),
                ("Queue Order", "queue_order"),
            ]

            for idx, (label_text, key) in enumerate(fields):
                ttk.Label(stats_frame, text=label_text).grid(row=idx, column=0, sticky="w", padx=8, pady=4)
                value_lbl = ttk.Label(stats_frame, text="-")
                value_lbl.grid(row=idx, column=1, sticky="e", padx=8, pady=4)
                self._value_labels[key] = value_lbl

            self._status_var = tk.StringVar(value="Idle")
            status_bar = ttk.Label(self, textvariable=self._status_var, anchor="w")
            status_bar.pack(fill="x", padx=12, pady=(0, 8))

            self._apply_target_selection_to_ui(self._current_target_ids)

            self.protocol("WM_DELETE_WINDOW", self._on_close)
            self._schedule_refresh()

        def _on_target_list_changed(self, _event=None) -> None:
            if self._selection_blocked:
                return
            selection = [int(self._target_list.get(i)) for i in self._target_list.curselection()]
            if not self._try_update_targets(selection):
                self._apply_target_selection_to_ui(self._current_target_ids)

        def _apply_target_selection_to_ui(self, selection: Tuple[int, ...]) -> None:  # type: ignore[override]
            self._selection_blocked = True
            try:
                self._target_list.selection_clear(0, tk.END)
                lookup = set(selection)
                for idx, tid in enumerate(self._available_target_ids):
                    if tid in lookup:
                        self._target_list.selection_set(idx)
            finally:
                self._selection_blocked = False

        def _on_radius_spin(self) -> None:
            try:
                value = float(self._radius_var.get())
            except tk.TclError:
                return
            self._update_switch_radius(value)

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
            self._value_labels["active_target_id"].config(text=str(state.active_target_id))
            self._value_labels["distance_to_active_mm"].config(text=f"{state.distance_to_active_mm:8.2f}")
            self._value_labels["switch_radius_mm"].config(text=f"{state.switch_radius_mm:8.2f}")
            queue_text = ", ".join(str(tid) for tid in state.queue_order) if state.queue_order else "-"
            self._value_labels["queue_order"].config(text=queue_text)

        def _handle_error(self, message: str) -> None:  # type: ignore[override]
            self._status_var.set(message)

        def _on_close(self) -> None:
            self._deactivate_camera()
            self.destroy()

        def run(self) -> None:
            self.mainloop()

        def stop(self) -> None:
            self._sender.stop()
            self._mission.stop()
            self._vision.stop_tracking()
            self._vision.stop_camera()


def create_gui(
    vision: VisionSystem,
    mission: TargetQueueController,
    sender: UDPSender,
    state_supplier: Callable[[], IDESAState],
    send_hz: float,
    available_target_ids: Sequence[int],
    default_target_ids: Sequence[int],
    switch_radius_mm: float,
):
    """Factory that hides whether PySide6 or Tkinter is backing the GUI."""
    if PYSIDE_AVAILABLE:
        return PySideAppWrapper(
            vision,
            mission,
            sender,
            state_supplier,
            send_hz,
            available_target_ids,
            default_target_ids,
            switch_radius_mm,
        )
    return TkMissionControl(
        vision,
        mission,
        sender,
        state_supplier,
        send_hz,
        available_target_ids,
        default_target_ids,
        switch_radius_mm,
    )
