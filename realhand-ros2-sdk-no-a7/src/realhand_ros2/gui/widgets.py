"""Qt widgets used by the RealHand ROS2 GUI."""

from __future__ import annotations

import csv
import json
import os
import signal
import time
from collections.abc import Callable
from typing import Any

from PyQt5 import QtCore, QtGui, QtWidgets
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from realhand_ros2.arm.model import ARM_MODELS, get_arm_model_spec
from realhand_ros2.gui.autodetect import (
    DEFAULT_CAN_BITRATE,
    DetectedHand,
    autodetect_hands,
    parse_serial_identity,
)
from realhand_ros2.gui.presets import get_gui_hand_config
from realhand_ros2.hand.model import get_hand_model_spec


LOOP_TIME_MS = 1000
PUBLISH_INTERVAL_MS = 100
LIVE_COMMAND_INTERVAL_MS = 100
BLOCKING_HEATMAP_INTERVAL_MS = 250
BLOCKING_HEATMAP_TIMEOUT_MS = 1000
MAX_JOINT_POSITION = 100
DEFAULT_MODEL = "L20"
DEFAULT_SIDE = "right"
DEFAULT_FAULT_REPORT_DIR = os.environ.get(
    "REALHAND_FAULT_REPORT_DIR",
    os.path.join(os.path.expanduser("~"), "realhand_fault_reports"),
)
SENSOR_READ_MODES = ("stream", "snapshot", "get_blocking")
SENSOR_READ_MODE_ALIASES = {"blocking": "get_blocking"}
DEFAULT_SENSOR_READ_MODE = "stream"
HAND_SENSOR_REQUESTS = (
    "angle",
    "speed",
    "torque",
    "acceleration",
    "temperature",
    "current",
    "fault",
    "force_sensor",
    "touch",
)
AUTO_GRAB_START_TORQUE = 10
AUTO_GRAB_MAX_DURATION_S = 10.0


GUI_STYLESHEET = """
QWidget {
    font-family: 'Microsoft YaHei', 'SimHei', sans-serif;
    font-size: 12px;
}
QGroupBox {
    border: 1px solid #CCCCCC;
    border-radius: 6px;
    margin-top: 6px;
    padding: 10px;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 10px;
    padding: 0 5px 0 5px;
    color: #165DFF;
    font-weight: bold;
}
QSlider::groove:horizontal {
    border: 1px solid #999999;
    height: 8px;
    border-radius: 4px;
    background: #CCCCCC;
    margin: 2px 0;
}
QSlider::handle:horizontal {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #165DFF, stop:1 #0E42D2);
    border: 1px solid #5C8AFF;
    width: 18px;
    margin: -5px 0;
    border-radius: 9px;
}
QPushButton {
    background-color: #E0E0E0;
    border: 1px solid #CCCCCC;
    border-radius: 4px;
    padding: 5px 10px;
    min-width: 80px;
}
QPushButton:hover { background-color: #F0F0F0; }
QPushButton:pressed { background-color: #D0D0D0; }
QPushButton[category="preset"] {
    background-color: #E6F7FF;
    color: #1890FF;
    border-color: #91D5FF;
}
QPushButton[category="preset"]:hover { background-color: #B3E0FF; }
QPushButton[category="action"] {
    background-color: #FFF7E6;
    color: #FA8C16;
    border-color: #FFD591;
}
QPushButton[category="danger"] {
    background-color: #FFF1F0;
    color: #F5222D;
    border-color: #FFCCC7;
}
QLabel#StatusLabel {
    padding: 5px;
    border-radius: 4px;
}
QLabel#StatusInfo {
    background-color: #F0F7FF;
    color: #0066CC;
}
QLabel#StatusError {
    background-color: #FFF0F0;
    color: #CC0000;
}
QTextEdit#ValueDisplay {
    background-color: #F8F8F8;
    border: 1px solid #CCCCCC;
    border-radius: 4px;
    padding: 10px;
    font-family: Consolas, monospace;
    font-size: 12px;
}
"""


class DotMatrixWidget(QtWidgets.QWidget):
    """Small touch heatmap widget compatible with the Python SDK GUI layout."""

    def __init__(
        self,
        parent: QtWidgets.QWidget | None = None,
        rows: int = 12,
        cols: int = 6,
        cell_px: int = 12,
    ) -> None:
        super().__init__(parent)
        self.rows = rows
        self.cols = cols
        self.cell_px = cell_px
        self.data = [[0.0 for _ in range(cols)] for _ in range(rows)]
        self.setMinimumSize(cols * cell_px + 4, rows * cell_px + 4)
        self.setMaximumSize(cols * cell_px + 4, rows * cell_px + 4)

    def set_data(self, data: Any) -> None:
        self.data = self._normalize_matrix(data)
        self.update()

    def _normalize_matrix(self, data: Any) -> list[list[float]]:
        flat = _flatten_numeric(data)
        needed = self.rows * self.cols
        values = [0.0 for _ in range(needed)]
        for index, value in enumerate(flat[:needed]):
            values[index] = float(value)
        return [
            values[row * self.cols : (row + 1) * self.cols]
            for row in range(self.rows)
        ]

    def paintEvent(self, event) -> None:  # noqa: N802
        painter = QtGui.QPainter(self)
        painter.fillRect(self.rect(), QtGui.QColor("white"))
        values = [value for row in self.data for value in row]
        max_value = max(values) if values else 0.0
        denom = max(max_value, 1.0)
        for row in range(self.rows):
            for col in range(self.cols):
                value = max(0.0, min(1.0, float(self.data[row][col]) / denom))
                if max_value <= 0:
                    color = QtGui.QColor("#C8C8C8")
                else:
                    red = int(60 + 195 * value)
                    green = int(190 * (1.0 - value))
                    blue = int(255 * (1.0 - value))
                    color = QtGui.QColor(red, green, blue)
                x_pos = 2 + col * self.cell_px
                y_pos = 2 + row * self.cell_px
                painter.fillRect(
                    x_pos, y_pos, self.cell_px - 1, self.cell_px - 1, color
                )


class MatrixDisplayWidget(QtWidgets.QWidget):
    """Five-finger touch heatmap panel."""

    def __init__(
        self,
        parent: QtWidgets.QWidget | None = None,
        rows: int = 12,
        cols: int = 6,
        cell_px: int = 12,
    ) -> None:
        super().__init__(parent)
        self.rows = rows
        self.cols = cols
        self.cell_px = cell_px
        self.finger_matrices: dict[str, DotMatrixWidget] = {}
        self._init_ui()

    def _init_ui(self) -> None:
        main_layout = QtWidgets.QVBoxLayout(self)
        main_layout.setSpacing(10)
        main_layout.setContentsMargins(5, 5, 5, 5)

        row_layout = QtWidgets.QHBoxLayout()
        for display_name, key in (
            ("Thumb", "thumb_matrix"),
            ("Index", "index_matrix"),
            ("Middle", "middle_matrix"),
            ("Ring", "ring_matrix"),
            ("Pinky", "pinky_matrix"),
        ):
            row_layout.addWidget(self._create_finger_frame(display_name, key))
        row_layout.addStretch()

        main_layout.addLayout(row_layout)
        main_layout.addStretch()

    def _create_finger_frame(self, display_name: str, key: str) -> QtWidgets.QWidget:
        frame = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(frame)
        layout.setSpacing(5)
        label = QtWidgets.QLabel(display_name)
        label.setAlignment(QtCore.Qt.AlignCenter)
        label.setStyleSheet("font-weight: bold;")
        matrix = DotMatrixWidget(rows=self.rows, cols=self.cols, cell_px=self.cell_px)
        layout.addWidget(label)
        layout.addWidget(matrix, 0, QtCore.Qt.AlignCenter)
        self.finger_matrices[key] = matrix
        return frame

    def update_matrix_data(self, key: str, data: Any) -> None:
        if key == "little_matrix":
            key = "pinky_matrix"
        matrix = self.finger_matrices.get(key)
        if matrix is not None:
            matrix.set_data(data)


class SliderRow(QtWidgets.QWidget):
    value_changed = QtCore.pyqtSignal()

    def __init__(self, label: str, value: int = 50, parent=None) -> None:
        super().__init__(parent)
        self.label = QtWidgets.QLabel(label)
        self.label.setMinimumWidth(150)
        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider.setRange(0, 100)
        self.spin = QtWidgets.QSpinBox()
        self.spin.setRange(0, 100)
        self.spin.setSuffix(" %")
        self.slider.setValue(value)
        self.spin.setValue(value)

        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(0, 2, 0, 2)
        layout.addWidget(self.label)
        layout.addWidget(self.slider, 1)
        layout.addWidget(self.spin)

        self.slider.valueChanged.connect(self.spin.setValue)
        self.spin.valueChanged.connect(self.slider.setValue)
        self.slider.valueChanged.connect(lambda _value: self.value_changed.emit())

    def value(self) -> int:
        return int(self.slider.value())

    def set_value(self, value: int) -> None:
        self.slider.setValue(max(0, min(100, int(value))))


class DoubleRow(QtWidgets.QWidget):
    value_changed = QtCore.pyqtSignal()

    def __init__(
        self,
        label: str,
        value: float = 0.0,
        minimum: float = -6.283,
        maximum: float = 6.283,
        step: float = 0.01,
        suffix: str = "",
        parent=None,
    ) -> None:
        super().__init__(parent)
        self.label = QtWidgets.QLabel(label)
        self.label.setMinimumWidth(110)
        self.spin = QtWidgets.QDoubleSpinBox()
        self.spin.setDecimals(4)
        self.spin.setRange(minimum, maximum)
        self.spin.setSingleStep(step)
        self.spin.setValue(value)
        if suffix:
            self.spin.setSuffix(f" {suffix}")

        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(0, 2, 0, 2)
        layout.addWidget(self.label)
        layout.addWidget(self.spin, 1)
        self.spin.valueChanged.connect(lambda _value: self.value_changed.emit())

    def value(self) -> float:
        return float(self.spin.value())

    def set_value(self, value: float) -> None:
        self.spin.setValue(float(value))


class AutoDetectWorker(QtCore.QObject):
    finished = QtCore.pyqtSignal(object, object)
    failed = QtCore.pyqtSignal(str)

    def __init__(
        self,
        *,
        interface_type: str,
        setup_can: bool = True,
        bitrate: int = DEFAULT_CAN_BITRATE,
    ) -> None:
        super().__init__()
        self.interface_type = interface_type
        self.setup_can = setup_can
        self.bitrate = bitrate

    @QtCore.pyqtSlot()
    def run(self) -> None:
        try:
            hands, messages = autodetect_hands(
                interface_type=self.interface_type,
                bitrate=self.bitrate,
                setup_can=self.setup_can,
            )
        except Exception as exc:
            self.failed.emit(str(exc))
            return
        self.finished.emit(hands, messages)


class HandPanel(QtWidgets.QWidget):
    status_updated = QtCore.pyqtSignal(str, str)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.publish_command: Callable[
            [list[str], list[float] | None, list[float] | None, list[float] | None],
            None,
        ] | None = None
        self.publish_json: Callable[[str], None] | None = None
        self.configure_topics: Callable[[str], None] | None = None

        self.finger_order = ["thumb", "index", "middle", "ring", "pinky"]
        self.hand_config = get_gui_hand_config(DEFAULT_MODEL)
        self.joint_names = list(get_hand_model_spec(DEFAULT_MODEL).joint_names)
        self.sliders: list[QtWidgets.QSlider] = []
        self.slider_labels: list[QtWidgets.QLabel] = []
        self.preset_buttons: list[QtWidgets.QPushButton] = []
        self.per_joint_speed_sliders: list[QtWidgets.QSlider] = []
        self.per_joint_torque_sliders: list[QtWidgets.QSlider] = []
        self.realtime_labels: dict[str, dict[str, QtWidgets.QLabel]] = {}
        self.latest_sensor_values: dict[str, list[Any]] = {}
        self.latest_touch_matrices: dict[str, Any] = {}
        self.latest_device_info: dict[str, Any] = {}
        self.latest_control_status: dict[str, Any] = {}
        self.latest_blocking_result: dict[str, Any] = {}
        self.sensor_read_mode = DEFAULT_SENSOR_READ_MODE
        self._request_counter = 0
        self._blocking_heatmap_request_id: str | None = None
        self._pending_fault_report_id: str | None = None
        self._pending_fault_report_path: str | None = None
        self._last_device_info_status: tuple[Any, ...] | None = None
        self._last_control_status_log: tuple[Any, ...] | None = None
        self._last_touch_keys_logged: tuple[str, ...] = ()
        self.prev_touch: dict[str, list[int]] = {}
        self.touch_history: dict[str, list[list[int]]] = {}
        self.last_slip_time: dict[str, int] = {}
        self._state_seen = False
        self.current_torque_values: list[int] = [
            100 for _ in self.hand_config.joint_names
        ]
        self._last_published_positions: list[int] | None = None
        self._syncing_per_joint_settings = False

        self.cycle_timer: QtCore.QTimer | None = None
        self.current_action_index = -1
        self.cycle_loop_active = False
        self.cycle_loop_index = -1
        self.cycle_loop_iterations = 0

        self.auto_grab_running = False
        self.auto_grab_sensor_fail_count = 0
        self.auto_grab_fingers_stopped: dict[str, bool] = {}
        self.auto_grab_above_count: dict[str, int] = {}
        self.auto_grab_baseline: dict[str, int] = {}
        self.auto_grab_debounce_limit = 2
        self.auto_grab_started_at = 0.0
        self.closed_threshold = 5
        self.backend_process: QtCore.QProcess | None = None
        self._backend_stop_requested = False
        self._backend_owned_by_gui = False
        self._detect_thread: QtCore.QThread | None = None
        self._detect_worker: AutoDetectWorker | None = None
        self._auto_detect_started = False

        self.model_combo = QtWidgets.QComboBox()
        self.model_combo.addItems(["L6", "O6", "L20", "L20lite", "L25"])
        self.model_combo.setCurrentText(DEFAULT_MODEL)
        self.side_combo = QtWidgets.QComboBox()
        self.side_combo.addItems(["left", "right"])
        self.side_combo.setCurrentText(DEFAULT_SIDE)
        self.interface_edit = QtWidgets.QLineEdit("can0")
        self.interface_edit.setMaximumWidth(90)
        self.interface_type_combo = QtWidgets.QComboBox()
        self.interface_type_combo.addItems(["socketcan"])

        self.live_speed_timer = QtCore.QTimer(self)
        self.live_speed_timer.setInterval(LIVE_COMMAND_INTERVAL_MS)
        self.live_speed_timer.setSingleShot(True)
        self.live_speed_timer.timeout.connect(self._publish_live_speed)

        self.live_torque_timer = QtCore.QTimer(self)
        self.live_torque_timer.setInterval(LIVE_COMMAND_INTERVAL_MS)
        self.live_torque_timer.setSingleShot(True)
        self.live_torque_timer.timeout.connect(self._publish_live_torque)

        self.live_joint_speed_timer = QtCore.QTimer(self)
        self.live_joint_speed_timer.setInterval(LIVE_COMMAND_INTERVAL_MS)
        self.live_joint_speed_timer.setSingleShot(True)
        self.live_joint_speed_timer.timeout.connect(self._publish_live_joint_speeds)

        self.live_joint_torque_timer = QtCore.QTimer(self)
        self.live_joint_torque_timer.setInterval(LIVE_COMMAND_INTERVAL_MS)
        self.live_joint_torque_timer.setSingleShot(True)
        self.live_joint_torque_timer.timeout.connect(self._publish_live_joint_torques)

        self.auto_grab_timer = QtCore.QTimer(self)
        self.auto_grab_timer.setInterval(50)
        self.auto_grab_timer.timeout.connect(self._auto_grab_step)

        self.blocking_heatmap_timer = QtCore.QTimer(self)
        self.blocking_heatmap_timer.setInterval(BLOCKING_HEATMAP_INTERVAL_MS)
        self.blocking_heatmap_timer.timeout.connect(self.request_blocking_heatmap)

        self.publish_timer = QtCore.QTimer(self)
        self.publish_timer.setInterval(PUBLISH_INTERVAL_MS)
        self.publish_timer.timeout.connect(self.publish_joint_state)
        self.publish_timer.start()

        self._init_ui()
        self.status_updated.connect(self.update_status)
        self.model_combo.currentTextChanged.connect(
            lambda _text: self._rebuild_controls()
        )
        self.side_combo.currentTextChanged.connect(lambda _text: self._apply_topics())
        self._rebuild_controls()
        QtCore.QTimer.singleShot(250, self._auto_detect_once)

    def _init_ui(self) -> None:
        self.setStyleSheet(GUI_STYLESHEET)
        main_layout = QtWidgets.QVBoxLayout(self)

        target_bar = QtWidgets.QHBoxLayout()
        target_bar.addWidget(QtWidgets.QLabel("Model"))
        target_bar.addWidget(self.model_combo)
        target_bar.addWidget(QtWidgets.QLabel("Side"))
        target_bar.addWidget(self.side_combo)
        target_bar.addWidget(QtWidgets.QLabel("Interface"))
        target_bar.addWidget(self.interface_edit)
        target_bar.addWidget(QtWidgets.QLabel("Backend"))
        target_bar.addWidget(self.interface_type_combo)
        self.detect_button = QtWidgets.QPushButton("Auto Detect")
        self.detect_button.setProperty("category", "action")
        self.detect_button.clicked.connect(self.auto_detect_hand)
        target_bar.addWidget(self.detect_button)
        apply_button = QtWidgets.QPushButton("Apply Topics")
        apply_button.setProperty("category", "action")
        apply_button.clicked.connect(self._apply_topics)
        target_bar.addWidget(apply_button)
        self.backend_button = QtWidgets.QPushButton("Start Backend")
        self.backend_button.setProperty("category", "action")
        self.backend_button.clicked.connect(self.on_backend_button_clicked)
        target_bar.addWidget(self.backend_button)
        target_bar.addStretch(1)
        main_layout.addLayout(target_bar)

        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        splitter.addWidget(self.create_joint_control_panel())
        splitter.addWidget(self.create_preset_actions_panel())
        splitter.addWidget(self.create_status_monitor_panel())
        splitter.setSizes([320, 520, 360])
        main_layout.addWidget(splitter, stretch=1)
        main_layout.addWidget(self.create_value_display_panel(), stretch=0)

    def create_joint_control_panel(self) -> QtWidgets.QWidget:
        panel = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(panel)
        self.joint_title = QtWidgets.QLabel()
        self.joint_title.setFont(QtGui.QFont("Microsoft YaHei", 14, QtGui.QFont.Bold))
        layout.addWidget(self.joint_title)

        scroll_area = QtWidgets.QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setFrameShape(QtWidgets.QFrame.NoFrame)
        scroll_content = QtWidgets.QWidget()
        self.sliders_layout = QtWidgets.QGridLayout(scroll_content)
        self.sliders_layout.setSpacing(10)
        scroll_area.setWidget(scroll_content)
        layout.addWidget(scroll_area)
        return panel

    def create_preset_actions_panel(self) -> QtWidgets.QWidget:
        panel = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(panel)

        matrix_group = QtWidgets.QGroupBox("Finger Touch Heatmap")
        matrix_layout = QtWidgets.QVBoxLayout(matrix_group)
        self.matrix_display = MatrixDisplayWidget(rows=12, cols=6, cell_px=12)
        self.matrix_display.setSizePolicy(
            QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed
        )
        matrix_layout.addWidget(self.matrix_display)
        layout.addWidget(matrix_group, stretch=0)

        preset_group = QtWidgets.QGroupBox("System Presets")
        self.preset_layout = QtWidgets.QGridLayout(preset_group)
        self.preset_layout.setSpacing(8)
        preset_group.setSizePolicy(
            QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding
        )
        layout.addWidget(preset_group, stretch=1)

        actions = QtWidgets.QHBoxLayout()
        self.cycle_button = QtWidgets.QPushButton("Cycle Preset Actions")
        self.cycle_button.setProperty("category", "action")
        self.cycle_button.clicked.connect(self.on_cycle_clicked)
        actions.addWidget(self.cycle_button)

        self.home_button = QtWidgets.QPushButton("Return to Home")
        self.home_button.setProperty("category", "action")
        self.home_button.clicked.connect(self.on_home_clicked)
        actions.addWidget(self.home_button)

        self.stop_button = QtWidgets.QPushButton("Stop All Actions")
        self.stop_button.setProperty("category", "danger")
        self.stop_button.clicked.connect(self.on_stop_clicked)
        actions.addWidget(self.stop_button)
        layout.addLayout(actions)
        return panel

    def create_status_monitor_panel(self) -> QtWidgets.QWidget:
        panel = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(panel)
        title = QtWidgets.QLabel("Status Monitor")
        title.setFont(QtGui.QFont("Microsoft YaHei", 14, QtGui.QFont.Bold))
        layout.addWidget(title)

        quick_group = QtWidgets.QGroupBox("Quick Settings")
        quick_layout = QtWidgets.QVBoxLayout(quick_group)
        quick_layout.addLayout(self._sensor_read_mode_row())
        quick_layout.addLayout(self._live_slider_row("Speed:", "speed"))
        quick_layout.addLayout(self._live_slider_row("Torque:", "torque"))
        layout.addWidget(quick_group)

        realtime_group = QtWidgets.QGroupBox("Realtime Sensors (Per Finger)")
        realtime_layout = QtWidgets.QGridLayout(realtime_group)
        headers = ["Finger", "Angle", "Torque", "Temp", "Current"]
        for col, text in enumerate(headers):
            realtime_layout.addWidget(QtWidgets.QLabel(text), 0, col)
        titles = {
            "thumb": "Thumb",
            "index": "Index",
            "middle": "Middle",
            "ring": "Ring",
            "pinky": "Pinky",
        }
        for row, finger in enumerate(self.finger_order, start=1):
            realtime_layout.addWidget(QtWidgets.QLabel(titles[finger]), row, 0)
            self.realtime_labels[finger] = {}
            for col, key in enumerate(
                ("angle", "torque", "temperature", "current"), start=1
            ):
                label = QtWidgets.QLabel("--")
                realtime_layout.addWidget(label, row, col)
                self.realtime_labels[finger][key] = label
        layout.addWidget(realtime_group)

        tabs = QtWidgets.QTabWidget()
        tabs.addTab(self._system_info_tab(), "System Info")
        tabs.addTab(self._joint_settings_tab(), "Joint Settings")
        tabs.addTab(self._touch_control_tab(), "Touch Control")
        tabs.addTab(self._fault_report_tab(), "Hand Test")
        tabs.addTab(self._status_log_tab(), "Status Log")
        self.status_tabs = tabs
        layout.addWidget(tabs)
        return panel

    def _sensor_read_mode_row(self) -> QtWidgets.QVBoxLayout:
        layout = QtWidgets.QVBoxLayout()
        mode_row = QtWidgets.QHBoxLayout()
        mode_row.addWidget(QtWidgets.QLabel("Sensor read:"))

        self.sensor_read_mode_combo = QtWidgets.QComboBox()
        self.sensor_read_mode_combo.addItems(list(SENSOR_READ_MODES))
        self.sensor_read_mode_combo.setCurrentText(self.sensor_read_mode)
        self.sensor_read_mode_combo.currentTextChanged.connect(
            self.on_sensor_read_mode_changed
        )
        mode_row.addWidget(self.sensor_read_mode_combo)

        mode_row.addWidget(QtWidgets.QLabel("Sensor"))
        self.sensor_request_combo = QtWidgets.QComboBox()
        self.sensor_request_combo.addItems(list(HAND_SENSOR_REQUESTS))
        self.sensor_request_combo.setCurrentText("fault")
        mode_row.addWidget(self.sensor_request_combo)

        mode_row.addWidget(QtWidgets.QLabel("Timeout"))
        self.blocking_timeout_spin = QtWidgets.QSpinBox()
        self.blocking_timeout_spin.setRange(50, 10000)
        self.blocking_timeout_spin.setSingleStep(50)
        self.blocking_timeout_spin.setValue(500)
        self.blocking_timeout_spin.setSuffix(" ms")
        mode_row.addWidget(self.blocking_timeout_spin)
        layout.addLayout(mode_row)

        action_row = QtWidgets.QHBoxLayout()
        for text, slot in (
            ("Start Poll", self.start_polling),
            ("Stop Poll", self.stop_polling),
            ("Start Stream", self.start_stream),
            ("Stop Stream", self.stop_stream),
            ("Snapshot", self.request_snapshot),
            ("Blocking", self.request_blocking),
        ):
            button = QtWidgets.QPushButton(text)
            button.setMinimumWidth(72)
            button.clicked.connect(slot)
            action_row.addWidget(button)
        layout.addLayout(action_row)

        self.sensor_status_label = QtWidgets.QLabel("Mode: stream")
        self.sensor_status_label.setObjectName("StatusInfo")
        self.sensor_status_label.setWordWrap(True)
        layout.addWidget(self.sensor_status_label)
        return layout

    def _live_slider_row(self, label_text: str, kind: str) -> QtWidgets.QHBoxLayout:
        row = QtWidgets.QHBoxLayout()
        row.addWidget(QtWidgets.QLabel(label_text))
        slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        slider.setRange(0, 100)
        slider.setValue(100)
        slider.setMinimumWidth(150)
        row.addWidget(slider)
        value_label = QtWidgets.QLabel("100")
        value_label.setMinimumWidth(30)
        row.addWidget(value_label)
        if kind == "speed":
            self.speed_slider = slider
            self.speed_val_lbl = value_label
            slider.valueChanged.connect(self.on_global_speed_changed)
        else:
            self.torque_slider = slider
            self.torque_val_lbl = value_label
            slider.valueChanged.connect(self.on_global_torque_changed)
        row.addStretch()
        return row

    def _joint_settings_tab(self) -> QtWidgets.QWidget:
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(widget)

        group = QtWidgets.QGroupBox("Per-Joint Controls")
        group.setCheckable(True)
        group.setChecked(True)
        group_layout = QtWidgets.QVBoxLayout(group)

        content = QtWidgets.QWidget()
        content_layout = QtWidgets.QVBoxLayout(content)
        control_tabs = QtWidgets.QTabWidget()
        self.per_joint_speed_layout = QtWidgets.QGridLayout()
        self.per_joint_torque_layout = QtWidgets.QGridLayout()
        control_tabs.addTab(
            self._scroll_for_grid(self.per_joint_speed_layout), "Speed"
        )
        control_tabs.addTab(
            self._scroll_for_grid(self.per_joint_torque_layout), "Torque"
        )
        content_layout.addWidget(control_tabs)
        group_layout.addWidget(content)
        group.toggled.connect(content.setVisible)

        layout.addWidget(group)
        layout.addStretch()
        return widget

    def _touch_control_tab(self) -> QtWidgets.QWidget:
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(widget)

        auto_group = QtWidgets.QGroupBox("Touch Auto Grab")
        auto_layout = QtWidgets.QVBoxLayout(auto_group)

        auto_settings = QtWidgets.QGridLayout()
        auto_settings.addWidget(QtWidgets.QLabel("Threshold"), 0, 0)
        self.auto_grab_threshold_spin = QtWidgets.QSpinBox()
        self.auto_grab_threshold_spin.setRange(1, 1000)
        self.auto_grab_threshold_spin.setValue(30)
        auto_settings.addWidget(self.auto_grab_threshold_spin, 0, 1)

        auto_settings.addWidget(QtWidgets.QLabel("Speed"), 0, 2)
        self.auto_grab_speed_spin = QtWidgets.QSpinBox()
        self.auto_grab_speed_spin.setRange(1, 100)
        self.auto_grab_speed_spin.setValue(50)
        auto_settings.addWidget(self.auto_grab_speed_spin, 0, 3)
        auto_layout.addLayout(auto_settings)

        auto_buttons = QtWidgets.QHBoxLayout()
        self.auto_grab_start_button = QtWidgets.QPushButton("Auto Grab")
        self.auto_grab_start_button.setProperty("category", "action")
        self.auto_grab_start_button.clicked.connect(self.start_auto_grab)
        auto_buttons.addWidget(self.auto_grab_start_button)

        self.auto_grab_stop_button = QtWidgets.QPushButton("Stop")
        self.auto_grab_stop_button.setProperty("category", "danger")
        self.auto_grab_stop_button.clicked.connect(self.stop_auto_grab)
        self.auto_grab_stop_button.setEnabled(False)
        auto_buttons.addWidget(self.auto_grab_stop_button)

        self.auto_grab_status_label = QtWidgets.QLabel("Ready")
        auto_buttons.addWidget(self.auto_grab_status_label)
        auto_buttons.addStretch()
        auto_layout.addLayout(auto_buttons)

        finger_row = QtWidgets.QHBoxLayout()
        self.auto_grab_finger_labels: dict[str, QtWidgets.QLabel] = {}
        for finger in self.finger_order:
            label = QtWidgets.QLabel(f"{finger[0].upper()}:--")
            label.setMinimumWidth(48)
            finger_row.addWidget(label)
            self.auto_grab_finger_labels[finger] = label
        finger_row.addStretch()
        auto_layout.addLayout(finger_row)
        layout.addWidget(auto_group)

        torque_group = QtWidgets.QGroupBox("Commanded Torque")
        torque_layout = QtWidgets.QVBoxLayout(torque_group)
        self.commanded_torque_summary_label = QtWidgets.QLabel("Command: --")
        torque_layout.addWidget(self.commanded_torque_summary_label)

        torque_finger_row = QtWidgets.QHBoxLayout()
        self.commanded_torque_finger_labels: dict[str, QtWidgets.QLabel] = {}
        for finger in self.finger_order:
            label = QtWidgets.QLabel(f"{finger[0].upper()}:--")
            label.setMinimumWidth(70)
            torque_finger_row.addWidget(label)
            self.commanded_torque_finger_labels[finger] = label
        torque_finger_row.addStretch()
        torque_layout.addLayout(torque_finger_row)

        self.last_torque_boost_label = QtWidgets.QLabel("Last boost: none")
        torque_layout.addWidget(self.last_torque_boost_label)
        layout.addWidget(torque_group)

        slip_group = QtWidgets.QGroupBox("Slip Detection")
        slip_layout = QtWidgets.QVBoxLayout(slip_group)

        slip_settings = QtWidgets.QGridLayout()
        slip_settings.addWidget(QtWidgets.QLabel("Contact Max >="), 0, 0)
        self.slip_contact_spin = QtWidgets.QSpinBox()
        self.slip_contact_spin.setRange(0, 10000)
        self.slip_contact_spin.setValue(5)
        slip_settings.addWidget(self.slip_contact_spin, 0, 1)

        slip_settings.addWidget(QtWidgets.QLabel("Mag Delta >="), 0, 2)
        self.slip_mag_spin = QtWidgets.QSpinBox()
        self.slip_mag_spin.setRange(0, 1000000)
        self.slip_mag_spin.setValue(50)
        slip_settings.addWidget(self.slip_mag_spin, 0, 3)

        slip_settings.addWidget(QtWidgets.QLabel("Loc Delta >="), 1, 0)
        self.slip_loc_spin = QtWidgets.QDoubleSpinBox()
        self.slip_loc_spin.setRange(0.0, 100.0)
        self.slip_loc_spin.setDecimals(1)
        self.slip_loc_spin.setSingleStep(0.5)
        self.slip_loc_spin.setValue(2.0)
        slip_settings.addWidget(self.slip_loc_spin, 1, 1)

        slip_settings.addWidget(QtWidgets.QLabel("Cooldown ms"), 1, 2)
        self.slip_cooldown_spin = QtWidgets.QSpinBox()
        self.slip_cooldown_spin.setRange(0, 60000)
        self.slip_cooldown_spin.setValue(500)
        slip_settings.addWidget(self.slip_cooldown_spin, 1, 3)
        slip_layout.addLayout(slip_settings)

        slip_options = QtWidgets.QHBoxLayout()
        self.slip_window_checkbox = QtWidgets.QCheckBox("Use N-frame detection")
        slip_options.addWidget(self.slip_window_checkbox)
        slip_options.addWidget(QtWidgets.QLabel("Frames"))
        self.slip_window_frames_spin = QtWidgets.QSpinBox()
        self.slip_window_frames_spin.setRange(2, 60)
        self.slip_window_frames_spin.setValue(5)
        slip_options.addWidget(self.slip_window_frames_spin)
        self.slip_torque_boost_checkbox = QtWidgets.QCheckBox("Boost torque on slip")
        slip_options.addWidget(self.slip_torque_boost_checkbox)
        slip_options.addWidget(QtWidgets.QLabel("Step"))
        self.slip_torque_boost_step_spin = QtWidgets.QSpinBox()
        self.slip_torque_boost_step_spin.setRange(1, 100)
        self.slip_torque_boost_step_spin.setValue(5)
        slip_options.addWidget(self.slip_torque_boost_step_spin)
        slip_options.addStretch()
        slip_layout.addLayout(slip_options)

        slip_status = QtWidgets.QHBoxLayout()
        self.slip_labels: dict[str, QtWidgets.QLabel] = {}
        for finger in self.finger_order:
            label = QtWidgets.QLabel(f"{finger[0].upper()}:--")
            label.setMinimumWidth(48)
            slip_status.addWidget(label)
            self.slip_labels[finger] = label
        slip_status.addStretch()
        slip_layout.addLayout(slip_status)
        layout.addWidget(slip_group)

        layout.addStretch()
        return widget

    def _fault_report_tab(self) -> QtWidgets.QWidget:
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(widget)

        output_group = QtWidgets.QGroupBox("Fault Check CSV")
        output_layout = QtWidgets.QGridLayout(output_group)
        output_layout.addWidget(QtWidgets.QLabel("Output folder"), 0, 0)
        self.fault_report_dir_edit = QtWidgets.QLineEdit(DEFAULT_FAULT_REPORT_DIR)
        output_layout.addWidget(self.fault_report_dir_edit, 0, 1)
        browse_btn = QtWidgets.QPushButton("Browse")
        browse_btn.clicked.connect(self._browse_fault_report_dir)
        output_layout.addWidget(browse_btn, 0, 2)

        output_layout.addWidget(QtWidgets.QLabel("Fault timeout"), 1, 0)
        self.fault_timeout_spin = QtWidgets.QSpinBox()
        self.fault_timeout_spin.setRange(50, 10000)
        self.fault_timeout_spin.setSingleStep(50)
        self.fault_timeout_spin.setValue(500)
        self.fault_timeout_spin.setSuffix(" ms")
        output_layout.addWidget(self.fault_timeout_spin, 1, 1)
        layout.addWidget(output_group)

        action_row = QtWidgets.QHBoxLayout()
        self.run_fault_report_btn = QtWidgets.QPushButton("Run Fault Check and Save CSV")
        self.run_fault_report_btn.setProperty("category", "action")
        self.run_fault_report_btn.clicked.connect(self.run_fault_report)
        action_row.addWidget(self.run_fault_report_btn)
        action_row.addStretch()
        layout.addLayout(action_row)

        self.fault_report_status_label = QtWidgets.QLabel("Ready")
        self.fault_report_status_label.setWordWrap(True)
        layout.addWidget(self.fault_report_status_label)

        preview_group = QtWidgets.QGroupBox("Last Fault Check")
        preview_layout = QtWidgets.QVBoxLayout(preview_group)
        self.fault_report_preview = QtWidgets.QTextEdit()
        self.fault_report_preview.setObjectName("ValueDisplay")
        self.fault_report_preview.setReadOnly(True)
        self.fault_report_preview.setMinimumHeight(180)
        preview_layout.addWidget(self.fault_report_preview)
        layout.addWidget(preview_group)

        layout.addStretch()
        return widget

    def _browse_fault_report_dir(self) -> None:
        current_dir = self.fault_report_dir_edit.text().strip()
        if not current_dir:
            current_dir = DEFAULT_FAULT_REPORT_DIR
        path = QtWidgets.QFileDialog.getExistingDirectory(
            self,
            "Choose fault report folder",
            current_dir,
        )
        if path:
            self.fault_report_dir_edit.setText(path)

    def _system_info_tab(self) -> QtWidgets.QWidget:
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(widget)

        conn_group = QtWidgets.QGroupBox("Connection Status")
        conn_layout = QtWidgets.QVBoxLayout(conn_group)
        self.connection_status = QtWidgets.QLabel("Waiting for ROS2 hand state")
        self.connection_status.setObjectName("StatusInfo")
        conn_layout.addWidget(self.connection_status)

        info_group = QtWidgets.QGroupBox("Hand Info")
        info_layout = QtWidgets.QVBoxLayout(info_group)
        self.info_label = QtWidgets.QLabel()
        self.info_label.setWordWrap(True)
        info_layout.addWidget(self.info_label)

        clear_faults_btn = QtWidgets.QPushButton("Clear Faults")
        clear_faults_btn.clicked.connect(self.clear_faults)
        info_layout.addWidget(clear_faults_btn)

        layout.addWidget(conn_group)
        layout.addWidget(info_group)
        layout.addStretch()
        return widget

    def _status_log_tab(self) -> QtWidgets.QWidget:
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(widget)
        self.status_log = QtWidgets.QLabel("Waiting for system startup...")
        self.status_log.setObjectName("StatusInfo")
        self.status_log.setWordWrap(True)
        self.status_log.setMinimumHeight(300)
        layout.addWidget(self.status_log)
        clear_btn = QtWidgets.QPushButton("Clear Log")
        clear_btn.clicked.connect(self.clear_status_log)
        layout.addWidget(clear_btn)
        return widget

    def create_value_display_panel(self) -> QtWidgets.QGroupBox:
        panel = QtWidgets.QGroupBox("Joint Value List")
        layout = QtWidgets.QVBoxLayout(panel)
        layout.setContentsMargins(10, 20, 10, 20)
        self.value_display = QtWidgets.QTextEdit()
        self.value_display.setObjectName("ValueDisplay")
        self.value_display.setReadOnly(True)
        self.value_display.setMinimumHeight(60)
        self.value_display.setMaximumHeight(80)
        layout.addWidget(self.value_display)
        return panel

    def _rebuild_controls(self) -> None:
        model = self.model_combo.currentText()
        spec = get_hand_model_spec(model)
        self.hand_config = get_gui_hand_config(model)
        self.joint_names = list(spec.joint_names)
        self._state_seen = False
        self.current_torque_values = [
            self.torque_slider.value() for _ in self.hand_config.joint_names
        ]
        self._last_published_positions = None
        self.joint_title.setText(f"Joint Control - {model}")
        self._create_joint_sliders()
        self._create_system_preset_buttons()
        self._create_per_joint_sliders("speed")
        self._create_per_joint_sliders("torque")
        self.update_value_display()
        self._reset_sensor_labels()
        self._update_commanded_torque_display()
        self.update_system_info()
        self.status_updated.emit("info", f"Loaded {model} controls")

    def _reset_sensor_labels(self) -> None:
        current_supported = self.model_combo.currentText() == "L6"
        for finger in self.finger_order:
            for key, label in self.realtime_labels.get(finger, {}).items():
                if key == "current" and not current_supported:
                    label.setText("N/A")
                else:
                    label.setText("--")

    def _create_joint_sliders(self) -> None:
        self._clear_layout(self.sliders_layout)
        self.sliders = []
        self.slider_labels = []
        initial = self._fit_values(
            self.hand_config.init_pos, len(self.hand_config.joint_names), 50
        )
        for row, (name, value) in enumerate(
            zip(self.hand_config.joint_names, initial, strict=False)
        ):
            label = QtWidgets.QLabel(f"{name}: {value}")
            label.setMinimumWidth(150)
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            slider.setRange(0, 100)
            slider.setValue(int(value))
            slider.valueChanged.connect(
                lambda val, idx=row: self.on_slider_value_changed(idx, val)
            )
            self.sliders_layout.addWidget(label, row, 0)
            self.sliders_layout.addWidget(slider, row, 1)
            self.slider_labels.append(label)
            self.sliders.append(slider)

    def _create_system_preset_buttons(self) -> None:
        self._clear_layout(self.preset_layout)
        self.preset_buttons = []
        for idx, (name, positions) in enumerate(self.hand_config.preset_actions.items()):
            button = QtWidgets.QPushButton(name)
            button.setProperty("category", "preset")
            button.clicked.connect(
                lambda checked, pos=positions: self.on_preset_action_clicked(pos)
            )
            row, col = divmod(idx, 2)
            self.preset_layout.addWidget(button, row, col)
            self.preset_buttons.append(button)

    def _create_per_joint_sliders(self, kind: str) -> None:
        layout = (
            self.per_joint_speed_layout
            if kind == "speed"
            else self.per_joint_torque_layout
        )
        self._clear_layout(layout)
        sliders: list[QtWidgets.QSlider] = []
        for row, name in enumerate(self.hand_config.joint_names):
            name_label = QtWidgets.QLabel(name)
            name_label.setMinimumWidth(115)
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            slider.setRange(0, 100)
            slider.setValue(100)
            slider.setMinimumWidth(120)
            value_label = QtWidgets.QLabel("100")
            value_label.setMinimumWidth(28)
            slider.valueChanged.connect(
                lambda val, setting=kind, label=value_label: self.on_per_joint_setting_changed(
                    setting, label, val
                )
            )
            layout.addWidget(name_label, row, 0)
            layout.addWidget(slider, row, 1)
            layout.addWidget(value_label, row, 2)
            sliders.append(slider)

        if kind == "speed":
            self.per_joint_speed_sliders = sliders
        else:
            self.per_joint_torque_sliders = sliders

    def _auto_detect_once(self) -> None:
        if self._auto_detect_started:
            return
        self._auto_detect_started = True
        self.auto_detect_hand(auto=True)

    def auto_detect_hand(
        self, _checked: bool = False, *, auto: bool = False
    ) -> None:
        if self._detect_thread is not None and self._detect_thread.isRunning():
            self.status_updated.emit("warning", "Autodetect is already running")
            return
        if self._backend_is_running() or self._backend_owned_by_gui:
            self.stop_backend()

        self.detect_button.setEnabled(False)
        self.detect_button.setText("Detecting...")
        self._detect_is_auto = auto
        self.status_updated.emit("info", "Autodetect probing can0-can3")

        thread = QtCore.QThread(self)
        worker = AutoDetectWorker(
            interface_type=self.interface_type_combo.currentText(),
            setup_can=True,
            bitrate=DEFAULT_CAN_BITRATE,
        )
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.finished.connect(self._on_autodetect_finished)
        worker.failed.connect(self._on_autodetect_failed)
        worker.finished.connect(thread.quit)
        worker.failed.connect(thread.quit)
        thread.finished.connect(worker.deleteLater)
        thread.finished.connect(thread.deleteLater)
        thread.finished.connect(self._clear_autodetect_worker)
        self._detect_thread = thread
        self._detect_worker = worker
        thread.start()

    def _on_autodetect_finished(
        self, hands: list[DetectedHand], messages: list[str]
    ) -> None:
        self._restore_detect_button()
        if not hands:
            detail = "\n".join(messages[-8:])
            self.status_updated.emit("warning", "No supported RealHand autodetected")
            if not getattr(self, "_detect_is_auto", False):
                QtWidgets.QMessageBox.warning(
                    self,
                    "No RealHand Autodetected",
                    "No supported RealHand was detected on can0 through can3."
                    + (f"\n\nLast probe messages:\n{detail}" if detail else ""),
                )
            return

        selected = hands[0]
        if len(hands) > 1:
            labels = [hand.label() for hand in hands]
            selected_label, accepted = QtWidgets.QInputDialog.getItem(
                self,
                "Select RealHand",
                "Multiple RealHand devices detected. Select one to connect:",
                labels,
                0,
                False,
            )
            if not accepted:
                self.status_updated.emit("warning", "Autodetect selection cancelled")
                return
            selected = hands[labels.index(selected_label)]

        self._apply_detected_hand(selected)
        self.status_updated.emit("info", f"Autodetected {selected.label()}")

    def _on_autodetect_failed(self, message: str) -> None:
        self._restore_detect_button()
        self.status_updated.emit("error", f"Autodetect failed: {message}")
        if not getattr(self, "_detect_is_auto", False):
            QtWidgets.QMessageBox.critical(self, "Autodetect Failed", message)

    def _apply_detected_hand(self, hand: DetectedHand) -> None:
        for combo, value in (
            (self.model_combo, hand.model),
            (self.side_combo, hand.side),
            (self.interface_type_combo, hand.interface_type),
        ):
            combo.blockSignals(True)
            combo.setCurrentText(value)
            combo.blockSignals(False)
        self.interface_edit.setText(hand.interface)
        self._rebuild_controls()
        self._apply_topics()
        self.latest_device_info = {
            "available": True,
            "model": hand.model,
            "side": hand.side,
            "interface_name": hand.interface,
            "interface_type": hand.interface_type,
            "serial_number": hand.serial_number,
        }
        self.update_system_info()

    def _restore_detect_button(self) -> None:
        if hasattr(self, "detect_button"):
            self.detect_button.setEnabled(True)
            self.detect_button.setText("Auto Detect")

    def _clear_autodetect_worker(self) -> None:
        self._detect_thread = None
        self._detect_worker = None

    def _apply_topics(self) -> None:
        if self.configure_topics is not None:
            self.configure_topics(self.side_combo.currentText())
        self._state_seen = False
        self.latest_device_info = {}
        self.latest_control_status = {}
        self.latest_blocking_result = {}
        self._last_device_info_status = None
        self._last_control_status_log = None
        self._update_sensor_status_label()
        self.update_system_info()
        self.status_updated.emit(
            "info", f"ROS topics set to side={self.side_combo.currentText()}"
        )

    def on_backend_button_clicked(self) -> None:
        if self._backend_is_running():
            self.stop_backend()
        else:
            self.start_backend()

    def start_backend(self) -> None:
        if self._backend_is_running():
            self.status_updated.emit("warning", "Backend is already running")
            return

        interface_name = self.interface_edit.text().strip() or "can0"
        self.interface_edit.setText(interface_name)
        interface_type = self.interface_type_combo.currentText()
        model = self.model_combo.currentText()
        side = self.side_combo.currentText()
        self._apply_topics()
        poll_on_start, stream_on_start = self._backend_sensor_start_flags()
        stopped_stale = self._stop_stale_hand_backends(side)
        if stopped_stale:
            self.status_updated.emit(
                "warning",
                f"Stopped {stopped_stale} stale hand backend process(es) for side={side}",
            )

        args = [
            "launch",
            "realhand_ros2",
            "hand.launch.py",
            f"model:={model}",
            f"side:={side}",
            f"interface_name:={interface_name}",
            f"interface_type:={interface_type}",
            f"poll_on_start:={str(poll_on_start).lower()}",
            f"stream_on_start:={str(stream_on_start).lower()}",
            "stream_queue_size:=300",
        ]

        process = QtCore.QProcess(self)
        process.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(self._on_backend_output)
        process.errorOccurred.connect(self._on_backend_error)
        process.finished.connect(self._on_backend_finished)

        self.backend_process = process
        self._backend_stop_requested = False
        self._set_backend_controls_enabled(False)
        self.backend_button.setText("Stop Backend")
        self.backend_button.setProperty("category", "danger")
        self._refresh_button_style(self.backend_button)
        process.start("ros2", args)
        if not process.waitForStarted(3000):
            self.status_updated.emit("error", "Failed to start backend process")
            self.backend_process = None
            self._backend_owned_by_gui = False
            self._set_backend_controls_enabled(True)
            self.backend_button.setText("Start Backend")
            self.backend_button.setProperty("category", "action")
            self._refresh_button_style(self.backend_button)
            return
        self._backend_owned_by_gui = True
        self.status_updated.emit(
            "info",
            "Started backend: "
            f"ros2 {' '.join(args)}",
        )

    def _backend_sensor_start_flags(self) -> tuple[bool, bool]:
        mode = self._normalized_sensor_read_mode(self.sensor_read_mode)
        if mode == "stream":
            return True, True
        if mode == "snapshot":
            return True, False
        return False, False

    def stop_backend(self) -> None:
        if not self._backend_is_running() and not self._backend_owned_by_gui:
            return
        self.stop_blocking_heatmap()
        self._backend_stop_requested = True
        owned_by_gui = self._backend_owned_by_gui
        self.status_updated.emit("warning", "Stopping backend")
        if self._backend_is_running():
            assert self.backend_process is not None
            self.backend_process.terminate()
            if not self.backend_process.waitForFinished(2000):
                self.backend_process.kill()
                self.backend_process.waitForFinished(1000)
        if owned_by_gui:
            stopped = self._stop_stale_hand_backends(self.side_combo.currentText())
            if stopped:
                self.status_updated.emit(
                    "warning",
                    f"Stopped {stopped} hand backend child process(es)",
                )
        self._backend_owned_by_gui = False

    def _backend_is_running(self) -> bool:
        return (
            self.backend_process is not None
            and self.backend_process.state() != QtCore.QProcess.NotRunning
        )

    def _on_backend_output(self) -> None:
        if self.backend_process is None:
            return
        output = bytes(self.backend_process.readAllStandardOutput()).decode(
            errors="replace"
        )
        for line in output.splitlines():
            line = line.strip()
            if line:
                self.status_updated.emit("info", f"Backend: {line}")

    def _on_backend_error(self, error: QtCore.QProcess.ProcessError) -> None:
        if error == QtCore.QProcess.FailedToStart:
            self.status_updated.emit("error", "Backend command failed to start")
        else:
            self.status_updated.emit("error", f"Backend process error: {int(error)}")

    def _on_backend_finished(
        self, exit_code: int, exit_status: QtCore.QProcess.ExitStatus
    ) -> None:
        if self._backend_stop_requested:
            self.status_updated.emit("warning", "Backend stopped")
        elif exit_status == QtCore.QProcess.CrashExit:
            self.status_updated.emit("error", "Backend crashed")
        else:
            self.status_updated.emit("warning", f"Backend exited with code {exit_code}")
        self.backend_process = None
        if self._backend_stop_requested:
            self._backend_owned_by_gui = False
        self._backend_stop_requested = False
        self._set_backend_controls_enabled(True)
        self.backend_button.setText("Start Backend")
        self.backend_button.setProperty("category", "action")
        self._refresh_button_style(self.backend_button)
        self.update_system_info()

    def _set_backend_controls_enabled(self, enabled: bool) -> None:
        self.model_combo.setEnabled(enabled)
        self.side_combo.setEnabled(enabled)
        self.interface_edit.setEnabled(enabled)
        self.interface_type_combo.setEnabled(enabled)

    def _stop_stale_hand_backends(self, side: str) -> int:
        stopped = 0
        current_pid = os.getpid()
        for pid in self._realhand_hand_node_pids(side):
            if pid == current_pid:
                continue
            if self._terminate_pid(pid):
                stopped += 1
        return stopped

    def _realhand_hand_node_pids(self, side: str) -> list[int]:
        pids: list[int] = []
        try:
            entries = os.listdir("/proc")
        except OSError:
            return pids
        for entry in entries:
            if not entry.isdigit():
                continue
            pid = int(entry)
            cmdline = self._proc_cmdline(pid)
            if not cmdline or "realhand_hand_node" not in " ".join(cmdline):
                continue
            params_file = self._params_file_from_cmdline(cmdline)
            proc_side = self._params_file_side(params_file) if params_file else None
            proc_side = proc_side or self._cmdline_side(cmdline)
            if proc_side != side:
                continue
            pids.append(pid)
        return pids

    @staticmethod
    def _proc_cmdline(pid: int) -> list[str]:
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as handle:
                raw = handle.read()
        except OSError:
            return []
        return [part.decode(errors="replace") for part in raw.split(b"\0") if part]

    @staticmethod
    def _params_file_from_cmdline(cmdline: list[str]) -> str | None:
        for index, value in enumerate(cmdline):
            if value == "--params-file" and index + 1 < len(cmdline):
                return cmdline[index + 1]
        return None

    @staticmethod
    def _params_file_side(path: str) -> str | None:
        try:
            with open(path, encoding="utf-8") as handle:
                for line in handle:
                    stripped = line.strip()
                    if stripped.startswith("side:"):
                        return stripped.split(":", 1)[1].strip().strip("'\"")
        except OSError:
            return None
        return None

    @staticmethod
    def _cmdline_side(cmdline: list[str]) -> str | None:
        for value in cmdline:
            if value.startswith("side:="):
                return value.split(":=", 1)[1].strip()
            if value.startswith("side:"):
                return value.split(":", 1)[1].strip()
        for index, value in enumerate(cmdline):
            if value == "-p" and index + 1 < len(cmdline):
                param = cmdline[index + 1]
                if param.startswith("side:="):
                    return param.split(":=", 1)[1].strip()
        return None

    @staticmethod
    def _terminate_pid(pid: int) -> bool:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            return False
        except PermissionError:
            return False
        deadline = time.time() + 2.0
        while time.time() < deadline:
            if not HandPanel._pid_exists(pid):
                return True
            time.sleep(0.05)
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            return True
        except PermissionError:
            return False
        return True

    @staticmethod
    def _pid_exists(pid: int) -> bool:
        try:
            os.kill(pid, 0)
            return True
        except ProcessLookupError:
            return False
        except PermissionError:
            return True

    @staticmethod
    def _refresh_button_style(button: QtWidgets.QPushButton) -> None:
        button.style().unpolish(button)
        button.style().polish(button)

    def on_slider_value_changed(self, index: int, value: int) -> None:
        if 0 <= index < len(self.slider_labels):
            self.slider_labels[index].setText(
                f"{self.hand_config.joint_names[index]}: {value}"
            )
        self.update_value_display()

    def update_value_display(self) -> None:
        self.value_display.setText(str([slider.value() for slider in self.sliders]))

    def on_preset_action_clicked(self, positions: list[int]) -> None:
        if len(positions) != len(self.sliders):
            QtWidgets.QMessageBox.warning(
                self,
                "Action Mismatch",
                "Preset action joint count "
                f"({len(positions)}) does not match current joint count "
                f"({len(self.sliders)})",
            )
            return
        for idx, (slider, position) in enumerate(zip(self.sliders, positions)):
            slider.setValue(int(position))
            self.on_slider_value_changed(idx, int(position))
        self.publish_joint_state(force=True)

    def on_home_clicked(self) -> None:
        for slider, position in zip(self.sliders, self.hand_config.init_pos):
            slider.setValue(int(position))
        self.publish_joint_state(force=True)
        self.status_updated.emit("info", "Return to Home")

    def on_stop_clicked(self) -> None:
        if self.cycle_timer and self.cycle_timer.isActive():
            self.cycle_timer.stop()
            self.cycle_timer = None
            self.cycle_button.setText("Cycle Preset Actions")
            self.reset_preset_buttons_color()
        if self.auto_grab_running:
            self.stop_auto_grab()
        for timer in (
            self.live_speed_timer,
            self.live_torque_timer,
            self.live_joint_speed_timer,
            self.live_joint_torque_timer,
        ):
            if timer.isActive():
                timer.stop()
        self.status_updated.emit("warning", "All actions stopped")

    def on_cycle_clicked(self) -> None:
        if not self.hand_config.preset_actions:
            QtWidgets.QMessageBox.warning(
                self,
                "No Preset Actions",
                "Current hand model has no preset actions to cycle",
            )
            return
        if self.cycle_timer and self.cycle_timer.isActive():
            self.cycle_timer.stop()
            self.cycle_timer = None
            self.cycle_button.setText("Cycle Preset Actions")
            self.reset_preset_buttons_color()
            self.status_updated.emit("info", "Stopped cycling preset actions")
            return
        self.current_action_index = -1
        self.cycle_loop_active = False
        self.cycle_loop_index = -1
        self.cycle_loop_iterations = 0
        self.cycle_timer = QtCore.QTimer(self)
        self.cycle_timer.timeout.connect(self.run_next_action)
        self.cycle_timer.start(LOOP_TIME_MS)
        self.cycle_button.setText("Stop Cycling")
        self.status_updated.emit("info", "Started cycling preset actions")
        self.run_next_action()

    def run_next_action(self) -> None:
        if not self.hand_config.preset_actions:
            return
        self.reset_preset_buttons_color()
        name = self._next_cycle_preset_name()
        self.on_preset_action_clicked(self.hand_config.preset_actions[name])
        preset_index = list(self.hand_config.preset_actions).index(name)
        if 0 <= preset_index < len(self.preset_buttons):
            self.preset_buttons[preset_index].setStyleSheet(
                "background-color: green; color: white; border-color: #91D5FF;"
            )
        self.status_updated.emit("info", f"Running preset action: {name}")

    def _next_cycle_preset_name(self) -> str:
        loop_names = self._cycle_loop_names()
        if self.cycle_loop_active and loop_names:
            self.cycle_loop_index = (self.cycle_loop_index + 1) % len(loop_names)
            name = loop_names[self.cycle_loop_index]
            if self.cycle_loop_index == len(loop_names) - 1:
                self.cycle_loop_iterations += 1
                if self._cycle_loop_limit_reached():
                    self.cycle_loop_active = False
                    self.cycle_loop_index = -1
                    self.cycle_loop_iterations = 0
            return name

        names = list(self.hand_config.preset_actions)
        self.current_action_index = (self.current_action_index + 1) % len(names)
        name = names[self.current_action_index]
        if loop_names and name == loop_names[-1]:
            self.cycle_loop_active = True
            self.cycle_loop_index = len(loop_names) - 1
            self.cycle_loop_iterations = 0
        return name

    def _cycle_loop_names(self) -> list[str]:
        names = [
            name
            for name in self.hand_config.cycle_loop_actions
            if name in self.hand_config.preset_actions
        ]
        return names if len(names) >= 2 else []

    def _cycle_loop_limit_reached(self) -> bool:
        repeats = max(0, int(self.hand_config.cycle_loop_repeats))
        return repeats > 0 and self.cycle_loop_iterations >= repeats

    def reset_preset_buttons_color(self) -> None:
        for button in self.preset_buttons:
            button.setStyleSheet("")
            button.setProperty("category", "preset")
            button.style().unpolish(button)
            button.style().polish(button)

    def publish_joint_state(self, *, force: bool = False) -> None:
        positions = [slider.value() for slider in self.sliders]
        if not force and positions == self._last_published_positions:
            return
        self._publish(self._joint_command_values(positions), None, None)
        self._last_published_positions = list(positions)

    def _joint_command_values(self, positions: list[int]) -> list[float]:
        values = []
        for idx, position in enumerate(positions):
            if (
                idx < len(self.hand_config.joint_names)
                and self._is_non_thumb_abduction(self.hand_config.joint_names[idx])
            ):
                position = MAX_JOINT_POSITION - position
            values.append(float(position))
        return values

    @staticmethod
    def _is_non_thumb_abduction(joint_name: str) -> bool:
        lowered = joint_name.lower()
        return "abduction" in lowered and "thumb" not in lowered

    def on_sensor_read_mode_changed(self, mode: str) -> None:
        mode = self._normalized_sensor_read_mode(mode)
        self.sensor_read_mode = mode
        if hasattr(self, "sensor_status_label"):
            self.sensor_status_label.setText(f"Mode: {mode}")
        self.update_system_info()

        if mode == "stream":
            self.stop_blocking_heatmap()
            self.start_polling(log_success=False)
            self.start_stream(log_success=False)
            self.status_updated.emit("info", "Sensor read mode set to stream")
        elif mode == "snapshot":
            self.stop_blocking_heatmap()
            self.start_polling(log_success=False)
            self.stop_stream(log_success=False)
            self.request_snapshot(log_success=False)
            self.status_updated.emit("info", "Sensor read mode set to snapshot")
        elif mode == "get_blocking":
            self.stop_stream(log_success=False)
            self.stop_polling(log_success=False)
            if hasattr(self, "sensor_request_combo") and self.sensor_request_combo.currentText() not in (
                "force_sensor",
                "touch",
            ):
                self.sensor_request_combo.setCurrentText("force_sensor")
            self.start_blocking_heatmap()
            self.status_updated.emit("info", "Sensor read mode set to get_blocking")

    @staticmethod
    def _normalized_sensor_read_mode(mode: str) -> str:
        normalized = str(mode).lower()
        normalized = SENSOR_READ_MODE_ALIASES.get(normalized, normalized)
        if normalized not in SENSOR_READ_MODES:
            return DEFAULT_SENSOR_READ_MODE
        return normalized

    def _start_live_timer(self, timer: QtCore.QTimer) -> None:
        if not timer.isActive():
            timer.start()

    def on_global_speed_changed(self, value: int) -> None:
        self.speed_val_lbl.setText(str(value))
        if self.live_joint_speed_timer.isActive():
            self.live_joint_speed_timer.stop()
        self._start_live_timer(self.live_speed_timer)

    def _publish_live_speed(self) -> None:
        value = self.speed_slider.value()
        self._publish(None, [float(value) for _ in self.joint_names], None)

    def on_global_torque_changed(self, value: int) -> None:
        self.torque_val_lbl.setText(str(value))
        if self.live_joint_torque_timer.isActive():
            self.live_joint_torque_timer.stop()
        self.current_torque_values = [value] * len(self.hand_config.joint_names)
        self._sync_per_joint_torque_sliders()
        self._update_commanded_torque_display()
        self._set_last_torque_boost_text("Last boost: reset", "black")
        self._start_live_timer(self.live_torque_timer)

    def _publish_live_torque(self) -> None:
        value = self.torque_slider.value()
        self.current_torque_values = [value] * len(self.hand_config.joint_names)
        self._sync_per_joint_torque_sliders()
        self._update_commanded_torque_display()
        self._publish(None, None, [float(value) for _ in self.joint_names])

    def on_per_joint_setting_changed(
        self, kind: str, value_label: QtWidgets.QLabel, value: int
    ) -> None:
        value_label.setText(str(value))
        if self._syncing_per_joint_settings:
            return
        if kind == "speed":
            if self.live_speed_timer.isActive():
                self.live_speed_timer.stop()
            self._start_live_timer(self.live_joint_speed_timer)
            return
        if self.live_torque_timer.isActive():
            self.live_torque_timer.stop()
        self.current_torque_values = [
            slider.value() for slider in self.per_joint_torque_sliders
        ]
        self._update_commanded_torque_display()
        self._set_last_torque_boost_text("Last boost: reset", "black")
        self._start_live_timer(self.live_joint_torque_timer)

    def _publish_live_joint_speeds(self) -> None:
        if not self.per_joint_speed_sliders:
            return
        values = [float(slider.value()) for slider in self.per_joint_speed_sliders]
        self._publish(None, values, None)

    def _publish_live_joint_torques(self) -> None:
        if not self.per_joint_torque_sliders:
            return
        values = [slider.value() for slider in self.per_joint_torque_sliders]
        self.current_torque_values = list(values)
        self._update_commanded_torque_display()
        self._publish(None, None, [float(value) for value in values])

    def _sync_per_joint_torque_sliders(self) -> None:
        if len(self.per_joint_torque_sliders) != len(self.current_torque_values):
            return
        self._syncing_per_joint_settings = True
        try:
            for slider, value in zip(
                self.per_joint_torque_sliders, self.current_torque_values
            ):
                slider.setValue(int(value))
        finally:
            self._syncing_per_joint_settings = False

    def _update_commanded_torque_display(
        self,
        *,
        boosted_finger: str | None = None,
        boosted_values: list[int] | None = None,
        maxed_out: bool = False,
    ) -> None:
        if not hasattr(self, "torque_val_lbl"):
            return

        values = self._current_joint_torque_values()
        if values:
            unique_values = sorted(set(values))
            self._sync_global_torque_slider_position(
                values, uniform=len(unique_values) == 1
            )
            if len(unique_values) == 1:
                self.torque_val_lbl.setText(str(unique_values[0]))
                self.torque_val_lbl.setToolTip("Uniform commanded torque")
            else:
                self.torque_val_lbl.setText("mixed")
                self.torque_val_lbl.setToolTip(
                    "Per-joint commanded torques differ; see Touch Control"
                )

        if hasattr(self, "commanded_torque_summary_label") and values:
            self.commanded_torque_summary_label.setText(
                f"Command: min={min(values)} max={max(values)}"
            )

        if hasattr(self, "commanded_torque_finger_labels"):
            finger_map = self._finger_joint_map()
            for finger, label in self.commanded_torque_finger_labels.items():
                indices = [
                    idx for idx in finger_map.get(finger, []) if idx < len(values)
                ]
                if not indices:
                    self._set_label_status(label, f"{finger[0].upper()}:--", "gray")
                    continue
                finger_values = [values[idx] for idx in indices]
                color = (
                    "#B26A00"
                    if boosted_finger == finger and not maxed_out
                    else "black"
                )
                text = ",".join(str(value) for value in finger_values)
                self._set_label_status(label, f"{finger[0].upper()}:{text}", color)

        if boosted_finger and hasattr(self, "last_torque_boost_label"):
            if maxed_out:
                self._set_last_torque_boost_text(
                    f"Last boost: {boosted_finger} already at max",
                    "#B26A00",
                )
            else:
                values_text = boosted_values if boosted_values is not None else []
                self._set_last_torque_boost_text(
                    f"Last boost: {boosted_finger} -> {values_text}",
                    "green",
                )

    def _set_last_torque_boost_text(self, text: str, color: str) -> None:
        if not hasattr(self, "last_torque_boost_label"):
            return
        self.last_torque_boost_label.setText(text)
        self.last_torque_boost_label.setStyleSheet(f"color: {color};")

    def _sync_global_torque_slider_position(
        self, values: list[int], *, uniform: bool
    ) -> None:
        if not hasattr(self, "torque_slider") or not values:
            return
        display_value = values[0] if uniform else max(values)
        was_blocked = self.torque_slider.blockSignals(True)
        try:
            self.torque_slider.setValue(int(display_value))
        finally:
            self.torque_slider.blockSignals(was_blocked)

    def _publish(
        self,
        positions: list[float] | None,
        velocities: list[float] | None,
        efforts: list[float] | None,
    ) -> None:
        if self.publish_command is None:
            return
        self.publish_command(self.joint_names, positions, velocities, efforts)

    def _publish_json_payload(
        self, payload: dict[str, Any], *, success_message: str | None = None
    ) -> bool:
        if self.publish_json is None:
            self.status_updated.emit("warning", "ROS JSON topic is not configured")
            return False
        self.publish_json(json.dumps(payload))
        if success_message:
            self.status_updated.emit("info", success_message)
        return True

    def start_polling(self, _checked: bool = False, *, log_success: bool = True) -> None:
        self._publish_json_payload(
            {"action": "start_polling"},
            success_message="Start polling command sent" if log_success else None,
        )

    def stop_polling(self, _checked: bool = False, *, log_success: bool = True) -> None:
        self._publish_json_payload(
            {"action": "stop_polling"},
            success_message="Stop polling command sent" if log_success else None,
        )

    def start_stream(self, _checked: bool = False, *, log_success: bool = True) -> None:
        self._publish_json_payload(
            {"action": "start_stream", "maxsize": 300},
            success_message="Start stream command sent" if log_success else None,
        )

    def stop_stream(self, _checked: bool = False, *, log_success: bool = True) -> None:
        self._publish_json_payload(
            {"action": "stop_stream"},
            success_message="Stop stream command sent" if log_success else None,
        )

    def request_snapshot(self, _checked: bool = False, *, log_success: bool = True) -> None:
        sensor = self._selected_request_sensor()
        request_id = self._next_request_id("snapshot")
        self._publish_json_payload(
            {
                "action": "get_snapshot",
                "sensor": sensor,
                "request_id": request_id,
            },
            success_message=(
                f"Snapshot request sent: {sensor}" if log_success else None
            ),
        )

    def request_blocking(self, _checked: bool = False, *, log_success: bool = True) -> None:
        sensor = self._selected_request_sensor()
        request_id = self._next_request_id("blocking")
        self._publish_json_payload(
            {
                "action": "get_blocking",
                "sensor": sensor,
                "timeout_ms": int(self.blocking_timeout_spin.value()),
                "pause_polling": True,
                "request_id": request_id,
            },
            success_message=(
                f"Blocking request sent: {sensor}" if log_success else None
            ),
        )

    def start_blocking_heatmap(self) -> None:
        if not self.blocking_heatmap_timer.isActive():
            self.blocking_heatmap_timer.start()
        self.request_blocking_heatmap()

    def stop_blocking_heatmap(self) -> None:
        if self.blocking_heatmap_timer.isActive():
            self.blocking_heatmap_timer.stop()
        self._blocking_heatmap_request_id = None

    def request_blocking_heatmap(self) -> None:
        if self._normalized_sensor_read_mode(self.sensor_read_mode) != "get_blocking":
            self.stop_blocking_heatmap()
            return
        if self._blocking_heatmap_request_id is not None:
            return
        if self.publish_json is None:
            return

        request_id = self._next_request_id("heatmap-blocking")
        self._blocking_heatmap_request_id = request_id
        timeout_ms = max(
            BLOCKING_HEATMAP_TIMEOUT_MS,
            int(self.blocking_timeout_spin.value()),
        )
        if not self._publish_json_payload(
            {
                "action": "get_blocking",
                "sensor": "force_sensor",
                "timeout_ms": timeout_ms,
                "pause_polling": True,
                "request_id": request_id,
            }
        ):
            self._blocking_heatmap_request_id = None

    def run_fault_report(self, _checked: bool = False) -> None:
        output_dir = self.fault_report_dir_edit.text().strip()
        if not output_dir:
            output_dir = DEFAULT_FAULT_REPORT_DIR
            self.fault_report_dir_edit.setText(output_dir)
        request_id = self._next_request_id("fault-report")
        filename = f"{request_id}-{self.model_combo.currentText()}-{self.side_combo.currentText()}.csv"
        self._pending_fault_report_id = request_id
        self._pending_fault_report_path = os.path.join(output_dir, filename)
        self.fault_report_status_label.setText("Running fault check...")
        self.fault_report_preview.setPlainText("")
        self._publish_json_payload(
            {
                "action": "get_blocking",
                "sensor": "fault",
                "timeout_ms": int(self.fault_timeout_spin.value()),
                "pause_polling": True,
                "request_id": request_id,
            },
            success_message="Fault check request sent",
        )

    def _selected_request_sensor(self) -> str:
        if not hasattr(self, "sensor_request_combo"):
            return "fault"
        return self.sensor_request_combo.currentText()

    def _next_request_id(self, prefix: str) -> str:
        self._request_counter += 1
        return f"{prefix}-{self._request_counter}"

    def clear_faults(self) -> None:
        self._publish_json_payload(
            {"action": "clear_faults"},
            success_message="Clear Faults command sent",
        )

    def update_state(self, msg: JointState) -> None:
        payload: dict[str, list[Any]] = {}
        if msg.position:
            payload["angle"] = list(msg.position)
        if msg.velocity:
            payload["speed"] = list(msg.velocity)
        if msg.effort:
            payload["torque"] = list(msg.effort)
        if payload:
            self.update_sensor_values(payload)
        if not self._state_seen:
            self._state_seen = True
            self.update_status("info", "ROS2 hand state received")
        elif hasattr(self, "connection_status"):
            self.connection_status.setText("ROS2 Hand Connected")
            self.connection_status.setObjectName("StatusInfo")
            self.connection_status.style().unpolish(self.connection_status)
            self.connection_status.style().polish(self.connection_status)

    def update_snapshot(self, msg: String) -> None:
        try:
            parsed = json.loads(msg.data)
        except json.JSONDecodeError:
            self.status_updated.emit("error", "Received invalid snapshot JSON")
            return
        sensor_payload = self._extract_sensor_payload(parsed)
        if sensor_payload:
            self.update_sensor_values(sensor_payload)
        matrix_payload = self._extract_matrix_payload(parsed)
        if matrix_payload:
            self.update_matrix_display(matrix_payload)
            keys = tuple(sorted(matrix_payload))
            if keys != self._last_touch_keys_logged:
                self._last_touch_keys_logged = keys
                self.status_updated.emit("info", f"Touch data received: {', '.join(keys)}")
        else:
            self.status_updated.emit("warning", "Touch message had no matrix data")

    def update_device_info(self, msg: String) -> None:
        try:
            parsed = json.loads(msg.data)
        except json.JSONDecodeError:
            self.status_updated.emit("error", "Received invalid device info JSON")
            return
        if not isinstance(parsed, dict):
            self.status_updated.emit("error", "Received non-object device info JSON")
            return

        self.latest_device_info = parsed
        self.update_system_info()

        status = (
            parsed.get("available"),
            parsed.get("serial_number"),
            parsed.get("firmware_version"),
            parsed.get("mechanical_version"),
            parsed.get("pcb_version"),
            parsed.get("error"),
        )
        if status == self._last_device_info_status:
            return
        self._last_device_info_status = status

        if parsed.get("available") is False:
            error = parsed.get("error") or "unavailable"
            self.status_updated.emit("warning", f"Device info unavailable: {error}")
            return
        serial = parsed.get("serial_number") or "unknown serial"
        firmware = parsed.get("firmware_version") or "unknown firmware"
        self.status_updated.emit(
            "info", f"Device info received: {serial}, firmware {firmware}"
        )

    def update_temperature(self, msg: String) -> None:
        self._update_sensor_topic(msg, "temperature")

    def update_current(self, msg: String) -> None:
        self._update_sensor_topic(msg, "current")

    def update_touch(self, msg: String) -> None:
        try:
            parsed = json.loads(msg.data)
        except json.JSONDecodeError:
            self.status_updated.emit("error", "Received invalid touch JSON")
            return
        matrix_payload = self._extract_matrix_payload(parsed)
        if matrix_payload:
            self.update_matrix_display(matrix_payload)

    def update_control_status(self, msg: String) -> None:
        try:
            parsed = json.loads(msg.data)
        except json.JSONDecodeError:
            self.status_updated.emit("error", "Received invalid control status JSON")
            return
        if not isinstance(parsed, dict):
            return

        self.latest_control_status = parsed
        self._update_sensor_status_label()
        self.update_system_info()

        log_key = (
            parsed.get("action"),
            parsed.get("ok"),
            parsed.get("streaming"),
            self._polling_active(),
            parsed.get("request_id"),
            parsed.get("error"),
        )
        if log_key == self._last_control_status_log:
            return
        self._last_control_status_log = log_key

        action = parsed.get("action") or "control"
        if parsed.get("ok") is False:
            error = parsed.get("error") or "failed"
            self.status_updated.emit("error", f"{action} failed: {error}")
        else:
            self.status_updated.emit("info", f"{action} accepted")

    def update_blocking_result(self, msg: String) -> None:
        try:
            parsed = json.loads(msg.data)
        except json.JSONDecodeError:
            self.status_updated.emit("error", "Received invalid result JSON")
            return
        if not isinstance(parsed, dict):
            return

        request_id = str(parsed.get("request_id") or "")
        is_heatmap_result = request_id == self._blocking_heatmap_request_id or (
            request_id.startswith("heatmap-blocking-")
        )
        if is_heatmap_result:
            self._blocking_heatmap_request_id = None

        self.latest_blocking_result = parsed
        self._maybe_finish_fault_report(parsed)
        sensor = str(parsed.get("sensor") or "result")
        action = str(parsed.get("action") or "result")
        if parsed.get("ok") is False:
            error = parsed.get("error") or "failed"
            if not is_heatmap_result:
                self.status_updated.emit("error", f"{action} {sensor} failed: {error}")
            self._update_sensor_status_label()
            self.update_system_info()
            return

        data = parsed.get("data")
        self._consume_sensor_result(sensor, data)
        self._update_sensor_status_label()
        self.update_system_info()
        if not is_heatmap_result:
            self.status_updated.emit("info", f"{action} {sensor} result received")

    def _consume_sensor_result(self, sensor: str, data: Any) -> None:
        sensor_payload = self._extract_sensor_payload({sensor: data})
        if not sensor_payload:
            sensor_payload = self._extract_sensor_payload(data)
        if sensor_payload:
            self.update_sensor_values(sensor_payload)

        matrix_payload = self._extract_matrix_payload(data)
        if matrix_payload:
            self.update_matrix_display(matrix_payload)

    def _maybe_finish_fault_report(self, result: dict[str, Any]) -> None:
        if result.get("request_id") != self._pending_fault_report_id:
            return
        path = self._pending_fault_report_path
        self._pending_fault_report_id = None
        self._pending_fault_report_path = None
        if not path:
            return

        if result.get("ok") is False:
            error = result.get("error") or "fault check failed"
            self.fault_report_status_label.setText(str(error))
            self.fault_report_preview.setPlainText(str(error))
            return

        try:
            rows = self._fault_report_rows(result)
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, "w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(
                    handle,
                    fieldnames=(
                        "request_id",
                        "timestamp",
                        "model",
                        "side",
                        "sensor",
                        "joint",
                        "fault",
                    ),
                )
                writer.writeheader()
                writer.writerows(rows)
        except OSError as exc:
            self.fault_report_status_label.setText(f"Save failed: {exc}")
            self.fault_report_preview.setPlainText(str(exc))
            return

        self.fault_report_status_label.setText(f"Saved: {path}")
        self.fault_report_preview.setPlainText(self._fault_report_preview(rows))

    def _fault_report_rows(self, result: dict[str, Any]) -> list[dict[str, Any]]:
        timestamp = result.get("timestamp", time.time())
        base = {
            "request_id": result.get("request_id", ""),
            "timestamp": timestamp,
            "model": self.model_combo.currentText(),
            "side": self.side_combo.currentText(),
            "sensor": result.get("sensor", "fault"),
        }
        faults = self._extract_fault_map(result.get("data"))
        if not faults:
            return [{**base, "joint": "raw", "fault": result.get("data", "")}]
        return [
            {**base, "joint": str(joint), "fault": str(fault)}
            for joint, fault in faults.items()
        ]

    def _extract_fault_map(self, payload: Any) -> dict[str, Any]:
        if not isinstance(payload, dict):
            return {}
        faults = payload.get("faults")
        if isinstance(faults, dict):
            return {str(key): value for key, value in faults.items()}
        simple = {
            str(key): value
            for key, value in payload.items()
            if key != "timestamp" and not isinstance(value, (dict, list))
        }
        if simple and any(key in self.joint_names for key in simple):
            return simple
        for value in payload.values():
            nested = self._extract_fault_map(value)
            if nested:
                return nested
        return {}

    def _fault_report_preview(self, rows: list[dict[str, Any]]) -> str:
        if not rows:
            return "No fault data"
        lines = [f"{row['joint']}: {row['fault']}" for row in rows]
        return "\n".join(lines)

    def _update_sensor_status_label(self) -> None:
        if not hasattr(self, "sensor_status_label"):
            return
        stream = self.latest_control_status.get("streaming")
        polling = self._polling_active()
        result = self.latest_blocking_result
        last = result.get("sensor") if isinstance(result, dict) else None
        mode = self._normalized_sensor_read_mode(self.sensor_read_mode)
        status = f"Mode: {mode} | polling: {self._bool_text(polling)}"
        status += f" | stream: {self._bool_text(stream)}"
        if last:
            ok = "ok" if result.get("ok") is not False else "error"
            status += f" | last: {last} {ok}"
        self.sensor_status_label.setText(status)

    def _polling_active(self) -> bool | None:
        polling = self.latest_control_status.get("polling")
        if isinstance(polling, dict) and "active" in polling:
            return bool(polling.get("active"))
        if "polling_active" in self.latest_control_status:
            return bool(self.latest_control_status.get("polling_active"))
        return None

    @staticmethod
    def _bool_text(value: Any) -> str:
        if value is None:
            return "unknown"
        return "on" if bool(value) else "off"

    def _update_sensor_topic(self, msg: String, key: str) -> None:
        try:
            parsed = json.loads(msg.data)
        except json.JSONDecodeError:
            self.status_updated.emit("error", f"Received invalid {key} JSON")
            return
        sensor_payload = self._extract_sensor_payload({key: parsed})
        if sensor_payload:
            self.update_sensor_values(sensor_payload)

    def _extract_sensor_payload(self, payload: Any) -> dict[str, list[Any]]:
        if not isinstance(payload, dict):
            return {}
        extracted: dict[str, list[Any]] = {}
        specs = {
            "angle": ("angle", "angles", "position", "positions"),
            "speed": ("speed", "speeds", "velocity", "velocities"),
            "torque": ("torque", "torques", "effort", "efforts"),
            "temperature": ("temperature", "temperatures", "temp", "temps"),
            "current": ("current", "currents"),
        }
        for canonical, names in specs.items():
            value = self._find_first_vector(payload, names)
            if value:
                extracted[canonical] = value
        return extracted

    def _find_first_vector(self, payload: Any, names: tuple[str, ...]) -> list[Any]:
        if isinstance(payload, dict):
            for name in names:
                if name in payload:
                    direct = _as_list(payload[name])
                    if direct:
                        return direct
                    if isinstance(payload[name], dict):
                        nested = self._find_first_vector(payload[name], names)
                        if nested:
                            return nested
            for value in payload.values():
                nested = self._find_first_vector(value, names)
                if nested:
                    return nested
        return []

    def _extract_matrix_payload(self, payload: Any) -> dict[str, Any]:
        matrices: dict[str, Any] = {}
        self._collect_matrices(payload, matrices)
        return matrices

    def _collect_matrices(self, payload: Any, matrices: dict[str, Any]) -> None:
        if isinstance(payload, dict):
            for key, value in payload.items():
                normalized = self._matrix_key(key)
                if normalized is not None:
                    matrices[normalized] = value
                else:
                    self._collect_matrices(value, matrices)
        elif isinstance(payload, list):
            for value in payload:
                self._collect_matrices(value, matrices)

    def _matrix_key(self, key: str) -> str | None:
        lowered = key.lower()
        aliases = {
            "thumb": "thumb_matrix",
            "thumb_matrix": "thumb_matrix",
            "index": "index_matrix",
            "index_matrix": "index_matrix",
            "middle": "middle_matrix",
            "middle_matrix": "middle_matrix",
            "ring": "ring_matrix",
            "ring_matrix": "ring_matrix",
            "pinky": "pinky_matrix",
            "little": "pinky_matrix",
            "pinky_matrix": "pinky_matrix",
            "little_matrix": "pinky_matrix",
        }
        return aliases.get(lowered)

    def update_matrix_display(self, matrix_data: dict[str, Any]) -> None:
        for key, data in matrix_data.items():
            self.matrix_display.update_matrix_data(key, data)
            finger = self._finger_from_matrix_key(key)
            if finger is not None:
                self.latest_touch_matrices[finger] = data
                self._update_slip_detection(finger, data)

    def start_auto_grab(self) -> None:
        if self.auto_grab_running:
            return
        self._set_auto_grab_start_torque()
        if not self.latest_touch_matrices:
            self.auto_grab_status_label.setText("No touch data")
            self.status_updated.emit("warning", "Auto Grab requires touch matrix data")
            return
        self.auto_grab_running = True
        self.auto_grab_sensor_fail_count = 0
        self.auto_grab_fingers_stopped = {
            finger: False for finger in self.finger_order
        }
        self.auto_grab_above_count = {finger: 0 for finger in self.finger_order}
        self.auto_grab_started_at = time.monotonic()
        self.auto_grab_baseline = self._touch_max_by_finger()
        self.auto_grab_start_button.setEnabled(False)
        self.auto_grab_stop_button.setEnabled(True)
        self.auto_grab_status_label.setText("Grabbing...")
        for finger, label in self.auto_grab_finger_labels.items():
            self._set_label_status(label, f"{finger[0].upper()}:--", "black")
        self.auto_grab_timer.start()
        self.status_updated.emit(
            "info", f"Auto Grab started; baseline={self.auto_grab_baseline}"
        )

    def _set_auto_grab_start_torque(self) -> None:
        joint_count = len(self.hand_config.joint_names)
        if joint_count <= 0:
            return
        values = [AUTO_GRAB_START_TORQUE] * joint_count
        self.current_torque_values = list(values)
        self._sync_per_joint_torque_sliders()
        self._update_commanded_torque_display()
        self._set_last_torque_boost_text("Last boost: auto-grab baseline", "black")
        self._publish(None, None, [float(value) for value in values])

    def stop_auto_grab(self) -> None:
        if self.auto_grab_timer.isActive():
            self.auto_grab_timer.stop()
        self.auto_grab_running = False
        self.auto_grab_start_button.setEnabled(True)
        self.auto_grab_stop_button.setEnabled(False)
        self.auto_grab_status_label.setText("Stopped")
        self.status_updated.emit("warning", "Auto Grab stopped")

    def _auto_grab_step(self) -> None:
        if not self.auto_grab_running:
            self.auto_grab_timer.stop()
            return

        if time.monotonic() - self.auto_grab_started_at > AUTO_GRAB_MAX_DURATION_S:
            self._finish_auto_grab("Timeout")
            return

        touch_data = self._touch_max_by_finger()
        if not touch_data:
            self.auto_grab_sensor_fail_count += 1
            if self.auto_grab_sensor_fail_count >= 3:
                self._finish_auto_grab("Touch read error")
            return
        self.auto_grab_sensor_fail_count = 0

        current_pos = [slider.value() for slider in self.sliders]
        target_pos = self._auto_grab_target_positions()
        if len(target_pos) != len(current_pos):
            self._finish_auto_grab("Target mismatch")
            return

        threshold = self.auto_grab_threshold_spin.value()
        step_size = max(1, self.auto_grab_speed_spin.value() // 10)
        finger_joint_map = self._finger_joint_map()
        all_stopped = True
        changed = False

        for finger, joint_indices in finger_joint_map.items():
            if self.auto_grab_fingers_stopped.get(finger, False):
                continue

            raw_pressure = int(touch_data.get(finger, 0))
            baseline = int(self.auto_grab_baseline.get(finger, 0))
            pressure = max(0, raw_pressure - baseline)
            self._update_auto_grab_finger_label(finger, pressure, threshold)

            if pressure >= threshold:
                self.auto_grab_above_count[finger] = (
                    self.auto_grab_above_count.get(finger, 0) + 1
                )
                if (
                    self.auto_grab_above_count[finger]
                    >= self.auto_grab_debounce_limit
                ):
                    self.auto_grab_fingers_stopped[finger] = True
                    self._update_auto_grab_finger_stopped(
                        finger, pressure, raw_pressure, baseline
                    )
                continue

            self.auto_grab_above_count[finger] = 0
            all_stopped = False
            for idx in joint_indices:
                if idx >= len(current_pos):
                    continue
                current = current_pos[idx]
                target = target_pos[idx]
                if current == target:
                    continue
                if current > target:
                    current_pos[idx] = max(target, current - step_size)
                else:
                    current_pos[idx] = min(target, current + step_size)
                changed = True

        reached_target = all(
            self.auto_grab_fingers_stopped.get(finger, False)
            or all(
                idx >= len(current_pos) or current_pos[idx] == target_pos[idx]
                for idx in finger_joint_map.get(finger, [])
            )
            for finger in finger_joint_map
        )

        if changed:
            self._set_slider_positions(current_pos)
            self.publish_joint_state(force=True)
        if reached_target or all_stopped or all(self.auto_grab_fingers_stopped.values()):
            stopped_count = sum(
                1 for stopped in self.auto_grab_fingers_stopped.values() if stopped
            )
            self._finish_auto_grab(f"Done ({stopped_count}/5 stopped)")

    def _finish_auto_grab(self, status: str) -> None:
        if self.auto_grab_timer.isActive():
            self.auto_grab_timer.stop()
        self.auto_grab_running = False
        self.auto_grab_start_button.setEnabled(True)
        self.auto_grab_stop_button.setEnabled(False)
        self.auto_grab_status_label.setText(status)
        self.status_updated.emit("info", f"Auto Grab {status}")

    def _auto_grab_target_positions(self) -> list[int]:
        for name, positions in self.hand_config.preset_actions.items():
            if name.lower() == "fist":
                return list(positions)
        return [0] * len(self.sliders)

    def _set_slider_positions(self, positions: list[int]) -> None:
        for slider, value in zip(self.sliders, positions):
            slider.setValue(int(value))

    def _touch_max_by_finger(self) -> dict[str, int]:
        return {
            finger: self._get_max_pressure(self.latest_touch_matrices.get(finger))
            for finger in self.finger_order
        }

    def _finger_from_matrix_key(self, key: str) -> str | None:
        if key.endswith("_matrix"):
            return self._finger_key_from_name(key[:-7])
        return self._finger_key_from_name(key)

    def _finger_joint_map(self) -> dict[str, list[int]]:
        mapping = {finger: [] for finger in self.finger_order}
        for idx, name in enumerate(self.hand_config.joint_names):
            finger = self._finger_key_from_name(name)
            if finger is not None:
                mapping[finger].append(idx)
        return mapping

    def _update_auto_grab_finger_label(
        self, finger: str, pressure: int, threshold: int
    ) -> None:
        label = self.auto_grab_finger_labels.get(finger)
        if label is None:
            return
        if pressure >= threshold:
            color = "red"
        elif pressure >= threshold * 0.7:
            color = "#B26A00"
        else:
            color = "black"
        self._set_label_status(label, f"{finger[0].upper()}:{pressure:3d}", color)

    def _update_auto_grab_finger_stopped(
        self,
        finger: str,
        pressure: int,
        raw_pressure: int,
        baseline: int,
    ) -> None:
        label = self.auto_grab_finger_labels.get(finger)
        if label is not None:
            self._set_label_status(label, f"{finger[0].upper()}:STP", "green")
        self.status_updated.emit(
            "info",
            f"{finger.capitalize()} stopped at pressure {pressure} "
            f"(raw={raw_pressure}, baseline={baseline})",
        )

    def _update_slip_detection(self, finger: str, touch_data: Any) -> None:
        if not hasattr(self, "slip_labels"):
            return
        label = self.slip_labels.get(finger)
        positions = [slider.value() for slider in self.sliders]
        if not self._finger_is_active(finger, positions):
            self.prev_touch.pop(finger, None)
            self.touch_history.pop(finger, None)
            if label is not None:
                self._set_label_status(label, f"{finger[0].upper()}:OFF", "gray")
            return

        flat = self._flatten_touch(touch_data)
        if not flat:
            self.touch_history.pop(finger, None)
            if label is not None:
                self._set_label_status(label, f"{finger[0].upper()}:--", "black")
            return

        contact_threshold = self.slip_contact_spin.value()
        max_pressure = max(flat)
        if max_pressure < contact_threshold:
            self.touch_history.pop(finger, None)
            if label is not None:
                self._set_label_status(label, f"{finger[0].upper()}:NO", "black")
            return

        if self.slip_window_checkbox.isChecked():
            frames = max(2, self.slip_window_frames_spin.value())
            history = self.touch_history.setdefault(finger, [])
            history.append(flat)
            if len(history) > frames:
                history.pop(0)
            if len(history) < frames:
                if label is not None:
                    self._set_label_status(label, f"{finger[0].upper()}:OK", "green")
                return
            previous = history[0]
            current = history[-1]
            mode = f"{frames}f"
        else:
            previous = self.prev_touch.get(finger)
            self.prev_touch[finger] = flat
            if not previous or len(previous) != len(flat):
                if label is not None:
                    self._set_label_status(label, f"{finger[0].upper()}:OK", "green")
                return
            current = flat
            mode = "1f"

        mag_delta = abs(sum(current) - sum(previous))
        loc_delta = self._touch_location_delta(previous, current)
        now_ms = int(time.time() * 1000)
        last_ms = self.last_slip_time.get(finger, 0)
        if (
            mag_delta >= self.slip_mag_spin.value()
            and loc_delta >= self.slip_loc_spin.value()
            and now_ms - last_ms >= self.slip_cooldown_spin.value()
        ):
            self.last_slip_time[finger] = now_ms
            if label is not None:
                self._set_label_status(label, f"{finger[0].upper()}:SLP", "red")
            self.status_updated.emit(
                "warning",
                f"Slip detected: {finger} "
                f"(mode={mode}, mag={mag_delta}, loc={loc_delta:.2f})",
            )
            if self.slip_torque_boost_checkbox.isChecked():
                self._boost_finger_torque(finger)
        elif label is not None:
            self._set_label_status(label, f"{finger[0].upper()}:OK", "green")

    def _finger_is_active(self, finger: str, positions: list[int]) -> bool:
        indices = self._finger_joint_map().get(finger, [])
        if not indices:
            return False
        return not all(
            positions[idx] < self.closed_threshold
            for idx in indices
            if idx < len(positions)
        )

    def _boost_finger_torque(self, finger: str) -> None:
        values = self._current_joint_torque_values()
        indices = self._finger_joint_map().get(finger, [])
        if not indices:
            return
        step = max(1, self.slip_torque_boost_step_spin.value())
        changed = False
        for idx in indices:
            if idx >= len(values):
                continue
            boosted = min(MAX_JOINT_POSITION, values[idx] + step)
            if boosted != values[idx]:
                values[idx] = boosted
                changed = True
        if not changed:
            self._update_commanded_torque_display(
                boosted_finger=finger, maxed_out=True
            )
            return
        self.current_torque_values = values
        self._sync_per_joint_torque_sliders()
        self._publish(None, None, [float(value) for value in values])
        boosted_values = [values[idx] for idx in indices if idx < len(values)]
        self._update_commanded_torque_display(
            boosted_finger=finger, boosted_values=boosted_values
        )
        self.status_updated.emit(
            "info", f"Slip torque boost: {finger} -> {boosted_values}"
        )

    def _current_joint_torque_values(self) -> list[int]:
        joint_count = len(self.hand_config.joint_names)
        if len(self.current_torque_values) != joint_count:
            self.current_torque_values = [self.torque_slider.value()] * joint_count
        return list(self.current_torque_values)

    def _flatten_touch(self, data: Any) -> list[int]:
        return [int(value) for value in _flatten_numeric(data)]

    def _get_max_pressure(self, data: Any) -> int:
        flat = self._flatten_touch(data)
        return max(flat) if flat else 0

    def _touch_location_delta(self, previous: list[int], current: list[int]) -> float:
        previous_loc = self._pressure_centroid(previous)
        current_loc = self._pressure_centroid(current)
        if previous_loc is None or current_loc is None:
            return 0.0
        dx = current_loc[0] - previous_loc[0]
        dy = current_loc[1] - previous_loc[1]
        return (dx * dx + dy * dy) ** 0.5

    def _pressure_centroid(self, flat: list[int]) -> tuple[float, float] | None:
        if not flat:
            return None
        rows = 12
        cols = 6
        total = sum(flat)
        if total <= 0:
            return None
        cx = 0.0
        cy = 0.0
        for idx, value in enumerate(flat[: rows * cols]):
            if value <= 0:
                continue
            row = idx // cols
            col = idx % cols
            cx += col * value
            cy += row * value
        return cx / total, cy / total

    def _set_label_status(
        self, label: QtWidgets.QLabel, text: str, color: str
    ) -> None:
        label.setText(text)
        label.setStyleSheet(f"color: {color};")

    def update_sensor_values(self, payload: dict[str, list[Any]]) -> None:
        self.latest_sensor_values.update(payload)
        for sensor_key in ("angle", "torque", "temperature", "current"):
            if sensor_key not in self.latest_sensor_values:
                continue
            grouped = self._group_values_by_finger(
                self.latest_sensor_values.get(sensor_key)
            )
            for finger, values in grouped.items():
                label = self.realtime_labels.get(finger, {}).get(sensor_key)
                if label is not None:
                    label.setText(self._format_cell_value(values))

    def _finger_key_from_name(self, name: str) -> str | None:
        lowered = name.lower()
        if "thumb" in lowered:
            return "thumb"
        if "index" in lowered:
            return "index"
        if "middle" in lowered:
            return "middle"
        if "ring" in lowered:
            return "ring"
        if "pinky" in lowered or "little" in lowered:
            return "pinky"
        return None

    def _group_values_by_finger(self, values: list[Any] | None) -> dict[str, list[Any]]:
        grouped = {finger: [] for finger in self.finger_order}
        if not values:
            return grouped
        for idx, name in enumerate(self.hand_config.joint_names):
            if idx >= len(values):
                break
            finger = self._finger_key_from_name(name)
            if finger:
                grouped[finger].append(values[idx])
        return grouped

    def _format_cell_value(self, values: list[Any] | Any) -> str:
        if values is None:
            return "--"
        if isinstance(values, list):
            if not values:
                return "--"
            return "[" + ", ".join(self._format_scalar(value) for value in values) + "]"
        return self._format_scalar(values)

    def _format_scalar(self, value: Any) -> str:
        if isinstance(value, (int, float)):
            return f"{value:.0f}"
        return str(value)

    def update_status(self, status_type: str, message: str) -> None:
        if hasattr(self, "connection_status"):
            if status_type == "error":
                self.connection_status.setText("ROS2 Hand Error")
                self.connection_status.setObjectName("StatusError")
            elif "state received" in message.lower() or "connected" in message.lower():
                self.connection_status.setText("ROS2 Hand Connected")
                self.connection_status.setObjectName("StatusInfo")
            self.connection_status.style().unpolish(self.connection_status)
            self.connection_status.style().polish(self.connection_status)
        if not hasattr(self, "status_log"):
            return
        current_time = time.strftime("%H:%M:%S")
        log_entry = f"[{current_time}] {message}\n"
        current_log = self.status_log.text()
        if len(current_log) > 10000:
            current_log = current_log[-10000:]
        self.status_log.setText(log_entry + current_log)
        self.status_log.setObjectName(
            "StatusError" if status_type == "error" else "StatusInfo"
        )
        self.status_log.style().unpolish(self.status_log)
        self.status_log.style().polish(self.status_log)

    def clear_status_log(self) -> None:
        self.status_log.setText("Log cleared")
        self.status_log.setObjectName("StatusInfo")
        self.status_log.style().unpolish(self.status_log)
        self.status_log.style().polish(self.status_log)

    def update_system_info(self) -> None:
        side = self.side_combo.currentText()
        model = self.model_combo.currentText()
        interface_name = self.interface_edit.text().strip() or "can0"
        interface_type = self.interface_type_combo.currentText()
        backend_status = "Running" if self._backend_is_running() else "Stopped"
        serial_number = self._device_info_value("serial_number")
        firmware_version = self._device_info_value("firmware_version")
        mechanical_version = self._device_info_value("mechanical_version")
        pcb_version = self._device_info_value("pcb_version")
        polling_status = self._bool_text(self._polling_active())
        streaming_status = self._bool_text(self.latest_control_status.get("streaming"))
        last_action = self.latest_control_status.get("action") or "None"
        last_result = self._last_result_summary()
        info = (
            f"Hand Type: {side}\n"
            f"Joint Model: {model}\n"
            f"Serial Number: {serial_number}\n"
            f"Firmware Version: {firmware_version}\n"
            f"Mechanical Version: {mechanical_version}\n"
            f"PCB Version: {pcb_version}\n"
            f"CAN: {interface_name}\n"
            f"Backend: {interface_type}\n"
            f"Backend Status: {backend_status}\n"
            f"Sensor Read Mode: {self.sensor_read_mode}\n"
            f"Polling: {polling_status}\n"
            f"Streaming: {streaming_status}\n"
            f"Last Control Action: {last_action}\n"
            f"Last Result: {last_result}\n"
            f"Command Topic: /realhand/{side}/hand/command\n"
            f"JSON Topic: /realhand/{side}/hand/command_json\n"
            f"State Topic: /realhand/{side}/hand/state\n"
            f"Snapshot Topic: /realhand/{side}/hand/snapshot\n"
            f"Device Info Topic: /realhand/{side}/hand/device_info\n"
            f"Touch Topic: /realhand/{side}/hand/touch\n"
            f"Control Status Topic: /realhand/{side}/hand/control_status\n"
            f"Blocking Result Topic: /realhand/{side}/hand/blocking_result\n"
            f"Temperature Topic: /realhand/{side}/hand/temperature\n"
            f"Current Topic: /realhand/{side}/hand/current\n"
            f"Joint Count: {len(self.hand_config.joint_names)}"
        )
        self.info_label.setText(info)

    def _last_result_summary(self) -> str:
        if not self.latest_blocking_result:
            return "None"
        action = self.latest_blocking_result.get("action") or "result"
        sensor = self.latest_blocking_result.get("sensor") or "unknown"
        if self.latest_blocking_result.get("ok") is False:
            error = self.latest_blocking_result.get("error") or "failed"
            return f"{action} {sensor}: {error}"
        return f"{action} {sensor}: ok"

    def _device_info_value(self, key: str) -> str:
        value = self.latest_device_info.get(key)
        if value in (None, ""):
            return "Unavailable"
        return str(value)

    def shutdown(self) -> None:
        if self._detect_thread is not None and self._detect_thread.isRunning():
            self._detect_thread.quit()
            self._detect_thread.wait(500)
        if self.cycle_timer and self.cycle_timer.isActive():
            self.cycle_timer.stop()
        if self.auto_grab_timer and self.auto_grab_timer.isActive():
            self.auto_grab_timer.stop()
        if self.blocking_heatmap_timer and self.blocking_heatmap_timer.isActive():
            self.blocking_heatmap_timer.stop()
        if self.publish_timer and self.publish_timer.isActive():
            self.publish_timer.stop()
        for timer in (
            self.live_speed_timer,
            self.live_torque_timer,
            self.live_joint_speed_timer,
            self.live_joint_torque_timer,
        ):
            if timer.isActive():
                timer.stop()
        if self._backend_is_running():
            self.stop_backend()

    def closeEvent(self, event) -> None:  # noqa: N802
        self.shutdown()
        super().closeEvent(event)

    @staticmethod
    def _scroll_for_grid(layout: QtWidgets.QGridLayout) -> QtWidgets.QScrollArea:
        widget = QtWidgets.QWidget()
        widget.setLayout(layout)
        area = QtWidgets.QScrollArea()
        area.setWidgetResizable(True)
        area.setFrameShape(QtWidgets.QFrame.NoFrame)
        area.setWidget(widget)
        return area

    @staticmethod
    def _scroll(content_layout: QtWidgets.QVBoxLayout) -> QtWidgets.QScrollArea:
        widget = QtWidgets.QWidget()
        widget.setLayout(content_layout)
        area = QtWidgets.QScrollArea()
        area.setWidgetResizable(True)
        area.setWidget(widget)
        return area

    @staticmethod
    def _clear_layout(layout: QtWidgets.QLayout) -> None:
        while layout.count():
            item = layout.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.deleteLater()
            nested = item.layout()
            if nested is not None:
                HandPanel._clear_layout(nested)

    @staticmethod
    def _fit_values(values: list[int], count: int, default: int) -> list[int]:
        fitted = [int(value) for value in values[:count]]
        fitted.extend(default for _ in range(count - len(fitted)))
        return fitted


class ArmPanel(QtWidgets.QWidget):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.publish_command: Callable[
            [list[str], list[float] | None, list[float] | None, list[float] | None],
            None,
        ] | None = None
        self.publish_json: Callable[[str], None] | None = None
        self.configure_topics: Callable[[str], None] | None = None
        self.joint_rows: list[DoubleRow] = []
        self.velocity_rows: list[DoubleRow] = []
        self.acceleration_rows: list[DoubleRow] = []
        self.joint_names = list(get_arm_model_spec("A7lite").joint_names)
        self.backend_process: QtCore.QProcess | None = None
        self._backend_stop_requested = False
        self._backend_owned_by_gui = False

        self.model_combo = QtWidgets.QComboBox()
        self.model_combo.addItems([spec.name for spec in ARM_MODELS.values()])
        self.side_combo = QtWidgets.QComboBox()
        self.side_combo.addItems(["left", "right"])
        self.interface_edit = QtWidgets.QLineEdit("can0")
        self.interface_edit.setMaximumWidth(110)
        self.interface_type_combo = QtWidgets.QComboBox()
        self.interface_type_combo.addItems(["socketcan", "lbot"])
        self.world_frame_edit = QtWidgets.QLineEdit("urdf")
        self.world_frame_edit.setMaximumWidth(110)
        apply_button = QtWidgets.QPushButton("Apply Topics")
        apply_button.clicked.connect(self._apply_topics)
        self.backend_button = QtWidgets.QPushButton("Start Backend")
        self.backend_button.setProperty("category", "action")
        self.backend_button.clicked.connect(self.on_backend_button_clicked)

        header = QtWidgets.QHBoxLayout()
        header.addWidget(QtWidgets.QLabel("Model"))
        header.addWidget(self.model_combo)
        header.addWidget(QtWidgets.QLabel("Side"))
        header.addWidget(self.side_combo)
        header.addWidget(QtWidgets.QLabel("Interface"))
        header.addWidget(self.interface_edit)
        header.addWidget(QtWidgets.QLabel("Backend"))
        header.addWidget(self.interface_type_combo)
        header.addWidget(QtWidgets.QLabel("World"))
        header.addWidget(self.world_frame_edit)
        header.addWidget(apply_button)
        header.addWidget(self.backend_button)
        header.addStretch(1)

        self.tabs = QtWidgets.QTabWidget()
        self.joint_layout = QtWidgets.QVBoxLayout()
        self.velocity_layout = QtWidgets.QVBoxLayout()
        self.acceleration_layout = QtWidgets.QVBoxLayout()
        self.tabs.addTab(HandPanel._scroll(self.joint_layout), "Joints")
        self.tabs.addTab(HandPanel._scroll(self.velocity_layout), "Velocity")
        self.tabs.addTab(HandPanel._scroll(self.acceleration_layout), "Acceleration")

        send_joints = QtWidgets.QPushButton("Send Joints")
        send_velocity = QtWidgets.QPushButton("Send Velocity")
        send_acceleration = QtWidgets.QPushButton("Send Acceleration")
        send_joints.clicked.connect(self.send_joints)
        send_velocity.clicked.connect(self.send_velocity)
        send_acceleration.clicked.connect(self.send_acceleration)
        buttons = QtWidgets.QHBoxLayout()
        buttons.addWidget(send_joints)
        buttons.addWidget(send_velocity)
        buttons.addWidget(send_acceleration)
        buttons.addStretch(1)

        enable = QtWidgets.QPushButton("Enable")
        disable = QtWidgets.QPushButton("Disable")
        home = QtWidgets.QPushButton("Home")
        stop = QtWidgets.QPushButton("Emergency Stop")
        enable.clicked.connect(lambda: self.send_json_action("enable"))
        disable.clicked.connect(lambda: self.send_json_action("disable"))
        home.clicked.connect(lambda: self.send_json_action("home"))
        stop.clicked.connect(lambda: self.send_json_action("emergency_stop"))
        actions = QtWidgets.QHBoxLayout()
        actions.addWidget(enable)
        actions.addWidget(disable)
        actions.addWidget(home)
        actions.addWidget(stop)
        actions.addStretch(1)

        self.state_text = QtWidgets.QPlainTextEdit()
        self.state_text.setReadOnly(True)
        self.pose_text = QtWidgets.QPlainTextEdit()
        self.pose_text.setReadOnly(True)
        self.pose_text.setMaximumHeight(110)
        self.backend_status = QtWidgets.QLabel("Backend stopped")
        self.backend_status.setObjectName("StatusInfo")
        self.backend_status.setWordWrap(True)

        layout = QtWidgets.QVBoxLayout(self)
        layout.addLayout(header)
        layout.addWidget(self.backend_status)
        layout.addWidget(self.tabs, 1)
        layout.addLayout(buttons)
        layout.addLayout(actions)
        layout.addWidget(QtWidgets.QLabel("Joint State"))
        layout.addWidget(self.state_text)
        layout.addWidget(QtWidgets.QLabel("Pose JSON"))
        layout.addWidget(self.pose_text)

        self.model_combo.currentTextChanged.connect(self._on_model_changed)
        self._rebuild_controls()

    def _on_model_changed(self, model: str) -> None:
        self._apply_model_defaults(model)
        self._rebuild_controls()

    def _apply_model_defaults(self, model: str) -> None:
        spec = get_arm_model_spec(model)
        self.interface_edit.setText(spec.default_interface_name)
        index = self.interface_type_combo.findText(spec.default_interface_type)
        if index >= 0:
            self.interface_type_combo.setCurrentIndex(index)

    def _rebuild_controls(self) -> None:
        spec = get_arm_model_spec(self.model_combo.currentText())
        self.joint_names = list(spec.joint_names)
        labels = [name.replace("_", " ").title() for name in self.joint_names]
        self.joint_rows = self._double_rows(
            self.joint_layout, labels, -6.283, 6.283, 0.01, "rad"
        )
        self.velocity_rows = self._double_rows(
            self.velocity_layout,
            labels,
            spec.velocity_range[0],
            spec.velocity_range[1],
            0.05,
            "rad/s",
        )
        self.acceleration_rows = self._double_rows(
            self.acceleration_layout,
            labels,
            spec.acceleration_range[0],
            spec.acceleration_range[1],
            0.1,
            "rad/s^2",
        )

    def _double_rows(
        self,
        layout: QtWidgets.QVBoxLayout,
        labels: list[str],
        minimum: float,
        maximum: float,
        step: float,
        suffix: str,
    ) -> list[DoubleRow]:
        HandPanel._clear_layout(layout)
        rows: list[DoubleRow] = []
        for label in labels:
            row = DoubleRow(label, 0.0, minimum, maximum, step, suffix)
            layout.addWidget(row)
            rows.append(row)
        layout.addStretch(1)
        return rows

    def _apply_topics(self) -> None:
        if self.configure_topics is not None:
            self.configure_topics(self.side_combo.currentText())
        self._set_backend_status(
            f"Topics set to side={self.side_combo.currentText()}", "info"
        )

    def on_backend_button_clicked(self) -> None:
        if self._backend_is_running():
            self.stop_backend()
        else:
            self.start_backend()

    def start_backend(self) -> None:
        if self._backend_is_running():
            self._set_backend_status("Arm backend is already running", "warning")
            return

        model = self.model_combo.currentText()
        spec = get_arm_model_spec(model)
        side = self.side_combo.currentText()
        interface_name = self.interface_edit.text().strip() or spec.default_interface_name
        interface_type = self.interface_type_combo.currentText() or spec.default_interface_type
        world_frame = self.world_frame_edit.text().strip() or "urdf"
        self.interface_edit.setText(interface_name)
        self.world_frame_edit.setText(world_frame)
        self._apply_topics()

        stopped_stale = self._stop_stale_arm_backends(side)
        if stopped_stale:
            self._set_backend_status(
                f"Stopped {stopped_stale} stale arm backend process(es) for side={side}",
                "warning",
            )

        args = [
            "launch",
            "realhand_ros2",
            "arm.launch.py",
            f"model:={model}",
            f"side:={side}",
            f"interface_name:={interface_name}",
            f"interface_type:={interface_type}",
            f"world_frame:={world_frame}",
        ]

        process = QtCore.QProcess(self)
        process.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(self._on_backend_output)
        process.errorOccurred.connect(self._on_backend_error)
        process.finished.connect(self._on_backend_finished)

        self.backend_process = process
        self._backend_stop_requested = False
        self._set_backend_controls_enabled(False)
        self.backend_button.setText("Stop Backend")
        self.backend_button.setProperty("category", "danger")
        HandPanel._refresh_button_style(self.backend_button)
        process.start("ros2", args)
        if not process.waitForStarted(3000):
            self._set_backend_status("Failed to start arm backend process", "error")
            self.backend_process = None
            self._backend_owned_by_gui = False
            self._set_backend_controls_enabled(True)
            self.backend_button.setText("Start Backend")
            self.backend_button.setProperty("category", "action")
            HandPanel._refresh_button_style(self.backend_button)
            return

        self._backend_owned_by_gui = True
        self._set_backend_status(f"Started arm backend: ros2 {' '.join(args)}", "info")

    def stop_backend(self) -> None:
        if not self._backend_is_running() and not self._backend_owned_by_gui:
            return

        self._backend_stop_requested = True
        owned_by_gui = self._backend_owned_by_gui
        self._set_backend_status("Stopping arm backend", "warning")
        if self._backend_is_running():
            assert self.backend_process is not None
            self.backend_process.terminate()
            if not self.backend_process.waitForFinished(2000):
                self.backend_process.kill()
                self.backend_process.waitForFinished(1000)
        if owned_by_gui:
            stopped = self._stop_stale_arm_backends(self.side_combo.currentText())
            if stopped:
                self._set_backend_status(
                    f"Stopped {stopped} arm backend child process(es)", "warning"
                )
        self._backend_owned_by_gui = False

    def _backend_is_running(self) -> bool:
        return (
            self.backend_process is not None
            and self.backend_process.state() != QtCore.QProcess.NotRunning
        )

    def _on_backend_output(self) -> None:
        if self.backend_process is None:
            return
        output = bytes(self.backend_process.readAllStandardOutput()).decode(
            errors="replace"
        )
        for line in output.splitlines():
            line = line.strip()
            if line:
                self._set_backend_status(f"Backend: {line}", "info")

    def _on_backend_error(self, error: QtCore.QProcess.ProcessError) -> None:
        if error == QtCore.QProcess.FailedToStart:
            self._set_backend_status("Arm backend command failed to start", "error")
            return
        self._set_backend_status(f"Arm backend process error: {int(error)}", "error")

    def _on_backend_finished(
        self, exit_code: int, exit_status: QtCore.QProcess.ExitStatus
    ) -> None:
        if self._backend_stop_requested:
            self._set_backend_status("Arm backend stopped", "warning")
        elif exit_status == QtCore.QProcess.CrashExit:
            self._set_backend_status("Arm backend crashed", "error")
        else:
            self._set_backend_status(
                f"Arm backend exited with code {exit_code}", "warning"
            )
        self.backend_process = None
        if self._backend_stop_requested:
            self._backend_owned_by_gui = False
        self._backend_stop_requested = False
        self._set_backend_controls_enabled(True)
        self.backend_button.setText("Start Backend")
        self.backend_button.setProperty("category", "action")
        HandPanel._refresh_button_style(self.backend_button)

    def _set_backend_controls_enabled(self, enabled: bool) -> None:
        self.model_combo.setEnabled(enabled)
        self.side_combo.setEnabled(enabled)
        self.interface_edit.setEnabled(enabled)
        self.interface_type_combo.setEnabled(enabled)
        self.world_frame_edit.setEnabled(enabled)

    def _stop_stale_arm_backends(self, side: str) -> int:
        stopped = 0
        current_pid = os.getpid()
        for pid in self._realhand_arm_node_pids(side):
            if pid == current_pid:
                continue
            if HandPanel._terminate_pid(pid):
                stopped += 1
        return stopped

    def _realhand_arm_node_pids(self, side: str) -> list[int]:
        pids: list[int] = []
        try:
            entries = os.listdir("/proc")
        except OSError:
            return pids
        for entry in entries:
            if not entry.isdigit():
                continue
            pid = int(entry)
            cmdline = HandPanel._proc_cmdline(pid)
            if not cmdline or "realhand_arm_node" not in " ".join(cmdline):
                continue
            params_file = HandPanel._params_file_from_cmdline(cmdline)
            proc_side = HandPanel._params_file_side(params_file) if params_file else None
            proc_side = proc_side or HandPanel._cmdline_side(cmdline)
            if proc_side != side:
                continue
            pids.append(pid)
        return pids

    def _set_backend_status(self, message: str, status_type: str = "info") -> None:
        self.backend_status.setText(message)
        self.backend_status.setObjectName(
            "StatusError" if status_type == "error" else "StatusInfo"
        )
        self.backend_status.style().unpolish(self.backend_status)
        self.backend_status.style().polish(self.backend_status)

    def shutdown(self) -> None:
        if self._backend_is_running():
            self.stop_backend()

    def closeEvent(self, event) -> None:  # noqa: N802
        self.shutdown()
        super().closeEvent(event)

    def send_joints(self) -> None:
        self._publish([row.value() for row in self.joint_rows], None, None)

    def send_velocity(self) -> None:
        self._publish(None, [row.value() for row in self.velocity_rows], None)

    def send_acceleration(self) -> None:
        self._publish(None, None, [row.value() for row in self.acceleration_rows])

    def _publish(
        self,
        positions: list[float] | None,
        velocities: list[float] | None,
        efforts: list[float] | None,
    ) -> None:
        if self.publish_command is None:
            return
        self.publish_command(self.joint_names, positions, velocities, efforts)

    def send_json_action(self, action: str) -> None:
        if self.publish_json is not None:
            self.publish_json(json.dumps({"action": action}))

    def update_state(self, msg: JointState) -> None:
        payload = {
            "name": list(msg.name),
            "position": [round(value, 4) for value in msg.position],
            "velocity": [round(value, 4) for value in msg.velocity],
            "effort": [round(value, 4) for value in msg.effort],
        }
        self.state_text.setPlainText(json.dumps(payload, indent=2))

    def update_pose(self, msg: String) -> None:
        try:
            parsed = json.loads(msg.data)
            self.pose_text.setPlainText(json.dumps(parsed, indent=2))
        except json.JSONDecodeError:
            self.pose_text.setPlainText(msg.data)


def _as_list(value: Any) -> list[Any]:
    if value is None:
        return []
    if isinstance(value, list):
        return value
    if isinstance(value, tuple):
        return list(value)
    if isinstance(value, dict):
        for key in ("values", "data", "items"):
            if key in value:
                return _as_list(value[key])
        if value and all(isinstance(item, (int, float)) for item in value.values()):
            return list(value.values())
        return []
    to_list = getattr(value, "to_list", None)
    if callable(to_list):
        return list(to_list())
    tolist = getattr(value, "tolist", None)
    if callable(tolist):
        return _as_list(tolist())
    return []


def _flatten_numeric(value: Any) -> list[float]:
    values: list[float] = []
    if value is None:
        return values
    if isinstance(value, dict):
        for candidate in ("values", "data", "matrix"):
            if candidate in value:
                return _flatten_numeric(value[candidate])
        for nested in value.values():
            values.extend(_flatten_numeric(nested))
        return values
    if isinstance(value, (list, tuple)):
        for item in value:
            values.extend(_flatten_numeric(item))
        return values
    to_list = getattr(value, "to_list", None)
    if callable(to_list):
        return _flatten_numeric(to_list())
    tolist = getattr(value, "tolist", None)
    if callable(tolist):
        return _flatten_numeric(tolist())
    try:
        values.append(float(value))
    except (TypeError, ValueError):
        pass
    return values
