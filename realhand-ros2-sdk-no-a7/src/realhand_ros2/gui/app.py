"""Qt application entry point for the RealHand ROS2 GUI."""

from __future__ import annotations

import sys

import rclpy
from PyQt5 import QtCore, QtWidgets

from realhand_ros2.gui.ros_client import GuiRosClient
from realhand_ros2.gui.widgets import ArmPanel, GUI_STYLESHEET, HandPanel


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, node: GuiRosClient) -> None:
        super().__init__()
        self.node = node
        self.setWindowTitle("Realhand Dexterous Hand Control Interface - ROS2")
        self.setMinimumSize(1200, 900)
        self.setStyleSheet(GUI_STYLESHEET)

        self.hand_panel = HandPanel()
        self.arm_panel = ArmPanel()
        tabs = QtWidgets.QTabWidget()
        tabs.addTab(self.hand_panel, "Hand")
        tabs.addTab(self.arm_panel, "Arm")
        self.setCentralWidget(tabs)

        self.hand_panel.publish_command = self.node.publish_hand_command
        self.hand_panel.publish_json = self.node.publish_hand_json
        self.hand_panel.configure_topics = self._configure_hand_topics
        self.arm_panel.publish_command = self.node.publish_arm_command
        self.arm_panel.publish_json = self.node.publish_arm_json
        self.arm_panel.configure_topics = self._configure_arm_topics

        self._configure_hand_topics(self.hand_panel.side_combo.currentText())
        self._configure_arm_topics(self.arm_panel.side_combo.currentText())

        self.spin_timer = QtCore.QTimer(self)
        self.spin_timer.timeout.connect(self._spin_ros)
        self.spin_timer.start(20)

    def _configure_hand_topics(self, side: str) -> None:
        self.node.configure_hand(
            side=side,
            state_callback=self.hand_panel.update_state,
            snapshot_callback=self.hand_panel.update_snapshot,
            device_info_callback=self.hand_panel.update_device_info,
            temperature_callback=self.hand_panel.update_temperature,
            current_callback=self.hand_panel.update_current,
            touch_callback=self.hand_panel.update_touch,
            control_status_callback=self.hand_panel.update_control_status,
            blocking_result_callback=self.hand_panel.update_blocking_result,
        )
        self.statusBar().showMessage(f"Hand topics set to side={side}", 3000)

    def _configure_arm_topics(self, side: str) -> None:
        self.node.configure_arm(
            side=side,
            state_callback=self.arm_panel.update_state,
            pose_callback=self.arm_panel.update_pose,
        )
        self.statusBar().showMessage(f"Arm topics set to side={side}", 3000)

    def _spin_ros(self) -> None:
        rclpy.spin_once(self.node, timeout_sec=0.0)

    def closeEvent(self, event) -> None:
        self.spin_timer.stop()
        self.hand_panel.shutdown()
        self.arm_panel.shutdown()
        event.accept()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow(GuiRosClient())
    window.show()
    exit_code = app.exec_()
    window.node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
