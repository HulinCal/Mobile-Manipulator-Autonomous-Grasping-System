#!/usr/bin/env python3
"""
关节滑块 GUI：拖动 6 个关节滑块直接控制 mycobot280 真实机械臂位置。

通过 /arm_controller/joint_trajectory 话题发送单点 JointTrajectory，
不依赖 MoveIt 规划，滑块一释放就动。

用法：
    ros2 run mycobot_handeye_calibration joint_slider_gui.py
    或在标定 launch 中随 arm bringup 一起启动。
"""
import sys
import threading

import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QSlider, QPushButton, QDoubleSpinBox, QGroupBox,
)

# 6 个机械臂关节名（与 arm_controller 配置一致）
JOINT_NAMES = [
    'link1_to_link2',
    'link2_to_link3',
    'link3_to_link4',
    'link4_to_link5',
    'link5_to_link6',
    'link6_to_link6_flange',
]

# 滑块范围（弧度），略小于 URDF 限位 ±165°≈±2.88
SLIDER_MIN = -2.8
SLIDER_MAX = 2.8
SLIDER_STEP = 0.005
# 滑块内部用整数（放大 1000 倍）
SCALE = 1000


class JointSliderNode(Node):
    """订阅 controller_state 显示当前角，发布目标到 /arm_controller/joint_trajectory。"""

    def __init__(self):
        super().__init__('joint_slider_gui')
        self.current_pos = {name: 0.0 for name in JOINT_NAMES}
        self.target_pos = {name: 0.0 for name in JOINT_NAMES}
        self.sync_requested = False   # GUI 用: 发送后把界面目标同步为实际下发值

        self.pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        # 用控制器反馈而不是 /joint_states (该话题有多个发布者, 会读到旧值)
        self.sub = self.create_subscription(
            JointTrajectoryControllerState, '/arm_controller/controller_state',
            self._on_controller_state, 10)

    def _on_controller_state(self, msg):
        for name, pos in zip(msg.joint_names, msg.feedback.positions):
            if name in self.current_pos:
                self.current_pos[name] = pos

    def send_target(self, changed=None, duration_sec=2.0):
        """发送轨迹.

        changed 为本次被拖动的关节名: 其余关节一律使用实测当前角作为目标,
        避免把上一次遗留的目标值一起重发导致其他关节跟着动.
        changed=None 表示按界面上的全部目标值发送.
        """
        if changed is None:
            targets = {n: self.target_pos[n] for n in JOINT_NAMES}
        else:
            targets = {}
            for n in JOINT_NAMES:
                if n == changed:
                    targets[n] = self.target_pos[n]
                else:
                    targets[n] = self.current_pos.get(n, self.target_pos[n])
        self.target_pos.update(targets)

        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = [targets[n] for n in JOINT_NAMES]
        from rclpy.duration import Duration
        pt.time_from_start = Duration(seconds=duration_sec).to_msg()
        traj.points = [pt]
        self.pub.publish(traj)
        self.sync_requested = True


class SliderRow(QWidget):
    """单个关节的一行：名称 + 当前值标签 + 滑块 + 目标值输入框。"""

    def __init__(self, name, node, parent=None):
        super().__init__(parent)
        self.name = name
        self.node = node

        lay = QHBoxLayout(self)

        self.lbl_name = QLabel(name)
        self.lbl_name.setFixedWidth(150)

        self.lbl_current = QLabel('cur: 0.000')
        self.lbl_current.setFixedWidth(90)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(int(SLIDER_MIN * SCALE), int(SLIDER_MAX * SCALE))
        self.slider.setSingleStep(int(SLIDER_STEP * SCALE))
        self.slider.setValue(0)
        self.slider.sliderReleased.connect(self._on_slider_release)

        self.spin = QDoubleSpinBox()
        self.spin.setRange(SLIDER_MIN, SLIDER_MAX)
        self.spin.setSingleStep(SLIDER_STEP)
        self.spin.setDecimals(3)
        self.spin.valueChanged.connect(self._on_spin_change)

        lay.addWidget(self.lbl_name)
        lay.addWidget(self.lbl_current)
        lay.addWidget(self.slider, 1)
        lay.addWidget(self.spin)

    def _on_slider_release(self):
        val = self.slider.value() / SCALE
        self.spin.blockSignals(True)
        self.spin.setValue(val)
        self.spin.blockSignals(False)
        self.node.target_pos[self.name] = val
        self.node.send_target(self.name)   # 只让本关节动，其余锁在当前实测角

    def _on_spin_change(self, val):
        self.slider.blockSignals(True)
        self.slider.setValue(int(val * SCALE))
        self.slider.blockSignals(False)
        self.node.target_pos[self.name] = val

    def update_current(self):
        cur = self.node.current_pos.get(self.name, 0.0)
        self.lbl_current.setText(f'cur: {cur:+.3f}')

    def set_target(self, val):
        self.spin.setValue(val)

    def get_target(self):
        return self.spin.value()


class MainWindow(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle('MyCobot280 Joint Slider Control')
        self.resize(700, 400)

        lay = QVBoxLayout(self)

        grp = QGroupBox('关节目标角度（弧度）')
        grid = QGridLayout(grp)
        self.rows = []
        for i, name in enumerate(JOINT_NAMES):
            row = SliderRow(name, node)
            self.rows.append(row)
            grid.addWidget(row, i, 0)
        lay.addWidget(grp)

        btn_row = QHBoxLayout()
        self.btn_send = QPushButton('发送当前目标 (2s)')
        self.btn_send.clicked.connect(lambda: node.send_target(None, 2.0))
        self.btn_home = QPushButton('回零 (0,0,0,0,0,0)')
        self.btn_home.clicked.connect(self._go_home)
        self.btn_get = QPushButton('获取当前位姿为目标')
        self.btn_get.clicked.connect(self._get_current)
        btn_row.addWidget(self.btn_send)
        btn_row.addWidget(self.btn_home)
        btn_row.addWidget(self.btn_get)
        lay.addLayout(btn_row)

        self.lbl_hint = QLabel(
            '提示：拖动滑块松手即发送；点"获取当前位姿"把当前关节角填入目标；'
            '与标定界面并行使用。')
        self.lbl_hint.setWordWrap(True)
        lay.addWidget(self.lbl_hint)

        # 定时器刷新当前关节角显示
        from python_qt_binding.QtCore import QTimer
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._refresh)
        self.timer.start(100)

    def _refresh(self):
        # 发送后把界面目标同步成实际下发值(其余关节=当时实测角)
        if self.node.sync_requested:
            self.node.sync_requested = False
            for row in self.rows:
                row.set_target(self.node.target_pos.get(row.name, 0.0))
        for row in self.rows:
            row.update_current()

    def _go_home(self):
        for row in self.rows:
            row.set_target(0.0)
        self.node.send_target(None, 3.0)

    def _get_current(self):
        for row in self.rows:
            row.set_target(self.node.current_pos.get(row.name, 0.0))


def main():
    rclpy.init(args=sys.argv)
    node = JointSliderNode()

    app = QApplication(sys.argv)
    win = MainWindow(node)
    win.show()

    # 用线程跑 rclpy spin，避免阻塞 Qt 事件循环
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    ret = app.exec_()

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(ret)


if __name__ == '__main__':
    main()
