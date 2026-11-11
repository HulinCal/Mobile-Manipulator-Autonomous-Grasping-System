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
from sensor_msgs.msg import JointState
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
    """订阅 /joint_states 显示当前角，发布目标到 /arm_controller/joint_trajectory。"""

    def __init__(self):
        super().__init__('joint_slider_gui')
        self.current_pos = {name: 0.0 for name in JOINT_NAMES}
        self.target_pos = {name: 0.0 for name in JOINT_NAMES}

        self.pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.sub = self.create_subscription(
            JointState, '/joint_states', self._on_joint_state, 10)

    def _on_joint_state(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name in self.current_pos:
                self.current_pos[name] = pos

    def send_target(self, duration_sec=2.0):
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = [self.target_pos[n] for n in JOINT_NAMES]
        from rclpy.duration import Duration
        pt.time_from_start = Duration(seconds=duration_sec).to_msg()
        traj.points = [pt]
        self.pub.publish(traj)


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
        self.node.send_target()

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
        self.btn_send.clicked.connect(lambda: node.send_target(2.0))
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
        for row in self.rows:
            row.update_current()

    def _go_home(self):
        for row in self.rows:
            row.set_target(0.0)
        self.node.send_target(3.0)

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
