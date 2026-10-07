#!/usr/bin/env python3
"""Grasp angle node - 由规划器在多个抓取俯仰角中选出最优可达角度的抓取节点。

与 grasp_node.py (固定竖直向下俯抓) 的区别仅在抓取姿态的生成方式,
其余 MoveGroup/夹爪/到位静止判定/急停逻辑全部继承自已验证的 GraspNode,
服务与话题接口完全一致 (/pick_place/pick 等), coordinator 无需改动。

设计动机
--------
相机安装在车头, 沿 base_link +x (小车前方) 观察苹果:
* 竖直向下俯抓 (grasp_node) 时, 夹爪从苹果正上方接近, 而相机从未看到过
  苹果顶面 —— 顶面信息对相机而言是一维盲区, 该角度最差;
* 夹爪轴线水平朝前 (平行于小车 x 轴) 接近时, 手指从苹果左右两侧环绕,
  利用的正是相机看得最清楚的正面/侧面信息, 该角度最优。

实现
----
抓取姿态用"俯仰角 pitch"参数化: 法兰 +z 轴 (夹爪趋近方向) 为
    a = cos(pitch) * d_xy - sin(pitch) * z_link1
其中 d_xy 是 link1 系下机械臂基座指向苹果的水平单位向量 (苹果在正前方时
即 link1 +y, 经 URDF yaw=-90° 安装关系正好等于 base_link +x 小车前方)。
pitch=0 -> 水平前抓 (最优); pitch=90 -> 竖直俯抓 (最差, 兜底)。

候选按 grasp_pitch_angles_deg 参数给定的"最优->最差"顺序排列, 逐个交给
MoveIt 规划器判定可达性, 第一个规划成功的角度即为当前最优抓取角度;
全部失败时对最优候选做一次完整时限兜底。竖直候选复用 grasp_node 已验证
的两个等价翻转四元数 (q=y / q=x)。

非竖直候选的 roll 角固定为: 法兰 x 轴 (夹爪开合方向, 见
adaptive_gripper.urdf.xacro: gripper_base x -> flange x) 保持水平且垂直于
趋近方向 —— 手指始终左右环绕苹果, 不会一上一下夹到苹果下方的台面。

水平抓取时法兰位于苹果后方 tcp_flange_z 处 (比俯抓更靠近臂基座,
可达性反而更好); pre-grasp 再沿趋近方向后退 grasp_approach_distance,
随后以 vel 0.1 缓慢前进到抓取位。抓取后的提升/收臂流程与 grasp_node
相同 (先纯竖直小提离地, 再升高+内收, 最后收臂回 home)。

Modes
-----
* ``sim_mode:=true`` : 无 move_group 时每个动作等待 sim_duration 后成功。
* ``sim_mode:=false``: 需要 move_group / ros2_control。
"""

import math
import time
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, PlanningOptions, Constraints, JointConstraint)

from wheeltec_mycobot_pick_place.grasp_node import GraspNode


class GraspAngleNode(GraspNode):

    def __init__(self):
        super().__init__(node_name='grasp_angle_node')

        # ---- parameters (angle-grasp specific) ---------------------------
        # 抓取俯仰角候选 [deg], 按"最优->最差"排序, 由规划器逐个判定:
        #   0  = 法兰轴水平朝苹果 (平行于小车前方 x 轴, 最优);
        #   90 = 竖直向下俯抓 (最差, 兜底, 与 grasp_node 一致)。
        self.declare_parameter(
            'grasp_pitch_angles_deg',
            [0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0])
        self.grasp_pitch_angles = [
            float(v) for v in
            self.get_parameter('grasp_pitch_angles_deg').value]

        # 上一次抓取成功的俯仰角, 下次优先尝试 (同父类 _last_grasp_q 思路)。
        self._last_pitch = None

        self.get_logger().info(
            f'grasp_angle_node pitch candidates (best->worst): '
            f'{self.grasp_pitch_angles} deg')

    # ------------------------------------------------------------------
    # 关节目标运动 (重写父类: 增加重试)
    # ------------------------------------------------------------------
    def _move_to_joint_values(self, values, retries: int = 2) -> bool:
        """机械臂到指定 6 关节角, 失败时短暂等待关节稳定后重试。

        父类不重试: 小车刚停稳 / 抓取间隙关节未完全静止时, MoveIt 会报
        "start point deviates from current robot state" 瞬态错误, 一次失败
        即放弃整个 pick。这里等关节稳定后重试 (真机验证该瞬态可恢复)。
        """
        names = ['link1_to_link2', 'link2_to_link3', 'link3_to_link4',
                 'link4_to_link5', 'link5_to_link6', 'link6_to_link6_flange']
        jcs = []
        for n, v in zip(names, values):
            jc = JointConstraint()
            jc.joint_name = n
            jc.position = float(v)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            jcs.append(jc)
        cons = Constraints(); cons.joint_constraints = jcs

        req = MotionPlanRequest()
        req.group_name = self.arm_group
        req.num_planning_attempts = 10
        req.allowed_planning_time = self.planning_time
        # 必须显式设置 (>0), 否则 Ruckig 轨迹平滑失败 (见父类注释)。
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3
        req.goal_constraints = [cons]
        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False
        for attempt in range(retries + 1):
            if self._send_move_goal(goal):
                return True
            if attempt < retries:
                self.get_logger().warn(
                    'Joint move failed (transient start-state deviation?), '
                    'wait for joints to settle and retry ...')
                time.sleep(1.0)
        return False

    # ------------------------------------------------------------------
    # 姿态构造
    # ------------------------------------------------------------------
    @staticmethod
    def _vertical_quats():
        """竖直俯抓的两个等价翻转候选 (与 grasp_node 完全相同, 已验证)。"""
        q1 = Quaternion(); q1.y = 1.0; q1.w = 0.0
        q2 = Quaternion(); q2.x = 1.0; q2.w = 0.0
        return [q1, q2]

    @staticmethod
    def _quat_from_basis(x, y, z):
        """由法兰坐标系三轴在 link1 系下的表示构造四元数 (Shepperd 法)。"""
        m00, m01, m02 = x[0], y[0], z[0]
        m10, m11, m12 = x[1], y[1], z[1]
        m20, m21, m22 = x[2], y[2], z[2]
        t = m00 + m11 + m22
        if t > 0.0:
            s = math.sqrt(t + 1.0) * 2.0
            qw, qx, qy, qz = 0.25 * s, (m21 - m12) / s, \
                (m02 - m20) / s, (m10 - m01) / s
        elif m00 > m11 and m00 > m22:
            s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
            qw, qx, qy, qz = (m21 - m12) / s, 0.25 * s, \
                (m01 + m10) / s, (m02 + m20) / s
        elif m11 > m22:
            s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
            qw, qx, qy, qz = (m02 - m20) / s, (m01 + m10) / s, \
                0.25 * s, (m12 + m21) / s
        else:
            s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
            qw, qx, qy, qz = (m10 - m01) / s, (m02 + m20) / s, \
                (m12 + m21) / s, 0.25 * s
        q = Quaternion()
        q.x, q.y, q.z, q.w = qx, qy, qz, qw
        return q

    def _quat_from_approach(self, a):
        """由趋近方向 a (法兰 +z) 构造法兰姿态。

        roll 角约束: 法兰 x 轴 (夹爪开合方向) = normalize(a x up), 对任意
        非竖直 pitch 该向量都水平且垂直于趋近方向 -> 手指左右环绕苹果。
        """
        n = math.sqrt(a[0] ** 2 + a[1] ** 2 + a[2] ** 2)
        z = (a[0] / n, a[1] / n, a[2] / n)
        # x = z x up (up = link1 +z)
        x = (z[1], -z[0], 0.0)
        nx = math.hypot(x[0], x[1])
        x = (x[0] / nx, x[1] / nx, 0.0)
        # y = z x x
        y = (z[1] * x[2] - z[2] * x[1],
             z[2] * x[0] - z[0] * x[2],
             z[0] * x[1] - z[1] * x[0])
        return self._quat_from_basis(x, y, z)

    def _grasp_orientations(self, d):
        """按"最优->最差"顺序生成 (pitch_deg, quat) 候选。"""
        out = []
        for pitch in self.grasp_pitch_angles:
            if pitch >= 89.0:
                # 竖直: 直接复用 grasp_node 已验证的两个翻转候选
                for q in self._vertical_quats():
                    out.append((pitch, q))
                continue
            th = math.radians(pitch)
            a = (math.cos(th) * d[0], math.cos(th) * d[1], -math.sin(th))
            out.append((pitch, self._quat_from_approach(a)))
        return out

    @staticmethod
    def _approach_vec(pitch, d):
        """俯仰角 pitch + 水平指向 d -> 法兰趋近方向 (法兰 +z, link1 系)。"""
        th = math.radians(pitch)
        return (math.cos(th) * d[0], math.cos(th) * d[1], -math.sin(th))

    # ------------------------------------------------------------------
    # 目标位姿
    # ------------------------------------------------------------------
    def _flange_along(self, apple: PoseStamped, quat: Quaternion,
                      approach, back_off: float) -> PoseStamped:
        """苹果中心沿 -approach 方向退 (tcp_flange_z + back_off) 得法兰位姿。

        back_off=0 即抓取位 (夹持中心落在苹果中心); back_off=approach_dist
        为 pre-grasp 位。pitch>0 时法兰自然位于苹果上方 sin(pitch) 分量处。
        """
        dist = self.tcp_flange_z + back_off
        out = PoseStamped()
        out.header = apple.header
        out.pose.position.x = apple.pose.position.x - approach[0] * dist
        out.pose.position.y = apple.pose.position.y - approach[1] * dist
        out.pose.position.z = apple.pose.position.z - approach[2] * dist
        out.pose.orientation = quat
        return out

    @staticmethod
    def _offset_target(src: PoseStamped, dz: float = 0.0,
                       dx: float = 0.0, dy: float = 0.0) -> PoseStamped:
        out = PoseStamped()
        out.header = src.header
        out.pose.position.x = src.pose.position.x + dx
        out.pose.position.y = src.pose.position.y + dy
        out.pose.position.z = src.pose.position.z + dz
        out.pose.orientation = src.pose.orientation
        return out

    # ------------------------------------------------------------------
    # 抓取流程 (仅姿态生成与父类不同, 时序/判定逻辑保持一致)
    # ------------------------------------------------------------------
    def _do_pick(self) -> bool:
        if self.apple_pose is None:
            self.get_logger().error('No apple pose received')
            return False

        if self.sim_mode:
            self.get_logger().info('[SIM] Picking apple (angle grasp) ...')
            time.sleep(self.sim_duration)
            self.get_logger().info('[SIM] Pick succeeded')
            return True

        # base_link 系加标定偏移, 再转到 link1 (与父类相同的注意事项:
        # 必须在臂未动时完成变换)。
        apple_base = self._apply_calibration_offset(self.apple_pose)
        apple = self._to_frame(apple_base, self.arm_base_frame)
        ax = apple.pose.position.x
        ay = apple.pose.position.y
        az = apple.pose.position.z

        # 水平指向: 臂基座 -> 苹果。苹果在正前方时即 link1 +y
        # (= base_link +x 小车前方); 有横向偏移时自动对准苹果。
        horiz = math.hypot(ax, ay)
        if horiz < 1e-3:
            d = (0.0, 1.0)
        else:
            d = (ax / horiz, ay / horiz)
        self.get_logger().info(
            f'Apple in {self.arm_base_frame}: '
            f'x={ax:.3f} y={ay:.3f} z={az:.3f}; '
            f'approach xy dir=({d[0]:.2f},{d[1]:.2f}) '
            f'(pitch 0 = horizontal / car front)')

        # 0) 回 home 与夹爪张开并行 (与父类相同)。
        self._publish_status('Moving to home and opening gripper')
        results = {}

        def _do_home():
            results['home'] = self._move_to_joint_values([0.0] * 6)

        def _do_open():
            results['open'] = self._move_gripper(self.gripper_open_value)

        t_home = threading.Thread(target=_do_home)
        t_open = threading.Thread(target=_do_open)
        t_home.start(); t_open.start()
        t_home.join(); t_open.join()
        if not results.get('home'):
            self.get_logger().error('Cannot reach home pose')
            return False
        if not results.get('open'):
            self.get_logger().error('Cannot open gripper')
            return False
        if self.grasp_init_only:
            self._publish_status(
                'Reached home - stop (grasp_init_only=true)')
            return True

        # 俯仰角候选 (最优水平 -> 最差竖直), 上次成功的角度排最前。
        cands = self._grasp_orientations(d)
        if self._last_pitch is not None:
            cands.sort(
                key=lambda c: 0 if abs(c[0] - self._last_pitch) < 1e-6
                else 1)
        grasp_ori_tol = 0.15

        # 1. pre-grasp: 第一个被规划器判定可达的角度即当前最优抓取角度。
        self._publish_status('Searching best grasp angle (planner)')
        chosen = None
        for pitch, q in cands:
            a = self._approach_vec(pitch, d)
            tgt = self._flange_along(apple, q, a, self.approach_dist)
            tp = tgt.pose.position
            self.get_logger().info(
                f'Try pre-grasp pitch={pitch:.0f}deg '
                f'flange=({tp.x:.3f},{tp.y:.3f},{tp.z:.3f})')
            if self._move_to_pose(
                    tgt,
                    planning_time_override=self.explore_planning_time,
                    ori_tol=grasp_ori_tol, retries=0):
                chosen = (pitch, q)
                break
        if chosen is None:
            # 兜底: 对最优 (最水平) 候选做一次完整时限 + 重试。
            self.get_logger().warn(
                'No reachable angle in quick attempts, '
                'one full-time retry on best (horizontal) candidate ...')
            pitch, q = cands[0]
            a = self._approach_vec(pitch, d)
            if self._move_to_pose(
                    self._flange_along(apple, q, a, self.approach_dist),
                    ori_tol=grasp_ori_tol):
                chosen = (pitch, q)
        if chosen is None:
            self.get_logger().error('Cannot reach pre-grasp at any angle')
            return False
        self._last_pitch = chosen[0]
        self._publish_status(
            f'Best grasp angle: pitch={chosen[0]:.0f}deg')

        # 2. 沿趋近方向缓慢前进到抓取位 (vel 0.1)。优先沿用 pre-grasp
        #    已验证可达的角度; 前进后法兰离臂更远, 可能不可达, 故按顺序
        #    尝试其余角度 (更竖直的角度法兰更高更近, 更易达)。
        self._publish_status('Approaching apple (along grasp axis)')
        order = [chosen] + [c for c in cands if c[1] is not chosen[1]]
        reached = False
        grasp_target = None
        for pitch, q in order:
            a = self._approach_vec(pitch, d)
            tgt = self._flange_along(apple, q, a, 0.0)
            tp = tgt.pose.position
            self.get_logger().info(
                f'Try grasp pose pitch={pitch:.0f}deg '
                f'flange=({tp.x:.3f},{tp.y:.3f},{tp.z:.3f})')
            if self._move_to_pose(
                    tgt,
                    planning_time_override=self.explore_planning_time,
                    ori_tol=grasp_ori_tol, vel_scale=0.1, retries=0):
                chosen = (pitch, q)
                grasp_target = tgt
                reached = True
                break
        if not reached:
            self.get_logger().warn(
                'Grasp pose not found in quick attempts, '
                'one full-time retry ...')
            pitch, q = chosen
            a = self._approach_vec(pitch, d)
            tgt = self._flange_along(apple, q, a, 0.0)
            if self._move_to_pose(tgt, ori_tol=grasp_ori_tol, vel_scale=0.1):
                grasp_target = tgt
                reached = True
        if not reached:
            self.get_logger().error('Cannot reach apple at any angle')
            return False
        self._last_pitch = chosen[0]

        # 3. MoveGroup 报成功不等于物理到位: TF 确认法兰到位且静止,
        #    未到位补发一次目标, 然后才闭合 (与父类一致)。
        self._publish_status('Waiting arm to settle at apple')
        if not self._wait_flange_settled(grasp_target):
            self._publish_status('Re-commanding grasp pose')
            if not self._move_to_pose(
                    grasp_target, ori_tol=grasp_ori_tol, vel_scale=0.1):
                return False
            if not self._wait_flange_settled(grasp_target):
                return False

        # 4. 闭合夹爪 (约 1/3 张开), 等 2s 确认夹住 (与父类一致)。
        self._publish_status('Closing gripper')
        if not self._move_gripper(self.gripper_close_value):
            return False
        time.sleep(2.0)

        # 5. 提起: 阶段 A 纯竖直小提让苹果离地; 阶段 B 升高+内收到运输
        #    位姿 (姿态保持抓取角度不变, 与父类两段式相同)。
        self._publish_status('Lifting apple')
        pitch, q = chosen
        a = self._approach_vec(pitch, d)
        grasp_flange = self._flange_along(apple, q, a, 0.0)
        clear_h = 0.0
        for h in (0.08, 0.05, 0.03):
            if self._move_to_pose(
                    self._offset_target(grasp_flange, dz=h),
                    planning_time_override=self.explore_planning_time,
                    ori_tol=grasp_ori_tol, vel_scale=0.15, retries=0):
                clear_h = h
                self.get_logger().info(
                    f'Initial vertical lift: {h:.2f}m (apple clear)')
                break

        lift_options = []
        if clear_h == 0.0:
            lift_options.append((self.lift_dist, 0.0))
        lift_options += [
            (0.15, 0.10),
            (0.20, 0.12),
            (0.10, 0.08),
        ]
        lifted = False
        for h, inner in lift_options:
            if self._move_to_pose(
                    self._offset_target(
                        grasp_flange, dz=h,
                        dx=-d[0] * inner, dy=-d[1] * inner),
                    planning_time_override=self.explore_planning_time,
                    ori_tol=grasp_ori_tol, retries=0):
                self.get_logger().info(
                    f'Lift succeeded: height={h:.2f}m inner={inner:.2f}m')
                lifted = True
                break
        if not lifted:
            self.get_logger().warn(
                'Lift not found in quick attempts, one full-time retry ...')
            h, inner = 0.15, 0.10
            if self._move_to_pose(
                    self._offset_target(
                        grasp_flange, dz=h,
                        dx=-d[0] * inner, dy=-d[1] * inner),
                    ori_tol=grasp_ori_tol):
                self.get_logger().info(
                    f'Lift succeeded (fallback): height={h:.2f}m '
                    f'inner={inner:.2f}m')
                lifted = True
        if not lifted:
            self.get_logger().error('Cannot lift to any tested pose')
            return False

        # 6. 收臂回 home (夹爪保持夹紧), 等臂物理收稳 (与父类一致)。
        self._publish_status('Retracting arm to home before driving back')
        if not self._move_to_joint_values([0.0] * 6):
            self.get_logger().error('Cannot retract arm to home')
            return False
        time.sleep(2.0)

        self.get_logger().info('Pick succeeded')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = GraspAngleNode()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
