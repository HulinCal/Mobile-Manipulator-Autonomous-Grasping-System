#!/usr/bin/env python3
"""Grasp node - 通过 move_group action 驱动机械臂抓取苹果。

不依赖 MoveItPy (``moveit_py`` 未安装): 直接使用 move_group 提供的
``/move_action`` (moveit_msgs/MoveGroup) action 规划执行机械臂, 用
``/gripper_action_controller/gripper_cmd`` (control_msgs/GripperCommand)
驱动自适应夹爪。只要求 move_group + ros2_control 在运行。

服务:
* ``/pick_place/pick``  (std_srvs/Trigger) - 抓取
  ``/pick_place/apple_pose_final`` (base_link, 导航停车后发布的最终
  位姿, latched) 上的苹果位姿。
* ``/pick_place/place`` (std_srvs/Trigger) - 在
  ``/pick_place/place_pose`` 位姿放开苹果。
* ``/pick_place/arm_home`` (std_srvs/Trigger) - 回到 home。
* ``/pick_place/arm_stop`` (std_srvs/Trigger) - 机械臂急停:
  取消在途 MoveGroup goal (正在执行的抓取序列也随之中止, 不再继续
  后续动作), 并发布 ``/mycobot/stop`` 让固件立即停止物理运动。

抓取流程 (俯抓):
  1. 回 home 与张开夹爪 (并行)
  2. 移到苹果上方 pre-grasp (夹爪法兰 z 轴竖直朝下, 内收候选优先)
  3. 缓慢下降到苹果高度, TF 确认真实到位停稳
  4. 闭合夹爪 (约 1/3 张开)
  5. 先纯竖直小提安全离地, 再升高+内收到运输位姿
  6. 收臂回 home (爪保持夹紧)

Modes
-----
* ``sim_mode:=true`` : 无 move_group 时每个动作等待 sim_duration 后成功。
* ``sim_mode:=false``: 需要 move_group / ros2_control。
"""

import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)

from std_msgs.msg import String, Empty
from geometry_msgs.msg import PoseStamped, Quaternion
from std_srvs.srv import Trigger
from action_msgs.msg import GoalStatus

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, PlanningOptions, Constraints, PositionConstraint,
    OrientationConstraint, JointConstraint)
from moveit_msgs.msg import BoundingVolume
from shape_msgs.msg import SolidPrimitive

from control_msgs.action import GripperCommand

import tf2_ros
from tf2_geometry_msgs import do_transform_pose_stamped


class GraspNode(Node):

    def __init__(self, node_name='grasp_node'):
        super().__init__(node_name)

        # ---- parameters --------------------------------------------------
        self.declare_parameter('sim_mode', False)
        self.declare_parameter('arm_group', 'arm')
        self.declare_parameter('arm_base_frame', 'link1')
        self.declare_parameter('eef_link', 'link6_flange')
        self.declare_parameter('world_frame', 'base_link')
        # link6_flange +z 到夹爪夹持中心 (两指闭合中心) 的距离 [m]。
        # URDF: flange->gripper_base origin z=0.034, rpy=1.579; 手指沿
        # gripper_base +y 伸出, 变换后即法兰 +z。夹大苹果时接触点在手指
        # 中段, 综合约 0.06m, 可按实测微调。
        self.declare_parameter('tcp_flange_z', 0.06)
        # 视觉/手眼标定残差: 检测位姿在 world_frame(base_link) 各方向的
        # 整体偏差, 机械臂使用前加该偏移。x 实测约 0.09m; y/z 默认 0。
        self.declare_parameter('pose_x_offset', 0.055)
        self.declare_parameter('pose_y_offset', 0.0)
        self.declare_parameter('pose_z_offset', 0.0)
        # 苹果中心在 base_link 系的固定高度 [m]。测试时苹果放置高度恒定,
        # 而视觉深度给出的 z 不可靠。由真实成功夹取姿态标定为 0.109:
        # 固件法兰 link1 z=0.062, 经 base->link1 偏移0.167 + TCP 0.12 反推。
        # >=0: 直接用该值覆盖检测 z (忽略 pose_z_offset); <0: 用检测 z。
        self.declare_parameter('apple_fixed_z', 0.109)
        self.declare_parameter('grasp_approach_distance', 0.10)
        # pre-grasp 在 link1 y 方向(机械臂前方)向内收的距离: 苹果在臂展
        # 极限附近时, 正上方 pre-grasp 不可达, 先到内收的高位再下降。
        self.declare_parameter('pre_grasp_inner', 0.06)
        self.declare_parameter('lift_distance', 0.25)
        # 抓取前统一先回 home (6 关节全零); grasp_init_only=true 时 pick
        # 到达 home 后即返回 (分步测试用)。
        self.declare_parameter('grasp_init_only', True)
        self.declare_parameter('place_approach_distance', 0.10)
        self.declare_parameter('retreat_distance', 0.10)
        self.declare_parameter('planning_time', 5.0)
        # 候选位姿试错(pre-grasp/下降/提升)的单次规划时限。这些候选按顺序
        # 尝试, 不可达目标若用 5s 满超时且每个再重试一次, 单次抓取会空耗
        # 20s+。TRAC-IK 下可达目标通常 <0.5s 出解, 1.5s 足够; 全部失败后
        # 最后一次补救仍用完整 planning_time。
        self.declare_parameter('explore_planning_time', 1.5)
        self.declare_parameter('sim_duration', 2.0)
        self.declare_parameter('apple_radius', 0.02)
        # 夹爪 gripper_controller 关节值 (rad)。
        # 夹爪关节范围 [-0.7, 0.15] rad, 驱动线性映射为 pymycobot 0..100
        # (0=全闭, 100=全开): 0.15 -> 100 全开。
        # 夹取不夹死 (value 0 是机械硬限位, 实测只能到 rad -0.683, 控制器
        # 会在限位上停滞并可能二次发力), 按用户要求夹到"张开约 1/3":
        # value ≈ 33 -> rad = -0.7 + 0.33*0.85 ≈ -0.42。
        self.declare_parameter('gripper_open_value', 0.15)
        self.declare_parameter('gripper_close_value', -0.42)
        self.declare_parameter('move_action', '/move_action')
        self.declare_parameter('gripper_action',
                               '/gripper_action_controller/gripper_cmd')

        self.sim_mode = self.get_parameter('sim_mode').value
        self.arm_group = self.get_parameter('arm_group').value
        self.arm_base_frame = self.get_parameter('arm_base_frame').value
        self.eef_link = self.get_parameter('eef_link').value
        self.world_frame = self.get_parameter('world_frame').value
        self.tcp_flange_z = float(self.get_parameter('tcp_flange_z').value)
        self.pose_x_offset = float(self.get_parameter('pose_x_offset').value)
        self.pose_y_offset = float(self.get_parameter('pose_y_offset').value)
        self.pose_z_offset = float(self.get_parameter('pose_z_offset').value)
        self.apple_fixed_z = float(self.get_parameter('apple_fixed_z').value)
        self.approach_dist = float(
            self.get_parameter('grasp_approach_distance').value)
        self.pre_grasp_inner = float(
            self.get_parameter('pre_grasp_inner').value)
        self.lift_dist = float(self.get_parameter('lift_distance').value)
        self.grasp_init_only = bool(
            self.get_parameter('grasp_init_only').value)
        self.place_approach_dist = float(
            self.get_parameter('place_approach_distance').value)
        self.retreat_dist = float(self.get_parameter('retreat_distance').value)
        self.planning_time = float(self.get_parameter('planning_time').value)
        self.explore_planning_time = float(
            self.get_parameter('explore_planning_time').value)
        self.sim_duration = float(self.get_parameter('sim_duration').value)
        self.apple_radius = float(self.get_parameter('apple_radius').value)
        self.gripper_open_value = float(
            self.get_parameter('gripper_open_value').value)
        self.gripper_close_value = float(
            self.get_parameter('gripper_close_value').value)
        move_action = self.get_parameter('move_action').value
        gripper_action = self.get_parameter('gripper_action').value

        # ---- state -------------------------------------------------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.apple_pose = None
        self.place_pose = None
        # 上一次抓取成功的法兰朝向 (x,y,z,w 元组), 下次优先尝试,
        # 省掉臂展极限处对必然不可达朝向的规划超时。
        self._last_grasp_q = None
        # 最近一次 MoveGroup goal 的 client handle, 供 /pick_place/arm_stop
        # 急停服务取消在途规划/执行。
        self._move_goal_handle = None

        # ---- publishers --------------------------------------------------
        self.status_pub = self.create_publisher(String, '/pick_place/status', 10)
        # 机械臂固件急停 (mycobot_driver 订阅, pymycobot stop + 丢弃目标)
        self.arm_stop_pub = self.create_publisher(Empty, '/mycobot/stop', 5)

        # ---- subscribers -------------------------------------------------
        latch_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # 只接收 navigation_node 停车后发布的最终苹果位姿 (专用 latched
        # 话题)。不订阅原始 /pick_place/apple_pose: 近距离苹果出画后检测
        # 节点会持续重发陈旧的远距位姿, 覆盖精确的最终位姿。
        self.create_subscription(
            PoseStamped, '/pick_place/apple_pose_final', self._on_apple_pose,
            latch_qos)
        self.create_subscription(
            PoseStamped, '/pick_place/place_pose', self._on_place_pose, 10)

        # ---- action clients (可重入组, 服务回调阻塞期间仍能收到结果) ----
        self.cbg = ReentrantCallbackGroup()
        self.move_cli = ActionClient(
            self, MoveGroup, move_action, callback_group=self.cbg)
        self.gripper_cli = ActionClient(
            self, GripperCommand, gripper_action,
            callback_group=self.cbg)

        # ---- services ----------------------------------------------------
        self.create_service(
            Trigger, '/pick_place/pick', self._on_pick,
            callback_group=self.cbg)
        self.create_service(
            Trigger, '/pick_place/place', self._on_place,
            callback_group=self.cbg)
        self.create_service(
            Trigger, '/pick_place/arm_home', self._on_home,
            callback_group=self.cbg)
        self.create_service(
            Trigger, '/pick_place/arm_stop', self._on_arm_stop,
            callback_group=self.cbg)

        self.get_logger().info(
            f'{node_name} started (sim_mode={self.sim_mode})')

    # ------------------------------------------------------------------
    def _publish_status(self, text: str):
        msg = String(); msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def _on_apple_pose(self, msg: PoseStamped):
        self.apple_pose = msg

    def _on_place_pose(self, msg: PoseStamped):
        self.place_pose = msg

    # ------------------------------------------------------------------
    def _to_frame(self, pose: PoseStamped, frame: str) -> PoseStamped:
        if pose.header.frame_id == frame:
            return pose
        t = self.tf_buffer.lookup_transform(
            frame, pose.header.frame_id, rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=2.0))
        return do_transform_pose_stamped(pose, t)

    def _to_base_link(self, pose: PoseStamped) -> PoseStamped:
        return self._to_frame(pose, self.world_frame)

    def _wait_flange_settled(self, target: PoseStamped,
                              pos_tol: float = 0.012,
                              stationary: float = 0.002,
                              timeout: float = 6.0) -> bool:
        """等待法兰物理到位且静止后才返回 True。

        MoveGroup 报成功只代表控制器轨迹走完; 驱动侧把 50Hz 插值点合并后用
        阻塞 sync_send_angles 下发, 固件最后一段运动可能滞后 1~2s。若此时
        立即闭合夹爪, 会在"还在下降"时闭合。这里用 /joint_states 经
        robot_state_publisher 发布的真实 link6_flange 位姿做双重判定:
          1) 与目标位置误差 < pos_tol (到位);
          2) 连续两次读数位移 < stationary (确实停住)。
        """
        frame = target.header.frame_id
        goal_p = target.pose.position
        deadline = time.time() + timeout
        prev = None
        last_err = None
        while time.time() < deadline:
            try:
                tf_st = self.tf_buffer.lookup_transform(
                    frame, self.eef_link, rclpy.time.Time())
            except Exception:
                time.sleep(0.05)
                continue
            p = tf_st.transform.translation
            err = math.sqrt((p.x - goal_p.x) ** 2 +
                            (p.y - goal_p.y) ** 2 +
                            (p.z - goal_p.z) ** 2)
            last_err = err
            if err < pos_tol and prev is not None:
                drift = math.sqrt((p.x - prev[0]) ** 2 +
                                  (p.y - prev[1]) ** 2 +
                                  (p.z - prev[2]) ** 2)
                if drift < stationary:
                    return True
            prev = (p.x, p.y, p.z)
            time.sleep(0.05)
        last_xyz = None
        # 重新取一次实际位姿用于诊断输出
        try:
            tf_st = self.tf_buffer.lookup_transform(
                frame, self.eef_link, rclpy.time.Time())
            t = tf_st.transform.translation
            last_xyz = (t.x, t.y, t.z)
        except Exception:
            pass
        self.get_logger().warn(
            f'Flange not settled at target within {timeout:.1f}s '
            f'(err={last_err if last_err is not None else -1:.3f}m); '
            f'goal=({goal_p.x:.3f},{goal_p.y:.3f},{goal_p.z:.3f}) '
            f'actual={tuple(round(v, 3) for v in last_xyz) if last_xyz else None}')
        return False

    def _apply_calibration_offset(self, pose: PoseStamped) -> PoseStamped:
        """在 world_frame(base_link) 系修正检测位姿。

        两部分:
        1. 苹果半径: 深度测到的是苹果近侧表面到相机的距离, 得到的是表面点;
           夹爪闭合应对准苹果中心, 相机近似沿 base_link x 前向安装, 故沿
           车体前向 x 再补一个 apple_radius (表面 -> 中心)。
        2. pose_x/y_offset: 手眼/深度聚类的经验标定残差。
        3. apple_fixed_z>=0 时 z 直接取该固定高度 (现场苹果高度恒定)。
        """
        p = self._to_frame(pose, self.world_frame)
        p.pose.position.x += self.apple_radius + self.pose_x_offset
        p.pose.position.y += self.pose_y_offset
        if self.apple_fixed_z >= 0.0:
            # 测试现场苹果高度恒定: 直接采用用户指定的固定高度,
            # 忽略不可靠的深度 z 与 pose_z_offset。
            p.pose.position.z = self.apple_fixed_z
        else:
            p.pose.position.z += self.pose_z_offset
        return p

    # ------------------------------------------------------------------
    def _send_move_goal(self, goal) -> bool:
        """发送 MoveGroup goal 并等待执行完成。"""
        if not self.move_cli.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('move_action server not available')
            return False
        gh_fut = self.move_cli.send_goal_async(goal)
        if not self._spin_until(gh_fut, 10.0):
            return False
        gh = gh_fut.result()
        if not gh.accepted:
            self.get_logger().warn('Move goal rejected')
            return False
        # 记录 handle 供 /pick_place/arm_stop 取消在途 goal
        self._move_goal_handle = gh
        res_fut = gh.get_result_async()
        if not self._spin_until(res_fut, 60.0):
            return False
        res = res_fut.result().result
        # MoveItErrorCodes.SUCCESS == 1
        ok = res.error_code.val == 1
        if ok:
            # 动作成功后短暂让出: 等轨迹尾段物理执行 + 串口 worker 收尾
            # (去重后到位即不再发指令, 无需 0.5s)。闭合前另有专门的
            # _wait_flange_settled 到位静止判定, 不靠这个固定延时。
            time.sleep(0.2)
        return ok

    @staticmethod
    def _spin_until(fut, timeout: float) -> bool:
        """等待 future (executor 在其他线程上驱动)。"""
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout:
            time.sleep(0.02)
        return fut.done()

    # ------------------------------------------------------------------
    def _move_to_pose(self, pose: PoseStamped,
                      planning_time_override: float = None,
                      ori_tol: float = 0.01,
                      vel_scale: float = 0.3,
                      retries: int = 1) -> bool:
        """机械臂末端 (link6_flange) 到指定位姿。

        ori_tol: 姿态容差 [rad]。臂展极限附近的抓取点用紧容差 (0.01)
        IK 采不到解 (Unable to sample any valid states for goal tree),
        俯抓类动作放宽到 ~0.15 rad (≈8.6°, 仍基本竖直) 即可大幅提高
        可达性。
        vel_scale: 速度/加速度缩放 (0~1)。下降接近苹果等精细动作用
        小值 (0.1) 缓慢运动, 避免冲撞到苹果。
        """
        p = pose.pose.position
        o = pose.pose.orientation

        pc = PositionConstraint()
        pc.header.frame_id = pose.header.frame_id
        pc.link_name = self.eef_link
        pc.weight = 1.0
        # Jazzy: PositionConstraint 用 constraint_region (BoundingVolume),
        # 旧的 primitives/primitive_poses 字段已移除。
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.01, 0.01, 0.01]
        from geometry_msgs.msg import Pose as GPose
        cpose = GPose()
        cpose.position.x = p.x
        cpose.position.y = p.y
        cpose.position.z = p.z
        cpose.orientation.w = 1.0
        bv = BoundingVolume()
        bv.primitives = [box]
        bv.primitive_poses = [cpose]
        pc.constraint_region = bv

        oc = OrientationConstraint()
        oc.header.frame_id = pose.header.frame_id
        oc.link_name = self.eef_link
        oc.orientation = o
        oc.absolute_x_axis_tolerance = ori_tol
        oc.absolute_y_axis_tolerance = ori_tol
        oc.absolute_z_axis_tolerance = ori_tol
        oc.weight = 1.0

        cons = Constraints()
        cons.position_constraints = [pc]
        cons.orientation_constraints = [oc]

        req = MotionPlanRequest()
        req.group_name = self.arm_group
        req.num_planning_attempts = 10
        req.allowed_planning_time = (
            planning_time_override
            if planning_time_override is not None else self.planning_time)
        req.max_velocity_scaling_factor = vel_scale
        req.max_acceleration_scaling_factor = vel_scale
        req.goal_constraints = [cons]

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False
        # retries=1: 失败重试一次, 覆盖 OMPL 随机规划失败, 以及夹持负载下
        # 关节轻微下垂导致的 "start point deviates" (CONTROL_FAILED)
        # 瞬态错误。候选试错循环传 retries=0, 不可达目标只花一个短规划
        # 时限即切换下一候选, 避免每个候选都翻倍烧满超时。
        for attempt in range(retries + 1):
            if self._send_move_goal(goal):
                return True
            if attempt < retries:
                self.get_logger().warn('Move failed, retrying once ...')
                time.sleep(0.5)
        return False

    def _move_to_joint_values(self, values) -> bool:
        """机械臂到指定 6 关节角。"""
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
        # 必须显式设置 (>0): 缺省 0 会导致 TOTG 回退 1.0 且 Ruckig
        # 轨迹平滑失败 (Ruckig error -100), 整个规划返回 FAILURE。
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3
        req.goal_constraints = [cons]
        # start state 用当前状态
        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False
        return self._send_move_goal(goal)

    def _move_gripper(self, value: float) -> bool:
        if not self.gripper_cli.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('gripper action server not available')
            return False
        g = GripperCommand.Goal()
        g.command.position = value
        g.command.max_effort = 50.0
        gh_fut = self.gripper_cli.send_goal_async(g)
        if not self._spin_until(gh_fut, 10.0):
            return False
        gh = gh_fut.result()
        if not gh.accepted:
            return False
        res_fut = gh.get_result_async()
        if not self._spin_until(res_fut, 30.0):
            return False
        result = res_fut.result().result
        # 正常到位, 或夹住苹果后停滞 (到不了目标位置) 都算成功。
        return bool(result.reached_goal or result.stalled)

    # ------------------------------------------------------------------
    @staticmethod
    def _with(src: PoseStamped, quat: Quaternion,
              z_extra: float = 0.0) -> PoseStamped:
        out = PoseStamped()
        out.header = src.header
        out.pose.position.x = src.pose.position.x
        out.pose.position.y = src.pose.position.y
        out.pose.position.z = src.pose.position.z + z_extra
        out.pose.orientation = quat
        return out

    def _flange_target(self, src: PoseStamped, quat: Quaternion,
                       z_extra: float = 0.0,
                       y_inner: float = 0.0) -> PoseStamped:
        """物体中心位姿 -> link6_flange 目标位姿。

        俯抓时法兰 z 轴竖直朝下, 夹持中心在法兰 +z (tcp_flange_z),
        故法兰需在物体上方 tcp_flange_z; z_extra 为相对物体的额外高度
        (pre-grasp / lift); y_inner 为 link1 y 方向(机械臂前方)向内收
        的距离, 用于在臂展极限附近保证目标可达。
        """
        out = self._with(src, quat, z_extra + self.tcp_flange_z)
        out.pose.position.y -= y_inner
        return out

    def _do_pick(self) -> bool:
        if self.apple_pose is None:
            self.get_logger().error('No apple pose received')
            return False

        if self.sim_mode:
            self.get_logger().info('[SIM] Picking apple ...')
            time.sleep(self.sim_duration)
            self.get_logger().info('[SIM] Pick succeeded')
            return True

        # 1) base_link 系加标定偏移, 再转到机械臂 link1 坐标系。
        #    注意: link1->link6_flange 是动态的(随关节角变化), 必须在
        #    机械臂还没动(home/初始位姿之前)时完成 base_link->link1
        #    的变换, 否则会把当前臂姿态带入 apple 的 link1 坐标导致错乱。
        apple_base = self._apply_calibration_offset(self.apple_pose)
        apple = self._to_frame(apple_base, self.arm_base_frame)
        self.get_logger().info(
            f'Apple in {self.arm_base_frame}: '
            f'x={apple.pose.position.x:.3f} y={apple.pose.position.y:.3f} '
            f'z={apple.pose.position.z:.3f} '
            f'(x fix: radius {self.apple_radius} + calib '
            f'{self.pose_x_offset}; calib y={self.pose_y_offset}; '
            f'z={"fixed %.3f" % self.apple_fixed_z if self.apple_fixed_z>=0 else "detect+%.3f" % self.pose_z_offset}; '
            f'TCP {self.tcp_flange_z})')

        # 0) 回 home 与夹爪张开并行执行 (臂/爪是独立资源, 驱动 worker 会
        #    在串口上自动排队)。旧实现串行: 先回 home (~0.5-3s) 再等张爪
        #    (~1.5s), 并行后总耗时取两者最大值。回 home (6 关节全零) 是
        #    每次抓取的统一初始姿态; 夹爪在向苹果运动前已开到最大。
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

        # ---- 俯抓姿态: link6_flange z 轴竖直朝下 (两个等价翻转候选) ----
        candidates = []
        q = Quaternion(); q.y = 1.0; q.w = 0.0
        candidates.append(q)
        q = Quaternion(); q.x = 1.0; q.w = 0.0
        candidates.append(q)
        # 上次成功的朝向排最前 (同一现场两次抓取构型几乎相同, 实测两次
        # 都是 q=y 成功、q=x 不可达)。
        if self._last_grasp_q is not None:
            candidates.sort(
                key=lambda c: 0 if (c.x, c.y, c.z, c.w) ==
                self._last_grasp_q else 1)
        # pre-grasp 候选顺序: 实测苹果在 link1 y≈0.27 (臂展极限), 正上方
        # inner=0 必然不可达, 配置的 pre_grasp_inner(0.06) 才可达 —— 配置
        # 了非零内收量时先试内收位姿, 省掉两次 ~1.5s 的必然失败; 配置为
        # 0 则保持"先正上方"。短规划时限 + 不逐个重试。
        grasp_ori_tol = 0.15
        inners = ([self.pre_grasp_inner, 0.0]
                  if self.pre_grasp_inner > 1e-6 else [0.0])
        chosen = None
        chosen_inner = None
        for inner in inners:
            for q in candidates:
                if self._move_to_pose(
                        self._flange_target(
                            apple, q, self.approach_dist, inner),
                        planning_time_override=self.explore_planning_time,
                        ori_tol=grasp_ori_tol, retries=0):
                    chosen = q
                    chosen_inner = inner
                    break
            if chosen is not None:
                break
        if chosen is None:
            # 补救: 全部短时限候选失败时, 对内收候选做一次完整时限+重试
            # (应对 OMPL 在边界可达点偶发需要更长采样时间)。
            self.get_logger().warn(
                'Pre-grasp not found in quick attempts, '
                'one full-time retry on tucked pose ...')
            q = candidates[0]
            if self._move_to_pose(
                    self._flange_target(
                        apple, q, self.approach_dist, self.pre_grasp_inner),
                    ori_tol=grasp_ori_tol):
                chosen = q
                chosen_inner = self.pre_grasp_inner
        if chosen is None:
            self.get_logger().error('Cannot reach pre-grasp in any orientation')
            return False
        self._last_grasp_q = (chosen.x, chosen.y, chosen.z, chosen.w)

        # 1. 缓慢下降到苹果高度。优先沿用 pre-grasp 已验证可达的法兰朝向
        #    (旧实现固定从 q=y 开始试, 若 pre-grasp 是另一朝向会白烧一次
        #    规划超时)。低速 (vel 0.1) 避免冲撞到苹果; 夹爪已张到最大。
        self._publish_status('Approaching apple (down)')
        order = [chosen] + [q for q in candidates if (
            q.x, q.y, q.z, q.w) != (chosen.x, chosen.y, chosen.z, chosen.w)]
        reached = False
        grasp_target = None
        for q in order:
            tgt = self._flange_target(apple, q)
            tp = tgt.pose.position
            self.get_logger().info(
                f'Descend target in {tgt.header.frame_id}: '
                f'({tp.x:.3f},{tp.y:.3f},{tp.z:.3f})')
            if self._move_to_pose(
                    tgt,
                    planning_time_override=self.explore_planning_time,
                    ori_tol=grasp_ori_tol, vel_scale=0.1, retries=0):
                chosen = q
                grasp_target = tgt
                reached = True
                break
        if not reached:
            # 完整时限 + 重试兜底一次。
            self.get_logger().warn(
                'Grasp descend not found in quick attempts, '
                'one full-time retry ...')
            tgt = self._flange_target(apple, chosen)
            if self._move_to_pose(
                    tgt, ori_tol=grasp_ori_tol, vel_scale=0.1):
                grasp_target = tgt
                reached = True
        if not reached:
            self.get_logger().error('Cannot reach apple in any orientation')
            return False
        self._last_grasp_q = (chosen.x, chosen.y, chosen.z, chosen.w)

        # 1.5) 关键: MoveGroup 报成功不等于物理到位 (驱动阻塞下发, 固件最后
        #      一段可能滞后 1~2s)。必须等法兰真实到位且静止, 未到位则补发
        #      一次目标, 然后才允许闭合 —— 否则会在下降过程中就闭合夹爪。
        self._publish_status('Waiting arm to settle at apple')
        if not self._wait_flange_settled(grasp_target):
            self._publish_status('Re-commanding grasp pose')
            if not self._move_to_pose(
                    grasp_target, ori_tol=grasp_ori_tol, vel_scale=0.1):
                return False
            if not self._wait_flange_settled(grasp_target):
                return False

        # 2. 闭合夹爪到夹取位置 (gripper_close_value ≈ 张开 1/3,
        #    pymycobot value ≈ 33), 不夹到硬限位。自适应夹爪无真实位置
        #    反馈, get_gripper_value 返回指令目标值, action 会立即报成功,
        #    机械上手指走完 100->33 约需 1.5s (speed=30), 额外等 2s 确认
        #    夹住苹果后才提起。
        self._publish_status('Closing gripper')
        if not self._move_gripper(self.gripper_close_value):
            return False
        time.sleep(2.0)

        # 4. 提起: 分两段, 避免 OMPL 自由路径(关节空间弧线, 非竖直直线)
        #    起始段先下沉再上升 —— 真机曾观察到"夹住后先下降一点点", 夹着
        #    苹果下探有磕碰桌面风险。
        #    阶段 A: 纯竖直小幅提升(inner=0), 让苹果先垂直安全离地;
        #    阶段 B: 再到"升高+内收"运输位姿, 此时即使弧线有轻微下探,
        #    苹果已有离地余量。
        self._publish_status('Lifting apple')
        clear_h = 0.0
        for h in (0.08, 0.05, 0.03):
            if self._move_to_pose(
                    self._flange_target(apple, chosen, h, 0.0),
                    planning_time_override=self.explore_planning_time,
                    ori_tol=grasp_ori_tol, vel_scale=0.15, retries=0):
                clear_h = h
                self.get_logger().info(
                    f'Initial vertical lift: {h:.2f}m (apple clear)')
                break

        # 阶段 B 候选: 苹果在臂展极限附近 (link1 y≈0.27) 时纯竖直不可达,
        # 必须配合适量内收; 内收上限约 0.12m, 过大(曾用 0.22)会让夹爪上
        # 未建模的苹果扫撞机械臂本体。阶段 A 未成功时才把纯竖直高目标放
        # 回候选首位兜底。
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
                    self._flange_target(apple, chosen, h, inner),
                    planning_time_override=self.explore_planning_time,
                    ori_tol=grasp_ori_tol, retries=0):
                self.get_logger().info(
                    f'Lift succeeded: height={h:.2f}m inner={inner:.2f}m')
                lifted = True
                break
        if not lifted:
            # 完整时限 + 重试对最稳妥的内收提升位姿兜底一次。
            self.get_logger().warn(
                'Lift not found in quick attempts, one full-time retry ...')
            h, inner = 0.15, 0.10
            if self._move_to_pose(
                    self._flange_target(apple, chosen, h, inner),
                    ori_tol=grasp_ori_tol):
                self.get_logger().info(
                    f'Lift succeeded (fallback): height={h:.2f}m '
                    f'inner={inner:.2f}m')
                lifted = True
        if not lifted:
            self.get_logger().error('Cannot lift to any tested pose')
            return False

        # 5. 回程前先把机械臂收回 home (夹爪保持夹取开合度, 不松爪)。
        #    用户要求: 夹爪/机械臂回收必须在小车返回起点之前完成, 避免
        #    苹果举在臂展极限处随车移动造成晃动、碰撞或掉落。
        self._publish_status('Retracting arm to home before driving back')
        if not self._move_to_joint_values([0.0] * 6):
            self.get_logger().error('Cannot retract arm to home')
            return False
        # MoveGroup 报成功后固件运动略有滞后, 等臂物理收稳再让小车走
        time.sleep(2.0)

        self.get_logger().info('Pick succeeded')
        return True

    # ------------------------------------------------------------------
    def _do_place(self) -> bool:
        if self.place_pose is None:
            self.get_logger().error('No place pose received')
            return False

        if self.sim_mode:
            self.get_logger().info('[SIM] Placing apple ...')
            time.sleep(self.sim_duration)
            self.get_logger().info('[SIM] Place succeeded')
            return True

        place = self._to_frame(self.place_pose, self.arm_base_frame)
        q = Quaternion(); q.y = 1.0; q.w = 0.0
        # 与 pick 相同的放宽姿态容差: 放置点与抓取点同区域, 同样处于
        # 臂展极限附近, 紧容差 (0.01) IK 采不到解。
        place_ori_tol = 0.15

        # 1. 移到 place 上方
        self._publish_status('Moving to pre-place')
        if not self._move_to_pose(
                self._flange_target(place, q, self.place_approach_dist),
                ori_tol=place_ori_tol):
            return False
        # 2. 下降 (低速)
        self._publish_status('Lowering apple')
        if not self._move_to_pose(self._flange_target(place, q),
                                  ori_tol=place_ori_tol, vel_scale=0.1):
            return False
        # 3. 张开
        self._publish_status('Opening gripper (release)')
        if not self._move_gripper(self.gripper_open_value):
            return False
        # 4. 上抬
        self._publish_status('Retreating')
        if not self._move_to_pose(
                self._flange_target(place, q, self.retreat_dist),
                ori_tol=place_ori_tol):
            return False

        self.get_logger().info('Place succeeded')
        return True

    # ------------------------------------------------------------------
    def _on_pick(self, request, response):
        ok = self._do_pick()
        response.success = ok
        response.message = 'Pick succeeded' if ok else 'Pick failed'
        return response

    def _on_place(self, request, response):
        ok = self._do_place()
        response.success = ok
        response.message = 'Place succeeded' if ok else 'Place failed'
        return response

    def _on_home(self, request, response):
        if self.sim_mode:
            time.sleep(self.sim_duration)
            response.success = True
            response.message = '[SIM] Arm home'
            return response
        ok = self._move_to_joint_values([0.0] * 6)
        response.success = ok
        response.message = 'Arm home' if ok else 'Arm home failed'
        return response

    def _on_arm_stop(self, request, response):
        """机械臂急停。

        两层停止:
        1) 取消在途 MoveGroup goal —— 不仅停下当前规划/执行, 也让正在
           _send_move_goal 中等待结果的抓取流程拿到取消结果 (非 SUCCESS)
           而中止, 不再继续后续下降/闭合/提升动作;
        2) 发 /mycobot/stop —— MoveGroup goal 取消后固件仍可能有 1~2s
           残留运动, 由 mycobot_driver 调 pymycobot stop() 立即刹停。
        """
        if self.sim_mode:
            response.success = True
            response.message = '[SIM] Arm stop'
            return response

        cancelled = False
        gh = self._move_goal_handle
        if gh is not None and gh.status in (
                GoalStatus.STATUS_ACCEPTED,
                GoalStatus.STATUS_EXECUTING,
                GoalStatus.STATUS_CANCELING):
            try:
                cancel_fut = self.move_cli.cancel_goal_async(gh)
                self._spin_until(cancel_fut, 2.0)
                cancelled = cancel_fut.done()
            except Exception as e:
                self.get_logger().warn(f'Cancel move goal failed: {e}')

        self.arm_stop_pub.publish(Empty())
        self._publish_status(
            f'Arm STOP requested (move goal cancelled={cancelled}, '
            f'firmware stop sent)')
        response.success = True
        response.message = 'Arm stop sent'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GraspNode()
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
