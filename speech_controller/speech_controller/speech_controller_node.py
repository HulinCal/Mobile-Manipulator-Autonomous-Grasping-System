#!/usr/bin/env python3
"""Text-command to velocity / gripper controller.

Subscribes to ``/asr_command`` (std_msgs/msg/String) and:

* publishes ``geometry_msgs/Twist`` velocity commands on ``/cmd_vel`` for
  base-motion commands (e.g. 小车向前);
* sends ``control_msgs/GripperCommand`` action goals to the robotic-arm
  gripper action server (e.g. 夹爪松开 / 夹爪闭合);
* calls ``std_srvs/Trigger`` services for whole-arm commands
  (e.g. 机械臂停止 / 机械臂回家, served by grasp_node).

The mapping from each text command to its velocity or gripper position is
loaded from a YAML file (default: config/commands.yaml), so commands can be
changed without modifying code.
"""

from functools import partial
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from control_msgs.action import GripperCommand
import yaml


def load_command_velocities(path: Path) -> dict:
    """Load the command -> Twist mapping from a YAML file.

    Returns a dict: command text -> geometry_msgs.msg.Twist.
    """
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
    if not data or 'commands' not in data:
        raise RuntimeError(f'Invalid commands config: missing "commands" in {path}')

    commands = {}
    for name, spec in data['commands'].items():
        twist = Twist()
        spec = spec or {}
        linear = spec.get('linear') or {}
        angular = spec.get('angular') or {}
        twist.linear.x = float(linear.get('x', 0.0))
        twist.linear.y = float(linear.get('y', 0.0))
        twist.linear.z = float(linear.get('z', 0.0))
        twist.angular.x = float(angular.get('x', 0.0))
        twist.angular.y = float(angular.get('y', 0.0))
        twist.angular.z = float(angular.get('z', 0.0))
        commands[str(name).strip()] = twist

    if not commands:
        raise RuntimeError(f'No valid commands found in {path}')
    return commands


def load_gripper_commands(path: Path) -> dict:
    """Load the command -> gripper position (rad) mapping from a YAML file.

    The ``gripper_commands`` section is optional; an empty dict is returned
    when it is absent.
    """
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}

    gripper_commands = {}
    for name, position in (data.get('gripper_commands') or {}).items():
        gripper_commands[str(name).strip()] = float(position)
    return gripper_commands


def load_service_commands(path: Path) -> dict:
    """Load the command -> Trigger service name mapping from a YAML file.

    The ``service_commands`` section is optional; an empty dict is returned
    when it is absent.
    """
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}

    service_commands = {}
    for name, service_name in (data.get('service_commands') or {}).items():
        service_commands[str(name).strip()] = str(service_name).strip()
    return service_commands


def default_commands_file() -> Path:
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('speech_controller')) / \
        'config' / 'commands.yaml'


class SpeechControllerNode(Node):

    def __init__(self):
        super().__init__('speech_controller')

        self.declare_parameter('commands_file', '')
        self.declare_parameter('command_topic', '/asr_command')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('gripper_action',
                               '/gripper_action_controller/gripper_cmd')
        self.declare_parameter('gripper_max_effort', 50.0)

        commands_param = str(self.get_parameter('commands_file').value)
        commands_path = Path(commands_param) if commands_param \
            else default_commands_file()
        if not commands_path.is_file():
            raise RuntimeError(f'Commands file not found: {commands_path}')

        self.command_velocities = load_command_velocities(commands_path)
        self.gripper_commands = load_gripper_commands(commands_path)
        self.service_commands = load_service_commands(commands_path)

        command_topic = str(self.get_parameter('command_topic').value)
        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.gripper_action_name = str(
            self.get_parameter('gripper_action').value)
        self.gripper_max_effort = float(
            self.get_parameter('gripper_max_effort').value)

        self.publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.subscription = self.create_subscription(
            String, command_topic, self._on_command, 10)

        # Gripper action client (same tested interface as grasp_node).
        self.gripper_cli = ActionClient(
            self, GripperCommand, self.gripper_action_name)

        # Whole-arm Trigger service clients (e.g. /pick_place/arm_stop),
        # one client per unique service name.
        self.service_clients = {}
        for service_name in sorted(set(self.service_commands.values())):
            self.service_clients[service_name] = self.create_client(
                Trigger, service_name)

        # Publish a zero command at startup for safety.
        self.publisher.publish(Twist())

        self.get_logger().info(
            f'Loaded {len(self.command_velocities)} base commands, '
            f'{len(self.gripper_commands)} gripper commands and '
            f'{len(self.service_commands)} arm service commands from '
            f'{commands_path}')
        self.get_logger().info(
            f'Listening on {command_topic}, publishing Twist on '
            f'{cmd_vel_topic}')
        self.get_logger().info(
            'Base commands: ' + ', '.join(self.command_velocities))
        if self.gripper_commands:
            self.get_logger().info(
                f'Gripper commands (action {self.gripper_action_name}): '
                + ', '.join(f'{c} -> {p:+.3f} rad'
                            for c, p in self.gripper_commands.items()))
        else:
            self.get_logger().info('No gripper commands configured.')
        if self.service_commands:
            self.get_logger().info(
                'Arm service commands: '
                + ', '.join(f'{c} -> {s}'
                            for c, s in self.service_commands.items()))
        else:
            self.get_logger().info('No arm service commands configured.')

    def _on_command(self, msg: String):
        command = msg.data.strip()

        twist = self.command_velocities.get(command)
        if twist is not None:
            self.publisher.publish(twist)
            self.get_logger().info(
                f'Command "{command}" -> linear.x={twist.linear.x:.3f}, '
                f'angular.z={twist.angular.z:.3f}')
            return

        gripper_position = self.gripper_commands.get(command)
        if gripper_position is not None:
            self._send_gripper_goal(command, gripper_position)
            return

        service_name = self.service_commands.get(command)
        if service_name is not None:
            self._call_service_command(command, service_name)
            return

        self.get_logger().warn(f'Unknown command: "{command}"')

    # ------------------------------------------------------------------
    def _send_gripper_goal(self, command: str, position: float):
        """Send a GripperCommand goal without blocking the executor.

        The whole send-goal -> get-result chain runs via future done
        callbacks processed in the node's executor thread.
        """
        if not self.gripper_cli.server_is_ready():
            self.get_logger().warn(
                f'Command "{command}" ignored: gripper action server '
                f'{self.gripper_action_name} is not available '
                f'(is the arm stack running?).')
            return

        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = self.gripper_max_effort

        goal_future = self.gripper_cli.send_goal_async(goal)
        goal_future.add_done_callback(
            partial(self._on_gripper_goal_accepted, command, position))

    def _on_gripper_goal_accepted(self, command: str, position: float,
                                  future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(
                f'Gripper goal for "{command}" (position={position:+.3f} rad) '
                f'was rejected by {self.gripper_action_name}')
            return
        self.get_logger().info(
            f'Gripper "{command}" -> position={position:+.3f} rad, '
            f'goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            partial(self._on_gripper_result, command))

    def _on_gripper_result(self, command: str, future):
        result = future.result().result
        # reached_goal: arrived at commanded position; stalled: made
        # progress then blocked (e.g. gripping an apple). Both are the
        # success cases used by grasp_node.
        if result.reached_goal or result.stalled:
            self.get_logger().info(
                f'Gripper "{command}" done '
                f'(reached_goal={result.reached_goal}, stalled={result.stalled}, '
                f'position={result.position:+.3f}, effort={result.effort:.2f})')
        else:
            self.get_logger().warn(
                f'Gripper "{command}" finished without reaching the goal '
                f'(position={result.position:+.3f}, effort={result.effort:.2f})')

    # ------------------------------------------------------------------
    def _call_service_command(self, command: str, service_name: str):
        """Call a std_srvs/Trigger service without blocking the executor."""
        client = self.service_clients.get(service_name)
        if client is None or not client.service_is_ready():
            self.get_logger().warn(
                f'Command "{command}" ignored: service {service_name} is not '
                f'available (is the arm stack running?).')
            return

        request = Trigger.Request()
        future = client.call_async(request)
        future.add_done_callback(
            partial(self._on_service_result, command, service_name))

    def _on_service_result(self, command: str, service_name: str, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(
                f'Service {service_name} for "{command}" failed: {e}')
            return
        if response.success:
            self.get_logger().info(
                f'Arm "{command}" -> {service_name} done: {response.message}')
        else:
            self.get_logger().warn(
                f'Arm "{command}" -> {service_name} returned failure: '
                f'{response.message}')


def main(args=None):
    rclpy.init(args=args)
    node = SpeechControllerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
