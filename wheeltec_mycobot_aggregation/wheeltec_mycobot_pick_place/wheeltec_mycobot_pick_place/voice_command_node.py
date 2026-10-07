#!/usr/bin/env python3
"""Voice command node - multi-command dispatch framework.

Subscribes to ``/asr_command`` (std_msgs/String) and dispatches actions
defined in a command table (YAML). This makes adding new voice commands
a pure-configuration task: edit ``config/voice_commands.yaml`` and call
``/pick_place/reload_commands`` - no code changes needed.

Command table format (config/voice_commands.yaml):

    commands:
      - name: pick_apple
        patterns: ["把苹果拿过来", "把苹果带过来"]
        match_mode: exact          # exact | contains
        enabled: true
        actions:
          - type: topic            # publish a topic
            topic: /pick_place/trigger
            msg_type: std_msgs/Bool
            data: true
          - type: service          # call a std_srvs/Trigger service
            service: /some_service

Dispatch logic
--------------
1. Received text is normalized (strip whitespace).
2. For each enabled command, check every pattern:
   - exact:    text == pattern
   - contains: pattern in text
3. First matching command wins; all its actions are dispatched in order.

For ASR robustness, asr_keywords publishes the *canonical* matched
keyword, so ``exact`` against the canonical text is the default. If you
want fuzzy matching on raw sentences, use ``contains`` with a short
pattern (e.g. "苹果").

Services
--------
* ``/pick_place/trigger_manual`` (std_srvs/Trigger) - re-dispatch the
  command named by the ``manual_trigger_command`` parameter
  (default: pick_apple). Useful for testing without ASR.
* ``/pick_place/reload_commands`` (std_srvs/Trigger) - reload the
  command table from disk (no node restart needed).
* ``/pick_place/list_commands``  (std_srvs/Trigger) - list loaded
  commands in the response message.

Topics
------
* Sub: ``/asr_command`` (std_msgs/String)
* Pub: ``/pick_place/status`` (std_msgs/String)
* Pub: any topic configured in the command table.
"""

import importlib
import os
import threading

import yaml

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_srvs.srv import Trigger


def _import_msg_class(type_str: str):
    """Import a ROS message class from 'pkg/Type' or 'pkg/msg/Type'."""
    parts = [p for p in type_str.strip('/').split('/') if p]
    if len(parts) == 3 and parts[1] == 'msg':
        pkg, name = parts[0], parts[2]
    elif len(parts) == 2:
        pkg, name = parts
    else:
        raise ValueError(f'Invalid msg_type: {type_str!r}')
    mod = importlib.import_module(f'{pkg}.msg')
    return getattr(mod, name)


class VoiceCommandNode(Node):

    def __init__(self):
        super().__init__('voice_command_node')

        # ---- parameters --------------------------------------------------
        self.declare_parameter('sim_mode', True)
        self.declare_parameter('commands_file', '')
        self.declare_parameter('manual_trigger_command', 'pick_apple')

        self.sim_mode = self.get_parameter('sim_mode').value
        # 用 ament_index 解析包 share 目录: colcon 安装后本脚本位于
        # install/<pkg>/lib/python3.12/site-packages/, 相对 __file__ 推导
        # 的 ../../config 不存在; 真实位置是 share/<pkg>/config/。
        from ament_index_python.packages import get_package_share_directory
        default_file = os.path.join(
            get_package_share_directory('wheeltec_mycobot_pick_place'),
            'config', 'voice_commands.yaml')
        self.commands_file = self.get_parameter('commands_file').value \
            or default_file
        self.manual_trigger_command = \
            self.get_parameter('manual_trigger_command').value

        # ---- state -------------------------------------------------------
        self.commands = []
        self._topic_pubs = {}  # (topic, type_name) -> publisher

        # ---- publishers / subscribers ------------------------------------
        self.status_pub = self.create_publisher(String, '/pick_place/status', 10)
        self.asr_sub = self.create_subscription(
            String, '/asr_command', self._on_asr_command, 10)

        # ---- services ----------------------------------------------------
        self.trigger_srv = self.create_service(
            Trigger, '/pick_place/trigger_manual', self._on_trigger_manual)
        self.reload_srv = self.create_service(
            Trigger, '/pick_place/reload_commands', self._on_reload)
        self.list_srv = self.create_service(
            Trigger, '/pick_place/list_commands', self._on_list)

        # ---- load command table ------------------------------------------
        self._load_commands()

    # ==================================================================
    # Command table
    # ==================================================================
    def _load_commands(self):
        """Load the command table from YAML. Never raises."""
        self.commands = []
        try:
            if not os.path.exists(self.commands_file):
                self.get_logger().error(
                    f'commands_file not found: {self.commands_file}')
                return
            with open(self.commands_file, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            cmds = data.get('commands', [])
            for c in cmds:
                if not c.get('enabled', True):
                    continue
                self.commands.append({
                    'name': c.get('name', 'unnamed'),
                    'patterns': list(c.get('patterns', [])),
                    'match_mode': c.get('match_mode', 'exact'),
                    'actions': list(c.get('actions', [])),
                })
            names = ', '.join(c['name'] for c in self.commands)
            self.get_logger().info(
                f'Loaded {len(self.commands)} commands from '
                f'{self.commands_file}: [{names}]')
            self._publish_status(
                f'voice_command_node ready | {len(self.commands)} commands '
                f'loaded | sim_mode={self.sim_mode}')
        except Exception as e:
            self.get_logger().error(f'Failed to load commands: {e}')

    # ==================================================================
    # Matching & dispatch
    # ==================================================================
    def _match_command(self, text: str):
        """Return the first command matching text, or None."""
        for cmd in self.commands:
            for pattern in cmd['patterns']:
                if not pattern:
                    continue
                if cmd['match_mode'] == 'contains':
                    matched = pattern in text
                else:
                    matched = text == pattern
                if matched:
                    return cmd, pattern
        return None, None

    def _dispatch(self, cmd, matched_pattern: str):
        """Dispatch all actions of a command (in a worker thread)."""
        self.get_logger().info(
            f'Command "{cmd["name"]}" matched (pattern="{matched_pattern}"), '
            f'dispatching {len(cmd["actions"])} action(s)')
        self._publish_status(f'COMMAND matched: {cmd["name"]}')

        thread = threading.Thread(
            target=self._dispatch_actions, args=(cmd,), daemon=True)
        thread.start()

    def _dispatch_actions(self, cmd):
        for i, action in enumerate(cmd['actions']):
            try:
                self._dispatch_action(action)
            except Exception as e:
                self.get_logger().error(
                    f'Action {i} of "{cmd["name"]}" failed: {e}')

    def _dispatch_action(self, action: dict):
        a_type = action.get('type', 'topic')
        if a_type == 'topic':
            topic = action['topic']
            msg_type_str = action.get('msg_type', 'std_msgs/Bool')
            cls = _import_msg_class(msg_type_str)
            pub = self._get_publisher(topic, cls, msg_type_str)
            msg = cls()
            data = action.get('data')
            if data is not None and hasattr(msg, 'data'):
                if isinstance(data, list) and not isinstance(msg.data, list):
                    # single scalar into typed field
                    msg.data = type(msg.data)(data[0]) if data else msg.data
                else:
                    msg.data = data
            pub.publish(msg)
            self.get_logger().info(
                f'  -> topic {topic} [{msg_type_str}] data={data}')
        elif a_type == 'service':
            srv_name = action['service']
            self.get_logger().info(f'  -> service {srv_name}')
            cli = self.create_client(Trigger, srv_name)
            if not cli.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(
                    f'Service {srv_name} not available')
                return
            future = cli.call_async(Trigger.Request())
            deadline = 10.0
            import time
            t0 = time.time()
            while rclpy.ok() and not future.done() and \
                    time.time() - t0 < deadline:
                time.sleep(0.05)
            if future.done():
                try:
                    resp = future.result()
                    self.get_logger().info(
                        f'  <- {srv_name}: success={resp.success} '
                        f'msg="{resp.message}"')
                except Exception as e:
                    self.get_logger().error(
                        f'  <- {srv_name} error: {e}')
            else:
                self.get_logger().error(f'  <- {srv_name} timeout')
        else:
            raise ValueError(f'Unknown action type: {a_type}')

    def _get_publisher(self, topic: str, cls, type_name: str):
        key = (topic, type_name)
        if key not in self._topic_pubs:
            self._topic_pubs[key] = self.create_publisher(cls, topic, 10)
        return self._topic_pubs[key]

    # ==================================================================
    # Callbacks
    # ==================================================================
    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def _on_asr_command(self, msg: String):
        text = (msg.data or '').strip()
        self.get_logger().info(f'Received /asr_command: "{text}"')
        if not text:
            return
        cmd, pattern = self._match_command(text)
        if cmd is None:
            self.get_logger().info(
                f'No command matched "{text}" '
                f'(loaded: {[c["name"] for c in self.commands]})')
            return
        self._dispatch(cmd, pattern)

    # ==================================================================
    # Service handlers
    # ==================================================================
    def _find_command(self, name: str):
        for c in self.commands:
            if c['name'] == name:
                return c
        return None

    def _on_trigger_manual(self, request, response):
        cmd = self._find_command(self.manual_trigger_command)
        if cmd is None:
            response.success = False
            response.message = (
                f'Command "{self.manual_trigger_command}" not found. '
                f'Loaded: {[c["name"] for c in self.commands]}')
            return response
        self._dispatch(cmd, '<manual>')
        response.success = True
        response.message = f'Dispatched command "{cmd["name"]}"'
        return response

    def _on_reload(self, request, response):
        self._load_commands()
        response.success = True
        response.message = (
            f'Reloaded {len(self.commands)} commands: '
            f'{[c["name"] for c in self.commands]}')
        return response

    def _on_list(self, request, response):
        lines = []
        for c in self.commands:
            pats = '|'.join(c['patterns'])
            acts = '; '.join(
                f"{a.get('type', 'topic')}:{a.get('topic', a.get('service', '?'))}"
                for a in c['actions'])
            lines.append(f'{c["name"]} [{c["match_mode"]}] "{pats}" -> {acts}')
        response.success = True
        response.message = '\n'.join(lines) if lines else 'No commands loaded'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
