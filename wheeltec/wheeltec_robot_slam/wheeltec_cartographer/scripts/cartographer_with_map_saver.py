#!/usr/bin/env python3
"""运行 cartographer_node，并在其退出后完成 pbstream 到 PGM/YAML 的转换。"""

import argparse
import os
import signal
import subprocess
import sys

from ament_index_python.packages import get_package_prefix

current_process = None


def forward_signal(signum, _frame):
    if current_process is not None and current_process.poll() is None:
        current_process.send_signal(signum)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--configuration-directory', required=True)
    parser.add_argument('--configuration-basename', required=True)
    parser.add_argument('--save-state-filename', required=True)
    parser.add_argument('--map-filestem', required=True)
    parser.add_argument('--resolution', default='0.05')
    parser.add_argument('--use-sim-time', default='false')
    args, _unknown_ros_args = parser.parse_known_args()

    os.makedirs(os.path.dirname(os.path.abspath(args.save_state_filename)),
                exist_ok=True)
    os.makedirs(os.path.dirname(os.path.abspath(args.map_filestem)),
                exist_ok=True)

    signal.signal(signal.SIGINT, forward_signal)
    signal.signal(signal.SIGTERM, forward_signal)

    cartographer_ros_dir = os.path.join(
        get_package_prefix('cartographer_ros'), 'lib', 'cartographer_ros')

    global current_process
    current_process = subprocess.Popen([
        os.path.join(cartographer_ros_dir, 'cartographer_node'),
        '-configuration_directory', args.configuration_directory,
        '-configuration_basename', args.configuration_basename,
        '-save_state_filename', args.save_state_filename,
        '--ros-args',
        '-r', 'odom:=odom_combined',
        '-r', 'imu:=/mobile_base/sensors/imu_data',
        '-p', f'use_sim_time:={args.use_sim_time}',
    ])
    node_return_code = current_process.wait()

    if not os.path.isfile(args.save_state_filename):
        print(f'错误：pbstream 未生成：{args.save_state_filename}',
              file=sys.stderr, flush=True)
        return node_return_code if node_return_code != 0 else 1

    print(f'pbstream 已保存，正在生成 PGM/YAML：{args.map_filestem}',
          flush=True)
    current_process = subprocess.Popen([
        os.path.join(cartographer_ros_dir, 'cartographer_pbstream_to_ros_map'),
        '-pbstream_filename', args.save_state_filename,
        '-map_filestem', args.map_filestem,
        '-resolution', args.resolution,
    ])
    converter_return_code = current_process.wait()

    if converter_return_code == 0:
        print(f'地图保存完成：{args.map_filestem}.pgm / '
              f'{args.map_filestem}.yaml', flush=True)
    else:
        print(f'错误：PGM/YAML 转换失败，返回码 {converter_return_code}',
              file=sys.stderr, flush=True)
    return converter_return_code


if __name__ == '__main__':
    sys.exit(main())
