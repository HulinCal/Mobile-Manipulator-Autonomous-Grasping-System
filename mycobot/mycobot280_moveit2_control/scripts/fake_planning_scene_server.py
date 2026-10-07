#!/usr/bin/env python3
"""
Fake GetPlanningScene server for running MTC pick & place on real hardware
WITHOUT the perception pipeline (point cloud + segmentation).

The real `get_planning_scene_server` node (from mycobot_mtc_pick_place_demo)
subscribes to a point cloud topic, runs plane + object segmentation, and
responds to the `get_planning_scene_mycobot` service of type
`mycobot_interfaces/srv/GetPlanningScene`. When no camera is connected, that
server cannot produce a response, and `mtc_node` blocks forever inside
`setupPlanningScene()` waiting for the service.

This node provides the same service but answers immediately with a single
hardcoded collision object built from this node's parameters, so that
`mtc_node` can proceed to plan the pick & place task on real hardware with a
known object pose. The object pose / dimensions / name are read from
parameters that mirror those of `mtc_node` (loaded from the same
`mtc_node_params.yaml` file), so editing that one file configures both the
fake scene and the MTC target.

Typical usage (via mycobot_280_real_pick_place.launch.py):

    ros2 launch mycobot280_moveit2_control mycobot_280_real_pick_place.launch.py \
        use_perception:=false

Service:
    get_planning_scene_mycobot (mycobot_interfaces/srv/GetPlanningScene):
        Request:  target_shape (str), target_dimensions (float64[])
        Response: scene_world (moveit_msgs/PlanningSceneWorld),
                  full_cloud (sensor_msgs/PointCloud2)  -- left empty,
                  rgb_image (sensor_msgs/Image)          -- left empty,
                  target_object_id (str),
                  support_surface_id (str)               -- left empty,
                  success (bool)
"""

import math
from typing import List

import rclpy
from rclpy.node import Node

from mycobot_interfaces.srv import GetPlanningScene
from moveit_msgs.msg import CollisionObject, PlanningSceneWorld
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


def _rpy_to_quaternion(roll: float, pitch: float, yaw: float):
    """Convert RPY (radians) to a geometry_msgs/Quaternion (xyzw)."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return (qx, qy, qz, qw)


class FakePlanningSceneServer(Node):
    def __init__(self) -> None:
        super().__init__('fake_planning_scene_server')

        # These parameters intentionally mirror those of mtc_node so that the
        # same mtc_node_params.yaml file configures both nodes.
        self.declare_parameter('object_name', 'object')
        self.declare_parameter('object_type', 'cylinder')
        self.declare_parameter('object_dimensions', [0.1, 0.0225])
        self.declare_parameter('object_pose', [0.22, 0.12, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('object_reference_frame', 'base_link')

        self.srv = self.create_service(
            GetPlanningScene,
            'get_planning_scene_mycobot',
            self._handle_request,
        )
        self.get_logger().info(
            'Fake GetPlanningScene server ready on service '
            '"get_planning_scene_mycobot". Will return ONE hardcoded '
            'collision object built from object_name/object_type/'
            'object_dimensions/object_pose parameters.')

    def _handle_request(self, request, response):
        object_name = self.get_parameter('object_name').value
        object_type = self.get_parameter('object_type').value
        dims: List[float] = list(self.get_parameter('object_dimensions').value)
        pose_arr: List[float] = list(self.get_parameter('object_pose').value)
        frame = self.get_parameter('object_reference_frame').value

        # Request also carries target_shape / target_dimensions, but we
        # prefer the node-level parameters so the operator edits only one
        # place (mtc_node_params.yaml) to configure both mtc_node and this
        # fake server.
        self.get_logger().info(
            f'Service request received: target_shape='
            f'"{request.target_shape}", dims={list(request.target_dimensions)}.'
            f' Responding with hardcoded object: name="{object_name}", '
            f'type="{object_type}", dims={dims}, pose={pose_arr}, '
            f'frame="{frame}".')

        primitive = SolidPrimitive()
        if object_type == 'box':
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [
                dims[0] if len(dims) > 0 else 0.05,
                dims[1] if len(dims) > 1 else 0.05,
                dims[2] if len(dims) > 2 else 0.05,
            ]
        else:
            # Default to cylinder for any non-box type, matching mtc_node's
            # default object_type "cylinder".
            primitive.type = SolidPrimitive.CYLINDER
            primitive.dimensions = [
                dims[0] if len(dims) > 0 else 0.1,   # CYLINDER_HEIGHT
                dims[1] if len(dims) > 1 else 0.0225,  # CYLINDER_RADIUS
            ]

        pose = Pose()
        if len(pose_arr) >= 3:
            pose.position.x = pose_arr[0]
            pose.position.y = pose_arr[1]
            pose.position.z = pose_arr[2]
        if len(pose_arr) >= 6:
            qx, qy, qz, qw = _rpy_to_quaternion(
                pose_arr[3], pose_arr[4], pose_arr[5])
            pose.orientation.x = qx
            pose.orientation.y = qy
            pose.orientation.z = qz
            pose.orientation.w = qw
        else:
            pose.orientation.w = 1.0

        co = CollisionObject()
        co.header.frame_id = frame
        co.header.stamp = self.get_clock().now().to_msg()
        co.id = object_name
        co.operation = CollisionObject.ADD
        co.primitives = [primitive]
        co.primitive_poses = [pose]

        world = PlanningSceneWorld()
        world.collision_objects = [co]

        response.scene_world = world
        # full_cloud / rgb_image are left at their default (empty) values.
        response.target_object_id = object_name
        response.support_surface_id = ''
        response.success = True

        self.get_logger().info(
            f'Responding with 1 collision object: id="{object_name}", '
            f'frame="{frame}". target_object_id="{object_name}".')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = FakePlanningSceneServer()
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
