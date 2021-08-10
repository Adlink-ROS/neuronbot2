#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import sys
from typing import Tuple
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
from osrf_pycommon.terminal_color import ansi
from xacro4sdf.xacro4sdf import XMLMacro

def to_quaternion(
        roll: float,
        pitch: float,
        yaw: float) -> Tuple[float, float, float, float]:
    """Convert Euler to Quaternion."""
    c_r = np.cos(roll * 0.5)
    s_r = np.sin(roll * 0.5)
    c_p = np.cos(pitch * 0.5)
    s_p = np.sin(pitch * 0.5)
    c_y = np.cos(yaw * 0.5)
    s_y = np.sin(yaw * 0.5)

    x = s_r * c_p * c_y - c_r * s_p * s_y
    y = c_r * s_p * c_y + s_r * c_p * s_y
    z = c_r * c_p * s_y - s_r * s_p * c_y
    w = c_r * c_p * c_y + s_r * s_p * s_y

    return (x, y, z, w)

def gen_sdf_from_xmacro(sdf_path, namespace):
    pkg_neuronbot2_gazebo = get_package_share_directory('neuronbot2_gazebo')
    sdf_target_path = os.path.join(pkg_neuronbot2_gazebo, 'models', 'neuronbot2', f"model_{namespace}.sdf")

    xmacro=XMLMacro()
    xmacro.set_xml_file(sdf_path)
    custom_property={"robot_namespace":namespace+"/"}
    xmacro.generate(custom_property)
    xmacro.to_file(sdf_target_path,banner_info="spawn_nb2.py")
    return sdf_target_path


def main(args=None):
    print(ansi("yellow"), sys.argv, ansi("reset"))

    rclpy.init(args=args)
    node = rclpy.create_node('minimal_client')
    cli = node.create_client(SpawnEntity, '/spawn_entity')

    # pkg_neuronbot2_description = get_package_share_directory('neuronbot2_description')
    # urdf = os.path.join(pkg_neuronbot2_description, 'urdf/', 'neuronbot2.urdf')
    sdf_xmacro = os.path.join(get_package_share_directory('neuronbot2_gazebo'), 'models', 'neuronbot2', 'model.sdf.xmacro')
    sdf = gen_sdf_from_xmacro(sdf_xmacro, sys.argv[2])

    content = ""
    if sdf is not None:
        with open(sdf, 'r') as content_file:
            content = content_file.read()

    req = SpawnEntity.Request()
    req.name = sys.argv[1]
    req.xml = content
    req.robot_namespace = sys.argv[2]
    req.reference_frame = "world"
    req.initial_pose.position.x = float(sys.argv[3])
    req.initial_pose.position.y = float(sys.argv[4])
    req.initial_pose.position.z = float(sys.argv[5])

    quaternion = to_quaternion(float(sys.argv[6]), float(sys.argv[7]), float(sys.argv[8]))
    req.initial_pose.orientation.x = quaternion[0]
    req.initial_pose.orientation.y = quaternion[1]
    req.initial_pose.orientation.z = quaternion[2]
    req.initial_pose.orientation.w = quaternion[3]

    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info(
            'Result ' + str(future.result().success) + " " + future.result().status_message)
    else:
        node.get_logger().info('Service call failed %r' % (future.exception(),))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
