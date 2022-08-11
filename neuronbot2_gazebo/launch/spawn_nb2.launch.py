# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    NEURONBOT2_MODEL = 'nb2'
    model_path_prefix = os.path.join(
        get_package_share_directory('neuronbot2_gazebo'), 'models'
    )

    # Launch configuration variables specific to simulation
    use_camera = LaunchConfiguration('use_camera', default='none')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    # Declare the launch arguments
    declare_camera_position_cmd = DeclareLaunchArgument(
        'use_camera',
        default_value='none',
        description='Use camera?')

    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Specify namespace of the robot')

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Specify namespace of the robot')

    # Spawn NB2 without camera
    spawn_nb2_wo_camera_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', NEURONBOT2_MODEL,
            '-file', os.path.join(model_path_prefix, 'neuronbot2', 'model.sdf'),
            '-timeout', '300',
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01'
        ],
        condition=IfCondition(PythonExpression(['"', use_camera, '" == "none"'])),
        output='screen',
    )

    # Spawn NB2 with top camera
    spawn_nb2_w_top_camera_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', NEURONBOT2_MODEL,
            '-file', os.path.join(model_path_prefix, 'neuronbot2_w_top_camera', 'model.sdf'),
            '-timeout', '300',
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01'
        ],
        condition=IfCondition(PythonExpression(['"', use_camera, '" == "top"'])),
        output='screen',
    )

    # Spawn NB2 with front camera
    spawn_nb2_w_front_camera_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', NEURONBOT2_MODEL,
            '-file', os.path.join(model_path_prefix, 'neuronbot2_w_front_camera', 'model.sdf'),
            '-timeout', '300',
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01'
        ],
        condition=IfCondition(PythonExpression(['"', use_camera, '" == "front"'])),
        output='screen',
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_camera_position_cmd)
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)

    # Add any conditioned actions
    ld.add_action(spawn_nb2_wo_camera_cmd)
    ld.add_action(spawn_nb2_w_top_camera_cmd)
    ld.add_action(spawn_nb2_w_front_camera_cmd)

    return ld