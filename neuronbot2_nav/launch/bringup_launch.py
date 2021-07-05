# Copyright (c) 2018 Intel Corporation
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
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    # Get the launch directory
    my_nav_dir = get_package_share_directory('neuronbot2_nav')
    my_launch_dir = os.path.join(my_nav_dir, 'launch')
    my_param_dir = os.path.join(my_nav_dir, 'param')
    my_param_file = 'neuronbot_namespaced_params.yaml'
    my_bt_file = 'navigate_w_replanning_time.xml'
    my_map_dir = os.path.join(my_nav_dir, 'map')
    my_map_file = 'mememan.yaml'
    # warehouse_pkg_dir = get_package_share_directory('aws_robomaker_small_warehouse_world')
    # my_map_dir = os.path.join(warehouse_pkg_dir, 'maps', '005')
    # my_map_file = 'map.yaml'

    # Create the launch configuration variables
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_slam = LaunchConfiguration('use_slam', default='false')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    autostart = LaunchConfiguration('autostart')
    open_rviz = LaunchConfiguration('open_rviz')

    ld = LaunchDescription([
        SetEnvironmentVariable(
            'RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(my_map_dir, my_map_file),
            description='Full path to map yaml file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'use_slam',
            default_value='false',
            description='Whether run a SLAM'),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(my_param_dir, my_param_file),
            description='Full path to the ROS2 parameters file to use for all launched nodes'),

        DeclareLaunchArgument(
            'default_bt_xml_filename',
            default_value=os.path.join(my_param_dir, my_bt_file),
            description='Full path to the behavior tree xml file to use'),
        
        DeclareLaunchArgument(
            'autostart', 
            default_value='true',
            description='Automatically startup the nav2 stack'),
        
        DeclareLaunchArgument(
            'open_rviz',
            default_value='false',
            description='Launch Rviz?'),
    ])

    # SLAM does not work for now
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('neuronbot2_slam'),
                    'launch',
                    'gmapping_launch.py')),
            condition=IfCondition(use_slam),
            launch_arguments={'namespace': 'robot0',
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                            #   'params_file': params_file
                              }.items())
    )

    # Specify the actions
    NUM_OF_ROBOTS = 1
    for i in range(NUM_OF_ROBOTS):
        robot_name = f'robot{i}'

        ld.add_action(
            IncludeLaunchDescription(
                # Run Localization only when we don't use SLAM
                PythonLaunchDescriptionSource(os.path.join(my_launch_dir, 'localization_launch.py')),
                condition=UnlessCondition(use_slam),
                launch_arguments={'namespace': robot_name,
                                'initialpose_x': str(0.3 * i),
                                'map': map_yaml_file,
                                'use_sim_time': use_sim_time,
                                'autostart': autostart,
                                'params_file': params_file,
                                'use_lifecycle_mgr': 'false'}.items()))
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(my_launch_dir, 'navigation_launch.py')),
                launch_arguments={'namespace': robot_name,
                                'use_sim_time': use_sim_time,
                                'autostart': autostart,
                                'params_file': params_file,
                                'default_bt_xml_filename': default_bt_xml_filename,
                                'use_lifecycle_mgr': 'false',
                                'map_subscribe_transient_local': 'true'}.items()))
        ld.add_action(                          
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(my_launch_dir, 'rviz_view_launch.py')),
                launch_arguments={'namespace': robot_name,
                                'use_sim_time': use_sim_time,
                                'open_rviz': open_rviz,
                                'map_subscribe_transient_local': 'true'}.items()))               

    return ld
