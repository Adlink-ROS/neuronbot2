#!/usr/bin/python3
# -*- coding: utf-8 -*-
import sys
import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions.execute_process import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_camera = LaunchConfiguration('use_camera', default='none')

    launch_file_dir = os.path.join(get_package_share_directory('neuronbot2_gazebo'), 'launch')
    pkg_neuronbot2_description = get_package_share_directory('neuronbot2_description')
    urdf = os.path.join(pkg_neuronbot2_description, 'urdf/', 'neuronbot2.urdf')
    assert os.path.exists(urdf), "neuronbot2.urdf doesnt exist in "+str(urdf)

    ld = LaunchDescription()

    NUM_OF_ROBOTS = 2
    for i in range(NUM_OF_ROBOTS):
        robot_name = f'robot{i}'

        ld.add_entity(
            Node(
                package='neuronbot2_description', 
                executable='spawn_nb2.py',
                name=f'spawn_{i}',
                arguments=[
                    robot_name,
                    robot_name,
                    str(0.3 * i),'1.45','0.91','0','0','-1.57'
                ],
                output='screen'
            )
        )

        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
                launch_arguments={'robot_namespace': robot_name,
                                  'use_sim_time': use_sim_time,
                                  'use_camera': use_camera}.items(),
            ),
        )

    return ld