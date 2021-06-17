#!/usr/bin/python3
# -*- coding: utf-8 -*-
import sys
import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions.execute_process import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():

    pkg_neuronbot2_description = get_package_share_directory('neuronbot2_description')
    urdf = os.path.join(pkg_neuronbot2_description, 'urdf/', 'neuronbot2.urdf')
    assert os.path.exists(urdf), "neuronbot2.urdf doesnt exist in "+str(urdf)

    ld = LaunchDescription([
        #Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen', arguments=[urdf]),
    ])

    NUM_OF_ROBOTS = 2
    for i in range(NUM_OF_ROBOTS):
        ld.add_entity(
            Node(
                package='neuronbot2_description', 
                executable='spawn_nb2.py',
                name=f'spawn_{i}',
                namespace='',
                arguments=[
                    f'bot{i}',
                    f'bot{i}',
                    str(0.3 * i),'1.45','0.91','0','0','-1.57'
                ],
                output='screen'
            )
        )

    return ld