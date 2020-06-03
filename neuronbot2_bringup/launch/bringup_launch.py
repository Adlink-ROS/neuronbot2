import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')

    ekf_config = Path(get_package_share_directory('neuronbot2_bringup'), 'cfg', 'ekf.yaml')
    urdf_path = Path(get_package_share_directory('neuronbot2_description'), 'urdf', 'neuronbot2.urdf')
    hardware_config = Path(get_package_share_directory('neuronbot2_bringup'), 'cfg', 'hardware.yaml')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[str(urdf_path)],
        ),

        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar',
            output='screen',
            parameters=[hardware_config],
        ),

        #Node(
        #    package='joint_state_publisher', 
        #    executable='joint_state_publisher', 
        #    name='joint_state_publisher',
        #    output='screen',
        #    arguments=[str(urdf_path)],
        #    parameters=[hardware_config]
        #),


        Node(
            package='neuronbot2_bringup',
            executable='neuronbot2_driver',
            output='screen',
            parameters=[{'update_status_freq': 30.0}],
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            output='screen',
            parameters=[ekf_config],
            remappings=[("odometry/filtered", "odom")]
        ),
    ])
