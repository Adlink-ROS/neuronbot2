import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    urdf_path = Path(get_package_share_directory('neuronbot2_description'), 'urdf', 'neuronbot2.urdf')
    hardware_config = Path(get_package_share_directory('neuronbot2_bringup'), 'cfg', 'hardware.yaml')

    # Whether to use EKF for multi-sensor fusion
    use_ekf =  LaunchConfiguration('use_ekf', default='false')
    ekf_config = Path(get_package_share_directory('neuronbot2_bringup'), 'cfg', 'ekf.yaml')

    # Whether to open RealSense D435
    use_camera = LaunchConfiguration('use_camera', default='false')

    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            node_executable='robot_state_publisher',
            node_name='robot_state_publisher',
            output='screen',
            arguments=[str(urdf_path)],
        ),
#        Node(
#            package='joint_state_publisher', 
#            node_executable='joint_state_publisher', 
#            node_name='joint_state_publisher',
#            output='screen',
#            arguments=[str(urdf_path)],
#            parameters=[hardware_config]
#        ),

        Node(
            package='rplidar_ros',
            node_executable='rplidar_node',
            node_name='rplidar',
            output='screen',
            parameters=[hardware_config],
        ),
       
        # If we use EKF:
        Node(
            condition=IfCondition(use_ekf),
            package='neuronbot2_bringup',
            node_executable='neuronbot2_driver',
            output='screen',
            parameters=[{
                'publish_tf': False,
                'calibrate_imu' : True,
                'odom_topic': 'raw_odom',
                'odom_freq' : 50.0,
                'imu_topic': 'raw_imu',
                'imu_freq' : 100.0,
                'cmd_vel_timeout' : 1.0
                }],
        ),
        Node(
            condition=IfCondition(use_ekf),
            package='robot_localization',
            node_executable='ekf_node',
            output='screen',
            parameters=[ekf_config],
            remappings=[("odometry/filtered", "odom")]
        ),

        # If we don't use EKF:
        Node(
            condition=UnlessCondition(use_ekf),
            package='neuronbot2_bringup',
            node_executable='neuronbot2_driver',
            output='screen',
            parameters=[{
                'publish_tf': True,
                'calibrate_imu' : False,
                'odom_topic': 'odom',
                'odom_freq' : 50.0,
                'imu_topic': 'raw_imu',
                'imu_freq' : 100.0,
                'cmd_vel_timeout' : 1.0
                }],
        ),

        # RealSense node
        Node(
            condition=IfCondition(use_camera),
            package='realsense_node',
            node_executable='realsense_node',
            namespace='',
            output='screen'
        ),

    ])
