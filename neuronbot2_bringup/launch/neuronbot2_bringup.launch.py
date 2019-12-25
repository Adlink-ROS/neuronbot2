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
    description_launch_file_dir = os.path.join(get_package_share_directory('neuronbot2_gazebo'), 'launch')

    ekf_config = Path(get_package_share_directory('neuronbot2_bringup'), 'cfg', 'ekf.yaml')
    urdf_path = Path(get_package_share_directory('neuronbot2_description'), 'urdf', 'neuronbot2.urdf')
    hardware_config = Path(get_package_share_directory('neuronbot2_bringup'), 'cfg', 'hardware.yaml')
    # hardware_config = Path(get_package_share_directory('omni_ros2'), 'cfg', 'hardware.yaml')
    assert hardware_config.is_file()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            node_executable='robot_state_publisher',
            node_name='robot_state_publisher',
            output='screen',
            arguments=[str(urdf_path)],
        ),

        Node(
            package='rplidar_ros',
            node_executable='rplidar_composition',
            output='screen',
            parameters=[hardware_config],
        ),

        # Node(
        #     package='joint_state_publisher', 
        #     node_executable='joint_state_publisher', 
        #     output='screen',
        #     arguments=[str(urdf_path)],
        #     parameters=[hardware_config]
        # ),


        Node(
            package='neuronbot2_bringup',
            node_executable='neuronbot2_driver',
            node_name='neuronbot2_driver',
            # output='screen',

            # parameters=[{"calibrate_imu": False}]
        ),

        Node(
            package='robot_localization',
            node_executable='se_node',
            output='screen',
            parameters=[ekf_config],
            remappings=[("odometry/filtered", "odom")]
        ),
    ])
