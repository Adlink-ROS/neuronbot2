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
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():

    # Whether to open RealSense D435
    use_camera = LaunchConfiguration('use_camera', default='none')
    camera_config = Path(get_package_share_directory('neuronbot2_bringup'), 'cfg', 'camera.yaml')

    # Whether to use front or top camera
    urdf_file_name = 'neuronbot2.urdf'
    nb2_urdf = os.path.join(
        get_package_share_directory('neuronbot2_description'),
        'urdf',
        urdf_file_name)

    urdf_file_name = 'neuronbot2_w_front_camera.urdf'
    nb2_w_front_camera_urdf = os.path.join(
        get_package_share_directory('neuronbot2_description'),
        'urdf',
        urdf_file_name)

    urdf_file_name = 'neuronbot2_w_top_camera.urdf'
    nb2_w_top_camera_urdf = os.path.join(
        get_package_share_directory('neuronbot2_description'),
        'urdf',
        urdf_file_name)
    
    hardware_config = Path(get_package_share_directory('neuronbot2_bringup'), 'cfg', 'hardware.yaml')

    # Whether to use EKF for multi-sensor fusion
    use_ekf =  LaunchConfiguration('use_ekf', default='false')
    ekf_config = Path(get_package_share_directory('neuronbot2_bringup'), 'cfg', 'ekf.yaml')

    return LaunchDescription([

        # if we don't use camera:
        Node(
            condition=IfCondition(PythonExpression(['"', use_camera, '" == "none"'])),
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[nb2_urdf],
        ),
        # else if we use front camera:
        Node(
            condition=IfCondition(PythonExpression(['"', use_camera, '" == "front"'])),
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[nb2_w_front_camera_urdf],
        ),
        # else if we use top camera:
        Node(
            condition=IfCondition(PythonExpression(['"', use_camera, '" == "top"'])),
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[nb2_w_top_camera_urdf],
        ),

        # open realsense D435 if use_camera != none
        Node(
            condition=IfCondition(PythonExpression(['"', use_camera, '" != "none"'])),
            package='realsense2_camera',
            executable='realsense2_camera_node',
            namespace='',
            output='screen',
            parameters=[camera_config],
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
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar',
            output='screen',
            parameters=[hardware_config],
        ),
       
        # If we use EKF:
        Node(
            condition=IfCondition(use_ekf),
            package='neuronbot2_bringup',
            executable='neuronbot2_driver',
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
            executable='ekf_node',
            output='screen',
            parameters=[ekf_config],
            remappings=[("odometry/filtered", "odom")]
        ),
        # else if we don't use EKF:
        Node(
            condition=UnlessCondition(use_ekf),
            package='neuronbot2_bringup',
            executable='neuronbot2_driver',
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

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('neuronbot2_led'), '/led_control_launch.py'
            ])
        ),

    ])
