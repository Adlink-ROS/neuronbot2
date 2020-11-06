import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_camera = LaunchConfiguration('use_camera', default='none')

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

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_camera',
            default_value='none',
            description='Use camera?'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # if we don't use camera:
        Node(
            condition=IfCondition(PythonExpression(['"', use_camera, '" == "none"'])),
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[nb2_urdf]),

        # else if we use front camera:
        Node(
            condition=IfCondition(PythonExpression(['"', use_camera, '" == "front"'])),
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[nb2_w_front_camera_urdf]),

        # else if we use top camera:
        Node(
            condition=IfCondition(PythonExpression(['"', use_camera, '" == "top"'])),
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[nb2_w_top_camera_urdf]),                        
    ])
