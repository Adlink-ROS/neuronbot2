import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    rviz_config_dir = os.path.join(
        get_package_share_directory('neuronbot2_slam'),
        'rviz',
        'slam.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'open_rviz',
            default_value='false',
            description='open rviz'),

        launch_ros.actions.Node(
            parameters=[
                {get_package_share_directory("neuronbot2_slam") + '/config/slam_toolbox_params.yaml'},
                {'use_sim_time': use_sim_time}
                ],
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen'
            ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration("open_rviz"))
            ),

        ])
