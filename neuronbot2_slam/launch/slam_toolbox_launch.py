import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from nav2_common.launch import ReplaceString, RewrittenYaml

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    namespace = LaunchConfiguration('namespace')
    rviz_config_file = os.path.join(
        get_package_share_directory('neuronbot2_slam'),
        'rviz',
        'namespaced_slam.rviz')
    slam_config_file = os.path.join(
        get_package_share_directory("neuronbot2_slam"),
        'config',
        'slam_toolbox_params.yaml')
    
    namespaced_rviz_config_file = ReplaceString(
            source_file=rviz_config_file,
            replacements={'<robot_namespace>': namespace})
    
    namespaced_slam_config_file = RewrittenYaml(
        source_file=slam_config_file,
        root_key=namespace,
        param_rewrites={'scan_topic': 'scan',
                        'map_name': 'map',
                        'map_frame': (namespace, '/map'),
                        'odom_frame': (namespace, '/odom'),
                        'base_frame': (namespace, '/base_footprint')}
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'open_rviz',
            default_value='false',
            description='open rviz'),
        
        DeclareLaunchArgument(
            'namespace',
            default_value='robot0',
            description='namespace',
        ),

        Node(
            parameters=[
                namespaced_slam_config_file,
                {'use_sim_time': use_sim_time}
                ],
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            namespace=namespace,
            output='screen'
            ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            namespace=namespace,
            arguments=['-d', namespaced_rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration("open_rviz"))
            ),

        ])
