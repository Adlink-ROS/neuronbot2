import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    namespace = LaunchConfiguration('namespace')
    open_rviz = LaunchConfiguration('open_rviz', default='true')

    rviz_config_file = os.path.join(
            get_package_share_directory('neuronbot2_nav'),
            'rviz',
            'nav2_namespaced_view.rviz')
    
    namespaced_rviz_config_file = ReplaceString(
            source_file=rviz_config_file,
            replacements={'<robot_namespace>': namespace})

    ld = LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='robot0',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'open_rviz',
            default_value='true',
            description='Launch Rviz?'),

        Node(
            package='rviz2',
            executable='rviz2',
            # name='rviz2',
            namespace=namespace,
            arguments=['-d', namespaced_rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(open_rviz)
            # output='log'
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', (namespace,'/map')]
        )
    ])

    return ld
