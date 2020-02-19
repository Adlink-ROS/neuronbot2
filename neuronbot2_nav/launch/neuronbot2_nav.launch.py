import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    open_rviz = LaunchConfiguration('open_rviz', default='False')

    map_dir = LaunchConfiguration(
        'map_dir',
        default=os.path.join(
            get_package_share_directory('neuronbot2_nav'),
            'map',
            'mememan.yaml'))

    param_dir = LaunchConfiguration(
        'params',
        default=os.path.join(
            get_package_share_directory('neuronbot2_nav'),
            'param',
            'nav2.yaml'))

    bt_xml_path = LaunchConfiguration(
        'bt_xml',
        default=os.path.join(
            get_package_share_directory('neuronbot2_nav'),
            'param',
            'bt_nav2.xml'
        )
    )
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    rviz_config_dir = os.path.join(
        get_package_share_directory('neuronbot2_nav'),
        'rviz',
        'nav2.rviz')

    return LaunchDescription([

        DeclareLaunchArgument(
            'map_dir',
            default_value=map_dir,
            description='Full path to map directory'),

        DeclareLaunchArgument(
            'params',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'bt_xml_filename',
            default_value=bt_xml_path,
            description='Full path to behavior_tree xml file'),
    
        DeclareLaunchArgument(
            'open_rviz', 
            default_value=open_rviz, 
            description='Launch Rviz?'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/nav2_bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'bt_xml_filename': bt_xml_path,
                'use_sim_time': use_sim_time,
                'params': param_dir}.items(),
        ),
        
        Node(
            package='rviz2',
            node_executable='rviz2',
            node_name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration("open_rviz"))
            # output='log'
            ),
    ])
