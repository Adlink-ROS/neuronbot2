import os
from  ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from nav2_common.launch import ReplaceString


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', 
        default=os.path.join(get_package_share_directory('neuronbot2_slam') , 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename', default='cartographer.lua')
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='0.5')
    rviz_config_file = os.path.join(get_package_share_directory('neuronbot2_slam'), 'rviz', 'namespaced_slam.rviz')

    namespaced_rviz_config_file = ReplaceString(
            source_file=rviz_config_file,
            replacements={'<robot_namespace>': namespace})

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='robot0',
            description='namespace'),

        DeclareLaunchArgument(
            'open_rviz',
            default_value='false',
            description='open rviz'),

        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config file to load'),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'),
        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'),

        SetEnvironmentVariable(
            name='ROBOT_NAMESPACE', 
            value=namespace),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            namespace=namespace,
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', configuration_basename]),

        Node(
            package='cartographer_ros',
            executable='occupancy_grid_node',
            name='occupancy_grid_node',
            namespace=namespace,
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec]),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            namespace=namespace,
            arguments=['-d', namespaced_rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration("open_rviz")),
            output='screen'),
    ])
