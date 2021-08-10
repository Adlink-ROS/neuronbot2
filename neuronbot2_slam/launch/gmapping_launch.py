import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from nav2_common.launch import ReplaceString

def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    namespace = LaunchConfiguration('namespace')

    rviz_config_file = os.path.join(
        get_package_share_directory('neuronbot2_slam'),
        'rviz',
        'namespaced_slam.rviz')
    
    namespaced_rviz_config_file = ReplaceString(
            source_file=rviz_config_file,
            replacements={'<robot_namespace>': namespace})

    remappings = [((namespace, '/tf'), 'tf'),
                  ((namespace, '/tf_static'), 'tf_static')]

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'scan_topic': 'scan',
        'base_frame': (namespace, '/base_link'),
        'odom_frame': (namespace, '/odom'),
        'map_frame': (namespace, '/map'),
        'map_update_interval': 3.0,
        'maxUrange': 10.0,
        'sigma': 0.05,
        'kernelSize': 1,
        'lstep': 0.05,
        'astep': 0.05,
        'iterations': 5,
        'lsigma': 0.075,
        'ogain': 3.0,
        'lskip': 0,
        'srr': 0.1,
        'srt': 0.2,
        'str': 0.1,
        'stt': 0.2,
        'linearUpdate': 0.3,
        'angularUpdate': 3.14,
        'temporalUpdate': 5.0,
        'resampleThreshold': 0.5,
        'particles': 30,
        'xmin': -15.0,
        'ymin': -15.0,
        'xmax': 15.0,
        'ymax': 15.0,
        'delta': 0.025,
        'llsamplerange': 0.01,
        'llsamplestep': 0.01,
        'lasamplerange': 0.005,
        'lasamplestep': 0.005,
    }
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='robot0',
            description='namespace'),

        DeclareLaunchArgument(
            'open_rviz',
            default_value='false',
            description='open rviz'),

        Node(
            package='slam_gmapping', 
            executable='slam_gmapping', 
            namespace=namespace,
            parameters=[param_substitutions],
            output='screen'),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            namespace=namespace,
            arguments=['-d', namespaced_rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration("open_rviz")),
            remappings=remappings
            ),
    ])
