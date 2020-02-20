import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions

def generate_launch_description():
    
    bt_path = [get_package_share_directory('neuronbot2_bt'), '/config/', LaunchConfiguration('bt_xml', default='neuronbt.xml')]
    param_substitutions = { 'bt_path' : bt_path }

    return LaunchDescription([
        launch_ros.actions.Node(
            package='neuronbot2_bt',
            node_executable='neuronbot2_bt',
            parameters=[param_substitutions],
            output='screen'),
        ])
