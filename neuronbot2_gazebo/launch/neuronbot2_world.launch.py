import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    world = [get_package_share_directory('neuronbot2_gazebo'), '/worlds/']
    world.append(LaunchConfiguration('world_model', default='mememan_world.model'))
    launch_file_dir = os.path.join(get_package_share_directory('neuronbot2_gazebo'), 'launch')
    gazebo_model_path = os.path.join(get_package_share_directory('neuronbot2_gazebo'), 'models')
    print(gazebo_model_path)

    return LaunchDescription([
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=[EnvironmentVariable('GAZEBO_MODEL_PATH'), gazebo_model_path]),

        ExecuteProcess(
            cmd=['gzserver', '--verbose', world , '-s', 'libgazebo_ros_init.so'],
            # additional_env=EnvironmentVariable('GAZEBO_MODEL_PATH'),
            output='screen'),

        ExecuteProcess(
            cmd=['gzclient'],
            output='screen'),

        ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/gazebo', 'use_sim_time', use_sim_time],
            output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ])
