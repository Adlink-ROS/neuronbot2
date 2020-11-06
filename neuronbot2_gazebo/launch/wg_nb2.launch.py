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
    world = os.path.join('worlds/willowgarage.world')
    launch_file_dir = os.path.join(get_package_share_directory('neuronbot2_gazebo'), 'launch')
    gazebo_model_path = os.path.join(get_package_share_directory('neuronbot2_gazebo'), 'models')
    print(gazebo_model_path)

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += ":" + gazebo_model_path
    else :
        os.environ['GAZEBO_MODEL_PATH'] = gazebo_model_path

    print(os.environ['GAZEBO_MODEL_PATH'])

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gzserver', '--verbose', world,
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so'],
            # additional_env=EnvironmentVariable('GAZEBO_MODEL_PATH'),
            output='screen'),

        ExecuteProcess(
            cmd=['gzclient'],
            output='screen'),

        ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/gazebo', 'use_sim_time', use_sim_time],
            output='screen'),
            # ros2 run gazebo_ros spawn_entity.py -file /home/ros/nb2_sim_foxy_ws/install/neuronbot2_gazebo/share/neuronbot2_gazebo/models/neuronbot2/model.sdf -entity nb2
        
        ExecuteProcess(
            cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py', '-file' , gazebo_model_path + '/neuronbot2/' + 'model.sdf',
                '-entity', 'nb2'],
            output='screen'),       

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ])
