import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable
from launch.conditions import IfCondition

from osrf_pycommon.terminal_color import ansi

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = [get_package_share_directory('neuronbot2_gazebo'), '/worlds/']
    world.append(LaunchConfiguration('world_model', default='mememan_world.model'))
    use_camera = LaunchConfiguration('use_camera', default='none')
    launch_file_dir = os.path.join(get_package_share_directory('neuronbot2_gazebo'), 'launch')
    gazebo_model_path = os.path.join(get_package_share_directory('neuronbot2_gazebo'), 'models')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += ":" + gazebo_model_path
    else :
        os.environ['GAZEBO_MODEL_PATH'] = gazebo_model_path

    print(ansi("yellow"), "If it's your 1st time to download Gazebo model on your computer, it may take few minutes to finish.", ansi("reset"))

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_camera',
            default_value='none',
            description='Use camera?'),

        ExecuteProcess(
            cmd=['gzserver', world , 
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so',
                '--verbose'
                ],
            # additional_env=EnvironmentVariable('GAZEBO_MODEL_PATH'),
            output='screen'),

        ExecuteProcess(
            cmd=['gzclient'],
            output='screen'),

        ExecuteProcess(
            cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py', '-file' , gazebo_model_path + '/neuronbot2/' + 'model.sdf',
                '-entity', 'nb2', '-spawn_service_timeout', '300'],
            condition=IfCondition(PythonExpression(['"', use_camera, '" == "none"'])),
            output='screen'),

        ExecuteProcess(
            cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py', '-file' , gazebo_model_path + '/neuronbot2_w_front_camera/' + 'model.sdf',
                '-entity', 'nb2', '-spawn_service_timeout', '300'],
            condition=IfCondition(PythonExpression(['"', use_camera, '" == "front"'])),
            output='screen'),

        ExecuteProcess(
            cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py', '-file' , gazebo_model_path + '/neuronbot2_w_top_camera/' + 'model.sdf',
                '-entity', 'nb2', '-spawn_service_timeout', '300'],
            condition=IfCondition(PythonExpression(['"', use_camera, '" == "top"'])),
            output='screen'),

        ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/gazebo', 'use_sim_time', use_sim_time],
            output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time,
                              'use_camera': use_camera}.items(),
        ),
    ])
