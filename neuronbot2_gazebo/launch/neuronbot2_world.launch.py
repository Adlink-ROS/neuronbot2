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
from launch.launch_description_sources import PythonLaunchDescriptionSource

from osrf_pycommon.terminal_color import ansi

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_camera = LaunchConfiguration('use_camera', default='none')

    world = [get_package_share_directory('neuronbot2_gazebo'), '/worlds/']
    world.append(LaunchConfiguration('world_model', default='mememan_world.model'))
    gazebo_model_path = os.path.join(get_package_share_directory('neuronbot2_gazebo'), 'models')
    warehouse_pkg_dir = get_package_share_directory('aws_robomaker_small_warehouse_world')
    warehouse_model_path = os.path.join(warehouse_pkg_dir, 'models')
    pkg_neuronbot2_description = get_package_share_directory('neuronbot2_description')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        if -1 == os.environ['GAZEBO_MODEL_PATH'].find(gazebo_model_path):
            os.environ['GAZEBO_MODEL_PATH'] += ":" + gazebo_model_path 
        if -1 == os.environ['GAZEBO_MODEL_PATH'].find(warehouse_model_path):
            os.environ['GAZEBO_MODEL_PATH'] += ":" + warehouse_model_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = gazebo_model_path + ":" + warehouse_model_path

    print(ansi("yellow"), "If it's your 1st time to download Gazebo model on your computer, it may take few minutes to finish.", ansi("reset"))
    print(ansi("yellow"), os.environ['GAZEBO_MODEL_PATH'], ansi("reset"))

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
            # cwd=[warehouse_pkg_dir],
            output='screen'),

        ExecuteProcess(
            cmd=['gzclient'],
            # cwd=[warehouse_pkg_dir],
            output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_neuronbot2_description, 'launch', 'spawn_neuronbot2.launch.py'),
            ),
            launch_arguments={'use_sim_time': use_sim_time,
                                'use_camera': use_camera}.items(),
        ),
    ])
