import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

from osrf_pycommon.terminal_color import ansi

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    use_camera = LaunchConfiguration('use_camera', default='none')
    custom_world = LaunchConfiguration('world_model', default='none')
    headless = LaunchConfiguration('headless', default='False')
    
    custom_world_path = os.path.join(get_package_share_directory('neuronbot2_gazebo'), 'worlds')
    gazebo_model_path = os.path.join(get_package_share_directory('neuronbot2_gazebo'), 'models')
    warehouse_launch_path = os.path.join(get_package_share_directory('aws_robomaker_small_warehouse_world'), 'launch')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        if -1 == os.environ['GAZEBO_MODEL_PATH'].find(gazebo_model_path):
            os.environ['GAZEBO_MODEL_PATH'] += ":" + gazebo_model_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = gazebo_model_path

    print(ansi("yellow"), "If it's your 1st time to download Gazebo model on your computer, it may take few minutes to finish.", ansi("reset"))
    
    is_custom_world_defined = PythonExpression(('"', custom_world, '" != "none"'))

    # if 'world_model' is not set, aws_small_warehouse would be launched in Gazebo
    warehouse_world_cmd = IncludeLaunchDescription(
        condition=UnlessCondition(is_custom_world_defined),
        launch_description_source=PythonLaunchDescriptionSource(
            [warehouse_launch_path, '/no_roof_small_warehouse_launch.py']
        ),
        launch_arguments={'headless': headless}.items()
    )

    # otherwise, we launch the given world file in neuronbot2_gazebo/worlds in Gazebo
    custom_world_cmd = GroupAction(
        condition=IfCondition(is_custom_world_defined),
        actions = [
            ExecuteProcess(
                cmd=['gzserver', (custom_world_path , '/', custom_world),
                    '-s', 'libgazebo_ros_init.so',
                    '-s', 'libgazebo_ros_factory.so',
                    '--verbose'
                    ],
                output='screen'),

            ExecuteProcess(
                condition=UnlessCondition(headless),
                cmd=['gzclient'],
                output='screen')
        ]
    )

    return LaunchDescription([

        warehouse_world_cmd,

        custom_world_cmd,

        DeclareLaunchArgument(
            'use_camera',
            default_value='none',
            description='Use camera?'),
        
        DeclareLaunchArgument(
            'headless',
            default_value='False',
            description='Whether to execute gzclient)'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('neuronbot2_description'), 'launch', 'spawn_neuronbot2.launch.py'),
            ),
            launch_arguments={'use_sim_time': use_sim_time,
                                'use_camera': use_camera}.items(),
        ),
    ])