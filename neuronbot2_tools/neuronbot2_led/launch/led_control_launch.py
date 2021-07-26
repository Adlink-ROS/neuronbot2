from pathlib import Path
from ament_index_python.packages import get_package_share_directory, get_resource, get_package_prefix
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():

    ld = LaunchDescription()

    led_cmd = Path(get_package_prefix('neuronbot2_led'), 'lib', 'neuronbot2_led', 'led_control')

    led_process = ExecuteProcess(cmd=[str(led_cmd), '--port', '/dev/neuronbotLED', '--mode', '5', '--loop'])

    ld.add_action(led_process)

    return ld
