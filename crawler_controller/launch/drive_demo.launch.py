from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = 'crawler_controller'
    cfg = os.path.join(get_package_share_directory(pkg), 'config', 'driver.params.yaml')

    return LaunchDescription([
        Node(
            package=pkg,
            executable='epos_driver.py',   # installed via install(PROGRAMS)
            name='epos_driver',
            output='screen',
            parameters=[cfg],
        ),
        Node(
            package=pkg,
            executable='teleop_cli.py',
            name='teleop_cli',
            output='screen',
        ),
    ])
