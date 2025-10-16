from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = 'crawler_controller'
    cfg = os.path.join(get_package_share_directory(pkg), 'config', 'driver.params.yaml')

    # ---- Launch args ----
    serial_port_arg = DeclareLaunchArgument(
        'lidar_serial_port',
        default_value='/dev/ttyUSB0',
        description='RPLIDAR serial port'
    )

    # ---- Nodes ----
    driver = Node(
        package=pkg,
        executable='epos_driver2.py',   # see tip at bottom about console_scripts
        name='epos_driver',
        output='screen',
        parameters=[cfg],
        emulate_tty=True
    )

    odometry_node = Node(
        package=pkg,
        executable='odom.py',
        name='odom',
        output='screen',
        parameters=[{'wheel_radius_mm': 68.91, 'track_width_mm': 512.0}],
        emulate_tty=True
    )

    # RPLIDAR include (adjust the file name if your rplidar_ros package uses a different launch)
    rplidar_launch_path = os.path.join(
        get_package_share_directory('rplidar_ros'),
        'launch',
        'rplidar_c1_launch.py'
    )
    rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rplidar_launch_path),
        launch_arguments={
            # Only works if that launch exposes this arg; otherwise remove/change key name
            'serial_port': LaunchConfiguration('lidar_serial_port')
        }.items()
    )

    lidar_avoid = Node(
        package=pkg,
        executable='obstacle_gate.py',
        name='obstacle_gate',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'scan_topic': '/scan',
            'cmd_vel_topic': '/cmd_vel',
            'front_deg_min': -25.0, 'front_deg_max':  25.0,
            'left_deg_min':   25.0, 'left_deg_max':   90.0,
            'right_deg_min': -90.0, 'right_deg_max': -25.0,
            'stop_dist': 0.30, 'left_dist': 0.50, 'right_dist': 0.60,
            'cruise_lin': 0.30, 'slow_lin': 0.15, 'linear_sign': -1.0, 'ang_cap': 0.6,
            'steer_timeout_s': 0.20, 'decay_factor': 0.85, 'publish_hz': 20.0,
            'scan_yaw_offset_deg': 0.0,
            'ignore_deg_spans_str': ['20,180', '-180,-20'],
        }],
    )

    teleop_gui = Node(
        package=pkg,
        executable='teleop_gui.py',
        name='teleop_gui',
        output='screen',
        emulate_tty=True,
        parameters=[{'cmd_vel_topic': '/cmd_vel', 'publish_hz': 50.0}],
    )

    return LaunchDescription([
        serial_port_arg,
        driver,
        # odometry_node,   # <- uncomment when you want odom
        rplidar,
        # lidar_avoid,     # <- uncomment when you want obstacle gate active
        teleop_gui,
    ])
