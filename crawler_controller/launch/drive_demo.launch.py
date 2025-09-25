
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = 'crawler_controller'
    cfg = os.path.join(get_package_share_directory(pkg), 'config', 'driver.params.yaml')

    driver = Node(
        package=pkg,
        executable='epos_driver2.py',
        name='epos_driver',
        output='screen',
        parameters=[cfg],
    )

    odometery = Node(
        package=pkg,
        executable='odom.py',
        name='odom',
        output='screen',
        parameters=[{'wheel_radius_mm': 68.91, 'track_width_mm': 512.0}],
    )

  


    # --- Include RPLIDAR launch from rplidar_ros ---
    rplidar_launch_path = os.path.join(
        get_package_share_directory('rplidar_ros'),
        'launch',
        'rplidar_c1_launch.py'
    )
    rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rplidar_launch_path)
        # , launch_arguments={'serial_port': '/dev/ttyUSB0'}.items()  # <- if that launch supports args
    )

    lidar_avoid = Node(
        package=pkg,
        executable='obstacle_gate.py',  # OK with ament_cmake install(PROGRAMS)
        name='obstacle_gate',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'scan_topic': '/scan',
            'cmd_vel_topic': '/cmd_vel',
            # For -pi..+pi scans (your case):
            'front_deg_min': -25.0, 'front_deg_max':  25.0,
            'left_deg_min':   25.0, 'left_deg_max':   90.0,
            'right_deg_min': -90.0, 'right_deg_max': -25.0,
            # Thresholds
            'stop_dist': 0.30, 'left_dist': 0.50, 'right_dist': 0.60,
            # Motion & smoothing
            'cruise_lin': 0.30, 'slow_lin': 0.15, 'linear_sign': -1.0, 'ang_cap': 0.6,
            'steer_timeout_s': 0.20, 'decay_factor': 0.85, 'publish_hz': 20.0,
            # Mounting (0 unless your LiDAR is rotated)
            'scan_yaw_offset_deg': 0.0,
            # Ignore rear: (two spans for -180..+180° domain)
            'ignore_deg_spans_str': ['20,180', '-180,-20'],
        }],
    )

    teleop_gui = Node(
    package=pkg,
    executable='teleop_gui.py',
    name='teleop_gui',
    output='screen',
    parameters=[{'cmd_vel_topic': '/cmd_vel', 'publish_hz': 50.0}],
    )
# then include `teleop_gui` in the LaunchDescription list


    return LaunchDescription([
        driver,
        #odometery,
        #teleop,
        #teleop2,
        rplidar,
        #lidar_avoid,
        teleop_gui,
        

    ])
