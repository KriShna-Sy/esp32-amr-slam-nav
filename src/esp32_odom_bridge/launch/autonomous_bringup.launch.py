import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('esp32_odom_bridge')

    # Arguments passed down to localization
    launch_viz_arg = DeclareLaunchArgument(
        'launch_viz',
        default_value='true',
        description='Whether to launch RViz2 visualizer GUI'
    )
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for ESP32 motor controller'
    )
    nav2_delay_arg = DeclareLaunchArgument(
        'nav2_delay',
        default_value='7.0',
        description='Seconds to wait for camera, odometry, and map TF before starting Nav2'
    )
    auto_localize_arg = DeclareLaunchArgument(
        'auto_localize',
        default_value='false',
        description='Whether to automatically initialize pose to start landmark (default: false so robot can start anywhere in map)'
    )

    # 1. Hardware, Odometry, Camera, and RTAB-Map Localization
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'localization.launch.py')
        ),
        launch_arguments={
            'launch_viz': LaunchConfiguration('launch_viz'),
            'serial_port': LaunchConfiguration('serial_port'),
            'launch_camera': 'true',
        }.items(),
    )

    # 2. Nav2 Autonomous Navigation Stack (Planner, Controller, Costmaps, BT Navigator)
    # Delayed by nav2_delay seconds to guarantee that /odom and map->odom TF are streaming
    # before Nav2 lifecycle nodes attempt configuration and activation.
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'nav2.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'False',
            'autostart': 'True',
        }.items(),
    )

    delayed_nav2 = TimerAction(
        period=7.0,
        actions=[
            LogInfo(msg="[AUTONOMOUS BRINGUP] Localization warmed up. Initializing Nav2 Autonomous Stack..."),
            navigation_launch,
        ]
    )

    # 3. Optional auto-initialization to 'start' (DISABLED by default so real localization isn't corrupted)
    auto_localize = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='esp32_odom_bridge',
                executable='landmark_manager',
                name='auto_localizer',
                arguments=['setpose', 'start'],
                output='screen',
                condition=IfCondition(LaunchConfiguration('auto_localize')),
            ),
        ]
    )

    return LaunchDescription([
        launch_viz_arg,
        serial_port_arg,
        nav2_delay_arg,
        auto_localize_arg,
        LogInfo(msg="[AUTONOMOUS BRINGUP] Starting Hardware, Odometry & Localization Stack..."),
        localization_launch,
        delayed_nav2,
        auto_localize,
    ])
