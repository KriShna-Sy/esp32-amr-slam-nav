import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('esp32_odom_bridge')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Default parameters file
    default_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use for all sustained nodes'
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # Include Nav2 navigation stack (planner, controller, costmaps, recoveries, bt_navigator)
    # We do NOT include amcl or map_server because RTAB-Map provides localization (map->odom) and /map directly.
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': LaunchConfiguration('params_file'),
            'autostart': LaunchConfiguration('autostart'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_composition': 'False',
            'use_respawn': 'False',
        }.items()
    )

    return LaunchDescription([
        params_file_arg,
        autostart_arg,
        use_sim_time_arg,
        navigation_launch,
    ])
