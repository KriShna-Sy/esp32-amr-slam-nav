import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    depthai_prefix = get_package_share_directory('depthai_descriptions_v3')
    bridge_prefix = get_package_share_directory('esp32_odom_bridge')

    # Odometry parameters
    serial_port_arg = DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0')
    baud_arg = DeclareLaunchArgument('baud', default_value='115200')
    wheelbase_arg = DeclareLaunchArgument('wheelbase', default_value='0.5458')
    wheel_diameter_arg = DeclareLaunchArgument('wheel_diameter', default_value='0.127')
    ticks_per_rev_arg = DeclareLaunchArgument('ticks_per_rev', default_value='49415')
    max_linear_speed_arg = DeclareLaunchArgument('max_linear_speed', default_value='0.60')
    max_angular_speed_arg = DeclareLaunchArgument('max_angular_speed', default_value='1.00')
    linear_scale_arg = DeclareLaunchArgument('linear_scale', default_value='1.0')
    angular_scale_arg = DeclareLaunchArgument('angular_scale', default_value='1.0')

    # Camera mount offset relative to base_link.
    # Defaults come from nav_data/camera.yaml = the mount the ACTIVE map was made with
    # (written by save_nav_map.sh, swapped by nav_map_switch.sh). Fallback: current mount.
    cam = {'cam_x': '0.25', 'cam_y': '0.0', 'cam_z': '0.795',
           'cam_roll': '0.005', 'cam_pitch': '0.121', 'cam_yaw': '0.0'}
    cam_yaml = os.path.expanduser('~/mapping_ws/nav_data/camera.yaml')
    if os.path.exists(cam_yaml):
        for line in open(cam_yaml):
            k, _, v = line.partition(':')
            if k.strip() in cam and v.strip():
                cam[k.strip()] = v.strip()
    print(f"[localization] camera mount (nav_data/camera.yaml): {cam}")
    cam_x_arg = DeclareLaunchArgument('cam_x', default_value=cam['cam_x'])
    cam_y_arg = DeclareLaunchArgument('cam_y', default_value=cam['cam_y'])
    cam_z_arg = DeclareLaunchArgument('cam_z', default_value=cam['cam_z'])
    cam_roll_arg = DeclareLaunchArgument('cam_roll', default_value=cam['cam_roll'])
    cam_pitch_arg = DeclareLaunchArgument('cam_pitch', default_value=cam['cam_pitch'])
    cam_yaw_arg = DeclareLaunchArgument('cam_yaw', default_value=cam['cam_yaw'])

    # Database path: prefer the frozen navigation map (maps/nav_map.db) so a new
    # mapping run in ~/.ros/rtabmap.db can never silently replace the nav map.
    nav_db = os.path.expanduser('~/mapping_ws/nav_data/nav_map.db')
    if not os.path.exists(nav_db):
        raise RuntimeError(
            f"{nav_db} not found. Make a map first:\n"
            "  ros2 launch esp32_odom_bridge mapping.launch.py delete_db:=true\n"
            "  ros2 run esp32_odom_bridge save_nav_map.sh <name>")
    db_path_arg = DeclareLaunchArgument('database_path', default_value=nav_db)

    # Depth camera_info used to back-project depth into 3D for obstacle detection.
    # Must have the SAME width/height as /oak/stereo/image_raw.
    depth_info_arg = DeclareLaunchArgument('depth_info_topic', default_value='/oak/stereo/camera_info')
    # 'device' = point cloud computed on the OAK's Myriad X (/oak/points)
    # 'host'   = depth image -> cloud on the laptop (rtabmap_util point_cloud_xyz)
    obstacle_source_arg = DeclareLaunchArgument('obstacle_source', default_value='device')
    camera_model_arg = DeclareLaunchArgument('camera_model', default_value='OAK-D-LITE',
                                             description='OAK-D or OAK-D-LITE (URDF frames)')

    # Subsystem toggles
    launch_camera_arg = DeclareLaunchArgument('launch_camera', default_value='true')
    # RViz2 OFF by default — it consumes significant CPU/RAM and crashes (SIGSEGV).
    # Run with launch_viz:=true only when you need to debug visually.
    launch_viz_arg = DeclareLaunchArgument('launch_viz', default_value='true')

    # 1. ESP32 Motor Controller Bridge (subscribes to /cmd_vel from Nav2 / teleop, does not publish TF)
    odom_node = Node(
        package='esp32_odom_bridge',
        executable='odom_bridge_node',
        name='esp32_odom_bridge',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud': LaunchConfiguration('baud'),
            'wheelbase': LaunchConfiguration('wheelbase'),
            'wheel_diameter': LaunchConfiguration('wheel_diameter'),
            'ticks_per_rev': LaunchConfiguration('ticks_per_rev'),
            'max_linear_speed': LaunchConfiguration('max_linear_speed'),
            'max_angular_speed': LaunchConfiguration('max_angular_speed'),
            'linear_scale': LaunchConfiguration('linear_scale'),
            'angular_scale': LaunchConfiguration('angular_scale'),
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'publish_tf': False,
        }],
        remappings=[
            ('odom', '/wheel_odom'),
        ],
    )

    # 2. Robot Localization EKF (Fuses wheel odometry + camera IMU)
    #    Publishes /odometry/filtered and the authoritative odom -> base_link TF.
    ekf_config = os.path.join(bridge_prefix, 'config', 'ekf.yaml')
    src_ekf = os.path.expanduser('~/mapping_ws/src/esp32_odom_bridge/config/ekf.yaml')
    if os.path.exists(src_ekf):
        ekf_config = src_ekf

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[
            ('odometry/filtered', '/odom'),
        ],
    )

    # Static transform for IMU frame relative to camera body frame
    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='oak_imu_tf_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'oak_rgb_camera_frame',
                   '--child-frame-id', 'oak_imu_frame'],
    )

    # 2b. Direct depth-to-laserscan for OBSTACLE AVOIDANCE — independent of RTAB-Map latency.
    # Reads depth DIRECTLY from camera at ~15 FPS to produce /scan for Nav2.
    # IMPORTANT: output_frame MUST be a body-oriented frame (oak_rgb_camera_frame, X forward, Y left),
    # NOT the optical frame. Setting it to optical frame rotates the laser scan 90 degrees to the right!
    depth_to_scan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        output='screen',
        parameters=[{
            'scan_height': 30,          # 30 rows (centered band, covers obstacle height without hitting floor)
            'scan_time': 0.067,         # 15 FPS: 1/15s per scan
            'range_min': 0.35,          # OAK-D Lite min stereo depth
            'range_max': 3.2,           # OAK-D Lite max reliable stereo depth
            'output_frame': 'oak_rgb_camera_frame',
        }],
        remappings=[
            ('depth', '/oak/stereo/image_raw'),
            ('depth_camera_info', '/oak/stereo/camera_info'),
            ('scan', '/scan'),
        ],
    )

    # 2c. 3D OBSTACLE PIPELINE (replaces the flat /scan for costmaps + collision monitor).
    # The camera sits at 1.24 m with only 6 deg tilt, so a 30-row laser scan is a slice at
    # ~1.2 m height and misses chairs, boxes, desks, legs. Instead we use the full depth image:
    #   depth -> /camera_cloud (decimated, voxelized, in camera frame)
    #   -> obstacles_detection (height split in base_link) -> /obstacles_cloud + /ground_cloud
    depth_cloud_node = Node(
        package='rtabmap_util',
        executable='point_cloud_xyz',
        name='depth_to_cloud',
        output='screen',
        condition=LaunchConfigurationEquals('obstacle_source', 'host'),
        parameters=[{
            'decimation': 4,          # 1/4 resolution in each axis
            'voxel_size': 0.05,       # 5 cm voxels
            'min_depth': 0.30,
            'max_depth': 3.0,         # OAK-D Lite depth is noisy past ~3 m
            'approx_sync': True,
            'approx_sync_max_interval': 0.05,
            'queue_size': 5,
        }],
        remappings=[
            ('depth/image', '/oak/stereo/image_raw'),
            ('depth/camera_info', LaunchConfiguration('depth_info_topic')),
            ('cloud', '/camera_cloud'),
        ],
    )

    obstacles_params = {
            'frame_id': 'base_link',
            'wait_for_transform': 0.2,
            'Grid/NormalsSegmentation': 'false',
            'Grid/MaxGroundHeight': '0.08',
            'Grid/MaxObstacleHeight': '1.60',
            'Grid/RangeMax': '3.0',
            'Grid/CellSize': '0.05',             # voxelize the dense device cloud
    }
    obstacles_device_node = Node(
        package='rtabmap_util',
        executable='obstacles_detection',
        name='obstacles_detection',
        output='screen',
        condition=LaunchConfigurationEquals('obstacle_source', 'device'),
        parameters=[obstacles_params],
        remappings=[
            ('cloud', '/oak/points'),
            ('obstacles', '/obstacles_cloud'),
            ('ground', '/ground_cloud'),
        ],
    )

    obstacles_node = Node(
        package='rtabmap_util',
        executable='obstacles_detection',
        name='obstacles_detection',
        output='screen',
        condition=LaunchConfigurationEquals('obstacle_source', 'host'),
        parameters=[{
            'frame_id': 'base_link',
            'wait_for_transform': 0.2,
            'Grid/NormalsSegmentation': 'false',  # fast height pass-through split
            'Grid/MaxGroundHeight': '0.08',       # z < 8 cm in base_link = floor
            'Grid/MaxObstacleHeight': '1.60',     # ignore ceiling / hanging stuff above robot
            'Grid/RangeMax': '3.0',
        }],
        remappings=[
            ('cloud', '/camera_cloud'),
            ('obstacles', '/obstacles_cloud'),
            ('ground', '/ground_cloud'),
        ],
    )

    # 3. OAK-D Camera Driver (URDF for TF + Python bridge for streaming)
    camera_urdf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(depthai_prefix, 'launch', 'urdf_launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('launch_camera')),
        launch_arguments={
            'camera_model': LaunchConfiguration('camera_model'),
            'parent_frame': 'base_link',
            'cam_pos_x': LaunchConfiguration('cam_x'),
            'cam_pos_y': LaunchConfiguration('cam_y'),
            'cam_pos_z': LaunchConfiguration('cam_z'),
            'cam_roll': LaunchConfiguration('cam_roll'),
            'cam_pitch': LaunchConfiguration('cam_pitch'),
            'cam_yaw': LaunchConfiguration('cam_yaw'),
            'use_composition': 'False',
        }.items(),
    )

    camera_bridge = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('launch_camera')),
        cmd=[
            '/home/krishna/ros_oak_env/bin/python3',
            '/home/krishna/oak_bsvkln/src/oak_bringup/src/oak_ros_bridge.py',
        ],
        output='screen',
        respawn=True,
        respawn_delay=3.0,
    )

    # 4. RTAB-Map in Localization Mode (Read-only on database, publishes map -> odom TF)
    # gen_scan=False: Nav2 gets /scan from depthimage_to_laserscan (fresh, real-time).
    # RTAB-Map no longer needs to publish /scan. Its internal grid still uses depth directly.
    rtabmap_parameters = [{
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'database_path': LaunchConfiguration('database_path'),
        'subscribe_odom': True,
        'subscribe_rgb': True,
        'subscribe_depth': True,
        'subscribe_odom_info': False,
        'subscribe_imu': False,            # EKF already fuses IMU into /odom & TF
        'approx_sync': True,
        'approx_sync_max_interval': 0.1,
        'queue_size': 10,       # Buffered queue for robust sync
        'gen_scan': False,      # Nav2 gets /scan from depthimage_to_laserscan (fresh)
        'wait_for_transform': 0.8,
    }]

    rtabmap_arguments = [
        '--Mem/IncrementalMemory', 'false',         # Read-only localization mode
        '--Mem/InitWMWithAllNodes', 'true',         # Load entire map graph into working memory
        '--Mem/ImageKept', 'true',                  # Keep images in memory for feature & depth matching
        '--Mem/RawDescriptorsKept', 'true',
        '--Reg/Force3DoF', 'true',                  # Flat 2D plane
        '--RGBD/NeighborLinkRefining', 'false',
        '--RGBD/OptimizeMaxError', '3.0',           # was 5.0: reject loop closures that bend the graph (false jumps)
        '--RGBD/LoopClosureReextractFeatures', 'true',  # Re-extract keypoints with loaded depth images
        '--Vis/MinInliers', '10',   # 2026-11-19: 12 -> 10, evening light gave 9-11 near-misses;                   # was 6 (false 6-13 cm jumps), 15 (too few fixes: 34 in 29 m)
        '--Kp/MaxFeatures', '1000',                 # same as mapping (more loop-closure words)
        '--Vis/EstimationType', '1',                # 1 = PnP (OpenCV solvePnPRansac) - robust to depth noise
        '--Vis/PnPReprojError', '3.0',              # was 5.0: tighter PnP -> only geometrically consistent matches
        '--Vis/InlierDistance', '0.08',             # was 0.15
        '--Grid/FromDepth', 'true',
        '--Grid/MaxGroundHeight', '0.06',           # Anything > 6cm above floor is an obstacle
        '--Grid/MaxObstacleHeight', '1.8',          # Up to 1.8m
        '--Grid/MaxGroundAngle', '30',              # Strict ground normal angle (< 30 deg)
        '--Grid/MinClusterSize', '5',               # Detect small/low ground obstacles
        '--Grid/RangeMin', '0.35',                  # OAK-D Lite min stereo range
        '--Grid/RangeMax', '3.2',                   # OAK-D Lite reliable stereo depth
        '--Grid/RayTracing', 'true',                # Clear dynamic space accurately
        '--RGBD/MarkerDetection', 'true',           # ArUco markers on walls -> fixed landmarks in the graph
        '--Marker/Dictionary', '0',                 # 0 = DICT_4X4_50 (printed sheet)
        '--Marker/Length', '0.16',                  # black square size in metres (measure the print!)
        '--Marker/MaxDepthError', '0.05',
        '--Reg/Strategy', '0',
        '--Rtabmap/DetectionRate', '2.0',           # 2 loop-closure checks/sec for quick global relocalization
        '--Rtabmap/MaxRepublished', '10',
        '--Kp/MaxFeatures', '1000',                 # Rich keypoint extraction matching mapping quality
        '--Vis/MaxFeatures', '1000',                # Match rich features from map database
        '--RGBD/LinearUpdate', '0.0',               # Process frames for relocalization even when robot is stationary
        '--RGBD/AngularUpdate', '0.0',              # Process frames for relocalization even when robot is stationary
        '--Rtabmap/LoopThr', '0.08',                # Responsive loop closure detection
    ]

    rtabmap_remappings = [
        ('rgb/image', '/oak/rgb/image_raw'),
        ('rgb/camera_info', '/oak/rgb/camera_info'),
        ('depth/image', '/oak/stereo/image_raw'),
        ('odom', '/odom'),
        ('initial_pose', '/initialpose'),
    ]

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=rtabmap_parameters,
        arguments=rtabmap_arguments,
        remappings=rtabmap_remappings,
    )

    # 5. RViz2 for full navigation view (map, costmaps, path, robot pose)
    # NOTE: rtabmap_viz is intentionally disabled — it causes CPU overload during navigation
    #       (processing time 1.2s >> detection rate 0.5s) which breaks TF timing and aborts Nav2.
    #       Use RViz2 alone for visualization. If you need SLAM debug, run rtabmap_viz separately
    #       ONLY during mapping, never during navigation.
    rviz_config = os.path.join(bridge_prefix, 'config', 'nav_view.rviz')
    src_rviz = os.path.expanduser('~/mapping_ws/src/esp32_odom_bridge/config/nav_view.rviz')
    if os.path.exists(src_rviz):
        rviz_config = src_rviz
    # Delay 12s: wait for map->odom->base_link TF to be streaming before RViz2 connects.
    # Without this delay, RViz2 can't find the 'map' frame and falls back to 'base_link'.
    delayed_rviz2 = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                output='screen',
                condition=IfCondition(LaunchConfiguration('launch_viz')),
            ),
        ]
    )

    # 6. Static TF bridges: connect the OAK ROS bridge's legacy frame names to the URDF's TF.
    # depthimage_to_laserscan's default output_frame 'camera_depth_frame' is now overridden
    # to 'oak_rgb_camera_optical_frame' above. But other consumers (rviz2, costmap debug) may
    # still reference 'camera_depth_frame' — so we link it as an identity alias of the real frame.
    camera_depth_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_depth_frame_bridge',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'oak_rgb_camera_optical_frame',
            '--child-frame-id', 'camera_depth_frame',
        ],
    )

    # Also bridge camera_color_optical_frame (used by rviz2 display of rgb image if needed)
    camera_color_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_color_frame_bridge',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'oak_rgb_camera_optical_frame',
            '--child-frame-id', 'camera_color_optical_frame',
        ],
    )

    activity_monitor_node = Node(
        package='esp32_odom_bridge',
        executable='activity_monitor',
        name='localization_activity_monitor',
        output='screen',
        parameters=[{
            'activity_type': 'localization',
            'title': 'Active RTAB-Map Localization & Scan Session',
            'database_path': LaunchConfiguration('database_path'),
        }],
    )

    map_downsampler_node = Node(
        package='esp32_odom_bridge',
        executable='map_downsampler',
        name='map_downsampler',
        output='screen',
    )

    return LaunchDescription([
        serial_port_arg,
        baud_arg,
        wheelbase_arg,
        wheel_diameter_arg,
        ticks_per_rev_arg,
        max_linear_speed_arg,
        max_angular_speed_arg,
        linear_scale_arg,
        angular_scale_arg,
        cam_x_arg,
        cam_y_arg,
        cam_z_arg,
        cam_roll_arg,
        cam_pitch_arg,
        cam_yaw_arg,
        db_path_arg,
        depth_info_arg,
        obstacle_source_arg,
        camera_model_arg,
        launch_camera_arg,
        launch_viz_arg,
        camera_depth_tf,
        camera_color_tf,
        odom_node,
        ekf_node,
        imu_tf,
        depth_to_scan_node,      # kept only for RViz / debugging
        depth_cloud_node,
        obstacles_node,
        obstacles_device_node,
        camera_urdf,
        camera_bridge,
        rtabmap_node,
        map_downsampler_node,
        delayed_rviz2,
        activity_monitor_node,
    ])
