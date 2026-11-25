import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    depthai_prefix = get_package_share_directory('depthai_descriptions_v3')
    bridge_prefix = get_package_share_directory('esp32_odom_bridge')

    # ESP32 teleop bridge parameters
    serial_port_arg = DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0')
    baud_arg = DeclareLaunchArgument('baud', default_value='115200')
    wheelbase_arg = DeclareLaunchArgument('wheelbase', default_value='0.5458')   # calibrated (firmware + localization use 0.5458; 0.56 skewed yaw 2.6 %)
    wheel_diameter_arg = DeclareLaunchArgument('wheel_diameter', default_value='0.127')
    ticks_per_rev_arg = DeclareLaunchArgument('ticks_per_rev', default_value='49415')
    # Slow, steady speed limits for SLAM mapping to prevent motion blur and dropped frames
    max_linear_speed_arg = DeclareLaunchArgument('max_linear_speed', default_value='0.12')   # 0.12 m/s (ultra-gentle mapping creep)
    max_angular_speed_arg = DeclareLaunchArgument('max_angular_speed', default_value='0.25')  # 0.25 rad/s (~14 deg/s smooth turning)
    # 1.0 = commanded speed is sent as-is (caps above still apply)
    linear_scale_arg = DeclareLaunchArgument('linear_scale', default_value='1.0')
    angular_scale_arg = DeclareLaunchArgument('angular_scale', default_value='1.0')

    # Camera mount offset relative to base_link (calibrated from measurements and IMU)
    cam_x_arg = DeclareLaunchArgument('cam_x', default_value='0.25')     # measured: camera 25 cm ahead of drive axle
    cam_y_arg = DeclareLaunchArgument('cam_y', default_value='0.0')      # centered
    cam_z_arg = DeclareLaunchArgument('cam_z', default_value='0.795')   # 2026-11-04: cam_level 0.794, tape ~0.80
    cam_roll_arg = DeclareLaunchArgument('cam_roll', default_value='0.005')
    cam_pitch_arg = DeclareLaunchArgument('cam_pitch', default_value='0.121')   # set by: ros2 run esp32_odom_bridge imu_tilt --apply
    cam_yaw_arg = DeclareLaunchArgument('cam_yaw', default_value='0.0')

    # Database path and reset option
    database_path_arg = DeclareLaunchArgument(
        'database_path',
        default_value=os.path.expanduser('~/.ros/rtabmap.db'),
        description='Database path for RTAB-Map'
    )
    delete_db_arg = DeclareLaunchArgument(
        'delete_db',
        default_value='true',
        description='Start a FRESH map (default). delete_db:=false appends a new session to the old db'
    )
    delete_db_action = ExecuteProcess(
        cmd=['rm', '-f', os.path.expanduser('~/.ros/rtabmap.db')],
        condition=IfCondition(LaunchConfiguration('delete_db'))
    )

    # Subsystem toggles
    launch_camera_arg = DeclareLaunchArgument('launch_camera', default_value='true')
    launch_rtabmap_arg = DeclareLaunchArgument('launch_rtabmap', default_value='true')
    launch_viz_arg = DeclareLaunchArgument('launch_viz', default_value='true')

    # ---------------------------------------------------------------------------
    # 1. ESP32 Bridge — TELEOP ONLY (forwards cmd_vel to motors, publishes wheel odom
    #    on /wheel_odom so it does NOT conflict with visual odometry on /odom)
    # ---------------------------------------------------------------------------
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
            'publish_tf': False,           # DO NOT publish TF — visual odom owns odom→base_link
        }],
        remappings=[
            ('odom', '/wheel_odom'),       # Publish wheel odom on separate topic (not used for SLAM)
        ],
    )

    # ---------------------------------------------------------------------------
    # 2. Robot Localization EKF (Fuses wheel odometry + camera IMU)
    #    Publishes /odometry/filtered and the authoritative odom -> base_link TF.
    # ---------------------------------------------------------------------------
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

    # ---------------------------------------------------------------------------
    # 3. OAK-D Camera Driver (URDF for TF + Python bridge for streaming)
    # ---------------------------------------------------------------------------
    camera_model_arg = DeclareLaunchArgument('camera_model', default_value='OAK-D-LITE',
                                             description='OAK-D or OAK-D-LITE (URDF frames)')

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
        respawn=True,          # OAK X_LINK crash -> restart the bridge in 3 s instead of losing the camera
        respawn_delay=3.0,
    )

    # ---------------------------------------------------------------------------
    # 4. RTAB-Map SLAM Node — Multi-Lap High Precision Mapping
    # ---------------------------------------------------------------------------
    rtabmap_parameters = [{
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'database_path': LaunchConfiguration('database_path'),
        'subscribe_odom': True,
        'subscribe_rgb': True,
        'subscribe_depth': True,
        'subscribe_odom_info': False,
        'subscribe_imu': False,            # EKF already fuses IMU into /odom & TF (eliminates TF wait delay)
        'approx_sync': True,
        'approx_sync_max_interval': 0.05,
        'queue_size': 30,
        'gen_scan': False,
    }]

    rtabmap_arguments = [
        # --- Pose Constraints ---
        '--Reg/Force3DoF', 'true',                  # Strictly flat 2D ground floor (x, y, yaw): NO tilted walls
        # --- Registration: Visual PnP (robust over multi-lap drift) ---
        '--RGBD/MarkerDetection', 'false',          # ArUco landmarks: set 'true' once markers are on the walls
        '--Marker/Dictionary', '0',                 # 0 = DICT_4X4_50 (printed sheet)
        '--Marker/Length', '0.16',                  # black square size in metres (measure the print!)
        '--Marker/MaxDepthError', '0.05',
        '--Reg/Strategy', '0',                      # 0 = Visual PnP (matches appearance directly across laps)
        '--Vis/MinInliers', '12',                   # was 8: only solid links in the map graph
        '--Vis/EstimationType', '1',                # 3D->2D PnP
        '--Vis/PnPReprojError', '4.5',              # Realistic reprojection error tolerance for stereo depth
        '--Vis/MaxFeatures', '1200',                # Rich feature extraction for sharp visual alignment
        '--Kp/MaxFeatures', '1000',                 # more loop-closure words -> relocalizes in feature-poor spots
        '--Vis/BundleAdjustment', '1',              # Local bundle adjustment to lock consecutive frames together
        '--Vis/RefineIterations', '5',              # 5 refinement iterations for sub-pixel accuracy
        # --- Multi-Lap Loop Closure Tuning ---
        '--Rtabmap/DetectionRate', '2.0',           # 2.0 Hz keyframe processing (responsive loop closure)
        '--Rtabmap/LoopThr', '0.11',                # Standard responsive loop closure threshold
        '--RGBD/LinearUpdate', '0.08',              # Move >= 8 cm before creating node
        '--RGBD/AngularUpdate', '0.08',             # Rotate >= 4.5 deg before creating node
        '--RGBD/NeighborLinkRefining', 'true',      # CRITICAL: Visually lock consecutive frames to eliminate ghosting
        '--RGBD/OptimizeMaxError', '3.0',           # was 5.0: reject loop closures that bend the graph (smeared/double walls)
        '--RGBD/ProximityBySpace', 'true',          # Local loop closures by spatial proximity
        '--RGBD/ProximityMaxGraphDepth', '100',     # Search up to 100 nodes deep for proximity closures
        '--RGBD/ProximityPathMaxNeighbors', '10',   # Consider 10 nearby nodes for proximity matching
        '--RGBD/LoopClosureReextractFeatures', 'true',  # Re-extract features at closure time
        '--Mem/ImageKept', 'true',                  # CRITICAL: Retain images in RAM so feature & depth re-extraction succeeds
        '--Mem/RawDescriptorsKept', 'true',
        '--Mem/RehearsalSimilarity', '1.0',         # was 0.45: NEVER merge/drop frames - keep every keyframe for offline reprocessing
        '--Mem/BinDataKept', 'true',                # store RGB + depth of every node -> map can be rebuilt offline with new params
        '--Mem/NotLinkedNodesKept', 'true',
        '--Mem/InitWMWithAllNodes', 'true',         # Load previous round nodes into WM for instant relocalization
        # --- Occupancy Grid (Sharp, crisp boundaries without stereo noise) ---
        '--Grid/FromDepth', 'true',
        '--Grid/NormalsSegmentation', 'true',       # Normal vector segmentation for floor vs obstacles
        '--Grid/MaxGroundAngle', '45',
        '--Grid/MaxGroundHeight', '0.08',           # Points <= 8cm classified as floor, not obstacle walls
        '--Grid/MinGroundHeight', '-0.10',          # 2-level venue: a LOWER floor seen over an edge is ignored (stays unknown), never marked as free floor
        '--Grid/CellSize', '0.03',                  # 3 cm high-resolution grid
        '--Grid/RangeMin', '0.35',
        '--Grid/RangeMax', '2.5',                   # was 3.2: OAK-D Lite depth error ~+-6 cm at 3 m thickened walls
        '--Grid/RayTracing', 'true',                # Clear dynamic space behind moved objects
        '--Grid/NoiseFilteringRadius', '0.05',      # 2026-11: remove isolated specks (they blocked corridors)
        '--Grid/NoiseFilteringMinNeighbors', '3',   # a real obstacle has >= 3 neighbours within 5 cm
        '--Grid/MaxObstacleHeight', '1.60',         # same height band as Nav2 costmaps
        '--Grid/MinClusterSize', '10',              # Prevent dropping sparse obstacle clusters
        '--Grid/PreVoxelFiltering', 'true',         # Pre-voxel filtering for clean grid
        '--Grid/ClusterRadius', '0.10',             # Discard clusters smaller than 10cm
        '--Grid/FlatObstacleDetected', 'true',
        '--Rtabmap/MaxRepublished', '10',
    ]

    rtabmap_remappings = [
        ('rgb/image', '/oak/rgb/image_raw'),
        ('rgb/camera_info', '/oak/rgb/camera_info'),
        ('depth/image', '/oak/stereo/image_raw'),
        ('odom', '/odom'),
    ]

    rtabmap_slam_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_rtabmap')),
        parameters=rtabmap_parameters,
        arguments=rtabmap_arguments,
        remappings=rtabmap_remappings,
    )

    # ---------------------------------------------------------------------------
    # 5. RTAB-Map 3D/2D Visualizer GUI
    # ---------------------------------------------------------------------------
    rtabmap_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_viz')),
        parameters=rtabmap_parameters,
        arguments=rtabmap_arguments,
        remappings=rtabmap_remappings,
    )

    activity_monitor_node = Node(
        package='esp32_odom_bridge',
        executable='activity_monitor',
        name='mapping_activity_monitor',
        output='screen',
        parameters=[{
            'activity_type': 'mapping',
            'title': 'SLAM 3D/2D Mapping Session',
            'database_path': LaunchConfiguration('database_path'),
        }],
    )

    return LaunchDescription([
        camera_model_arg,
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
        database_path_arg,
        delete_db_arg,
        delete_db_action,
        launch_camera_arg,
        launch_rtabmap_arg,
        launch_viz_arg,
        odom_node,
        ekf_node,
        imu_tf,
        camera_urdf,
        camera_bridge,
        rtabmap_slam_node,
        rtabmap_viz_node,
        activity_monitor_node,
    ])
