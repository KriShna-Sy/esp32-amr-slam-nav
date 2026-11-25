from glob import glob
from setuptools import setup, find_packages

package_name = 'esp32_odom_bridge'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/firmware', glob('firmware/*.ino')),
        ('share/' + package_name + '/firmware/amr_base_pid', glob('firmware/amr_base_pid/*.ino')),
        # shell/venv scripts -> runnable with `ros2 run esp32_odom_bridge <script>`
        ('lib/' + package_name, glob('scripts/*')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='krishna',
    maintainer_email='siddapuchinnikrishna@gmail.com',
    description='Diff-drive AMR (ESP32 + OAK-D Lite): odometry bridge, RTAB-Map SLAM, '
                'landmark navigation with 3D obstacle avoidance, calibration tools, firmware',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # runtime nodes
            'odom_bridge_node = esp32_odom_bridge.odom_bridge_node:main',
            'landmark_manager = esp32_odom_bridge.landmark_manager:main',
            'activity_monitor = esp32_odom_bridge.activity_monitor:main',
            'responsive_teleop = esp32_odom_bridge.responsive_teleop:main',
            'view_logs = esp32_odom_bridge.view_logs:main',
            # calibration / diagnostics
            'calibrate_odometry = esp32_odom_bridge.tools.calibrate_odometry:main',
            'calibrate_ticks = esp32_odom_bridge.tools.calibrate_ticks:main',
            'live_ticks = esp32_odom_bridge.tools.live_ticks:main',
            'motor_diagnostics = esp32_odom_bridge.tools.motor_diagnostics:main',
            'yaw_check = esp32_odom_bridge.tools.yaw_check:main',
            'fw_test = esp32_odom_bridge.tools.fw_test:main',
            'dir_test = esp32_odom_bridge.tools.dir_test:main',
            'nav_diag = esp32_odom_bridge.tools.nav_diag:main',
            'costmap_snapshot = esp32_odom_bridge.tools.costmap_snapshot:main',
            'map_downsampler = esp32_odom_bridge.map_downsampler:main',
            'cam_level = esp32_odom_bridge.tools.cam_level:main',
            'imu_tilt = esp32_odom_bridge.tools.imu_tilt:main',
            'keepout = esp32_odom_bridge.tools.keepout:main',
        ],
    },
)
