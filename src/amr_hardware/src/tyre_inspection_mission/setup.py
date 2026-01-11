from setuptools import setup
from glob import glob
import os

package_name = 'tyre_inspection_mission'

setup(
    name='tyre-inspection-mission',  # Use hyphens for distribution name to match entry point scripts
    version='0.0.0',
    packages=[
        package_name,
        f'{package_name}.core',
        f'{package_name}.detection',
        f'{package_name}.navigation',
        f'{package_name}.capture',
        f'{package_name}.common',
        f'{package_name}.diagnostics',
        f'{package_name}.scripts',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            [os.path.join('resource', package_name)]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py') if os.path.exists('launch') else []),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') if os.path.exists('config') else []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='Autonomous tyre inspection mission controller',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_controller = tyre_inspection_mission.core.mission_controller:main',
            'photo_capture_service = tyre_inspection_mission.capture.photo_capture:main',
            'diagnostic_check = tyre_inspection_mission.scripts.diagnostic_check:main',
            'mission_monitor = tyre_inspection_mission.diagnostics.mission_monitor:main',
            'mission_status = tyre_inspection_mission.scripts.mission_status:main',
            'mission_log_recorder = tyre_inspection_mission.diagnostics.mission_logger:main',
            'analyze_mission_log = tyre_inspection_mission.scripts.analyze_mission_log:main',
            'set_log_levels = tyre_inspection_mission.scripts.set_log_levels:set_log_levels',
            'visualize_detections = tyre_inspection_mission.scripts.visualize_detections:main',
            'save_annotated_images = tyre_inspection_mission.scripts.save_annotated_images:main',
            'image_http_server = tyre_inspection_mission.scripts.image_http_server:main',
            'camera_recovery = tyre_inspection_mission.diagnostics.camera_recovery:main',
            'movement_diagnostic = tyre_inspection_mission.diagnostics.movement_diagnostic:main',
            'command_effectiveness_monitor = tyre_inspection_mission.diagnostics.command_effectiveness_monitor:main',
            'cmd_vel_multiplexer = tyre_inspection_mission.navigation.cmd_vel_multiplexer:main',
            'nav2_cmd_vel_relay = tyre_inspection_mission.navigation.nav2_cmd_vel_relay:main',
        ],
    },
)
