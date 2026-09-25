from setuptools import find_packages, setup

package_name = 'sage_px4_interface'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    scripts=[
        'sage_px4_interface/yolo_detector_launcher.py',
    ],
    maintainer='Haidar Saad',
    maintainer_email='117441256+haidar996@users.noreply.github.com',
    description='SAGE-UAV: mission-driven autonomous person search for a PX4 quadrotor',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'offboard_position_node = sage_px4_interface.offboard_position_node:main',
        'camera_test = sage_px4_interface.camera_test:main',
        'sage_target_localizer = sage_px4_interface.sage_target_localizer:main',
        'sage_semantic_world_model = sage_px4_interface.sage_semantic_world_model:main',
        'sage_viewpoint_planner = sage_px4_interface.sage_viewpoint_planner:main',
        'sage_mission_manager = sage_px4_interface.sage_mission_manager:main',
        'sage_energy_monitor = sage_px4_interface.sage_energy_monitor:main',
        'sage_mission_parser = sage_px4_interface.sage_mission_parser:main',
    ],
},
)
