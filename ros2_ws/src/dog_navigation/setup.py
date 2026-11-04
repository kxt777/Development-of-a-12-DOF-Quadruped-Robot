from glob import glob
from setuptools import setup

setup(
    name='dog_navigation', version='0.1.0',
    packages=['dog_navigation'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/dog_navigation']),
        ('share/dog_navigation', ['package.xml']),
        ('share/dog_navigation/launch', glob('launch/*.launch.py')),
        ('share/dog_navigation/config', glob('config/*.json') + glob('config/*.rviz') + glob('config/*.yaml') + glob('config/*.xml')),
        ('share/dog_navigation/scripts', glob('scripts/*.sh')),
    ],
    entry_points={'console_scripts': [
        'dog_lidar_audit = dog_navigation.lidar_audit:main',
        'dog_lidar_config = dog_navigation.lidar_config:main',
        'dog_saved_map = dog_navigation.saved_map:main',
        'dog_plan_probe = dog_navigation.plan_probe:main',
        'dog_nav_velocity_bridge = dog_navigation.velocity_bridge:main',
        'dog_nav_route = dog_navigation.route:main',
        'dog_nav_fixture = dog_navigation.fixture:main',
        'dog_nav_odometry = dog_navigation.odometry:main',
        'dog_nav_alignment = dog_navigation.alignment:main',
    ]},
)
