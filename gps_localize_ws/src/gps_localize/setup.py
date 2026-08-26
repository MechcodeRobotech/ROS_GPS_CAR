import os
from glob import glob

from setuptools import setup

package_name = 'gps_localize'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'web'), glob('web/*')),
        (os.path.join('share', package_name, 'systemd'), glob('systemd/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='GPS_Localize',
    maintainer_email='manman7852078520@gmail.com',
    description='GPS waypoint robot: navigation, safety watchdog and local web UI.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_nav = gps_localize.waypoint_nav:main',
            'gps_odom = gps_localize.gps_odom:main',
            'safety_watchdog = gps_localize.safety_watchdog:main',
            'web_server = gps_localize.web_server:main',
            'heading_check = gps_localize.heading_check:main',
        ],
    },
)
