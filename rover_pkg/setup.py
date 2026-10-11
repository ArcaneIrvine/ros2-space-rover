from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rover_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/rover_pkg']),
        ('share/rover_pkg', ['package.xml']),
        (os.path.join('share', 'rover_pkg', 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', 'rover_pkg', 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='stavrose',
    maintainer_email='stavrose@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'motor_driver_node = rover_pkg.motor_driver_node:main',
        'ultrasonic_node   = rover_pkg.ultrasonic_node:main',
        'imu_node          = rover_pkg.imu_node:main',
        'camera_node       = rover_pkg.camera_node:main',
        'odometry_node     = rover_pkg.odometry_node:main',
	'safety_node = rover_pkg.safety_node:main',
	'mission_node = rover_pkg.mission_node:main',
	'depth_node = rover_pkg.depth_node:main',
	'camera_info_node = rover_pkg.camera_info_node:main',
      ],
    },
)
