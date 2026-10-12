from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'ugv_sensors'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anshu',
    maintainer_email='you@example.com',
    description='Stage 1: encoder wheel odometry and simulated GPS for the UGV',
    license='MIT',
    entry_points={
        'console_scripts': [
            'wheel_odometry_node = ugv_sensors.wheel_odometry_node:main',
            'gps_node = ugv_sensors.gps_node:main',
        ],
    },
)
