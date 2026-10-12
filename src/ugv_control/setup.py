from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'ugv_control'

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
    description='Stages 4-5: four-wheel differential-drive kinematics and per-wheel PID control',
    license='MIT',
    entry_points={
        'console_scripts': [
            'kinematics_node = ugv_control.kinematics_node:main',
            'wheel_pid_node = ugv_control.wheel_pid_node:main',
            'pid_step_test = ugv_control.pid_step_test:main',
            'slip_calibration = ugv_control.slip_calibration:main',
        ],
    },
)
