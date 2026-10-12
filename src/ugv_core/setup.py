from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'ugv_core'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anshu',
    maintainer_email='you@example.com',
    description='ROS-independent algorithms: EKF, kinematics, PID, odometry, navigation law',
    license='MIT',
    tests_require=['pytest'],
)
