from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'car_detection_system'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'depthai',
        'numpy',
        'opencv-python',
        'blobconverter',
        'gpiozero',
    ],
    zip_safe=False,
    maintainer='user',
    maintainer_email='user@todo.com',
    description='Car detection system using OAK-D Lite and LiDAR with ROS 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detection_node = car_detection_system.detection_node:main',
        ],
    },
) 