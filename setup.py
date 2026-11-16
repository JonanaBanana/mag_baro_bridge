from glob import glob
import os

from setuptools import setup

package_name = 'mag_baro_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='airlab',
    description='PX4 mag/baro from MAVLink HIGHRES_IMU to ROS 2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mag_baro_node = mag_baro_bridge.mag_baro_node:main',
        ],
    },
)
