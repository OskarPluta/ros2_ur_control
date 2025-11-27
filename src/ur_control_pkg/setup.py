from setuptools import setup
import os
from glob import glob

package_name = 'ur_control_pkg'

setup(
    name=package_name,
    version='1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Oskar',
    maintainer_email='oskarpluta1906@gmail.com',
    description='Projekt ROS2 UR',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ur_node = ur_control_pkg.ur_node:main',
        ],
    },
)
