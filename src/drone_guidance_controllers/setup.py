import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'drone_guidance_controllers'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=False,
    maintainer='zuoyangjkpi',
    maintainer_email='zuoyang0601@gmail.com',
    description='Guidance-layer controllers (waypoint & yaw) for the drone stack.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_controller = drone_guidance_controllers.waypoint_controller:main',
            'yaw_controller = drone_guidance_controllers.yaw_controller:main',
        ],
    },
)
