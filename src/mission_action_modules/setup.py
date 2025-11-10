from setuptools import setup

package_name = 'mission_action_modules'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=False,
    maintainer='AVIANS Team',
    maintainer_email='zuoyangjkpi@example.com',
    description='Action modules for high-level mission execution, interfacing with low-level controllers.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'action_manager = mission_action_modules.action_manager_node:main',
            'mission_sequence_controller = mission_action_modules.mission_sequence_controller:main',
            'waypoint_test_runner = mission_action_modules.waypoint_test_runner:main',
        ],
    },
)
