from setuptools import setup

package_name = 'mission_executor'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=False,
    maintainer='Yang Zuo',
    maintainer_email='zuoyang0601@gmail.com',
    description='Behavior-tree style mission executor that consumes YAML plans and dispatches mission action modules.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_executor_node = mission_executor.mission_executor_node:main',
        ],
    },
)
