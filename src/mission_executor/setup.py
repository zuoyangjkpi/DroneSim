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
    maintainer='Soja',
    maintainer_email='zuoyangjkpi@gmail.com',
    description='Behavior-tree style mission executor that consumes YAML plans and dispatches mission action modules.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_executor_node = mission_executor.mission_executor_node:main',
        ],
    },
)
