from setuptools import setup
import sys

package_name = 'manual_mission_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    options={
        'build_scripts': {
            'executable': '/usr/bin/env python3',
        },
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'openai', 'pyyaml'],
    zip_safe=False,
    maintainer='Yang Zuo',
    maintainer_email='zuoyang0601@gmail.com',
    description='Manual mission planner that uses a large language model to generate YAML plans.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'manual_prompt_runner = manual_mission_planner.prompt_runner:main',
        ],
    },
)
