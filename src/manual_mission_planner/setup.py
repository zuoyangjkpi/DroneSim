from setuptools import setup

package_name = 'manual_mission_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'requests', 'pyyaml'],
    zip_safe=False,
    maintainer='AVIANS Team',
    maintainer_email='zuoyangjkpi@example.com',
    description='Manual mission planner that uses a large language model to generate YAML plans.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'manual_prompt_runner = manual_mission_planner.prompt_runner:main',
        ],
    },
)
