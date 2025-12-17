from setuptools import setup

package_name = 'drone_tf_publisher'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yang Zuo',
    maintainer_email='zuoyang0601@gmail.com',
    description='TF broadcaster for the X3 drone built on /X3/odometry.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_tf_publisher = drone_tf_publisher.drone_tf_publisher:main',
        ],
    },
)
