from setuptools import find_packages, setup

package_name = 'bme_gazebo_sensors_py'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='David Dudas',
    maintainer_email='david.dudas@outlook.com',
    description='Python nodes for simulation of various sensors with Gazebo Harmonic and ROS Jazzy for BME MOGI ROS2 course',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_republisher = bme_gazebo_sensors_py.image_republisher:main',
            'gps_waypoint_follower = bme_gazebo_sensors_py.gps_waypoint_follower:main',
        ],
    },
)
