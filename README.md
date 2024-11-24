# Week-5-6-Gazebo-sensors

sudo apt install ros-jazzy-topic-tools

sudo apt install ros-jazzy-rviz-imu-plugin
sudo apt install ros-jazzy-rviz-satellite


relay camera info topic:
sudo apt install ros-jazzy-topic-tools

SDF reference:
http://sdformat.org/spec?elem=sensor&ver=1.6

ros_gz_bridge message types:
https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge


IMU plugin:
sudo apt install ros-jazzy-rviz-imu-plugin

WORLD plugin!!!!

GPS:

OpenStreetMap
https://www.openstreetmap.org/search?lat=47.479099&lon=19.057811


sudo apt install ros-jazzy-rviz-satellite
https://github.com/nobleo/rviz_satellite

WORLD plugin!!!!

sudo apt install ros-jazzy-tf-transformations

ros2 launch bme_gazebo_sensors spawn_robot.launch.py world:=empty.sdf rviz_config:=gps.rviz x:=0.0 y:=0.0 yaw:=0.0


EKF:
sudo apt install ros-jazzy-robot-localization

To see who is the publisher:
ros2 topic info /odom --verbose

Reload URDF:

david@david-ubuntu24:~$ ros2 param set /robot_state_publisher robot_description "$(xacro $(ros2 pkg prefix bme_gazebo_sensors)/share/bme_gazebo_sensors/urdf/mogi_bot.urdf)"
Set parameter successful

OpenCV

sudo apt install ros-jazzy-cv-bridge

