[//]: # (Image References)

[image1]: ./assets/gazebo.png "Shapes.sdf"
[image2]: ./assets/gazebo-1.png "Gazebo GUI"
[image3]: ./assets/gazebo-2.png "Gazebo models"


# Week 5-6: Gazebo sensors

## This is how far we will get by the end of this lesson: 
  <a href="https://youtu.be/0Xokl5dHRoQ"><img width="600" src="./assets/youtube-gazebo.png"></a>  

  <a href="https://youtu.be/ELwRqeNR_NA"><img width="600" src="./assets/youtube-gazebo-1.png"></a>  


# Table of Contents
1. [Download ROS package](#download-ros-package)
2. [Camera](#install-gazebo)  
2.1. [Image transport](#diff-drive-plugin)  
2.2. [rqt reconfigure](#diff-drive-plugin)  
2.3. [Wide angle camera](#diff-drive-plugin) 
3. [IMU](#download-ros-package)  
4. [GPS](#creating-a-gazebo-world)  
4.1. [Haversine formula](#diff-drive-plugin)  
4.2. [GPS waypoint following](#diff-drive-plugin) 
5. [Lidar](#urdf)  
5.1. [3D lidar](#diff-drive-plugin) 
6. [RGBD camera](#gazebo-integration)  
7. [Image processing with OpenCV](#3d-models)  


# Download ROS package




---






sudo apt install ros-jazzy-topic-tools

sudo apt install ros-jazzy-rviz-imu-plugin
sudo apt install ros-jazzy-rviz-satellite


GZ examples:
https://github.com/gazebosim/gz-sim/tree/gz-sim8/examples/worlds

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

