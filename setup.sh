#!/bin/bash

sudo apt-get update
sudo apt-get install ros-noetic-ddynamic-reconfigure -y
sudo apt-get install ros-noetic-realsense2-camera -y
sudo apt-get install ros-noetic-ur-gazebo ros-noetic-ur3-moveit-config -y
sudo apt-get install ros-noetic-ros-numpy -y
sudo apt-get install ros-noetic-tf2-sensor-msgs -y
sudo apt-get install ros-noetic-sensor-filters -y
sudo apt-get install python3-pcl -y
sudo apt-get install ros-noetic-moveit -y
#sudo apt-get install libcgal-dev -y
sudo apt install xdotool -y

pip install -r requirements.txt

rosdep install --from-paths src --ignore-src --rosdistro noetic -y --skip-keys dmp_generate_messages

