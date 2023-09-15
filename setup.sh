#!/bin/bash

sudo apt-get update
sudo apt-get install ros-noetic-ddynamic-reconfigure -y
sudo apt-get install ros-noetic-realsense2-camera -y
sudo apt-get install ros-noetic-ur-gazebo ros-noetic-ur3-moveit-config -y

#cd src/

#git clone --recurse-submodules https://github.com/rickstaa/realsense-ros-gazebo.git src
#git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver
#git clone -b melodic-devel https://github.com/ros-industrial/universal_robot.git src/universal_robot
#git clone https://github.com/sniekum/dmp

#cd ..

rosdep install --from-paths src --ignore-src --rosdistro noetic -y
#rosdep install --from-paths src --ignore-src -y

#roslaunch roy_dmp ur3_gazebo_with_dmp.launch
#roslaunch realsense2_description view_d435i_model_rviz_gazebo.launch

