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
pip install numpy==1.20.3
pip install -U scikit-learn
pip install open3d
pip install pyvista
pip install pymeshfix

rosdep install --from-paths src --ignore-src --rosdistro noetic -y --skip-keys dmp_generate_messages

#roslaunch roy_dmp ur3_gazebo_with_dmp.launch
#roslaunch realsense2_description view_d435i_model_rviz_gazebo.launch

