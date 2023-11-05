cmake_minimum_required(VERSION 3.0.2)
project(obstacle_avoidance)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  rospy
  roscpp
  pcl_ros
  tf2
  tf2_ros
  tf2_sensor_msgs
  geometry_msgs
  sensor_msgs
  
  std_msgs
  moveit_core
  moveit_ros_planning_interface
  moveit_ros_planning
  visualization_msgs
)

find_package(PCL 1.7 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(VTK REQUIRED)
#find_package(CGAL REQUIRED)
#find_package(GMP REQUIRED)

include(${VTK_USE_FILE})

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs tf2_ros std_msgs sensor_msgs moveit_core
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(workspace_filter src/workspace_filter.cpp)
target_link_libraries(workspace_filter ${catkin_LIBRARIES})

add_executable(safety_check src/safety_check.cpp)
target_link_libraries(safety_check ${catkin_LIBRARIES} 
                                     ${PCL_LIBRARIES} 
                                     ${Boost_LIBRARIES} 
                                     ${VTK_LIBRARIES} 
                                     )

add_executable(collsion_volume_reconstruction src/collsion_volume_reconstruction.cpp)
target_link_libraries(collsion_volume_reconstruction ${catkin_LIBRARIES} 
                                                     ${PCL_LIBRARIES} 
                                                     ${Boost_LIBRARIES} 
                                                     )

