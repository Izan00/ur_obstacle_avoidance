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
  eigen_conversions
  std_msgs
  moveit_core
  moveit_ros_planning_interface
  moveit_ros_planning
  visualization_msgs
  dmp
  message_generation
)

#add_message_files(
#  FILES
#)

generate_messages(
  DEPENDENCIES
  std_msgs
  trajectory_msgs
  geometry_msgs
)

find_package(PCL 1.7 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(VTK REQUIRED)
#find_package(CGAL REQUIRED)
#find_package(GMP REQUIRED)

include(${VTK_USE_FILE})

catkin_package(
  CATKIN_DEPENDS roscpp dmp sensor_msgs tf2_ros std_msgs sensor_msgs moveit_core
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(workspace_filter src/workspace_filter.cpp)
target_link_libraries(workspace_filter ${catkin_LIBRARIES})

add_executable(safety_check src/safety_check.cpp)
target_link_libraries(safety_check ${catkin_LIBRARIES} 
                                     ${PCL_LIBRARIES} 
                                     ${Boost_LIBRARIES} 
                                     ${VTK_LIBRARIES} 
                                     )

add_executable(collision_volume_reconstruction src/collision_volume_reconstruction.cpp)
target_link_libraries(collision_volume_reconstruction ${catkin_LIBRARIES} 
                                                     ${PCL_LIBRARIES} 
                                                     ${Boost_LIBRARIES} 
                                                     )

add_executable(collision_check src/collision_check.cpp)
target_link_libraries(collision_check ${catkin_LIBRARIES})
add_dependencies(collision_check ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#add_executable(move_test src/move_test.cpp)
#target_link_libraries(move_test ${catkin_LIBRARIES})
#add_dependencies(move_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#add_executable(dmp_move src/dmp_move.cpp)
#target_link_libraries(dmp_move ${catkin_LIBRARIES})