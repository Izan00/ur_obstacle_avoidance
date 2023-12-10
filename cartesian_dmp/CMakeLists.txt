cmake_minimum_required(VERSION 2.8.3)
project(cartesian_dmp)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  rospy
  geometry_msgs
  message_generation
)

find_package(Eigen REQUIRED)

catkin_python_setup()

#add_message_files()

add_service_files(
   FILES
   DMPRequest.srv
 )

generate_messages(
   DEPENDENCIES
   geometry_msgs
)

catkin_package(
CATKIN_DEPENDS common_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS nodes/cartesian_dmp_node.py scripts/custom_dmp.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#add_library(dmp src/dmp.cpp src/fourier_approx.cpp src/radial_approx.cpp src/linear_approx.cpp)
#add_executable(dmp_server nodes/dmp_server.cpp)
#add_dependencies(dmp
#                 ${catkin_EXPORTED_TARGETS}
#                 dmp_gencpp
#                 dmp_generate_messages
#                 dmp_generate_messages_cpp
#                 dmp_generate_messages_py
#                 dmp_generate_messages_lisp)
#target_link_libraries(dmp_server dmp ${catkin_LIBRARIES} ${Eigen_LIBRARIES})
