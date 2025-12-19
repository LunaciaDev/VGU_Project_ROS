cmake_minimum_required(VERSION 3.0.2)
project(ros_unity_bridge)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   ros_unity_messages
   rtde_echo
   moveit_ros_planning_interface
   moveit_ros_warehouse
   rviz_visual_tools
)

# System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pickandplace_project
   CATKIN_DEPENDS ros_unity_messages rtde_echo
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})
add_executable(unity_bridge src/main.cpp src/unity_targets_listener.cpp src/dynamic_object_listener.cpp)
add_dependencies(unity_bridge
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(unity_bridge
   ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS unity_bridge DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})