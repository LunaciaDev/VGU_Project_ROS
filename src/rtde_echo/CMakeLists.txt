cmake_minimum_required(VERSION 3.0.2)
project(rtde_echo)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   message_generation
)

# System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

###################################
##            message            ##
###################################

add_message_files(
   FILES
   RtdeData.msg
)

generate_messages()

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
   CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})
add_executable(rtde_echo src/main.cpp)
add_dependencies(rtde_echo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rtde_echo
   ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS rtde_echo DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
