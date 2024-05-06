cmake_minimum_required(VERSION 3.0.2)
project(offboard_exec)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_BUILD_TYPE "RelWithDebInfo")
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(cmake_modules REQUIRED)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS geometry_msgs mavros_msgs nav_msgs pluginlib rosconsole_bridge roscpp sensor_msgs std_msgs tf quadrotor_msgs)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES offboard_exec
  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

#add_definitions(${EIGEN_DEFINITIONS})
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include ${catkin_INCLUDE_DIRS}
    SYSTEM
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIR}
)

link_directories(
        ${catkin_LIB_DIRS} lib
)

add_executable(offb_node3
  src/offb_node.cpp
)
add_dependencies(offb_node3 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(offb_node3
  ${catkin_LIBRARIES}
  liboffboard_release.so
)

