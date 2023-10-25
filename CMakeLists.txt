cmake_minimum_required(VERSION 3.0.2)
project(offboard_sample)

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
  LIBRARIES offboard_sample
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

add_library(offboard_sample
  src/pos_controller.cpp
  src/offboard_fsm.cpp
  src/pid.cpp
)
add_dependencies(offboard_sample ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(offboard_sample
  ${catkin_LIBRARIES}
)

add_executable(offb_node
  include/offboard_sample/darc_height_controller.h
  src/darc_height_controller.cpp
  src/offb_node.cpp
  src/pos_controller.cpp
  src/offboard_fsm.cpp
  src/pid.cpp
  src/traj_fsm.cpp
)
add_dependencies(offb_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(offb_node
  ${catkin_LIBRARIES}
)


add_executable(offb_att
  src/PX4CtrlParam.cpp 
  src/controller.cpp 
  src/traj_fsm_att.cpp
  src/offboard_att.cpp
  src/input.cpp
  
)
add_dependencies(offb_att ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(offb_att
  ${catkin_LIBRARIES}
)


add_executable(offbvel_node
  include/offboard_sample/darc_height_controller.h
  src/darc_height_controller.cpp
  src/offbvel_node.cpp
  src/pos_controller.cpp
  src/offboard_fsm.cpp
  src/pid.cpp
  src/vel_fsm_test.cpp
)
add_dependencies(offbvel_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(offbvel_node
  ${catkin_LIBRARIES}
)

add_executable(offbvel_node_sim
  include/offboard_sample/darc_height_controller.h
  src/darc_height_controller.cpp
  src/offbvel_node_sim.cpp
  src/pos_controller.cpp
  src/offboard_fsm.cpp
  src/pid.cpp
  src/vel_fsm.cpp
)
add_dependencies(offbvel_node_sim ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(offbvel_node_sim
  ${catkin_LIBRARIES}
)

