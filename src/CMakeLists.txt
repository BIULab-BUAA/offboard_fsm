cmake_minimum_required(VERSION 3.5)
project(offboard_sample)

# # Default to C99
# if(NOT CMAKE_C_STANDARD)
#   set(CMAKE_C_STANDARD 99)
# endif()

# # Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 17)
# endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(mavros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(YAML-CPP REQUIRED)


include_directories(include)

add_executable(offb_node src/traj_fsm.cpp)
ament_target_dependencies(offb_node rclcpp mavros_msgs visualization_msgs quadrotor_msgs  mavros geometry_msgs nav_msgs sensor_msgs std_msgs tf2)
install(TARGETS offb_node DESTINATION lib/${PROJECT_NAME})



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(PROGRAMS
  scripts/key_board2.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime  Eigen3  rclcpp mavros_msgs visualization_msgs quadrotor_msgs mavros geometry_msgs nav_msgs sensor_msgs std_msgs tf2)

ament_package()
