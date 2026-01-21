cmake_minimum_required(VERSION 3.8)
project(amr_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(san_msgs REQUIRED)

include_directories(include)
include_directories(${Eigen3_INCLUDE_DIRS})

add_executable(swerve_controller_node
  src/swerve_controller/swerve_controller_node.cpp
  src/swerve_controller/drive_module.cpp
  src/swerve_controller/control_model.cpp
)

ament_target_dependencies(swerve_controller_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  Eigen3
  san_msgs
)

install(TARGETS swerve_controller_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch rviz param data description robots worlds
  DESTINATION share/${PROJECT_NAME})

ament_package()
