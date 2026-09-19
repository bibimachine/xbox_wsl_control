cmake_minimum_required(VERSION 3.8)
project(joint_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gamepad_msgs REQUIRED)

add_executable(joint_teleop src/joint_teleop.cpp)
ament_target_dependencies(joint_teleop rclcpp control_msgs std_msgs gamepad_msgs)

install(TARGETS joint_teleop
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
