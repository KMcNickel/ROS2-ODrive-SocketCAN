cmake_minimum_required(VERSION 3.8)
project(socketcan)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_interface REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(receiver src/socketcan_receiver.cpp)
ament_target_dependencies(receiver rclcpp can_interface)

add_executable(sender src/socketcan_sender.cpp)
ament_target_dependencies(sender rclcpp_lifecycle can_interface)

install(TARGETS
  receiver
  sender
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME})

ament_package()
