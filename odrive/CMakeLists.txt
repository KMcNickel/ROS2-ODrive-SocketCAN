cmake_minimum_required(VERSION 3.8)
project(odrive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(can_interface REQUIRED)
find_package(odrive_interface REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(odrive src/odrive.cpp)

ament_target_dependencies(odrive
  rclcpp_lifecycle
  can_interface
  odrive_interface
  std_srvs
  std_msgs)

install(TARGETS odrive
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME})

ament_package()
