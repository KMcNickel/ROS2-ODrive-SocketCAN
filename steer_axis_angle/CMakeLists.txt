cmake_minimum_required(VERSION 3.8)
project(steer_axis_angle)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(odrive_interface REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
# Not needed yet
# find_package(can_interface REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(steer_axis_angle src/steer_axis_angle.cpp)

ament_target_dependencies(steer_axis_angle
  rclcpp_lifecycle
  odrive_interface
#  can_interface
  std_srvs
  std_msgs)

install(TARGETS steer_axis_angle
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME})

ament_package()