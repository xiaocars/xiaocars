cmake_minimum_required(VERSION 3.8)
project(xiaoduckie)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(sensors REQUIRED)
find_package(devices REQUIRED)
find_package(control REQUIRED)
find_package(xiaocar_msgs REQUIRED)

include_directories(
  include
  ${colcon_INCLUDE_DIRS}
)

add_executable(xiaoduckie
  src/main.cc
  src/xiaoduckie.cc
  src/wheels_driver.cc
  src/differential_drive_controller.cc)

ament_target_dependencies(xiaoduckie rclcpp std_msgs geometry_msgs xiaocar_msgs sensors devices control)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  xiaoduckie
  DESTINATION lib/${PROJECT_NAME})

ament_package()
