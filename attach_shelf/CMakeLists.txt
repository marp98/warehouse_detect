cmake_minimum_required(VERSION 3.8)
project(attach_shelf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(approach_shelf_msg REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

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

add_executable(pre_approach src/pre_approach.cpp)
ament_target_dependencies(pre_approach rclcpp sensor_msgs geometry_msgs nav_msgs)

add_executable(pre_approach_v2 src/pre_approach_v2.cpp)
ament_target_dependencies(pre_approach_v2 rclcpp sensor_msgs geometry_msgs nav_msgs approach_shelf_msg)

add_executable(approach_shelf_server src/approach_service_server.cpp)
ament_target_dependencies(approach_shelf_server
  rclcpp
  approach_shelf_msg
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  std_msgs
)


install(TARGETS 
  pre_approach
  pre_approach_v2
  approach_shelf_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
