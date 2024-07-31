cmake_minimum_required(VERSION 3.8)
project(testing_interfaces_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tutorial_interfaces REQUIRED)

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

add_executable(sphere_publisher src/sphere_publisher.cpp)
ament_target_dependencies(sphere_publisher rclcpp tutorial_interfaces)

add_executable(sphere_listener src/sphere_listener.cpp)
ament_target_dependencies(sphere_listener rclcpp tutorial_interfaces)

add_executable(add_service_node src/add_service_node.cpp)
ament_target_dependencies(add_service_node rclcpp tutorial_interfaces)

add_executable(add_client_node src/add_client_node.cpp)
ament_target_dependencies(add_client_node rclcpp tutorial_interfaces)

install(TARGETS
  sphere_publisher
  sphere_listener
  add_service_node
  add_client_node
DESTINATION lib/${PROJECT_NAME})

ament_package()
