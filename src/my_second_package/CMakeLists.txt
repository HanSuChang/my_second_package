cmake_minimum_required(VERSION 3.8)
project(my_second_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(my_first_package_msgs REQUIRED)

add_executable(dist_turtle_action_server src/dist_turtle_action_server.cpp)

# ament_target_dependencies를 쓰면 훨씬 편합니다
ament_target_dependencies(dist_turtle_action_server
  rclcpp
  rclcpp_action
  turtlesim
  geometry_msgs
  my_first_package_msgs
)

install(TARGETS
  dist_turtle_action_server
  DESTINATION lib/${PROJECT_NAME})

ament_package()