cmake_minimum_required(VERSION 3.8)
project(sura_navigator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(sura_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

add_executable(navigator_node
  src/navigator_node.cpp
)

add_executable(navigator_sim
  src/navigator_sim.cpp
)

add_executable(position_setpoint_adapter
  src/position_setpoint_adapter.cpp
)

ament_target_dependencies(navigator_node
  rclcpp
  nav_msgs
  sensor_msgs
  sura_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

ament_target_dependencies(navigator_sim
  rclcpp
  sensor_msgs
  sura_msgs
  geometry_msgs
  tf2
  tf2_ros
)

ament_target_dependencies(position_setpoint_adapter
  rclcpp
  geometry_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

install(TARGETS
  navigator_node
  navigator_sim
  position_setpoint_adapter
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
