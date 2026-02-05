cmake_minimum_required(VERSION 3.8)
project(simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(crp_msgs REQUIRED)
find_package(autoware_control_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_executable(simulator src/simulator.cpp)
  ament_target_dependencies(simulator rclcpp 
  autoware_control_msgs
  crp_msgs)

add_executable(tf_broadcaster src/tf_broadcaster.cpp)
ament_target_dependencies(tf_broadcaster rclcpp tf2_ros geometry_msgs nav_msgs crp_msgs)

install(TARGETS
  simulator
  tf_broadcaster
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
