cmake_minimum_required(VERSION 3.18)
project(simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(crp_msgs REQUIRED)
find_package(autoware_control_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(autoware_perception_msgs REQUIRED)
find_package(tier4_planning_msgs REQUIRED)
find_path(MATIO_INCLUDE_DIR matio.h REQUIRED)
find_library(MATIO_LIBRARY matio REQUIRED)

include_directories(include)

add_library(scenario_data src/scenario_data.cpp src/vehicle_model.cpp)
target_compile_features(scenario_data PUBLIC cxx_std_17)
target_include_directories(scenario_data PRIVATE ${MATIO_INCLUDE_DIR})
target_link_libraries(scenario_data ${MATIO_LIBRARY})

add_executable(simulator src/simulator_node.cpp src/scenario_playback.cpp)
target_compile_features(simulator PUBLIC cxx_std_17)
target_link_libraries(simulator scenario_data)
ament_target_dependencies(simulator rclcpp autoware_control_msgs crp_msgs
  autoware_perception_msgs tier4_planning_msgs geometry_msgs)

add_executable(tf_broadcaster src/tf_broadcaster.cpp)
ament_target_dependencies(tf_broadcaster rclcpp tf2_ros geometry_msgs nav_msgs crp_msgs)

install(TARGETS
  scenario_data
  simulator
  tf_broadcaster
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../scenario_files")
  install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../scenario_files/"
    DESTINATION share/${PROJECT_NAME}/scenario_files
    FILES_MATCHING PATTERN "*.mat")
endif()

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_scenario_data test/test_scenario_data.cpp src/scenario_playback.cpp)
  target_link_libraries(test_scenario_data scenario_data)
  ament_target_dependencies(test_scenario_data crp_msgs autoware_perception_msgs
    tier4_planning_msgs geometry_msgs)
  set(SIMULATOR_SCENARIO_TEST_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../scenario_files"
    CACHE PATH "Directory containing the three example MAT scenarios")
  if(EXISTS "${SIMULATOR_SCENARIO_TEST_DIR}/occluded_pedestrian_costmap_data.mat" AND
     EXISTS "${SIMULATOR_SCENARIO_TEST_DIR}/occluded_pedestrian_crossing_costmap_data.mat" AND
     EXISTS "${SIMULATOR_SCENARIO_TEST_DIR}/roundabout_with_conflicting_traffic_costmap_data.mat")
    add_test(NAME scenario_data COMMAND test_scenario_data ${SIMULATOR_SCENARIO_TEST_DIR})
  endif()
endif()

ament_package()
