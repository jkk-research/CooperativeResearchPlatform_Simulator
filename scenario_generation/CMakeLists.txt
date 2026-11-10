cmake_minimum_required(VERSION 3.18)
project(scenario_generation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(crp_msgs REQUIRED)
find_package(autoware_perception_msgs REQUIRED)
find_package(tier4_planning_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Boost REQUIRED)
find_path(MATIO_INCLUDE_DIR matio.h REQUIRED)
find_library(MATIO_LIBRARY matio REQUIRED)

add_library(scenario_generation_core
  src/scenario_data.cpp
  src/scenario_playback.cpp
  src/occlusion.cpp)
target_compile_features(scenario_generation_core PUBLIC cxx_std_17)
target_include_directories(scenario_generation_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE ${MATIO_INCLUDE_DIR} ${Boost_INCLUDE_DIRS})
target_link_libraries(scenario_generation_core ${MATIO_LIBRARY})
ament_target_dependencies(scenario_generation_core crp_msgs autoware_perception_msgs
  tier4_planning_msgs geometry_msgs tf2)

add_executable(scenario_generator src/scenario_generator_node.cpp)
target_compile_features(scenario_generator PUBLIC cxx_std_17)
target_link_libraries(scenario_generator scenario_generation_core)
ament_target_dependencies(scenario_generator ament_index_cpp rclcpp crp_msgs geometry_msgs tf2 tf2_ros)

install(TARGETS scenario_generation_core
  EXPORT export_scenario_generation_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(TARGETS scenario_generator DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../../scenario_files")
  install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../../scenario_files/"
    DESTINATION share/${PROJECT_NAME}/scenario_files
    FILES_MATCHING PATTERN "*.mat"
    PATTERN "recordings" EXCLUDE)
endif()

if(BUILD_TESTING)
  enable_testing()
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_executable(test_scenario_data test/test_scenario_data.cpp)
  target_compile_features(test_scenario_data PUBLIC cxx_std_17)
  target_link_libraries(test_scenario_data scenario_generation_core)
  ament_target_dependencies(test_scenario_data crp_msgs autoware_perception_msgs
    tier4_planning_msgs geometry_msgs tf2)
  add_test(NAME occlusion COMMAND test_scenario_data --occlusion)
  add_test(NAME scenario_range COMMAND test_scenario_data --range)
  set(SCENARIO_TEST_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../scenario_files"
    CACHE PATH "Directory containing the example MAT scenarios")
  if(EXISTS "${SCENARIO_TEST_DIR}/urban_right_turn_pedestrian_near_road_costmap_data.mat")
    add_test(NAME scenario_urban_right_turn COMMAND test_scenario_data --load
      "${SCENARIO_TEST_DIR}/urban_right_turn_pedestrian_near_road_costmap_data.mat")
  endif()
  if(EXISTS "${SCENARIO_TEST_DIR}/occluded_pedestrian_costmap_data.mat" AND
     EXISTS "${SCENARIO_TEST_DIR}/occluded_pedestrian_crossing_costmap_data.mat" AND
     EXISTS "${SCENARIO_TEST_DIR}/roundabout_with_conflicting_traffic_costmap_data.mat")
    add_test(NAME scenario_data COMMAND test_scenario_data ${SCENARIO_TEST_DIR})
  endif()
  if(EXISTS "${SCENARIO_TEST_DIR}/occluded_pedestrian_costmap_data.mat")
    add_test(NAME scenario_generator_ros COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_scenario_generator_ros.py
      $<TARGET_FILE:scenario_generator> ${SCENARIO_TEST_DIR}/occluded_pedestrian_costmap_data.mat)
    set_tests_properties(scenario_generator_ros PROPERTIES TIMEOUT 30)
  endif()
endif()

ament_export_targets(export_scenario_generation_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(crp_msgs autoware_perception_msgs tier4_planning_msgs geometry_msgs tf2)
ament_package()