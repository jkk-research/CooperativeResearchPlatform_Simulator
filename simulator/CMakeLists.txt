cmake_minimum_required(VERSION 3.18)
project(simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(crp_msgs REQUIRED)
find_package(scenario_generation REQUIRED)
find_package(autoware_control_msgs REQUIRED)
find_package(tier4_planning_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(include)

add_library(vehicle_model src/vehicle_model.cpp src/powertrain_model.cpp)
target_compile_features(vehicle_model PUBLIC cxx_std_17)
target_link_libraries(vehicle_model scenario_generation::scenario_generation_core)
ament_target_dependencies(vehicle_model scenario_generation)

add_executable(simulator src/simulator_node.cpp)
target_compile_features(simulator PUBLIC cxx_std_17)
target_link_libraries(simulator vehicle_model scenario_generation::scenario_generation_core)
ament_target_dependencies(simulator rclcpp ament_index_cpp autoware_control_msgs crp_msgs
  scenario_generation tier4_planning_msgs geometry_msgs tf2)

add_executable(tf_broadcaster src/tf_broadcaster.cpp)
ament_target_dependencies(tf_broadcaster rclcpp tf2_ros geometry_msgs nav_msgs crp_msgs)

install(TARGETS
  vehicle_model
  simulator
  tf_broadcaster
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_vehicle_model test/test_vehicle_model.cpp)
  target_compile_features(test_vehicle_model PUBLIC cxx_std_17)
  target_link_libraries(test_vehicle_model vehicle_model scenario_generation::scenario_generation_core)
  ament_target_dependencies(test_vehicle_model scenario_generation tier4_planning_msgs)
  add_test(NAME vehicle_model COMMAND test_vehicle_model
    "${CMAKE_CURRENT_SOURCE_DIR}/config/motor_efficiency.csv")
  set(SIMULATOR_SCENARIO_TEST_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../scenario_files"
    CACHE PATH "Directory containing the example MAT scenarios")
  if(EXISTS "${SIMULATOR_SCENARIO_TEST_DIR}/occluded_pedestrian_costmap_data.mat" AND
     EXISTS "${SIMULATOR_SCENARIO_TEST_DIR}/occluded_pedestrian_crossing_costmap_data.mat" AND
     EXISTS "${SIMULATOR_SCENARIO_TEST_DIR}/roundabout_with_conflicting_traffic_costmap_data.mat")
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    add_test(NAME vehicle_model_ros COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_vehicle_model_ros.py $<TARGET_FILE:simulator>
      ${SIMULATOR_SCENARIO_TEST_DIR}/occluded_pedestrian_crossing_costmap_data.mat)
    set_tests_properties(vehicle_model_ros PROPERTIES TIMEOUT 60)
    add_test(NAME occlusion_ros COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_vehicle_model_ros.py --occlusion $<TARGET_FILE:simulator>
      ${SIMULATOR_SCENARIO_TEST_DIR}/occluded_pedestrian_costmap_data.mat)
    set_tests_properties(occlusion_ros PROPERTIES TIMEOUT 60)
  endif()
endif()

ament_package()
