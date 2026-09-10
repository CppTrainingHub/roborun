cmake_minimum_required(VERSION 3.16)
project(roborun_ros2_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(roborun_interfaces REQUIRED)

option(ROBORUN_BRIDGE_ENABLE_COPPELIASIM "Enable the CoppeliaSim backend in the bridge" OFF)
if(ROBORUN_BRIDGE_ENABLE_COPPELIASIM)
  set(ROBORUN_ENABLE_COPPELIASIM ON CACHE BOOL "" FORCE)
endif()

# The core remains a normal CMake project.  Building this package never makes
# the root project discover ROS 2, and no Runtime source is copied here.
set(_roborun_build_testing "${BUILD_TESTING}")
set(BUILD_TESTING OFF)
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../.."
                 "${CMAKE_CURRENT_BINARY_DIR}/roborun_core")
set(BUILD_TESTING "${_roborun_build_testing}")
# The root project has its own project() call; restore this package identity
# before ament_package() reads it.
project(roborun_ros2_bridge)

add_executable(roborun_bridge_node src/bridge_node.cpp)
target_link_libraries(roborun_bridge_node roborun_core)
if(ROBORUN_BRIDGE_ENABLE_COPPELIASIM)
  target_link_libraries(roborun_bridge_node roborun_coppeliasim)
  target_compile_definitions(roborun_bridge_node PRIVATE ROBORUN_WITH_COPPELIASIM=1)
endif()
ament_target_dependencies(roborun_bridge_node
  rclcpp rclcpp_action sensor_msgs roborun_interfaces)
roborun_enable_warnings(roborun_bridge_node)

install(TARGETS roborun_bridge_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME roborun_ros2_client_observation_test
    COMMAND "${Python3_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/../../tests/ros2_client_observation_tests.py")
  set_tests_properties(roborun_ros2_client_observation_test PROPERTIES TIMEOUT 10)
  ament_add_gtest(roborun_bridge_interface_contract_test test/interface_contract_test.cpp)
  ament_target_dependencies(roborun_bridge_interface_contract_test roborun_interfaces)
  roborun_enable_warnings(roborun_bridge_interface_contract_test)
endif()
ament_package()
