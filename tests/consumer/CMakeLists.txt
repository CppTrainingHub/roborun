cmake_minimum_required(VERSION 3.24)

project(RoboRunConsumer LANGUAGES CXX)

set(ROBORUN_REQUIRED_VERSION "1.0" CACHE STRING "Required RoboRun package version")
find_package(RoboRun ${ROBORUN_REQUIRED_VERSION} CONFIG REQUIRED)

add_executable(roborun_consumer main.cpp)
target_link_libraries(roborun_consumer PRIVATE RoboRun::core)

include(CTest)
if(BUILD_TESTING)
  add_test(NAME roborun_consumer_execution COMMAND roborun_consumer)
endif()
