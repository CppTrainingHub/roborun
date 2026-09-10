cmake_minimum_required(VERSION 3.24)

project(RoboRun VERSION 1.0.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(ROBORUN_ENABLE_COPPELIASIM "Build the CoppeliaSim ZeroMQ backend" OFF)
option(ROBORUN_WARNINGS_AS_ERRORS "Treat first-party compiler warnings as errors" ON)
set(COPPELIASIM_ROOT_DIR "" CACHE PATH
    "CoppeliaSim Resources directory containing programming/")

function(roborun_enable_warnings target_name)
  if(MSVC)
    target_compile_options(${target_name} PRIVATE /W4)
    if(ROBORUN_WARNINGS_AS_ERRORS)
      target_compile_options(${target_name} PRIVATE /WX)
    endif()
  else()
    target_compile_options(${target_name} PRIVATE -Wall -Wextra -Wpedantic)
    if(ROBORUN_WARNINGS_AS_ERRORS)
      target_compile_options(${target_name} PRIVATE -Werror)
    endif()
  endif()
endfunction()

add_library(
  roborun_core
  STATIC
  src/roborun/config.cpp
  src/roborun/mock_robot_backend.cpp
  src/roborun/parser.cpp
  src/roborun/runtime.cpp
  src/roborun/scenario.cpp
  src/roborun/scripted_mock_backend.cpp
  src/roborun/simulator_lifecycle.cpp
  src/roborun/types.cpp
  src/roborun/validator.cpp
  src/roborun/workcell.cpp)
add_library(RoboRun::core ALIAS roborun_core)
set_target_properties(roborun_core PROPERTIES EXPORT_NAME core)
target_compile_features(roborun_core PUBLIC cxx_std_20)
target_include_directories(
  roborun_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/nlohmann_json/include)
target_compile_definitions(
  roborun_core
  PRIVATE JSON_USE_IMPLICIT_CONVERSIONS=0 JSON_DIAGNOSTIC_POSITIONS=1)
roborun_enable_warnings(roborun_core)

add_executable(roborun src/roborun/main.cpp)
target_link_libraries(roborun PRIVATE roborun_core)
target_compile_definitions(roborun PRIVATE ROBORUN_VERSION="${PROJECT_VERSION}")
roborun_enable_warnings(roborun)

add_library(roborun_cli_support STATIC src/roborun/cli_evidence_barrier.cpp)
target_include_directories(roborun_cli_support PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(roborun PRIVATE roborun_cli_support)
roborun_enable_warnings(roborun_cli_support)

if(ROBORUN_ENABLE_COPPELIASIM)
  if(NOT COPPELIASIM_ROOT_DIR)
    message(FATAL_ERROR
            "ROBORUN_ENABLE_COPPELIASIM requires COPPELIASIM_ROOT_DIR. Set it to the "
            "CoppeliaSim Resources directory containing programming/zmqRemoteApi/clients/cpp.")
  endif()
  if(NOT EXISTS "${COPPELIASIM_ROOT_DIR}/programming/zmqRemoteApi/clients/cpp/CMakeLists.txt")
    message(FATAL_ERROR
            "COPPELIASIM_ROOT_DIR='${COPPELIASIM_ROOT_DIR}' is missing "
            "programming/zmqRemoteApi/clients/cpp/CMakeLists.txt. Set COPPELIASIM_ROOT_DIR "
            "to the official CoppeliaSim Resources directory.")
  endif()

  # The official client CMake project reads this value from the environment.
  set(ENV{COPPELIASIM_ROOT_DIR} "${COPPELIASIM_ROOT_DIR}")
  add_subdirectory(
    "${COPPELIASIM_ROOT_DIR}/programming/zmqRemoteApi/clients/cpp"
    "${CMAKE_BINARY_DIR}/coppeliasim_zmq_remote_api"
    EXCLUDE_FROM_ALL)

  add_library(roborun_coppeliasim src/roborun/coppeliasim_backend.cpp)
  target_include_directories(
    roborun_coppeliasim
    PRIVATE
      "${CMAKE_CURRENT_SOURCE_DIR}/src"
      "${COPPELIASIM_ROOT_DIR}/programming/zmqRemoteApi/clients/cpp")
  target_link_libraries(roborun_coppeliasim PUBLIC roborun_core RemoteAPIClient)
  roborun_enable_warnings(roborun_coppeliasim)

  target_link_libraries(roborun PRIVATE roborun_coppeliasim)
  target_compile_definitions(roborun PRIVATE ROBORUN_WITH_COPPELIASIM=1)

  add_executable(roborun_coppeliasim_stop_probe tests/coppeliasim_stop_probe.cpp)
  target_link_libraries(roborun_coppeliasim_stop_probe PRIVATE roborun_core roborun_coppeliasim)
  roborun_enable_warnings(roborun_coppeliasim_stop_probe)
endif()

set(ROBORUN_PUBLIC_HEADERS
    src/roborun/clock.h
    src/roborun/config.h
    src/roborun/event_queue.h
    src/roborun/mock_robot_backend.h
    src/roborun/parser.h
    src/roborun/robot_backend.h
    src/roborun/runtime.h
    src/roborun/scenario.h
    src/roborun/scripted_mock_backend.h
    src/roborun/simulator_lifecycle.h
    src/roborun/types.h
    src/roborun/validator.h
    src/roborun/workcell.h
    src/roborun/workcell_backend.h)

install(
  TARGETS roborun_core roborun
  EXPORT RoboRunTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(FILES ${ROBORUN_PUBLIC_HEADERS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/roborun)
install(DIRECTORY examples/config examples/tasks DESTINATION ${CMAKE_INSTALL_DATADIR}/roborun)
install(FILES README.md THIRD_PARTY_NOTICES.md DESTINATION ${CMAKE_INSTALL_DATADIR}/roborun)
install(
  EXPORT RoboRunTargets
  FILE RoboRunTargets.cmake
  NAMESPACE RoboRun::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/RoboRun)
configure_package_config_file(
  cmake/RoboRunConfig.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/RoboRunConfig.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/RoboRun)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/RoboRunConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion)
install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/RoboRunConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/RoboRunConfigVersion.cmake"
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/RoboRun)

include(CTest)
if(BUILD_TESTING)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
  add_subdirectory(third_party/googletest EXCLUDE_FROM_ALL)

  include(GoogleTest)
  add_executable(roborun_tests tests/roborun_tests.cpp)
  target_link_libraries(roborun_tests PRIVATE roborun_core GTest::gtest_main)
  roborun_enable_warnings(roborun_tests)
  gtest_discover_tests(roborun_tests)

  add_executable(roborun_lifecycle_control_tests tests/lifecycle_control_tests.cpp)
  target_link_libraries(roborun_lifecycle_control_tests PRIVATE roborun_core GTest::gtest_main)
  roborun_enable_warnings(roborun_lifecycle_control_tests)
  gtest_discover_tests(roborun_lifecycle_control_tests)

  add_executable(roborun_runtime_snapshot_tests tests/runtime_snapshot_tests.cpp)
  target_link_libraries(roborun_runtime_snapshot_tests PRIVATE roborun_core GTest::gtest_main)
  roborun_enable_warnings(roborun_runtime_snapshot_tests)
  gtest_discover_tests(roborun_runtime_snapshot_tests)

  add_executable(roborun_workcell_tests tests/workcell_tests.cpp)
  target_link_libraries(roborun_workcell_tests PRIVATE roborun_core GTest::gtest_main)
  target_compile_definitions(roborun_workcell_tests PRIVATE
                             ROBORUN_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  roborun_enable_warnings(roborun_workcell_tests)
  gtest_discover_tests(roborun_workcell_tests)

  add_executable(roborun_gui_evidence_barrier_tests tests/gui_evidence_barrier_tests.cpp)
  target_include_directories(roborun_gui_evidence_barrier_tests
                             PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(roborun_gui_evidence_barrier_tests
                        PRIVATE roborun_cli_support GTest::gtest_main)
  roborun_enable_warnings(roborun_gui_evidence_barrier_tests)
  gtest_discover_tests(roborun_gui_evidence_barrier_tests)

  add_test(NAME roborun_cli_version COMMAND roborun --version)
  set_tests_properties(roborun_cli_version PROPERTIES PASS_REGULAR_EXPRESSION "RoboRun 1\\.0\\.0")
  add_test(
    NAME roborun_coppeliasim_missing_dependency_diagnostic
    COMMAND ${CMAKE_COMMAND}
            -DROBORUN_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
            -DROBORUN_BUILD_DIR=${CMAKE_CURRENT_BINARY_DIR}/coppeliasim-missing-dependency
            -P ${CMAKE_CURRENT_SOURCE_DIR}/scripts/verify_coppeliasim_config_failure.cmake)
  add_test(
    NAME roborun_cli_coppeliasim_port
    COMMAND ${CMAKE_COMMAND}
            -DROBORUN_BIN=$<TARGET_FILE:roborun>
            -DROBORUN_PROGRAM=${CMAKE_CURRENT_SOURCE_DIR}/examples/tasks/ur5_move.task
            -DROBORUN_COPPELIASIM_ENABLED=${ROBORUN_ENABLE_COPPELIASIM}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_cli_coppeliasim_port.cmake)
  add_test(
    NAME roborun_cli_legacy_mock
    COMMAND roborun run --backend mock --program
            "${CMAKE_CURRENT_SOURCE_DIR}/examples/tasks/ur5_move.task")
  add_test(
    NAME roborun_cli_named_mock
    COMMAND roborun run --backend mock --format human --robot-config
            "${CMAKE_CURRENT_SOURCE_DIR}/examples/config/ur5_robot.json" --points-config
            "${CMAKE_CURRENT_SOURCE_DIR}/examples/config/ur5_points.json" --program
            "${CMAKE_CURRENT_SOURCE_DIR}/examples/tasks/ur5_named_points.task")
  add_test(
    NAME roborun_cli_named_mock_trace
    COMMAND roborun run --backend mock --format trace --robot-config
            "${CMAKE_CURRENT_SOURCE_DIR}/examples/config/ur5_robot.json" --points-config
            "${CMAKE_CURRENT_SOURCE_DIR}/examples/config/ur5_points.json" --program
            "${CMAKE_CURRENT_SOURCE_DIR}/examples/tasks/ur5_named_points.task")
  add_test(
    NAME roborun_cli_invalid_format
    COMMAND roborun run --backend mock --format yaml --program
            "${CMAKE_CURRENT_SOURCE_DIR}/examples/tasks/ur5_move.task")
  set_tests_properties(roborun_cli_named_mock PROPERTIES PASS_REGULAR_EXPRESSION "RoboRun execution")
  set_tests_properties(roborun_cli_named_mock_trace PROPERTIES
                       PASS_REGULAR_EXPRESSION "TRACE .*command_index=2")
  set_tests_properties(roborun_cli_invalid_format PROPERTIES WILL_FAIL TRUE)
  add_test(
    NAME roborun_cli_invalid_task
    COMMAND roborun run --backend mock --program
            "${CMAKE_CURRENT_SOURCE_DIR}/examples/tasks/invalid_move_before_servo.task")
  set_tests_properties(roborun_cli_invalid_task PROPERTIES WILL_FAIL TRUE)
  add_test(
    NAME roborun_cli_mock_workcell
    COMMAND roborun run --backend mock --format trace
            --robot-config "${CMAKE_CURRENT_SOURCE_DIR}/examples/config/ur5_robot.json"
            --points-config "${CMAKE_CURRENT_SOURCE_DIR}/examples/config/ur5_points.json"
            --io-config "${CMAKE_CURRENT_SOURCE_DIR}/examples/config/digital_io.json"
            --tool-config "${CMAKE_CURRENT_SOURCE_DIR}/examples/config/tool_feedback.json"
            --scenario "${CMAKE_CURRENT_SOURCE_DIR}/examples/config/mock_workcell.scenario.json"
            --program "${CMAKE_CURRENT_SOURCE_DIR}/examples/tasks/mock_workcell.task")
  set_tests_properties(roborun_cli_mock_workcell PROPERTIES
                       PASS_REGULAR_EXPRESSION "RESULT status=passed.*finished_business_time_ms=375")
  add_test(
    NAME roborun_cli_deterministic_workcell_trace
    COMMAND ${CMAKE_COMMAND}
            -DROBORUN_BIN=$<TARGET_FILE:roborun>
            -DROBORUN_PROGRAM=${CMAKE_CURRENT_SOURCE_DIR}/examples/tasks/mock_workcell.task
            -DROBORUN_ROBOT=${CMAKE_CURRENT_SOURCE_DIR}/examples/config/ur5_robot.json
            -DROBORUN_POINTS=${CMAKE_CURRENT_SOURCE_DIR}/examples/config/ur5_points.json
            -DROBORUN_IO=${CMAKE_CURRENT_SOURCE_DIR}/examples/config/digital_io.json
            -DROBORUN_TOOLS=${CMAKE_CURRENT_SOURCE_DIR}/examples/config/tool_feedback.json
            -DROBORUN_SCENARIO=${CMAKE_CURRENT_SOURCE_DIR}/examples/config/mock_workcell.scenario.json
            -P ${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_deterministic_trace.cmake)
  add_test(
    NAME roborun_cli_digital_io_wait_timeout
    COMMAND roborun run --backend mock --format trace
            --io-config "${CMAKE_CURRENT_SOURCE_DIR}/examples/config/digital_io.json"
            --program "${CMAKE_CURRENT_SOURCE_DIR}/examples/tasks/digital_io_wait_timeout.task")
  set_tests_properties(roborun_cli_digital_io_wait_timeout PROPERTIES WILL_FAIL TRUE)
  add_test(
    NAME roborun_cli_pause_resume
    COMMAND roborun run --backend mock --format trace
            --scenario "${CMAKE_CURRENT_SOURCE_DIR}/examples/config/pause_resume.scenario.json"
            --program "${CMAKE_CURRENT_SOURCE_DIR}/examples/tasks/ur5_move.task")
  set_tests_properties(roborun_cli_pause_resume PROPERTIES
                       PASS_REGULAR_EXPRESSION "outcome=program_completed")
endif()
