if(NOT DEFINED ROBORUN_SOURCE_DIR OR ROBORUN_SOURCE_DIR STREQUAL "")
  message(FATAL_ERROR "Set ROBORUN_SOURCE_DIR.")
endif()
if(NOT DEFINED ROBORUN_BUILD_DIR OR ROBORUN_BUILD_DIR STREQUAL "")
  message(FATAL_ERROR "Set ROBORUN_BUILD_DIR.")
endif()

execute_process(
  COMMAND "${CMAKE_COMMAND}" -S "${ROBORUN_SOURCE_DIR}" -B "${ROBORUN_BUILD_DIR}"
          -DBUILD_TESTING=OFF -DROBORUN_ENABLE_COPPELIASIM=ON
          -DCOPPELIASIM_ROOT_DIR=${ROBORUN_BUILD_DIR}/missing-resources
  RESULT_VARIABLE configure_result
  OUTPUT_VARIABLE configure_output
  ERROR_VARIABLE configure_error)
if(configure_result EQUAL 0)
  message(FATAL_ERROR "CoppeliaSim configuration unexpectedly succeeded without its official client.")
endif()
string(CONCAT configure_log "${configure_output}" "\n" "${configure_error}")
string(FIND "${configure_log}" "programming/zmqRemoteApi/clients/cpp/CMakeLists.txt" layout_index)
string(FIND "${configure_log}" "COPPELIASIM_ROOT_DIR" setting_index)
if(layout_index EQUAL -1 OR setting_index EQUAL -1)
  message(FATAL_ERROR "CoppeliaSim configuration diagnostic omitted the required layout or setting.")
endif()
