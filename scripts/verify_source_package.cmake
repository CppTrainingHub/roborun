if(NOT DEFINED ROBORUN_ARCHIVE OR ROBORUN_ARCHIVE STREQUAL "")
  message(FATAL_ERROR "Set ROBORUN_ARCHIVE to a RoboRun source tar.gz archive.")
endif()
if(NOT EXISTS "${ROBORUN_ARCHIVE}")
  message(FATAL_ERROR "ROBORUN_ARCHIVE does not exist: ${ROBORUN_ARCHIVE}")
endif()
if(NOT DEFINED ROBORUN_WORK_DIR OR ROBORUN_WORK_DIR STREQUAL "")
  message(FATAL_ERROR "Set ROBORUN_WORK_DIR to a dedicated disposable directory.")
endif()
cmake_path(ABSOLUTE_PATH ROBORUN_WORK_DIR NORMALIZE OUTPUT_VARIABLE normalized_work_dir)
set(project_build_root "${CMAKE_CURRENT_LIST_DIR}/../.build")
cmake_path(ABSOLUTE_PATH project_build_root NORMALIZE OUTPUT_VARIABLE normalized_build_root)
cmake_path(IS_PREFIX normalized_build_root "${normalized_work_dir}" NORMALIZE work_dir_is_safe)
if(NOT work_dir_is_safe OR normalized_work_dir STREQUAL normalized_build_root)
  message(FATAL_ERROR
          "ROBORUN_WORK_DIR must be a dedicated child of ${normalized_build_root}: "
          "${normalized_work_dir}")
endif()
set(ROBORUN_WORK_DIR "${normalized_work_dir}")

set(build_dir "${ROBORUN_WORK_DIR}/.build/core")
set(install_dir "${ROBORUN_WORK_DIR}/install")
set(relocated_dir "${ROBORUN_WORK_DIR}/relocated")
set(incompatible_dir "${ROBORUN_WORK_DIR}/.build/incompatible-consumer")

file(REMOVE_RECURSE "${ROBORUN_WORK_DIR}")
file(MAKE_DIRECTORY "${ROBORUN_WORK_DIR}/unpacked")

function(run_checked)
  execute_process(
    COMMAND ${ARGN}
    RESULT_VARIABLE result
    OUTPUT_VARIABLE output
    ERROR_VARIABLE error)
  if(NOT result EQUAL 0)
    message(FATAL_ERROR "Command failed (${result}): ${ARGN}\n${output}\n${error}")
  endif()
endfunction()

execute_process(
  COMMAND "${CMAKE_COMMAND}" -E tar xzf "${ROBORUN_ARCHIVE}"
  WORKING_DIRECTORY "${ROBORUN_WORK_DIR}/unpacked"
  RESULT_VARIABLE extract_result
  OUTPUT_VARIABLE extract_output
  ERROR_VARIABLE extract_error)
if(NOT extract_result EQUAL 0)
  message(FATAL_ERROR "Could not extract ${ROBORUN_ARCHIVE}: ${extract_output}\n${extract_error}")
endif()
file(GLOB archive_roots LIST_DIRECTORIES TRUE "${ROBORUN_WORK_DIR}/unpacked/RoboRun-*")
list(LENGTH archive_roots archive_root_count)
if(NOT archive_root_count EQUAL 1)
  message(FATAL_ERROR "Archive must extract one RoboRun-<version> source root.")
endif()
list(GET archive_roots 0 archive_root)
if(NOT EXISTS "${archive_root}/CMakeLists.txt")
  message(FATAL_ERROR "Archive root does not contain CMakeLists.txt: ${archive_root}")
endif()

foreach(forbidden_directory IN ITEMS .scratch .build github-references)
  if(EXISTS "${archive_root}/${forbidden_directory}")
    message(FATAL_ERROR
            "Source archive contains excluded directory: ${forbidden_directory}")
  endif()
endforeach()
file(
  GLOB_RECURSE embedded_dependency_archives
  LIST_DIRECTORIES FALSE
  "${archive_root}/third_party/*.tar.gz"
  "${archive_root}/third_party/*.tar.xz"
  "${archive_root}/third_party/*.zip")
if(embedded_dependency_archives)
  message(FATAL_ERROR
          "Source archive contains duplicate dependency downloads: ${embedded_dependency_archives}")
endif()

run_checked("${CMAKE_COMMAND}" -S "${archive_root}" -B "${build_dir}" -DBUILD_TESTING=ON
            -DROBORUN_ENABLE_COPPELIASIM=OFF -DBUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Release
            -DCMAKE_INSTALL_PREFIX=${install_dir})
run_checked("${CMAKE_COMMAND}" --build "${build_dir}" --config Release --parallel 2)
run_checked("${CMAKE_CTEST_COMMAND}" --test-dir "${build_dir}" -C Release --output-on-failure)
run_checked("${CMAKE_COMMAND}" --install "${build_dir}" --config Release)

file(RENAME "${install_dir}" "${relocated_dir}")
file(GLOB package_metadata "${relocated_dir}/*/cmake/RoboRun/*.cmake")
foreach(metadata_file IN LISTS package_metadata)
  file(READ "${metadata_file}" metadata_contents)
  string(FIND "${metadata_contents}" "${archive_root}" source_path_index)
  if(NOT source_path_index EQUAL -1)
    message(FATAL_ERROR "Installed package metadata leaks a source path: ${metadata_file}")
  endif()
  string(FIND "${metadata_contents}" "${build_dir}" build_path_index)
  if(NOT build_path_index EQUAL -1)
    message(FATAL_ERROR "Installed package metadata leaks a build path: ${metadata_file}")
  endif()
endforeach()

foreach(consumer_configuration IN ITEMS Release Debug)
  string(TOLOWER "${consumer_configuration}" consumer_configuration_lower)
  set(consumer_dir "${ROBORUN_WORK_DIR}/.build/consumer-${consumer_configuration_lower}")
  run_checked("${CMAKE_COMMAND}" -S "${archive_root}/tests/consumer" -B "${consumer_dir}"
              -DCMAKE_PREFIX_PATH=${relocated_dir} -DBUILD_TESTING=ON
              -DCMAKE_BUILD_TYPE=${consumer_configuration})
  run_checked("${CMAKE_COMMAND}" --build "${consumer_dir}" --config "${consumer_configuration}"
              --parallel 2)
  run_checked("${CMAKE_CTEST_COMMAND}" --test-dir "${consumer_dir}"
              -C "${consumer_configuration}" --output-on-failure)
endforeach()

execute_process(
  COMMAND "${CMAKE_COMMAND}" -S "${archive_root}/tests/consumer" -B "${incompatible_dir}"
          -DCMAKE_PREFIX_PATH=${relocated_dir} -DROBORUN_REQUIRED_VERSION=2.0
  RESULT_VARIABLE incompatible_result
  OUTPUT_VARIABLE incompatible_output
  ERROR_VARIABLE incompatible_error)
if(incompatible_result EQUAL 0)
  message(FATAL_ERROR "An incompatible RoboRun major-version request unexpectedly configured.")
endif()

find_program(ROBORUN_INSTALLED_EXECUTABLE roborun
             HINTS "${relocated_dir}/bin" "${relocated_dir}/bin/Release" NO_DEFAULT_PATH)
if(NOT ROBORUN_INSTALLED_EXECUTABLE)
  message(FATAL_ERROR "Installed roborun executable was not found under the relocated prefix.")
endif()
file(GLOB installed_task "${relocated_dir}/share/roborun/tasks/ur5_move.task")
file(GLOB invalid_task "${relocated_dir}/share/roborun/tasks/invalid_move_before_servo.task")
run_checked("${ROBORUN_INSTALLED_EXECUTABLE}" --version)
run_checked("${ROBORUN_INSTALLED_EXECUTABLE}" run --backend mock --program "${installed_task}")
execute_process(
  COMMAND "${ROBORUN_INSTALLED_EXECUTABLE}" run --backend mock --program "${invalid_task}"
  RESULT_VARIABLE invalid_result
  OUTPUT_VARIABLE invalid_output
  ERROR_VARIABLE invalid_error)
if(invalid_result EQUAL 0)
  message(FATAL_ERROR "Installed CLI accepted an invalid task.")
endif()
string(FIND "${invalid_error}" "ERROR code=INVALID_RUNTIME_STATE" invalid_diagnostic_index)
if(invalid_diagnostic_index EQUAL -1)
  message(FATAL_ERROR "Installed CLI invalid-task diagnostic changed unexpectedly: ${invalid_error}")
endif()

message(STATUS "source release package seam passed from ${ROBORUN_ARCHIVE}")
