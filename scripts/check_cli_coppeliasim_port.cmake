foreach(port IN ITEMS 0 65536)
  execute_process(
    COMMAND "${ROBORUN_BIN}" run --backend coppeliasim --coppeliasim-port "${port}"
            --program "${ROBORUN_PROGRAM}"
    RESULT_VARIABLE status
    OUTPUT_VARIABLE output
    ERROR_VARIABLE error)
  if(status EQUAL 0 OR NOT error MATCHES "Usage: roborun run")
    message(FATAL_ERROR "invalid CoppeliaSim port ${port} was not rejected: ${output}${error}")
  endif()
endforeach()

if(NOT ROBORUN_COPPELIASIM_ENABLED)
  execute_process(
    COMMAND "${ROBORUN_BIN}" run --backend coppeliasim --coppeliasim-port 24000
            --program "${ROBORUN_PROGRAM}"
    TIMEOUT 10
    RESULT_VARIABLE status
    OUTPUT_VARIABLE output
    ERROR_VARIABLE error)
  if(NOT status EQUAL 2 OR NOT error MATCHES "backend was not enabled at build time")
    message(FATAL_ERROR "valid CoppeliaSim port was not accepted: ${output}${error}")
  endif()
endif()

message(STATUS "CoppeliaSim CLI port boundaries passed")
