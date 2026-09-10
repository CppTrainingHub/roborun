#!/usr/bin/env bash

set -euo pipefail

project_root="$(cd "$(dirname "$0")/.." && pwd)"
app_path="${COPPELIASIM_APP:?Set COPPELIASIM_APP to the CoppeliaSim application path.}"
resources_path="$app_path/Contents/Resources"
simulator_binary="$app_path/Contents/MacOS/coppeliaSim"
model_path="$resources_path/models/robots/non-mobile/UR5.ttm"
task_path="${ROBORUN_TASK:-$project_root/examples/tasks/ur5_named_points.task}"
robot_config_path="${ROBORUN_ROBOT_CONFIG:-$project_root/examples/config/ur5_robot.json}"
points_config_path="${ROBORUN_POINTS_CONFIG:-$project_root/examples/config/ur5_points.json}"
named_task_path="$project_root/examples/tasks/ur5_named_points.task"
fast_task_path="$project_root/examples/tasks/ur5_speed_fast.task"
slow_task_path="$project_root/examples/tasks/ur5_speed_slow.task"
half_speed_robot_config_path="$project_root/examples/config/ur5_robot_half_speed.json"
build_directory="${ROBORUN_BUILD_DIR:-$project_root/.build/configured-program-coppeliasim}"
log_file="${TMPDIR:-/tmp}/roborun-coppeliasim-mvp.log"
coppeliasim_python="${COPPELIASIM_PYTHON:-}"

if [[ "${ROBORUN_SINGLE_RUN:-0}" != "1" && -z "${ROBORUN_TASK+x}" ]]; then
  run_single() {
    ROBORUN_SINGLE_RUN=1 ROBORUN_TASK="$1" "$0"
  }

  for run_number in 1 2 3; do
    echo "configured program named-task simulator run $run_number/3"
    run_single "$named_task_path"
  done

  fast_output="$(run_single "$fast_task_path")"
  printf '%s\n' "$fast_output"
  slow_output="$(run_single "$slow_task_path")"
  printf '%s\n' "$slow_output"
  half_speed_output="$(ROBORUN_SINGLE_RUN=1 ROBORUN_TASK="$fast_task_path" \
    ROBORUN_ROBOT_CONFIG="$half_speed_robot_config_path" \
    ROBORUN_POINTS_CONFIG="$points_config_path" "$0")"
  printf '%s\n' "$half_speed_output"
  fast_steps="$(printf '%s\n' "$fast_output" | awk '/point="PICK"/ && /command_status=succeeded/ { for (i = 1; i <= NF; ++i) if ($i ~ /^backend_steps=/) { sub("backend_steps=", "", $i); print $i } }')"
  slow_steps="$(printf '%s\n' "$slow_output" | awk '/point="PICK"/ && /command_status=succeeded/ { for (i = 1; i <= NF; ++i) if ($i ~ /^backend_steps=/) { sub("backend_steps=", "", $i); print $i } }')"
  half_speed_steps="$(printf '%s\n' "$half_speed_output" | awk '/point="PICK"/ && /command_status=succeeded/ { for (i = 1; i <= NF; ++i) if ($i ~ /^backend_steps=/) { sub("backend_steps=", "", $i); print $i } }')"
  if [[ ! "$fast_steps" =~ ^[0-9]+$ || ! "$slow_steps" =~ ^[0-9]+$ || "$slow_steps" -le "$fast_steps" ]]; then
    echo "CoppeliaSim speed check failed: fast=$fast_steps slow=$slow_steps" >&2
    exit 1
  fi
  if [[ ! "$half_speed_steps" =~ ^[0-9]+$ || "$half_speed_steps" -le "$fast_steps" ]]; then
    echo "CoppeliaSim reference-speed check failed: full=$fast_steps half=$half_speed_steps" >&2
    exit 1
  fi
  echo "RESULT status=passed phase=configured_program_speed_check fast_steps=$fast_steps slow_steps=$slow_steps"
  echo "RESULT status=passed phase=configured_program_reference_speed_check full_steps=$fast_steps half_steps=$half_speed_steps"
  exit 0
fi

runtime_directory="$(mktemp -d)"
input_fifo="$runtime_directory/coppeliasim.stdin"
server_pid=""

if [[ ! -x "$simulator_binary" || ! -f "$model_path" ]]; then
  echo "CoppeliaSim application is incomplete: $app_path" >&2
  exit 2
fi

if [[ ! -f "$task_path" || ! -f "$robot_config_path" || ! -f "$points_config_path" ]]; then
  echo "configured program task or configuration file is missing" >&2
  exit 2
fi

if [[ -n "$coppeliasim_python" && ! -x "$coppeliasim_python" ]]; then
  echo "COPPELIASIM_PYTHON is not an executable Python interpreter: $coppeliasim_python" >&2
  exit 2
fi

if lsof -nP -iTCP:23000 -sTCP:LISTEN >/dev/null 2>&1; then
  echo "TCP port 23000 is already in use; stop the existing service first" >&2
  exit 2
fi

cmake -S "$project_root" -B "$build_directory" \
  -DROBORUN_ENABLE_COPPELIASIM=ON \
  -DCOPPELIASIM_ROOT_DIR="$resources_path"
cmake --build "$build_directory" --target roborun -j4

mkfifo "$input_fifo"
# CoppeliaSim's headless console must keep stdin open for the process lifetime.
sleep 2147483647 >"$input_fifo" &
input_pid=$!
# Commander starts an interactive stdin thread that aborts during headless shutdown on macOS.
simulator_args=(-h "-GsimCmd.autoStart=false")
if [[ -n "$coppeliasim_python" ]]; then
  simulator_args+=("-Gpython=$coppeliasim_python")
fi
"$simulator_binary" "${simulator_args[@]}" <"$input_fifo" >"$log_file" 2>&1 &
simulator_pid=$!
cleanup() {
  if [[ -n "${server_pid:-}" && "$server_pid" != "${simulator_pid:-}" ]]; then
    kill "$server_pid" 2>/dev/null || true
    wait "$server_pid" 2>/dev/null || true
  fi
  if [[ -n "${simulator_pid:-}" ]]; then
    kill "$simulator_pid" 2>/dev/null || true
    wait "$simulator_pid" 2>/dev/null || true
  fi
  kill "$input_pid" 2>/dev/null || true
  wait "$input_pid" 2>/dev/null || true
  rm -f "$input_fifo"
  rmdir "$runtime_directory" 2>/dev/null || true
}
trap cleanup EXIT

for _ in {1..30}; do
  if nc -z 127.0.0.1 23000 2>/dev/null; then
    server_pid="$(lsof -tiTCP:23000 -sTCP:LISTEN)"
    if [[ -z "$server_pid" ]]; then
      echo "CoppeliaSim port opened without a discoverable server process" >&2
      exit 1
    fi
    sleep 2
    "$build_directory/roborun" run --backend coppeliasim \
      --robot-config "$robot_config_path" --points-config "$points_config_path" \
      --program "$task_path" --model "$model_path" --format trace &
    client_pid=$!
    client_status=1
    for _ in {1..10}; do
      if ! kill -0 "$client_pid" 2>/dev/null; then
        if wait "$client_pid"; then
          client_status=0
        else
          client_status=$?
        fi
        break
      fi
      sleep 1
    done
    if kill -0 "$client_pid" 2>/dev/null; then
      kill "$client_pid" 2>/dev/null || true
      wait "$client_pid" 2>/dev/null || true
      echo "RoboRun CoppeliaSim client timed out after 10 seconds" >&2
      exit 1
    fi

    for _ in {1..10}; do
      if ! lsof -nP -iTCP:23000 -sTCP:LISTEN >/dev/null 2>&1; then
        if wait "$simulator_pid"; then
          exit "$client_status"
        else
          simulator_status=$?
          echo "RESULT status=failed phase=simulator_exit exit_status=$simulator_status" >&2
          echo "CoppeliaSim exited unsuccessfully (status $simulator_status). Log: $log_file" >&2
          exit "$simulator_status"
        fi
      fi
      sleep 1
    done
    echo "CoppeliaSim did not exit after the remote quit request" >&2
    exit 1
  fi
  sleep 1
done

echo "CoppeliaSim ZeroMQ Remote API did not start. Log: $log_file" >&2
exit 1
