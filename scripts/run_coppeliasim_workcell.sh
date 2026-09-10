#!/usr/bin/env bash

set -euo pipefail

project_root="$(cd "$(dirname "$0")/.." && pwd)"
app_path="${COPPELIASIM_APP:?Set COPPELIASIM_APP to the CoppeliaSim application path.}"
resources_path="$app_path/Contents/Resources"
simulator_binary="$app_path/Contents/MacOS/coppeliaSim"
coppeliasim_python="${COPPELIASIM_PYTHON:-}"
build_directory="${ROBORUN_BUILD_DIR:-$project_root/.build/workcell-coppeliasim}"
results_directory="$build_directory/workcell-check-results"
scenes_directory="$build_directory/workcell-check-scenes"
diagnostics_directory="$HOME/Library/Logs/DiagnosticReports"
active_simulator_pid=""
active_input_pid=""
active_runtime_directory=""

stop_owned_process() {
  local process_id="$1"
  local attempt

  kill "$process_id" 2>/dev/null || true
  for attempt in {1..50}; do
    if ! kill -0 "$process_id" 2>/dev/null; then
      break
    fi
    sleep 0.1
  done
  if kill -0 "$process_id" 2>/dev/null; then
    kill -KILL "$process_id" 2>/dev/null || true
  fi
  wait "$process_id" 2>/dev/null || true
}

cleanup_active_processes() {
  if [[ -n "$active_input_pid" ]]; then
    stop_owned_process "$active_input_pid"
  fi
  if [[ -n "$active_simulator_pid" ]]; then
    stop_owned_process "$active_simulator_pid"
  fi
  if [[ -n "$active_runtime_directory" ]]; then
    rm -f "$active_runtime_directory/coppeliasim.stdin"
    rmdir "$active_runtime_directory" 2>/dev/null || true
  fi
}
trap cleanup_active_processes EXIT

if [[ -z "$coppeliasim_python" ]]; then
  echo "Set COPPELIASIM_PYTHON to the Python interpreter used by CoppeliaSim." >&2
  exit 2
fi
if [[ ! -x "$coppeliasim_python" ]]; then
  echo "COPPELIASIM_PYTHON is not an executable Python interpreter: $coppeliasim_python" >&2
  exit 2
fi
if ! "$coppeliasim_python" -c 'import zmq, cbor2' >/dev/null 2>&1; then
  echo "COPPELIASIM_PYTHON must provide pyzmq and cbor2: $coppeliasim_python" >&2
  exit 2
fi

if [[ ! -x "$simulator_binary" || \
      ! -f "$resources_path/models/robots/non-mobile/UR5.ttm" || \
      ! -f "$resources_path/models/components/grippers/RG2.ttm" ]]; then
  echo "CoppeliaSim application is incomplete: $app_path" >&2
  exit 2
fi
if lsof -nP -iTCP:23000 -sTCP:LISTEN >/dev/null 2>&1; then
  echo "TCP port 23000 is already in use; stop the existing service first" >&2
  exit 2
fi

cmake -S "$project_root" -B "$build_directory" -DBUILD_TESTING=ON \
  -DROBORUN_ENABLE_COPPELIASIM=ON -DCOPPELIASIM_ROOT_DIR="$resources_path"
cmake --build "$build_directory" --target roborun -j2
mkdir -p "$results_directory" "$scenes_directory"

count_crash_reports() {
  if [[ ! -d "$diagnostics_directory" ]]; then
    echo 0
    return
  fi
  find "$diagnostics_directory" -maxdepth 1 -type f \
    \( -iname '*coppeliaSim*.crash' -o -iname '*coppeliaSim*.ips' \) | wc -l | tr -d ' '
}

run_case() {
  local name="$1"
  local workcell_config="$2"
  local task="$3"
  local expected_exit="$4"
  local rebuild_scene="$5"
  shift 5
  local scene_key="${name%%-*}"
  local scene_path="$scenes_directory/$scene_key.ttt"
  local simulator_log="$results_directory/$name-simulator.log"
  local result_file="$results_directory/$name-result.log"
  local input_fifo
  local client_status

  if [[ "$rebuild_scene" == "true" ]]; then
    rm -f "$scene_path"
  fi
  rm -f "$simulator_log" "$result_file"
  active_runtime_directory="$(mktemp -d)"
  input_fifo="$active_runtime_directory/coppeliasim.stdin"
  mkfifo "$input_fifo"
  while :; do sleep 30; done >"$input_fifo" &
  active_input_pid=$!
  "$simulator_binary" -h -GsimCmd.autoStart=false \
    "-Gpython=$coppeliasim_python" \
    <"$input_fifo" >"$simulator_log" 2>&1 &
  active_simulator_pid=$!

  for _ in {1..60}; do
    if nc -z 127.0.0.1 23000 2>/dev/null; then
      break
    fi
    if ! kill -0 "$active_simulator_pid" 2>/dev/null; then
      echo "$name: CoppeliaSim exited before opening port 23000" >&2
      return 1
    fi
    sleep 1
  done
  if ! nc -z 127.0.0.1 23000 2>/dev/null; then
    echo "$name: CoppeliaSim ZeroMQ Remote API did not start" >&2
    return 1
  fi

  set +e
  "$build_directory/roborun" run --backend coppeliasim \
    --robot-config "$project_root/examples/config/ur5_robot.json" \
    --points-config "$project_root/examples/config/ur5_points.json" \
    --io-config "$project_root/examples/config/workcell_io.json" \
    --tool-config "$project_root/examples/config/tool_feedback.json" \
    --workcell-config "$project_root/$workcell_config" \
    --resources "$resources_path" --scene "$scene_path" \
    --program "$project_root/$task" --format trace "$@" >"$result_file" 2>&1
  client_status=$?
  set -e
  if [[ "$client_status" -ne "$expected_exit" ]]; then
    echo "$name: expected client exit $expected_exit, got $client_status" >&2
    return 1
  fi

  for _ in {1..30}; do
    if ! kill -0 "$active_simulator_pid" 2>/dev/null; then
      break
    fi
    sleep 1
  done
  if kill -0 "$active_simulator_pid" 2>/dev/null; then
    echo "$name: simulator did not exit after quitSimulator" >&2
    return 1
  fi
  wait "$active_simulator_pid" 2>/dev/null || true
  active_simulator_pid=""
  stop_owned_process "$active_input_pid"
  active_input_pid=""
  rm -f "$input_fifo"
  rmdir "$active_runtime_directory"
  active_runtime_directory=""
  if lsof -nP -iTCP:23000 -sTCP:LISTEN >/dev/null 2>&1; then
    echo "$name: TCP port 23000 remained open" >&2
    return 1
  fi
  rg -q '^LIFECYCLE operation=stop_simulation status=passed$' "$result_file"
  rg -q '^LIFECYCLE operation=disconnect status=passed$' "$result_file"
  rg -q '^LIFECYCLE operation=quit_simulator status=passed$' "$result_file"
  rg '^(LIFECYCLE|ALARM|WORKCELL|RESULT)' "$result_file"
}

crashes_before="$(count_crash_reports)"
for run in 1 2 3; do
  rebuild_scene=false
  if [[ "$run" -eq 1 ]]; then
    rebuild_scene=true
  fi
  run_case "normal-$run" examples/config/workcell.json \
    examples/tasks/pick_place.task 0 "$rebuild_scene"
  result_file="$results_directory/normal-$run-result.log"
  rg -q '^RESULT status=passed .*outcome=program_completed$' "$result_file"
  rg -q '^WORKCELL .*workpiece_present=true .*workpiece_attached=false .*workpiece_at_place=true .*collision=false .*simulation_running=true' "$result_file"
done

run_case missing examples/config/missing_workpiece_workcell.json \
  examples/tasks/pick_place.task 1 true
rg -q '^ALARM .*code=DI_TIMEOUT ' "$results_directory/missing-result.log"
rg -q '^WORKCELL .*workpiece_present=false ' "$results_directory/missing-result.log"

run_case grip-failure examples/config/workcell.json \
  examples/tasks/grip_failure.task 1 true
rg -q '^ALARM .*code=GRIP_FAILURE ' "$results_directory/grip-failure-result.log"

run_case collision examples/config/collision_workcell.json \
  examples/tasks/collision.task 1 true
rg -q '^ALARM .*code=COLLISION_DETECTED ' "$results_directory/collision-result.log"
rg -q '^WORKCELL .*collision=true ' "$results_directory/collision-result.log"

run_case estop examples/config/workcell.json \
  examples/tasks/pick_place.task 1 true --estop-after-move-polls 5
rg -q '^RESULT status=failed .*outcome=emergency_stopped$' "$results_directory/estop-result.log"
rg -q '^TRACE .*control=estop .*control_source="cli-check"' "$results_directory/estop-result.log"
if rg -q '^TRACE .*command_index=([5-9]|10) ' "$results_directory/estop-result.log"; then
  echo "estop: a later task step ran after ESTOP" >&2
  exit 1
fi

crashes_after="$(count_crash_reports)"
if [[ "$crashes_after" -ne "$crashes_before" ]]; then
  echo "CoppeliaSim generated a new crash report" >&2
  exit 1
fi
echo "RESULT status=passed phase=workcell_check processes=7 normal_runs=3 fault_runs=4"
