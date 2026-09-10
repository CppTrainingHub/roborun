#!/usr/bin/env bash

set -euo pipefail

project_root="$(cd "$(dirname "$0")/.." && pwd)"
app_path="${COPPELIASIM_APP:?Set COPPELIASIM_APP to the CoppeliaSim application path.}"
resources_path="$app_path/Contents/Resources"
simulator_binary="$app_path/Contents/MacOS/coppeliaSim"
model_path="$resources_path/models/robots/non-mobile/UR5.ttm"
build_directory="${ROBORUN_BUILD_DIR:-$project_root/.build/lifecycle-control-coppeliasim}"
runtime_directory="$(mktemp -d)"
input_fifo="$runtime_directory/coppeliasim.stdin"
log_file="${TMPDIR:-/tmp}/roborun-lifecycle-control-coppeliasim-stop.log"

if [[ ! -x "$simulator_binary" || ! -f "$model_path" ]]; then
  echo "CoppeliaSim application is incomplete: $app_path" >&2
  exit 2
fi
if lsof -nP -iTCP:23000 -sTCP:LISTEN >/dev/null 2>&1; then
  echo "TCP port 23000 is already in use; stop the existing service first" >&2
  exit 2
fi

if [[ ! -x "$build_directory/roborun_coppeliasim_stop_probe" ]]; then
  cmake -S "$project_root" -B "$build_directory" -DBUILD_TESTING=OFF \
    -DROBORUN_ENABLE_COPPELIASIM=ON -DCOPPELIASIM_ROOT_DIR="$resources_path"
  cmake --build "$build_directory" --target roborun_coppeliasim_stop_probe -j2
fi

mkfifo "$input_fifo"
sleep 2147483647 >"$input_fifo" &
input_pid=$!
"$simulator_binary" -h -GsimCmd.autoStart=false <"$input_fifo" >"$log_file" 2>&1 &
simulator_pid=$!
cleanup() {
  kill "$input_pid" "$simulator_pid" 2>/dev/null || true
  wait "$input_pid" "$simulator_pid" 2>/dev/null || true
  rm -f "$input_fifo"
  rmdir "$runtime_directory" 2>/dev/null || true
}
trap cleanup EXIT

for _ in {1..30}; do
  if nc -z 127.0.0.1 23000 2>/dev/null; then
    "$build_directory/roborun_coppeliasim_stop_probe" "$model_path"
    exit $?
  fi
  sleep 1
done

echo "CoppeliaSim ZeroMQ Remote API did not start. Log: $log_file" >&2
exit 1
