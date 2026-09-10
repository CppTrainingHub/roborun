#!/usr/bin/env bash

set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
app_path="${COPPELIASIM_APP:?Set COPPELIASIM_APP to the CoppeliaSim application path.}"
resources_path="$app_path/Contents/Resources"
image="roborun-ros2-humble:local"
started_simulator_pid=""

cleanup() {
  if [[ -n "$started_simulator_pid" ]]; then
    kill "$started_simulator_pid" 2>/dev/null || true
    wait "$started_simulator_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT

if [[ ! -x "$app_path/Contents/MacOS/coppeliaSim" || \
      ! -f "$resources_path/models/robots/non-mobile/UR5.ttm" ]]; then
  echo "CoppeliaSim application is incomplete: $app_path" >&2
  exit 2
fi

if ! nc -z 127.0.0.1 23000 2>/dev/null; then
  open -a "$app_path" --args -h -GsimCmd.autoStart=false
  for _ in {1..30}; do
    if nc -z 127.0.0.1 23000 2>/dev/null; then
      started_simulator_pid="$(lsof -tiTCP:23000 -sTCP:LISTEN | head -n 1)"
      break
    fi
    sleep 1
  done
fi
if ! nc -z 127.0.0.1 23000 2>/dev/null; then
  echo "CoppeliaSim ZeroMQ Remote API did not start on port 23000" >&2
  exit 2
fi

if [[ "${ROBORUN_SKIP_DOCKER_BUILD:-0}" != "1" ]]; then
  docker build -f "$project_root/docker/ros2-humble/Dockerfile" -t "$image" "$project_root"
fi

docker run --rm -i \
  --mount "type=bind,source=$project_root,target=$project_root" \
  --mount "type=bind,source=$resources_path,target=$resources_path,readonly" \
  --workdir "$project_root" \
  -e PROJECT_ROOT="$project_root" \
  -e COPPELIASIM_RESOURCES="$resources_path" \
  "$image" \
  bash -s <<'CONTAINER'
source /opt/ros/humble/setup.bash
set -eo pipefail

build_root="$PROJECT_ROOT/.build/ros2-coppeliasim"
build_base="$build_root/colcon-build"
install_base="$build_root/colcon-install"
log_base="$build_root/colcon-log"
scene_path="$build_root/workcell_bridge_scene.ttt"

colcon --log-base "$log_base" build --base-paths ros2 \
  --packages-select roborun_interfaces roborun_ros2_bridge \
  --build-base "$build_base" --install-base "$install_base" \
  --cmake-args -DROBORUN_BRIDGE_ENABLE_COPPELIASIM=ON \
    -DCOPPELIASIM_ROOT_DIR="$COPPELIASIM_RESOURCES"
source "$install_base/setup.bash"
colcon --log-base "$log_base" test --base-paths ros2 \
  --packages-select roborun_interfaces roborun_ros2_bridge \
  --build-base "$build_base" --install-base "$install_base" \
  --event-handlers console_direct+
colcon --log-base "$log_base" test-result --test-result-base "$build_base" --verbose

run_case() {
  local mode="$1"
  "$install_base/roborun_ros2_bridge/lib/roborun_ros2_bridge/roborun_bridge_node" \
    --ros-args \
    -p backend:=coppeliasim \
    -p coppeliasim_host:=host.docker.internal \
    -p coppeliasim_port:=23000 \
    -p robot_config:="$PROJECT_ROOT/examples/config/ur5_robot.json" \
    -p points_config:="$PROJECT_ROOT/examples/config/ur5_points.json" \
    -p io_config:="$PROJECT_ROOT/examples/config/workcell_io.json" \
    -p tool_config:="$PROJECT_ROOT/examples/config/tool_feedback.json" \
    -p workcell_config:="$PROJECT_ROOT/examples/config/workcell.json" \
    -p resources_path:="$COPPELIASIM_RESOURCES" \
    -p scene_path:="$scene_path" &
  local bridge_pid=$!
  trap 'kill "$bridge_pid" 2>/dev/null || true; wait "$bridge_pid" 2>/dev/null || true' RETURN
  python3 "$PROJECT_ROOT/ros2/roborun_ros2_bridge/test/dss_coppeliasim_client.py" --mode "$mode"
  kill "$bridge_pid" 2>/dev/null || true
  wait "$bridge_pid" 2>/dev/null || true
  trap - RETURN
}

run_case normal
run_case cancel
run_case estop
CONTAINER
