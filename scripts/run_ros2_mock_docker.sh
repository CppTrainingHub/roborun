#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
image="roborun-ros2-humble:local"

docker build -f "$project_root/docker/ros2-humble/Dockerfile" -t "$image" "$project_root"

docker run --rm \
  --mount "type=bind,source=$project_root,target=/workspace" \
  --workdir /workspace \
  "$image" \
  bash -lc '
    source /opt/ros/humble/setup.bash
    set -eo pipefail
    colcon --log-base /tmp/roborun-colcon/log build --base-paths ros2 \
      --packages-select roborun_interfaces roborun_ros2_bridge \
      --build-base /tmp/roborun-colcon/build \
      --install-base /tmp/roborun-colcon/install
    source /tmp/roborun-colcon/install/setup.bash
    colcon --log-base /tmp/roborun-colcon/log test --base-paths ros2 \
      --packages-select roborun_interfaces roborun_ros2_bridge \
      --build-base /tmp/roborun-colcon/build \
      --install-base /tmp/roborun-colcon/install \
      --event-handlers console_direct+
    colcon --log-base /tmp/roborun-colcon/log test-result \
      --test-result-base /tmp/roborun-colcon/build --verbose
    /tmp/roborun-colcon/install/roborun_ros2_bridge/lib/roborun_ros2_bridge/roborun_bridge_node &
    bridge_pid=$!
    trap "kill $bridge_pid 2>/dev/null || true; wait $bridge_pid 2>/dev/null || true" EXIT
    python3 ros2/roborun_ros2_bridge/test/dss_mock_client.py
  '
