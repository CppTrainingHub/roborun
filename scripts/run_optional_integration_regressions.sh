#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

"$project_root/scripts/run_ros2_mock_docker.sh"

if [[ "${ROBORUN_RUN_COPPELIASIM_REGRESSION:-0}" != "1" ]]; then
  echo "RESULT status=skipped phase=source_release_coppeliasim_regression reason=ROBORUN_RUN_COPPELIASIM_REGRESSION_not_set"
  exit 0
fi

: "${COPPELIASIM_APP:?Set COPPELIASIM_APP to run the CoppeliaSim/ROS 2 Bridge real integration regression.}"
ROBORUN_BUILD_DIR="$project_root/.build/source-release-workcell-coppeliasim" \
  "$project_root/scripts/run_coppeliasim_workcell.sh"
ROBORUN_SKIP_DOCKER_BUILD=1 \
  "$project_root/scripts/run_ros2_coppeliasim_docker.sh"
