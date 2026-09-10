#!/usr/bin/env bash

set -euo pipefail

project_root="$(cd "$(dirname "$0")/.." && pwd)"
exec python3 "$project_root/scripts/linux_gui_evidence.py" run "$@"
