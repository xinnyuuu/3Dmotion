#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

set +u
source scripts/source_openvins_ros2.bash
set -u

exec /usr/bin/python3 scripts/session_replay_publisher.py "$@"
