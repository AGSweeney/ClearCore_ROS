#!/usr/bin/env bash
# MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com>
# 30-minute stay-enabled ROS random-move endurance (JTC). No mid-run disable/disconnect.
set -eo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export MAMBA_ROOT_PREFIX="${HOME}/micromamba"
eval "$("${HOME}/bin/micromamba" shell hook -s bash)"
set +u
micromamba activate ros_env
cd "$REPO_ROOT/ros2_ws"
source install/setup.bash
set +u

export CCROS_HOST=172.16.82.114
export CCROS_BATTERY_SECONDS="${CCROS_BATTERY_SECONDS:-1800}"
export CCROS_WS="$REPO_ROOT/ros2_ws"
export CCROS_LOG_ROOT="$REPO_ROOT/logs"
export CCROS_REPO="$REPO_ROOT"

exec python3 "$REPO_ROOT/host/run_ros_random_stay_enabled_30m.py"
