#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /opt/rover/Rover/ros2/install/local_setup.bash
export PATH="/opt/companion-venv/bin:$PATH"
export PYTHONPATH="/opt/rover/Rover/python:${PYTHONPATH:-}"
cd /opt/rover/Rover
if [[ ${1:-stack} == stack ]]; then
  exec python companion/stack.py --config "${ROVER_CONFIG:-/config/simulation.yaml}"
fi
exec "$@"
