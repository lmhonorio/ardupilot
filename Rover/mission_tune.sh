#!/usr/bin/env bash
set -e
DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$DIR/ros2/setup.bash"
exec "$DIR/ros2/venv/bin/python" "$DIR/python/missions/run.py" "$@"
