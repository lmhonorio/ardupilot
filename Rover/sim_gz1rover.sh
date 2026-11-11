#!/usr/bin/env bash
set -eo pipefail
ROVER_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "$ROVER_DIR/ros2/sitl.sh" "$@"
