#!/usr/bin/env bash
DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# ROS 2: modulo em primeiro plano; Ctrl+C encerra seus dois processos.
exec "$DIR/ros2/service.sh" obstaculos "$@"
