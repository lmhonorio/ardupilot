#!/usr/bin/env bash
set -eo pipefail
here=$(cd -- "$(dirname -- "$0")" && pwd)
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/mid360-bridge-$(id -u).lock"
flock -n 9 || { echo 'A ponte MID360 ja esta rodando neste computador.' >&2; exit 1; }
source "$here/../ros2/setup.bash"
unset FASTRTPS_DEFAULT_PROFILES_FILE
export ROS_DOMAIN_ID=21 ROS_LOCALHOST_ONLY=1 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
exec python "$here/receber.py" "$@"
