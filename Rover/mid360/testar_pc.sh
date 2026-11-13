#!/usr/bin/env bash
set -eo pipefail
here=$(cd -- "$(dirname -- "$0")" && pwd)
source "$here/../ros2/setup.bash"
unset FASTRTPS_DEFAULT_PROFILES_FILE
export ROS_DOMAIN_ID=21 ROS_LOCALHOST_ONLY=1 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
exec python "$here/testar.py" "$@"
