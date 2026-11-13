#!/usr/bin/env bash
set -eo pipefail
here=$(cd -- "$(dirname -- "$0")" && pwd)
source "$here/../../ros2/setup.bash"
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE ROS_DISCOVERY_SERVER
export ROS_DOMAIN_ID=23 ROS_LOCALHOST_ONLY=1 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROVER_USE_SIM_TIME=false OPENBLAS_NUM_THREADS=1
exec python "$here/visualizar.py" "$@"
