#!/usr/bin/env bash
# Apenas RViz no computador. O driver fisico fica na Pi em dominio separado.
set -eo pipefail
here=$(cd -- "$(dirname -- "$0")" && pwd)
source "$here/../ros2/setup.bash"
unset FASTRTPS_DEFAULT_PROFILES_FILE
export ROS_DOMAIN_ID=21 ROS_LOCALHOST_ONLY=1 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
exec rviz2 -d "$here/mid360.rviz" --ros-args -p use_sim_time:=false
