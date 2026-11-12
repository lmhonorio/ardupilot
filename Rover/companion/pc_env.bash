# source ~/ardupilot/Rover/companion/pc_env.bash
companion_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source "$companion_root/ros2/setup.bash"
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export ROVER_REMOTE=${ROVER_REMOTE:-leo@192.168.0.198}
unset companion_root
