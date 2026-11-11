#!/usr/bin/env bash
set -eo pipefail
ROVER_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source /opt/ros/noetic/setup.bash
source "$HOME/catkin_ws/devel/setup.bash"
cd "$ROVER_DIR"
case "${1:-}" in
  gazebo)
    roslaunch rover_argo_gazebo multi_rover_argo.launch N:=1 world_name:=gravel_plane \
      start_x:=0 start_y:=0 z:=0 mid360_enabled:=true insta360_enabled:=true \
      samsung_s23_ultra_cam_enabled:=true hokuyo_enabled:=true ;;
  sitl)
    PYTHON38="$HOME/.pyenv/versions/3.8.10/bin/python3.8" ./sim_gz1rover.sh --foreground --no-rebuild --avoidance ;;
  fastlio)
    /usr/bin/python3.8 python/slam/fast_lio/run_fast_lio.py ;;
  glim)
    /usr/bin/python3.8 python/slam/glim/run_glim.py ;;
  obstaculos)
    if ./start_obstacle_avoidance.sh --status >/dev/null 2>&1; then
      echo 'O modulo ja esta em execucao; esta janela nao e proprietaria dele.'; exit 1
    fi
    ./start_obstacle_avoidance.sh
    trap './start_obstacle_avoidance.sh --stop' EXIT INT TERM
    pid=$(cat python/run/obstacle_avoidance/pid)
    tail --pid="$pid" -F python/run/obstacle_avoidance/module.log ;;
  mapa)
    /usr/bin/python3.8 python/slam/slam_mid360_3d.py ;;
  camera)
    /usr/bin/python3.8 python/camera/read_ros_camera.py ;;
  rviz)
    rviz -d python/slam/mid360_3d.rviz ;;
  rviz_glim)
    rviz -d python/slam/glim/glim_mid360.rviz ;;
  *) echo "Componente desconhecido: ${1:-}" >&2; exit 2 ;;
esac
