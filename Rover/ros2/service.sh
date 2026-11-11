#!/usr/bin/env bash
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
caller_dir="$PWD"
source "$root/setup.bash"
cd "$root/.."
service="${1:-help}"
(($# == 0)) || shift
mkdir -p "$root/run"
if [[ "$service" != help ]]; then
  # Descriptor remains open across exec, preventing duplicate commanders/subscribers.
  lock="$service"
  case "$service" in docking-*|sair-*|mission-manager) lock=docking-controller; export ROVER_DOCKING_LOCKED=1;; esac
  exec 9>"$root/run/$lock.lock"
  flock -n 9 || { echo "Servico ja esta rodando: $lock" >&2; exit 1; }
fi
case "$service" in
  gazebo) exec ros2 launch "$root/launch/simulation.launch.py" "$@";;
  sitl) exec "$root/sitl.sh" "$@";;
  mavros) exec ros2 launch "$root/launch/mavros.launch.py" "$@";;
  fastlio) exec python python/slam/fast_lio/run_fast_lio.py "$@";;
  glim) exec python python/slam/glim/run_glim.py "$@";;
  obstaculos) exec python python/obstacle_avoidance/manager.py "$@";;
  mapa) exec python python/slam/slam_mid360_3d.py "$@";;
  mapa2d) exec python python/slam/slam_mid360_2d.py "$@";;
  camera) exec python python/camera/read_ros_camera.py "$@";;
  observar-qr) exec python python/docking/observar_qr.py "$@";;
  lidar) exec python python/lidar/read_ros_lidar.py "$@";;
  docking-qr) exec python python/docking/entrar_container.py "$@";;
  sair-qr) exec python python/docking/sair_container.py "$@";;
  docking-lidar) exec python python/docking_lidar/entrar_lidar.py "$@";;
  sair-lidar) exec python python/docking_lidar/sair_lidar.py "$@";;
  mission-manager) export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1; exec python python/mission_manager/manager.py "$@";;
  mission-start) cd "$caller_dir"; exec python "$root/../python/mission_manager/start_mission.py" "$@";;
  rviz) exec rviz2 -d "$root/config/mid360_3d.rviz" --ros-args -p use_sim_time:=true "$@";;
  rviz-glim|rviz_glim) exec rviz2 -d "$root/config/glim.rviz" --ros-args -p use_sim_time:=true "$@";;
  *) echo 'Servicos: gazebo sitl mavros fastlio glim obstaculos mapa mapa2d camera observar-qr lidar rviz rviz-glim docking-qr sair-qr docking-lidar sair-lidar mission-manager mission-start';;
esac
