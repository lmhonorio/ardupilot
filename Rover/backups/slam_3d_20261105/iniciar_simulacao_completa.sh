#!/usr/bin/env bash
set -euo pipefail
DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKER="$DIR/python/simulation/terminal_integrado.sh"
usage() { echo "Uso: $0 [--sem-camera] [--sem-mapa] [--sem-rviz] [--sem-qgc]"; }
CAM=1 MAPA=1 RVIZ=1 QGC=1
for arg in "$@"; do
  case "$arg" in
    --sem-camera) CAM=0 ;; --sem-mapa) MAPA=0 ;; --sem-rviz) RVIZ=0 ;; --sem-qgc) QGC=0 ;;
    --help|-h) usage; exit 0 ;; *) usage >&2; exit 2 ;;
  esac
done
for file in /opt/ros/noetic/setup.bash "$HOME/catkin_ws/devel/setup.bash" \
  "$HOME/.pyenv/versions/3.8.10/bin/python3.8" "$DIR/../build/sitl/bin/ardurover" \
  "$DIR/python/.deps/fast_lio_ws/devel/lib/fast_lio/fastlio_mapping"; do
  if [[ ! -e "$file" ]]; then echo "Dependencia ausente: $file" >&2; exit 1; fi
done
for executable in gnome-terminal timeout ss pgrep; do
  if ! command -v "$executable" >/dev/null; then echo "Programa ausente: $executable" >&2; exit 1; fi
done
if [[ -z "${DISPLAY:-}" ]]; then echo 'Abra uma sessao grafica para iniciar as janelas.' >&2; exit 1; fi
if pgrep -x gzserver >/dev/null || pgrep -f '[/]Tools/autotest/sim_vehicle.py' >/dev/null ||
   pgrep -x -f '.*/(ardurover|arducopter|arduplane|ardusub)( .*)?' >/dev/null ||
   ss -H -ltn 'sport = :5760' | grep -q .; then
  echo 'Ja existe Gazebo, sim_vehicle ou SITL em execucao. Encerre a simulacao anterior antes de iniciar.' >&2; exit 1
fi
if pgrep -f '[p]ython.*/slam/fast_lio/run_fast_lio.py' >/dev/null || "$DIR/start_obstacle_avoidance.sh" --status >/dev/null 2>&1; then
  echo 'FAST-LIO ou o modulo de obstaculos ja esta ativo. Encerre-o antes de iniciar.' >&2; exit 1
fi
source /opt/ros/noetic/setup.bash
source "$HOME/catkin_ws/devel/setup.bash"
if timeout 3 rosnode list 2>/dev/null | grep -Eq '^/(gazebo|olhe_obstacle_processor|olhe_obstacle_mavlink_bridge|rover_slam/hector_mapping)$'; then
  echo 'Ha componentes ROS do cenario em execucao. Encerre-os antes de iniciar.' >&2; exit 1
fi
open_window() {
  local title="$1" component="$2"
  gnome-terminal --title="$title" -- bash -c '
    component=$1; worker=$2
    echo "Iniciando $component. Feche com Ctrl+C."
    "$worker" "$component"; result=$?
    echo "Componente $component encerrado (codigo $result). Pressione Enter para fechar."
    read -r || true
  ' _ "$component" "$WORKER"
}
wait_until() {
  local label="$1" limit="$2"; shift 2
  local deadline=$((SECONDS+limit))
  echo "Aguardando $label (ate ${limit}s)..."
  while (( SECONDS < deadline )); do
    if "$@"; then return 0; fi
    sleep 1
  done
  echo "Tempo esgotado aguardando $label. Confira a janela correspondente e feche os componentes iniciados." >&2
  return 1
}
world_ready() {
  timeout 3 rostopic list 2>/dev/null | grep -Fxq '/rover_argo_N1/Instance1/livox/lidar' &&
  timeout 3 rostopic list 2>/dev/null | grep -Fxq '/rover_argo_N1/Instance1/livox/imu'
}
sitl_ready() {
  ss -H -ltn 'sport = :5760' | grep -q . && pgrep -f '[m]avproxy.py.*14557' >/dev/null
}
open_window '1 - Gazebo e ROS' gazebo
wait_until 'topicos MID360 e IMU no Gazebo' 120 world_ready
open_window '2 - ArduRover e MAVProxy' sitl
wait_until 'SITL e MAVProxy com porta 14557' 90 sitl_ready
open_window '3 - FAST-LIO' fastlio
open_window '4 - Percepcao de obstaculos' obstaculos
if (( MAPA )); then open_window '5 - Mapa 2D Hector' mapa; fi
if (( CAM )); then open_window '6 - Camera Samsung' camera; fi
if (( RVIZ )); then open_window '7 - RViz' rviz; fi
if (( QGC )); then
  if pgrep -x QGroundControl >/dev/null; then
    echo 'QGroundControl ja esta aberto; mantendo a instancia existente.'
  elif [[ -x "$HOME/Desktop/QGroundControl.AppImage" ]]; then
    nohup "$HOME/Desktop/QGroundControl.AppImage" > "$DIR/python/run/qgroundcontrol_integrado.log" 2>&1 < /dev/null &
    echo 'QGroundControl iniciado.'
  else
    echo 'QGroundControl nao encontrado em ~/Desktop/QGroundControl.AppImage; abra-o manualmente.' >&2
  fi
fi
cat <<'MSG'
Janelas solicitadas. Confira os terminais para confirmar que cada programa iniciou.
O perfil habilita BendyRuler em SITL; a percepcao e experimental e nao garante parada.
Para fechar, Ctrl+C nas janelas de percepcao, FAST-LIO, mapa/camera, SITL e Gazebo.
MSG
