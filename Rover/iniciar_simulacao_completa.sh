#!/usr/bin/env bash
set -eo pipefail
DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$DIR/ros2/setup.bash"
AVOIDANCE=1; GLIM=0; GUI=true; REBUILD=1
while (($#)); do
 case "$1" in
  --without-avoidance|--sem-desvio) AVOIDANCE=0;;
  --avoidance) AVOIDANCE=1;;
  --glim) GLIM=1;;
  --headless) GUI=false;;
  --no-rebuild) REBUILD=0;;
  --lua) :;;
  --help|-h) echo 'Uso: ./iniciar_simulacao_completa.sh [--without-avoidance] [--glim] [--headless] [--no-rebuild]'; exit 0;;
  *) echo "Opcao desconhecida: $1" >&2; exit 2;;
 esac
 shift
done
for file in "$DIR/ros2/generated/worlds/gravel_plane.sdf" "$DIR/ros2/install/lib/fast_lio/fastlio_mapping"; do
 [[ -f "$file" ]] || { echo "Ausente: $file. Execute ./ros2/build.sh" >&2; exit 1; }
done
if ss -H -ltn 'sport = :5760' | grep -q .; then echo 'SITL ja esta em execucao na porta 5760.' >&2; exit 1; fi
command -v gnome-terminal >/dev/null || { echo 'gnome-terminal ausente; use ./ros2/service.sh em terminais separados.' >&2; exit 1; }
open_service() { local name=$1; shift; gnome-terminal --title="Rover ROS2: $name" -- bash --noprofile --norc -c '"$1" "$2" "${@:3}"; result=$?; echo "Servico encerrado (codigo $result). Pressione Enter."; read -r' _ "$DIR/ros2/service.sh" "$name" "$@"; }
open_service gazebo "gui:=$GUI"
sitl_args=(--add-param-file "$DIR/nroverobst.param")
(( AVOIDANCE )) || sitl_args=(--add-param-file "$DIR/nrover_sem_desvio.param")
(( REBUILD )) || sitl_args+=(--no-rebuild)
open_service sitl "${sitl_args[@]}"
open_service mavros
open_service fastlio
open_service mapa
(( AVOIDANCE == 0 )) || open_service obstaculos
if (( GLIM )); then open_service glim; open_service rviz-glim; fi
open_service camera
open_service rviz
echo 'Servicos abertos. O rover permanece desarmado; carregue a missao pelo QGroundControl.'
echo 'Docking e iniciado separadamente: ./ros2/service.sh docking-qr OU docking-lidar.'
