#!/usr/bin/env bash
set -eo pipefail
here=$(cd -- "$(dirname -- "$0")" && pwd)
source "$here/pc_env.bash"
action=${1:-help}; (($# == 0)) || shift
case "$action" in
  mission) exec python "$here/mission_remote.py" "$@";;
  status) exec ros2 topic echo /mission_manager/status --once --full-length;;
  watch) exec ros2 topic echo /mission_manager/status --full-length;;
  reconcile|cancel) exec ros2 service call "/mission_manager/$action" std_srvs/srv/Trigger '{}';;
  up) python "$here/check_local.py"; exec ssh "$ROVER_REMOTE" 'cd ~/rover_companion; docker compose up -d';;
  down) exec ssh "$ROVER_REMOTE" 'cd ~/rover_companion; docker compose down';;
  logs) exec ssh "$ROVER_REMOTE" 'bash ~/rover_companion/source/companion/logs.sh';;
  *) echo 'Uso: companion/control.sh {mission ARQUIVO|status|watch|reconcile|cancel|up|down|logs}';;
esac
