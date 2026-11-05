#!/usr/bin/env bash
set -euo pipefail
DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
RUN="$DIR/python/run/obstacle_avoidance"
MANAGER="$DIR/python/obstacle_avoidance/manager.py"
mkdir -p "$RUN"
exec 9>"$RUN/lock"
flock -x 9
running() {
  [[ -f "$RUN/pid" ]] || return 1
  local pid; read -r pid < "$RUN/pid"
  [[ "$pid" =~ ^[0-9]+$ && -r "/proc/$pid/cmdline" ]] || return 1
  tr '\0' '\n' < "/proc/$pid/cmdline" | grep -Fxq "$MANAGER"
}
case "${1:-start}" in
  --status)
    if running; then echo "Modulo ativo (PID $(cat "$RUN/pid")); log: $RUN/module.log"; else echo 'Modulo parado'; exit 1; fi ;;
  --stop)
    if running; then kill -TERM "$(cat "$RUN/pid")"; echo 'Encerramento solicitado somente ao modulo de obstaculos.'; else echo 'Modulo ja estava parado.'; fi ;;
  start)
    if running; then echo 'Modulo ja esta ativo.'; exit 0; fi
    if ! /usr/bin/python3.8 -c 'import rospy; rospy.get_master().getPid()' >/dev/null 2>&1; then echo 'ROS master indisponivel. Inicie a simulacao primeiro.' >&2; exit 1; fi
    if command -v rosnode >/dev/null && { timeout 4 rosnode ping -c 1 /olhe_obstacle_processor 2>&1 | grep -q 'xmlrpc reply from' || timeout 4 rosnode ping -c 1 /olhe_obstacle_mavlink_bridge 2>&1 | grep -q 'xmlrpc reply from'; }; then echo 'Ja existe um dos nos OLH-E; nao iniciar duplicado.' >&2; exit 1; fi
    nohup /usr/bin/python3.8 "$MANAGER" > "$RUN/module.log" 2>&1 < /dev/null 9>&- &
    echo "$!" > "$RUN/pid"
    echo "Modulo iniciado. Log: $RUN/module.log"
    ;;
  *) echo "Uso: $0 [--status|--stop]" >&2; exit 2 ;;
esac
