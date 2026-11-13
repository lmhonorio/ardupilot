#!/usr/bin/env bash
set -euo pipefail
here=$(cd -- "$(dirname -- "$0")" && pwd)
remote=${ROVER_REMOTE:-rover@192.168.0.167}
[[ "$remote" =~ ^[a-zA-Z0-9_.-]+@[a-zA-Z0-9_.-]+$ ]] || exit 2
case "${1:-}" in ''|--sem-rviz) ;; *) echo 'Uso: iniciar.sh [--sem-rviz]'; exit 2;; esac
if ! ssh "$remote" 'test -x ~/mid360/fast_lio_raspberry/ws/install/lib/mid360_lio_adapter/timing_adapter'; then
  "$here/preparar.sh"
fi
# The physical sensor driver is independent of simulation and vehicle control.
had_alias=0
if ssh "$remote" 'ip -4 -o addr show dev eth0 | grep -Fq "192.168.1.50/24"'; then had_alias=1; fi
"$here/../control.sh" up
if [[ $had_alias == 0 ]]; then ssh "$remote" 'docker restart rover-mid360 >/dev/null'; fi
ssh "$remote" 'set -e
  ping -I 192.168.1.50 -c 1 -W 2 192.168.1.161 >/dev/null
  cd ~/mid360/fast_lio_raspberry
  docker compose up -d'
echo 'FAST-LIO na Raspberry iniciado. Fechar o RViz nao encerra o FAST-LIO.'
if [[ ${1:-} != --sem-rviz ]]; then exec "$here/visualizar.sh"; fi
