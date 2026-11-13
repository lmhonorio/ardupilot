#!/usr/bin/env bash
# Execute no computador; aquisicao real pela Ethernet da Raspberry.
set -euo pipefail
here=$(cd -- "$(dirname -- "$0")" && pwd)
remote=${ROVER_REMOTE:-rover@192.168.0.167}
[[ "$remote" =~ ^[a-zA-Z0-9_.-]+@[a-zA-Z0-9_.-]+$ ]] || exit 2
case "${1:-help}" in
  up)
    ssh "$remote" 'mkdir -p ~/mid360/run'
    scp "$here"/{descobrir.py,rede.py,testar.py,compose.yaml,mid360.json,livox.yaml,enviar.py} "$remote:mid360/"
    ssh "$remote" 'set -e; docker run --rm --network host --cap-add NET_ADMIN -v "$HOME/mid360:/mid360" rover-companion:jazzy python /mid360/rede.py up; cd ~/mid360; docker compose up -d'
    ;;
  down) ssh "$remote" 'cd ~/mid360 && docker compose down';;
  status) ssh "$remote" 'cd ~/mid360 && docker compose ps; ip -4 addr show dev eth0';;
  logs) ssh "$remote" 'cd ~/mid360 && docker compose logs --tail 50 -f';;
  test) ssh "$remote" 'docker exec -e ROS_DOMAIN_ID=21 rover-mid360 /opt/rover/Rover/companion/entrypoint.sh python /mid360/testar.py --seconds 20';;
  network-down)
    ssh "$remote" 'set -e; cd ~/mid360; docker compose down; docker run --rm --network host --cap-add NET_ADMIN -v "$HOME/mid360:/mid360" rover-companion:jazzy python /mid360/rede.py down'
    ;;
  *) echo 'Uso: ./mid360/control.sh {up|status|test|logs|down|network-down}';;
esac
