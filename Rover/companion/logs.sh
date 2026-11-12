#!/usr/bin/env bash
set -eo pipefail
docker logs --tail 20 rover-companion 2>&1 || true
root="$HOME/rover_companion/data"
[[ -f "$root/latest.txt" ]] || exit 0
run=$(cat "$root/latest.txt")
[[ "$run" == /data/runs/* ]] || exit 1
run="$root/${run#/data/}"
for service in manager fastlio map mavproxy sensor_ingress simulation_adapter; do
  if [[ -f "$run/$service.log" ]]; then
    printf '\n%s: %s\n' "$service" "$run/$service.log"
    tail -15 "$run/$service.log"
  fi
done
