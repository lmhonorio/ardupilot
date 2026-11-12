#!/usr/bin/env bash
# Package only source code; never transfer old runs, virtualenvs or credentials.
set -euo pipefail
here=$(cd -- "$(dirname -- "$0")" && pwd)
rover=$(cd -- "$here/.." && pwd)
remote=${ROVER_REMOTE:-leo@192.168.0.198}
[[ "$remote" =~ ^[a-zA-Z0-9_.-]+@[a-zA-Z0-9_.-]+$ ]] || exit 2
archive=$(mktemp --suffix=.tgz);trap 'rm -f "$archive"' EXIT
# FAST-LIO sources are pinned by ros2/dependencies.lock and patched on build.
tar -czf "$archive" --exclude=.git --exclude=__pycache__ --exclude=python/run --exclude=python/.deps \
  --exclude=companion/data --exclude=companion/missions --exclude=companion/.env \
  --exclude=ros2/deps/FAST_LIO/Log --exclude=ros2/deps/FAST_LIO/PCD \
  -C "$rover" companion python estacoes_recarga.txt ros2/dependencies.lock \
  ros2/src/rover_mission_interfaces ros2/src/rover_companion_bridge ros2/src/livox_ros_driver2 ros2/deps/FAST_LIO
ssh "$remote" 'mkdir -p ~/rover_companion/source ~/rover_companion/data ~/rover_companion/missions ~/rover_companion/config'
scp "$archive" "$remote:rover_companion/source.tgz"
ssh "$remote" 'set -e; cd ~/rover_companion; tar -xzf source.tgz -C source; cp source/companion/compose.yaml .; cp source/companion/compose.real.yaml .; for f in source/companion/config/*; do test -e "config/$(basename "$f")" || cp "$f" config/; done; docker build --progress plain -t rover-companion:jazzy -f source/companion/Dockerfile source'
echo 'Imagem criada. Pare os controladores locais antes de companion/control.sh up.'
