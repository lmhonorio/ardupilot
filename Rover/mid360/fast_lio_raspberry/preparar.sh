#!/usr/bin/env bash
# Run on the PC. Only builds the native timing adapter on the Pi.
set -euo pipefail
here=$(cd -- "$(dirname -- "$0")" && pwd)
remote=${ROVER_REMOTE:-rover@192.168.0.167}
[[ "$remote" =~ ^[a-zA-Z0-9_.-]+@[a-zA-Z0-9_.-]+$ ]] || exit 2
ssh "$remote" 'mkdir -p ~/mid360/fast_lio_raspberry/ws/src'
scp -q "$here"/{compose.yaml,config.yaml,enviar_resultados.py,verificar.py} "$remote:mid360/fast_lio_raspberry/"
scp -qr "$here/../fast_lio_local/adapter" "$remote:mid360/fast_lio_raspberry/ws/src/"
ssh "$remote" 'docker run --rm -v "$HOME/mid360/fast_lio_raspberry:/test" rover-companion:jazzy bash -c "set -e; cd /test/ws; CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build --merge-install --packages-select mid360_lio_adapter --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release"'
