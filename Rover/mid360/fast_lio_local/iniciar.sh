#!/usr/bin/env bash
set -eo pipefail
here=$(cd -- "$(dirname -- "$0")" && pwd)
source "$here/../../ros2/setup.bash"
[[ -x "$here/ws/install/lib/fast_lio/fastlio_mapping" ]] || { echo 'Compile primeiro com ./mid360/fast_lio_local/build.sh' >&2; exit 1; }
source "$here/ws/install/local_setup.bash"
unset FASTRTPS_DEFAULT_PROFILES_FILE
export ROS_DOMAIN_ID=22 ROS_LOCALHOST_ONLY=1 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROVER_USE_SIM_TIME=false OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2
remote=${ROVER_REMOTE:-rover@192.168.0.167}
[[ "$remote" =~ ^[a-zA-Z0-9_.-]+@[a-zA-Z0-9_.-]+$ ]] || exit 2
ssh -o BatchMode=yes "$remote" 'set -e
  mkdir -p ~/mid360/fast_lio_local
  test "$(docker inspect -f "{{.State.Running}}" rover-mid360)" = true || {
    echo "Inicie o driver fisico com ./mid360/control.sh up" >&2; exit 1;
  }
  if ! ip -4 -o addr show dev eth0 | grep -Fq "192.168.1.50/24"; then
    echo "Restaurando o endereco temporario de aquisicao do MID360..."
    docker run --rm --network host --cap-add NET_ADMIN -v "$HOME/mid360:/mid360" \
      rover-companion:jazzy python /mid360/rede.py up
    docker restart rover-mid360 >/dev/null
  fi
  ping -I 192.168.1.50 -c 1 -W 2 192.168.1.161 >/dev/null || {
    echo "MID360 192.168.1.161 sem resposta; confira alimentacao e cabo antes do teste." >&2; exit 1;
  }'
scp -q "$here/enviar_teste.py" "$remote:mid360/fast_lio_local/"
exec python "$here/executar.py" "$@"
