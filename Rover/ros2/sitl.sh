#!/usr/bin/env bash
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$root/setup.bash"
params="$root/../nroverobst.param"
instance=0
args=()
while (($#)); do
  case "$1" in
    --add-param-file) params="$2"; shift 2;;
    --add-param-file=*) params="${1#*=}"; shift;;
    --foreground) shift;;
    --instance|-I) instance="$2"; shift 2;;
    --without-avoidance) params="$root/../nrover_sem_desvio.param"; shift;;
    --avoidance) params="$root/../nroverobst.param"; shift;;
    --help|-h) echo 'Uso: ./sim_1_gzrover.sh --foreground [--add-param-file ARQUIVO] [--no-rebuild] [--instance 0..9]'; exit 0;;
    *) args+=("$1"); shift;;
  esac
done
[[ "$instance" =~ ^[0-9]$ ]] || { echo 'Instancia deve estar entre 0 e 9.' >&2; exit 2; }
tcp=$((5760+10*instance)); sysid=$((instance+1))
if ss -H -ltn "sport = :$tcp" | grep -q .; then echo "Ja existe SITL na porta $tcp." >&2; exit 1; fi
run="$root/run/sitl"
((instance == 0)) || run="$root/run/sitl_$instance"
"$root/venv/bin/python" "$root/../python/simulation/ros2/prepare_sitl.py" --add-param-file "$params" --run-dir "$run" --sysid "$sysid"
# JSON is the Harmonic plugin protocol. gazebo-rover is the old Classic protocol.
exec "$root/venv/bin/python" "$root/../../Tools/autotest/sim_vehicle.py" -v Rover -f rover-skid \
  --model JSON --sysid="$sysid" -I "$instance" -L SEParnaiba --use-dir "$run" \
  --add-param-file "$run/defaults.parm" --no-extra-ports \
  --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:$((14555+10*instance)) --out=udp:127.0.0.1:$((14556+10*instance)) \
  --out=udp:127.0.0.1:$((14557+10*instance)) --out=udp:127.0.0.1:$((14551+10*instance)) \
  --out=udp:192.168.0.131:14550 --out=udp:192.168.0.131:14551 \
  --mavproxy-args='--streamrate=10 --cmd="set mavfwd true; rc 1 1500; rc 3 1500"' "${args[@]}"
