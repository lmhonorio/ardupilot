#!/usr/bin/env bash
set -euo pipefail

ROVER_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
LAUNCHER="$ROVER_DIR/sim_gz_simple.sh"
ASSETS="$ROVER_DIR/gazebo_simple"
headless=false
sitl_mode=sitl
sitl_args=()
stop_requested=false
while (($#)); do
  case "$1" in
    --headless) headless=true ;;
    --lua) sitl_mode=sitl-lua ;;
    --no-rebuild|-N) sitl_args+=(--no-rebuild) ;;
    --stop) stop_requested=true ;;
    -h|--help)
      echo "Uso: bash $0 [--lua] [--headless] [--no-rebuild] [--stop]"
      echo "Inicia Gazebo, SITL em SEParnaiba e camera/LiDAR com Python 3.8."
      echo "Ctrl+C encerra os processos desta execucao."
      echo "--stop encerra a simulacao iniciada por este lancador em outro terminal."
      echo "--lua inicia o binario ArduRover e MAVProxy diretamente, com sua alocacao Lua."
      exit 0 ;;
    *) echo "Opcao desconhecida: $1" >&2; exit 2 ;;
  esac
  shift
done

pid_file="$ASSETS/run/stack.pid"
if "$stop_requested"; then
  if [[ -r "$pid_file" ]]; then
    read -r owner_pid owner_start < "$pid_file"
    if [[ "$owner_pid" =~ ^[0-9]+$ && -r "/proc/$owner_pid/stat" ]] &&
       [[ "$(awk '{print $22}' "/proc/$owner_pid/stat")" == "$owner_start" ]]; then
      kill -TERM "$owner_pid"
      if flock -w 10 "$ASSETS/run/stack.lock" true; then
        echo "Simulacao encerrada."
      else
        echo "O encerramento ainda nao concluiu; verifique o terminal do lancador." >&2
        exit 1
      fi
      exit 0
    fi
  fi
  echo "Nao ha simulacao ativa registrada por este lancador."
  exit 0
fi

mkdir -p "$ASSETS/run"
exec 8>"$ASSETS/run/stack.lock"
flock -n 8 || {
  echo "Este cenario ja esta em execucao. Para encerrar: bash $0 --stop" >&2
  exit 1
}
export LOCATION=SEParnaiba
export MAP_SERVICE=GoogleSat
export SITL_RITW_TERMINAL="$ASSETS/run_terminal.sh"
export PYTHONUNBUFFERED=1

# sim_vehicle sem psutil pode encerrar outras instancias de firmware pelo nome.
if pgrep -x 'ardurover|arducopter|arduplane|ardusub|antennatracker' >/dev/null; then
  echo "Ja existe um SITL em execucao. Encerre-o antes de iniciar este cenario." >&2
  exit 1
fi
"${PYTHON38:-python3.8}" - <<'PY'
import socket
for kind, port in [(socket.SOCK_DGRAM, 9002), (socket.SOCK_DGRAM, 9003),
                   (socket.SOCK_STREAM, 5760)]:
    with socket.socket(socket.AF_INET, kind) as sock:
        if kind == socket.SOCK_STREAM:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(('0.0.0.0', port))
        except OSError as exc:
            raise SystemExit('Porta %d ocupada: %s' % (port, exc))
PY

pids=()
cleanup() {
  trap - EXIT INT TERM
  echo "Encerrando a simulacao..."
  # Gazebo creates process groups for gzserver/gzclient inside its session.
  # Killing only the original process group can leave these children alive.
  for pid in "${pids[@]}"; do pkill -TERM -s "$pid" 2>/dev/null || true; done
  sleep 2
  for pid in "${pids[@]}"; do pkill -KILL -s "$pid" 2>/dev/null || true; done
  for pid in "${pids[@]}"; do wait "$pid" 2>/dev/null || true; done
  rm -f -- "$pid_file"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
printf '%s %s\n' "$$" "$(awk '{print $22}' "/proc/$$/stat")" > "$pid_file"

gazebo_mode=gazebo
sensor_args=(--timeout 0)
if "$headless"; then
  gazebo_mode=server
  sensor_args+=(--no-gui)
elif "${PYTHON38:-python3.8}" -c 'import wx' 2>/dev/null; then
  sitl_args+=(--console --map)
else
  echo "wxPython ausente no Python 3.8: MAVProxy usara o prompt deste terminal."
fi

# Only the launcher owns the flock descriptor; children must not inherit it.
setsid bash "$LAUNCHER" "$gazebo_mode" 8>&- >"$ASSETS/run/gazebo.log" 2>&1 &
gazebo_pid=$!
pids+=("$gazebo_pid")
echo "Aguardando mundo SEParnaiba... (log: $ASSETS/run/gazebo.log)"
ready=false
for ((attempt=0; attempt<60; attempt++)); do
  if ! kill -0 "$gazebo_pid" 2>/dev/null; then
    tail -30 "$ASSETS/run/gazebo.log" >&2
    exit 1
  fi
  # Consume the complete listing so pipefail does not see a producer SIGPIPE.
  if timeout 2 gz topic -l 2>/dev/null | grep -F '/gazebo/rover_simple/rover_mid360/mid360_link/mid360/scan' >/dev/null; then
    ready=true
    break
  fi
  sleep 1
done
"$ready" || { echo "O mundo nao ficou pronto; veja gazebo.log." >&2; exit 1; }

setsid bash "$LAUNCHER" sensors "${sensor_args[@]}" 8>&- >"$ASSETS/run/sensors.log" 2>&1 &
pids+=("$!")
echo "Camera/LiDAR: $ASSETS/run/sensors/ (log: $ASSETS/run/sensors.log)"
echo "SITL: SYSID 1, SEParnaiba; mesmas saidas UDP de sim_1rover_new.sh."
echo "QGroundControl local: UDP 127.0.0.1:14550. Aguarde o heartbeat do rover."
echo "Log do SITL/MAVProxy: $ASSETS/run/sitl.log"
if [[ "$sitl_mode" == sitl-lua ]]; then
  echo "Alocacao Lua: binario direto (sem sim_vehicle), estado em $ASSETS/run/lua/."
fi
# Explicit stdin keeps the interactive MAVProxy prompt available.
setsid bash "$LAUNCHER" "$sitl_mode" "${sitl_args[@]}" 8>&- <&0 \
  > >(tee "$ASSETS/run/sitl.log" 8>&-) 2>&1 &
sitl_pid=$!
pids+=("$sitl_pid")
if wait "$sitl_pid"; then
  exit 0
else
  sitl_exit=$?
  echo "SITL/MAVProxy encerrou (codigo $sitl_exit); veja $ASSETS/run/sitl.log." >&2
  exit "$sitl_exit"
fi
