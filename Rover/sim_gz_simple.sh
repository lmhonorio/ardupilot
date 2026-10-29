#!/usr/bin/env bash
set -euo pipefail

ROVER_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ASSETS="$ROVER_DIR/gazebo_simple"
GAZEBO_DIR="${ARDUPILOT_GAZEBO_DIR:-/home/grin/ardupilot_gazebo}"
export GAZEBO_MODEL_PATH="$ASSETS/models:$GAZEBO_DIR/models:$GAZEBO_DIR/models_gazebo${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
export GAZEBO_PLUGIN_PATH="$GAZEBO_DIR/build${GAZEBO_PLUGIN_PATH:+:$GAZEBO_PLUGIN_PATH}"
# Todos os recursos deste mundo sao locais.
export GAZEBO_MODEL_DATABASE_URI=""

setup_python38() {
  ROVER_PYTHON38="$("${PYTHON38:-python3.8}" -c 'import sys; assert sys.version_info[:2] == (3, 8), "Python 3.8 necessario"; print(sys.executable)')"
  export ROVER_PYTHON38
  export ROVER_MAVPROXY_SCRIPT
  ROVER_MAVPROXY_SCRIPT="$("$ROVER_PYTHON38" -c 'import pathlib, MAVProxy; print(pathlib.Path(MAVProxy.__path__[0]) / "mavproxy.py")')"
  # sim_vehicle chama mavproxy.py e o build pode chamar python/python3.
  mkdir -p "$ASSETS/run/bin"
  for command_name in python python3; do
    printf '#!/usr/bin/env bash\nexec "$ROVER_PYTHON38" "$@"\n' > "$ASSETS/run/bin/$command_name"
    chmod +x "$ASSETS/run/bin/$command_name"
  done
  printf '#!/usr/bin/env bash\nexec "$ROVER_PYTHON38" "$ROVER_MAVPROXY_SCRIPT" "$@"\n' > "$ASSETS/run/bin/mavproxy.py"
  chmod +x "$ASSETS/run/bin/mavproxy.py"
  export PATH="$ASSETS/run/bin:$PATH"
  echo "Python: $ROVER_PYTHON38" >&2
}

case "${1:-help}" in
  gazebo|server)
    gazebo_mode="$1"
    shift
    [[ -f "$GAZEBO_DIR/build/libArduPilotPlugin.so" ]] || {
      echo "Plugin ausente: $GAZEBO_DIR/build/libArduPilotPlugin.so" >&2; exit 1;
    }
    [[ -f "$GAZEBO_DIR/models_gazebo/husky/model.sdf" ]] || {
      echo "Modelo husky ausente em $GAZEBO_DIR/models_gazebo" >&2; exit 1;
    }
    gazebo_command=gazebo
    [[ "$gazebo_mode" != server ]] || gazebo_command=gzserver
    exec "$gazebo_command" --verbose "$ASSETS/rover_simple.world" "$@"
    ;;
  sitl-lua)
    shift
    setup_python38
    exec bash "$ASSETS/start_lua_sitl.sh" "$@"
    ;;
  sitl)
    shift
    setup_python38
    # Estado e logs isolados dos outros cenarios; caminhos independem do cwd.
    mkdir -p "$ASSETS/run"
    cd "$ASSETS/run"
    exec "$ROVER_PYTHON38" "$ROVER_DIR/../Tools/autotest/sim_vehicle.py" \
      -v Rover -f gazebo-rover --model gazebo-rover -I 0 --sysid 1 --no-extra-ports \
      -L "${LOCATION:-SEParnaiba}" -S 1 \
      --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14570 \
      --out=udp:127.0.0.1:14551 --out=udp:127.0.0.1:14555 \
      --out=udp:127.0.0.1:14552 --out=udp:127.0.0.1:14560 \
      --out=udp:192.168.0.131:14550 --out=udp:192.168.0.131:14551 \
      --out=udp:192.168.0.136:14551 --out=udp:192.168.0.170:14570 \
      --add-param-file "$ASSETS/rover_simple.parm" "$@"
    ;;
  sensors)
    shift
    setup_python38
    binary="$ASSETS/run/sensor_bridge"
    if [[ ! -x "$binary" || "$ASSETS/sensor_bridge.cc" -nt "$binary" ]]; then
      echo "Compilando leitor nativo do Gazebo..." >&2
      read -r -a gazebo_flags <<< "$(pkg-config --cflags --libs gazebo)"
      g++ -std=c++17 -O2 "$ASSETS/sensor_bridge.cc" -o "$binary" "${gazebo_flags[@]}"
    fi
    exec "$ROVER_PYTHON38" "$ROVER_DIR/python/simulation/read_sensors.py" --bridge "$binary" "$@"
    ;;
  check)
    shift
    exec gz sdf -k "$ASSETS/rover_simple.world" "$@"
    ;;
  help|--help|-h)
    echo "Uso: bash $0 {gazebo|server|sitl|sitl-lua|sensors|check} [opcoes adicionais]"
    echo "Abra gazebo e sitl em dois terminais. SITL opcional: --console --map."
    echo "sitl-lua: binario ArduRover direto com alocacao Lua, sem sim_vehicle."
    ;;
  *) echo "Modo desconhecido: $1" >&2; exit 2 ;;
esac
