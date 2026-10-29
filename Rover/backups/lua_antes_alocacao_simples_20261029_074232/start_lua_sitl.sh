#!/usr/bin/env bash
# Called by sim_gz_simple.sh sitl-lua after Python 3.8 setup.
set -euo pipefail
ASSETS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROVER_DIR="$(dirname -- "$ASSETS")"
ARDUPILOT_DIR="$(dirname -- "$ROVER_DIR")"
RUN_DIR="$ASSETS/run/lua"
: "${ROVER_PYTHON38:?Inicie pelo sim_gz_simple.sh sitl-lua}"
: "${ROVER_MAVPROXY_SCRIPT:?MAVProxy nao configurado}"
no_rebuild=false
mavproxy_args=()
for arg in "$@"; do
  case "$arg" in
    --no-rebuild|-N) no_rebuild=true ;;
    --console|--map) mavproxy_args+=("$arg") ;;
    *) echo "Opcao nao suportada em sitl-lua: $arg" >&2; exit 2 ;;
  esac
done

if ! "$no_rebuild"; then
  (
    cd "$ARDUPILOT_DIR"
    "$ROVER_PYTHON38" ./waf configure --board sitl
    "$ROVER_PYTHON38" ./waf rover
  )
fi
binary="$ARDUPILOT_DIR/build/sitl/bin/ardurover"
[[ -x "$binary" ]] || { echo "Compile o firmware antes de usar --no-rebuild: $binary" >&2; exit 1; }
mkdir -p "$RUN_DIR/scripts/modules"
# Copy only the deployed scripts, never backups or unit-test fixtures.
cp -- "$ROVER_DIR/scripts/system-start.lua" "$ROVER_DIR/scripts/rover-controlallocation.lua" "$ROVER_DIR/scripts/lidar_odometry.lua" "$RUN_DIR/scripts/"
cp -- "$ROVER_DIR/scripts/modules/functions.lua" "$ROVER_DIR/scripts/modules/pid.lua" "$RUN_DIR/scripts/modules/"

# Generate a simulation-only initializer from the selected parameter files.
# It waits for system-start.lua, configures while disarmed, then enables the
# allocator. This also applies the selected profile over saved SITL values.
"$ROVER_PYTHON38" - "$ASSETS" "$RUN_DIR" <<'PY'
import math
from pathlib import Path
import sys
assets, run = map(Path, sys.argv[1:])
values = {}
for name in ('rover_simple.parm', 'rover_lua.parm'):
    for line in (assets/name).read_text().splitlines():
        fields = line.split('#', 1)[0].split()
        if not fields:
            continue
        key, value = fields
        number = float(value)
        if not key.replace('_', '').isalnum() or not math.isfinite(number):
            raise SystemExit('Parametro invalido: '+line)
        values[key] = number
entries = '\n'.join('  {"%s", %.10g},' % pair for pair in values.items())
script = '''-- Generated only for Gazebo; do not deploy to the physical rover.
local selected = {
%s
}
local function configure()
  if arming:is_armed() then return configure, 200 end
  if param:get('BATT_SOC_ALLOC') == nil or param:get('BATT_SOC_WPYAW') == nil then
    return configure, 200
  end
  assert(param:set('BATT_SOC_ALLOC', 0), 'could not pause allocation')
  for _, entry in ipairs(selected) do
    assert(param:set(entry[1], entry[2]), 'could not set '..entry[1])
  end
  assert(param:set('BATT_SOC_WPYAW', 0), 'could not disable waypoint yaw')
  assert(param:set('BATT_SOC_ALLOC', 1), 'could not enable allocation')
  gcs:send_text(6, 'Gazebo: Lua allocation enabled; selected parameters applied')
end
return configure, 1500
''' % entries
(run/'scripts/00-gazebo-config.lua').write_text(script)
PY

firmware_pid=""
cleanup() {
  trap - EXIT INT TERM
  if [[ -n "$firmware_pid" ]] && kill -0 "$firmware_pid" 2>/dev/null; then
    kill -TERM "$firmware_pid" 2>/dev/null || true
    wait "$firmware_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
cd "$RUN_DIR"
echo "ArduRover direto: $binary"
echo "Scripts Lua: $RUN_DIR/scripts/"
echo "Log do firmware: $RUN_DIR/ardurover.log"
"$binary" -S --model gazebo-rover --speedup 1 --sysid 1 --slave 0 -I 0 \
  --sim-address 127.0.0.1 --home=-3.12315,-41.764495,77,0 \
  --defaults "$ARDUPILOT_DIR/Tools/autotest/default_params/rover.parm,$ARDUPILOT_DIR/Tools/autotest/default_params/rover-skid.parm,$ASSETS/rover_simple.parm,$ASSETS/rover_lua.parm" \
  > "$RUN_DIR/ardurover.log" 2>&1 &
firmware_pid=$!
# Check readiness without consuming a MAVLink TCP client connection.
ready=false
for ((attempt=0; attempt<100; attempt++)); do
  if ! kill -0 "$firmware_pid" 2>/dev/null; then
    tail -30 "$RUN_DIR/ardurover.log" >&2
    exit 1
  fi
  if grep -Fq 'bind port 5760' "$RUN_DIR/ardurover.log"; then
    ready=true
    break
  fi
  sleep 0.1
done
"$ready" || { echo "SITL nao abriu TCP 5760; veja ardurover.log." >&2; exit 1; }
"$ROVER_PYTHON38" "$ROVER_MAVPROXY_SCRIPT" \
  --master=tcp:127.0.0.1:5760 --sitl=127.0.0.1:5501 \
  --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14570 \
  --out=udp:127.0.0.1:14551 --out=udp:127.0.0.1:14555 \
  --out=udp:127.0.0.1:14552 --out=udp:127.0.0.1:14560 \
  --out=udp:192.168.0.131:14550 --out=udp:192.168.0.131:14551 \
  --out=udp:192.168.0.136:14551 --out=udp:192.168.0.170:14570 \
  --cmd='set mavfwd true; rc 1 1500; rc 3 1500' "${mavproxy_args[@]}"
