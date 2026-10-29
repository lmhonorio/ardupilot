# source /opt/ros/noetic/setup.bash
# source ~/catkin_ws/devel/setup.bash

# roslaunch rover_argo_gazebo multi_rover_argo.launch \
#   N:=1 \
#   world_name:=gravel_plane \
#   start_x:=0 start_y:=0 z:=0 \
#   mid360_enabled:=true \
#   insta360_enabled:=true \
#   samsung_s23_ultra_cam_enabled:=true \
#   hokuyo_enabled:=true









#!/usr/bin/env bash
set -euo pipefail
ROVER_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" != --foreground ]]; then
  exec gnome-terminal -- bash "$ROVER_DIR/sim_gz1rover.sh" --foreground "$@"
fi
shift
PY38="$("${PYTHON38:-python3.8}" -c 'import sys; assert sys.version_info[:2] == (3, 8); print(sys.executable)')"
RUN_DIR="$ROVER_DIR/gazebo_simple/run/ros_argo_lua"
"$PY38" "$ROVER_DIR/python/simulation/prepare_ros_argo_lua.py" "$ROVER_DIR" "$RUN_DIR"
if [[ "${1:-}" == --prepare-only ]]; then exit 0; fi
if ss -H -ltn 'sport = :5760' | grep -q .; then
  echo 'Ja existe um SITL na porta 5760. Encerre-o no terminal anterior antes de iniciar este perfil Lua.' >&2
  exit 1
fi
mkdir -p "$RUN_DIR/bin"
export ROVER_PYTHON38="$PY38"
export ROVER_MAVPROXY_SCRIPT
ROVER_MAVPROXY_SCRIPT="$("$PY38" -c 'import pathlib, MAVProxy; print(pathlib.Path(MAVProxy.__path__[0]) / "mavproxy.py")')"
for executable in python python3; do
  printf '#!/usr/bin/env bash\nexec "$ROVER_PYTHON38" "$@"\n' > "$RUN_DIR/bin/$executable"
  chmod +x "$RUN_DIR/bin/$executable"
done
printf '#!/usr/bin/env bash\nexec "$ROVER_PYTHON38" "$ROVER_MAVPROXY_SCRIPT" "$@"\n' > "$RUN_DIR/bin/mavproxy.py"
chmod +x "$RUN_DIR/bin/mavproxy.py"
export PATH="$RUN_DIR/bin:$PATH"
echo "Lua: $RUN_DIR/scripts; Python: $PY38"
echo "Use apenas apos encerrar o SITL anterior. O mundo ROS deve estar aberto."
exec "$PY38" "$ROVER_DIR/../Tools/autotest/sim_vehicle.py" -v Rover \
  -I 0 --sysid=1 -L ARGO -f rover-skid --model gazebo-rover \
  --use-dir "$RUN_DIR" --add-param-file "$RUN_DIR/defaults.parm" --no-extra-ports \
  --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14555 --out=udp:127.0.0.1:14556 \
  --out=udp:192.168.0.131:14550 --out=udp:192.168.0.131:14551 \
  --mavproxy-args='--cmd="set mavfwd true; rc 1 1500; rc 3 1500"' "$@"

# Terminal para Robô 2

# gnome-terminal -- bash -c "sim_vehicle.py -D \
#     f rover-skid -I 1 --sysid=2 -L SEParnaiba -f rover-skid --model gazebo-rover --console --add-param-file gzrover.param\
#   --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14561 \
#   --out=udp:192.168.0.131:14550 --out=udp:192.168.0.131:14561 ; exec bash"

#gnome-terminal -- bash -c "sim_vehicle.py -L SEParnaiba -S 5 -v Rover --sysid 2 --instance 1 -f rover-skid --out=udp:127.0.0.1:14552 --out=udp:127.0.0.1:14572 --out=udp:192.168.0.131:14552 --sysid=2 --add-param-file nrover.param; exec bash"

# # Terminal para o MAVProxy sim_vehicle.py -D 
#--out=udpout:127.0.0.1:14550 --out=udpout:127.0.0.1:14551 --out=udpout:127.0.0.1:14552 --out=udp:192.168.0.131:14551 
#--out=udp:192.168.0.131:14550 
#--sysid=1 -L SEParnaiba -f rover-skid -I 0 --model gazebo-rover --console --add-param-file gzrover.param




# gnome-terminal -- bash -c "mavproxy.py \
#   --master=udp:127.0.0.1:14551 \
#   --out=udp:127.0.0.1:14561 \
#   --out=udp:
#   --out=udp:192.168.0.131:14551 \
#   --source-system=250 \
#   --console \
#   --cmd='map grid; map follow'; exec bash"

# # # Terminal para o MAVProxy
# gnome-terminal -- bash -c "mavproxy.py \
#   --master=udp:127.0.0.1:14552 \
#   --out=udp:127.0.0.1:14562 \
#   --out=udp:192.168.0.131:14552 \
#   --out=udp:127.0.0.1:14572 \
#   --source-system=251 \
#   --console \
#   --cmd='map grid; map follow'; exec bash"


## 14550 - envio de dados para o qgroundcontrol
# gnome-terminal -- bash -c "mavproxy.py \
#   --master=udp:127.0.0.1:14571 \
#   --master=udp:127.0.0.1:14572 \
#   --out=udp:192.168.0.131:14550 \
#   --console \
#   --map \
#   --cmd='map grid; map follow'; exec bash"



# gnome-terminal -- bash -c "mavproxy.py \
#   --master=udp:127.0.0.1:14551 \
#   --out=udp:192.168.0.131:14551 \
#   --master=udp:127.0.0.1:14552 \
#   --out=udp:192.168.0.131:14552 \
#   --console \
#   --map \
#   --exec bash"

# # Robô 1 (porta interna: 14551)
# sim_vehicle.py -L SEParnaiba -S 5 --mcast -v Rover --sysid 1 --instance 0 -f rover-skid --out=udp:127.0.0.1:14551 --add-param-file nrover.param

# # Robô 2 (porta interna: 14552)
# sim_vehicle.py -L SEParnaiba -S 5 --mcast -v Rover --sysid 2 --instance 1 -f rover-skid --out=udp:127.0.0.1:14552 --add-param-file nrover.param

# mavproxy.py --master=udp:127.0.0.1:14551 \
#             --out=udp:192.168.0.131:14551 \
#             --master=udp:127.0.0.1:14552 \
#             --out=udp:192.168.0.131:14552 \
#             --console \
#             --map
# # Robô 1 (porta interna: 14551)
# sim_vehicle.py -L SEParnaiba -S 5 --mcast -v Rover --instance 0 -f rover-skid --out=udp:192.168.0.131:14550--console --map --add-param-file nrover.param

# # Robô 2 (porta interna: 14552)
# sim_vehicle.py -L SEParnaiba -S 5 --mcast -v Rover --instance 1 -f rover-skid --out=udp:192.168.0.131:14551 --console --map --add-param-file nrover.param


