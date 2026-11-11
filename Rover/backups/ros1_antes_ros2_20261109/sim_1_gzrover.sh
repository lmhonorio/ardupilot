#!/usr/bin/env bash
# Remove Conda python from PATH so Ardupilot scripts use the system/pyenv python
export PATH=$(echo $PATH | sed -e 's|/home/grin/anaconda3/bin:||g' -e 's|/home/grin/anaconda3/condabin:||g')
# Add pyenv explicitly just in case
export PATH="$HOME/.pyenv/shims:$PATH"

# sim_vehicle.py  -D -L SEParnaiba --mcast -f rover-skid --model gazebo-rover --console --map --add-param-file nroverobst.param
#   --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14555 --out=udp:127.0.0.1:14556 --out=udp:127.0.0.1:14557 \
#   --out=udp:192.168.0.131:14550 --out=udp:192.168.0.131:14551 \

# Usar o perfil anterior e a EEPROM do diretorio Rover/.
ROVER_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PY38="$HOME/.pyenv/versions/3.8.10/bin/python3.8"
PARAM_FILE="$ROVER_DIR/nroverobst.param"
FOREGROUND=0
while (($#)); do
    case "$1" in
        --add-param-file)
            if (($# < 2)); then echo 'Informe o arquivo apos --add-param-file.' >&2; exit 2; fi
            PARAM_FILE="$2"; shift 2 ;;
        --add-param-file=*) PARAM_FILE="${1#*=}"; shift ;;
        --foreground) FOREGROUND=1; shift ;;
        -h|--help)
            echo "Uso: $0 [--foreground] [--add-param-file ARQUIVO]"
            echo 'Padrao: nroverobst.param. --foreground executa o SITL neste terminal e nao inicia a ponte.'
            exit 0 ;;
        *) echo "Opcao desconhecida: $1" >&2; exit 2 ;;
    esac
done
if [[ ! -f "$PARAM_FILE" || ! -r "$PARAM_FILE" ]]; then
    echo "Arquivo de parametros nao encontrado ou sem leitura: $PARAM_FILE" >&2; exit 1
fi
PARAM_FILE="$(cd -- "$(dirname -- "$PARAM_FILE")" && pwd)/$(basename -- "$PARAM_FILE")"
if ss -H -ltn 'sport = :5760' | grep -q .; then
    echo 'SITL ja utiliza a porta 5760; encerre-o antes de iniciar.' >&2
    exit 1
fi

if (( FOREGROUND )); then
    cd "$ROVER_DIR" || exit 1
    echo "Parametros adicionais: $PARAM_FILE"
    echo 'A ponte de obstaculos deve ser iniciada separadamente, quando necessaria.'
    exec "$PY38" "$ROVER_DIR/../Tools/autotest/sim_vehicle.py" -D -L SEParnaiba -f rover-skid --model gazebo-rover \
      --sysid=1 --instance 0 --console \
      --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14555 --out=udp:127.0.0.1:14556 \
      --out=udp:127.0.0.1:14557 --out=udp:127.0.0.1:14551 \
      --out=udp:192.168.0.131:14550 --out=udp:192.168.0.131:14551 \
      --mavproxy-args='--cmd="set mavfwd true"' --add-param-file "$PARAM_FILE"
fi

# Terminal para Robô 1
gnome-terminal -- bash -c '
  rover_dir=$1; py38=$2; param_file=$3
  cd "$rover_dir" || exit 1
  "$py38" "$rover_dir/../Tools/autotest/sim_vehicle.py" -D -L SEParnaiba -f rover-skid --model gazebo-rover \
    --sysid=1 --instance 0 --console \
    --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14555 --out=udp:127.0.0.1:14556 \
    --out=udp:127.0.0.1:14557 --out=udp:127.0.0.1:14551 \
    --out=udp:192.168.0.131:14550 --out=udp:192.168.0.131:14551 \
    --mavproxy-args="--cmd=\"set mavfwd true\"" --add-param-file "$param_file"
  echo "SITL encerrado; pressione Enter para fechar."
  read -r
' _ "$ROVER_DIR" "$PY38" "$PARAM_FILE"

# A ponte de proximidade precisa rodar junto ao SITL; nenhuma alteracao de modo/armamento.
# O Gazebo e o ROS devem ser iniciados primeiro, com mid360_enabled:=true.
if [[ -f /opt/ros/noetic/setup.bash && -f "$HOME/catkin_ws/devel/setup.bash" ]]; then
    source /opt/ros/noetic/setup.bash
    source "$HOME/catkin_ws/devel/setup.bash"
    bridge_live=0; processor_live=0
    timeout 4 rosnode ping -c 1 /olhe_obstacle_mavlink_bridge 2>&1 | grep -q 'xmlrpc reply from' && bridge_live=1
    timeout 4 rosnode ping -c 1 /olhe_obstacle_processor 2>&1 | grep -q 'xmlrpc reply from' && processor_live=1
    if (( bridge_live && processor_live )); then
        echo 'Ponte de obstaculos ja esta em execucao.'
    elif (( bridge_live || processor_live )); then
        echo 'Somente um componente da percepcao responde; encerre-o antes de reiniciar a ponte.' >&2
    else
        lidar_ready=0
        for _ in {1..30}; do
            if timeout 3 rostopic list 2>/dev/null | grep -Fxq '/rover_argo_N1/Instance1/livox/lidar'; then
                lidar_ready=1
                break
            fi
            sleep 1
        done
        if (( lidar_ready )); then
            gnome-terminal --title='Percepcao de obstaculos MID360' -- bash -c '
                source /opt/ros/noetic/setup.bash
                source "$HOME/catkin_ws/devel/setup.bash"
                cd "$HOME/ardupilot/Rover" || exit 1
                echo "Iniciando percepcao; Ctrl+C encerra este modulo."
                /usr/bin/python3.8 python/obstacle_avoidance/manager.py
                echo "Percepcao encerrada. Pressione Enter para fechar."
                read -r
            '
        else
            echo 'MID360 nao encontrado. Inicie o roslaunch com mid360_enabled:=true e execute a ponte manualmente.' >&2
        fi
    fi
else
    echo 'Ambiente ROS nao encontrado; a ponte de obstaculos nao foi iniciada.' >&2
fi


# # 14550 - envio de dados para o qgroundcontrol
# gnome-terminal -- bash -c "export MAP_SERVICE=GoogleSat; mavproxy.py \
#   --master=udp:127.0.0.1:14551 \
#   --out=udp:127.0.0.1:14552 \
#   --out=udp:192.168.0.131:14550 --out=udp:127.0.0.1:14560 --out=udp:192.168.0.170:14570\
#   --cmd='param set AVOID_ENABLE 1; param set PRX_TYPE 1; param set AVOID_MARGIN 2.0' \
#   --exec bash"
