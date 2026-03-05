#!/bin/bash

#!/usr/bin/env bash
# set -euo pipefail

# # ===================== ENTRADAS =====================
# DEFAULT_LOCATION="ARGO"
# read -p "Digite a localização (padrão: $DEFAULT_LOCATION): " LOCATION
# LOCATION=${LOCATION:-$DEFAULT_LOCATION}

# read -p "Quantos robôs (N)? [2]: " N
# N=${N:-2}

# # ===================== PARAMS =====================
# QGC_PORT=14550          # QGC recebe de todas as instâncias
# QGC_EXT=14551          # QGC recebe de todas as instâncias
# IP_OUT1="udp:192.168.0.131:"  # IP do computador rodando o QGC
# IP_OUT2="udp:192.168.0.131:"  # IP do computador rodando o QGC
# IP_OUT3="udp:127.0.0.1:"  # IP do computador rodando o QGC
# MAV_BASE=14555          # porta base p/ MAVROS (instância 0)
# STEP=10                 # incremento por instância: 14555, 14565, 14575...
# SYSID0=1                # SYSID da instância 0
# I0=0                    # índice base do --instance (-I)
# FRAME="rover-skid"
# MODEL="gazebo-rover"
# PARAM_FILE="gzrover.param"
# PAUSE=0.3

# # Caminho do ArduPilot (ajuste se diferente)
# ARDUPILOT_DIR="${HOME}/ardupilot"


#  --cmd='param set AVOID_ENABLE 0; param set PRX_TYPE 0' \



# conda deactivate
# hash -r
# cd /home/grin/ardupilot
# ./waf distclean
# cd ./Rover

# Terminal para Robô 1
gnome-terminal -- bash -c "sim_vehicle.py -L SEParnaiba -S 1 -v Rover -f rover-skid --sysid=1 --instance 0 --console \
                --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14570 --out=udp:127.0.0.1:14551 \
                --out=udp:192.168.0.131:14550 --out=udp:192.168.0.131:14551 \
                --out=udp:192.168.0.136:14551\
                --out=udp:192.168.0.170:14570\
                --add-param-file gzrover.param; exec bash"


# # 14550 - envio de dados para o qgroundcontrol
gnome-terminal -- bash -c "export MAP_SERVICE=GoogleSat; mavproxy.py \
  --master=udp:127.0.0.1:14551 \
  --out=udp:127.0.0.1:14552 \
  --out=udp:192.168.0.131:14550 --out=udp:127.0.0.1:14560 \
  --cmd='param set AVOID_ENABLE 1; param set PRX_TYPE 1; param set AVOID_MARGIN 2.0' 
  --map; exec bash"
