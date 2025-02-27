#!/bin/bash

# Define o valor padrão da localização
DEFAULT_LOCATION="ARGO"

# Solicita ao usuário a localização, informando o valor padrão
read -p "Digite a localização (padrão: $DEFAULT_LOCATION): " LOCATION

# Se o usuário não inserir nada, usa o valor padrão
LOCATION=${LOCATION:-$DEFAULT_LOCATION}

# Executa o comando com a localização fornecida ou padrão
sim_vehicle.py -D --out=udpout:127.0.0.1:14550 --out=udpout:127.0.0.1:14551 --out=udpout:127.0.0.1:14552 -L "$LOCATION" -f rover-skid --model gazebo-rover --console --add-param-file gzrover.param

