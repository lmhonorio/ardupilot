#!/bin/bash


# Executa o comando com a localização fornecida ou padrão
sim_vehicle.py -D --out=udpout:127.0.0.1:14550 --out=udpout:127.0.0.1:14551 --out=udpout:127.0.0.1:14552 --out=udp:192.168.0.131:14551 --out=udp:192.168.0.131:14550 --sysid=1 -L SEParnaiba -f rover-skid -I 0 --model gazebo-rover --console --add-param-file gzrover.param

