#!/bin/bash

# Terminal para Robô 1
gnome-terminal -- bash -c "sim_vehicle.py -L SEParnaiba -S 5 -v Rover --sysid 1  --instance 0 -f rover-skid --out=udp:127.0.0.1:14551 --out=udp:127.0.0.1:14571 --out=udp:192.168.0.131:14551  --sysid=1 --add-param-file nrover.param; exec bash"

# Terminal para Robô 2
gnome-terminal -- bash -c "sim_vehicle.py -L SEParnaiba -S 5 -v Rover --sysid 2 --instance 1 -f rover-skid --out=udp:127.0.0.1:14552 --out=udp:127.0.0.1:14572 --out=udp:192.168.0.131:14552 --sysid=2 --add-param-file nrover.param; exec bash"

# # Terminal para o MAVProxy
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
gnome-terminal -- bash -c "mavproxy.py \
  --master=udp:127.0.0.1:14571 \
  --master=udp:127.0.0.1:14572 \
  --out=udp:192.168.0.131:14550 \
  --console \
  --map \
  --cmd='map grid; map follow'; exec bash"



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


