# Remove Conda python from PATH so Ardupilot scripts use the system/pyenv python
export PATH=$(echo $PATH | sed -e 's|/home/grin/anaconda3/bin:||g' -e 's|/home/grin/anaconda3/condabin:||g')
# Add pyenv explicitly just in case
export PATH="$HOME/.pyenv/shims:$PATH"

# sim_vehicle.py  -D -L SEParnaiba --mcast -f rover-skid --model gazebo-rover --console --map --add-param-file nrover.param


# Terminal para Robô 1
gnome-terminal -- bash -c "sim_vehicle.py -D -L SEParnaiba -f rover-skid --model gazebo-rover --sysid=1 --instance 0 --console \
                --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14570 --out=udp:127.0.0.1:14556 --out=udp:127.0.0.1:14551 \
                --out=udp:192.168.0.131:14550 --out=udp:192.168.0.131:14551 \
                --out=udp:192.168.0.136:14551\
                --out=udp:192.168.0.170:14570\
                --add-param-file nrover.param; exec bash"


# # 14550 - envio de dados para o qgroundcontrol
# gnome-terminal -- bash -c "export MAP_SERVICE=GoogleSat; mavproxy.py \
#   --master=udp:127.0.0.1:14551 \
#   --out=udp:127.0.0.1:14552 \
#   --out=udp:192.168.0.131:14550 --out=udp:127.0.0.1:14560 --out=udp:192.168.0.170:14570\
#   --cmd='param set AVOID_ENABLE 1; param set PRX_TYPE 1; param set AVOID_MARGIN 2.0' \
#   --exec bash"