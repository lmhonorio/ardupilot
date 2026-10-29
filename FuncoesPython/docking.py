from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('udp:0.0.0.0:14552')
master.wait_heartbeat()

mode_map = master.mode_mapping()
print("Modos disponíveis:", mode_map)

# if 'DOCK' in mode_map:
#     dock_id = mode_map['DOCK']

#     master.mav.command_long_send(
#         master.target_system,
#         master.target_component,
#         mavutil.mavlink.MAV_CMD_DO_SET_MODE,
#         0,
#         mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
#         dock_id,
#         0, 0, 0, 0, 0
#     )

#     time.sleep(1)

#     hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
#     if hb:
#         print("custom_mode atual:", hb.custom_mode)
# else:
#     print("DOCK não está disponível neste firmware.")

def mover(x, y):
    master.mav.set_position_target_local_ned_send(
        0,                                  # time_boot_ms
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b110111111000,                     # mascara: usar posição x,y
        x,                                  # x (frente)
        y,                                  # y (direita)
        0,                                  # z
        0, 0, 0,                            # vx, vy, vz
        0, 0, 0,                            # afx, afy, afz
        0,                                  # yaw
        0                                   # yaw_rate
    )

print("Forçando modo GUIDED para permitir os movimentos de posição local...")
master.set_mode('GUIDED')
time.sleep(2)

print("Avançando 10 metros para frente...")
mover(0.01, 0.0)
time.sleep(20)

print("Movendo 10 metros para a direita...")
mover(0.0, 0.01)
time.sleep(20)

print("Movendo 2 metros para a esquerda...")
mover(0.0, -0.01)
time.sleep(20)

print("Movimentos concluídos.")