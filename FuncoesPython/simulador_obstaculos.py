import time
import math
import socket
from pymavlink import mavutil

# ==========================================
# CONFIGURAÇÕES
# ==========================================
# Informe a porta em que o simulador/SITL ou placa física está escutando.
# Se for SITL local, geralmente tcp:127.0.0.1:5760 ou udp:127.0.0.1:14550
CONNECTION_STRING = 'udp:127.0.0.1:14552' 

# Lista de obstáculos virtuais (Latitude, Longitude)
# ATENÇÃO: Substitua por coordenadas extremamente próximas de onde o seu Rover vai estar!
OBSTACLES = [
    # Exemplo: coloque coordenadas capturadas no Mission Planner
    (-3.1231321, -41.7646546),
    (-3.1231212, -41.7644028)
]

# Configurações do "Sensor de Distância" (Lidar frontal simulado)
MAX_DISTANCE_M = 10.0      # Distância máxima que o sensor enxerga (metros)
MIN_DISTANCE_M = 0.2       # Distância mínima do sensor (metros)
FOV_DEGREES = 30.0         # Campo de visão (cone de +- 15 graus na frente do rover)

# Radar 2D (OBSTACLE_DISTANCE): array de distâncias ao redor do veículo
N_BINS = 72                          # Número de setores (ex.: 72 = 5° cada)
INCREMENT_DEG = 360.0 / N_BINS       # Largura angular de cada bin em graus
ANGLE_OFFSET_DEG = 0.0               # 0 = índice 0 é a frente do Rover (body frame)
INVALID_DISTANCE = 65535              # UINT16_MAX = desconhecido/não usado
# ==========================================


def get_distance_and_bearing(lat1, lon1, lat2, lon2):
    """
    Calcula a distância (em metros) e o azimute (rumo verdadeiro em graus)
    entre dois pontos geográficos usando a fórmula de Haversine.
    """
    R = 6371000.0 # Raio médio da Terra em metros
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    # Distância (Haversine)
    a = math.sin(delta_phi / 2.0)**2 + \
        math.cos(phi1) * math.cos(phi2) * \
        math.sin(delta_lambda / 2.0)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c

    # Azimute (Bearing)
    y = math.sin(delta_lambda) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - \
        math.sin(phi1) * math.cos(phi2) * math.cos(delta_lambda)
    bearing = math.degrees(math.atan2(y, x))
    bearing = (bearing + 360) % 360

    return distance, bearing


def main():
    print(f"Conectando ao ArduPilot em: {CONNECTION_STRING} ...")
    master = mavutil.mavlink_connection(CONNECTION_STRING, source_system=1, source_component=196)
    master.wait_heartbeat()
    print("Conectado! Solicitando telemetria de posição (GLOBAL_POSITION_INT)...")

    # Socket extra para injetar mensagens direto no QGC e no monitor_missao
    sock_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Solicita stream de dados de posição a ~10 Hz
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10, 1
    )

    start_time = time.time()
    last_print = 0

    while True:
        # Puxa mensagens da fila, procura especificamente por GLOBAL_POSITION_INT
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1.0)
        
        if msg:
            # Converte coordenadas do MAVLink (inteiro) para graus (float)
            veh_lat = msg.lat / 1e7
            veh_lon = msg.lon / 1e7
            # Converte centigraus para graus
            veh_hdg = msg.hdg / 100.0 if msg.hdg != 65535 else 0.0

            min_distance_cm = int(MIN_DISTANCE_M * 100)
            max_distance_cm = int(MAX_DISTANCE_M * 100)
            no_obstacle_cm = max_distance_cm + 1

            # Array de distâncias do radar 2D: cada índice = um setor angular (0 = frente)
            distances = [no_obstacle_cm] * N_BINS

            for obs_lat, obs_lon in OBSTACLES:
                dist, bearing = get_distance_and_bearing(veh_lat, veh_lon, obs_lat, obs_lon)

                # Ângulo do obstáculo em relação à frente do Rover (body frame, 0° = frente, 0..360)
                rel_bearing = (bearing - veh_hdg + 360) % 360

                # Índice do bin que corresponde a esse ângulo (0 = frente, incremento clockwise)
                idx = int(round(rel_bearing / INCREMENT_DEG)) % N_BINS

                if MIN_DISTANCE_M <= dist <= MAX_DISTANCE_M:
                    dist_cm = int(dist * 100)
                    dist_cm = min(max(dist_cm, min_distance_cm), max_distance_cm)
                    if distances[idx] == no_obstacle_cm or dist_cm < distances[idx]:
                        distances[idx] = dist_cm

            # Log resumido: obstáculo mais próximo na frente (bins centrais)
            half_fov_bins = int((FOV_DEGREES / 2.0) / INCREMENT_DEG)
            front_indices = list(range(N_BINS - half_fov_bins, N_BINS)) + list(range(0, half_fov_bins + 1))
            closest_front = min((distances[i] for i in front_indices if distances[i] != no_obstacle_cm), default=None)
            now = time.time()
            if now - last_print > 0.5:
                timestamp = time.strftime("[%H:%M:%S]")
                if closest_front is not None:
                    print(f"{timestamp} Obstáculo frontal mais próximo: {closest_front/100:.2f}m (radar 2D com {N_BINS} bins)")
                else:
                    print(f"{timestamp} Caminho limpo na frente (radar 2D enviando OBSTACLE_DISTANCE)")
                last_print = now

            time_usec = int((now - start_time) * 1e6)
            increment_uint8 = int(INCREMENT_DEG) if INCREMENT_DEG == int(INCREMENT_DEG) else 0

            # =======================================================
            # CONSTRÓI E ENVIA A MENSAGEM MAVLINK: OBSTACLE_DISTANCE
            # =======================================================
            msg_obstacle = master.mav.obstacle_distance_encode(
                time_usec,
                mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
                distances,
                increment_uint8,
                min_distance_cm,
                max_distance_cm,
                float(INCREMENT_DEG),
                ANGLE_OFFSET_DEG,
                mavutil.mavlink.MAV_FRAME_BODY_FRD
            )
            buf = msg_obstacle.pack(master.mav)

            # 1. Envia pro ArduPilot (via MAVProxy)
            master.write(buf)

            # 2. Injeta uma cópia no QGroundControl pra forçar ele a ver no Radar
            sock_out.sendto(buf, ('192.168.0.131', 14550))

            # 3. Injeta uma cópia no monitor_missao
            sock_out.sendto(buf, ('127.0.0.1', 14560))

        # Espera curta só pra manter um refresh de aprox 10-20Hz pra não inundar a conexão
        time.sleep(0.05)


if __name__ == '__main__':
    main()
