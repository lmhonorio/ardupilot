#!/usr/bin/env python3
import time
from pymavlink import mavutil

# Configuração da conexão
# Escuta na porta UDP 14550 (onde o sim_nrover_new.sh envia dados via broadcast/unicast)
# 0.0.0.0 permite escutar em todas as interfaces
CONNECTION_STRING = 'udp:0.0.0.0:14550'


def download_and_print_mission(master, target_system, target_component):
    print(f"--- Iniciando download da missão do Sistema {target_system} Componente {target_component} ---")
    
    # Solicita a lista de missões
    # Usa-se mission_request_list_send para pedir ao drone a lista de WPs
    master.mav.mission_request_list_send(target_system, target_component)
    
    # Aguarda MISSION_COUNT
    # Filtra por sistema para evitar conflito com outros drones
    msg = None
    start_time = time.time()
    while time.time() - start_time < 5:
        m = master.recv_match(type='MISSION_COUNT', blocking=True, timeout=1)
        if m and m.get_srcSystem() == target_system:
            msg = m
            break
            
    if not msg:
        print("Erro: Timeout aguardando MISSION_COUNT do sistema correto")
        return

    count = msg.count
    print(f"Missão contém {count} itens.")
    
    # Loop para baixar cada item da missão
    for i in range(count):
        # Solicita o item i
        # mission_request_int_send é preferível para obter coordenadas em inteiro (precisão), se suportado.
        # Fallback para mission_request se necessario, mas geralmente MISSION_ITEM_INT é padrão moderno.
        master.mav.mission_request_int_send(target_system, target_component, i)
        
        # Aguarda a resposta (MISSION_ITEM ou MISSION_ITEM_INT)
        msg_item = None
        item_start_time = time.time()
        while time.time() - item_start_time < 5:
            m = master.recv_match(type=['MISSION_ITEM_INT', 'MISSION_ITEM'], blocking=True, timeout=1)
            # Verifica se é do sistema correto e se é a sequência correta
            if m and m.get_srcSystem() == target_system and m.seq == i:
                msg_item = m
                break
         
        if msg_item:
            # Imprime os dados do WP
            # Ajusta formatação dependendo se é _INT ou float
            x = msg_item.x
            y = msg_item.y
            z = msg_item.z
            if msg_item.get_type() == 'MISSION_ITEM_INT':
                x = x / 1e7
                y = y / 1e7
            
            # Decodificação específica para Condition Yaw na lista
            extra_info = ""
            if msg_item.command == 115: # MAV_CMD_CONDITION_YAW
                rel = "Rel" if msg_item.param4 == 1 else "Abs"
                dir_str = "CW" if msg_item.param3 == 1 else ("CCW" if msg_item.param3 == -1 else "Shortest")
                extra_info = f" [YAW: {msg_item.param1}deg {msg_item.param2}d/s {dir_str} {rel}]"
            elif msg_item.command == 2000: # MAV_CMD_IMAGE_START_CAPTURE
                extra_info = f" [TAKE PHOTO: {int(msg_item.param3)} shots]"
                if msg_item.param2 > 0:
                    extra_info += f", Interval={msg_item.param2}s"
            elif msg_item.command == 205: # MAV_CMD_DO_MOUNT_CONTROL
                extra_info = f" [MOUNT: Pitch={msg_item.param1}, Roll={msg_item.param2}, Yaw={msg_item.param3}]"

            print(f"WP {msg_item.seq}: Cmd={msg_item.command} P1={msg_item.param1} P2={msg_item.param2} P3={msg_item.param3} P4(Yaw)={msg_item.param4} Lat={x} Lon={y} Alt={z}{extra_info}")
        else:
            print(f"Erro: Timeout recebendo WP {i}")
            break
            
    print("--- Fim da Missão ---\n")


def main():
    print(f"Conectando em {CONNECTION_STRING}...")
    # source_system=255 indica que somos um GCS (ou script monitor)
    master = mavutil.mavlink_connection(CONNECTION_STRING, source_system=200)
    
    print("Aguardando heartbeat...")
    master.wait_heartbeat()
    print("Conectado! Aguardando novas missões...")

    while True:
        # Lê mensagens
        msg = master.recv_match(blocking=True)
        
        if not msg:
            continue

        if msg.get_type() in ['MISSION_ITEM', 'MISSION_ITEM_INT']:
            # Identifica quem mandou
            src = msg.get_srcSystem()
            
            # Formata os dados
            cmd = msg.command
            p1 = msg.param1
            p2 = msg.param2
            p3 = msg.param3
            p4 = msg.param4
            seq = msg.seq
            
            # Tenta pegar coordenadas depndendo do tipo
            if msg.get_type() == 'MISSION_ITEM_INT':
                lat = msg.x / 1e7
                lon = msg.y / 1e7
                alt = msg.z
            else:
                lat = msg.x
                lon = msg.y
                alt = msg.z

            # Decodificação de Yaw (Condition Yaw)
            extra = ""
            if cmd == 115:
                dir_str = "CW" if p3 == 1 else ("CCW" if p3 == -1 else "Def")
                extra = f" [COND_YAW: {p1}deg]"

            # Identifica se é Upload (GCS->Drone) ou Download (Drone->GCS)
            # Geralmente Drone é 1, GCS é 255. Nosso script é 200.
            if src == 1 or src == 2: # Veículos
                # print(f"Drone {src} reportando WP {seq}: P4(Yaw)={p4}")
                pass # Já mostramos isso na função download
            else:
                # Provavelmente GCS Enviando (Upload)
                print(f"--> GCS (Sys{src}) Enviando WP {seq}: Cmd={cmd} P4(Yaw)={p4} {extra}")


        # Verifica se é um MISSION_ACK indicando sucesso (type 0 = MAV_MISSION_ACCEPTED)
        # Quando o QGroundControl termina de enviar a missão, o drone envia um MISSION_ACK para o QGC.
        # Nós interceptamos esse ACK para saber que a missão foi recebida com sucesso.
        if msg.get_type() == 'MISSION_ACK':
            if msg.type == 0: # MAV_MISSION_ACCEPTED
                print(f"\nNova missão aceita pelo veículo (SysID: {msg.get_srcSystem()})!")
                
                # Inicia o processo de download para visualizar o que foi enviado
                download_and_print_mission(master, msg.get_srcSystem(), msg.get_srcComponent())

        # Monitora Comandos de Longa Distância (COMMAND_LONG e COMMAND_INT)
        # É por aqui que o QQC envia comandos como "Set ROI", "Mount Control", etc.
        if msg.get_type() in ['COMMAND_LONG', 'COMMAND_INT']:
            cmd_id = msg.command
            
            # Mapeamento de alguns comandos relevantes
            # 195: MAV_CMD_DO_SET_ROI
            # 196: MAV_CMD_DO_SET_ROI_LOCATION
            # 197: MAV_CMD_DO_SET_ROI_NONE
            # 201: MAV_CMD_DO_SET_ROI (Legacy)
            # 204: MAV_CMD_DO_MOUNT_CONFIGURE
            # 205: MAV_CMD_DO_MOUNT_CONTROL
            # 115: MAV_CMD_CONDITION_YAW
            
            if cmd_id in [115, 195, 196, 197, 201, 204, 205]:
                print(f"\n--- Comando de Câmera/Yaw Detectado! (ID: {cmd_id}) ---")
                
                if cmd_id == 115: # CONDITION_YAW
                    # P1: Angle, P2: Speed, P3: Dir, P4: Rel
                    rel = "Rel" if msg.param4 == 1 else "Abs"
                    dir_str = "CW" if msg.param3 == 1 else ("CCW" if msg.param3 == -1 else "Shortest")
                    print("Tipo: CONDITION YAW (Girar Veículo)")
                    print(f"Angulo: {msg.param1} deg, Vel: {msg.param2} deg/s, Dir: {dir_str}, {rel}")

                elif cmd_id == 195 or cmd_id == 196: # SET_ROI / SET_ROI_LOCATION
                    # Parametros: 5=Lat, 6=Lon, 7=Alt
                    lat = msg.param5
                    lon = msg.param6
                    alt = msg.param7
                    
                    if msg.get_type() == 'COMMAND_INT':
                        lat = msg.x / 1e7
                        lon = msg.y / 1e7
                        alt = msg.z
                        
                    print("Tipo: SET ROI (Apontar Câmera)")
                    print(f"Alvo: Lat={lat}, Lon={lon}, Alt={alt}")

                elif cmd_id == 201: # DO_SET_ROI (Legacy)
                    # Param1: Mode, Param5/6/7: Lat/Lon/Alt
                    mode = msg.param1
                    lat = msg.param5
                    lon = msg.param6
                    alt = msg.param7
                    print(f"Tipo: SET ROI (ID 201) Modo={mode}")
                    print(f"Alvo: Lat={lat}, Lon={lon}, Alt={alt}")
                    
                elif cmd_id == 197: # SET_ROI_NONE
                    print("Tipo: Cancelar ROI (Resetar Câmera)")
                    
                elif cmd_id == 205: # DO_MOUNT_CONTROL
                    # Param1: Pitch/Tilt, Param2: Roll, Param3: Yaw/Pan
                    print("Tipo: Controle de Mount (Gimbal)")
                    print(f"Pitch: {msg.param1}")
                    print(f"Roll:  {msg.param2}")
                    print(f"Yaw:   {msg.param3}")
                
        # Monitora alteração de parâmetros individuais (PARAM_SET/PARAM_VALUE)
        # Caso o usuário mude um parâmetro na aba Parameters do QGC
        if msg.get_type() == 'PARAM_VALUE':
            # Filtra apenas parâmetros que podem ser interessantes (opcional)
            # ou imprime todos. Aqui filtramos por string 'MNT' (Mount) ou 'CAM' (Camera) ou 'YAW'
            param_id = msg.param_id
            if any(x in param_id for x in ['MNT', 'CAM', 'YAW', 'ROI']):
                print(f"Parâmetro Alterado/Recebido: {param_id} = {msg.param_value}")


if __name__ == '__main__':
    main()
