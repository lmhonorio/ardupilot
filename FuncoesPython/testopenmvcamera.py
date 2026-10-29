import cv2
import cv2.aruco as aruco
import numpy as np
import time
import math
from pymavlink import mavutil

# =======================================================
# Este script atua como uma Companion Computer (OpenMV simulada).
# Ele abre sua webcam, identifica um marcador ArUco
# e envia uma mensagem LANDING_TARGET para o Ardupilot SITL.
# =======================================================

print("========================================")
print(" Simulador OpenMV - Alvo de Atracagem")
print("========================================")

# 1. Configura a conexão MAVLink
# Certifique-se de que a porta bate com a que você utiliza no simulador.
# Em geral, o simulador abre 14550 ou 14552 para APIs externas.
master = mavutil.mavlink_connection('udp:0.0.0.0:14552')
print("Aguardando heartbeat do simulador (Isso garante que o Rover está detectado)...")
master.wait_heartbeat()
print("Conectado ao Veículo na porta 14552!")

# 2. Configurações da Câmera Simulada (Ex: Field of View)
# Estes valores dependem da sua webcam. Como é simulação, 60º e 45º funcionam como base.
H_FOV = 60.0  # FOV Horizontal em graus
V_FOV = 45.0  # FOV Vertical em graus

# 3. Inicializa leitura do ArUco e Webcam
cap = cv2.VideoCapture(0)

# Usa o dicionário DICT_4X4_50. 
# Você pode procurar imagens "ArUco Marker 4x4" no seu celular no Google Imagens e mostrar para a câmera.
ar_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

# Compatibilidade de versões do OpenCV 4.x
if hasattr(aruco, "DetectorParameters_create"):
    parameters = aruco.DetectorParameters_create()
else:
    parameters = aruco.DetectorParameters()

# Detector para OpenCV 4.7+
detector = None
if hasattr(aruco, "ArucoDetector"):
    detector = aruco.ArucoDetector(ar_dict, parameters)

print("Verifique se a Webcam iniciou.")
print("Mostre um Marcador ArUco 4x4 para a tela.")
print("Pressione 'q' na janela de imagem para encerrar.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Falha ao capturar a câmera. A webcam está disponível?")
        time.sleep(1)
        continue

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.imshow("Gray", gray)
    height, width, _ = frame.shape

    # Detecta os marcadores na imagem
    if detector is not None:
        corners, ids, rejectedImgPoints = detector.detectMarkers(gray)
    else:
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, ar_dict, parameters=parameters)

    # Se um ou mais marcadores foram encontrados
    if ids is not None and len(ids) > 0:
        # Vamos guiar o Rover para o primeiro marcador (index 0)
        c = corners[0][0]
        
        # Acha o centro (X e Y médios) do quadrado do ArUco na imagem de pixel
        center_x = int((c[0][0] + c[1][0] + c[2][0] + c[3][0]) / 4)
        center_y = int((c[0][1] + c[1][1] + c[2][1] + c[3][1]) / 4)
        
        # Desenha uma bolinha verde no centro detectado
        cv2.circle(frame, (center_x, center_y), 6, (0, 255, 0), -1)
        
        # Calcula do centro da imagem para o marcador (em px)
        # Assumindo que o centro da câmera mira reto pra frente.
        dx = center_x - (width / 2)
        dy = center_y - (height / 2)
        
        # Converte pixels para radianos multiplicando pela proporção de visualização do FOV
        angle_x = (dx / width) * math.radians(H_FOV)
        angle_y = (dy / height) * math.radians(V_FOV)
        
        # -------------------------------------------------------------------
        # Calculando a distância real baseada no tamanho do marcador
        # -------------------------------------------------------------------
        # AQUI DEFINE O TAMANHO REAL DO SEU ARUCO DA TELA OU IMPRESSO
        MARKER_SIZE = 0.10 # Neste exemplo, 10 cm (0.10 metros)
        
        # Criando matriz focal aproximada baseada no FOV da Webcam para perspectiva 3D
        fx = width / (2 * math.tan(math.radians(H_FOV)/2))
        fy = height / (2 * math.tan(math.radians(V_FOV)/2))
        cameraMatrix = np.array([[fx, 0, width/2], [0, fy, height/2], [0, 0, 1]], dtype=np.float32)
        distCoeffs = np.zeros((4,1), dtype=np.float32)
        
        half_size = MARKER_SIZE / 2.0
        # Pontos 3D do modelo do marcador (ordem: sup-esq, sup-dir, inf-dir, inf-esq)
        objPoints = np.array([
            [-half_size,  half_size, 0],
            [ half_size,  half_size, 0],
            [ half_size, -half_size, 0],
            [-half_size, -half_size, 0]
        ], dtype=np.float32)
        
        # Transforma os pixels de detecção pra float32
        imgPoints = c.astype(np.float32)
        
        # Resolve PnP: Estima a Posição e Orientação 3D (Rvec=Rotação, Tvec=Translação)
        sucesso, rvec, tvec = cv2.solvePnP(objPoints, imgPoints, cameraMatrix, distCoeffs)
        
        if sucesso:
            # A magnitude Euclidiana do vetor de translação é a distância fiel em metros
            distanciamento_metros = float(np.linalg.norm(tvec))
            
            # (Opcional) Desenha os eixos dimensionais 3D X/Y/Z no centro do marcador visual
            cv2.drawFrameAxes(frame, cameraMatrix, distCoeffs, rvec, tvec, (MARKER_SIZE/2))
        else:
            distanciamento_metros = 3.0 # Ficar em 3m caso falhe PnP
        # -------------------------------------------------------------------

        # Envia a mensagem pro Ardupilot! 
        master.mav.landing_target_send(
            int(time.time() * 1000000),         # time_usec (microsegundos)
            0,                                  # target_num (0 é padrão)
            mavutil.mavlink.MAV_FRAME_BODY_NED, # frame_coord referencial
            angle_x,                            # Desvio horizontal de alvo (rad)
            angle_y,                            # Desvio vertical de alvo (rad)
            distanciamento_metros,              # Distância real calculada do vetor (m)
            0,                                  # size_x
            0                                   # size_y
        )
        
        # Desenha o quadrado em torno do ArUco detectado
        aruco.drawDetectedMarkers(frame, corners)
        print(f"[>>] Alvo Id:{ids[0][0]} | X:{math.degrees(angle_x):>5.1f}° Y:{math.degrees(angle_y):>5.1f}° | Dist:{distanciamento_metros:>5.2f}m")

    try:
        # Exibe a imagem processada com as anotações
        cv2.imshow("Mapeamento OpenMV - MAVLink", frame)

        # Teclando 'q' encerra a visão e o script
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Finalizando sistema...")
            break
    except cv2.error:
        # Tratamento de erro para WSL e sistemas sem interface gráfica
        # Remove a dependência da tela e permite a simulação puramente no terminal
        pass

cap.release()
cv2.destroyAllWindows()
