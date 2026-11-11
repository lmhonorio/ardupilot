# Entrada e saída com MID360 e mapa 3D

As rotinas experimentais para Gazebo/SITL usam a nuvem XYZ recente `/rover_slam_3d/local_cloud`, registrada pelo FAST-LIO. O ajuste considera fundo, laterais, piso e teto do contêiner. Não usa câmera/QR. A missão não é criada ou modificada.

**Passo a passo completo:** [SLAM e docking 3D](../SLAM_DOCKING_3D.md). O início integrado já usa o novo mapa 3D e o RViz configurado.

## Pré-requisitos

Inicie Gazebo com MID360/IMU, SITL/MAVProxy, `python/slam/fast_lio/run_fast_lio.py` e `python/slam/slam_mid360_3d.py`. Prepare cada terminal com:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
cd ~/ardupilot/Rover
```

Encerre o monitor QR ou qualquer outro controlador na porta UDP 14551 antes de usar estas rotinas. O GPS e o rumo selecionam a estação cadastrada em `python/docking_lidar/estacoes_lidar.json`; a pose relativa final é medida nas superfícies LiDAR. O modelo atual assume chão nivelado, dimensões internas de 1 m e os offsets do sensor da simulação Argo.

## Verificação sem movimento

```bash
/usr/bin/python3.8 python/docking_lidar/entrar_lidar.py --diagnostico
rostopic echo -n 1 /rover_slam_3d/status
```

O diagnóstico confirma fluxo, frame e timestamps, sem conectar ao MAVProxy. Para investigar falhas:

```bash
rostopic hz /clock
rostopic hz /rover_argo_N1/Instance1/livox/lidar
rostopic hz /fast_lio/cloud_registered
rostopic hz /rover_slam_3d/local_cloud
```

Um tópico listado não garante publicação. Se `/clock` parar, confira pausa ou queda do Gazebo. Se reiniciar o mundo, reinicie também FAST-LIO e mapa. O buffer 3D é invalidado quando a odometria fica antiga, inconsistente ou muda de referência. Não se usa o mapa histórico como substituto de uma leitura recente.

## Entrada

```bash
/usr/bin/python3.8 python/docking_lidar/entrar_lidar.py
```

Espera rover armado em GUIDED e estação a menos de 8 m. Solicita HOLD, confirma três poses e volta a GUIDED para aproximar, alinhar e entrar. Perto do alvo, solicita HOLD e só desarma após parada confirmada e pose dentro de 8 cm longitudinais, 10 cm laterais e 8° de rumo. Não tenta manobrar se já estiver junto ao centro fora da tolerância.

O padrão é a nuvem local 3D compensada. `--cloud /rover_argo_N1/Instance1/livox/lidar` serve para diagnóstico da varredura bruta, mas perde a acumulação e pode voltar a ter pontos insuficientes.

## Saída

```bash
/usr/bin/python3.8 python/docking_lidar/sair_lidar.py
```

Começa com rover em HOLD, dentro da região central (até 0,30 m longitudinal/lateral e 12°). Exige FAST-LIO saudável e estável. Solicita armamento normal quando necessário, recua aproximadamente 2 m pela odometria LiDAR/IMU, verifica o recuo parado, gira 180° e desarma. Uma posição a 0,61 m do centro não atende à condição inicial; use a entrada primeiro.

O recuo permanece limitado a 18 cm laterais e 18° de erro de rumo, e a distância após parar deve ficar entre 1,85 e 2,15 m. A área atrás do rover deve estar livre; esta rotina não planeja um trajeto de saída com obstáculos.

## Desvio, dados e limites

As rotinas salvam `OA_TYPE` e `AVOID_ENABLE` antes de alterá-los para a passagem e tentam restaurá-los ao encerrar. `OA_TYPE` pode exigir reinício do ArduPilot; confirmar o parâmetro não demonstra alteração imediata do planejador. Ctrl+C tenta HOLD quando a rotina assumiu o controle.

Resultados e backup de parâmetros ficam em `python/run/docking_lidar/`. A altura dos pontos é preservada. A correção radial de 10 cm é específica do plugin simulado e não serve para o sensor físico.

A validação atual confirmou geometria em amostras estáticas perto do contêiner e a aproximadamente 4,3 m, além de testes automáticos. A entrada e a saída completas com o novo fluxo 3D ainda não foram validadas em movimento. Consulte os [resultados e limitações](../SLAM_DOCKING_3D.md#validação-de-05112026).
