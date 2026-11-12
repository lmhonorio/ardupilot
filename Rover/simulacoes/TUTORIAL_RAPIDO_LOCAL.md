# Início rápido — controle local

Tudo roda neste computador: Gazebo, SITL/Lua/MAVProxy, FAST-LIO, mapa,
Mission Manager e RViz. Requer a instalação e compilação já realizadas.
Perfil: MID360 e câmera ligados, Insta360/Hokuyo e desvio desligados;
docking por LiDAR e recarga simulada.

Use este fluxo **ou** o [remoto](TUTORIAL_RAPIDO_REMOTO.md). Comece sem uma
simulação anterior aberta. Se estava usando a Raspberry, com o rover parado
em HOLD, encerre o container antes de abrir o gerenciador local:

```bash
cd ~/ardupilot/Rover
./companion/control.sh down
```

## Preparação em cada terminal

Execute este bloco em **cada novo terminal** antes do comando daquele serviço:

```bash
cd ~/ardupilot/Rover
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
unset ROVER_COMPANION_HOST
source ros2/setup.bash
```

Os comandos abaixo permanecem abertos; use um terminal para cada um.

## Terminal 1 — Gazebo

```bash
./ros2/service.sh gazebo \
  world:=gravel_plane N:=1 start_x:=2.5 start_y:=-2 \
  real_time_factor:=1.0 \
  mid360_enabled:=true samsung_s23_ultra_cam_enabled:=true \
  insta360_enabled:=false hokuyo_enabled:=false
```

## Terminal 2 — SITL, Lua e MAVProxy

```bash
./sim_1_gzrover.sh --foreground \
  --add-param-file nrover_sem_desvio.param --no-rebuild
```

Aguarde GPS/EKF prontos. O QGroundControl, se aberto neste computador,
recebe telemetria na porta UDP 14550. `--no-rebuild` usa o firmware já compilado.

## Terminal 3 — FAST-LIO

```bash
./ros2/service.sh fastlio
```

Deixe o rover parado durante a inicialização e espere o contador `poses` aumentar.

## Terminal 4 — mapa 3D e nuvem local

```bash
./ros2/service.sh mapa --wall-timeout 3
```

## Terminal 5 — gerenciamento de missão e docking

```bash
./ros2/service.sh mission-manager --vehicle sitl \
  --no-qr --geometry-process --simulate-charge
```

Abra com o rover parado. Aguarde a reconciliação: `ESPERA` quando estiver fora;
`RECARGA`/`RECARGA_CONCLUIDA` quando estiver corretamente estacionado.
Não abra `aguardar_docking.py`, `entrar_container.py` ou outro controlador junto.

## Terminal 6 — RViz

```bash
./ros2/service.sh rviz
```

Fixed Frame: `camera_init`. Mapa: `/rover_slam_3d/map`.
Nuvem local: `/rover_slam_3d/local_cloud`.

## Terminal 7 — consultar e iniciar a missão

Consultar uma vez:

```bash
ros2 topic echo /mission_manager/status --once --full-length
```

Enviar a missão e iniciar o ciclo:

```bash
./ros2/service.sh mission-start "$PWD/Missions/new_argo.plan"
```

Troque o caminho para usar outro `.plan` ou `.waypoints`. O último waypoint
deve terminar diante da abertura de uma estação cadastrada, a menos de 8 m
do centro. Se já estiver estacionado, o gerenciador aguarda a carga, sai de
ré 3 m e inicia a missão. Ao terminar, faz docking, desarma e simula a recarga.
Não é necessário mudar para AUTO ou GUIDED manualmente.

Acompanhar continuamente (Ctrl+C encerra apenas esta visualização):

```bash
ros2 topic echo /mission_manager/status --full-length
```

## Opcional — câmera em outro terminal

```bash
./ros2/service.sh camera
```

## Encerrar

No terminal de comandos:

```bash
ros2 service call /mission_manager/cancel std_srvs/srv/Trigger '{}'
ros2 topic echo /mission_manager/status --once --full-length
```

Confirme HOLD e parada. Depois use Ctrl+C, nesta ordem: Mission Manager,
mapa, FAST-LIO, câmera/RViz, SITL e Gazebo. Se houver `FALHA`, leia `last_error`
antes de tentar outra missão; HOLD sozinho não limpa a falha do gerenciador.

Detalhes: [tutorial do Mission Manager](mission_manager/TUTORIAL.md).
