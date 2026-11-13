# Início rápido — controle local

Tudo roda neste computador: Gazebo, SITL/Lua/MAVProxy, FAST-LIO, mapa,
Mission Manager e RViz. Requer a instalação e compilação já realizadas.
Perfil: MID360 e câmera ligados, Insta360/Hokuyo e desvio desligados;
docking por LiDAR e recarga simulada.

Use este fluxo **ou** o [remoto](../companion/TUTORIAL_RAPIDO_REMOTO.md). Comece sem uma
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
unset FASTRTPS_DEFAULT_PROFILES_FILE
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

## Parar a missão e encerrar tudo

### 1. Solicitar parada, mantendo os serviços abertos

No terminal 7, se estiver acompanhando o status, pressione Ctrl+C para voltar
ao prompt. Esse Ctrl+C fecha somente a visualização. Depois execute:

```bash
ros2 service call /mission_manager/cancel std_srvs/srv/Trigger '{}'
ros2 topic echo /mission_manager/status --once --full-length
```

Espere `mode: 4` (HOLD), telemetria recente (`fresh: true`) e rover parado.
A resposta ao cancelamento confirma o pedido, não a conclusão da parada.
O cancelamento pode aparecer como `FALHA` com motivo de cancelamento pelo operador.
Ele não garante desarmamento; para encerrar, desarme pelo QGroundControl após
confirmar a parada e confira `armed: false`.

Se o serviço não responder, interrompa a chamada com Ctrl+C e selecione HOLD
no QGroundControl ou no console MAVProxy. Confirme a parada antes de continuar.

### 2. Fechar os processos nesta ordem

Em cada terminal indicado, pressione **Ctrl+C** e espere o processo terminar:

| Ordem | Terminal / programa |
| --- | --- |
| 1 | Terminal 5 — Mission Manager |
| 2 | Terminal 4 — mapa 3D (aguarde a exportação do mapa) |
| 3 | Terminal 3 — FAST-LIO |
| 4 | Terminal opcional — câmera; terminal 6 — RViz |
| 5 | Terminal 2 — SITL e MAVProxy |
| 6 | Terminal 1 — Gazebo |

Feche também o QGroundControl pelo menu Sair ou pelo botão de fechar a janela.
Não use Ctrl+Z: ele suspende o processo e pode deixar portas e bloqueios ocupados.
Fechar apenas a janela do Gazebo não garante o encerramento do servidor.

### 3. Conferir se restaram processos

No terminal 7:

```bash
pgrep -af '[s]im_vehicle.py|[a]rdurover|[m]avproxy.py|[g]z sim|[g]z-sim|[r]viz2|[r]un_fast_lio.py|[f]astlio_mapping|[s]lam_mid360_3d.py|[m]ission_manager/manager.py|[r]ead_ros_camera.py|[Q]GroundControl'
```

Nenhuma linha indica que não há processos com esses nomes. Se aparecer algum,
confira o PID e o comando; volte ao terminal correspondente para encerrá-lo.
Não remova arquivos `.lock` nem mate todos os processos Python/ROS indiscriminadamente.

Para apenas interromper a missão e manter a simulação aberta, execute somente
o passo 1. Antes de outra missão, leia `last_error`, corrija a causa se necessário
e solicite reconciliação:

```bash
ros2 service call /mission_manager/reconcile std_srvs/srv/Trigger '{}'
```

HOLD sozinho não limpa a falha do gerenciador.

Detalhes: [tutorial do Mission Manager](../mission_manager/TUTORIAL.md).
