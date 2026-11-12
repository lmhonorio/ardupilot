# Início rápido — controle na Raspberry

No computador: Gazebo, SITL/Lua/MAVProxy, RViz, câmera e comandos de missão.
Na Raspberry `leo@192.168.0.198`: Docker com FAST-LIO, mapa e Mission Manager
(docking LiDAR, saída e recarga simulada).

Este guia usa a imagem já instalada e os buffers de rede já configurados.
Comece sem a simulação anterior aberta. Encerre os serviços **locais** de
FAST-LIO, mapa, Mission Manager e docking antes de usar este fluxo.
Não execute `iniciar_simulacao_completa.sh` junto.

## Preparação em cada terminal do computador

Todos os comandos deste tutorial são digitados **neste computador**, inclusive
os que iniciam o container por SSH. Em cada novo terminal:

```bash
cd ~/ardupilot/Rover
export ROS_DOMAIN_ID=0
export ROVER_REMOTE=leo@192.168.0.198
source companion/pc_env.bash
```

## Terminal 1 — Gazebo local

```bash
./ros2/service.sh gazebo \
  world:=gravel_plane N:=1 start_x:=2.5 start_y:=-2 \
  real_time_factor:=0.5 \
  mid360_enabled:=true samsung_s23_ultra_cam_enabled:=true \
  insta360_enabled:=false hokuyo_enabled:=false
```

O fator `0.5` é o perfil validado na Pi 4: cada segundo simulado leva cerca
de dois segundos reais. Não altera a velocidade configurada em m/s.

## Terminal 2 — SITL local com MAVLink para a Raspberry

```bash
ROVER_COMPANION_HOST=192.168.0.198 \
  ./sim_1_gzrover.sh --foreground \
  --add-param-file nrover_sem_desvio.param --no-rebuild
```

Aguarde GPS/EKF prontos. O encaminhamento à Raspberry usa UDP 14600;
o QGroundControl local continua recebendo na porta 14550.

## Terminal 3 — iniciar os serviços remotos e acompanhar logs

```bash
./companion/control.sh up
./companion/control.sh logs
```

O container inicia MAVProxy, recepção de sensores, FAST-LIO, mapa e gerenciador.
Aguarde os sensores e a reconciliação. Não abra cópias locais desses serviços.
Ctrl+C no acompanhamento dos logs **não** encerra o container.
O perfil instalado usa LiDAR sem QR e desvio desligado.

## Terminal 4 — RViz local

```bash
./ros2/service.sh rviz
```

Fixed Frame: `camera_init`. Mapa: `/rover_slam_3d/map`.
Nuvem local: `/rover_slam_3d/local_cloud`. O mapa é calculado na Raspberry.

## Terminal 5 — consultar e iniciar a missão

Consultar uma vez:

```bash
./companion/control.sh status
```

Enviar um arquivo deste computador e iniciar:

```bash
./companion/control.sh mission Missions/new_argo.plan
```

Esse comando copia o arquivo para a Raspberry e chama `/mission_manager/start_file`.
Troque o caminho para usar outro `.plan` ou `.waypoints`. Não passe um caminho
`/home/grin/...` diretamente ao serviço remoto: ele lê arquivos no container.
O último waypoint deve terminar diante da abertura da estação cadastrada,
a menos de 8 m do centro.

Se estiver estacionado, aguarda a recarga e sai de ré 3 m antes de iniciar.
Ao terminar a missão, entra, desarma e simula a recarga até 100%.
A aceitação do pedido não significa que o ciclo terminou. Acompanhe:

```bash
./companion/control.sh watch
```

O estado final esperado é `RECARGA_CONCLUIDA`, `inside=true`, `mode=4` (HOLD)
e `armed=false`. Ctrl+C encerra apenas o acompanhamento do status.

## Opcional — câmera local em outro terminal

```bash
./ros2/service.sh camera
```

## Encerrar

No terminal de comandos:

```bash
./companion/control.sh cancel
./companion/control.sh status
```

Confirme HOLD e parada; depois encerre os serviços remotos:

```bash
./companion/control.sh down
```

Feche câmera/RViz, SITL e Gazebo com Ctrl+C nos respectivos terminais.
Se o status estiver em `FALHA`, leia `last_error` e corrija a causa antes de
usar `./companion/control.sh reconcile` e solicitar outra missão.

Este fluxo é de simulação. Para hardware físico e configuração da imagem,
consulte o [tutorial completo](companion/RASPBERRY_DOCKER.md).
Alternativa: [executar tudo localmente](TUTORIAL_RAPIDO_LOCAL.md).
