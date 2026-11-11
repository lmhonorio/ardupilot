# GLIM com MID360 — ROS 2 Jazzy / Gazebo Harmonic

O GLIM usa LiDAR e IMU, odometria GICP em CPU e otimização global por grafo de poses com fechamento de laços. O [instalador ROS 2](ROS2_JAZZY.md#1-instalar-uma-vez) instala `ros-jazzy-glim-ros` pelo repositório oficial do autor. Não usa os binários ROS 1 de `python/.deps/glim`.

## Executar

Com o Gazebo e sensores ativos, deixe o rover parado durante a inicialização da IMU. Em um terminal:

```bash
cd ~/ardupilot/Rover
./ros2/service.sh glim
```

Em outro:

```bash
cd ~/ardupilot/Rover
./ros2/service.sh rviz-glim
```

Para abrir o conjunto completo, use `./iniciar_simulacao_completa.sh --glim`. O GLIM não precisa de FAST-LIO para mapear; o conjunto mantém FAST-LIO como odometria contínua enviada ao Lua e usada no docking. GLIM não envia comandos ao rover. Não injete a pose global corrigida por fechamento de laços no controle local sem tratar suas descontinuidades.

O **Fixed Frame** do RViz2 é `glim_map`. A varredura aparece antes do mapa global: o mapa depende da criação de submapas enquanto o rover se move. O ensaio parado da migração validou odometria e exportação de submapa/grafo; não validou um evento de fechamento de laço.

## Tópicos

| Tópico | Conteúdo |
| --- | --- |
| `/glim/input/points` | XYZ do Harmonic, sem correção radial Classic |
| `/glim_ros/points` | Varredura processada |
| `/glim_ros/odom` | Odometria local, `glim_odom` → `glim_imu` |
| `/glim_ros/pose` | Pose global em `glim_map` |
| `/glim_ros/map` | Mapa global |
| `/glim/status` | Estado do adaptador, contagens e idade da pose |

Diagnóstico:

```bash
source ~/ardupilot/Rover/ros2/setup.bash
ros2 topic echo /glim/status --once
ros2 topic hz /glim_ros/odom
```

O adaptador recusa timestamps inválidos ou antigos. Não transforma a varredura uniforme do simulador no padrão temporal não repetitivo do Livox real.

## Configuração e arquivos

O perfil está em `python/slam/glim/config/`. A inicialização copia os defaults da versão instalada e aplica as opções do rover: frames separados, CPU, sem visualizador interno e grafo global com otimização. As opções `--namespace`, `--cloud-topic`, `--imu-topic`, `--duration` e `--output-dir` podem ser passadas após `./ros2/service.sh glim`; consulte `--help`. Alterar o namespace dos sensores não cria automaticamente outro conjunto de frames/mapa: o serviço pronto atende a um rover.

Ctrl+C solicita o encerramento normal e a gravação em `python/run/glim/<data>/dump`. Aguarde terminar. O uso de SIGKILL pode impedir a exportação. Os patches de log `LOOP_ACCEPTED` e drenagem de fila do antigo build ROS 1 não fazem parte do pacote binário Jazzy; não use essa mensagem como critério obrigatório no novo ambiente.

A nuvem usada pelo docking continua sendo `/rover_slam_3d/local_cloud`, gerada a partir de FAST-LIO, com controle de validade temporal. Consulte [SLAM e docking 3D](SLAM_DOCKING_3D.md).
