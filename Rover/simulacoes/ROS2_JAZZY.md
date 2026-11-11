# Rover: migração para ROS 2 Jazzy e Gazebo Harmonic

## Estado da migração

Ambiente identificado em 09/11/2026: Ubuntu 24.04 (Noble). A integração foi portada para ROS 2 e validada em ambiente Jazzy/Harmonic isolado: sensores, FAST-LIO, GLIM, mapa 3D e envio de pose ao Lua publicaram dados reais da simulação. A instalação nativa e a compilação dos quatro pacotes terminaram. Passaram 65 testes de percepção/controle, 20 testes de missão e o teste real de comunicação DDS. Gazebo/RViz2 nativos abriram, inclusive com sensores em modo headless. Um ensaio isolado comandou avanço de 1,58 m, parada em HOLD e desarmamento. Os detalhes e limites estão em [VALIDACAO_ROS2.md](VALIDACAO_ROS2.md). O ensaio encontrou diferença relevante entre o deslocamento FAST-LIO e o GPS/GLIM; a precisão FAST-LIO/docking ainda não está aprovada. Os ganhos de controle foram preservados.

ROS 2 Jazzy usa Python 3.12 do Ubuntu 24.04. Python 3.8 e os binários Noetic não devem ser usados com as bibliotecas Jazzy. O projeto usa `ros2/venv` com acesso aos pacotes ROS instalados pelo APT. Não carregue `~/catkin_ws/devel/setup.bash` neste ambiente.

O backup dos scripts anteriores está em `Rover/backups/ros1_antes_ros2_20261109`. Os assets originais em `~/catkin_ws/src/rover-argo-gazebo` e o plugin antigo em `~/ardupilot_gazebo` permanecem separados da nova compilação.

## 1. Instalar uma vez

Em um terminal novo:

```bash
cd ~/ardupilot/Rover
sudo bash ros2/install_ros2.sh
./ros2/build.sh
```

O primeiro comando instala Jazzy, ros_gz/Harmonic, RViz2, slam_toolbox, MAVROS e GLIM CPU, além das dependências de compilação. Precisa da sua senha local de administrador. Não executa atualização de distribuição nem remove Noetic.

O segundo prepara o Python, compila o plugin ArduPilot do Harmonic e o FAST-LIO ROS 2, gera as interfaces Livox usadas pelo simulador e converte os assets locais. As revisões compiladas ficam em `ros2/dependencies.lock`. O pacote `livox_ros_driver2` deste workspace contém **somente mensagens**, não o driver para ligar um MID360 físico.

A geração mantém os visuais, massas, rodas, obstáculos e container do cenário local. Três malhas de colisão dos acessórios (poste, suporte e câmera) foram substituídas por caixas envolventes calculadas da malha: as originais causavam falha no DART. A carroceria e as rodas conservam suas colisões primitivas. Os materiais Ogre antigos são convertidos para PBR, incluindo o QR. O LiDAR é uma emulação `gpu_lidar` com campo de visão do MID360; **não reproduz o padrão temporal não repetitivo do Livox real**. Cada nuvem simulada corresponde a um único instante. Não se aplica a correção radial de 10 cm do plugin Classic.

## 2. Um serviço em cada terminal

Cada `service.sh` carrega o ambiente ROS 2 e o Python correto. Execute os comandos a partir de `~/ardupilot/Rover`.

### Terminal 1: mundo, rover e sensores

```bash
cd ~/ardupilot/Rover
./ros2/service.sh gazebo
```

Sem a janela Gazebo, mas com renderização dos sensores:

```bash
./ros2/service.sh gazebo gui:=false
```

Também existe o comando equivalente ao antigo `roslaunch`:

```bash
source ~/ardupilot/Rover/ros2/setup.bash
ros2 launch rover_argo_gazebo multi_rover_argo.launch.py \
  N:=1 world_name:=gravel_plane start_x:=0 start_y:=0 z:=0 \
  mid360_enabled:=true insta360_enabled:=false \
  samsung_s23_ultra_cam_enabled:=true hokuyo_enabled:=true
```

A configuração inicial usa **um rover**, mundo `gravel_plane`, posição inicial `(0, 0, 0.155)`, MID360, Samsung e Hokuyo habilitados. A **Insta360 fica desligada por padrão**, inclusive no lançador completo. Para habilitá-la explicitamente, inicie o mundo com `./ros2/service.sh gazebo insta360_enabled:=true`. O container mantém seu interior de 1 × 1 × 1 m, centro `(5, -2)` e entrada voltada para `-X`. A altura de spawn evita interpenetração inicial das rodas com o chão.

### Terminal 2: ArduPilot, Lua e MAVProxy

Com desvio de obstáculos:

```bash
./sim_1_gzrover.sh --foreground --add-param-file nroverobst.param
```

Sem desvio de obstáculos:

```bash
./sim_1_gzrover.sh --foreground --add-param-file nrover_sem_desvio.param
```

Após compilar o firmware uma vez, acrescente `--no-rebuild` para reutilizá-lo. O backend é `--model JSON`, protocolo do plugin Harmonic. A origem GPS é `SEParnaiba`, compatível com o cadastro do container. EEPROM, logs e cópia Lua ficam em `ros2/run/sitl`, separados da simulação Classic. O arquivo selecionado é carregado sem substituir seus ganhos por outro perfil.

MAVProxy publica para QGroundControl em `127.0.0.1:14550`, MAVROS em `14555`, odometria Lua em `14556`, proximidade em `14557` e rotinas de missão/docking em `14551`. As saídas de rede anteriores para `192.168.0.131` também são mantidas. Os scripts de docking continuam a controlar o ArduPilot por MAVProxy, não pelo Gazebo.

### MAVROS: terminal adicional, se necessário

```bash
./ros2/service.sh mavros
```

O lançador completo abre esse serviço. A conexão é `udp://:14555@`; o namespace é `/rover_argo_N1/Instance1`. As rotinas de docking e a ponte Lua continuam usando MAVProxy diretamente. O instalador inclui o modelo geoidal EGM96 exigido pelo MAVROS.

### Terminal 3: percepção de obstáculos — somente no perfil com desvio

```bash
./ros2/service.sh obstaculos
```

O módulo projeta o LiDAR e transmite `OBSTACLE_DISTANCE` por MAVProxy. Com `nroverobst.param`, dados ausentes continuam impedindo o prearm (`PRX1: No Data`). Não desative essa checagem para esconder falha do sensor. No perfil sem desvio, este terminal não é necessário.

### Terminal 4: FAST-LIO, odometria contínua e Lua

```bash
./ros2/service.sh fastlio
```

Mantenha o rover parado durante a inicialização da IMU. Aguarde `poses` e `enviadas` aumentarem. Para navegar, aguarde também `AHRS: EKF3 active` e as checagens PreArm saudáveis; com todos os sensores/SLAM ligados a física pode rodar mais devagar que o tempo real, prolongando essa espera. A ponte preserva o protocolo `DATA64/FLIO` do receptor Lua existente. A pose enviada deve ter timestamp e covariância válidos. Nenhuma covariância é inventada.

### Terminal 5: mapa 3D e nuvem local para docking

```bash
./ros2/service.sh mapa
```

Mapa de visualização: `/rover_slam_3d/map`; geometria recente, compensada pelo movimento: `/rover_slam_3d/local_cloud`. O mapa acumulado não é usado como se fossem obstáculos atuais. Ao perder odometria ou detectar salto de relógio/pose, a nuvem local é invalidada.

### Terminal 6: visualização

```bash
./ros2/service.sh rviz
```

Fixed Frame: `camera_init`. Para visualizar apenas o sensor antes do FAST-LIO, mude para `base_link` e adicione PointCloud2 no tópico `/rover_argo_N1/Instance1/livox/lidar`, Reliability `Best Effort`.

### Terminal 7: câmera

```bash
./ros2/service.sh camera
# Alternativa, somente se o mundo foi iniciado com insta360_enabled:=true:
./ros2/service.sh camera --camera insta360
```

Diagnóstico do LiDAR sem RViz:

```bash
./ros2/service.sh lidar
```

### Opcional: GLIM e fechamento de laços

```bash
./ros2/service.sh glim
```

Em outro terminal:

```bash
./ros2/service.sh rviz-glim
```

GLIM usa frames `glim_map/glim_odom/glim_imu/glim_lidar`, separados do FAST-LIO. O fechamento de laços corrige o mapa global. Essas correções não são injetadas no controlador de docking como saltos de pose. O FAST-LIO permanece como fonte de odometria contínua e de dados para Lua. GLIM salva sua sessão em `python/run/glim/<data>/dump` ao encerrar normalmente.

### Opcional: mapa 2D

Com FAST-LIO ativo:

```bash
./ros2/service.sh mapa2d
```

O backend é `slam_toolbox`, usando a projeção MID360 e TF contínua do FAST-LIO. Adicione Map `/map` no RViz2, frame `map_2d`. O antigo Hector/ROS 1 não é iniciado.

## 3. Docking

Escolha **um** controlador por vez. Eles continuam a depender de rover parado/armado em GUIDED, estação cadastrada e sensores recentes; não criam uma missão automaticamente.

Entrada visual:

```bash
./ros2/service.sh docking-qr
```

Entrada por LiDAR:

```bash
./ros2/service.sh docking-lidar
```

Saída após terminar o controlador de entrada:

```bash
./ros2/service.sh sair-qr
# Ou:
./ros2/service.sh sair-lidar
```

O pan/tilt usa os controladores do Harmonic e feedback medido em `joint_states`, substituindo o serviço Classic `get_joint_properties`. O cadastro e as tolerâncias geométricas permanecem; precisão de estacionamento e dinâmica precisam ser revalidadas no novo motor físico antes de considerar a migração concluída.

## 4. Abrir tudo em janelas

```bash
cd ~/ardupilot/Rover
./iniciar_simulacao_completa.sh
# Sem desvio:
./iniciar_simulacao_completa.sh --without-avoidance
# Acrescentar GLIM:
./iniciar_simulacao_completa.sh --glim
# Reutilizar firmware já compilado:
./iniciar_simulacao_completa.sh --no-rebuild
```

O lançador abre os serviços, sem armar nem iniciar missão. Docking é iniciado separadamente. `start_gz_simple.sh` e `sim_gz_simple.sh` encaminham para esse novo lançador. `sim_1_gzrover.sh` e `sim_gz1rover.sh` iniciam apenas SITL/MAVProxy; com desvio, abra a percepção no seu terminal.

## 5. Mais de um rover

O mundo aceita `N:=1` até `N:=10`, espaçamento `dx:=3 dy:=3` e as mesmas opções de sensores. Por exemplo:

```bash
./ros2/service.sh gazebo N:=3
```

Inicie cada SITL em um terminal, com o mesmo índice do modelo (o índice SITL começa em zero):

```bash
./sim_1_gzrover.sh --instance 0 --add-param-file nrover_sem_desvio.param
./sim_1_gzrover.sh --instance 1 --add-param-file nrover_sem_desvio.param
./sim_1_gzrover.sh --instance 2 --add-param-file nrover_sem_desvio.param
```

Não execute esses três comandos sequencialmente no mesmo terminal: cada um permanece em execução. O firmware deve ser compilado uma vez antes de iniciar instâncias adicionais com `--no-rebuild`. Cada instância tem EEPROM separada, SYSID `índice+1`, porta JSON `9002+10×índice`, TCP `5760+10×índice` e portas de consumidores MAVProxy incrementadas em 10. QGroundControl recebe todos em 14550. Os tópicos ficam em `/rover_argo_N1/Instance1`, `Instance2` etc.; TF das instâncias adicionais recebe prefixo `rover_2/`, `rover_3/`.

**Os serviços prontos de SLAM, docking, câmera e percepção atendem à primeira instância.** Executar essas funções em vários robôs exige instâncias desses serviços com tópicos, TF e portas próprios; `N:=3` sozinho não cria três pipelines de navegação. A validação integrada de movimento foi planejada para um rover.

## 6. Diagnóstico e encerramento

```bash
source ~/ardupilot/Rover/ros2/setup.bash
ros2 node list
ros2 topic list -t
ros2 topic hz /clock
ros2 topic hz /rover_argo_N1/Instance1/livox/lidar
ros2 topic hz /rover_argo_N1/Instance1/livox/imu
ros2 topic echo /fast_lio/odometry --once
ros2 run tf2_ros tf2_echo camera_init base_link
```

O setup também remove caminhos GTK/GIO do Snap herdados da IDE, que podem impedir RViz2/Gazebo de abrir.

Não existe `roscore`, `roslaunch`, `rosnode` ou `rostopic` neste fluxo. Use `ros2 launch`, `ros2 node` e `ros2 topic`. Todos os terminais devem usar o mesmo `ROS_DOMAIN_ID` (padrão 0) e relógio de simulação.

Para parar, interrompa primeiro docking/missão, coloque o rover em HOLD e desarme. Depois use Ctrl+C nos terminais de percepção/SLAM (GLIM salva o mapa), SITL e Gazebo. Fechar somente a janela RViz não encerra a simulação.

## Referências

- [ROS 2 Jazzy: instalação em Ubuntu 24.04](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [Pares compatíveis ROS e Gazebo](https://gazebosim.org/docs/harmonic/ros_installation/)
- [Plugin ArduPilot para Gazebo moderno](https://github.com/ArduPilot/ardupilot_gazebo)
- [FAST-LIO: port ROS 2 utilizado](https://github.com/Ericsii/FAST_LIO/tree/ros2)
- [GLIM: pacotes ROS 2 oficiais do autor](https://koide3.github.io/glim/installation.html)
