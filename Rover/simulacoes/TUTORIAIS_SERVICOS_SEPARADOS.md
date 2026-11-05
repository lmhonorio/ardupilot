# Simulação ROS: um serviço por terminal

Este roteiro usa ROS Noetic, Gazebo Classic, o mundo `gravel_plane`, o ArduRover, MAVProxy e Python 3.8 instalados neste computador. Mantenha cada terminal aberto e inicie os serviços na ordem. Não execute este roteiro ao mesmo tempo que `iniciar_simulacao_completa.sh` ou outro SITL/Gazebo.

Escolha **com desvio** ou **sem desvio** no Terminal 2. O primeiro arquivo é `nroverobst.param`; o segundo é `nrover_sem_desvio.param`, com os mesmos parâmetros de controle e `OA_TYPE=0`, `PRX1_TYPE=0`, `AVOID_ENABLE=0`. Desligar `AVOID_ENABLE` também desliga o uso da cerca pelo desvio simples. Sem desvio o rover pode colidir com os obstáculos.

Os arquivos passados com `--add-param-file` fornecem valores iniciais; os parâmetros gravados anteriormente na EEPROM `Rover/eeprom.bin` podem prevalecer. Os dois comandos abaixo usam a **mesma EEPROM e missão** e não apagam nenhuma delas. Confira os valores no MAVProxy conforme o Terminal 2, inclusive ao trocar de perfil.

## Terminal 1 — Gazebo, mundo e sensores

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch rover_argo_gazebo multi_rover_argo.launch \
  N:=1 world_name:=gravel_plane start_x:=0 start_y:=0 z:=0 \
  mid360_enabled:=true insta360_enabled:=true \
  samsung_s23_ultra_cam_enabled:=true hokuyo_enabled:=true
```

Aguarde o mundo e o rover aparecerem. Este comando inicia o ROS master se necessário. O contêiner fica em `(5,-2)` m; sua abertura fica em `x=4,5 m`.

## Terminal 2 — ArduRover, Lua e MAVProxy

Em **outro** terminal, escolha um comando:

**Com desvio:**

```bash
cd ~/ardupilot/Rover
./sim_1_gzrover.sh --foreground --add-param-file nroverobst.param
```

**Sem desvio:**

```bash
cd ~/ardupilot/Rover
./sim_1_gzrover.sh --foreground --add-param-file nrover_sem_desvio.param
```

`--foreground` mantém SITL e MAVProxy neste terminal e não abre a ponte de obstáculos automaticamente. O script usa o Python 3.8 da instalação pyenv, a posição `SEParnaiba`, a EEPROM e os scripts Lua do diretório `Rover`. Sem `--foreground`, o script abre outra janela e tenta iniciar a percepção automaticamente; sem `--add-param-file`, usa `nroverobst.param`. Um caminho absoluto para outro arquivo também é aceito.

Os perfis de simulação usam `WP_SPEED=0.4`, `WP_RADIUS=0.3`, `WP_PIVOT_ANGLE=60`, `ATC_STR_RAT_MAX=60` e `MOT_STR_THR_MIX=0.7` como ponto inicial de teste para a trajetória. Não são valores otimizados. A alocação Lua utiliza `MOT_STR_THR_MIX` para dar prioridade à direção quando os motores saturam; o seguimento da trajetória é feito pelo ArduRover. A EEPROM existente pode manter os valores anteriores; no MAVProxy, com o rover parado, confira e, se necessário, configure individualmente com `param set`. Isso modifica os valores persistentes do SITL.

Aguarde o prompt `MAV>` e confira os parâmetros neste próprio console:

```text
param show OA_TYPE
param show PRX1_TYPE
param show AVOID_ENABLE
param show BATT_SOC_ALLOC
```

| Perfil | OA_TYPE | PRX1_TYPE | AVOID_ENABLE |
| --- | ---: | ---: | ---: |
| Com desvio | 1 | 2 | 1 |
| Sem desvio | 0 | 0 | 0 |

Se os valores não coincidirem, ajuste-os no prompt MAVProxy **com o rover desarmado** usando `param set NOME VALOR`, para cada parâmetro divergente. Encerre esse SITL com Ctrl+C, reinicie o mesmo comando do Terminal 2 e confira novamente. `OA_TYPE` e `PRX1_TYPE` exigem reinicialização. Não use `-w`: apagaria a EEPROM e parâmetros/missão armazenados. A alocação Lua depende de `BATT_SOC_ALLOC=1`; confirme também as mensagens de inicialização do script Lua.

## Terminal 3 — percepção de obstáculos, somente com desvio

Após o Terminal 2 estar pronto, abra outro terminal:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
cd ~/ardupilot/Rover
/usr/bin/python3.8 python/obstacle_avoidance/manager.py
```

O processo mantém o processamento do MID360 e a ponte MAVLink ativos neste terminal. Não inicie também `start_obstacle_avoidance.sh` ou outra cópia do módulo. No perfil sem desvio, não execute este terminal; o LiDAR continua disponível para FAST-LIO e mapa.

Para verificar o envio, use um terminal adicional:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rostopic echo /olhe/obstacles/bridge_status
```

`ENVIADO` confirma o envio à porta 14557 do MAVProxy; não comprova que o desvio funcione em todas as situações. `SEM_MAVPROXY` ou `SCAN_DESATUALIZADO` indicam falha de conexão ou de dados. Em `PreArm: PRX1: No Data`, verifique a percepção, o MID360 e `set mavfwd` no MAVProxy. A percepção é experimental e a parada por perda de dados não foi validada.

## Terminal 4 — FAST-LIO

Com Gazebo e MAVProxy ativos, mantenha o rover parado e execute em outro terminal:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
cd ~/ardupilot/Rover
/usr/bin/python3.8 python/slam/fast_lio/run_fast_lio.py
```

Confira que `nuvens`, `imu`, `poses` e `enviadas` aumentam. O FAST-LIO encaminha a pose ao Lua pela porta 14556; não substitui automaticamente a posição do EKF. Se necessário, confira `rostopic hz /rover_argo_N1/Instance1/livox/imu` e `rostopic echo -n 1 /fast_lio/odometry` em outro terminal com os dois `source` acima.

## Terminal 5 — mapa 2D, opcional

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
cd ~/ardupilot/Rover
/usr/bin/python3.8 python/slam/slam_mid360_2d.py
```

O Hector Mapping usa o MID360 para gerar um mapa 2D próprio. Na janela, `S` salva e `Q` salva e encerra; o mapa e o referencial são independentes dos do FAST-LIO.

## Terminal 6 — câmera, opcional

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
cd ~/ardupilot/Rover
/usr/bin/python3.8 python/camera/read_ros_camera.py
```

Para sair do contêiner em ré e girar 180°, consulte [Saída do contêiner](docking/README%20DOCKING.md#saída-do-contêiner-de-ré).

## Terminal 7 — RViz, opcional

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rviz
```

Configure o referencial e os tópicos adequados ao mapa ou à nuvem que deseja observar. Abra o QGroundControl pelo menu ou atalho habitual; o MAVProxy envia telemetria para UDP local 14550.

## Missão e encerramento

Depois de conferir GPS, EKF e PreArm, o QGroundControl pode ser usado para acompanhar e enviar missões. Inicie a [espera automática](docking/README%20DOCKING.md#espera-automática-para-docking-em-guided) em outro terminal. Ela observa GUIDED e a distância inferior a 8 m da estação, sem depender de missão ou waypoint. Para a missão de volta própria, consulte a [volta e entrada automática](docking/README%20DOCKING.md#volta-e-entrada-automática-no-contêiner); não execute ambos na mesma porta. A sequência completa de volta e entrada ainda não foi validada em movimento.

Para encerrar, pressione Ctrl+C no terminal de percepção, FAST-LIO, mapa e câmera; depois no Terminal 2 e por último no Terminal 1. Feche RViz e QGroundControl pelas próprias janelas. Caso o console do MAVProxy não encerre com Ctrl+C, digite `exit` nele e aguarde o SITL terminar.
