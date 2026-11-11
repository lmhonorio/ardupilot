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

Os perfis de simulação usam o ajuste de navegação descrito em [resultados e parâmetros](AJUSTE_NAVEGACAO_20261105.md): velocidade de missão de 0,6 m/s, raio de aceitação de 0,30 m e giro no próprio eixo limitado a 30°/s, com 2 s de estabilização após o alinhamento. O seguimento da trajetória é feito pelo ArduRover; a alocação Lua é uma opção separada. Os ensaios foram feitos com `BATT_SOC_ALLOC=0`, como estava na instância observada.

**A EEPROM existente pode prevalecer sobre `--add-param-file`.** Para aplicar somente o ajuste de controle, no MAVProxy, com o rover parado e desarmado:

```text
param load /home/grin/ardupilot/Rover/navegacao_gazebo.param
param show PSC_POS_P
param show PSC_VEL_P
param show WP_SPEED
param show WP_PIVOT_DELAY
```

Os valores esperados são, respectivamente, `0.6`, `2`, `0.6` e `2`. Essa carga persiste na EEPROM e não muda a missão, os sensores, o desvio ou a habilitação do Lua. Os comandos abaixo conferem separadamente o perfil de obstáculos.

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

## Terminal 5 — mapa 3D e nuvem local para docking

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
cd ~/ardupilot/Rover
/usr/bin/python3.8 python/slam/slam_mid360_3d.py
```

O mapa usa XYZ e a odometria do FAST-LIO do Terminal 4. Publica `/rover_slam_3d/map` e `/rover_slam_3d/local_cloud`. A segunda reúne 1,5 s de varreduras compensadas pelo movimento e é necessária para o docking LiDAR. Ctrl+C exporta `mapa.pcd` em `python/run/slam_3d/`. Detalhes em [SLAM e docking 3D](SLAM_DOCKING_3D.md).

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
cd ~/ardupilot/Rover
rviz -d python/slam/mid360_3d.rviz
```

A configuração abre o mapa 3D e a geometria local, com Fixed Frame `camera_init`. Marque “MID360 XYZ corrigido” para ver também a varredura instantânea. Abra o QGroundControl pelo menu ou atalho habitual; o MAVProxy envia telemetria para UDP local 14550.

Para a alternativa de entrada e saída por LiDAR, consulte [Docking com MID360](docking/DOCKING_LIDAR.md).

## Missão e encerramento

Depois de conferir GPS, EKF e PreArm, o QGroundControl pode ser usado para acompanhar e enviar missões. Inicie a [espera automática](docking/README%20DOCKING.md#espera-automática-para-docking-em-guided) em outro terminal. Ela observa GUIDED e a distância inferior a 8 m da estação, sem depender de missão ou waypoint. Para a missão de volta própria, consulte a [volta e entrada automática](docking/README%20DOCKING.md#volta-e-entrada-automática-no-contêiner); não execute ambos na mesma porta. A sequência completa de volta e entrada ainda não foi validada em movimento.

Para encerrar, pressione Ctrl+C no terminal de percepção, FAST-LIO, mapa e câmera; depois no Terminal 2 e por último no Terminal 1. Feche RViz e QGroundControl pelas próprias janelas. Caso o console do MAVProxy não encerre com Ctrl+C, digite `exit` nele e aguarde o SITL terminar.

## Navegação com a alocação Lua

O script `scripts/rover-controlallocation.lua` repassa aos motores os comandos de direção e velocidade calculados pelo ArduRover. A rotação Lua que havia sido acrescentada após o último waypoint foi removida. O ArduRover continua calculando a trajetória e o desvio de obstáculos; o Lua não impõe uma correção direta sobre o caminho do planejador.

Para usar a alocação, confira no MAVProxy `SCR_ENABLE=1`, `SCR_USER5=2` e `BATT_SOC_ALLOC=1`. Se a alocação estiver em zero, com o rover parado e desarmado, use `param set BATT_SOC_ALLOC 1`; a gravação persiste na EEPROM. Reinicie o SITL para carregar as alterações dos arquivos Lua.
## GLIM opcional

O mapeador GLIM pode ser iniciado em outro terminal com `/usr/bin/python3.8 python/slam/glim/run_glim.py`. Veja instalação, RViz e fechamento de laços em [GLIM com MID360](GLIM_MID360.md). Ele não substitui automaticamente a odometria usada no docking.
