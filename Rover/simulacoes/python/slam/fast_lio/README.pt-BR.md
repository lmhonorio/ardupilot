# FAST-LIO2, MID360 e posição no Lua

Esta integração executa o [FAST-LIO2 original](https://github.com/hku-mars/FAST_LIO) com o MID360 e a IMU simulados no ROS Noetic. Ela envia a pose calculada ao Lua pelo MAVProxy. Não envia comandos de movimento, não altera o EKF e não executa docking. O mapa 2D de ocupação continua disponível separadamente pelo Hector.

## 1. Abrir o mundo e os sensores

No primeiro terminal, reinicie o mundo para carregar a IMU acrescentada ao modelo MID360:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch rover_argo_gazebo multi_rover_argo.launch \
  N:=1 world_name:=gravel_plane start_x:=0 start_y:=0 z:=0 \
  mid360_enabled:=true insta360_enabled:=true \
  samsung_s23_ultra_cam_enabled:=true hokuyo_enabled:=true
```

Em outro terminal, confira que ambos publicam, encerrando cada comando com Ctrl+C:

```bash
source /opt/ros/noetic/setup.bash
rostopic hz /rover_argo_N1/Instance1/livox/lidar
rostopic hz /rover_argo_N1/Instance1/livox/imu
```

A IMU foi adicionada ao arquivo `~/catkin_ws/src/rover-argo-gazebo/rover_argo_gazebo/urdf/components/mid360.xacro`. Se usar outra cópia do pacote, ela também precisará dessa alteração.

## 2. Iniciar o ArduRover, Lua e MAVProxy

Em outro terminal, com um SITL anterior encerrado:

```bash
cd ~/ardupilot/Rover
./sim_gz1rover.sh --foreground --no-rebuild
```

Esse lançador prepara o Lua, inicia o SITL e adiciona a saída local do MAVProxy na porta UDP 14556 para o FAST-LIO. Aguarde o início do ArduRover. O receptor `lidar_odometry.lua` é instalado junto aos demais scripts. Mantenha o rover parado durante a inicialização da IMU.

## 3. Iniciar o FAST-LIO

Em outro terminal:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
cd ~/ardupilot/Rover
/usr/bin/python3.8 python/slam/fast_lio/run_fast_lio.py
```

A saída informa `nuvens`, `imu`, `poses`, `enviadas` e `descartadas`. Para chegar ao Lua, `poses` e `enviadas` precisam aumentar; no MAVProxy aparece `FAST-LIO: Lua pose available`. Se `enviadas` ficar em zero, confira o SITL/MAVProxy e a porta 14556. Uma mensagem `FAST-LIO: Lua pose stale` indica ausência de atualização recente. O diagnóstico e `status.json` ficam em `python/run/fast_lio/`.

Para verificar a odometria ROS em outro terminal:

```bash
source /opt/ros/noetic/setup.bash
rostopic echo -n 1 /fast_lio/odometry
```

A odometria ROS descreve a origem da IMU nos eixos ROS; a posição entregue ao Lua é convertida para a referência da base e os eixos descritos abaixo. No MAVProxy, `watch NAMED_VALUE_FLOAT` mostra `LIO_X`, `LIO_Y`, `LIO_Z` e `LIO_YAW`; esses quatro valores também podem ser inspecionados no QGroundControl via MAVLink Inspector. O yaw é em radianos. Para visualizar em RViz, use `Fixed Frame: camera_init` e adicione `/fast_lio/cloud_registered` como PointCloud2 e `/fast_lio/odometry` como Odometry.

Caso o executável não esteja instalado nesta máquina, prepare-o uma vez, com acesso à internet e as dependências ROS/PCL do ambiente atual:

```bash
cd ~/ardupilot/Rover
/usr/bin/python3.8 python/slam/fast_lio/install_fast_lio.py
```

O instalador obtém uma versão fixada do código oficial e do ikd-Tree, verifica os arquivos e compila em `python/.deps/fast_lio_ws/`. Usa o driver Livox já existente em `~/scanner_ws/devel`; não instala pacotes globalmente. Para testar apenas o processamento ROS sem envio ao MAVProxy, acrescente `--no-mavlink`. Use `--help` para selecionar outro namespace/tópico.

## 4. Usar a pose no Lua para um futuro docking

Outro script Lua do mesmo ArduPilot pode consultar a função global:

```lua
local pose, idade_ms = get_lidar_odometry()
if pose then
    local x, y, z = pose.x, pose.y, pose.z
    local qw, qx, qy, qz = pose.qw, pose.qx, pose.qy, pose.qz
    local sessao = pose.session
    -- Acrescente aqui a lógica futura de docking.
end
```

`get_lidar_odometry()` retorna `nil` quando ainda não há pose ou a última chegou há mais de 500 ms; `get_lidar_odometry(idade_maxima_ms)` permite outro limite. A tabela inclui `sequence`, `session`, `time_us` e `reset`. `LIDAR_ODOM` também contém a última pose recente ou `nil`. A sessão muda quando a ponte é reiniciada: descarte referências antigas se ela mudar. `time_us` é tempo ROS, enquanto a idade é medida pelo relógio do ArduPilot; não compare relógios sem sincronização.

A posição está em metros, num referencial **local FRD**: X para a frente, Y para a direita e Z para baixo, considerando a orientação inicial do FAST-LIO. O quaternion representa orientação do corpo FRD em relação a esse referencial local. Não são latitude/longitude nem necessariamente norte/leste. A origem e o referencial podem mudar ao reiniciar o FAST-LIO. Para docking, também será necessário obter a pose do alvo no mesmo referencial, verificar a qualidade e tratar perda de dados e reinicializações. Mensagens recentes não garantem precisão.

A ponte usa `DATA64` por MAVProxy; o Lua consome a fila MAVLink e oferece a função acima para compartilhamento. Outros scripts devem usar essa função em vez de consumir a mesma fila independentemente. Os valores não são automaticamente inseridos em `ahrs:get_location()` ou no EKF.

## 5. Mapa 2D, parada e limites

O FAST-LIO gera odometria e nuvem 3D. Para o mapa 2D de navegação, em outro terminal, mantenha o SLAM existente:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
cd ~/ardupilot/Rover
/usr/bin/python3.8 python/slam/slam_mid360_2d.py
```

Os mapas e referências do Hector e do FAST-LIO são independentes; não suponha que as coordenadas sejam iguais. Câmera e leitura direta do LiDAR estão documentadas em [Programas Python](../../README.md). Para encerrar, use Ctrl+C primeiro no FAST-LIO, depois no SITL/MAVProxy e por último no roslaunch.

O plugin do MID360 simulado entrega todos os pontos de cada nuvem no mesmo instante, sem tempo individual de aquisição. A ponte representa essa condição sem inventar tempos distribuídos; portanto, a simulação não reproduz a distorção de uma varredura real. O extrínseco fixo da montagem simulada é x=-0,003, y=0, z=0,434 m em relação à base. Para o MID360 real serão necessários o driver e a adaptação adequados ao formato e timestamps reais, calibração da IMU/extrínsecos e validação no veículo. O código atual é específico para os tópicos e a montagem simulados; não é uma solução pronta para conectar o sensor físico. O FAST-LIO pode derivar em áreas com pouca geometria e esta integração não valida precisão ou segurança para docking.
