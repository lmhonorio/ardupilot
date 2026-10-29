# Do computador ligado à simulação com MID360 e FAST-LIO

Este roteiro usa a instalação existente neste computador: ROS Noetic, Gazebo, pacote `rover_argo_gazebo`, ArduRover, MAVProxy e Python 3.8. O resultado é um rover no mundo `gravel_plane`, com câmera, MID360, mapa 2D opcional, FAST-LIO e a pose do LiDAR acessível aos scripts Lua. Os comandos são para terminais separados; mantenha cada um aberto enquanto utiliza a simulação.

O roteiro descreve a instalação já preparada em `/home/grin`; não é uma instalação do Ubuntu ou do ROS em um computador novo. Não é necessário iniciar `roscore` separadamente: o `roslaunch` inicia o ROS master quando necessário.

## 1. Ligar o computador e abrir um terminal

Entre na sessão gráfica do Ubuntu. Abra um terminal com **Ctrl+Alt+T**. Se houver Conda ativo, não é necessário removê-lo: os comandos de leitura ROS indicam explicitamente `/usr/bin/python3.8`. Não inicie outro Gazebo ou SITL para o mesmo rover ao mesmo tempo.

Para verificar os arquivos principais, execute:

```bash
ls ~/catkin_ws/devel/setup.bash
ls ~/scanner_ws/devel/share/livox_ros_driver/cmake/livox_ros_driverConfig.cmake
ls ~/ardupilot/Rover/python/.deps/fast_lio_ws/devel/lib/fast_lio/fastlio_mapping
```

Se o último arquivo não existir, prepare o FAST-LIO uma vez, com internet:

```bash
cd ~/ardupilot/Rover
/usr/bin/python3.8 python/slam/fast_lio/install_fast_lio.py
```

O instalador pressupõe as dependências ROS/PCL e o driver Livox já presentes. Se faltar o `setup.bash` ou o driver Livox, a instalação atual precisa ser restaurada antes de continuar.

## 2. Terminal 1 — abrir Gazebo e o rover

Copie e execute:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

roslaunch rover_argo_gazebo multi_rover_argo.launch \
  N:=1 world_name:=gravel_plane start_x:=0 start_y:=0 z:=0 \
  mid360_enabled:=true insta360_enabled:=true \
  samsung_s23_ultra_cam_enabled:=true hokuyo_enabled:=true
```

Aguarde a janela do Gazebo e o modelo do rover aparecerem. Deixe o terminal aberto. O arquivo do modelo MID360 em `~/catkin_ws/src/rover-argo-gazebo/rover_argo_gazebo/urdf/components/mid360.xacro` contém a IMU simulada necessária ao FAST-LIO; se o pacote for substituído, essa alteração deve ser preservada.

O mundo também contém uma estação de docking em formato de contêiner, sem
porta, com vão interno de 1 m de largura, 1 m de altura e 1 m de comprimento.
Seu centro está em `x=5, y=-2` metros no referencial do Gazebo. A entrada
fica em `x=4,5`, voltada para o lado de X menor; aproxime-se alinhado com
`y=-2`, olhando na direção de X crescente. O QR na parede interna do fundo
contém `ROVER_DOCKING_STATION_01`. O modelo tem colisões nas paredes, no
teto e no piso, e não executa docking automaticamente. Antes de entrar,
confira se as dimensões do rover e seus sensores permitem a passagem.

## 3. Terminal 2 — iniciar ArduRover, controle Lua e MAVProxy

Abra outro terminal. Execute:

```bash
cd ~/ardupilot/Rover
PYTHON38=/home/grin/.pyenv/versions/3.8.10/bin/python3.8 \
  ./sim_gz1rover.sh --foreground --no-rebuild
```

Aguarde o SITL e o prompt do MAVProxy iniciarem. Este comando usa a instalação Python 3.8 com MAVProxy, prepara os scripts Lua e os parâmetros da simulação ROS, e encaminha MAVLink para o QGroundControl e para o FAST-LIO. `--no-rebuild` pressupõe que o binário SITL já esteja compilado. Se estiver ausente, repita sem `--no-rebuild` para compilá-lo.

Se aparecer aviso de SITL na porta 5760, encerre o SITL anterior no terminal em que foi iniciado e repita. Deixe este terminal aberto. Não é necessário armar para verificar os sensores e a estimativa de posição.

## 4. Terminal 3 — iniciar FAST-LIO

Abra outro terminal e execute:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
cd ~/ardupilot/Rover

/usr/bin/python3.8 python/slam/fast_lio/run_fast_lio.py
```

Mantenha o rover parado durante a inicialização da IMU. A cada alguns segundos o programa exibe contadores semelhantes a:

```text
nuvens=50 imu=500 poses=40 enviadas=40 descartadas=0 pose_FRD=[...]
```

Os números variam. `nuvens` e `imu` devem aumentar; depois `poses` e `enviadas` também. No terminal do MAVProxy, procure `FAST-LIO: Lua pose available`. Se os valores de `enviadas` continuarem em zero, verifique se o Terminal 2 está ativo. Se aparecer `FAST-LIO: Lua pose stale`, a pose deixou de chegar ao Lua e deve ser verificada antes de utilizá-la.

## 5. Acompanhar no QGroundControl

Abra o QGroundControl pelo menu do Ubuntu ou pelo atalho que você normalmente utiliza. Ele deve receber a telemetria local pela porta UDP **14550**, enviada pelo MAVProxy. Aguarde a conexão com o veículo. Caso não conecte, confira no QGroundControl se há uma conexão UDP local na porta 14550 e se o Terminal 2 está ativo; evite outra instância que esteja usando a mesma porta.

O QGroundControl mostra a telemetria do ArduRover. Os valores `LIO_X`, `LIO_Y`, `LIO_Z` e `LIO_YAW` também podem ser procurados no MAVLink Inspector como `NAMED_VALUE_FLOAT`; o yaw é em radianos. A estimativa FAST-LIO transmitida ao Lua não substitui automaticamente a posição do EKF exibida pelo QGroundControl.

## 6. Conferir sensores e posição

Abra um quarto terminal para os comandos de diagnóstico. Antes de cada grupo de comandos ROS nesse terminal:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
cd ~/ardupilot/Rover
```

Para verificar as taxas do LiDAR e da IMU, execute um comando por vez e encerre cada um com Ctrl+C:

```bash
rostopic hz /rover_argo_N1/Instance1/livox/lidar
rostopic hz /rover_argo_N1/Instance1/livox/imu
```

Para ler uma posição estimada em formato ROS:

```bash
rostopic echo -n 1 /fast_lio/odometry
```

Se os tópicos não aparecerem, volte ao Terminal 1 e confira se o roslaunch iniciou o MID360. Se a odometria não aparecer, veja os erros no Terminal 3 e o arquivo `fast_lio.log` na pasta de execução informada por ele. Um ROS master recém-iniciado pode precisar de alguns segundos para registrar os tópicos.

A pose em `/fast_lio/odometry` usa eixos ROS e a origem da IMU. A pose encaminhada ao Lua é convertida para o referencial local da base.

## 7. Ver câmera e LiDAR, opcional

No quarto terminal já configurado, execute um dos comandos:

```bash
/usr/bin/python3.8 python/camera/read_ros_camera.py
```

```bash
/usr/bin/python3.8 python/lidar/read_ros_lidar.py
```

As janelas precisam de uma sessão gráfica. Consulte as opções e outras câmeras no [manual dos programas Python](python/README.md).

## 8. Gerar mapa 2D para navegação, opcional

Em outro terminal, deixando os anteriores abertos:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
cd ~/ardupilot/Rover
/usr/bin/python3.8 python/slam/slam_mid360_2d.py
```

Esse programa usa Hector Mapping para criar um mapa 2D. Na janela, **S** salva o mapa; **Q**, fechar a janela ou Ctrl+C salva e encerra. O FAST-LIO gera odometria e nuvem 3D; os referenciais e mapas dos dois programas são independentes. Movimente o rover pelo controle habitual somente quando estiver pronto para mapear.

## 9. Usar a posição no Lua

Um script Lua no mesmo ArduPilot pode consultar:

```lua
local pose, idade_ms = get_lidar_odometry()
if pose then
    local x, y, z = pose.x, pose.y, pose.z
    local qw, qx, qy, qz = pose.qw, pose.qx, pose.qy, pose.qz
    local sessao = pose.session
    -- Implementar aqui a lógica futura de docking.
end
```

A função retorna `nil` se não houver pose recente; o padrão considera dados com mais de 500 ms antigos. As posições estão em metros no referencial local FRD, com X para frente, Y para a direita e Z para baixo, definidos pela orientação inicial. A sessão muda quando a ponte reinicia; a origem também pode mudar. O código não implementa docking, não usa esses dados como posição do EKF e não garante precisão. Para mais detalhes consulte o [manual FAST-LIO e Lua](python/slam/fast_lio/README.pt-BR.md).

## 10. Encerrar tudo

1. Encerre as janelas e o SLAM 2D, se estiverem abertos.
2. No Terminal 3, pressione **Ctrl+C** para encerrar o FAST-LIO.
3. No Terminal 2, pressione **Ctrl+C** para encerrar SITL e MAVProxy.
4. No Terminal 1, pressione **Ctrl+C** e aguarde o roslaunch fechar o Gazebo.
5. Feche o QGroundControl quando desejar.

## Limites da validação

O caminho MID360 e IMU simulados → FAST-LIO → MAVProxy → Lua foi testado com o rover parado. Ainda não foi validada a precisão em movimento ou durante docking. O MID360 simulado entrega os pontos de uma nuvem no mesmo instante; ele não reproduz todos os efeitos temporais do sensor real. Para um MID360 físico serão necessárias adaptação ao driver, calibração e testes no veículo.

Outros roteiros, incluindo simulação sem Gazebo e autotuning, estão no [manual geral de simulação](MANUAL_SIMULACAO.md). O [diagnóstico ROS Argo](DIAGNOSTICO_ROS_ARGO.md) registra detalhes do ambiente.
