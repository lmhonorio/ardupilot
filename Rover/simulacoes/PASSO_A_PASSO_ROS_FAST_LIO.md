# Do computador ligado à simulação com MID360 e FAST-LIO

Este roteiro usa a instalação existente neste computador: ROS Noetic, Gazebo, pacote `rover_argo_gazebo`, ArduRover, MAVProxy e Python 3.8. O resultado é um rover no mundo `gravel_plane`, com câmera, MID360, mapa 2D opcional, FAST-LIO e a pose do LiDAR acessível aos scripts Lua. Os comandos são para terminais separados; mantenha cada um aberto enquanto utiliza a simulação.

O roteiro descreve a instalação já preparada em `/home/grin`; não é uma instalação do Ubuntu ou do ROS em um computador novo. Não é necessário iniciar `roscore` separadamente: o `roslaunch` inicia o ROS master quando necessário.

Para observar uma missão existente e entrar no contêiner, consulte [Espera automática de docking](docking/README%20DOCKING.md#espera-automática-para-docking-em-guided). A sequência completa ainda precisa ser validada em movimento.

Para iniciar cada serviço manualmente, em seu próprio terminal, com ou sem desvio, consulte o [tutorial de serviços separados](TUTORIAIS_SERVICOS_SEPARADOS.md).

## Inicialização completa em janelas separadas

Depois de ligar o computador e entrar na sessão gráfica, abra um terminal com **Ctrl+Alt+T** e execute:

```bash
cd ~/ardupilot/Rover
./iniciar_simulacao_completa.sh
```

O lançador abre o Gazebo e aguarda os tópicos do MID360 e da IMU; em seguida abre ArduRover/MAVProxy, FAST-LIO, percepção de obstáculos, mapa 2D, câmera e RViz. Também abre o QGroundControl, ou usa a janela já aberta. Os terminais mostram mensagens e erros de cada componente; acompanhe-os até a inicialização terminar. O rover não é armado e nenhuma missão é iniciada automaticamente.

O lançador exige as dependências já instaladas e o binário SITL compilado, conforme a seção 1 abaixo. Ele se recusa a iniciar se já houver Gazebo, SITL ou `sim_vehicle.py` em execução; encerre a simulação anterior no terminal dela e execute o comando novamente. Se uma etapa expirar, confira a respectiva janela e encerre manualmente os componentes que já foram abertos.

Para abrir menos janelas, combine as opções conforme necessário:

```bash
./iniciar_simulacao_completa.sh --sem-camera --sem-mapa --sem-rviz --sem-qgc
```

No terminal do FAST-LIO, verifique se `nuvens`, `imu`, `poses` e `enviadas` aumentam. Na janela **Percepção de obstáculos**, procure mensagens da ponte. Você também pode consultar em outro terminal:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rostopic echo /olhe/obstacles/status
```

Em outro terminal, consulte `rostopic echo /olhe/obstacles/bridge_status`. `ENVIADO` indica envio ao MAVProxy; `COBERTURA_PARCIAL` indica setores sem retorno. O módulo utiliza a saída UDP **14557** do MAVProxy, enquanto o QGroundControl usa 14550, o MAVROS 14555 e o FAST-LIO 14556.

**Desvio experimental habilitado:** o lançador inicia um perfil SITL separado com BendyRuler (`OA_TYPE=1`, `PRX1_TYPE=2`). A ponte envia setores observados, mas setores sem retorno permanecem desconhecidos e o ArduPilot os ignora. Não há parada automática validada por perda de percepção, e ainda não foi comprovada uma missão com desvio. Supervisione os testes no Gazebo. Veja o [manual e os resultados](obstacle_avoidance/README.md).

Para encerrar, pressione **Ctrl+C** na janela de percepção, depois nas janelas do FAST-LIO, mapa e câmera, depois no SITL/MAVProxy e por último no Gazebo. Feche RViz e QGroundControl pelas próprias janelas. A janela de percepção encerra somente o módulo que ela iniciou. Também é possível encerrá-lo com `./start_obstacle_avoidance.sh --stop` em `~/ardupilot/Rover`.

As seções seguintes mostram como iniciar e diagnosticar os componentes manualmente. Não as execute em paralelo com o lançador completo.

### Alternativa: iniciar com `sim_1_gzrover.sh`

Primeiro inicie o mundo ROS conforme a seção 2. Em outro terminal:

```bash
cd ~/ardupilot/Rover
./sim_1_gzrover.sh
```

Esse lançador usa diretamente `Rover/nroverobst.param` e a EEPROM do diretório `Rover/`, abre o SITL/MAVProxy e, quando encontra o tópico do MID360, abre a janela **Percepção de obstáculos MID360**. Se a ponte já estiver ativa, reutiliza-a. O FAST-LIO, o mapa e a câmera ainda devem ser abertos separadamente conforme as seções seguintes.

Para escolher outro arquivo, use `./sim_1_gzrover.sh --add-param-file nrover_sem_desvio.param`. Para executar o SITL no terminal atual e iniciar a ponte separadamente, acrescente `--foreground`. Confira sempre os valores gravados conforme o tutorial de serviços separados; sem `--foreground`, a janela de percepção continua sendo iniciada automaticamente mesmo ao escolher o arquivo sem desvio.

Valores já gravados na EEPROM podem prevalecer sobre o arquivo de parâmetros. No console do MAVProxy, confirme `param show BATT_SOC_ALLOC`, `param show WP_SPEED`, `param show OA_TYPE` e `param show PRX1_TYPE`. A alocação Lua depende de `BATT_SOC_ALLOC=1`; este lançador não altera esse parâmetro automaticamente. A missão anterior permanece na EEPROM existente.

O perfil `nroverobst.param` usa `AVOID_ENABLE=1`: a cerca permanece como
entrada do desvio simples, enquanto a proximidade é tratada pelo BendyRuler.
Com `AVOID_ENABLE=3` e margem de 2 m, observou-se que o desvio simples podia
manter velocidade e giro em zero. Ao alterar parâmetros já gravados, confira
também `param show AVOID_ENABLE` e `param show OA_TYPE` no MAVProxy.

Se aparecer `PreArm: PRX1: No Data`, confira a janela de percepção e, em outro
terminal com o ambiente ROS configurado, execute:

```bash
rostopic echo /olhe/obstacles/bridge_status
```

`SEM_MAVPROXY` indica falta de mensagens do MAVProxy na porta 14557;
`SCAN_DESATUALIZADO` indica falta de nuvens recentes; `ENVIADO` indica que
a ponte está enviando dados, mas não comprova a capacidade de desvio. No
console do MAVProxy, confira `set mavfwd` e habilite o encaminhamento com
`set mavfwd true` se necessário. Não desabilite a verificação PreArm para
ocultar uma falha de percepção.

O lançador testa se os dois nós respondem antes de reutilizá-los; um nome
antigo ainda listado pelo ROS não é suficiente para considerar a ponte ativa.

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
contém `ROVER_DOCKING_STATION_01` e é o único QR no mundo `gravel_plane`.
O modelo tem colisões nas paredes, no
teto e no piso, e não executa docking automaticamente. Antes de entrar,
confira se as dimensões do rover e seus sensores permitem a passagem.

## 3. Terminal 2 — iniciar ArduRover, controle Lua e MAVProxy

Abra outro terminal. Execute:

```bash
cd ~/ardupilot/Rover
PYTHON38=/home/grin/.pyenv/versions/3.8.10/bin/python3.8 \
  ./sim_gz1rover.sh --foreground --no-rebuild --avoidance
```

Aguarde o SITL e o prompt do MAVProxy iniciarem. Este comando usa a instalação Python 3.8 com MAVProxy, prepara os scripts Lua e os parâmetros da simulação ROS, e encaminha MAVLink para o QGroundControl, FAST-LIO e percepção de obstáculos. `--no-rebuild` pressupõe que o binário SITL já esteja compilado. Se estiver ausente, repita sem `--no-rebuild` para compilá-lo.

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


### Conferir a configuração de desvio

No terminal do MAVProxy digite:

```text
param show OA_TYPE
param show PRX1_TYPE
param show OA_BR_LOOKAHEAD
param show OA_MARGIN_MAX
```

Os valores esperados são 1, 2, 5 e 1. O perfil fica em `gazebo_simple/run/ros_argo_avoid/`, com EEPROM separada do perfil sem desvio. Os parâmetros iniciais são aplicados a uma EEPROM nova; se tiverem sido alterados nessa EEPROM, configure-os no MAVProxy e reinicie o SITL, pois `OA_TYPE` e `PRX1_TYPE` exigem reinicialização. `ENVIADO` no tópico da ponte não prova que o rover desviará nem confirma a segurança do caminho.

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

Para a etapa experimental de entrada visual no contêiner após a aproximação,
consulte o [manual de docking em GUIDED](docking/README%20DOCKING.md). Ela usa o QR e
um programa Python separado; veja o resultado e os limites do ensaio no manual.

## 10. Encerrar tudo

1. Encerre as janelas e o SLAM 2D, se estiverem abertos.
2. No Terminal 3, pressione **Ctrl+C** para encerrar o FAST-LIO.
3. No Terminal 2, pressione **Ctrl+C** para encerrar SITL e MAVProxy.
4. No Terminal 1, pressione **Ctrl+C** e aguarde o roslaunch fechar o Gazebo.
5. Feche o QGroundControl quando desejar.

## Limites da validação

O caminho MID360 e IMU simulados → FAST-LIO → MAVProxy → Lua foi testado com o rover parado. Ainda não foi validada a precisão em movimento ou durante docking. O MID360 simulado entrega os pontos de uma nuvem no mesmo instante; ele não reproduz todos os efeitos temporais do sensor real. Para um MID360 físico serão necessárias adaptação ao driver, calibração e testes no veículo.

Outros roteiros, incluindo simulação sem Gazebo e autotuning, estão no [manual geral de simulação](MANUAL_SIMULACAO.md). O [diagnóstico ROS Argo](DIAGNOSTICO_ROS_ARGO.md) registra detalhes do ambiente.
