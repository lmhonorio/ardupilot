# ROS Noetic + Argo no Gazebo — 28/10/2026

## Chamada

As correções foram feitas na cópia que o ROS encontra nesta máquina:
`/home/grin/catkin_ws/src/rover-argo-gazebo`. As outras cópias em
`/home/grin/rover-argo-gazebo` e `/home/grin/ros-rover` não foram alteradas.

```bash
source /opt/ros/noetic/setup.bash
source /home/grin/catkin_ws/devel/setup.bash
rospack find rover_argo_gazebo
roslaunch rover_argo_gazebo multi_rover_argo.launch N:=3 world_name:=parnaibaiii_simple_v3
```

O `rospack find` deve devolver
`/home/grin/catkin_ws/src/rover-argo-gazebo/rover_argo_gazebo`.
Não é necessário recompilar para aplicar as mudanças em `.launch`, Python
ou SDF. O plugin de coordenadas corrigido já foi recompilado nesta máquina.
Encerre com Ctrl+C no terminal do roslaunch.

O launch usa `/usr/bin/python3`, que nesta instalação é **Python 3.8.10**,
e dá prioridade a `/usr/bin` no PATH dos processos filhos. Isso permite
executar a chamada mesmo com o ambiente `(base)` ativo, sem instalar ROS
no Python 3.13 do Conda nem alterar seu ambiente global.

Para executar sem a janela do Gazebo, acrescente `gui:=false`.

## Por que não apareciam os modelos

1. **Diretório dos modelos ausente.** O Gazebo registrou
   `Unable to find model[ARGO_PARNAIBAIII_V3]`. A pasta e os arquivos existem,
   mas o `GAZEBO_MODEL_PATH` da sessão continha caminhos de exemplo e caminhos
   para outras cópias do repositório. Na `.bashrc`, parte das referências a
   `$ROS_WS` aparecia antes da definição dessa variável. O launch agora
   acrescenta `$(find rover_argo_gazebo)/models` ao ambiente dos seus processos.
2. **Python incorreto no spawner.** O shebang `/usr/bin/env python3` escolhia
   Python 3.13.5 do Conda; a importação de ROS falhava com
   `ModuleNotFoundError: No module named 'rospkg'`. O interpretador e o PATH
   agora são selecionados no launch, incluindo as ferramentas ROS filhas.
3. **MAVROS repetido.** Um laço interno tentava iniciar todos os MAVROS para
   cada rover. Foi mantido um único gerenciador e uma inicialização por
   instância, evitando colisões de nomes e portas.
4. **Plugin geográfico com nome e ligação incorretos.** O SDF de V3 pedia
   `libgeo_coordinates_plugin.so`, mas o CMake gerava
   `libGeoCoordinatesPlugin.so`. Foi corrigido o nome no SDF e adicionada a
   ligação com GeographicLib ao alvo realmente compilado. A biblioteca foi
   recompilada e conferida com `ldd -r`, sem símbolos ausentes.

Depois de corrigir o carregamento, o teste revelou que os rovers nas posições
antigas rolavam para fora do terreno. O nascimento padrão era próximo da
borda inclinada, em `(5,-129)`, `(8,-126)` e `(11,-123)`. Sem SITL mantendo
o movimento, eles desciam e caíam fora da malha.

Somente para `parnaibaiii_simple_v3`, os novos padrões são `start_x=5`,
`start_y=-100`, `z=3.5`, com `dx=dy=3`. O piso nessa região está próximo de
Z=2,7 m; os rovers assentam com o referencial do modelo perto de Z=2,85 m.
É possível sobrescrever esses valores na chamada, mas Z deve ficar acima
do terreno e o ponto escolhido precisa ter colisão de piso.

## Dependências verificadas

| Componente | Situação nesta máquina |
| --- | --- |
| Gazebo Classic | 11.15.1 instalado |
| ROS | Noetic instalado |
| Python para ROS | `/usr/bin/python3`, versão 3.8.10; imports ROS funcionando |
| gazebo_ros, gazebo_plugins, gazebo_ros_control | Instalados |
| xacro, joint_state_publisher, robot_state_publisher | Instalados |
| MAVROS | 1.20.1 instalado, incluindo dados GeographicLib `egm96-5` |
| hector_gazebo_plugins, controller_manager, pointcloud_to_laserscan | Instalados |
| cv_bridge, camera_info_manager, image_transport | Instalados |
| Livox | Pacote e `liblivox_laser_simulation.so` presentes |
| ArduPilot para Gazebo | `libArduPilotPlugin.so` presente |
| Plugins de câmera Samsung e pan/tilt | Bibliotecas presentes |
| Malhas do modelo ARGO_PARNAIBAIII_V3 | URIs locais verificadas; nenhum arquivo referenciado ausente |

O manifesto declarava `rover_argo_control`, pacote inexistente e sem uso nos
arquivos desse repositório. Essa declaração foi removida; foram declaradas
as dependências de execução efetivamente usadas pelo launch/spawner.
Após a correção, `rospack depends rover_argo_gazebo` terminou sem erro.
Nenhum pacote foi instalado via apt ou pip durante este diagnóstico.

A recompilação inicial também encontrou entradas do Conda no cache CMake
de Protobuf/GTest. O cache foi reconfigurado com PATH do sistema, Python 3.8
e remoção das entradas correspondentes do Conda; a compilação do alvo
`GeoCoordinatesPlugin` terminou com sucesso.

## Escopo da execução

O roslaunch cria o mundo, os modelos, publicadores ROS e um MAVROS por rover.
Ele **não inicia as três instâncias de ArduRover/SITL**. Assim, um MAVROS
iniciado pode continuar sem conexão com FCU enquanto o SITL não estiver
enviando dados. Isso não impede os modelos de aparecerem no Gazebo.

| Rover | Namespace MAVROS | UDP MAVROS | Entrada FDM do plugin |
| --- | --- | --- | --- |
| rover_argo_N1 | `/rover_argo_N1/Instance1/mavros` | 14555 | 9002 |
| rover_argo_N2 | `/rover_argo_N1/Instance2/mavros` | 14565 | 9012 |
| rover_argo_N3 | `/rover_argo_N1/Instance3/mavros` | 14575 | 9022 |

Os sensores opcionais continuam desabilitados na chamada original.
Este teste não executou missões, não armou rovers e não validou o controle
de movimento nem os fluxos dos sensores opcionais.

Na interface, expanda **World → Models** para localizar os rovers.
A câmera inicial do mundo mostra outra região da subestação; os rovers
nascem perto de Y=−100 m.

## Ler as câmeras em Python 3.8

Com o roslaunch e os sensores habilitados, abra outro terminal:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
cd ~/ardupilot/Rover
/usr/bin/python3.8 python/camera/read_ros_camera.py
```

O programa abre a Samsung por padrão. Usa `rospy`, `sensor_msgs/Image`,
`cv_bridge` e OpenCV, já disponíveis no Python 3.8 do sistema nesta máquina.
Ele não depende do SITL ou de MAVROS para receber a imagem ROS.

| Seleção | Tópico no cenário de um rover |
| --- | --- |
| `--camera samsung` (padrão) | `/rover_argo_N1/Instance1/camera/image_raw` |
| `--camera insta360` | `/rover_argo_N1/Instance1/camera/insta360/image_raw` |

Na janela, **S** salva um PNG, **Q** ou **Esc** encerra. Ctrl+C também encerra.
As imagens salvas por S ficam em `gazebo_simple/run/camera/`, ou no diretório
escolhido por `--save-dir`. O programa converte as imagens para BGR antes de
mostrar ou salvar e mantém somente o frame mais recente.

```bash
# Selecionar a Insta360:
/usr/bin/python3.8 python/camera/read_ros_camera.py --camera insta360

# Descobrir os tópicos sensor_msgs/Image anunciados:
/usr/bin/python3.8 python/camera/read_ros_camera.py --list

# Receber um frame e salvar sem janela, encerrando depois:
/usr/bin/python3.8 python/camera/read_ros_camera.py --snapshot /tmp/rover_camera.png

# Receber continuamente sem janela:
/usr/bin/python3.8 python/camera/read_ros_camera.py --headless

# Outro namespace ou um tópico explícito:
/usr/bin/python3.8 python/camera/read_ros_camera.py --camera samsung --namespace /rover_argo_N1/Instance2
/usr/bin/python3.8 python/camera/read_ros_camera.py --topic /rover_argo_N1/Instance1/camera/image_raw
```

Por padrão, 30 segundos sem frames encerram o programa com uma mensagem
de diagnóstico. `--timeout 0` aguarda indefinidamente. Com o Gazebo pausado,
o sensor pode deixar de publicar. O tópico deve ter tipo `sensor_msgs/Image`;
os tópicos `/compressed` não são aceitos por este leitor.

A recepção das duas seleções, dimensões, conversão RGB/BGR, salvamento PNG,
listagem e timeout foram verificados com publicadores ROS de imagens de
cores conhecidas em master separado. O teste não iniciou o Gazebo; ele
valida o leitor, não o funcionamento dos plugins de câmera. Evidências em
`gazebo_simple/run/ros_camera_check/validation.json`.

## Muro com QR code e obstáculos no gravel_plane

O mundo ROS `rover_argo_gazebo/worlds/gravel_plane.world` recebeu os dez
obstáculos de `Rover/gazebo_simple/rover_simple.world`, mantendo dimensões,
cores, colisões e posições. O chão de cascalho e o rover continuam sendo
os modelos desse cenário ROS.

| Elemento | Primeiro conjunto X/Y (m) | Segundo conjunto X/Y (m) |
| --- | --- | --- |
| Caixa frontal vermelha | 10 / 0 | 10 / 24 |
| Caixa esquerda azul | 6 / 6 | 6 / 30 |
| Cilindro amarelo | 8 / −6 | 8 / 18 |
| Muro verde | 20 / 0 | 20 / 24 |
| Caixa traseira roxa | −8 / −4 | −8 / 20 |

O primeiro muro tem QR de 1,8 × 1,8 m, em X=19,845, Y=2 e Z=0,95 m,
voltado para −X, em direção à região da origem. O conteúdo é
`ROVER_PAREDE_VERDE_01`. O painel é visual; a colisão é a do muro atrás dele.
O modelo `wall_qr`, material e textura foram copiados para
`~/catkin_ws/src/rover-argo-gazebo/rover_argo_gazebo/models/wall_qr`,
sem depender do diretório de modelos do ArduPilot para encontrar o QR.

Reinicie o mesmo roslaunch com `world_name:=gravel_plane` para carregar
os elementos. Não é necessário recompilar. No Gazebo, os nomes são
`wall`, `wall_2`, `wall_qr`, `box_front`, `box_left`, `pillar_right`,
`box_back` e as versões com sufixo `_2`.

O SDF foi validado e o mundo foi carregado em Gazebo/ROS separado. Foram
conferidos os dez obstáculos estáticos com colisões, o painel QR e o rover.
A textura PNG foi decodificada e retornou o conteúdo indicado acima.
Registros em `gazebo_simple/run/ros_obstacles_check/`. O teste não executou
missão nem movimento do rover.

## Evidências

Foi verificado o carregamento gráfico da subestação e o serviço Gazebo
retornou `ARGO_PARNAIBAIII_V3` e os três rovers. O teste final conferiu
também a estabilidade das posições após pelo menos 30 segundos simulados,
com três nós MAVROS e as portas correspondentes abertas.

Os ensaios usaram ROS master `11321` e Gazebo master `11355` para isolar
o diagnóstico. A chamada normal acima continua usando as portas padrão.
Os processos do diagnóstico foram encerrados ao final.

Registros e cópias anteriores às mudanças ficam em
`gazebo_simple/run/ros_argo_check/`, ignorado pelo Git:

- `validation.json`: modelos, posições e nós conferidos no teste final.
- `launch.log`: inicialização final.
- `build_clean.log`: configuração e compilação com bibliotecas do sistema.
- `gazebo.png`: captura da janela com a subestação carregada.
- `*.before`: cópias dos fontes e biblioteca anteriores às alterações.

Permanecem mensagens do mundo sobre parâmetros redundantes de gravidade e
campo magnético dentro de `<physics>`, além de avisos de diretórios antigos
na aba Insert. Eles não impediram o carregamento e a estabilidade verificados.

## Aviso PositionTargetGlobal e encerramento

No registro de 28/10, o ArduRover informou `EKF3 IMU0 origin set` e
`EKF3 IMU1 origin set`. O tlog, porém, continha zero `GPS_GLOBAL_ORIGIN`,
apesar de receber GPS com fix e `HOME_POSITION`. O aviso vinha de
`mavros_extras/guided_target`, que aguarda o tópico
`/rover_argo_N1/Instance1/mavros/global_position/gp_origin`.
São estados distintos: o EKF pode ter origem e o consumidor ROS ainda não
ter recebido a mensagem correspondente. Confira o
[código do guided_target 1.20.1](https://github.com/mavlink/mavros/blob/1.20.1/mavros_extras/src/plugins/guided_target.cpp).

O inicializador gerado por `prepare_ros_argo_lua.py` agora solicita
`gcs:set_message_interval(0, 49, 1000000)`: mensagem GPS_GLOBAL_ORIGIN a
1 Hz no link TCP/MAVProxy do SITL. O firmware só a envia depois que a origem
real do EKF existe; não é criada uma origem artificial nem são removidas
verificações de armamento. O MAVProxy encaminha a mensagem ao MAVROS.
Reinicie o SITL para carregar o inicializador atualizado. Com ROS e SITL
abertos e GPS/EKF prontos, confira:

```bash
rostopic echo -n 1 /rover_argo_N1/Instance1/mavros/global_position/gp_origin
```

A exceção `boost::lock_error` relatada apareceu após Ctrl+C. Isso sugere uma
falha de sincronização no encerramento, mas o log apresentado não identifica
qual plugin a causou. Ela não comprova um defeito no alocador Lua e não foi
corrigida por este ajuste da origem. O `done` indica que o roslaunch concluiu
sua sequência de encerramento, mesmo tendo escalado processos para SIGTERM.

Encerre o SITL/MAVProxy no terminal dele antes de fechar o roslaunch/Gazebo.
Fechar apenas o roslaunch deixa o SITL aguardando a física que foi encerrada.

## Programas Python organizados e SLAM 2D

Os fontes Python de Rover estao em [`python/`](python/README.md), separados
por funcionalidade. Camera: `python/camera/read_ros_camera.py`; LiDAR ROS:
`python/lidar/read_ros_lidar.py`; mapa 2D do MID360:
`python/slam/slam_mid360_2d.py`. As chamadas atualizadas e dependencias
estao no [manual Python](python/README.md).
