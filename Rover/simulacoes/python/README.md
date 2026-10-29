# Programas Python do Rover

Os fontes Python de `Rover` estão reunidos aqui, por função. Os arquivos de
`FuncoesPython/` não foram movidos. Os lançadores `.sh`, Lua, mundos, modelos
e resultados antigos continuam nos diretórios anteriores.

| Pasta | Função |
| --- | --- |
| `camera/` | Leitor das câmeras ROS Samsung e Insta360 |
| `lidar/` | Leitor MID360 ROS e visualizador dos NPZ do Gazebo sem ROS |
| `slam/` | Projeção do MID360, Hector Mapping 2D e exportação de mapas |
| `missions/` | Missões círculo/quadrado, coleta, análise e autotuning via MAVProxy |
| `simulation/` | Leitura Gazebo Transport, geração de QR e preparação Lua/SITL |
| `common/` | Conversão PointCloud/PointCloud2 e projeção geométrica compartilhada |
| `tests/` | Testes Python e validação ROS isolada |
| `run/` | Nuvens, mapas e evidências novos; ignorado pelo Git |
| `.deps/` | Hector local e seu pacote/licença; ignorado pelo Git |

## Ambiente

Para as ferramentas ROS desta máquina, use Python 3.8 do sistema:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
cd ~/ardupilot/Rover
```

Em outro terminal, inicie o cenário:

```bash
roslaunch rover_argo_gazebo multi_rover_argo.launch \
  N:=1 world_name:=gravel_plane start_x:=0 start_y:=0 z:=0 \
  mid360_enabled:=true insta360_enabled:=true \
  samsung_s23_ultra_cam_enabled:=true hokuyo_enabled:=true
```

Câmera, LiDAR e SLAM recebem dados dos sensores ROS. Eles não exigem MAVROS
ou SITL para iniciar a leitura. Para movimentar o rover, use seu fluxo
habitual de controle separadamente; os programas de leitura/SLAM não enviam
comandos de movimento ou armamento.

## Câmera

```bash
/usr/bin/python3.8 python/camera/read_ros_camera.py
/usr/bin/python3.8 python/camera/read_ros_camera.py --camera insta360
/usr/bin/python3.8 python/camera/read_ros_camera.py --list
/usr/bin/python3.8 python/camera/read_ros_camera.py --snapshot /tmp/camera.png
```

A Samsung é a seleção padrão. Na janela, **S** salva PNG; **Q/Esc** encerra.
Os PNG salvos por S continuam em `gazebo_simple/run/camera/`.

## Ler e visualizar o MID360

```bash
/usr/bin/python3.8 python/lidar/read_ros_lidar.py
/usr/bin/python3.8 python/lidar/read_ros_lidar.py --list
/usr/bin/python3.8 python/lidar/read_ros_lidar.py --headless
/usr/bin/python3.8 python/lidar/read_ros_lidar.py --snapshot python/run/lidar/frame.npz
/usr/bin/python3.8 python/lidar/read_ros_lidar.py --save python/run/lidar/frame.png
```

O tópico padrão é `/rover_argo_N1/Instance1/livox/lidar`. O plugin instalado
publica `sensor_msgs/PointCloud`; o leitor aceita também `PointCloud2` com
campos x/y/z. Zeros de retornos inválidos e valores não finitos são removidos.
As coordenadas exibidas são locais ao sensor, em metros.
O leitor exibe/salva retornos entre 0,1 e 40 m por padrão; ajuste
`--min-range` e `--max-range` para inspecionar outra faixa.
Z é ampliado visualmente três vezes, indicado no eixo; `--z-scale 1`
usa proporções físicas. Os valores numéricos continuam em metros reais.

Use `--namespace /rover_argo_N1/Instance2` para outro rover, ou `--topic`
para selecionar explicitamente uma nuvem. O timeout padrão é de 30 s;
`--timeout 0` aguarda indefinidamente. Fechar a janela ou Ctrl+C encerra.

O Xacro do MID360 no pacote ROS foi ajustado com `always_on=true`, para
produzir varreduras também com Gazebo sem GUI. Reinicie o roslaunch após
essa alteração. Não é necessário recompilar o modelo.

## SLAM 2D com o MID360

Com o mundo aberto:

```bash
/usr/bin/python3.8 python/slam/slam_mid360_2d.py
```

Abre uma janela com o mapa e a posição estimada. Movimente o rover pelo
controle habitual para observar outras regiões. **S** salva o mapa;
**Q**, fechar a janela ou **Ctrl+C** salva e encerra o SLAM. O Gazebo continua
aberto. Para executar sem janela:

```bash
/usr/bin/python3.8 python/slam/slam_mid360_2d.py --headless
```

O programa imprime a pasta de saída, por padrão
`python/run/slam/AAAAMMDD_HHMMSS_.../`. Ela contém:

| Arquivo | Conteúdo |
| --- | --- |
| `map.pgm` | Ocupação: livre claro, ocupado preto, desconhecido cinza |
| `map.yaml` | Resolução e origem para ROS `map_server` |
| `trajectory.csv` | Poses estimadas pelo SLAM, com timestamp |
| `status.json` | Tópicos, frames, parâmetros e contagem de nuvens/scans |
| `hector.log` | Diagnóstico do motor de SLAM |

Os tópicos padrão são `/rover_slam/map`, `/rover_slam/scan`,
`/rover_slam/slam_out_pose` e `/rover_slam/scanmatch_odom`. O frame do mapa
é `rover_slam_map`. Em RViz, selecione esse **Fixed Frame** e adicione Map
com tópico `/rover_slam/map` e LaserScan com `/rover_slam/scan`.

O mapa usa coordenadas locais do SLAM, sem georreferenciamento automático.
Os frames `rover_slam_base` e `rover_slam_lidar` são separados dos frames
legados do Gazebo. O Hector estima `map -> base` diretamente, sem odometria
externa neste modo. O `.pgm/.yaml` pode alimentar uma configuração de
navegação ROS; este programa não inicia `move_base` nem evita obstáculos
por conta própria.

### Método e parâmetros

A nuvem é filtrada por altura no referencial do LiDAR. O padrão acumula
cinco nuvens consecutivas (`--accumulate 5`), cerca de 0,4 s entre a mais
antiga e a mais recente a 10 Hz, para estabilizar a cobertura não repetitiva.
Na partida, aguarde essa janela inicial antes de mover. Cada setor angular
recebe o retorno horizontal mais próximo. Setores sem observações ficam
infinitos e são ignorados pelo Hector; não são convertidos artificialmente
em espaço livre. A estimativa usa alinhamento das varreduras, sem consumir
`/gazebo/model_states`, GPS ou a pose real do simulador.

```bash
/usr/bin/python3.8 python/slam/slam_mid360_2d.py \
  --min-height -0.2 --max-height 0.8 \
  --min-range 0.35 --max-range 30 \
  --resolution 0.1 --map-size 1024
```

Esses são os padrões: 10 cm/célula, mapa de 102,4 × 102,4 m e 360 setores.
A faixa de altura exclui o piso plano e inclui obstáculos próximos da
altura do rover. O extrínseco padrão é x=−0,003, y=0, z=0,434 m em relação
ao base_link, combinando a posição da montagem e do sensor no Xacro atual.
Para outra montagem, ajuste `--lidar-x/y/z`. O programa presume eixos
alinhados e movimento aproximadamente plano; não implementa compensação
de inclinação ou de distorção por movimento dentro da nuvem.
Também não há compensação de movimento entre as nuvens acumuladas;
reduza a velocidade e as curvas bruscas durante o mapeamento. Aumentar
`--accumulate` aumenta cobertura e também o borramento durante movimento.
`--accumulate 1` usa somente o quadro atual, mas foi insuficiente na
validação deste cenário esparso: a pose derivou mesmo com o rover parado.

Como o MID360 tem cobertura variável, scans com menos de 10 setores válidos
são descartados. É possível ajustar `--min-hits`, a faixa de altura e os
limites de alcance conforme a cena. Regiões sem estrutura geométrica podem
não restringir a pose. Este modo não inclui fechamento explícito de laços;
o mapa pode acumular deriva. Movimentos lentos ajudam, especialmente nas
curvas. Reinicie o SLAM após resetar o relógio/mundo Gazebo.

O motor é [Hector Mapping](https://github.com/tu-darmstadt-ros-pkg/hector_slam/tree/noetic-devel/hector_mapping).
O processamento de LaserScan e a estimativa sem pose externa seguem o
[código oficial](https://github.com/tu-darmstadt-ros-pkg/hector_slam/blob/noetic-devel/hector_mapping/src/HectorMappingRos.cpp).

## FAST-LIO e posição para o Lua

A odometria LiDAR/IMU e sua transferência pelo MAVProxy para o Lua estão
documentadas no [manual do FAST-LIO](slam/fast_lio/README.pt-BR.md). O
FAST-LIO e o mapa 2D do Hector são processos e referenciais independentes.

### Dependência do SLAM 2D

Nesta máquina, o Hector Noetic 0.5.2 foi obtido do repositório ROS e extraído
localmente em `.deps/hector/`, sem instalação global ou senha sudo. Suas
dependências dinâmicas foram verificadas. Para conferir ou preparar outra cópia:

```bash
/usr/bin/python3.8 python/slam/install_backend.py --check
/usr/bin/python3.8 python/slam/install_backend.py
```

O instalador usa `apt-get download` e `dpkg-deb` no Ubuntu/ROS Noetic.
Também reconhece o Hector instalado normalmente em `/opt/ros/noetic`.
Não é necessário ativar o workspace de SLAM 3D em `scanner_ws`.

## Missões e ferramentas anteriores

As chamadas dos lançadores continuam na pasta Rover:

```bash
./mission_tune.sh --radius 3
./sim_gz1rover.sh --foreground --no-rebuild
./start_gz_simple.sh --lua --no-rebuild
```

Os fontes de missão estão em `python/missions/`, mas os resultados continuam
em `mission_tune/runs/`. EEPROM, logs e capturas Gazebo anteriores também
mantêm seus caminhos. Para as ferramentas Gazebo sem ROS:

```bash
python3.8 python/lidar/view_lidar.py
python3.8 python/simulation/generate_qr.py 'ROVER_PAREDE_VERDE_01'
python3.8 python/missions/plot_report.py mission_tune/runs/PASTA
```

## Verificação

Na raiz `~/ardupilot`:

```bash
/usr/bin/python3.8 -m unittest discover -s Rover/python/tests -v
python3.8 -m unittest discover -s Rover/python/missions/tests -v
/usr/bin/python3.8 Rover/python/tests/validate_ros_slam.py
```

O último comando cria um ROS master separado em 11321, publica uma nuvem
sintética de um ambiente conhecido com movimento e confere estimativa,
exportação do mapa e leitura PointCloud2. Não move o rover. Evidências em
`python/run/validation/`. O teste sintético não comprova precisão em uma
missão completa no Gazebo ou no rover real.

O MID360 real do Gazebo também foi conferido com o rover parado: leitura,
janela e mapa funcionaram, com variação de pose ainda da ordem de dezenas
de centímetros. Veja os [resultados e limites dos ensaios](VALIDATION.md).
