# GLIM com MID360 no Gazebo / ROS Noetic

O GLIM foi integrado como um mapeador 3D independente, usando LiDAR e IMU do rover. O perfil usa CPU, odometria GICP e otimização global por grafo de poses com detecção de fechamento de laços. Não precisa de CUDA, câmera ou FAST-LIO para funcionar.

O início completo com `--glim` mantém também FAST-LIO para a localização local já usada no Lua e no docking. O GLIM publica seu próprio mapa e sua própria odometria; não envia MAVLink nem comandos ao rover. Isso permite avaliar o novo mapeador sem misturar duas fontes de posição na mesma porta. A pose corrigida pelo fechamento de laços pode mudar descontinuamente e não deve ser enviada diretamente ao controle de docking.

## Instalação

```bash
cd ~/ardupilot/Rover
/usr/bin/python3.8 python/slam/glim/install_glim.py
```

Compila fontes oficiais fixadas em versões compatíveis: GLIM 1.1.0, gtsam_points 1.1.0, GTSAM 4.2a9, wrapper ROS 1, spdlog 1.10.0 e fmt 8.1.1. Fontes, ferramentas, build e instalação ficam em `python/.deps/glim/`; não usa sudo nem modifica o workspace catkin ou os pacotes Python existentes. A primeira compilação pode demorar bastante. `--jobs 1` reduz o uso de memória; o padrão é 2.

O instalador permite compilar o wrapper ROS 1 sem Iridescence, salva cada execução em sua própria pasta e corrige a reconstrução da nuvem no RViz quando o grafo atualiza submapas existentes. Também acrescenta ao log a mensagem `LOOP_ACCEPTED` para cada restrição de fechamento aceita, sem mudar os critérios do algoritmo. Outra correção drena a fila de candidatos antes de salvar: evita deixar fechamentos já aceitos fora do grafo exportado e processa candidatos restantes sem esperar por novos dados. O visualizador escolhido aqui é RViz. Esse perfil não instala a versão atual para ROS 2.

## Execução em terminais separados

Em cada terminal:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
cd ~/ardupilot/Rover
```

**Terminal 1 — Gazebo**, se ainda não estiver aberto:

```bash
roslaunch rover_argo_gazebo multi_rover_argo.launch \
  N:=1 world_name:=gravel_plane start_x:=0 start_y:=0 z:=0 \
  mid360_enabled:=true insta360_enabled:=true \
  samsung_s23_ultra_cam_enabled:=true hokuyo_enabled:=true
```

**Terminal 2 — GLIM:**

```bash
/usr/bin/python3.8 python/slam/glim/run_glim.py
```

Mantenha o rover parado por pelo menos 3 segundos para inicializar a IMU. O perfil usa `initialization_mode=NAIVE`, próprio dessa inicialização em repouso. Para outro namespace, use `--namespace /rover_argo_N2/Instance2`. Também existem `--cloud-topic`, `--imu-topic`, `--duration` e `--output-dir`; consulte `--help`.

**Terminal 3 — RViz:**

```bash
rviz -d python/slam/glim/glim_mid360.rviz
```

O **Fixed Frame** é `glim_map`. A varredura atual aparece amarela; o mapa global é colorido por altura. O mapa global depende da criação de submapas e sua publicação é limitada a aproximadamente uma vez a cada 10 segundos pelo wrapper oficial. Com o rover parado, a varredura pode aparecer antes do mapa global. Movimentar o rover produz novos quadros-chave e submapas.

O GLIM não movimenta o rover. Para navegar, inicie o SITL pelo `sim_1_gzrover.sh`, conforme o [tutorial dos serviços](TUTORIAIS_SERVICOS_SEPARADOS.md), e use os controles habituais. Para docking, mantenha também FAST-LIO e o gerador da nuvem local conforme [SLAM e docking 3D](SLAM_DOCKING_3D.md).

## Iniciar o conjunto em janelas separadas

Com as simulações anteriores encerradas:

```bash
./iniciar_simulacao_completa.sh --glim
```

Abre Gazebo, SITL, FAST-LIO, percepção, mapa local, GLIM, câmera e RViz GLIM. As opções `--sem-camera`, `--sem-rviz`, `--sem-qgc` e `--sem-mapa` continuam disponíveis. `--sem-mapa` desliga o gerador da nuvem local do docking; não desliga o mapa global interno do GLIM.

## Tópicos e referenciais

| Tópico | Conteúdo |
| --- | --- |
| `/glim/input/points` | MID360 XYZ corrigido para o plugin Gazebo, em `glim_lidar` |
| `/glim_ros/points` | Varredura processada pelo GLIM |
| `/glim_ros/odom` | Odometria local contínua: `glim_odom` → `glim_imu` |
| `/glim_ros/pose` | Pose corrigida pela otimização global, em `glim_map` |
| `/glim_ros/map` | Nuvem do mapa global corrigido |
| `/glim/status` | Contagem de entradas/poses/mapas, atraso e erro do adaptador |

Árvore TF própria: `glim_map → glim_odom → glim_imu → glim_lidar`. Não é o mesmo referencial do FAST-LIO (`camera_init`). Não sobreponha esses mapas sem calcular uma transformação entre suas origens.

```bash
rostopic echo -n 1 /glim/status
rostopic hz /glim_ros/odom
rostopic hz /glim_ros/points
rostopic echo -n 1 /glim_ros/pose
```

O wrapper oficial não fornece covariância de pose válida na mensagem de odometria. Por isso, o tópico não é conectado automaticamente ao receptor Lua/ArduPilot nem ao verificador de qualidade do docking. Receber poses não certifica a precisão delas.

## Fechamento de laços

Os parâmetros ficam em `python/slam/glim/config/config_global_mapping_pose_graph.json`:

- `enable_optimization: true`: otimização do grafo ligada.
- `min_travel_dist: 12.0`: procura revisitas após pelo menos 12 m percorridos entre submapas.
- `max_neighbor_dist: 3.0`: raio de procura dos candidatos, em metros.
- `min_inliear_fraction: 0.60`: sobreposição mínima para aceitar o alinhamento; a grafia é a usada pelo GLIM.
- `gicp_max_correspondence_dist: 0.5`: distância máxima entre correspondências, em metros.
- `odom_factor_stddev: 0.03` e `loop_factor_stddev: 0.05`: pesos relativos das restrições no grafo (desvios em radianos/metros). O perfil evita tratar cada deslocamento entre submapas como uma medida com precisão de 1 mm.

Percorra um circuito e retorne a uma região já observada, com boa sobreposição. A proximidade por si só não confirma um fechamento: o GLIM valida o registro geométrico. Os limiares são um perfil inicial para o cenário; ambientes repetitivos ou planos podem produzir resultados ruins. Não há garantia de precisão centimétrica apenas por habilitar a otimização.

Para confirmar fechamentos aceitos, procure `LOOP_ACCEPTED` em `glim.log`. A publicação de um mapa, sozinha, não demonstra que um laço foi fechado.

Cada execução copia os JSON para sua pasta de resultados. Edite os arquivos em `python/slam/glim/config/` e reinicie o GLIM para aplicar mudanças.

## Encerramento e resultados

Use Ctrl+C no terminal do GLIM e aguarde o salvamento. Resultados em `python/run/glim/<data>/`:

- `glim.log`: mensagens do algoritmo.
- `config/`: configuração exata daquela execução.
- `status.json`: resumo do adaptador.
- `dump/`: grafo, submapas e trajetórias do GLIM; `odom_imu.txt` contém a odometria local e `traj_imu.txt` a trajetória global otimizada.

Reiniciar Gazebo/relógio exige reiniciar o GLIM. O adaptador encerra se detectar regressão de timestamp para não misturar duas sessões no mesmo mapa.

## Particularidades deste sensor simulado

O adaptador preserva XYZ e corrige o deslocamento radial de 10 cm do plugin Livox usado neste Gazebo. Cada varredura é instantânea no simulador: `global_shutter_lidar=true`, sem inventar tempo de varredura. A IMU já fornece m/s², portanto `acc_scale=1.0`. LiDAR e IMU estão colocados na mesma origem no modelo atual.

Essas configurações são específicas da simulação. No MID360 físico, devem ser usados os timestamps por ponto, a calibração LiDAR/IMU e as unidades do driver real, sem a correção radial do Gazebo.

Fontes: [GLIM 1.1.0](https://github.com/koide3/glim/tree/v1.1.0), [wrapper ROS 1](https://github.com/koide3/glim_ros1), [documentação de execução](https://koide3.github.io/glim/quickstart.html).

## Validação realizada em 05/11/2026

- Compilação completa em CPU neste Ubuntu 20.04/ROS Noetic, sem CUDA ou instalação global das dependências.
- Testes automáticos do adaptador: preservação XYZ/timestamp, correção dos raios e configuração dos referenciais, módulos e unidades da IMU.
- Execução ao vivo com sensores do Gazebo e visualização no RViz. O perfil final acompanhou a entrada de aproximadamente 10 Hz; o atraso observado nas amostras do monitor ficou em torno de 0,01–0,10 s após inicializar.
- Reprodução de um circuito cinemático circular de cerca de 19 m, gravado em Gazebo isolado: **623 poses e dois fechamentos aceitos**. A sobreposição dos pares aceitos foi aproximadamente 62% e 90%.
- Verificado que a trajetória global salva difere da odometria local: correção máxima de cerca de 5,2 cm nessa reprodução. Isso confirma a aplicação das restrições ao grafo, não somente sua detecção.

Registros em `python/run/glim_validation/validated/`, com `glim.log`, `result.json` e `dump/`. A reprodução usou apenas as mensagens de LiDAR e IMU; posições de referência do teste não foram fornecidas ao GLIM. O circuito foi imposto ao modelo em uma simulação isolada, não executado como missão do ArduPilot. Portanto, este ensaio não valida o controle do rover nem garante precisão de docking. Ainda há deriva, principalmente vertical, e o resultado varia com geometria, IMU e densidade dos pontos. FAST-LIO permanece como fonte local do docking.
