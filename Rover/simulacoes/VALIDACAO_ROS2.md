# Validação da migração ROS 2 — 09/11/2026

## Ambiente instalado nativamente

Ubuntu 24.04, ROS 2 Jazzy, Python 3.12 e Gazebo Harmonic. Instalação APT concluída; modelo geoidal EGM96 do MAVROS presente. Noetic e os assets originais foram preservados, mas não são carregados no fluxo novo.

| Componente | Versão do pacote instalado |
| --- | --- |
| ros-jazzy-desktop | 0.11.0 |
| ros-jazzy-ros-gz | 1.0.24 |
| ros-jazzy-glim-ros | 1.2.2 |
| ros-jazzy-mavros | 2.16.0 |
| ros-jazzy-rviz2 | 14.1.24 |
| ros-jazzy-slam-toolbox | 2.8.5 |

O workspace compilou os quatro pacotes `ardupilot_gazebo`, `livox_ros_driver2` (somente interfaces), `rover_argo_gazebo` e `fast_lio`. Revisões de fontes: `../ros2/dependencies.lock`. Logs de compilação e testes: `../ros2/run/migration_validation/`.

## Testes concluídos

- 65 testes de percepção, timestamps, projeção, geometria, docking e ponte Lua: passaram.
- 20 testes de missão/MAVLink e ajuste de parâmetros: passaram.
- Janela OpenCV nativa: abriu e exibiu a imagem capturada da câmera.
- MAVROS: clock de simulação confirmado nos nós principal e de sincronização; conexão FCU recebida no ensaio integrado.
- Comunicação DDS real com QoS best effort, `/clock` e idade do sensor: passou nativamente.
- Geração do cenário com três modelos: portas JSON 9002/9012/9022 e namespaces separados conferidos. Não equivale a testar três robôs navegando simultaneamente.
- Gazebo gráfico e RViz2 nativos: abriram, OpenGL 4.6 inicializado; recebidas 10 nuvens e 33 imagens. O teste não iniciou SITL nem armou veículo. Logs: `../ros2/run/native_gui/`.
- Gazebo nativo sem janela: 10 nuvens e 18 imagens recebidas, sem SITL/armamento. Logs: `../ros2/run/native_headless/`.
- Primeiro ensaio integrado estacionário: 210 poses FAST-LIO, 179 poses GLIM, 206 nuvens MID360, imagens de ambas as câmeras, Hokuyo, juntas e percepção de obstáculos. As 210 poses foram enviadas ao Lua, com confirmação `FAST-LIO: Lua pose available`; mapa 3D sem rejeições. Arquivos em `../ros2/run/validation_20261109_174739/`.

Os ensaios integrados usam um container Jazzy/Harmonic com rede Docker interna e GPU Intel, separado das portas do host. A instalação e os testes nativos acima não dependem desse container para o usuário executar a simulação.

## Ensaio integrado com movimento

Execução `../ros2/run/validation_20261109_180004/`: sensores, FAST-LIO, GLIM, mapas 2D/3D e MAVROS conectados, nenhum tópico esperado ausente. Após GPS/EKF e PreArm saudáveis, o teste armou normalmente em GUIDED, comandou 0,2 m/s por 8 segundos de simulação, solicitou HOLD, aguardou parada e desarmou. Deslocamento GPS medido: **1,580 m**. Evidência: `motion.json`, `result.json` e `mav.tlog` nessa pasta.

O movimento foi realizado apenas no SITL isolado, sem missão, sem force-arm e sem desabilitar checagens. O perfil usado foi `nrover_sem_desvio.param`. A percepção publicou dados, mas esse ensaio não comprova contorno de obstáculos: o perfil de controle estava sem desvio.

No período inicial de 60 segundos de relógio real, foram observadas 116 nuvens, 114 poses FAST-LIO, 100 poses GLIM, 14 publicações do mapa 3D, 26 do mapa 2D e 16 estados MAVROS conectados. O processamento continuou durante a espera de GPS/EKF e o movimento. Os tempos de sensores são de simulação; não interprete essas contagens como a frequência nominal em tempo real.

O MAVROS integrado usou inicialmente o launch oficial; a configuração final adiciona `use_sim_time=true` para todos os seus nós. Esse parâmetro foi confirmado separadamente nos nós principal e de sincronização no sistema nativo.

## Correções necessárias na migração

- O plugin JSON exige os vetores da IMU em FRD. A IMU do piloto tem rotação de 180° em roll; a do MID360 mantém a convenção ROS. Sem isso, o EKF estima o veículo invertido e recusa armamento.
- Corrigidos limites de torque do PID de rodas cujo máximo era menor que o mínimo.
- Três malhas de colisão de acessórios incompatíveis com DART foram convertidas em caixas envolventes. Rodas, corpo, massas e visuais preservados.
- Sensores migrados para mensagens ROS 2, QoS de sensores e relógio de simulação. Feedback pan/tilt vem de juntas medidas.
- FAST-LIO publica a covariância atual junto da pose; IMU aceita QoS best effort.
- Removida a correção radial de 10 cm específica do plugin LiDAR Classic.
- Ambiente de execução remove bibliotecas GTK/GIO do Snap herdadas da IDE, que impediam RViz2 de abrir.

## Limites dos resultados

**Diferença de deslocamento encontrada:** no ensaio de avanço, o GPS registrou 1,580 m e o GLIM terminou próximo de 1,601 m; a pose final do FAST-LIO ficou próxima de 0,668 m no eixo de avanço. Portanto, a publicação de mensagens e o transporte ao Lua passaram, mas a exatidão da odometria FAST-LIO nesse cenário não está aprovada. Essa diferença deve ser investigada com trajetória registrada antes de confiar na distância integrada do FAST-LIO para docking. Não houve troca automática da fonte de localização nem ajuste de ganhos para mascarar a diferença.


A migração não executou autotuning nem certificou a precisão de seguir linhas ou estacionar no container com a nova física. O MID360 simulado usa uma grade angular uniforme; não reproduz a varredura não repetitiva real. O ensaio parado de GLIM confirma odometria e exportação de grafo/submapa, mas não demonstra um fechamento de laço aceito durante uma volta. As rotinas de docking foram portadas e seus testes passaram; entrada/saída física completa ainda exige ensaio específico.

Os scripts antigos fora dos lançadores principais continuam disponíveis como histórico e podem usar Gazebo Classic. Para o fluxo migrado, use os comandos de [ROS2_JAZZY.md](ROS2_JAZZY.md).
