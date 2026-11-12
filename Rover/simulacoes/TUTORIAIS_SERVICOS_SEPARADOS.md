# Serviços separados — ROS 2 Jazzy / Gazebo Harmonic

Para iniciar diretamente, use os tutoriais rápidos:
[tudo local](TUTORIAL_RAPIDO_LOCAL.md) ou
[controle na Raspberry](TUTORIAL_RAPIDO_REMOTO.md).

Para usar a Raspberry como computador de bordo, consulte o
[tutorial do container remoto](companion/RASPBERRY_DOCKER.md). Nesse modo,
Gazebo, SITL, RViz e clientes ROS ficam neste computador; FAST-LIO, mapa e
gerenciador ficam na Raspberry. Não abra suas cópias locais simultaneamente.


Para o ciclo integrado de missão, entrada por LiDAR com QR opcional, recarga
simulada e saída antes de AUTO, siga o
[tutorial do Mission Manager](mission_manager/TUTORIAL.md). Veja também a
[arquitetura](mission_manager/ARQUITETURA.md) e os [testes](mission_manager/TESTES.md).

O fluxo atualizado está em [ROS2_JAZZY.md](ROS2_JAZZY.md): instalação e compilação, cada serviço em um terminal, perfis com/sem desvio, FAST-LIO, GLIM, mapas, câmera e docking.

Instale e compile uma vez antes de seguir os comandos. O estado dos testes está registrado no guia principal. Não misture os comandos anteriores `source /opt/ros/noetic`/`roslaunch` com o novo ambiente.

```bash
cd ~/ardupilot/Rover
sudo bash ros2/install_ros2.sh
./ros2/build.sh
```

Depois da compilação, um comando em cada terminal:

```bash
./ros2/service.sh gazebo
./sim_1_gzrover.sh --foreground --add-param-file nroverobst.param
./ros2/service.sh mavros
./ros2/service.sh obstaculos
./ros2/service.sh fastlio
./ros2/service.sh mapa
./ros2/service.sh camera
./ros2/service.sh rviz
```

Sem desvio, troque o arquivo por `nrover_sem_desvio.param` e omita o serviço `obstaculos`. Os comandos listados acima não devem ser colados juntos em um único terminal: cada serviço permanece em execução.

Os perfis `nroverobst.param`, `nrover_sem_desvio.param` e `navegacao_gazebo.param`
usam `WP_SPEED = 1.5` para a velocidade nominal de missão, em m/s. Curvas,
aproximação dos waypoints e obstáculos podem reduzir a velocidade efetiva.
As velocidades comandadas pelas rotinas de docking continuam próprias dessas rotinas.

As câmeras Samsung e Insta360 usam **640 × 360 a 30 FPS**, com o mesmo campo de visão. A **Insta360 fica desligada por padrão**, também ao usar `iniciar_simulacao_completa.sh`. Para habilitá-la, inicie o Gazebo com `./ros2/service.sh gazebo insta360_enabled:=true`.

A resolução foi reduzida no sensor simulado, não apenas na janela: são nove vezes menos pixels por imagem. Reinicie a simulação para aplicar as mudanças; abrir novamente só o leitor de câmera ou o RViz não altera um mundo já carregado. O docking usa a calibração publicada em `camera_info`, mas o QR pode ser detectável a uma distância menor com essa resolução.

## Docking em outro terminal

O procedimento completo, incluindo ativação pelo MAVProxy e troca do monitor
de entrada pela rotina de saída, está no
[tutorial de entrada e saída do docking](docking/TUTORIAL_ENTRADA_SAIDA_ROS2.md).

As estações e seus eixos de entrada ficam em [estacoes_recarga.txt](../estacoes_recarga.txt).
Veja [como cadastrar os pontos a 4 m e 2 m e o QR](docking/CADASTRO_ESTACOES.md).

Com os serviços acima ativos, verifique os sensores antes de iniciar o controlador:

```bash
cd ~/ardupilot/Rover
./ros2/service.sh docking-qr --diagnostico
```

Depois, mantenha o monitor aberto com `./ros2/service.sh docking-qr`. Ele observa
a telemetria e inicia a entrada quando o rover estiver armado, em GUIDED e a
menos de 8 m de uma estação cadastrada. Não cria missão. Para a alternativa
LiDAR, com FAST-LIO e mapa ativos, use primeiro
`./ros2/service.sh docking-lidar --diagnostico` e depois
`./ros2/service.sh docking-lidar`. Execute apenas um controlador de docking.

Na entrada por QR, o limite padrão de aproximação é **0,25 m/s**, com limite
de **0,18 m/s na entrada** e redução progressiva junto ao centro. O controlador
alinha o rover ao eixo da estação, acompanha o QR com o pan/tilt e confirma a
pose após parar antes de desarmar. Para reduzir esses limites, por exemplo:

```bash
./ros2/service.sh docking-qr --velocidade 0.15
```

Ao atualizar os arquivos, encerre o monitor anterior com Ctrl+C e abra-o
novamente para carregar o código novo.

A revisão foi validada em seis entradas completas no Harmonic. Veja as
[trajetórias, erros medidos e limites dos ensaios](docking/VALIDACAO_ALINHAMENTO_VELOCIDADE_ROS2.md).

Comandos de saída, observação do QR, configuração dos tópicos e solução de
problemas estão em [Docking no ROS 2](<docking/README DOCKING.md#executar-os-scripts-no-ros-2>).
Os atalhos já selecionam o Python 3.12 do ambiente Jazzy; não use Python 3.8.
