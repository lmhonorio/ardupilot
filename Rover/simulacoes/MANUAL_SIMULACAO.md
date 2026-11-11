# Manual de simulação — ROS 2 Jazzy / Gazebo Harmonic

Consulte [instalação, serviços e diagnóstico](ROS2_JAZZY.md) e o [roteiro após ligar o computador](PASSO_A_PASSO_ROS_FAST_LIO.md). A versão ROS 1 anterior foi preservada em `../backups/ros1_antes_ros2_20261109/simulacoes/`.

## Gazebo, sensores, Lua e MAVProxy

```bash
cd ~/ardupilot/Rover
./iniciar_simulacao_completa.sh
```

Opções: `--without-avoidance`, `--glim`, `--no-rebuild` e `--headless`. Para abrir separadamente, siga [TUTORIAIS_SERVICOS_SEPARADOS.md](TUTORIAIS_SERVICOS_SEPARADOS.md). O comando `sim_1_gzrover.sh --add-param-file ARQUIVO` inicia SITL/MAVProxy com o perfil escolhido; exige o Gazebo aberto.

## Apenas SITL, sem Gazebo ou sensores ROS

Em um terminal separado, sem outra instância SITL ativa:

```bash
cd ~/ardupilot/Rover
source ros2/setup.bash
python ../Tools/autotest/sim_vehicle.py -v Rover -f rover-skid \
  -L SEParnaiba --sysid 1 -I 0 \
  --add-param-file nrover_sem_desvio.param \
  --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551
```

Este backend usa a física interna do SITL; não produz câmera/MID360. Não avalie SLAM ou docking com ele.

## Missão e ajuste automático

Com o simulador e MAVProxy já ativos, confira as opções do programa de missão:

```bash
cd ~/ardupilot/Rover
./mission_tune.sh --help
```

O lançador usa Python 3.12 e preserva o protocolo MAVLink através do MAVProxy. Antes de escolher uma execução que arma, envia missão ou altera ganhos, confira as opções e o perfil selecionado. Nenhum autotuning foi executado como parte da migração ROS 2; os ganhos existentes foram preservados. Os resultados anteriores pertencem à física anterior.

## Percepção e docking

- [Mapa 3D e nuvem local](SLAM_DOCKING_3D.md).
- [GLIM e fechamento de laços](GLIM_MID360.md).
- [Entrada/saída por QR](docking/README%20DOCKING.md).
- [Entrada/saída por LiDAR](docking/DOCKING_LIDAR.md).

Mapeamento e visualização não movimentam o rover. Monitores de docking podem assumir o controle quando as condições de ativação forem atendidas; execute apenas um controlador de docking por vez.
