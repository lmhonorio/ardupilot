# Do computador ligado à simulação — ROS 2 Jazzy

O roteiro completo, com um serviço por terminal, está em [ROS2_JAZZY.md](../ros2/ROS2_JAZZY.md).

1. Entre na sessão gráfica do Ubuntu 24.04 e abra um terminal.
2. Na primeira configuração, execute `cd ~/ardupilot/Rover`, `sudo bash ros2/install_ros2.sh` e depois `./ros2/build.sh`. Nas próximas inicializações, não repita a instalação.
3. Abra tudo com:

   ```bash
   cd ~/ardupilot/Rover
   ./iniciar_simulacao_completa.sh --no-rebuild
   ```

   Use `--without-avoidance` para o perfil sem desvio; `--glim` acrescenta GLIM e seu RViz2. Omita `--no-rebuild` se ainda precisar compilar o firmware.
4. Aguarde o Gazebo carregar o rover e o FAST-LIO inicializar com o veículo parado. O lançador abre também SITL/MAVProxy, MAVROS, percepção, mapa 3D, câmera e RViz2. Não arma nem cria missão.
5. Abra o QGroundControl normalmente: a telemetria chega por UDP 14550. Aguarde GPS, EKF e PreArm saudáveis antes de executar uma missão.
6. Para conferir a odometria em outro terminal:

   ```bash
   source ~/ardupilot/Rover/ros2/setup.bash
   ros2 topic echo /fast_lio/odometry --once
   ros2 topic hz /rover_argo_N1/Instance1/livox/lidar
   ```

7. Se desejar docking, escolha apenas um monitor: `./ros2/service.sh docking-qr` ou `./ros2/service.sh docking-lidar`. Ele aguarda as condições descritas no [manual de docking](../docking/README%20DOCKING.md). A migração de transporte não certifica precisão de estacionamento no novo motor físico.
8. Ao encerrar, coloque HOLD e desarme; interrompa os controladores e depois SLAM/percepção, SITL e Gazebo com Ctrl+C nos terminais. Aguarde o GLIM salvar o mapa antes de fechar seu terminal.

Use Python 3.12 do ambiente `ros2/venv`, carregado pelos scripts. Não carregue o catkin/Noetic nem use Python 3.8 com o Jazzy. O roteiro anterior está no backup `../backups/ros1_antes_ros2_20261109/simulacoes/`.
