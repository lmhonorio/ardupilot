# Leitura do LiDAR simulado — ROS 2

Execute no PC, em `~/ardupilot/Rover`, com o MID360 habilitado no Gazebo.
Para o sensor físico, use o [tutorial MID360 independente](../mid360/README.md).

Listar os tópicos e abrir a leitura:

```bash
./ros2/service.sh lidar --list
./ros2/service.sh lidar
```

Salvar uma nuvem em arquivo, sem janela:

```bash
./ros2/service.sh lidar --headless --snapshot /tmp/mid360.npz
```

Consultar opções:

```bash
./ros2/service.sh lidar --help
```

Use Ctrl+C para encerrar o leitor. A leitura isolada não produz um mapa SLAM
acumulado. Para compensar o movimento e gerar mapa/nuvem local, use
[FAST-LIO e mapa 3D](../slam/SLAM_DOCKING_3D.md).

Para processar o MID360 físico na Raspberry e visualizar no PC, siga
[FAST-LIO independente na Pi](../mid360/fast_lio_raspberry/README.md).
