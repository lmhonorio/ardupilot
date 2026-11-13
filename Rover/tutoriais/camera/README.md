# Câmera e observação do QR — ROS 2

Execute no PC, a partir de `~/ardupilot/Rover`, com o Gazebo e a câmera
Samsung habilitados. O lançador seleciona o Python compatível com ROS 2 Jazzy.

Abrir a imagem:

```bash
./ros2/service.sh camera
```

Observar o QR, em outro terminal:

```bash
./ros2/service.sh observar-qr
```

Para conferir as opções de câmera e de tópicos:

```bash
./ros2/service.sh camera --help
./ros2/service.sh observar-qr --help
```

Use Ctrl+C no terminal correspondente para encerrar. Ver a câmera ou observar
o QR não inicia uma entrada no contêiner. Para esse controle, consulte
[docking no ROS 2](../docking/TUTORIAL_ENTRADA_SAIDA_ROS2.md).

O perfil de simulação usa câmera Samsung de 640 × 360; Insta360 permanece
desligada por padrão. Os comandos para habilitar os sensores estão no
[guia local](../simulacao/TUTORIAL_RAPIDO_LOCAL.md).
