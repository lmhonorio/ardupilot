# Raspberry Pi 5 — instalar e operar com SITL/Gazebo

Destino desta instalação: **rover@192.168.0.167**, Ubuntu 24.04 ARM64,
Raspberry Pi 5 com 16 GB de RAM. A senha é solicitada pelo SSH/sudo; não fica
na imagem ou nos arquivos de configuração.

Gazebo, ArduRover SITL, Lua, RViz e QGroundControl ficam no computador.
FAST-LIO, mapa 3D, docking e gerenciamento de missão ficam no Docker da Pi.
Os comandos de status e início de missão continuam sendo executados no computador.
Para hardware físico, veja o [segundo tutorial](INSTALACAO_PI5_ROVER_REAL.md).

**Validação atual:** um ciclo de missão → docking → recarga passou. A repetição
partindo de dentro falhou durante a saída por alinhamento/odometria; a saída
completa de 3 m ainda não está validada nesta Pi/rede. Consulte os resultados
e limites no [relatório](VALIDACAO_PI5.md) antes de repetir ciclos automáticos.

## 1. Preparar uma Raspberry nova

Instale Ubuntu Server **24.04 de 64 bits**, habilite SSH e conecte a Pi à mesma
rede do computador. Esta etapa pressupõe que o projeto já está instalado em
`~/ardupilot/Rover` no computador. Os passos abaixo não reinstalam o sistema
operacional nem modificam os endereços de rede.

No computador:

```bash
cd ~/ardupilot/Rover
ssh rover@192.168.0.167 'uname -m; cat /etc/os-release; free -h; df -h /'
scp companion/install_docker.sh companion/configure_dds_buffers.sh \
  rover@192.168.0.167:
```

Na Raspberry:

```bash
ssh rover@192.168.0.167
sudo bash ~/install_docker.sh --classic-store
sudo bash ~/configure_dds_buffers.sh
exit
```

`--classic-store` é opcional e **somente para instalação nova**. Nesta Pi,
com cartão de 16 GB, seleciona `overlay2` para evitar a cópia adicional de
camadas comprimidas do armazenamento containerd. Se Docker já estiver instalado,
não troque o armazenamento: execute o instalador sem essa opção, se necessário.
O script recusa aplicar a opção sobre uma instalação existente.

Abra outra conexão para atualizar o grupo `docker`:

```bash
ssh rover@192.168.0.167 'docker version; docker compose version; docker info --format "{{.Driver}}"'
```

Docker e ROS 2 seguem os pacotes/imagens oficiais. ROS 2 e as dependências dos
serviços são instalados **dentro da imagem**; não é preciso instalar ROS na Pi
fora do Docker. Use cartão/SSD com espaço para a imagem, compilação e logs;
acompanhe `df -h` em armazenamento pequeno. Não é necessário swap nesta Pi de
16 GB para o procedimento usado aqui.

## 2. Preparar o computador e compilar a imagem

Os buffers precisam ser ajustados também no computador, uma vez:

```bash
cd ~/ardupilot/Rover
sudo bash companion/configure_dds_buffers.sh
```

Execute a versão atual mesmo se já ajustou os sockets na instalação da Pi 4:
ela também amplia a remontagem IP para pelo menos 64 MiB e limita a retenção
de fragmentos incompletos a 3 s. Os sockets permanecem com mínimo de 8 MiB.
Após ajustar, reinicie os processos ROS. O script salva os valores anteriores
em `/var/lib/rover-companion` e não altera IPs.

Enviar fontes e compilar nativamente na Pi:

```bash
cd ~/ardupilot/Rover
export ROVER_REMOTE=rover@192.168.0.167
./companion/deploy.sh 2>&1 | tee /tmp/rover-pi5-build.log
```

A primeira compilação demora: inclui driver Livox/SDK2, FAST-LIO e interfaces.
Ela cria `rover-companion:jazzy` e `~/rover_companion` na Pi, mas não inicia
controle nem missão. Os arquivos existentes em `~/rover_companion/config`
são preservados em atualizações.

Teste as dependências da imagem, ainda sem simulador:

```bash
ssh rover@192.168.0.167 \
  'docker run --rm rover-companion:jazzy python companion/tests/image_smoke.py'
```

## 3. Iniciar os serviços — um terminal para cada bloco

Prepare **cada terminal do computador**:

```bash
cd ~/ardupilot/Rover
export ROS_DOMAIN_ID=0
export ROVER_REMOTE=rover@192.168.0.167
source companion/pc_env.bash
```

Encerre qualquer cópia local de FAST-LIO, mapa ou gerenciador antes de abrir
as cópias remotas. Não execute `iniciar_simulacao_completa.sh` junto.

### Terminal 1 — Gazebo

```bash
./ros2/service.sh gazebo \
  world:=gravel_plane N:=1 start_x:=2.5 start_y:=-2 \
  real_time_factor:=0.5 \
  mid360_enabled:=true samsung_s23_ultra_cam_enabled:=true \
  insta360_enabled:=false hokuyo_enabled:=false
```

O perfil inicial usa `0.5`: um segundo simulado leva aproximadamente dois
segundos reais. Na rede deste ensaio, `1.0` apresentou entrega irregular das
nuvens, mesmo com a Pi 5. A taxa efetiva depende também do computador e da rede.
Os resultados efetivamente medidos estão no [relatório da Pi 5](VALIDACAO_PI5.md).

### Terminal 2 — SITL e MAVProxy

```bash
ROVER_COMPANION_HOST=192.168.0.167 \
  ./sim_1_gzrover.sh --foreground \
  --add-param-file nrover_sem_desvio.param --no-rebuild
```

Usa o firmware deste projeto já compilado, incluindo `DOCK_LOCK` e Lua.
MAVLink segue para UDP 14600 da Pi; QGroundControl local usa UDP 14550.
Aguarde GPS/EKF prontos.

### Terminal 3 — serviços na Pi

```bash
./companion/control.sh up
./companion/control.sh logs
```

O supervisor espera sensores recentes antes de abrir o gerenciador.
O perfil instalado usa docking LiDAR, QR opcional desativado e desvio desligado.
Não abra as rotinas independentes de docking junto do Mission Manager.

### Terminal 4 — RViz no computador

```bash
./ros2/service.sh rviz
```

Fixed Frame `camera_init`; mapa `/rover_slam_3d/map`; nuvem local
`/rover_slam_3d/local_cloud`; odometria `/fast_lio/odometry`.
Para abrir a câmera, use outro terminal com `./ros2/service.sh camera`.

### Terminal 5 — status e missão

```bash
./companion/control.sh status
./companion/control.sh mission Missions/new_argo.plan
./companion/control.sh watch
```

O comando copia a missão para a Pi e chama o serviço ROS. Se estiver dentro,
o gerenciador solicita a saída de ré 3 m antes de executar AUTO; esse percurso
ainda apresentou falha no segundo ensaio descrito acima. Ao concluir a missão, retorna ao docking,
desarma e simula a recarga na bateria do próprio SITL. A aceitação do pedido
não equivale à conclusão; espere `RECARGA_CONCLUIDA`.

Outro arquivo `.plan`/`.waypoints` deve terminar diante de uma estação cadastrada,
a menos de 8 m do centro. Os caminhos enviados diretamente a `start_file` devem
existir **no container**; prefira `control.sh mission`, que faz a cópia.

## 4. Parar tudo

Interrompa `watch` com Ctrl+C e execute:

```bash
./companion/control.sh cancel
./companion/control.sh status
```

Confirme HOLD (`mode=4`), parada e desarmamento. Depois:

```bash
./companion/control.sh down
ssh rover@192.168.0.167 'cd ~/rover_companion && docker compose ps --all'
```

Feche câmera/RViz, SITL/MAVProxy e por último Gazebo com Ctrl+C; feche QGroundControl.
`down` preserva a imagem, configurações e logs. Ctrl+C em `logs`/`watch` apenas
fecha a visualização. Não use Ctrl+Z para encerrar.

## 5. Diagnóstico e atualização

```bash
./companion/control.sh status
ssh rover@192.168.0.167 'docker stats --no-stream rover-companion; df -h /'
./companion/control.sh logs
```

Leia `last_error` se houver `FALHA`. Após corrigir a causa, com o rover parado:

```bash
./companion/control.sh reconcile
```

Para atualizar, pare os serviços, execute `deploy.sh` novamente e recrie com
`control.sh up`. Os logs ficam em `~/rover_companion/data/runs` na Pi. Copie os
ensaios que deseja guardar e acompanhe o espaço; não há rotação automática de
todos os arquivos de telemetria dentro de `data`.

Referências: [instalação oficial Docker](https://docs.docker.com/engine/install/ubuntu/)
e [armazenamento de imagens Docker](https://docs.docker.com/engine/storage/containerd/).

Ajuste de fragmentos baseado nas [orientações oficiais de DDS/ROS 2](https://raw.githubusercontent.com/ros2/ros2_documentation/jazzy/source/How-To-Guides/DDS-tuning.rst).
