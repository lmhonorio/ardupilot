# Início rápido — controle na Raspberry

Instalação na nova Pi 5: [SITL/Gazebo](INSTALACAO_PI5_SIMULACAO.md)
ou [rover físico](INSTALACAO_PI5_ROVER_REAL.md).

No computador: Gazebo, SITL/Lua/MAVProxy, RViz, câmera e comandos de missão.
Na Raspberry `rover@192.168.0.167`: Docker com FAST-LIO, mapa e Mission Manager
(docking LiDAR, saída e recarga simulada).

Na Pi 5, missão → entrada → recarga passou no ensaio, mas a repetição falhou
na saída por alinhamento/odometria. Veja a [validação e limitações atuais](VALIDACAO_PI5.md).

Este guia usa a imagem já instalada e os buffers de rede já configurados.
Comece sem a simulação anterior aberta. Encerre os serviços **locais** de
FAST-LIO, mapa, Mission Manager e docking antes de usar este fluxo.
Não execute `iniciar_simulacao_completa.sh` junto.

## Preparação em cada terminal do computador

Todos os comandos deste tutorial são digitados **neste computador**, inclusive
os que iniciam o container por SSH. Em cada novo terminal:

```bash
cd ~/ardupilot/Rover
export ROS_DOMAIN_ID=0
export ROVER_REMOTE=rover@192.168.0.167
source companion/pc_env.bash
```

## Terminal 1 — Gazebo local

```bash
./ros2/service.sh gazebo \
  world:=gravel_plane N:=1 start_x:=2.5 start_y:=-2 \
  real_time_factor:=0.5 \
  mid360_enabled:=true samsung_s23_ultra_cam_enabled:=true \
  insta360_enabled:=false hokuyo_enabled:=false
```

O fator `0.5` reduz a carga de sensores na rede: cada segundo simulado leva
cerca de dois segundos reais. Não altera a velocidade configurada em m/s.
Veja os resultados e limites da Pi 5 no [relatório](VALIDACAO_PI5.md).

## Terminal 2 — SITL local com MAVLink para a Raspberry

```bash
ROVER_COMPANION_HOST=192.168.0.167 \
  ./sim_1_gzrover.sh --foreground \
  --add-param-file nrover_sem_desvio.param --no-rebuild
```

Aguarde GPS/EKF prontos. O encaminhamento à Raspberry usa UDP 14600;
o QGroundControl local continua recebendo na porta 14550.

**Não omita `ROVER_COMPANION_HOST=192.168.0.167`.** Executar somente
`./sim_1_gzrover.sh` não cria a saída MAVLink remota. A missão pode ser copiada
por SSH mesmo assim, mas o gerenciador apresentará `Sem resposta HEARTBEAT via MAVProxy`.
Definir essa variável depois de iniciar o SITL não modifica o MAVProxy já aberto.

Se esqueceu a variável, na próxima inicialização use o bloco completo acima.
Para conectar um MAVProxy já aberto, sem reiniciar o SITL e **sem outra ponte
MAVLink ativa para a Pi**, digite no console **MAVProxy**, não no Bash:

```text
output add udp:192.168.0.167:14600
set mavfwd true
```

Depois, em um terminal Bash preparado como acima, com o rover parado:

```bash
./companion/control.sh reconcile
./companion/control.sh status
```

Aguarde `state=ESPERA`, `fresh=true`, `busy=false` e `last_error=null` antes
de enviar a missão. Se o rover estiver estacionado, o estado pode ser de
estacionamento/recarga. A resposta de aceitação de `reconcile` sozinha não
confirma a recuperação. `fresh=true` com erros de ACK/parâmetros indica que
a telemetria chega, mas o caminho de retorno dos comandos ainda precisa ser verificado.

## Terminal 3 — iniciar os serviços remotos e acompanhar logs

```bash
./companion/control.sh up
./companion/control.sh logs
```

O container inicia MAVProxy, recepção de sensores, FAST-LIO, mapa e gerenciador.
Aguarde os sensores e a reconciliação. Não abra cópias locais desses serviços.
Ctrl+C no acompanhamento dos logs **não** encerra o container.
O perfil instalado usa LiDAR sem QR e desvio desligado.

## Terminal 4 — RViz local

```bash
./ros2/service.sh rviz
```

Fixed Frame: `camera_init`. Mapa: `/rover_slam_3d/map`.
Nuvem local: `/rover_slam_3d/local_cloud`. O mapa é calculado na Raspberry.

## Terminal 5 — consultar e iniciar a missão

Consultar uma vez:

```bash
./companion/control.sh status
```

Enviar um arquivo deste computador e iniciar:

```bash
./companion/control.sh mission Missions/new_argo.plan
```

Esse comando copia o arquivo para a Raspberry e chama `/mission_manager/start_file`.
Troque o caminho para usar outro `.plan` ou `.waypoints`. Não passe um caminho
`/home/grin/...` diretamente ao serviço remoto: ele lê arquivos no container.
O último waypoint deve terminar diante da abertura da estação cadastrada,
a menos de 8 m do centro.

Se estiver estacionado, aguarda a recarga e sai de ré 3 m antes de iniciar.
Ao terminar a missão, entra, desarma e simula a recarga até 100%.
A aceitação do pedido não significa que o ciclo terminou. Acompanhe:

```bash
./companion/control.sh watch
```

O estado final esperado é `RECARGA_CONCLUIDA`, `inside=true`, `mode=4` (HOLD)
e `armed=false`. Ctrl+C encerra apenas o acompanhamento do status.

## Opcional — câmera local em outro terminal

```bash
./ros2/service.sh camera
```

## Parar a missão e encerrar tudo

### 1. Solicitar parada

No terminal 5, interrompa `watch` com Ctrl+C para voltar ao prompt. Isso fecha
somente a visualização do status. Depois execute:

```bash
./companion/control.sh cancel
./companion/control.sh status
```

Espere `mode: 4` (HOLD), telemetria recente (`fresh: true`) e rover parado.
A resposta ao cancelamento confirma o pedido, não a conclusão da parada.
O cancelamento pode aparecer como `FALHA` com motivo de cancelamento pelo operador.
Ele não garante desarmamento; desarme pelo QGroundControl após confirmar a
parada e confira `armed: false`.

Se o serviço não responder, interrompa a chamada com Ctrl+C e selecione HOLD
no QGroundControl ou no MAVProxy local. Confirme a parada antes de continuar.

### 2. Encerrar os serviços da Raspberry

Mantenha Gazebo e SITL abertos enquanto o container encerra:

```bash
./companion/control.sh down
```

Esse comando encerra o gerenciador, mapa, FAST-LIO e MAVProxy remotos e remove
o container em execução. A imagem, configurações, missões e logs persistentes
ficam disponíveis para a próxima inicialização. Não desliga a Raspberry.

Confira:

```bash
ssh rover@192.168.0.167 \
  'cd ~/rover_companion && docker compose ps --all'
```

Não deve haver serviço do projeto em execução. Ctrl+C em `logs` **não** substitui
`down`. Após o encerramento, o tópico de status deixa de ser publicado.

### 3. Fechar os programas deste computador

Pressione Ctrl+C e espere terminar, nesta ordem:

| Ordem | Terminal / programa |
| --- | --- |
| 1 | Terminal 3 — acompanhamento de logs, se ainda estiver aberto |
| 2 | Terminal opcional — câmera; terminal 4 — RViz |
| 3 | Terminal 2 — SITL e MAVProxy locais |
| 4 | Terminal 1 — Gazebo |

Feche também o QGroundControl pelo menu Sair ou pelo botão de fechar a janela.
Não use Ctrl+Z: ele apenas suspende o programa. Fechar só a janela do Gazebo
não garante que seu servidor terminou.

Para conferir processos locais restantes, no terminal 5:

```bash
pgrep -af '[s]im_vehicle.py|[a]rdurover|[m]avproxy.py|[g]z sim|[g]z-sim|[r]viz2|[r]ead_ros_camera.py|[Q]GroundControl'
```

Se aparecer algum, confira o PID e volte ao terminal correspondente para
encerrá-lo. Não use comandos que matem indiscriminadamente Python ou ROS.

Para apenas interromper a missão e manter os sistemas abertos, execute somente
o passo 1. Antes de solicitar outra missão, confira `last_error`, corrija a
causa se necessário e use `./companion/control.sh reconcile`.

Este fluxo é de simulação. Para hardware físico e configuração da imagem,
consulte o [tutorial completo](RASPBERRY_DOCKER.md).
Alternativa: [executar tudo localmente](../simulacao/TUTORIAL_RAPIDO_LOCAL.md).
