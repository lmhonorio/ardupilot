Para missão, entrada com LiDAR e QR opcional, recarga e saída automática antes de AUTO, use o [tutorial integrado do Mission Manager](../mission_manager/TUTORIAL.md). Os serviços independentes abaixo continuam disponíveis; execute apenas um controlador de docking por vez.

# Tutorial: acionar a entrada e a saída do docking no ROS 2

Atualizado em 10/11/2026. Ambiente: ROS 2 Jazzy, Gazebo Harmonic, um rover e
estação `container_gravel_plane`. O procedimento principal usa o QR do contêiner.

Os comandos `./ros2/service.sh ...` iniciam programas que ficam executando no
terminal. Não são chamadas `ros2 service call`. O lançador carrega o ROS 2 e o
Python 3.12 do projeto automaticamente; não use Python 3.8 ou o ambiente Noetic.

| Ação | Comando em `~/ardupilot/Rover` | Comportamento |
| --- | --- | --- |
| Diagnosticar a câmera | `./ros2/service.sh docking-qr --diagnostico` | Verifica imagem, calibração e juntas; não comanda o rover |
| Aguardar entrada por QR | `./ros2/service.sh docking-qr` | Monitora GUIDED, armamento e distância à estação |
| Sair por QR | `./ros2/service.sh sair-qr` | Verifica o rover dentro da estação, recua 2 m, gira 180° e encerra |
| Ver a câmera | `./ros2/service.sh camera` | Abre a imagem da câmera Samsung |

## 1. Preparar a simulação

Este tutorial pressupõe instalação e compilação já concluídas. Para preparar
o ambiente pela primeira vez, consulte [ROS2_JAZZY.md](../ros2/ROS2_JAZZY.md).
Se Gazebo, SITL e MAVProxy já estiverem funcionando, mantenha-os abertos e siga
para a etapa 2. Não abra uma segunda instância dos mesmos serviços.

### Terminal 1 — Gazebo e sensores

Para um ensaio começando em frente à entrada:

```bash
cd ~/ardupilot/Rover
./ros2/service.sh gazebo world:=gravel_plane N:=1 \
  start_x:=3.2 start_y:=-2 \
  mid360_enabled:=true samsung_s23_ultra_cam_enabled:=true \
  insta360_enabled:=false hokuyo_enabled:=false
```

Esse exemplo posiciona o rover 1,8 m antes do centro do contêiner, apontando
para sua abertura. O centro está em `(5, -2)` m e a abertura em `x=4,5 m`.
Os argumentos de posição só atuam ao iniciar o mundo.

Mantenha o Gazebo executando, sem pausar a física. O MID360 fica disponível para
a alternativa LiDAR; a entrada por QR usa a câmera Samsung.

### Terminal 2 — ArduPilot, Lua e MAVProxy

Para reproduzir o perfil sem desvio usado na validação da entrada:

```bash
cd ~/ardupilot/Rover
./sim_1_gzrover.sh --foreground --add-param-file nrover_sem_desvio.param
```

Aguarde GPS, EKF e verificações PreArm saudáveis. Nas próximas inicializações,
se o firmware já estiver compilado, pode acrescentar `--no-rebuild`.

Este terminal também apresenta o prompt do MAVProxy. Os comandos `mode hold`,
`arm throttle` e `mode guided` usados adiante são digitados **nesse prompt**, não
no Bash. O lançador já configura a saída local UDP 14551 e `mavfwd true` para o
docking; QGroundControl recebe pela porta 14550.

Para usar o perfil com desvio, selecione `nroverobst.param` no lugar do arquivo
acima e inicie, em outro terminal:

```bash
cd ~/ardupilot/Rover
./ros2/service.sh obstaculos
```

Nesse perfil, `PRX1: No Data` indica falta dos dados de proximidade. Verifique
MID360 e o serviço de obstáculos antes de armar. A série de seis entradas
registradas foi executada com o perfil sem desvio.

### Terminal 3 — Imagem da câmera, opcional

```bash
cd ~/ardupilot/Rover
./ros2/service.sh camera
```

A janela pode permanecer aberta durante entrada e saída. Para o fluxo por QR,
Gazebo, SITL/MAVProxy e câmera são suficientes; FAST-LIO, mapa, MAVROS e RViz
não são dependências do controlador visual.

## 2. Verificar a câmera antes de abrir o monitor

Em um terminal livre, sem outro controlador de docking executando:

```bash
cd ~/ardupilot/Rover
./ros2/service.sh docking-qr --diagnostico
```

O diagnóstico informa recepção de imagem, calibração e posições das juntas,
além da leitura do QR. Ele termina sem armar nem mover o rover. A câmera
funcionar não significa que o QR já esteja decodificável naquela posição.

O cadastro das estações está em [estacoes_recarga.txt](../../estacoes_recarga.txt),
com os pontos a 4 m e 2 m da abertura e o código QR. Consulte o
[formato e a interpretação do eixo](CADASTRO_ESTACOES.md). QR e LiDAR usam o mesmo TXT.
O código esperado neste cenário é `ROVER_DOCKING_STATION_01`.

## 3. Acionar a entrada automática

### Terminal 4 — Manter o monitor aberto

```bash
cd ~/ardupilot/Rover
./ros2/service.sh docking-qr
```

Esse comando inicia `entrar_container.py`, que utiliza o monitor de
`aguardar_docking.py`. Ele não cria, carrega ou altera missão e não exige que
o último waypoint tenha sido concluído.

O gatilho exige telemetria recente, GPS 3D, rover armado, modo **GUIDED (15)** e
distância horizontal **menor que 8 m do centro** de uma estação cadastrada.
Deixe o rover parado antes da ativação. A distância permite iniciar a tentativa;
o controlador ainda precisa reconhecer o QR e validar a pose para entrar.

### Terminal 2 — Ativar pelo MAVProxy

Com o rover na área livre em frente à abertura, digite no MAVProxy:

```text
mode hold
```

Aguarde o rover parar. Depois:

```text
arm throttle
```

Aguarde a confirmação de armamento. Então:

```text
mode guided
```

Também é possível armar e selecionar GUIDED pelo QGroundControl. O gatilho é o
modo efetivo informado pelo ArduPilot; não é necessário editar a missão.

### O que acontece durante a entrada

1. O controlador verifica sensores, procura e decodifica o QR cadastrado.
2. Alcança um ponto no eixo da estação, ainda fora da abertura. Para para
   corrigir o rumo e confirma novamente o erro lateral antes de entrar.
3. Quando necessário, recua fora da abertura para refazer a aproximação.
4. Entra em GUIDED, reduz a velocidade perto do centro e solicita HOLD.
5. Confirma parada e pose final, desarma e restaura os parâmetros temporários.
6. O monitor permanece aberto, aguardando outra ativação.

Os limites padrão são **0,25 m/s na aproximação** e **0,18 m/s na entrada**, com
redução progressiva perto do centro. Para usar um limite menor, encerre o monitor
anterior com Ctrl+C e reinicie, por exemplo:

```bash
./ros2/service.sh docking-qr --velocidade 0.15
```

A mensagem de conclusão inclui `Rover parado e desarmado confirmado.`. Confira
HOLD e desarmamento também no QGroundControl. Se houver falha na confirmação da
pose, o rover pode permanecer em HOLD e armado; leia o motivo no terminal.

## 4. Trocar da entrada para a saída

**Encerre o monitor de entrada antes de iniciar a saída**, mesmo que a entrada
já tenha terminado. No Terminal 4, pressione **Ctrl+C** e espere o prompt do
Bash voltar. Se houver uma manobra em andamento, aguarde as mensagens de HOLD
e restauração dos parâmetros.

Entrada e saída compartilham UDP 14551. O lançador também aplica um bloqueio
comum chamado `docking-controller`, para impedir controladores simultâneos.
Não apague o arquivo de bloqueio para contornar um processo ainda ativo.

## 5. Acionar a saída do contêiner

Antes de executar, o rover deve estar:

- Dentro do contêiner, próximo do centro e voltado para o QR.
- Parado em **HOLD (4)**. Pode estar desarmado, como fica após a entrada.
- Com a câmera recebendo imagens e o QR reconhecível.
- Com a área atrás da abertura e a área de giro livres.

No Terminal 4, agora disponível:

```bash
cd ~/ardupilot/Rover
./ros2/service.sh sair-qr
```

A saída **não espera você selecionar GUIDED**. Ao ser iniciada, verifica a pose
do rover dentro da estação; se estiver desarmado, solicita armamento normal e
só prossegue se o ArduPilot aceitar. A própria rotina muda para GUIDED.

A sequência é:

1. Recuar aproximadamente **2 m a partir da posição inicial**, mantendo o QR
   à frente e corrigindo alinhamento durante o recuo.
2. Solicitar HOLD e confirmar distância e alinhamento após parar.
3. Girar **180°**, usando o rumo do ArduPilot.
4. Solicitar HOLD, confirmar parada, desarmar e encerrar o processo.

A velocidade padrão de recuo é **0,08 m/s**; o limite aceito é 0,12 m/s. Exemplo
com velocidade explícita e prazo de até 300 s por etapa:

```bash
./ros2/service.sh sair-qr --velocidade 0.08 --timeout 300
```

O término normal apresenta `Saida concluida: HOLD, giro ... graus e rover
desarmado.`. A missão permanece carregada. Para outra entrada, abra novamente
o monitor e reposicione o rover em frente à estação antes de ativar GUIDED.

**Estado da validação:** a revisão de entrada por QR completou seis ensaios no
Harmonic. A saída tem testes de lógica, mas não foi revalidada em movimento
nessa revisão. Consulte os [resultados e limites](VALIDACAO_ALINHAMENTO_VELOCIDADE_ROS2.md).

## 6. Interromper ou repetir uma tentativa

Para interromper uma rotina, use **Ctrl+C no terminal do docking**. Ela tenta
parar o rover e restaurar os parâmetros que alterou. O Gazebo continua aberto.
Para solicitar parada pelo MAVProxy:

```text
mode hold
```

Após uma tentativa de entrada malsucedida, leia o erro, mantenha HOLD até o
rover parar e corrija a causa. Com o monitor ativo e o rover novamente em uma
posição adequada, confirme armamento e selecione GUIDED para tentar de novo.

Se for encerrar a simulação inteira, termine primeiro a rotina de docking;
depois encerre câmera, SITL/MAVProxy e Gazebo nos respectivos terminais.

## 7. Problemas frequentes

| Mensagem/situação | O que conferir |
| --- | --- |
| `Servico ja esta rodando: docking-controller` | Encerre o monitor de entrada ou a rotina de saída anterior com Ctrl+C e espere o processo terminar |
| `aguardando HEARTBEAT` | SITL/MAVProxy ativo e saída UDP 14551 configurada; nenhum outro programa de missão/docking usando essa porta |
| `rover desarmado` na entrada | Arme no MAVProxy/QGroundControl depois que as verificações PreArm estiverem saudáveis |
| `aguardando GUIDED` | Selecione GUIDED efetivo; AUTO não aciona este monitor |
| `fora dos 8 m` | Aproxime o rover da estação cadastrada antes de ativar o docking |
| `Aguardando QR` | Confira câmera Samsung, visibilidade do QR, identificação cadastrada e diagnóstico; somente ver o desenho não confirma sua decodificação |
| `Inicie com Rover em HOLD e parado` na saída | Solicite `mode hold` no MAVProxy e aguarde a parada |
| `Pose nao confirma rover dentro...` | A saída exige o rover dentro e alinhado; não serve para aproximá-lo de fora |
| Imagem ou juntas desatualizadas | Confira física do Gazebo executando e câmera habilitada; rode o diagnóstico após encerrar o controlador |

Registros de entrada: `Rover/python/run/docking/<data>/`.
Registros de saída: `Rover/python/run/docking/saida_<data>/`.
Os valores de parâmetros anteriores ficam nesses diretórios e são úteis caso
o processo seja encerrado à força antes de restaurá-los.

## 8. Alternativa: entrada e saída por LiDAR

Use esta alternativa separadamente do controlador por QR. Mantenha MID360
habilitado no Gazebo e inicie os dois serviços abaixo, **um em cada terminal**:

```bash
cd ~/ardupilot/Rover
./ros2/service.sh fastlio
```

```bash
cd ~/ardupilot/Rover
./ros2/service.sh mapa
```

Aguarde a inicialização com o rover parado. A rotina usa a nuvem local recente
`/rover_slam_3d/local_cloud` e a odometria `/fast_lio/odometry`.

Com os demais controladores de docking encerrados, no terminal de controle:

```bash
cd ~/ardupilot/Rover
./ros2/service.sh docking-lidar --diagnostico
```

Depois que o diagnóstico terminar:

```bash
./ros2/service.sh docking-lidar
```

A entrada aguarda GUIDED, rover armado e estação a menos de 8 m. Essa rotina
encerra após a tentativa. Para sair, com o processo anterior encerrado e o
rover dentro da estação, parado em HOLD:

```bash
./ros2/service.sh sair-lidar
```

Os seis ensaios da revisão visual não validam essas rotinas LiDAR. Veja os
pré-requisitos e limites em [Docking por LiDAR](DOCKING_LIDAR.md).
