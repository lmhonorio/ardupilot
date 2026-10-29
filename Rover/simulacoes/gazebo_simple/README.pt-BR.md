# Rover simples no Gazebo Classic

Para escolher e copiar as chamadas de cada modo, consulte o
[Manual de simulação, Lua e autotuning](../MANUAL_SIMULACAO.md).

Cenario para o Gazebo Classic 11 instalado nesta maquina. Contem um Husky
controlado pelo ArduPilot, piso plano de 120 x 120 m e dez obstaculos estaticos
com colisao: seis caixas, dois cilindros e duas paredes. O robo inicia na
origem, apontando para +X. A primeira caixa fica em X=10 m.

A origem WGS84 do mundo e SEParnaiba, conforme Tools/autotest/locations.txt:
latitude -3.123150, longitude -41.764495, altitude 77 m. O SITL usa o mesmo
local. No mundo, +X aponta para norte, +Y para oeste e +Z para cima, alinhado
com a conversao NED do plugin. O heading geografico foi conferido com a
transformacao SphericalCoordinates do Gazebo Classic instalado. A superficie
continua sendo um piso simples, georreferenciado nessa origem.

As coordenadas X/Y dos cinco obstaculos anteriores foram multiplicadas por
dois (distancias entre centros duplicadas), preservando suas dimensoes.
Foi acrescentada uma segunda copia do conjunto, deslocada 24 m em +Y.

| Obstaculo | Centro X/Y original ampliado (m) | Segunda copia X/Y (m) |
| --- | --- | --- |
| Caixa frontal | 10 / 0 | 10 / 24 |
| Caixa esquerda | 6 / 6 | 6 / 30 |
| Cilindro direito | 8 / -6 | 8 / 18 |
| Parede verde | 20 / 0 | 20 / 24 |
| Caixa traseira | -8 / -4 | -8 / 20 |

Os arquivos novos ficam neste diretorio e em `../sim_gz_simple.sh`.
O repositorio `/home/grin/ardupilot_gazebo` e usado somente para leitura:
modelo Husky, suas malhas e plugin compilado. O modelo `rover_mid360` foi
adaptado de `models/rover_ardupilot/model.sdf` desse repositorio, mantendo
os canais das quatro rodas e a IMU do piloto automatico. Os servos de
velocidade das rodas do modelo local usam P=10, I=0, D=0.02 e torque
limitado a +/-20 N m por junta. Essa configuracao entra ao reiniciar o
Gazebo; nao altera os ganhos ATC_* do ArduRover nem o rover real.
Veja os ensaios e as limitacoes em [Validacao da alocacao](ALLOCATION_VALIDATION.md).

## Executar

A partir de `/home/grin/ardupilot`, inicie tudo com um comando:

```bash
bash Rover/start_gz_simple.sh
# Reutilizar o firmware que ja esta compilado:
bash Rover/start_gz_simple.sh --no-rebuild
# Sem janelas, mantendo renderizacao disponivel via DISPLAY ou Xvfb:
bash Rover/start_gz_simple.sh --headless --no-rebuild
```

## Usar a alocacao Lua sem sim_vehicle.py

```bash
cd /home/grin/ardupilot/Rover
./start_gz_simple.sh --lua --no-rebuild
```

Essa opcao inicia diretamente `build/sitl/bin/ardurover --model gazebo-rover`
e o MAVProxy, ambos sem passar por sim_vehicle.py. Sem --no-rebuild, compila
o firmware usando waf e Python 3.8 antes de iniciar. O ArduRover continua
necessario: seus controladores produzem os comandos de aceleracao e giro,
e o Lua faz a mistura desses comandos para os motores. Nao e uma execucao
standalone do Lua fora do autopiloto.

Os scripts atuais de Rover/scripts/ sao copiados a cada inicio para
gazebo_simple/run/lua/scripts/ (apenas system-start.lua,
rover-controlallocation.lua e os modulos functions.lua e pid.lua). O estado
EEPROM e os logs desse modo ficam em run/lua/, separados do modo nativo.
Edicoes nos fontes entram na proxima partida.

O firmware carrega rover_simple.parm e rover_lua.parm. Um inicializador
gerado apenas para esse SITL espera os parametros dinamicos, aplica o perfil
selecionado enquanto desarmado e habilita BATT_SOC_ALLOC=1. O perfil inclui WP_SPEED=0.4 m/s, WP_RADIUS=0.2 m,
CRUISE_THROTTLE=24 e os limites de curva selecionados. Nao e necessario
apagar a EEPROM do modo nativo nem configurar os parametros manualmente.
Se SCR_ENABLE for posteriormente desabilitado manualmente nessa EEPROM Lua,
habilite-o novamente e reinicie para o inicializador voltar a executar.

Aguarde a mensagem no terminal:

```text
Gazebo: Lua allocation enabled; selected parameters applied
```

No prompt MAVProxy, consulte `param show BATT_SOC_ALLOC`,
`param show WP_SPEED` e `param show WP_RADIUS` para conferir a configuracao.
O QGroundControl continua recebendo UDP 14550 e o utilitario de missao usa
14555, ambos encaminhados pelo MAVProxy.
As entradas RC simuladas de direcao/aceleracao iniciam em 1500 (neutro),
para o canal 3 nao herdar o valor 1000 usado pelo SITL generico.

Para enviar a missao e aguardar enquanto prepara o QGC, em outro terminal:

```bash
cd /home/grin/ardupilot/Rover
./mission_tune.sh --radius 3 --speed 0.4 --wp-radius 0.2 --execute --wait-qgc
```

No QGC: Plan -> File -> Download, depois Fly. Pressione ENTER no terminal
da missao para partir. Ctrl+C no lancador encerra o cenario; em outro
terminal, `./start_gz_simple.sh --stop` tambem funciona.

Foi validada a inicializacao real do Gazebo/SITL/MAVProxy neste modo, sem
processo sim_vehicle.py, incluindo carregamento dos scripts e leitura dos
parametros acima. Em ensaio posterior com Lua ativo e atuadores ajustados,
o rover completou circulo de raio 3 m e quadrado de lado 6 m em area livre,
com referencia de velocidade de 0.4 m/s. Esse ensaio nao comprova uma
melhora comparativa no cenario com obstaculos nem ganhos otimos;
os detalhes estao no registro de validacao acima.

O lancador aguarda o mundo, inicia camera/LiDAR e conecta o SITL com MAVProxy.
O prompt MAVProxy fica no terminal atual; Ctrl+C encerra as sessoes de
processos iniciadas por ele, incluindo gzserver/gzclient mesmo quando o
Gazebo cria outros grupos de processos.

Para encerrar por outro terminal:

```bash
bash Rover/start_gz_simple.sh --stop
```

Esse comando aguarda o encerramento e a liberacao do bloqueio. O arquivo
stack.lock pode continuar no disco; apenas sua existencia nao impede novas
execucoes. O descritor do bloqueio pertence exclusivamente ao lancador e
nao e herdado pelos processos da simulacao. Nao e necessario iniciar outro MAVProxy para
encaminhar telemetria. O script usa um lock para impedir duas execucoes
simultaneas e verifica portas/SITL existentes antes de iniciar.

Se wxPython estiver disponivel no Python 3.8, abre tambem console/mapa como
no script de referencia, com MAP_SERVICE=GoogleSat. Neste Python 3.8 o modulo
wx nao esta instalado; nesse caso usa o prompt MAVProxy, e o QGroundControl
pode mostrar o posicionamento recebido por UDP.

Gazebo registra em `gazebo_simple/run/gazebo.log`, SITL/MAVProxy em
`gazebo_simple/run/sitl.log`, os sensores em `gazebo_simple/run/sensors.log`
e os frames em `gazebo_simple/run/sensors/`. Se o QGroundControl nao
conectar, confirme no sitl.log que apareceu `Detected vehicle 1:1` e que o
SITL nao encerrou com erro. O QGroundControl local deve escutar UDP 14550;
foi confirmado nesta maquina com a aplicacao aberta e recebendo o rover.
A preparacao do EKF/GPS pode levar algum tempo, especialmente sob renderizacao
por software. Aguarde GPS/EKF prontos antes de interpretar a posicao estimada.

Para iniciar os componentes separadamente, use dois terminais:

```bash
# Terminal 1: mundo, robo e sensores
bash Rover/sim_gz_simple.sh gazebo

# Terminal 2: piloto automatico e MAVProxy
bash Rover/sim_gz_simple.sh sitl
# Opcional, quando houver suporte aos terminais graficos:
# bash Rover/sim_gz_simple.sh sitl --console --map
```

O lancador usa explicitamente Python 3.8, incluindo sim_vehicle, MAVProxy e
processos de build que usam python/python3. Nesta maquina resolve para
`/home/grin/.pyenv/versions/3.8.10/bin/python3.8`. Para indicar outro Python
3.8, use `PYTHON38=/caminho/python3.8` antes do comando. Outros scripts do
Rover e o Python global da maquina nao sao alterados.

Inicie o Gazebo primeiro. O modelo `gazebo-rover` conecta o SITL a fisica
externa; `rover-skid` sozinho usa a fisica interna. A instancia e 0 e o SYSID
1. O plugin recebe atuadores em UDP 9002 e envia estado em UDP 9003.
Encerre outras instancias que estejam usando essas portas antes de iniciar.
As saidas MAVLink sao a uniao dos destinos dos dois comandos ativos de
`sim_1rover_new.sh`, mais a porta 14555 ja usada neste cenario:

| IP destinatario | Portas UDP |
| --- | --- |
| 127.0.0.1 | 14550, 14551, 14552, 14555, 14560, 14570 |
| 192.168.0.131 | 14550, 14551 |
| 192.168.0.136 | 14551 |
| 192.168.0.170 | 14570 |

Todos recebem o fluxo MAVLink do rover SYSID 1, incluindo posicao,
orientacao, velocidade e heartbeat. Os destinos repetidos do script de
referencia foram consolidados. `--no-extra-ports` evita a saida 14550
adicional que sim_vehicle criaria automaticamente.

Exemplo de receptor de posicionamento com Python 3.8 em uma porta livre:

```python
from pymavlink import mavutil
link = mavutil.mavlink_connection('udpin:0.0.0.0:14552')
link.wait_heartbeat()
while True:
    pos = link.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    print('lat/lon:', pos.lat / 1e7, pos.lon / 1e7,
          'altitude AMSL:', pos.alt / 1000,
          'altitude relativa:', pos.relative_alt / 1000)
```

As altitudes sao em metros apos a conversao acima. Durante a inicializacao,
GLOBAL_POSITION_INT pode conter estimativas ainda sem GPS/EKF validos.
Acompanhe GPS_RAW_INT.fix_type >= 3 e a inicializacao do EKF. Em outro
computador, escute na porta destinada ao respectivo IP da tabela. O envio
foi configurado para os IPs da LAN; recebimento remoto depende desses hosts
e sua rede. Camera e LiDAR continuam nos topicos/arquivos proprios, nao sao
incluidos nesse fluxo MAVLink.

O lancador completo fixa LOCATION=SEParnaiba. Se alterar manualmente a
localizacao do SITL, atualize tambem spherical_coordinates no mundo para
manter a mesma referencia geografica.
Para outra instalacao do plugin: defina `ARDUPILOT_GAZEBO_DIR` nos comandos.
A geometria usa metros em relacao a origem SEParnaiba.
Logs e EEPROM deste cenario ficam em `gazebo_simple/run/` (ignorado pelo Git).
Para o primeiro teste, use esse estado novo; parametros salvos em execucoes
posteriores podem exigir revisao. O lancador nao apaga EEPROM de outros cenarios.

No MAVProxy, apos inicializacao do EKF e verificacoes de armamento:

```text
mode manual
arm throttle
rc 3 1550
rc 3 1500
```

O ultimo comando neutraliza o acelerador. Use `disarm` para desarmar.
Nao foi configurada navegacao autonoma ou desvio automatico de obstaculos.

## Sensores

- Camera RGB frontal: 640 x 480, 20 Hz, FOV horizontal 1.2 rad, clipping
  de 0.1 a 100 m. Orientada para +X; montada em X=0.52, Z=0.27 no quadro
  do modelo, acima da frente do chassis. Publicacao nativa do Gazebo.
- LiDAR: sensor `ray` 3D, 1000 azimutes x 20 elevacoes a 10 Hz
  (200 mil medidas/s), campo horizontal completo sem duplicar o raio da
  emenda, vertical de -7 a +52 graus, alcance de 0.1 a 40 m e ruido
  gaussiano de 2 cm. Montado acima do chassis em Z=0.40, feixes a Z=0.431
  no quadro do modelo. Corpo visual com 65 x 65 x 60 mm e massa de 265 g.
  `visualize` fica desativado para evitar desenhar 20 mil raios na interface.

Referencia: https://www.livoxtech.com/mid-360/specs

Esta e uma aproximacao geometrica do MID360 com grade regular. O hardware
usa varredura nao repetitiva. Nao ha simulacao de intensidade/reflectividade,
retornos multiplos, tempo individual por ponto, protocolo UDP Livox ou IMU
interna do MID360; a IMU existente pertence a interface do ArduPilot. O limite
fixo de 40 m nao reproduz a variacao real de alcance com reflectividade.
Nao e um substituto direto do driver Livox nem uma validacao de FAST-LIO.

Liste os topicos apos iniciar o mundo:

```bash
gz topic -l
# Dados 3D nativos: gazebo.msgs.LaserScanStamped, nao PointCloud2 ROS.
gz topic -e /gazebo/rover_simple/rover_mid360/mid360_link/mid360/scan
```

Para visualizar a camera, abra o visualizador de imagens do Gazebo e selecione
`/gazebo/rover_simple/rover_mid360/cam_link/front_camera/image`.

O LaserScanStamped nativo inclui angulos horizontais e verticais e a grade
completa de distancias. Para gerar XYZ, para cada distancia r use
`x=r*cos(elevacao)*cos(azimute)`, `y=r*cos(elevacao)*sin(azimute)` e
`z=r*sin(elevacao)`, ignorando retornos invalidos/fora do alcance.
A camera exige renderizacao OpenGL e um display funcional mesmo com gzserver.
ROS nao e necessario para este mundo. O leitor Python descrito abaixo usa
o transporte nativo do Classic. Ainda nao ha ponte para imagens ROS,
PointCloud2, TF ou mensagens MAVLink OBSTACLE_DISTANCE. Ativar PRX/AVOID
sozinho nao conecta os dados do LiDAR ao ArduPilot.

Se o desempenho cair, reduza `horizontal/samples`, `vertical/samples` ou
`update_rate` em `models/rover_mid360/model.sdf`.

## QR code na parede verde

O painel fica na face voltada ao rover (X=19.845 m), em Y=2 m e Z=0.95 m.
A parede esta em X=20 m. O painel mede 1.8 x 1.8 m incluindo a margem branca.
Foi deslocado lateralmente para evitar a oclusao pela caixa em X=10, Y=0.
Com o novo espacamento o painel ficou a cerca de 20 m da origem; aproxime o
rover para ter resolucao suficiente para decodificar o QR com confianca.
Seu conteudo e `ROVER_PAREDE_VERDE_01`. E um QR code, diferente dos marcadores
ArUco usados em `FuncoesPython/testopenmvcamera.py`; aquele detector ArUco
nao detecta este QR, e `VideoCapture(0)` abre a webcam, nao a camera do Gazebo.

Para trocar o texto da textura e reiniciar o Gazebo:

```bash
python3.8 Rover/python/simulation/generate_qr.py 'MEU_ALVO_01'
```

## Acessar os dados em Python 3.8

Para visualizar a nuvem XYZ em 3D, com o lançador já lendo os sensores:

```bash
cd /home/grin/ardupilot/Rover
python3.8 python/lidar/view_lidar.py
```

A janela atualiza a cada segundo. Arraste com o mouse para girar a vista;
o X vermelho é o sensor e as cores indicam altura. A altura aparece ampliada
5 vezes para facilitar a leitura; use `--z-scale 1` para escala física.
Fechar o visualizador
não encerra a simulação. Para salvar uma imagem sem abrir janela:

```bash
python3.8 python/lidar/view_lidar.py --save gazebo_simple/run/lidar_preview.png
```

Com o mundo executando, abra um terceiro terminal:

```bash
# Mostrar camera, tentar ler o QR e salvar os dados mais recentes:
bash Rover/sim_gz_simple.sh sensors --timeout 0
# Sem janela: capturar uma imagem e uma varredura, depois encerrar:
bash Rover/sim_gz_simple.sh sensors --no-gui --once
```

O lancador compila uma pequena ponte C++ na primeira execucao, usando g++,
pkg-config e a biblioteca de desenvolvimento do Gazebo ja instalada.
`sensor_bridge.cc` assina os topicos nativos e envia mensagens binarias para
`read_sensors.py`, que usa NumPy e OpenCV do Python 3.8. Nao requer rospy,
pygazebo, driver Livox ou novos pacotes Python.

Os arquivos mais recentes sao salvos em `Rover/gazebo_simple/run/sensors/`:

| Arquivo | Dados |
| --- | --- |
| `camera.png` | Imagem RGB gravada como PNG; OpenCV le em BGR. |
| `camera.json` | Timestamp simulado, largura, altura, stride e formato da imagem. |
| `lidar.npz` | `ranges`: matriz 20 x 1000 em metros; `xyz`: pontos validos N x 3; `metadata`: JSON com angulos, limites e timestamp. |
| `lidar.json` | Mesmos metadados da varredura em texto. |

Os arquivos sao sobrescritos no maximo uma vez por segundo por sensor;
nao constituem uma gravacao completa de todos os frames de 20/10 Hz.
Use `--output-dir /tmp/captura` para mudar o destino. `--timeout` e tempo
real total (padrao 30 s; 0 ilimitado). A tecla q fecha a visualizacao;
Ctrl+C encerra o leitor. Camera e LiDAR tem timestamps independentes, sem
sincronizacao automatica.

Exemplo para usar a captura no seu codigo:

```python
# Executar com python3.8
import json
from pathlib import Path
import cv2
import numpy as np

folder = Path('/home/grin/ardupilot/Rover/gazebo_simple/run/sensors')
bgr = cv2.imread(str(folder / 'camera.png'))
print('Imagem BGR:', bgr.shape)
with np.load(folder / 'lidar.npz') as scan:
    ranges = scan['ranges']  # [elevacao, azimute], metros
    xyz = scan['xyz']        # N x 3, metros, somente retornos validos
    metadata = json.loads(str(scan['metadata']))
    print(ranges.shape, xyz.shape, metadata['sec'], metadata['nsec'])
```

`ranges` pode conter infinito ou o alcance maximo quando nao houve retorno;
esses valores nao entram em `xyz`. Os XYZ estao no quadro do sensor: +X para
frente, +Y para esquerda, +Z para cima (nao sao coordenadas GPS nem NED).
A menor distancia global pode incluir solo/corpo do proprio rover; para
medir um obstaculo frontal, selecione os azimutes e elevacoes de interesse.
Para acesso continuo em memoria, adapte o loop de `read_sensors.py` e use
`decode_camera`/`decode_lidar`; ali chegam as mensagens completas antes da
limitacao de salvamento. Para processar todos os frames, retire esse limite
ou separe captura e processamento se o consumidor nao acompanhar a taxa.

A imagem fornece pixels; nao fornece profundidade por pixel. Para estimar
distancia pela camera, use o tamanho fisico do marcador e calibracao/solvePnP,
ou projete a nuvem LiDAR na imagem usando os extrinsecos dos dois sensores.
O leitor apenas decodifica o texto do QR, sem enviar comandos ao rover.

## Analise dos scripts existentes

| Scripts em Rover | Comportamento e observacoes |
| --- | --- |
| `sim_1rover_new.sh`, `sim_nrover_new.sh`, `sim_2roverb.sh` | SITL interno: nao passam `--model gazebo-rover`. O primeiro altera PATH para retirar Conda; abre outro MAVProxy e envia parametros PRX/AVOID, sem ligar um sensor Gazebo. |
| `sim_1roverb.sh` | Comando ativo contem `python3.8 .../sim_vehicle.py sim_vehicle.py`; o segundo `sim_vehicle.py` e argumento indevido. Usa caminhos relativos. |
| `simrover.sh`, `simboat.sh`, `sim_2rover.sh`, `sim_mvehicle.sh` | SITL interno; scripts multi-robo criam varias instancias/encaminhadores MAVLink. |
| `gzrover.sh`, `gzroverv2.sh`, `gzboat.sh`, `sim_gz1rover.sh` | Selecionam backend `gazebo-rover`; dependem de um mundo iniciado separadamente. Em `sim_gz1rover.sh` o comando de abrir Gazebo esta comentado. |
| `sim_gz2rover.sh` | Tem `f rover-skid` sem hifen antes de `-I`; exige correcao para uso. Dois veiculos tambem exigem modelos com portas distintas. |
| `gzroverv2multi.sh`, `start_gzroverv2multi.sh` | Lancam varias instancias Gazebo/SITL, mas nao criam os modelos correspondentes no mundo. O fallback sem terminal grafico bloqueia no primeiro processo. O primeiro duplica destinos IP de saida. |

Os scripts existentes usam varios IPs fixos da LAN, parametros completos e
caminhos relativos. Nenhum deles foi alterado nesta implementacao.

No `ardupilot_gazebo`, o CMake usa `find_package(gazebo)` e o plugin
`libArduPilotPlugin.so`, confirmando Classic (nao Gazebo Sim/Harmonic).
`models/rover_ardupilot` inclui `model://husky`, que esta em `models_gazebo`;
ambos os caminhos precisam estar em GAZEBO_MODEL_PATH. A camera original e
uma chase camera atras do robo, 1920 x 1080 a 60 Hz, com always_on desligado.
O mundo original tem piso enorme/pista, sem o conjunto simples de obstaculos.
Os parametros antigos misturam nomes de versoes diferentes, calibracoes e
configuracoes Lua; o novo cenario usa um arquivo minimo e o frame Gazebo
padrao do ArduPilot para evitar herdar essas dependencias.

## Verificacao

```bash
bash -n Rover/sim_gz_simple.sh
bash Rover/sim_gz_simple.sh check
```

A validacao SDF exige acesso as interfaces de rede do sistema, mesmo sendo
uma verificacao local. Teste de publicacao de sensores e teste de movimento
com SITL sao verificacoes separadas da validacao XML/SDF.

Verificado nesta maquina com Gazebo 11.15.1: sintaxe Bash, XML/SDF,
carregamento do mundo e do plugin ArduPilot, compilacao da ponte nativa e
recebimento real dos dados dos dois sensores em Python 3.8. A mensagem LiDAR
confirmou 1000 azimutes x 20 elevacoes; a imagem confirmou 640 x 480.
A conversao XYZ foi conferida com raios de direcoes conhecidas e descarte de
retorno infinito. O QR foi validado na textura e na camera do cenario anterior
(parede a 10 m); agora a parede esta a 20 m, exigindo maior aproximacao.

No teste integrado do novo lancador, foram recebidas mensagens
GLOBAL_POSITION_INT, SYSID 1 e latitude/longitude de SEParnaiba em todas as
seis portas UDP locais (14550, 14551, 14552, 14555, 14560, 14570). Apos
a inicializacao do GPS/EKF, a altitude AMSL recebida foi aproximadamente
76.92 m, consistente com a origem de 77 m. O teste utilizou --no-rebuild
com o firmware existente. O lancador
iniciou e encerrou Gazebo, SITL e sensores. Recepcao nos computadores remotos
e movimento comandado do rover nao foram validados.

## Missao parametrica e avaliacao do controle

O utilitario ../mission_tune.sh gera um circulo de raio configuravel e um
quadrado circunscrito (lado 2*raio), e pode executar a missao e comparar
parametros via saida UDP do MAVProxy. Veja ../mission_tune/README.pt-BR.md.
Geracao sem movimento: `bash Rover/mission_tune.sh --radius 3` a partir da
raiz do repositorio. --execute habilita envio/execucao; --optimize habilita
comparacao de candidatos. O teste nao usa o LiDAR para evitar obstaculos.
