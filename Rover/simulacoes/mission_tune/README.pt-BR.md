# Missao circular, quadrado e sintonia pelo MAVProxy

As chamadas completas de SITL, Gazebo e autotuning estão no
[Manual de simulação](../MANUAL_SIMULACAO.md).

Python 3.8. O raio e o parametro principal. Um circulo de raio R e seguido
por um quadrado circunscrito de lado 2R, com o mesmo centro. A missao usa
waypoints NAV_WAYPOINT, sem comandos de decolagem, e termina com uma parada.
O circulo e aproximado por 36 segmentos (configuravel), no sentido horario.

## Gerar e executar

A partir de `/home/grin/ardupilot`:

```bash
# Apenas gerar o plano; nao conecta nem movimenta o rover:
bash Rover/mission_tune.sh --radius 3

# Com Gazebo/SITL/MAVProxy ja iniciados pelo start_gz_simple.sh:
# Enviar, executar uma vez, coletar e analisar; restaurar parametros ao final.
bash Rover/mission_tune.sh --radius 3 --execute

# Baseline, ate dois candidatos e repeticao de validacao do melhor candidato:
bash Rover/mission_tune.sh --radius 3 --execute --optimize

# Mesmo procedimento com outra geometria:
bash Rover/mission_tune.sh --radius 5 --execute --optimize
```

## Acompanhar pelo QGroundControl

Mantenha `Rover/start_gz_simple.sh` em execucao. O MAVProxy envia a
telemetria ao QGC deste computador em UDP 14550; o script usa outra porta,
14555. Se aparecer Communication Lost, confirme que a simulacao esta rodando.

Para preparar a missao e aguardar enquanto voce ajusta o QGC:

```bash
bash Rover/mission_tune.sh --radius 3 --execute --wait-qgc
```

Quando o terminal disser que a missao foi enviada, no QGC 4.2.9 abra
**Plan -> File -> Download** e confirme a substituicao do plano local, se
solicitado. Isso le os pontos do rover e atualiza uma missao antiga que
tenha ficado no QGC. Volte para **Fly** para acompanhar a posicao e o
progresso. Aumente o zoom: o circulo tem apenas 6 m de diametro quando
o raio e 3 m, e os numeros dos waypoints podem ficar sobrepostos.

Depois pressione ENTER **no terminal do script**, para armar e iniciar.
A espera nao inicia automaticamente e mantem a comunicacao pelo MAVProxy.
O QGC fica disponivel para observacao; deixe o script conduzir a execucao.
Ctrl+C no terminal solicita parada, desarmamento e restauracao dos parametros.

Para apenas enviar e conferir o plano, sem armar nem iniciar:

```bash
bash Rover/mission_tune.sh --radius 3 --upload-only
```

Toda geracao tambem produz `mission.plan`, no formato nativo do QGC.
Para visualizar um plano offline, use **Plan -> File -> Open** e escolha
esse arquivo na pasta de resultados. Para acompanhar uma execucao, prefira
Download depois do envio: a origem GPS efetiva pode mudar entre execucoes.
O arquivo exportado contem a missao, sem cercas ou pontos de rally.

`--radius` e em metros. Raio 3 gera lado 6; raio 5 gera lado 10.
Comprimento, tempo nominal, limite de tempo padrao e referencias da analise
sao recalculados a partir da geometria e da velocidade.
Nao sao gravadas alteracoes em rover_simple.parm ou gzrover.param. O novo
script ajusta parametros do autopiloto pela telemetria; so conserva os ganhos
candidatos se a melhora for confirmada e a missao terminar completamente.
Os parametros temporarios de navegacao e GCS_PID_MASK sao restaurados ao
final com confirmacao. Os intervalos de telemetria anteriores sao consultados
quando suportados e seu restauro e solicitado ao fechar o enlace.

O centro padrao e a posicao GPS atual no inicio da execucao. Para desloca-lo,
use `--center-north METROS --center-east METROS`; os eixos sao norte/leste,
nao os eixos do Gazebo (que usa +Y oeste). Mantenha todo o trajeto e a
aproximacao em uma area livre. O script nao usa o LiDAR para desviar de
obstaculos. Alterar o raio nao move os obstaculos do mundo.

A geracao offline usa SEParnaiba por padrao e permite `--origin-lat` e
`--origin-lon`. Na execucao conectada, a origem e medida pelo GPS/EKF e
a geometria e regenerada; os arquivos finais refletem a origem efetiva.

Outras opcoes:

| Opcao | Padrao / significado |
| --- | --- |
| `--radius` | Obrigatorio, raio em metros; em execucao, pelo menos 0.5 m |
| `--upload-only` | Envia e confere o plano; nao arma nem inicia |
| `--wait-qgc` | Com --execute, aguarda ENTER no terminal antes da primeira partida |
| `--speed` | 0.4 m/s; intervalo de execucao 0.15 a 1.0 m/s |
| `--wp-radius` | Raio de aceitacao em metros; padrao min(0.2, 0.1*R) |
| `--pivot-angle` | 60: pivo lento nos cantos; 0 seleciona curvas continuas |
| `--pivot-rate` | 30 graus/s; limite permitido ate 60 graus/s |
| `--circle-points` | 36; entre 12 e 360 segmentos |
| `--endpoint` | `udpin:127.0.0.1:14555`, uma saida do MAVProxy |
| `--sysid` | 1, identificacao do rover |
| `--source-system` | 246, identificacao MAVLink deste cliente (componente 191) |
| `--iterations` | Ate 2 candidatos; entre 1 e 6; apenas com --optimize |
| `--trial-timeout` | Automatico pelo comprimento/velocidade: max(1200, 10*tempo_nominal+120) segundos reais |
| `--max-speed` | 1.5 m/s; limite medido que interrompe o ensaio |
| `--output-dir` | Diretorio novo; padrao mission_tune/runs/data_hora |
| `--resume-run` | Reutiliza baseline completo; exige geometria, ganhos e condicoes documentadas iguais |
| `--real` | Permite o uso no rover fisico; sem ela o script exige SIMSTATE do SITL |

## Caminho de comunicacao

```text
mission_tune.sh / Python3.8
    <-> UDP encaminhado por MAVProxy --out (porta 14555)
        <-> master do MAVProxy (SITL, radio ou serial do rover)
```

O script usa pymavlink para falar o protocolo MAVLink, mas sua unica
conexao e a saida UDP do MAVProxy. Nao abre serial, TCP 5760 ou portas FDM.
Os itens sao enviados como MISSION_ITEM_INT, inclusive quando o autopiloto
emite o pedido legado MISSION_REQUEST. A escala 1e7 evita perda de precisao
das coordenadas em missao de poucos metros. Antes de armar, todos os pontos de navegacao
sao lidos de volta e comparados com o plano (home e fornecido pelo autopiloto); diferenca acima da quantizacao
centimetrica esperada interrompe a execucao.

Missao, parametros, comandos de modo/armamento e telemetria passam pelo
MAVProxy. Em um autopiloto com SYSID_ENFORCE ativo, use --source-system
com um identificador GCS permitido pela configuracao do veiculo; o script
nao altera essa politica. Ele precisa permitir encaminhamento nos dois sentidos, inclusive
quando armado: `set mavfwd true`. O utilitario bloqueia ensaios locais simultaneos para o mesmo SYSID.
Nao use dois consumidores na mesma porta 14555 e nao envie comandos concorrentes pelo QGroundControl durante o teste.
O QGroundControl continua recebendo sua propria saida UDP 14550.

No rover real, inicie o MAVProxy com o master apropriado, por exemplo:

```bash
python3.8 -m MAVProxy.mavproxy --master=/dev/ttyACM0 --baudrate=115200 \
  --out=udp:127.0.0.1:14555 --out=udp:127.0.0.1:14550 \
  --cmd='set mavfwd true'

# Em outro terminal, depois de preparar a area de ensaio:
bash Rover/mission_tune.sh --radius 3 --speed 0.4 --execute --real
# Otimizacao no rover real so quando explicitamente solicitada:
bash Rover/mission_tune.sh --radius 3 --speed 0.4 --execute --real --optimize
```

A mesma interface e usada na simulacao e no hardware. O caminho fisico ainda
nao foi validado. Comece com o rover parado e desarmado. O script exige GPS
3D e estimativa absoluta do EKF, nao altera ARMING_CHECK e nao forca armamento.
Antes de cada armamento, espera velocidade/rotacao baixas por tres segundos
do autopiloto e pre-armamento saudavel quando esse estado e publicado.
Uma mudanca externa de modo interrompe o ensaio. Ctrl+C solicita HOLD,
desarmamento e restauracao dos parametros originais. Se o enlace cair, a
restauracao pode falhar; o erro fica explicito no relatorio, e o controle
local/RC e necessario para recuperar o veiculo.

A missao anterior e arquivada em original_mission.json antes de ser
substituida; ela nao e restaurada automaticamente. Os ganhos originais sao
arquivados em original.parm. Eventos MISSION_ITEM_REACHED antigos nao
encerram uma nova repeticao: tambem e exigido o indice atual de finalizacao. Se nao houver melhora validada, permanecem os
originais. A missao nova pode ser visualizada no QGroundControl pelo
arquivo mission.plan; mission.waypoints (QGC WPL 110) tambem e gerado.

## O que e medido e otimizado

Os ensaios rodam em AUTO. Os controladores de taxa de giro e velocidade
ajustados tambem sao usados em outros modos assistidos, mas nao no controle
direto de Manual. Por isso uma melhora deste teste nao comprova melhora do
joystick em Manual, nem altera a fisica/atrito ou os ganhos internos de
velocidade das juntas do plugin Gazebo.

- Posicao/velocidade, atitude, taxas angulares e saidas de servo sao gravadas.
- `PID_TUNING` com `GCS_PID_MASK=3` fornece desejado/medido dos dois eixos:
  steering (axis 5, graus/s) e velocidade (axis 4, m/s). A taxa de giro e
  convertida a rad/s ao identificar os ganhos ATC_STR_RAT.
- A analise calcula RMSE de acompanhamento da trajetoria, erro radial no
  circulo, percentil 95 de erro, RMSE de velocidade e giro e saturacao.
  Aproximacao inicial e conexao entre figuras nao entram no score.
  A navegacao usa WP_RADIUS=min(0.2, 0.1*R), velocidade de 0.4 m/s e
  WP_PIVOT_ANGLE=60 e WP_PIVOT_RATE=30 graus/s por padrao. Esse perfil
  completou a repeticao sem exceder 1.5 m/s; a tentativa com curvas
  continuas (--pivot-angle 0) foi interrompida por excesso de velocidade.
  O erro do quadrado ainda nao melhorou frente ao perfil historico;
  aceitacao de 20 cm nao foi demonstrada. As mesmas condicoes sao
  mantidas em todos os ensaios de uma comparacao.
- O score combina erros normalizados de trajetoria, velocidade e giro,
  mais saturacao. Menor e melhor. Um ensaio incompleto recebe uma penalidade;
  ele pode fornecer dados para uma tentativa, mas nunca valida a sintonia.
- A identificacao estima saida/resposta durante a fase circular, excluindo
  amostras com resposta muito pequena, sentido incoerente ou saturacao.
  Sao necessarias pelo menos 20 amostras utilizaveis por eixo.
- A sequencia ajusta primeiro velocidade (ATC_SPEED_P/I e CRUISE_THROTTLE),
  depois giro (ATC_STR_RAT_FF/P/I). Assim a variacao de velocidade nao
  distorce a identificacao da resposta de giro.
  CRUISE_SPEED permanece como referencia; ATC_SPEED_FF/D e filtros nao sao
  modificados. A metodologia de razoes FF/P/I segue a ideia do QuickTune
  do ArduPilot, com limites incrementais e comparacao em missao completa.
- Os passos sao limitados a 50% por candidato no SITL e a 20% no hardware,
  respeitando tambem os limites absolutos do parametro. Parametros inteiros
  sao arredondados para um valor representavel e confirmados por leitura.
- Uma missao completa prevalece sobre uma incompleta. Entre ensaios com o
  mesmo estado de conclusao, os erros ponderados precisam melhorar pelo menos 2%. Uma melhora de pelo menos 2% no eixo da etapa (velocidade ou giro),
  com missao completa e erro p95 dentro do raio, pode ser usada temporariamente
  para identificar o eixo seguinte, mesmo sem melhora global. Isso nunca
  autoriza persistir esses ganhos: somente o vencedor global repetido e
  validado frente ao baseline pode permanecer ativo. Falha na repeticao restaura os parametros originais.

Se o baseline atingir o limite --max-speed, ele e interrompido e somente
os dados validos coletados podem identificar um candidato. Se faltarem
amostras, nenhuma alteracao e inventada. No SITL deste Husky, foram
observados picos acima de 1.5 m/s apesar da referencia de 0.6 m/s; para
caracterizar esses transitorios em simulacao foi usado --max-speed 2.0.
Esse valor nao e automaticamente aplicado no rover real.

O perfil atual de aproximacao usa WP_ACCEL=0.4 m/s2, WP_JERK=1 m/s3,
ATC_ACCEL_MAX=ATC_DECEL_MAX=0.5 m/s2, ATC_TURN_MAX_G=0.1,
ATC_STR_RAT_MAX=60 graus/s e ATC_STR_ACC_MAX=90 graus/s2.
O planejamento considera uma desaceleracao menor para comecar a frear
antes, respeitando a capacidade efetiva do veiculo. WP_PIVOT_DELAY=0.2 s
introduz uma breve espera apos alinhar o rumo. CRUISE_SPEED continua como
referencia da calibracao de CRUISE_THROTTLE; a velocidade de missao e WP_SPEED.
Todos esses parametros entram no backup e sao restaurados ao terminar.
O mesmo perfil fica em gazebo_simple/rover_simple.parm para o cenario.
Nesse arquivo, a estimativa de velocidade do Husky foi ajustada para
CRUISE_THROTTLE=24 com CRUISE_SPEED=1 e ATC_SPEED_P=0.15, a partir da coleta
do simulador. O utilitario de missao nao impoe esses valores ao rover real:
le os ganhos e a calibracao que estiverem ativos no veiculo.
O raio de aceitacao e um criterio de navegacao, nao uma garantia de erro GPS
menor que 20 cm. Os ensaios historicos no VALIDATION.md usam o perfil antigo;
nao podem ser retomados com --resume-run quando as condicoes mudam.

Circulos muito grandes em baixa velocidade podem nao excitar o giro o
suficiente para identificar esse eixo. Nesse caso a proposta e omitida,
em vez de inventar ganhos.

Nao e uma busca de otimo global. O resultado vale para a velocidade,
raio, carga e superficie ensaiados. A trajetoria tambem depende do navegador,
GPS e estimador; a comparacao PID desejado/medido ajuda a separar esses
fatores do controle de baixo nivel. Um raio muito menor que o raio fisico
minimo do veiculo nao e corrigido apenas por alterar ganhos.

Referencias primarias:

- [Navigation / S-Curves](https://ardupilot.org/rover/docs/rover-tuning-navigation.html)
- [Pivot Turns](https://ardupilot.org/rover/docs/rover-tuning-pivot-turns.html)
- [Turn Rate](https://ardupilot.org/rover/docs/rover-tuning-steering-rate.html)
- [Speed and Throttle](https://ardupilot.org/rover/docs/rover-tuning-throttle-and-speed.html)
- [Rover QuickTune](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/applets/rover-quicktune.md)
- [MAVLink Mission Protocol](https://mavlink.io/en/services/mission.html)

## Resultados

Cada execucao cria uma pasta nova em mission_tune/runs/.
No CSV/JSON bruto, lat/lon de GLOBAL_POSITION_INT estao em 1e-7 graus,
vx/vy em cm/s, yaw em radianos e yawspeed em rad/s; SERVO_OUTPUT_RAW esta em
microssegundos de PWM. PID_TUNING usa graus/s no eixo 5 e m/s no eixo 4.
boot_s esta em segundos do autopiloto. A analise faz as conversoes necessarias.
No backup/readback da missao, x/y sao normalizados para graus e marcados
com coordinate_units; wire_x/wire_y preservam o inteiro recebido.

Arquivos:

| Arquivo | Conteudo |
| --- | --- |
| plan.json / mission.waypoints / mission.plan | Geometria e missao; formato nativo do QGC incluido |
| original_mission.json | Backup da missao anterior |
| original.parm | Backup dos parametros alteraveis |
| telemetry.jsonl | Telemetria completa com tempo real, tempo do autopiloto e fase |
| trial_XX.csv / .json | Amostras selecionadas, metricas e parametros de cada ensaio |
| trial_XX.telemetry.jsonl | Telemetria por ensaio, incluida nas retomadas |
| candidate_XX.json | Proposta de parametros e dados de identificacao |
| report.json / report.html | Comparacao, motivo de falha, restauracao e vencedor |
| trajectory.png / trial_XX_response.png | Trajetoria e respostas desejada/medida |
| selected.parm | Parametros do melhor ensaio completo, podendo ser o baseline |
| optimized.parm | Somente quando ha melhora global confirmada pela repeticao |

NumPy e pymavlink sao necessarios. Matplotlib gera os graficos e foi
instalado neste Python 3.8; sem ele ainda sao gerados CSV, JSON e HTML.
Para recriar apenas os graficos:

```bash
python3.8 Rover/python/missions/plot_report.py Rover/mission_tune/runs/PASTA
```

Testes de geometria, unidades, limites e rejeicao de dados insuficientes:

```bash
python3.8 -m unittest discover -s Rover/python/missions/tests -v
```

Para continuar uma busca usando um baseline ja completo, sem repetir essa
primeira coleta, informe --resume-run PASTA com --execute --optimize. O
script preserva as coordenadas GPS do plano anterior, exige o mesmo raio e
velocidade, confere os ganhos atuais e as condicoes de navegacao registradas
e verifica que o rover esta proximo da area. Baselines incompletos ou sem
essas informacoes sao recusados. Use somente dados do mesmo veiculo e da
mesma superficie; identificacao de SYSID por si so nao identifica um chassis.
