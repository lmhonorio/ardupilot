# Validacao do utilitario de missao

Ensaio em 2026-10-27, Gazebo Classic 11.15.1, ArduRover 4.6.0-dev e
Python 3.8.10, comunicacao exclusivamente pela saida UDP do MAVProxy.
Nao houve teste no rover fisico.

As primeiras coletas abaixo foram feitas antes da revisao do transporte
para MISSION_ITEM_INT. A versao final tambem verifica o readback de todos os
pontos antes de armar. Os valores historicos nao demonstram a precisao da
missao final com coordenadas inteiras.

Missao: circulo de raio 3 m, 36 segmentos, seguido de quadrado circunscrito
com lado 6 m. Referencia de velocidade 0.6 m/s. O limite de interrupcao de
velocidade neste ensaio SITL foi 2.0 m/s. Todas as quatro coletas da tabela
completaram o circulo e o quadrado. Etapas completas anteriores foram
reutilizadas com verificacao de condicoes e telemetria, via --resume-run.

WP_RADIUS=0.45, WP_PIVOT_ANGLE=0, MIS_DONE_BEHAVE=0 e GCS_PID_MASK=3 foram
mantidos iguais durante as comparacoes e restaurados depois. O navegador
faz curvas continuas; nao e exigida uma parada exata nos quatro vertices.

| Coleta | RMSE trajetoria (m) | RMSE velocidade (m/s) | RMSE giro (graus/s) | Score |
| --- | --- | --- | --- | --- |
| trial_01 | 0.481 | 0.414 | 18.86 | 1.3278 |
| trial_02 | 1.027 | 0.393 | 19.38 | 1.5157 |
| trial_03 | 0.760 | 0.378 | 17.01 | 1.3102 |
| trial_04 | 0.834 | 0.389 | 19.78 | 1.4653 |

O terceiro ensaio reduziu os erros dos controladores, mas aumentou o erro
de trajetoria. A reducao do score combinado foi menor que o limiar de 2%.
O quarto ensaio nao melhorou o conjunto. Nenhuma sintonia nova foi aceita
ou persistida: os ganhos originais foram confirmados novamente por PARAM_VALUE.
O estado final foi confirmado como HOLD e desarmado, e o simulador de teste
foi encerrado. Nao foi alterada a fisica do mundo ou os ganhos das juntas.

Valores selecionados ao final: ATC_STR_RAT_FF/P/I=0.2/0.2/0.2,
ATC_SPEED_P/I=0.1/0.2, CRUISE_THROTTLE=75 (baseline). original.parm e
selected.parm documentam esses valores; optimized.parm nao foi produzido.
O relatorio HTML/JSON, CSV e graficos estao em runs/gazebo_r3_final/ na
maquina de validacao; esses resultados extensos sao ignorados pelo Git.

A verificacao automatizada cobre escalonamento do raio, quadrado circunscrito,
conversao GPS, unidades de giro, limites de ganhos, dados insuficientes ou
nao finitos, identificacao por eixo, diferenca entre progresso intermediario
e aceitacao global, parametro inteiro, repeticao de upload e rejeicao de
MISSION_ITEM_REACHED antigo. Os comandos de verificacao estao no README.

## Transporte final com coordenadas inteiras

Tambem foram completadas duas missoes com a versao MISSION_ITEM_INT,
com ACK de upload e leitura de volta de todos os pontos de navegacao antes
do armamento. As coordenadas foram conferidas novamente na telemetria
arquivada, com tolerancia de 1.5e-7 graus por eixo. Arquivos desta coleta:
runs/gazebo_r3_int/. Os dados historicos acima nao foram reutilizados como
baseline desta comparacao, pois o formato de transmissao mudou.

| Coleta INT | RMSE trajetoria (m) | RMSE velocidade (m/s) | RMSE giro (graus/s) | Score |
| --- | --- | --- | --- | --- |
| trial_01 | 0.881 | 0.405 | 18.80 | 1.4514 |
| trial_02 | 0.898 | 0.401 | 21.14 | 1.5560 |

O candidato de velocidade nao superou o baseline nesta repeticao. O
relatorio registra optimization_improved=false. Todos os parametros
originais foram relidos e confirmados, o rover ficou em HOLD/desarmado e o
simulador foi encerrado. O novo transporte teve a geometria confirmada;
isso nao demonstra, por si so, melhora do controle.

Verificacao final: 17 testes unitarios passaram, alem das verificacoes
Bash/Python e do teste integrado de missao pelo MAVProxy. O hardware real
permanece sem validacao. A sintonia avalia AUTO, nao o joystick em Manual.

## Visualizacao no QGroundControl

No QGC 4.2.9 instalado nesta maquina, uma missao antiga permanecia no mapa
apos o envio por outro cliente. Foi confirmado o recebimento da missao
atual de 43 itens por Plan -> File -> Download e a exibicao em Fly, com
telemetria pela saida local UDP 14550 do MAVProxy.

O modo --upload-only foi testado no SITL: upload aceito e leitura de volta
conferida, sem armar ou iniciar. O modo --execute --wait-qgc foi testado
ate a pausa antes do armamento, mantendo heartbeat; sua interrupcao por
SIGINT restaurou os parametros originais. Nao foi executada uma nova
missao nem uma nova otimizacao para essa verificacao da interface.

O exportador mission.plan foi verificado como plano nativo ArduPilot/Rover,
com 43 itens e home separado. Os 18 testes unitarios passaram no Python
3.8, incluindo a verificacao desse formato.

## Aproximacao aos waypoints e curvas

Perfil aplicado pelo MAVProxy e salvo em gazebo_simple/rover_simple.parm:
WP_SPEED=0.4, WP_RADIUS=0.2, WP_ACCEL=0.4, WP_JERK=1,
ATC_ACCEL_MAX=ATC_DECEL_MAX=0.5, ATC_TURN_MAX_G=0.1,
ATC_STR_RAT_MAX=60, ATC_STR_ACC_MAX=90, WP_PIVOT_ANGLE=60,
WP_PIVOT_RATE=30 e WP_PIVOT_DELAY=0.2. A coleta indicou aproximadamente
0.238 de saida normalizada por m/s; a calibracao especifica do Gazebo foi
ajustada de CRUISE_THROTTLE=75 para 24, mantendo CRUISE_SPEED=1, e
ATC_SPEED_P foi ajustado de 0.1 para 0.15. Os demais ganhos PID permaneceram
iguais. Isso nao constitui sintonia otima ou melhora global validada.

Antes da calibracao de velocidade, a tentativa waypoint_precision_r3 foi
interrompida no waypoint 10 por pico medido de 1.651 m/s. Depois da
calibracao, waypoint_precision_speed24 completou os 43 itens sem ultrapassar
o limite de 1.5 m/s. O erro RMSE de velocidade foi 0.312 m/s, o erro de
acompanhamento da trajetoria 0.828 m e o erro do quadrado 0.868 m.

A coleta anterior iniciada pelo QGC durante a pausa --wait-qgc foi
reavaliada separadamente a partir de seq_current, considerando somente
telemetria em AUTO/armado. Ela teve RMSE de velocidade 0.352 m/s,
trajetoria 0.722 m e quadrado 0.582 m. O novo perfil reduziu o erro de
velocidade, mas piorou o acompanhamento do quadrado. As velocidades e
condicoes de navegacao sao diferentes; nao e uma validacao de ganhos PID
pela regra de score do otimizador. A precisao de 20 cm nao foi atingida.

Uma ultima tentativa, waypoint_precision_continuous, manteve os demais
valores e desativou o pivo. Completou o circulo e foi interrompida no
waypoint 39 por velocidade acima de 1.5 m/s. Por isso foi mantido o perfil
com pivo lento, que teve uma repeticao completa dentro desse limite.

Os relatorios e telemetria estao nas pastas citadas em runs/. Backups e
leitura final confirmada ficam em navigation_profile_20261027_131952/;
final/confirmed.json confirma HOLD e desarmado. Todos os parametros
temporarios de cada ensaio foram restaurados ao perfil ativo. Os 20 testes
unitarios passaram no Python 3.8. A simulacao continua disponivel; o rover
real nao foi acessado.
