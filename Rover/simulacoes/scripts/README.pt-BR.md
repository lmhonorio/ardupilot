# Alocacao diferencial do Rover em Lua

Para as chamadas de inicialização e execução, consulte o
[Manual de simulação](../MANUAL_SIMULACAO.md).

`rover-controlallocation.lua` usa os comandos normalizados de aceleracao e
giro do ArduRover, ambos entre -1 e +1. A mistura em `modules/functions.lua`
e esquerda = aceleracao + giro, direita = aceleracao - giro. Quando algum
lado satura, `MOT_STR_THR_MIX` define a prioridade: 0 preserva aceleracao,
1 preserva giro e 0.5 divide ambos pelo mesmo fator, como no ArduRover.
A mistura respeita `MOT_THST_ASYM`; a conversao de cada lado aplica
`MOT_THR_MIN` e `MOT_THST_EXPO`, mantendo zero exatamente neutro.
Sem saturacao, um comando
pequeno de giro nao perde autoridade quando ha aceleracao. Aceleracao zero
continua zero; valores negativos permitem re e frenagem.

O limitador inclui corretamente os dois extremos. Antes, um valor exatamente
igual ao minimo era convertido para o maximo. A mistura antiga tambem elevava
os comandos ao quadrado ao calcular as proporcoes, alterando a resposta dos
controladores. O AUTO ainda impunha um minimo de 10% de aceleracao, removido.

## Saidas e configuracao

O script identifica os motores por `SERVOx_FUNCTION`:

| Funcao | Lado |
| --- | --- |
| 73 | Esquerdo |
| 74 | Direito |

Somente esses canais recebem overrides. Cada um usa seu proprio MIN, TRIM,
MAX e REVERSED. O trim deve estar estritamente entre os limites. A inversao
fixa do lado direito foi removida: o sentido fisico deve estar declarado em
`SERVOx_REVERSED`. A antiga numeracao fixa de quatro motores nao e mais usada.

O modelo Gazebo atual usa SERVO1=73 e SERVO3=74; o perfil gzrover.param usa
SERVO1/4=73 e SERVO2/3=74. Os dois mapeamentos sao reconhecidos. A polaridade
das rodas no hardware ainda precisa ser validada, pois nao houve ensaio fisico.
Este driver e para PWM bidirecional normal (`MOT_PWM_TYPE=0`). Outros
protocolos e o controlador de velocidade de roda (`WRC_ENABLE` diferente
de zero) ficam no driver nativo. O throttle lido do ArduRover ja passou
pelo limitador de aceleracao do firmware; o Lua nao aplica outra rampa.

O ciclo ativo e de 20 ms (50 Hz), com override de 100 ms. Quando a
alocacao esta habilitada, o Lua aplica throttle e direcao produzidos pelo
ArduRover, inclusive em AUTO, GUIDED e RTL, sem criar trajetoria ou mudar
modos. HOLD preserva a frenagem nativa. Desarmamento, E-Stop e desativacao
da alocacao liberam os overrides. O desvio nativo do ArduPilot depende de sua
propria configuracao; este Lua nao implementa desvio ou seguimento de linha.

## Habilitacao

Instale `system-start.lua`, `rover-controlallocation.lua` e os arquivos de
`modules/` no diretorio de scripts usado pelo autopiloto (`APM/scripts` no
cartao; `scripts/` no diretorio de execucao do SITL).

- `SCR_ENABLE=1` e `SCR_USER5=2` permitem executar o script neste Rover.
  `SCR_USER5` agora e lido a cada ciclo; a configuracao tardia nao encerra
  definitivamente o callback.
- `BATT_SOC_ALLOC=1` habilita a alocacao dos comandos do controlador.
- `BATT_SOC_ALLOC=0` deixa todos os modos com o controle nativo.
O alinhamento de yaw em waypoint e os parametros Lua de desvio foram
removidos. Os indices 2 e 5 da tabela de parametros continuam preservados.
Os scripts se vinculam ao parametro de habilitacao quando ele estiver pronto.
O receptor FAST-LIO continua separado em `lidar_odometry.lua`.

Para usar este driver no cenario Gazebo, execute
`./start_gz_simple.sh --lua --no-rebuild` no diretorio Rover. Esse modo inicia
o binario ArduRover diretamente, sem sim_vehicle.py, instala copias dos fontes
atuais em gazebo_simple/run/lua/scripts/ e habilita a alocacao com os parametros
selecionados. O modo padrao anterior usava SCR_ENABLE=0; os ensaios anteriores
de trajetoria usaram o controle nativo. A inicializacao com Lua foi conferida
com o rover desarmado. Veja o [procedimento completo](../gazebo_simple/README.pt-BR.md).

## Verificacao

### ROS Argo e sim_gz1rover.sh

No ensaio de 28/10/2026, o log do perfil anterior registrava `SCR_ENABLE=0`:
o Lua nao estava atuando. `newrovergazebo.params` esta no formato QGroundControl
de cinco colunas; nao pode ser passado diretamente como arquivo de defaults
`NOME VALOR` do SITL.

O lancador agora converte esse arquivo, prepara copias atuais dos scripts
e usa EEPROM/logs separados em `gazebo_simple/run/ros_argo_lua/`. O arquivo
original e a EEPROM anterior em Rover/ sao preservados. O inicializador
aplica `gazebo_simple/ros_argo_lua.parm` enquanto desarmado e habilita
`BATT_SOC_ALLOC=1`.

O perfil de partida limita a referencia a 0.4 m/s, o raio de waypoint a
0.2 m e usa FF/P/I de giro de 0.2/0.2/0.2. Isso substitui os valores
historicos FF=3/I=2 e o P=10 presente na exportacao antiga. Sao valores
iniciais conservadores, nao ganhos otimos nem uma sintonia validada em
missao no modelo ROS Argo. CRUISE_SPEED/THROTTLE continuam vindos do arquivo
original, sem copiar a calibracao do Husky.

Com o mundo aberto pelo roslaunch, encerre o SITL anterior no terminal dele
e execute, em Rover/:

```bash
./sim_gz1rover.sh --foreground --no-rebuild
```

Sem `--foreground`, abre outro terminal. Sem `--no-rebuild`, compila usando
Python 3.8. O console wxPython e opcional (`--console`); QGC recebe UDP 14550,
MAVROS recebe 14555. Nao execute o utilitario de missao escutando 14555 ao
mesmo tempo que MAVROS, pois ambos tentariam ocupar a mesma porta.

Aguarde `ROS Argo: Lua allocation enabled; simulation profile applied`.
No prompt MAVProxy, confira:

```text
param show SCR_ENABLE
param show SCR_USER5
param show BATT_SOC_ALLOC
```

Os valores esperados sao 1, 2 e 1. Se SCR_ENABLE tiver sido desabilitado
manualmente nessa nova EEPROM, reabilite-o e reinicie o SITL. O lancador
recusa iniciar quando ja existe SITL escutando TCP 5760.

O inicializador tambem habilita `GPS_GLOBAL_ORIGIN` a 1 Hz no link MAVProxy
(mensagem 49, porta de telemetria 0), para o MAVROS receber a origem real do
EKF. Isso corrige a ausencia da mensagem observada no aviso
`PositionTargetGlobal failed because no origin`; HOME_POSITION sozinho nao
alimenta esse consumidor ROS. Aguarde GPS/EKF prontos. Ao encerrar, pare
primeiro SITL/MAVProxy e depois roslaunch/Gazebo.

O carregamento dos scripts, o callback e os parametros foram conferidos
no firmware compilado, em SITL interno separado, desarmado, na instancia 20
(TCP 5960). Evidencias em
`gazebo_simple/run/ros_argo_allocation_check/validation.json`. Isso nao
substitui um ensaio de navegacao no mundo ROS com os sensores ligados.

### Testes automatizados

Na raiz do repositorio:

```bash
lua Rover/tests/test_control_allocation.lua
luac -p Rover/scripts/modules/functions.lua Rover/scripts/rover-controlallocation.lua Rover/scripts/system-start.lua
python3.8 -m unittest discover -s Rover/python/tests -p test_ros_argo_lua.py -v
```

Os testes usam dublês das APIs do ArduPilot, sem conectar ao veiculo. Cobrem
fronteiras do limitador, saturacao e simetria em re, comandos invalidos,
mapeamento de dois/quatro motores, limites e reversao independentes, frenagem,
desarmamento/E-Stop, alternancia de modos, criacao tardia dos parametros e yaw
opcional. Eles nao substituem a validacao do driver no firmware e no hardware.

Referencia: [API de scripting do ArduPilot](https://ardupilot.org/rover/docs/common-lua-scripts.html).
