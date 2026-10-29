# Manual de chamadas — Rover, Gazebo, Lua e autotuning

Este manual reúne os comandos disponíveis neste projeto, somente para
simulação. Todos os exemplos usam Python **3.8**, SYSID **1** e instância **0**.
Não use `--real` nos comandos deste manual.

O nome do lançador é **`start_gz_simple.sh`**, sem o `s` no final de `simple`.

## 1. Qual chamada usar

Os comandos resumidos abaixo pressupõem que o terminal está em
`/home/grin/ardupilot/Rover`.

| Objetivo | Chamada ou seção |
| --- | --- |
| Só gerar o círculo e o quadrado, sem iniciar simulador | `./mission_tune.sh --radius 3` |
| Simular sem Gazebo, com física interna do SITL | Seção 3: binário ArduRover + MAVProxy em dois terminais |
| Simular sem Gazebo, usando também sua alocação Lua | Seção 4 |
| Gazebo + sua alocação Lua + parâmetros selecionados | `./start_gz_simple.sh --lua --no-rebuild` |
| Gazebo + controle nativo | `./start_gz_simple.sh --no-rebuild` |
| Gazebo + Lua, sem janelas | `./start_gz_simple.sh --lua --headless --no-rebuild` |
| Enviar missão sem armar ou iniciar | `./mission_tune.sh --radius 3 --upload-only` |
| Executar uma missão, coletar e analisar | `./mission_tune.sh --radius 3 --execute --wait-qgc` |
| Executar missões com sintonia automática | `./mission_tune.sh --radius 3 --execute --optimize --wait-qgc` |
| Encerrar o cenário iniciado pelo lançador Gazebo | `./start_gz_simple.sh --stop` |

**Iniciar o simulador e executar a missão são chamadas separadas.** Primeiro
escolha um simulador; depois use o utilitário de missão em outro terminal.
Use apenas uma instância destes cenários por vez: eles compartilham portas.

## 2. Preparação e conceitos

### Diretório e Python

Comece os terminais de comandos do projeto com:

```bash
cd /home/grin/ardupilot/Rover
python3.8 --version
```

Nesta máquina o Python instalado é
`/home/grin/.pyenv/versions/3.8.10/bin/python3.8`. Os lançadores Gazebo
selecionam Python 3.8 também para o MAVProxy e o build. Se necessário:

```bash
PYTHON38=/home/grin/.pyenv/versions/3.8.10/bin/python3.8 ./start_gz_simple.sh --lua --no-rebuild
```

### O que cada componente faz

| Componente | Função |
| --- | --- |
| ArduRover/SITL | Executa o autopiloto, os PIDs, a missão e as APIs usadas pelo Lua |
| Física interna `rover-skid` | Simula o movimento sem Gazebo |
| Gazebo + modelo `gazebo-rover` | Simula física externa, obstáculos, câmera e LiDAR |
| Lua de alocação | Transforma aceleração/giro do ArduRover em comandos dos motores |
| MAVProxy | Encaminha MAVLink entre autopiloto, QGC e utilitário de missão |
| QGroundControl | Mostra posição, missão e progresso |
| `mission_tune.sh` | Gera/envia/executa a missão, registra dados e pode ajustar ganhos |

O Lua atual depende das APIs do ArduRover; não é um controlador independente
que possa ser iniciado com `lua functions.lua`. Os modos diretos abaixo
dispensam **sim_vehicle.py**, mantendo o binário ArduRover.

`--no-rebuild` reutiliza o firmware compilado. Se ainda não existir, ou
depois de alterar C++, compile antes de usar os exemplos diretos:

```bash
cd /home/grin/ardupilot
python3.8 ./waf configure --board sitl
python3.8 ./waf rover
cd /home/grin/ardupilot/Rover
```

Alterar somente Lua não exige recompilar C++. No modo Gazebo `--lua`,
reiniciar o lançador copia os fontes atualizados para o diretório de execução.

## 3. Simulação sem Gazebo — SITL interno e controle nativo

Este modo não abre mundo 3D e não fornece a câmera/LiDAR do Gazebo. Serve
para exercitar MAVLink, missões e o controle com o modelo interno. A física
é diferente da do Husky; uma sintonia obtida aqui não comprova desempenho
no Gazebo.

### Terminal 1: iniciar o ArduRover diretamente

```bash
ROVER_REPO=/home/grin/ardupilot
ROVER_STATE="$ROVER_REPO/Rover/gazebo_simple/run/sitl_internal_native"
mkdir -p "$ROVER_STATE"
cd "$ROVER_STATE"

"$ROVER_REPO/build/sitl/bin/ardurover" \
  -S --model rover-skid --speedup 1 --sysid 1 -I 0 \
  --home=-3.12315,-41.764495,77,0 \
  --defaults "$ROVER_REPO/Tools/autotest/default_params/rover.parm,$ROVER_REPO/Tools/autotest/default_params/rover-skid.parm,$ROVER_REPO/Rover/gazebo_simple/rover_simple.parm"
```

Na primeira execução, esse diretório tem estado próprio e scripting
desabilitado por padrão. Não copie os scripts Lua para esse diretório se
quiser manter o ensaio nativo.

### Terminal 2: iniciar o MAVProxy

```bash
cd /home/grin/ardupilot/Rover/gazebo_simple/run/sitl_internal_native

python3.8 -m MAVProxy.mavproxy \
  --master=tcp:127.0.0.1:5760 --sitl=127.0.0.1:5501 \
  --out=udp:127.0.0.1:14550 \
  --out=udp:127.0.0.1:14555 \
  --cmd='set mavfwd true; rc 1 1500; rc 3 1500'
```

Aguarde heartbeat e GPS/EKF. O RC simulado inicia neutro; não é necessário
armar no MAVProxy para usar o utilitário de missão.

Em um **terceiro terminal**, execute uma das chamadas da seção 6 ou 7.
Para encerrar este modo manual, use Ctrl+C nos terminais 1 e 2. O comando
`start_gz_simple.sh --stop` não gerencia esses processos iniciados manualmente.

## 4. Simulação sem Gazebo — SITL interno com alocação Lua

Também é possível carregar seu Lua no modelo interno, sem sim_vehicle.py.
Este procedimento usa outro diretório, para não misturar o estado dos modos.

### Terminal 1: preparar os scripts e iniciar

```bash
ROVER_REPO=/home/grin/ardupilot
ROVER_STATE="$ROVER_REPO/Rover/gazebo_simple/run/sitl_internal_lua"
mkdir -p "$ROVER_STATE/scripts/modules"

cp "$ROVER_REPO/Rover/scripts/system-start.lua" \
   "$ROVER_REPO/Rover/scripts/rover-controlallocation.lua" \
   "$ROVER_STATE/scripts/"
cp "$ROVER_REPO/Rover/scripts/modules/functions.lua" \
   "$ROVER_REPO/Rover/scripts/modules/pid.lua" \
   "$ROVER_STATE/scripts/modules/"

cd "$ROVER_STATE"
"$ROVER_REPO/build/sitl/bin/ardurover" \
  -S --model rover-skid --speedup 1 --sysid 1 -I 0 \
  --home=-3.12315,-41.764495,77,0 \
  --defaults "$ROVER_REPO/Tools/autotest/default_params/rover.parm,$ROVER_REPO/Tools/autotest/default_params/rover-skid.parm,$ROVER_REPO/Rover/gazebo_simple/rover_simple.parm,$ROVER_REPO/Rover/gazebo_simple/rover_lua.parm"
```

### Terminal 2: MAVProxy e habilitação do Lua

```bash
cd /home/grin/ardupilot/Rover/gazebo_simple/run/sitl_internal_lua

python3.8 -m MAVProxy.mavproxy \
  --master=tcp:127.0.0.1:5760 --sitl=127.0.0.1:5501 \
  --out=udp:127.0.0.1:14550 \
  --out=udp:127.0.0.1:14555 \
  --cmd='set mavfwd true; rc 1 1500; rc 3 1500'
```

Aguarde `System started properly`. Depois, **no prompt MAVProxy**:

```text
param fetch
```

Aguarde a atualização da lista e execute, ainda no MAVProxy:

```text
param set BATT_SOC_ALLOC 1
param show SCR_ENABLE
param show SCR_USER5
param show BATT_SOC_ALLOC
```

Os valores esperados são `SCR_ENABLE=1`, `SCR_USER5=2` e
`BATT_SOC_ALLOC=1`. Neste procedimento manual não existe o inicializador
automático exclusivo do modo Gazebo; por isso a habilitação foi explícita.
Se você tiver desabilitado `SCR_ENABLE` nessa EEPROM anteriormente,
habilite-o e reinicie o firmware antes de procurar os parâmetros dinâmicos.

Use o terceiro terminal para a missão. Ao editar Lua, repita a cópia dos
fontes e reinicie este modo. Encerre os dois processos com Ctrl+C.

## 5. Simulação com Gazebo

### 5.1 Gazebo + seu Lua + parâmetros selecionados

Esta é a chamada principal para testar sua alocação no Husky:

```bash
cd /home/grin/ardupilot/Rover
./start_gz_simple.sh --lua --no-rebuild
```

Ela inicia Gazebo, câmera/LiDAR, o **binário direto** ArduRover e MAVProxy,
sem sim_vehicle.py. Aplica os parâmetros selecionados e habilita o Lua.
Aguarde:

```text
Gazebo: Lua allocation enabled; selected parameters applied
```

O estado fica em `gazebo_simple/run/lua/`. Os fontes são copiados de
`scripts/` a cada partida. O inicializador gerado para a simulação mantém
`BATT_SOC_ALLOC=1`.

Sem firmware compilado, ou para recompilar antes de iniciar:

```bash
./start_gz_simple.sh --lua
```

### 5.2 Gazebo + alocação nativa

```bash
cd /home/grin/ardupilot/Rover
./start_gz_simple.sh --no-rebuild
```

Esta chamada usa o caminho anterior com sim_vehicle.py e o estado em
`gazebo_simple/run/`. Ela não instala nem habilita sua alocação Lua. No
estado observado, `SCR_ENABLE=0`; confirme isso se tiver alterado a EEPROM
desse modo anteriormente. Omitir `--lua` não troca automaticamente uma
configuração de scripting que você tenha salvo manualmente.

### 5.3 Gazebo sem janelas

Com Lua:

```bash
./start_gz_simple.sh --lua --headless --no-rebuild
```

Com alocação nativa:

```bash
./start_gz_simple.sh --headless --no-rebuild
```

**Headless continua sendo Gazebo**, com a mesma física externa. Ele apenas
não abre as janelas do Gazebo e do leitor OpenCV. A renderização dos sensores
ainda precisa de um DISPLAY utilizável ou de Xvfb; não equivale ao SITL
interno das seções 3 e 4.

### 5.4 Iniciar componentes separados

Para diagnóstico, em dois terminais, ambos no diretório Rover:

```bash
# Terminal 1: mundo Gazebo
./sim_gz_simple.sh gazebo
```

```bash
# Terminal 2: binário ArduRover + Lua + MAVProxy
./sim_gz_simple.sh sitl-lua --no-rebuild
```

Para ler os sensores, em um terceiro terminal:

```bash
./sim_gz_simple.sh sensors --timeout 0
```

O modo nativo alternativo do segundo terminal é `./sim_gz_simple.sh sitl`.
Essas chamadas separadas não usam o gerenciamento de processos do lançador
principal; encerre cada terminal com Ctrl+C.

## 6. Geração, envio e execução da missão

Os exemplos desta seção partem de `/home/grin/ardupilot/Rover`. Para envio
ou execução, deixe um dos simuladores anteriores funcionando.

### 6.1 Gerar somente os arquivos, sem conexão

```bash
./mission_tune.sh --radius 3
```

Gera o círculo de raio **3 m**, seguido do quadrado circunscrito de lado
**6 m**, com o mesmo centro. Não conecta, não arma e não movimenta o rover.

Para outra geometria:

```bash
./mission_tune.sh --radius 5 --speed 0.4
```

O quadrado terá lado 10 m. Os arquivos incluem `plan.json`,
`mission.waypoints` e `mission.plan`, em uma pasta nova de `mission_tune/runs/`.

### 6.2 Enviar e conferir, sem partir

```bash
./mission_tune.sh --radius 3 --upload-only
```

Lê a posição GPS atual, regenera a missão, faz backup da anterior, envia e
confere as coordenadas por leitura de volta. Não arma nem inicia. Esse modo
não configura os ganhos ou a velocidade do veículo; quem os aplica durante
um ensaio é `--execute`, ou a preparação do cenário.

### 6.3 Executar uma vez e acompanhar pelo QGC

```bash
./mission_tune.sh --radius 3 --speed 0.4 --wp-radius 0.2 --execute --wait-qgc
```

O script envia a missão e espera ENTER antes da partida. Enquanto ele espera:

1. No QGC, abra **Plan → File → Download**.
2. Confirme a substituição do plano local, se solicitado.
3. Abra **Fly** e aumente o zoom: o trajeto tem poucos metros.
4. Pressione **ENTER no terminal do script de missão**.

O script verifica parada/pre-armamento, arma, inicia AUTO, registra a
telemetria e gera a análise. Para esse fluxo, deixe o script conduzir a
partida; iniciar pelo QGC enquanto ele espera ENTER pode movimentar o rover
fora da coleta do ensaio.

Sem a pausa para o QGC:

```bash
./mission_tune.sh --radius 3 --speed 0.4 --wp-radius 0.2 --execute
```

### 6.4 Parâmetros mais usados

| Argumento | Significado / padrão |
| --- | --- |
| `--radius 3` | Raio geométrico do círculo em metros; obrigatório; na execução, mínimo 0,5 m |
| `--wp-radius 0.2` | Raio de aceitação do waypoint; positivo e menor que `--radius` |
| Sem `--wp-radius` | Usa `min(0.2, 0.1 * radius)` |
| `--speed 0.4` | Referência de velocidade em m/s; execução aceita 0,15 a 1,0 |
| `--circle-points 36` | Quantidade de segmentos do círculo; aceita 12 a 360 |
| `--pivot-angle 60` | Habilita pivô para curvas suficientemente grandes; é o padrão |
| `--pivot-angle 0` | Usa curvas contínuas, sem pivô |
| `--pivot-rate 30` | Limite de velocidade do pivô em graus/s; padrão 30, máximo 60 |
| `--center-north 2 --center-east -1` | Desloca o centro 2 m ao norte e 1 m a oeste da origem GPS da execução |
| `--max-speed 1.5` | Interrompe acima dessa velocidade medida; é o padrão |
| `--trial-timeout 1200` | Limite de tempo real por ensaio, em segundos |
| `--endpoint udpin:127.0.0.1:14555` | Porta recebida do MAVProxy; já é o padrão |

**`--radius` e `--wp-radius` são coisas diferentes.** O primeiro define a
figura; o segundo define o critério de aceitação dos pontos. Configurar
0,20 m não garante erro físico/GPS de 0,20 m. Alterar o raio também não
reposiciona os obstáculos do mundo Gazebo.

## 7. Simulação com autotuning

Neste projeto, autotuning significa a busca limitada implementada por
`mission_tune.sh --optimize`; não é um modo AUTOTUNE separado do firmware.
O script ajusta os PIDs do ArduRover e a estimativa de aceleração. Ele não
reescreve a mistura Lua nem ajusta `SCR_USER2/3/4` do yaw opcional.

### 7.1 Gazebo + Lua + sintonia automática

**Terminal 1:**

```bash
cd /home/grin/ardupilot/Rover
./start_gz_simple.sh --lua --no-rebuild
```

**Terminal 2**, depois da mensagem de habilitação do Lua:

```bash
cd /home/grin/ardupilot/Rover
./mission_tune.sh --radius 3 --speed 0.4 --wp-radius 0.2 \
  --execute --optimize --iterations 2 --wait-qgc
```

Prepare o QGC como na seção 6.3 e pressione ENTER. A pausa ocorre somente
antes da primeira partida; as repetições seguintes são conduzidas pelo script.

### 7.2 Outros simuladores com a mesma função

Para autotuning com Gazebo nativo, use no terminal 1:

```bash
./start_gz_simple.sh --no-rebuild
```

Depois execute a mesma chamada `mission_tune.sh --execute --optimize` do
terminal 2. Para SITL interno, inicie os dois terminais da seção 3 ou 4 e
execute o otimizador em um terceiro terminal. Não transfira um baseline
entre física interna, Gazebo, alocação nativa e Lua como se fossem o mesmo
ensaio.

### 7.3 O que será ajustado

| Grupo | Parâmetros |
| --- | --- |
| Velocidade | `ATC_SPEED_P`, `ATC_SPEED_I`, `CRUISE_THROTTLE` |
| Giro | `ATC_STR_RAT_FF`, `ATC_STR_RAT_P`, `ATC_STR_RAT_I` |

Com duas iterações, tenta-se primeiro velocidade e depois giro, caso existam
dados suficientes. O fluxo coleta um baseline, testa candidatos e repete o
melhor candidato quando necessário para validar a melhora. Falta de dados
pode encerrar a busca antes de usar todas as iterações.

`--iterations` aceita 1 a 6. Não há garantia de encontrar um ótimo global.
Se não houver melhora global validada, os ganhos anteriores são restaurados.
Os parâmetros temporários de navegação e telemetria também têm tratamento
de restauração; falhas ficam no relatório. A missão anterior fica em backup,
mas não é restaurada automaticamente ao terminar.

O limite padrão de velocidade medida continua sendo 1,5 m/s. Ensaios
históricos de caracterização do Gazebo usaram 2,0 m/s; isso não transforma
esse valor em requisito do autotuning nem em correção do overshoot.

## 8. QGroundControl e portas

O QGC deste computador recebe em **UDP 14550**. O utilitário de missão usa
**UDP 14555**, passando pelo MAVProxy. Eles não disputam a mesma porta.

| Porta | Uso |
| --- | --- |
| TCP 5760 | ArduRover → conexão master do MAVProxy |
| UDP 5501 | Entrada RC simulada usada pelo MAVProxy |
| UDP 14550 | Telemetria local do QGC |
| UDP 14555 | Missão, comandos, parâmetros e coleta do utilitário |
| UDP 14551, 14552, 14560, 14570 | Saídas adicionais dos lançadores Gazebo |
| UDP 9002/9003 | Troca de física entre Gazebo e SITL; não são portas de GCS |

Os lançadores Gazebo mantêm também os destinos LAN definidos nos scripts.
Os exemplos manuais de SITL interno deste manual criam somente as saídas
14550 e 14555.

Se o mapa mostrar uma missão antiga, faça **Plan → File → Download** depois
do envio. Para visualizar apenas um arquivo offline, use
**Plan → File → Open** e escolha o `mission.plan` gerado.

Durante uma coleta, use o QGC para observar e evite comandos concorrentes
de modo/armamento. Mudança externa de modo interrompe o ensaio. A porta
14555 deve ter somente um consumidor local por vez.

## 9. Câmera, QR code e LiDAR

São recursos do mundo Gazebo. O lançador principal já inicia a leitura.
Para abrir a nuvem 3D do LiDAR em outro terminal, sem reiniciar o cenário:

```bash
cd /home/grin/ardupilot/Rover
python3.8 python/lidar/view_lidar.py
```

A janela atualiza a cada segundo, usando o arquivo gravado pelo leitor.
Arraste com o mouse para girar a nuvem; feche a janela para sair. O X
vermelho marca a origem do sensor; as cores representam a altura dos pontos.
Para facilitar a leitura dos obstáculos baixos, a altura é ampliada 5 vezes
na exibição, mantendo as coordenadas em metros. Use `--z-scale 1` para escala física.
Essa visualização roda separadamente do Gazebo e também pode ser aberta
durante uma simulação `--headless`, desde que o terminal tenha um display.
Os feixes do próprio Gazebo estão ocultos por `<visualize>false</visualize>`
em `gazebo_simple/models/rover_mid360/model.sdf`.

Os arquivos mais recentes ficam em `gazebo_simple/run/sensors/`:

| Arquivo | Conteúdo |
| --- | --- |
| `camera.png` | Imagem da câmera visual |
| `camera.json` | Metadados da imagem |
| `lidar.npz` | Distâncias `ranges`, pontos `xyz` e metadados da varredura |
| `lidar.json` | Metadados da varredura |

O QR code está na parede verde. A detecção depende da distância e do
enquadramento da câmera. O LiDAR é uma aproximação do campo de visão do
MID360, não uma reprodução exata do padrão de varredura do equipamento.

Para salvar uma captura separada, sem disputar os arquivos do leitor principal:

```bash
cd /home/grin/ardupilot/Rover
./sim_gz_simple.sh sensors --once --no-gui \
  --output-dir gazebo_simple/run/sensors_snapshot
```

Para ler a nuvem salva pelo leitor principal:

```bash
python3.8 - <<'PY'
import numpy as np
with np.load('gazebo_simple/run/sensors/lidar.npz') as scan:
    print('Matriz de distâncias:', scan['ranges'].shape)
    print('Pontos XYZ:', scan['xyz'].shape)
    print(scan['xyz'][:5])  # coordenadas em metros no referencial do sensor
PY
```

Não existe ponte automática desses dados de proximidade para o MAVLink
neste cenário. A missão não usa o LiDAR para desviar dos obstáculos.

## 10. Resultados e parâmetros selecionados

Cada chamada do utilitário gera uma pasta nova em `mission_tune/runs/` e
imprime seu caminho. Se usar `--output-dir`, escolha um diretório que ainda
não exista. Nos exemplos abaixo, substitua **PASTA** pela pasta da execução.

```bash
xdg-open /home/grin/ardupilot/Rover/mission_tune/runs/PASTA/report.html
```

| Arquivo | Uso |
| --- | --- |
| `report.html`, `report.json` | Resultado, erros, métricas, parâmetros e restauração |
| `trajectory.png` | Trajetória de referência e trajetória medida |
| `trial_XX_response.png` | Resposta desejada/medida dos controladores |
| `telemetry.jsonl`, `trial_XX.csv` | Dados coletados |
| `original.parm` | Ganhos e parâmetros anteriores ao ensaio conectado |
| `original_mission.json` | Backup da missão anterior |
| `selected.parm` | Ganhos selecionados ao final de um ensaio válido; podem ser os originais |
| `optimized.parm` | Criado somente quando há melhora global validada |
| `mission.plan`, `mission.waypoints` | Missão gerada/enviada |

Arquivos exclusivos de execução não aparecem necessariamente em chamadas
que só geram ou só enviam o plano. A presença de `selected.parm`, sozinha,
não significa que houve melhoria. Consulte `success`, `optimization_improved`
e os erros do relatório.

Para refazer somente os gráficos, sem executar o rover:

```bash
python3.8 python/missions/plot_report.py mission_tune/runs/PASTA
```

### De onde vêm os parâmetros

| Fonte | Papel |
| --- | --- |
| `gazebo_simple/rover_simple.parm` | Perfil selecionado de motores, velocidade, aceitação e curvas |
| `gazebo_simple/rover_lua.parm` | Habilitação Lua e configuração específica desse SITL |
| EEPROM do diretório de execução | Valores persistidos do autopiloto |
| Argumentos de `mission_tune.sh` | Condições temporárias de cada ensaio |

No modo Gazebo `--lua`, o inicializador reaplica a cada partida os campos
presentes nos dois arquivos de perfil. Uma melhora aceita pode ficar ativa
ao fim do otimizador, mas não atualiza esses arquivos automaticamente.
Na próxima partida, os campos presentes nos perfis serão reaplicados.

Para reaplicar um resultado já validado, depois da mensagem de preparação
do cenário, use **no MAVProxy**, substituindo PASTA:

```text
param load /home/grin/ardupilot/Rover/mission_tune/runs/PASTA/optimized.parm
```

Confira a confirmação dos valores. Para tornar esses ganhos o novo perfil
de partida do Gazebo, atualize os campos correspondentes em
`gazebo_simple/rover_simple.parm`; não substitua o perfil inteiro por um
arquivo que contém somente seis ganhos.

Nos modos iniciados manualmente e no Gazebo nativo, arquivos passados como
`--defaults` são padrões: valores já salvos na EEPROM podem prevalecer.
Use `param show NOME` no MAVProxy para confirmar o valor efetivo. Não use
`-w` para uma simples troca de modo: ele apaga o estado persistente.

## 11. Encerramento e diagnóstico

### Interromper uma missão ou autotuning

Use Ctrl+C **no terminal de `mission_tune.sh`**. Ele solicita HOLD,
desarmamento e restauração. Aguarde o resultado dessa limpeza antes de
encerrar o simulador, para manter o enlace disponível durante a restauração.

### Encerrar Gazebo e os processos do lançador

Use Ctrl+C no terminal de `start_gz_simple.sh` ou, em outro terminal:

```bash
cd /home/grin/ardupilot/Rover
./start_gz_simple.sh --stop
```

Isso vale para os modos nativo/Lua e com/sem janelas do lançador principal.
Não precisa apagar `stack.lock`; o bloqueio é controlado pelo processo.

### Mensagens comuns

| Mensagem/sintoma | Como conferir |
| --- | --- |
| `Este cenario ja esta em execucao` | Encerre a missão e use `./start_gz_simple.sh --stop` antes de trocar de modo |
| `Ja existe um SITL em execucao` | Encerre também os binários/MAVProxy iniciados manualmente ou pelos scripts antigos |
| `Communication Lost` no QGC | Confirme que o SITL está ativo e que há saída local UDP 14550 |
| Missão antiga no mapa | Use Plan → File → Download após o envio |
| Sem resposta via MAVProxy | Confirme `--out=udp:127.0.0.1:14555` e `set mavfwd true` |
| `PreArm: Accels inconsistent` / `Gyros inconsistent` | São bloqueios de sensores/simulação; o script aguarda estabilidade, não os desabilita |
| `Velocidade excedeu o limite de ensaio` | Verifique a velocidade medida e os gráficos; `--speed` é referência, não um limite físico rígido |
| Lua não entra | Confira SCR_ENABLE=1, SCR_USER5=2 e BATT_SOC_ALLOC=1; aguarde a criação dos parâmetros dinâmicos |
| Sem novos valores após editar Lua | Reinicie o cenário `--lua` para copiar os fontes novamente |
| `Outro ensaio ja controla este SYSID` | Encerre o outro utilitário antes de iniciar uma nova coleta |

Logs do lançador Gazebo:

```bash
tail -n 60 gazebo_simple/run/sitl.log
tail -n 60 gazebo_simple/run/gazebo.log
tail -n 60 gazebo_simple/run/sensors.log
```

No modo direto com Lua, consulte também:

```bash
tail -n 60 gazebo_simple/run/lua/ardurover.log
```

## 12. Testes e estado de validação

Para verificar o código, a partir da raiz do repositório:

```bash
cd /home/grin/ardupilot
lua Rover/tests/test_control_allocation.lua
python3.8 -m unittest discover -s Rover/python/missions/tests -v
bash -n Rover/start_gz_simple.sh Rover/sim_gz_simple.sh Rover/gazebo_simple/start_lua_sitl.sh
```

A inicialização real de Gazebo + binário direto + MAVProxy + Lua foi
conferida, incluindo leitura dos parâmetros selecionados. Após ajustar os
atuadores das rodas no modelo Gazebo, uma missão com Lua ativo completou
círculo de raio 3 m e quadrado de lado 6 m em área livre: referência de
0,4 m/s, pico de 0,59 m/s e erro RMS de velocidade de 0,05 m/s.
Veja as condições e limitações na
[validação da alocação](gazebo_simple/ALLOCATION_VALIDATION.md).
Não houve comparação equivalente de missão antes/depois no cenário com
obstáculos. Os testes unitários de Lua usam APIs simuladas.
Os comandos de SITL interno deste manual foram
montados conforme as opções e o modelo disponíveis no código, sem uma nova
campanha de sintonia nesse modelo durante a escrita do manual.

Os ensaios anteriores de navegação não demonstraram precisão de 20 cm nem
um conjunto ótimo global de ganhos. O manual descreve como executar e medir;
ele não declara o problema de overshoot resolvido.

## 13. Scripts antigos e referências do projeto

`sim_1roverb.sh`, `sim_1rover_new.sh` e os scripts multi-rover continuam no
repositório, mas usam outros perfis/diretórios e destinos de rede. Em
particular, `sim_1roverb.sh` contém um argumento `sim_vehicle.py` duplicado
e depende de console/mapa com wxPython. As chamadas deste manual usam os
fluxos explícitos acima; não execute os scripts antigos simultaneamente.

Para o modelo ROS Argo, `sim_gz1rover.sh` foi atualizado para carregar o
Lua em um diretório separado e converter a exportação QGroundControl:
com o mundo ROS aberto e o SITL anterior encerrado, use
`./sim_gz1rover.sh --foreground --no-rebuild`. Consulte
[configuração e verificação do Lua no ROS Argo](scripts/README.pt-BR.md#ros-argo-e-sim_gz1roversh).

- [Detalhes do mundo, câmera e LiDAR](gazebo_simple/README.pt-BR.md)
- [ROS Noetic: subestação Argo e três rovers](DIAGNOSTICO_ROS_ARGO.md)
- [Missão, métricas e algoritmo de sintonia](mission_tune/README.pt-BR.md)
- [Histórico de validação e limitações](mission_tune/VALIDATION.md)
- [Alocação Lua, canais e parâmetros](scripts/README.pt-BR.md)

## Programas Python organizados e SLAM 2D

Os fontes Python de Rover estao em [`python/`](python/README.md), separados
por funcionalidade. Camera: `python/camera/read_ros_camera.py`; LiDAR ROS:
`python/lidar/read_ros_lidar.py`; mapa 2D do MID360:
`python/slam/slam_mid360_2d.py`. As chamadas atualizadas e dependencias
estao no [manual Python](python/README.md).

Para executar FAST-LIO com o MID360 simulado e acessar sua pose no Lua,
consulte o [manual FAST-LIO e MAVProxy](python/slam/fast_lio/README.pt-BR.md).
