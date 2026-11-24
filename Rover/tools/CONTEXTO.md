# Contexto do projeto Rover (para continuar em outro chat)

Repositório: `ardupilot`, branch `auto-tunning-can` (base `master`).
Atualizado em 24/11/2026. Último commit visto: `fb36bc455a` (missão no mapa da interface).

## Como trabalhar com o usuário

- Responder em português.
- **Só fazer commit quando o usuário pedir** ("commit", "faça o commit").
- Alterar parâmetro no rover só com o "sim" do usuário. Pela aba Parâmetros da GRIN ou por `mav_param.py`.
- O usuário **não quer mudar o `WP_RADIUS`** (fica 0,1).
- Manter o modo 5 do WheelTune (`SCR_USER1=5`) e o QuickTune: os dois são usados.
- O usuário testa em campo pela interface GRIN (`mode_panel.py`) ou pelo monitor, e cola no chat
  a saída ou o caminho de um CSV `esc_log_*.csv`.
  O monitor grava em `Rover/tools/Logs/`. A interface grava o CSV e os `.BIN` baixados em `Rover/tools/logs/`.
  Os dois ficam fora do git.
- O MAVFtp do Mission Planner falha pela HereLink (`kErrFailErrno`, `BadCrc`).
  Os scripts entram pela aba Scripts da GRIN ou por `ftp_scripts.py`.

## Hardware

- Rover skid-steer de 4 rodas, motores M3508 com ESC DJI C620 no CAN, controladora H7.
- Rádio e telemetria pela HereLink (o PC conecta no Wi-Fi AndroidAP).
- Dois caminhos de saída:
  - **PWM**: servos 0..3;
  - **CAN**: `esc_can.lua` com malha de velocidade por roda.
  - O caminho é escolhido pelo parâmetro `ESC_CAN`; o `esc_can` publica o caminho ativo em `ESC_STAT`.
- Convenção de sentido: no motor, **frente física = rpm negativo**.
  `THRO_INV = 1` inverte só o acelerador que vem do ArduPilot. O stick do modo manual já estava certo.
- Mapa de lados nos logs: em giro no lugar, ESC1/ESC4 vão para um sentido e ESC2/ESC3 para o outro.

## Arquivos

| Arquivo | Papel |
|---|---|
| `Rover/scripts/rover-controlallocation.lua` | Mixer: modos, alocação L/R, alinhamento de yaw em waypoint, Yaw tune, Circle tune, bancos de ganho PWM/CAN |
| `Rover/scripts/esc_can.lua` (v2611n) | Driver C620 no CAN: PI de rpm por roda, prefiltro, rampa, stall boost, WheelTune por degrau |
| `Rover/scripts/rover-quicktune.lua` | QuickTune do ArduPilot (ajusta `ATC_*`) |
| `Rover/scripts/modules/` | `pid.lua`, `autotune.lua` (relé), `circle-tune.lua` (`PERIOD_MS` = 25), `functions.lua` |
| `Rover/tools/esc_monitor.py` | Monitor no PC: gráfico, console e CSV, via HereLink |
| `Rover/tools/mode_panel.py` | Servidor da interface GRIN: MAVLink na UDP 14550 e página em `http://127.0.0.1:8765` (também no celular, mesma rede) |
| `Rover/tools/hmi/` | Página (`index.html`, `app.js`, `app.css`, logos). O `index.html` é ignorado pelo git; entra com `git add -f` |
| `Rover/tools/mission_core.py` | Regras e codec da missão, sem tela. Usado pela interface e pelo `mission_editor.py` |
| `Rover/tools/mission_editor.py` | Editor Tk da missão (bússola, ré, .waypoints). A aba Missão da GRIN faz o mesmo e ainda planeja no mapa. Os dois não abrem juntos: mesma UDP |
| `Rover/tools/param_docs.json` | Descrições dos parâmetros na aba Parâmetros (gerado por `build_param_docs.py`) |
| `Rover/tools/ftp_scripts.py` | CLI que envia os .lua de `Rover/scripts` (e `modules/`) para `/APM/scripts` pela HereLink (MAVFTP). A aba Scripts da GRIN faz o equivalente na página |
| `Rover/tools/mav_param.py` | Ler e gravar parâmetros pela HereLink (sem pymavlink) |
| `Rover/tools/sd_format.py` | Formata o cartão SD da controladora pela HereLink (`MAV_CMD_STORAGE_FORMAT`). Pede confirmação (ou `--yes`) e só envia com o rover desarmado. Apaga scripts, logs e terreno; parâmetros e missão ficam |
| `Rover/docs/` | Documentação: `esc_can.md`, `Controle-CAN.md`, `Autotunning.md`, `Manual-Tuning-CAN.md` |

### Comunicação mixer → esc_can

Por parâmetros em RAM:
- `ESC_CMD_L`, `ESC_CMD_R` (−1..1) e `ESC_CMD_SEQ`;
- `ESC_STAT` informa o caminho ativo.

No CAN, a mistura é `left = t + s`, `right = t − s`, normalizada pelo pico quando passa de 1.
Não há prioridade de direção.

O `esc_can` faz `alvo_rpm = cmd × ESC_RPM`. Em seguida aplica:
- a rampa `ESC_ACC`;
- o prefiltro `ESC_PREF × Ti`;
- o PI por roda, com ganhos por roda ou os compartilhados `ESC_P` / `ESC_I` / `ESC_FF`;
- o stall boost `ESC_STALL` (A/s);
- a parada sem corrente quando o comando é menor que 0,004 e a roda está parada.

### Seletor `SCR_USER1`

| Valor | Função |
|---|---|
| 0 | Normal |
| 1 | QuickTune (armado; usar Circle) |
| 2 | Yaw PI por relé (armado em HOLD). Ganhos em `SCR_USER2` / `SCR_USER3` |
| 3 | Circle tune do mixer (taxa de direção, relé) |
| 4 | WheelTune no chão (esc_can, degrau de corrente) |
| 5 | WheelTune com o rover suspenso |

### WheelTune por degrau (esc_can)

1. Kv vem da inclinação do rpm (mínimos quadrados) e o atraso vem do intercepto da reta.
2. `wc = min(ESC_WT_BW, 1/(2,65·T))`, `Kp = wc/Kv`, `Ti = 4/wc`.
3. O resultado é verificado em malha fechada. Se o overshoot passar de 15%, `wc` é multiplicado por 0,75 e o melhor resultado é mantido.
4. Se não houver resposta, tenta de novo com metade da corrente.
5. O atraso usado é a mediana dos atrasos válidos, com piso de 0,02.
6. O teste começa no sentido de frente, conforme o `THRO_INV`.

### Bancos de ganho PWM/CAN

O mixer guarda cópias de `ATC_STR_RAT_FF/P/I`, `ATC_SPEED_P/I` e `CRUISE_THROTTLE`:
- `CANT_C_*` para o CAN e `CANT_P_*` para o PWM;
- `CANT_ACTIVE` indica o banco carregado;
- a troca acontece só com o rover desarmado.

O QuickTune feito no CAN fica no banco do CAN.

### Período do mixer

| Modo | Período |
|---|---|
| Circle | 25 ms |
| AUTO, SIMPLE, ACRO, RTL | 25 ms (desde `b56b8e33f8`; antes eram 200 ms no PWM, o que causava oscilação) |
| Manual, Steering, Hold (CAN) | 20 ms |
| Manual, Steering, Hold (PWM) | 200 ms |

### Alinhamento de yaw no AUTO

Quando o rover chega a um `NAV_WAYPOINT`, o **campo de altitude (z)** define o rumo de alinhamento em graus:
- **`-1` = passar direto**;
- qualquer outro valor faz o rover parar, ir para o modo STEERING e girar até esse rumo, com o PI `steering_steady_pid`.

O Mission Planner pode preencher a altitude com algo como 927,5, e o rover então gira para 207,5°.
**Na missão, colocar Alt = -1 nos waypoints de passagem.**

Ré em missão: `DO_SET_REVERSE` (P1 1/0). Toda troca de modo zera a ré. Desde 19/11 o mixer, ao entrar em AUTO,
reinicia do `DO_SET_REVERSE 1` que estiver logo antes do waypoint atual (`restoreReverseOnAutoEntry`), então com
um DO 1 antes de CADA waypoint de ré o alinhamento funciona na ré. O `mission_editor.py` monta isso pela caixa
"Chegar de ré" e gira 180° o alinhamento anterior (rumo = frente do rover). Ver `Rover/docs/Missoes-AUTO.md`.

## Interface GRIN

Página local, azul da Argo no claro e laranja/azul mais claros no escuro. O botão **Escuro/Claro** fica no canto inferior direito e a escolha fica no navegador. O navegador não fala MAVLink: o `mode_panel.py` escuta a UDP, manda heartbeat para a HereLink (`192.168.43.1:14552`; `--kick none` desliga) e serve a página em `0.0.0.0:8765`. O sysid do GCS é 255, o mesmo do Mission Planner. Fechar o `esc_monitor.py`, o `mav_param.py` e o `mission_editor.py` antes: os quatro usam a porta 14550.

Nenhum botão de ensaio arma sozinho. Armar pede um segundo clique em 4 s. Ações que apagam (escrever missão, apagar script, apagar logs, formatar SD, limpar STRG_BAK, reiniciar, padrão de fábrica) também pedem confirmação.

```bash
py -3 Rover/tools/mode_panel.py
py -3 Rover/tools/mode_panel.py --conn udp:0.0.0.0:14551
py -3 Rover/tools/mode_panel.py --csv teste     # abre Rover/tools/logs/esc_log_teste.csv
```

| Aba | O que faz |
|---|---|
| Logs | Armar/desarmar. Grava CSV (`esc_log_*.csv` em `Rover/tools/logs`) com a mesma forma do monitor: telemetria, mensagens, parâmetros, modo e armamento. Gráficos ao vivo (20 s): corrente e comando das 4 rodas, bateria, rpm e alvos L/R, taxa de guinada (PID desejada/real e giro), desvio da trilha, temperatura dos motores. Leituras de velocidade, xtrack e bateria. Com o rover desarmado, lista, baixa e apaga os dataflash `.BIN` (`LOG_REQUEST_*` / `LOG_ERASE`). O `.BIN` baixa no navegador e também é gravado em `Rover/tools/logs`. |
| Mensagens | `STATUSTEXT` da controladora, com a gravidade. |
| Modos | MANUAL, ACRO, STEERING, HOLD, LOITER, SIMPLE, CIRCLE, AUTO, RTL, SMART_RTL, GUIDED. O interruptor do rádio não se mexe. Qualquer modo pode ser escolhido; se a placa recusar, volta o modo anterior. |
| Parâmetros | Lista o que está na placa, com busca por nome ou descrição (`param_docs.json`) e grupos curtos: Direção, Velocidade, Motores, Missão, Script. Aplicar um campo grava na hora (`PARAM_SET`). Carregar `.parm` só preenche a tela; **Gravar na placa** manda o que mudou. Também atualiza a lista, salva `.parm`, reinicia e restaura o padrão de fábrica (zera a memória e reinicia). |
| Missão | Lê e escreve a missão do rover (mesmo protocolo do `mission_editor.py`, via `mission_core.py`). Mapa Leaflet: clique cria WAYPOINT, arrastar move, botão direito insere WAYPOINT, Loiter, Loiter tempo, RTL, DO_JUMP, waypoint na posição do rover, apaga ou limpa. O Alt do WAYPOINT continua sendo **rumo** (−1 passa direto); bússola, “passa direto”, “chegar de ré” (refaz os `DO_SET_REVERSE`) e “rumo do próximo trecho”. **Rumos→trecho** alinha os waypoints ao trecho seguinte. Abre e salva `.waypoints`. Modo **Relativo**: novo ponto por distância e ângulo a partir do GPS do rover, de uma origem capturada, do último waypoint/HOME ou do waypoint selecionado. O ângulo pode ser a frente do rover (0° = frente), o norte ou o trecho. Opções: encadear a origem no ponto novo, alinhar o rumo do waypoint anterior ao trecho, e gravar o rumo do passo ou deixar −1. |
| Scripts | MAVFtp como o Mission Planner (Config, MAVFtp): lista, copia para o PC, envia e apaga `.lua` em `/APM/scripts` e `/APM/scripts/modules`. Nome igual substitui. A placa só usa a lista nova depois de reiniciar. **Limpar STRG_BAK** apaga os backups de storage sem listá-los. **Formatar SD** (`MAV_CMD_STORAGE_FORMAT`) exige rover desarmado e apaga scripts, logs e terreno; parâmetros e missão ficam. |
| Ensaios | Só prepara parâmetros e, quando o ensaio pede, o modo. Quem arma é o usuário. |

Ensaios (`Preparar`):

| Botão | Deixa |
|---|---|
| Calibração das rodas no chão | HOLD, `ESC_CAN` 1, `ESC_MODE` 0, `SCR_USER1` 4 |
| Calibração com as rodas no ar | HOLD, `ESC_CAN` 1, `ESC_MODE` 0, `SCR_USER1` 5 |
| QuickTune | CIRCLE, `RTUN_ENABLE` 1, `RTUN_AXES` 3, `SCR_USER1` 1 |
| Relé em Circle | CIRCLE, `SCR_USER1` 3 |
| Relé de yaw | HOLD, `SCR_USER1` 2 |
| Voltar ao normal | só `SCR_USER1` 0; não muda o modo |

`SCR_USER1` pela aba Parâmetros só entra se o modo já for o do ensaio: 1 e 3 exigem CIRCLE; 2, 4 e 5 exigem HOLD. O 0 entra em qualquer modo.

O `GCS_PID_MASK` precisa ser 3 para o gráfico de guinada (PID de direção e de velocidade).

## Limites do Lua do ArduPilot (já causaram problemas)

- No máximo 100 variáveis locais no corpo principal: o estado fica em tabelas (`S` no mixer, `M` e `K` no esc_can).
- `collectgarbage` não existe. Lua com números de 32 bits. Não carrega bytecode.
- Falta de memória (`SCR_HEAP_SIZE` = 520000) ao carregar acontece **sem mensagem de erro**.
  Se o script não sobe, reduzir o código. A v2611e caiu por isso.

## Parâmetros atuais no rover (lidos em 17/11/2026)

| Grupo | Parâmetros |
|---|---|
| Velocidade | `WP_SPEED` 1,0 · `CIRC_SPEED` 1 · `CRUISE_SPEED` 0,99 · `CRUISE_THROTTLE` 67 |
| esc_can | `ESC_RPM` 120 · `ESC_CUR` 20 · `ESC_IMAX` 15 · `ESC_P` 0,134 · `ESC_I` 0,150 · `ESC_WT_BW` 5 · `ESC_PREF` 0,5 · `ESC_ACC` 800 · `ESC_STALL` 30 · `ESC_OPT` 1 |
| Velocidade (ATC) | `ATC_SPEED_P` 0,684 · `ATC_SPEED_I` 0,684 · `ATC_ACCEL_MAX` 3 · `ATC_DECEL_MAX` 1 |
| Direção (ATC) | `ATC_STR_RAT_P` 0,141 · `ATC_STR_RAT_I` 0,141 · `ATC_STR_RAT_D` 0,002 · `ATC_STR_RAT_FF` 0,281 · `ATC_STR_RAT_MAX` 120 · `ATC_STR_ACC_MAX` 180 |
| Navegação | `WP_RADIUS` 0,1 · `WP_ACCEL` 2 · `WP_PIVOT_ANGLE` 45 · `WP_PIVOT_RATE` 20 · `PSC_VEL_P` 2 |
| Outros | `THRO_INV` 1 · `CANT_ACTIVE` 1 · `SCR_USER1` 0 |

A velocidade máxima é de cerca de 1,48 m/s com `ESC_RPM` 120 (0,99 / 0,67). O `WP_SPEED` deve ficar abaixo disso, com folga para a direção.

## Histórico recente e estado

- QuickTune no Circle (CAN): concluído.
- AUTO no CAN oscilava. As causas encontradas no log `esc_log_20261117_162708.csv`:
  1. a altitude do waypoint (927,5) foi lida como rumo;
  2. com `WP_SPEED` 1,7 (e depois 1,2), um lado batia em 120 rpm nas curvas;
  3. o giro no lugar do RTL ficava parado cerca de 10 s, com os integradores presos no `ESC_IMAX` 10 A.
- Correções aplicadas em 17/11:
  - `WP_SPEED` 1,7 → 1,0;
  - `ATC_DECEL_MAX` 0,2 → 1;
  - `ESC_IMAX` 10 → 15.
  - **Ainda não testado.** Falta o usuário colocar Alt = -1 nos waypoints.
- `THRO_INV` testado e funcionando, inclusive no Circle.
- O WheelTune v2611n ainda precisa ser validado em campo, agora com `ESC_WT_BW` 5 (com 8 dava overshoot de 97–188%).

- 19/11 (AUTO saindo em S dos pontos): MIS_RESTART 1, WP_PIVOT_DELAY 1, ATC_ACCEL_MAX 1, ATC_STR_RAT_FF 0,4 -> 0,3,
  ESC_PREF 0,5 -> 0,2 (o prefiltro atrasava 0,4 s cada aceleração de roda; atraso do giro caiu de 0,5 para 0,3 s),
  PSC_POS_P 0,4 -> 0,2, PSC_VEL_P 2 -> 1, PSC_VEL_I 0,2 -> 0 (padrão; o PSC pedia ~22 °/s de giro na arrancada).
  Monitor: PID de direção é PID_TUNING eixo 5 (corrigido); analisador compara rumo estimado x direção GPS.

- QuickTune (23/11): parâmetro `RTUN_MSG` (padrão 0) mostra `Tune INICIO` e `Tune NN%` a cada 5 s, mesmo parado (direção 0-50%, velocidade 50-100%),
  mais `Tune 100%`, `SALVO` e `CANCELADO`; `RTUN_MSG = 1` volta todas as mensagens detalhadas abaixo.
- QuickTune (19/11): mensagens em português, várias linhas curtas por evento: o que a etapa mede e o mínimo que precisa
  (giro >= 10 °/s e comando >= 5%; velocidade >= 0,5 m/s e acelerador >= 20%), progresso com tempo restante, `PARADO ha Ns`
  com os valores atuais/mínimos e `Faca: ...`, cada ganho antes -> depois (%), CONCLUIDO / SALVO / CANCELADO.
  O mínimo de comando de direção caiu de 10% para 5% (com FF ~0,3 um Circle de 3 m dava só 6-10% e não avançava).

## Pendências e ideias opcionais (não pedidas)

- Mistura com prioridade de direção no CAN: foi descartada a pedido do usuário. O patch está fora do repositório,
  na pasta temporária da sessão anterior (`descartado_v2611l_thro_inv.patch`), e pode já ter sido apagado.
- Se o giro no lugar continuar fraco, aumentar o `WP_PIVOT_RATE` (20 → 45–60).

## Ferramentas

```bash
py -3 Rover/tools/mode_panel.py                        # interface GRIN (página, CSV, missão, scripts, logs)
python Rover/tools/esc_monitor.py                      # monitor (grava em Rover/tools/Logs/)
python Rover/tools/esc_monitor.py --no-plot            # só console e CSV
python Rover/tools/mav_param.py WP_SPEED ESC_RPM       # ler parâmetros
python Rover/tools/mav_param.py ESC_IMAX=15            # gravar e confirmar
python Rover/tools/sd_format.py                        # formatar o SD (rover desarmado; pede FORMATAR)
python Rover/tools/mission_editor.py                   # editor Tk; feche a interface antes
```

- O monitor (desde 18/11) também pede atitude, posição, NAV_CONTROLLER_OUTPUT, PID_TUNING, servos, RC e
  missão a `--rate` Hz (padrão 10, só até reiniciar a placa), baixa todos os parâmetros no início
  (linhas `PARAM:<nome>` no CSV + arquivo `.parm`) e registra mudanças de parâmetro e de modo (`MODE`).
  PID_TUNING precisa `GCS_PID_MASK = 3`.
- Análise: `python Rover/tools/esc_monitor.py analyze esc_log_X.csv --plot` (relatório por modo + PNG
  `esc_log_X_analise.png`; `--from/--to` para um trecho). Aponta zigue-zague (período/amplitude
  do xtrack), atraso e ganho do yaw rate, saturação e atraso das rodas.
- A interface, o monitor, o `mav_param.py` e o `mission_editor.py` usam a porta UDP 14550. Só um de cada vez.
- A HereLink só começa a transmitir depois de receber heartbeat em `192.168.43.1:14552`.
  As mensagens podem chegar duplicadas; o monitor descarta as repetidas.
- No CSV (`hora_pc, t_s, boot_ms, nome, valor, texto`) os campos são (ESCn_T = temperatura do motor, 1 Hz; BAT_V/BAT_I = bateria do ArduPilot, desde 18/11).
  `ESCn_I` é corrente de torque/fase do motor (C620), NÃO corrente de bateria; a análise estima a da bateria por P = Kt·I·ω + R·I² (M3508: Kt 0,3 N·m/A, R 0,194 Ω):
  - `ESC1..4_RPM`, `ESC1..4_I`, `ESC1..4_ICMD`: rpm, corrente e corrente comandada de cada roda;
  - `ESC_TGT_L` / `ESC_TGT_R`: rpm alvo de cada lado;
  - `STATUSTEXT`: mensagens de texto da controladora.
